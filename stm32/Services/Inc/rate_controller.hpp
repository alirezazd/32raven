#pragma once

#include <Eigen/Core>
#include <array>

#include "control/pid.hpp"
#include "control/setpoint_smoother.hpp"

class RateController {
 public:
  struct AxisConfig {
    control::PidGains gains{};
    control::PidLimits limits{};
    // Slew-rate + first-order LPF applied to the rate setpoint before it
    // reaches the PID. Pass-through when rate_limit <= 0 AND lpf_alpha
    // >= 1. Only consulted when Config::smoothing_enabled is true.
    control::SmootherConfig smoother{};
    // D-input first-order IIR coefficient. alpha ∈ (0, 1]: 1 = no filtering.
    // Compute as `alpha = Ts / (Ts + 1/(2π·fc))` for a target cutoff fc.
    float d_term_lpf_alpha = 1.0f;
  };

  struct Config {
    // Global on/off for setpoint smoothing. When false, per-axis smoothers
    // are bypassed (rate_sp goes straight to PID). Per-axis smoother config
    // is still loaded so a runtime flip starts from a sane state.
    bool smoothing_enabled = true;
    AxisConfig roll{};
    AxisConfig pitch{};
    AxisConfig yaw{};

    // Below this commanded throttle, freeze the per-axis integrators for the
    // tick: Compute() still runs (D-term LPF keeps tracking, controller stays
    // warm) but the I-term doesn't accumulate. Prevents wind-up at descent
    // throttle where the mixer can't apply torque anyway. 0 disables.
    float iterm_freeze_below_throttle = 0.0f;

    // First-order IIR on yaw torque output (post-PID, pre-mixer). alpha ∈
    // (0, 1]: 1 = pass-through, smaller = more smoothing. Tames HF torque
    // transients from rotor accel on yaw, where QuadX geometry has the
    // weakest authority and highest noise. Roll/pitch unfiltered. PX4
    // equivalent: `_output_lpf_yaw`.
    float yaw_output_lpf_alpha = 1.0f;
  };

  RateController() = default;

  // Idempotent — resets integrator + filter state.
  void Init(const Config &cfg);

  // Clear integrator + filter state on every axis. Call on arm/disarm
  // transitions or whenever the measurement source restarts.
  void Reset();

  // Two-call API (preferred): routes the PID back-calc anti-windup
  // through downstream saturation, including mixer-disarm.
  //
  // 1. ComputeTorque → pre-mixer torque {τR, τP, τY}. Integrator not yet
  //    committed; pending state lives on each axis PID.
  // 2. CommitTorque(applied) — `applied` is what the actuator/mixer chain
  //    actually applied per axis (zero when disarmed, == commanded in the
  //    mixer's linear region). Back-calc unwinds when applied != commanded,
  //    so the integrator can't wind up during a 600 ms pre-arm pump.
  std::array<float, 3> ComputeTorque(const Eigen::Vector3f &rate_sp,
                                     const Eigen::Vector3f &gyro_measured,
                                     float dt_s);
  // pilot_throttle: [0, 1] stick value. Below iterm_freeze_below_throttle,
  // integrators freeze via Pid::CommitFilterOnly. Default 1.0f keeps callers
  // that omit throttle always committing the integrator.
  void CommitTorque(const std::array<float, 3> &applied, float dt_s,
                    float pilot_throttle = 1.0f);

  // Single-call Compute + CommitTorque(itself). Use when there is no
  // downstream clipping beyond the PID's own output_clamp (its back-calc
  // still fires for output_clamp saturation).
  std::array<float, 3> Step(const Eigen::Vector3f &rate_sp,
                            const Eigen::Vector3f &gyro_measured, float dt_s);

  // Per-axis runtime tune (autotune / live tuning). Gains shift live;
  // integrator state preserved.
  void SetRollGains(const control::PidGains &g) { roll_.SetGains(g); }
  void SetPitchGains(const control::PidGains &g) { pitch_.SetGains(g); }
  void SetYawGains(const control::PidGains &g) { yaw_.SetGains(g); }

  // Per-axis runtime limits (integrator_clamp, output_clamp,
  // i_factor_error_thresh). Replaces all three fields atomically;
  // integrator state preserved.
  void SetRollLimits(const control::PidLimits &l) { roll_.SetLimits(l); }
  void SetPitchLimits(const control::PidLimits &l) { pitch_.SetLimits(l); }
  void SetYawLimits(const control::PidLimits &l) { yaw_.SetLimits(l); }

  // Yaw output LPF alpha runtime setter. alpha ∈ (0, 1]: out-of-range
  // values fall back to 1.0 (pass-through). Filter state preserved.
  void SetYawOutputLpfAlpha(float a) {
    yaw_output_lpf_alpha_ = (a <= 0.0f || a > 1.0f) ? 1.0f : a;
  }
  float YawOutputLpfAlpha() const { return yaw_output_lpf_alpha_; }

  // Runtime equivalent of Config::smoothing_enabled. Smoother state is
  // retained so flipping back resumes from the same point. Used by tests
  // and GCS/autotune flows needing a clean step into the unshaped cascade.
  void SetSmoothingEnabled(bool on) { smoothing_enabled_ = on; }
  bool IsSmoothingEnabled() const { return smoothing_enabled_; }

  // Runtime adjust the freeze threshold. Integrator state is NOT
  // cleared on change. Pass 0 to disable.
  void SetItermFreezeBelowThrottle(float t) {
    iterm_freeze_below_throttle_ = (t > 0.0f) ? t : 0.0f;
  }
  float ItermFreezeBelowThrottle() const {
    return iterm_freeze_below_throttle_;
  }

  // Diagnostics — const reads of the underlying axis PIDs and smoothers.
  const control::Pid &Roll() const { return roll_; }
  const control::Pid &Pitch() const { return pitch_; }
  const control::Pid &Yaw() const { return yaw_; }
  const control::SetpointSmoother &RollSmoother() const {
    return roll_smoother_;
  }
  const control::SetpointSmoother &PitchSmoother() const {
    return pitch_smoother_;
  }
  const control::SetpointSmoother &YawSmoother() const { return yaw_smoother_; }

 private:
  bool smoothing_enabled_ = true;
  // Cached threshold from Config; see CommitTorque for the freeze policy.
  float iterm_freeze_below_throttle_ = 0.0f;
  // Cached LPF coefficient + state for yaw output filtering.
  float yaw_output_lpf_alpha_ = 1.0f;
  float yaw_output_lpf_prev_ = 0.0f;
  control::Pid roll_;
  control::Pid pitch_;
  control::Pid yaw_;
  control::SetpointSmoother roll_smoother_;
  control::SetpointSmoother pitch_smoother_;
  control::SetpointSmoother yaw_smoother_;
};
