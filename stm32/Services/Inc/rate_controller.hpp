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
    // Global on/off for setpoint smoothing. When false, the per-axis
    // smoother instances are bypassed entirely (rate_sp goes straight
    // to the PID). Per-axis `AxisConfig::smoother` is still loaded so
    // that flipping the flag at runtime starts from a sane state.
    bool smoothing_enabled = true;
    AxisConfig roll{};
    AxisConfig pitch{};
    AxisConfig yaw{};

    // When the pilot's commanded throttle is below this value, the
    // per-axis integrators are frozen for that tick — Compute() still
    // runs (so D-term LPF tracks measurement and the controller stays
    // warm) but the I-term doesn't accumulate. Prevents wind-up at
    // descent throttle where the mixer can't apply torque anyway.
    // Set to 0 to disable.
    float iterm_freeze_below_throttle = 0.0f;

    // First-order IIR on the yaw torque output (post-PID, pre-mixer).
    // alpha ∈ (0, 1]: 1 = pass-through; smaller = more aggressive
    // smoothing. Reduces high-frequency torque transients from rotor
    // acceleration on the yaw axis, where the QuadX geometry gives the
    // weakest authority and highest noise pickup. Roll/pitch are
    // unfiltered. PX4 equivalent: `_output_lpf_yaw`.
    float yaw_output_lpf_alpha = 1.0f;
  };

  RateController() = default;

  // Initialize all three axes. Idempotent — resets integrator + filter
  // state. Called once by System::InitComponent.
  void Init(const Config &cfg);

  // Clear integrator + filter state on every axis. Call on arm/disarm
  // transitions or whenever the measurement source restarts.
  void Reset();

  // Two-call API (preferred — exercises the PID's back-calculation
  // anti-windup through any downstream stage, including mixer-disarm).
  //
  // 1. ComputeTorque(rate_sp, gyro, dt) → torque demands {τR, τP, τY}.
  //    Per-axis PID computes, returns the pre-mixer torque. Integrator
  //    is NOT yet committed; pending state lives on each axis PID.
  //
  // 2. CommitTorque(applied, dt). `applied` is what the downstream
  //    actuator/mixer chain actually applied per axis (e.g., zero on
  //    every axis when disarmed, equal to the commanded value in the
  //    mixer's linear region). The per-axis PID's back-calc unwinds
  //    whenever applied != commanded — for the disarm case, this
  //    means the integrator can't accumulate stale state during a
  //    600 ms pre-arm pump.
  std::array<float, 3> ComputeTorque(const Eigen::Vector3f &rate_sp,
                                     const Eigen::Vector3f &gyro_measured,
                                     float dt_s);
  // pilot_throttle is the [0, 1] stick value forwarded by the caller.
  // If it falls below Config::iterm_freeze_below_throttle, integrators
  // are frozen via Pid::CommitFilterOnly instead of CommitIntegrator.
  // Default 1.0f (above any sensible threshold) keeps legacy callers
  // that don't pass throttle behaving identically — integrator always
  // commits.
  void CommitTorque(const std::array<float, 3> &applied, float dt_s,
                    float pilot_throttle = 1.0f);

  // Convenience single-call: Compute + CommitTorque(itself). Use when
  // there is no downstream clipping beyond the PID's own output_clamp
  // (the PID's own anti-windup still fires for output_clamp saturation
  // via the back-calc inside Update / Compute+CommitIntegrator).
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

  // Runtime toggle for the setpoint smoother. When false, the per-axis
  // smoothers are bypassed and rate_sp goes straight to the PID — the
  // same effect as constructing with Config::smoothing_enabled=false.
  // Smoother internal state is retained so flipping back resumes from
  // the same point. Use for SIL cascade-primitive tests that need a
  // clean step input to the PID, or for GCS/autotune flows that
  // briefly disable smoothing to identify the unshaped cascade.
  void SetSmoothingEnabled(bool on) { smoothing_enabled_ = on; }
  bool IsSmoothingEnabled() const   { return smoothing_enabled_; }

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
