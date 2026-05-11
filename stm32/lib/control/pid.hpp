// math::Pid (namespace control) — single-input single-output PID primitive.
//
// Lives under stm32/lib/control/ alongside stm32/lib/math/. Header-only,
// no allocations, no exceptions, no RTTI, no external dependencies.
// Cortex-M4 hard-FPU: every op is single-cycle VFP.
//
// Used as a reusable primitive. Compose one instance per axis / control
// loop:
//   - Rate controller   : 3 × Pid (roll/pitch/yaw body rate)
//   - Attitude loop     : 3 × Pid (angle-error → rate setpoint)
//   - Altitude / takeoff: 1 × Pid (z-velocity / z-position)
//   - Autotune scoring  : N × Pid with parameter sweeps
//
// Design choices (all standard practice; matches BetaFlight + ArduPilot):
//   - **D from filtered measurement, negated.** Avoids the setpoint-kick
//     spike on a stick step (d/dt of error = d/dt of setpoint − d/dt of
//     measurement; for a constant setpoint −d/dt of measurement is
//     equivalent and never spikes when the setpoint moves).
//   - **First-order IIR on the D-input.** `alpha ∈ (0, 1]`. alpha = 1 →
//     no filtering. Caller computes alpha from a target cutoff and the
//     sample period (`alpha = Ts / (Ts + 1/(2π·fc))`).
//   - **Clamping anti-windup.** When the output saturates AND the
//     proposed integrator update would push further into saturation,
//     the update is frozen for that tick. Cheap, effective, no
//     back-calculation gain to tune.
//
// Out of scope (deliberately):
//   - NaN/Inf filtering — fix at the source layer, not in every consumer.
//   - dt_s sanity caps — fix at the scheduler layer.
//   - Multiple integrator-windup strategies — clamping is enough.
//   - Feed-forward — PIDF is a one-line add at the call site if needed.

#pragma once

namespace control {

struct PidGains {
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
};

// |i_acc| ≤ integrator_clamp, |u| ≤ output_clamp.
// Values ≤ 0 disable the corresponding limit.
struct PidLimits {
  float integrator_clamp = 0.0f;
  float output_clamp = 0.0f;
};

class Pid {
 public:
  constexpr Pid() = default;

  // Initialize gains + limits + D-term low-pass alpha.
  // alpha ∈ (0, 1]: 1 = pass-through, smaller = more smoothing.
  // alpha outside the range is clamped silently rather than asserting —
  // misconfigured callers should bind better at startup, not crash mid-flight.
  void Init(const PidGains& gains, const PidLimits& limits = {},
            float d_term_lpf_alpha = 1.0f) {
    gains_ = gains;
    limits_ = limits;
    d_lpf_alpha_ = ClampUnit(d_term_lpf_alpha);
    Reset();
  }

  // Runtime updates. Cheap; safe to call from a tuning UI or autotuner.
  void SetGains(const PidGains& gains) { gains_ = gains; }
  void SetLimits(const PidLimits& limits) { limits_ = limits; }
  void SetDTermLpfAlpha(float alpha) { d_lpf_alpha_ = ClampUnit(alpha); }

  // Clear integrator + filter history. Call on arm/disarm transitions
  // and whenever the loop's measurement source is reinitialized.
  void Reset() {
    i_acc_ = 0.0f;
    filtered_meas_ = 0.0f;
    initialized_ = false;
    last_error_ = 0.0f;
    last_output_ = 0.0f;
  }

  // Compute one PID step.
  //   setpoint    : desired value
  //   measurement : current value
  //   dt_s        : seconds since the previous Update() (must be > 0)
  //
  // Returns the control output (clamped to ±output_clamp if set).
  // dt_s ≤ 0 returns the last output unchanged — protects against an
  // out-of-order tick without raising an error.
  float Update(float setpoint, float measurement, float dt_s) {
    if (dt_s <= 0.0f) return last_output_;

    const float error = setpoint - measurement;

    float d_term = 0.0f;
    if (initialized_) {
      const float prev = filtered_meas_;
      filtered_meas_ = d_lpf_alpha_ * measurement +
                       (1.0f - d_lpf_alpha_) * filtered_meas_;
      d_term = -gains_.kd * (filtered_meas_ - prev) / dt_s;
    } else {
      filtered_meas_ = measurement;
      initialized_ = true;
    }

    const float p_term = gains_.kp * error;
    const float i_proposed = i_acc_ + gains_.ki * error * dt_s;
    const float u_unsat = p_term + i_proposed + d_term;

    float u = u_unsat;
    if (limits_.output_clamp > 0.0f) {
      if (u > limits_.output_clamp) u = limits_.output_clamp;
      else if (u < -limits_.output_clamp) u = -limits_.output_clamp;
    }

    const bool saturated_high = (u_unsat > 0.0f) && (u < u_unsat);
    const bool saturated_low = (u_unsat < 0.0f) && (u > u_unsat);
    const bool i_pushes_high = (i_proposed > i_acc_);
    const bool i_pushes_low = (i_proposed < i_acc_);
    const bool freeze_i =
        (saturated_high && i_pushes_high) ||
        (saturated_low && i_pushes_low);

    if (!freeze_i) {
      i_acc_ = i_proposed;
      if (limits_.integrator_clamp > 0.0f) {
        if (i_acc_ > limits_.integrator_clamp) i_acc_ = limits_.integrator_clamp;
        else if (i_acc_ < -limits_.integrator_clamp) i_acc_ = -limits_.integrator_clamp;
      }
    }

    last_output_ = u;
    last_error_ = error;
    return u;
  }

  // Diagnostics. Cheap, no side effects.
  const PidGains& Gains() const { return gains_; }
  const PidLimits& Limits() const { return limits_; }
  float DTermLpfAlpha() const { return d_lpf_alpha_; }
  float Integrator() const { return i_acc_; }
  float LastError() const { return last_error_; }
  float LastOutput() const { return last_output_; }

 private:
  static constexpr float ClampUnit(float a) {
    return (a <= 0.0f) ? 1.0f : (a > 1.0f ? 1.0f : a);
  }

  PidGains gains_{};
  PidLimits limits_{};
  float d_lpf_alpha_ = 1.0f;
  float i_acc_ = 0.0f;
  float filtered_meas_ = 0.0f;
  bool initialized_ = false;
  float last_error_ = 0.0f;
  float last_output_ = 0.0f;
};

}  // namespace control
