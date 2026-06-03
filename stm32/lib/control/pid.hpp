// control::Pid — SISO PID with back-calculation anti-windup.
// Header-only, no allocations/exceptions/RTTI/deps. Cortex-M4 hard-FPU.
//
// Anti-windup uses Kt = Ki/Kp (Aström rule 1): the integrator unwinds as
// fast as it accumulates. Saturation is detected by comparing the PID's
// pre-clip output to the actuator's post-clip value passed by the caller,
// so one mechanism covers output_clamp, downstream clipping (mixer disarm),
// and actuator saturation (motor at max) without arm-edge resets or hints.
//
// Two-call API (committing integrator only after downstream clipping):
//   float u_pid = pid.Compute(setpoint, measurement, dt_s);
//   float u_post = downstream.Apply(u_pid);   // mixer / ESC / clip
//   pid.CommitIntegrator(u_post, dt_s);
// Update() collapses both for callers with no clipping beyond output_clamp.
//
// Design (matches BetaFlight + ArduPilot):
//   - D on filtered measurement, negated — no setpoint-step kick.
//   - First-order IIR on the D-input. alpha ∈ (0, 1]; alpha = 1 → off.
//   - Hard integrator_clamp safety net, independent of anti-windup, in
//     case Kt is mistuned.

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
  // Soft I-gain attenuation at large signed errors: scales Ki by
  // 1 − (error/threshold)², reaching 0 at |error| = threshold. Prevents
  // post-flip / stick-whip bounce-back from a huge transient I dump.
  // 0 (default) disables. PX4 equivalent: hard-coded 400°/s ≈ 7 rad/s.
  float i_factor_error_thresh = 0.0f;
};

class Pid {
 public:
  constexpr Pid() = default;

  // Initialize gains + limits + D-term low-pass alpha. Kt is derived
  // automatically (Rule 1): Kt = Ki / Kp; Kt = 0 when Kp ≤ 0 (degenerate).
  void Init(const PidGains &gains, const PidLimits &limits = {},
            float d_term_lpf_alpha = 1.0f) {
    gains_ = gains;
    limits_ = limits;
    d_lpf_alpha_ = ClampUnit(d_term_lpf_alpha);
    RecomputeKt();
    Reset();
  }

  // Runtime updates. Cheap; safe to call from a tuning UI or autotuner.
  // SetGains recomputes Kt so the anti-windup tracks the new tuning.
  void SetGains(const PidGains &gains) {
    gains_ = gains;
    RecomputeKt();
  }
  void SetLimits(const PidLimits &limits) { limits_ = limits; }
  void SetDTermLpfAlpha(float alpha) { d_lpf_alpha_ = ClampUnit(alpha); }

  // Clear integrator + filter history + pending state. Call on arm/disarm
  // transitions and whenever the loop's measurement source is reinitialized.
  void Reset() {
    i_acc_ = 0.0f;
    filtered_meas_ = 0.0f;
    initialized_ = false;
    last_error_ = 0.0f;
    last_output_ = 0.0f;
    pending_valid_ = false;
    pending_error_ = 0.0f;
    pending_u_unsat_ = 0.0f;
    pending_filtered_meas_ = 0.0f;
  }

  // Returns the PID output WITHOUT committing the integrator; caller must
  // pair with CommitIntegrator (see two-call API above). dt_s is seconds
  // since the previous Compute() and must be > 0; dt_s ≤ 0 returns the last
  // output unchanged and invalidates pending state so a stale
  // CommitIntegrator becomes a no-op.
  float Compute(float setpoint, float measurement, float dt_s) {
    if (dt_s <= 0.0f) {
      pending_valid_ = false;
      return last_output_;
    }

    const float error = setpoint - measurement;

    float d_term = 0.0f;
    float filtered_meas_new = filtered_meas_;
    if (initialized_) {
      filtered_meas_new =
          d_lpf_alpha_ * measurement + (1.0f - d_lpf_alpha_) * filtered_meas_;
      d_term = -gains_.kd * (filtered_meas_new - filtered_meas_) / dt_s;
    } else {
      filtered_meas_new = measurement;
    }

    const float p_term = gains_.kp * error;
    const float u_unsat = p_term + i_acc_ + d_term;

    float u_pid = u_unsat;
    if (limits_.output_clamp > 0.0f) {
      if (u_pid > limits_.output_clamp)
        u_pid = limits_.output_clamp;
      else if (u_pid < -limits_.output_clamp)
        u_pid = -limits_.output_clamp;
    }

    // Stash for CommitIntegrator. Filter state stays in filtered_meas_
    // until commit so an uncommitted Compute() doesn't desync the D filter.
    pending_error_ = error;
    pending_u_unsat_ = u_unsat;
    pending_filtered_meas_ = filtered_meas_new;
    pending_valid_ = true;

    last_output_ = u_pid;
    return u_pid;
  }

  // Commit the integrator update using back-calculation anti-windup.
  //   dI/dt = Ki·error − Kt·(u_unsat − u_post)
  // Kt = Ki/Kp from Init/SetGains. Then the hard integrator_clamp (if
  // configured) acts as a safety net on the result.
  //
  // No-op if Compute() wasn't called since the last commit (or dt_s ≤ 0
  // last time). dt_s must match the dt passed to the paired Compute().
  void CommitIntegrator(float u_post, float dt_s) {
    if (!pending_valid_ || dt_s <= 0.0f) return;

    // I-gain attenuation at huge errors (see PidLimits::i_factor_error_thresh).
    float i_factor = 1.0f;
    if (limits_.i_factor_error_thresh > 0.0f) {
      const float r = pending_error_ / limits_.i_factor_error_thresh;
      i_factor = 1.0f - r * r;
      if (i_factor < 0.0f) i_factor = 0.0f;
    }

    const float anti_wu = kt_ * (pending_u_unsat_ - u_post);
    i_acc_ += dt_s * (i_factor * gains_.ki * pending_error_ - anti_wu);

    if (limits_.integrator_clamp > 0.0f) {
      if (i_acc_ > limits_.integrator_clamp)
        i_acc_ = limits_.integrator_clamp;
      else if (i_acc_ < -limits_.integrator_clamp)
        i_acc_ = -limits_.integrator_clamp;
    }

    filtered_meas_ = pending_filtered_meas_;
    initialized_ = true;
    last_error_ = pending_error_;
    pending_valid_ = false;
  }

  // Commit the FILTER state from a pending Compute() but freeze the
  // integrator. For upstream policy (e.g. RateController) that must block
  // I accumulation this tick — e.g. throttle at zero, where any I-term
  // would dump as a spike when authority returns. The D-LPF and last_error_
  // still advance so the derivative branch keeps tracking and resumes
  // correctly once the freeze lifts.
  // No-op if Compute() wasn't called since the last commit.
  void CommitFilterOnly(float dt_s) {
    if (!pending_valid_ || dt_s <= 0.0f) return;
    filtered_meas_ = pending_filtered_meas_;
    initialized_ = true;
    last_error_ = pending_error_;
    pending_valid_ = false;
  }

  // Single-call convenience for callers with no downstream clipping
  // beyond the PID's own output_clamp. Equivalent to:
  //   const float u = Compute(setpoint, measurement, dt_s);
  //   CommitIntegrator(u, dt_s);
  //   return u;
  float Update(float setpoint, float measurement, float dt_s) {
    const float u = Compute(setpoint, measurement, dt_s);
    CommitIntegrator(u, dt_s);
    return u;
  }

  // Diagnostics. Cheap, no side effects.
  const PidGains &Gains() const { return gains_; }
  const PidLimits &Limits() const { return limits_; }
  float DTermLpfAlpha() const { return d_lpf_alpha_; }
  float Kt() const { return kt_; }
  float Integrator() const { return i_acc_; }
  float LastError() const { return last_error_; }
  float LastOutput() const { return last_output_; }
  bool HasPendingCommit() const { return pending_valid_; }

 private:
  static constexpr float ClampUnit(float a) {
    return (a <= 0.0f) ? 1.0f : (a > 1.0f ? 1.0f : a);
  }

  void RecomputeKt() {
    // Aström rule 1: Kt = Ki / Kp. Kp ≤ 0 is degenerate (no output to
    // saturate) → disable back-calc; hard integrator_clamp still applies.
    kt_ = (gains_.kp > 1e-9f) ? (gains_.ki / gains_.kp) : 0.0f;
  }

  PidGains gains_{};
  PidLimits limits_{};
  float d_lpf_alpha_ = 1.0f;
  float kt_ = 0.0f;
  float i_acc_ = 0.0f;
  float filtered_meas_ = 0.0f;
  bool initialized_ = false;
  float last_error_ = 0.0f;
  float last_output_ = 0.0f;

  // Pending state for the two-call Compute / CommitIntegrator API.
  bool pending_valid_ = false;
  float pending_error_ = 0.0f;
  float pending_u_unsat_ = 0.0f;
  float pending_filtered_meas_ = 0.0f;
};

}  // namespace control
