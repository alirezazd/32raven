#include "rate_controller.hpp"

namespace {

void InitAxis(control::Pid &pid, control::SetpointSmoother &smoother,
              const RateController::AxisConfig &cfg) {
  pid.Init(cfg.gains, cfg.limits, cfg.d_term_lpf_alpha);
  smoother.Init(cfg.smoother);
}

}  // namespace

void RateController::Init(const Config &cfg) {
  smoothing_enabled_ = cfg.smoothing_enabled;
  iterm_freeze_below_throttle_ = cfg.iterm_freeze_below_throttle;
  yaw_output_lpf_alpha_ =
      (cfg.yaw_output_lpf_alpha <= 0.0f || cfg.yaw_output_lpf_alpha > 1.0f)
          ? 1.0f
          : cfg.yaw_output_lpf_alpha;
  yaw_output_lpf_prev_ = 0.0f;
  InitAxis(roll_, roll_smoother_, cfg.roll);
  InitAxis(pitch_, pitch_smoother_, cfg.pitch);
  InitAxis(yaw_, yaw_smoother_, cfg.yaw);
}

void RateController::Reset() {
  roll_.Reset();
  pitch_.Reset();
  yaw_.Reset();
  roll_smoother_.Reset();
  pitch_smoother_.Reset();
  yaw_smoother_.Reset();
  yaw_output_lpf_prev_ = 0.0f;
}

std::array<float, 3> RateController::ComputeTorque(
    const Eigen::Vector3f &rate_sp, const Eigen::Vector3f &gyro_measured,
    float dt_s) {
  // Smoothing shapes the rate setpoint before it reaches the PID. When
  // disabled the PID sees the raw setpoint.
  float sp_roll = rate_sp.x();
  float sp_pitch = rate_sp.y();
  float sp_yaw = rate_sp.z();
  if (smoothing_enabled_) {
    sp_roll = roll_smoother_.Update(sp_roll, dt_s);
    sp_pitch = pitch_smoother_.Update(sp_pitch, dt_s);
    sp_yaw = yaw_smoother_.Update(sp_yaw, dt_s);
  }
  // Compute pre-clip torque per axis. Integrators are NOT committed here
  // — CommitTorque() must be called with the actually-applied torque
  // for back-calc anti-windup to fire on downstream saturation.
  const float roll_torque = roll_.Compute(sp_roll, gyro_measured.x(), dt_s);
  const float pitch_torque = pitch_.Compute(sp_pitch, gyro_measured.y(), dt_s);
  const float yaw_raw = yaw_.Compute(sp_yaw, gyro_measured.z(), dt_s);
  // Yaw output LPF (PX4 _output_lpf_yaw). Pass-through when alpha == 1.
  // The PID's pending_u_unsat_ remains the unfiltered value, so back-calc
  // will see the LPF transient as a small saturation signal — acceptable
  // at alpha close to 1; the integrator just drains slightly faster than
  // the un-LPF'd ideal.
  const float yaw_torque = yaw_output_lpf_alpha_ * yaw_raw +
                           (1.0f - yaw_output_lpf_alpha_) * yaw_output_lpf_prev_;
  yaw_output_lpf_prev_ = yaw_torque;
  return {roll_torque, pitch_torque, yaw_torque};
}

void RateController::CommitTorque(const std::array<float, 3> &applied,
                                  float dt_s, float pilot_throttle) {
  // If the pilot is commanding descent below the configured threshold,
  // freeze the integrators for this tick. Filter state still advances
  // inside CommitFilterOnly so D-term and last_error_ stay consistent
  // — only the I-term accumulation is suppressed. Disabled entirely
  // when threshold == 0.
  const bool freeze = (iterm_freeze_below_throttle_ > 0.0f) &&
                      (pilot_throttle < iterm_freeze_below_throttle_);
  if (freeze) {
    roll_.CommitFilterOnly(dt_s);
    pitch_.CommitFilterOnly(dt_s);
    yaw_.CommitFilterOnly(dt_s);
  } else {
    roll_.CommitIntegrator(applied[0], dt_s);
    pitch_.CommitIntegrator(applied[1], dt_s);
    yaw_.CommitIntegrator(applied[2], dt_s);
  }
}

std::array<float, 3> RateController::Step(const Eigen::Vector3f &rate_sp,
                                          const Eigen::Vector3f &gyro_measured,
                                          float dt_s) {
  const auto torque = ComputeTorque(rate_sp, gyro_measured, dt_s);
  CommitTorque(torque, dt_s);
  return torque;
}
