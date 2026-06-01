#include "attitude_controller.hpp"

#include "error_code.hpp"
#include "panic.hpp"

namespace {

bool IsAxisValid(const AttitudeController::AxisConfig &a) {
  if (a.kp < 0.0f) return false;
  if (a.rate_clamp < 0.0f) return false;
  return true;
}

bool IsConfigValid(const AttitudeController::Config &cfg) {
  return IsAxisValid(cfg.roll) && IsAxisValid(cfg.pitch) &&
         IsAxisValid(cfg.yaw);
}

float ClampSymmetric(float v, float clamp_mag) {
  if (clamp_mag <= 0.0f) return v;
  if (v > clamp_mag) return clamp_mag;
  if (v < -clamp_mag) return -clamp_mag;
  return v;
}

}  // namespace

void AttitudeController::Init(const Config &cfg) {
  if (!IsConfigValid(cfg)) {
    Panic(ErrorCode::Stm32::kAttitudeControllerInvalidConfig);
  }
  cfg_ = cfg;
}

void AttitudeController::SetConfig(const Config &cfg) {
  if (!IsConfigValid(cfg)) {
    Panic(ErrorCode::Stm32::kAttitudeControllerInvalidConfig);
  }
  cfg_ = cfg;
}

Eigen::Vector3f AttitudeController::Step(
    const Eigen::Quaternionf &q_setpoint,
    const Eigen::Quaternionf &q_measured) const {
  // q_err in body frame: applying q_err on the right of q_measured
  // rotates the body to the setpoint.
  //   q_setpoint = q_measured · q_err   →   q_err = q_measured^{-1} · q_setpoint
  Eigen::Quaternionf q_err = q_measured.conjugate() * q_setpoint;

  // Shortest-arc convention: if scalar part is negative, the same
  // rotation can be represented with the conjugate sign, taking the
  // short way around the unit sphere. Without this flip the controller
  // would happily rotate 270° instead of 90°.
  if (q_err.w() < 0.0f) {
    q_err.coeffs() = -q_err.coeffs();
  }

  // Small-angle axis-angle approximation: for small q_err,
  //   q_err ≈ (1, 0.5 · θ),   θ = rotation vector in body frame
  // so θ ≈ 2 · q_err.vec(). For larger errors the approximation
  // saturates at θ = 2 (180°), which is acceptable because the
  // per-axis rate clamp dominates the response anyway.
  const Eigen::Vector3f err_vec = 2.0f * q_err.vec();

  Eigen::Vector3f rate_sp;
  rate_sp.x() =
      ClampSymmetric(cfg_.roll.kp * err_vec.x(), cfg_.roll.rate_clamp);
  rate_sp.y() =
      ClampSymmetric(cfg_.pitch.kp * err_vec.y(), cfg_.pitch.rate_clamp);
  rate_sp.z() =
      ClampSymmetric(cfg_.yaw.kp * err_vec.z(), cfg_.yaw.rate_clamp);
  return rate_sp;
}
