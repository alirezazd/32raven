// AttitudeController — outer loop on SO(3) for Stabilize / Angle modes.
//
// Stateless pure-P on the small-angle attitude error; emits a body-frame
// angular-rate setpoint for the inner rate loop to track. Per Step():
//   q_err   = q_measured.conjugate() · q_setpoint   (body-frame error)
//   if q_err.w < 0: negate q_err                    (shortest-arc)
//   err_vec = 2 · q_err.vec()                        (rotation vector,
//   small-angle) rate_sp[i] = clamp(kp[i] · err_vec[i], ±rate_clamp[i])
//
// Per-axis gains: roll/pitch are typically symmetric, yaw is not. rate_clamp
// caps slew toward a commanded attitude, independent of the rate loop's
// own output clamp.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

class AttitudeController {
 public:
  struct AxisConfig {
    // P gain, (rad/s rate) per (rad attitude error). Typically 6–8 hover-tuned.
    float kp = 0.0f;
    // Symmetric rate-setpoint clamp, rad/s magnitude. <= 0 disables.
    float rate_clamp = 0.0f;
  };

  struct Config {
    AxisConfig roll;
    AxisConfig pitch;
    AxisConfig yaw;
  };

  AttitudeController() = default;

  // Apply cfg. Panics on invalid config. Reset() is a no-op (stateless);
  // kept for interface symmetry with the rate controller.
  void Init(const Config &cfg);
  void Reset() {}

  // Runtime config swap. Validates and Panics on invalid input.
  void SetConfig(const Config &cfg);
  const Config &GetConfig() const { return cfg_; }

  // Per-axis runtime setters for tuning UIs / autotune. No validation
  // (matches control::Pid::SetGains); caller ensures sane inputs. Use
  // SetConfig() for the strict validating path (e.g. EEPROM loads).
  void SetRollAxis(const AxisConfig &a) { cfg_.roll = a; }
  void SetPitchAxis(const AxisConfig &a) { cfg_.pitch = a; }
  void SetYawAxis(const AxisConfig &a) { cfg_.yaw = a; }

  // Per-axis read-only accessors, for telemetry and read-modify-write tuning.
  const AxisConfig &RollAxis() const { return cfg_.roll; }
  const AxisConfig &PitchAxis() const { return cfg_.pitch; }
  const AxisConfig &YawAxis() const { return cfg_.yaw; }

  // Body-frame angular-rate setpoint (rad/s) driving q_measured -> q_setpoint.
  // Both quaternions must be unit; not normalised here (caller's
  // responsibility).
  Eigen::Vector3f Step(const Eigen::Quaternionf &q_setpoint,
                       const Eigen::Quaternionf &q_measured) const;

 private:
  Config cfg_{};
};
