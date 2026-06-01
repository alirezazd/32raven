// AttitudeController — outer loop on SO(3) for Stabilize / Angle modes.
//
// Takes a desired attitude quaternion and the current attitude estimate,
// produces a body-frame angular-rate setpoint that the rate controller
// can track. Pure P-controller on the small-angle error vector — no
// integrator, no derivative, no internal state. The inner rate loop's
// own PID handles transient behaviour; the attitude controller only
// shapes the *direction* and *magnitude* of the rate command.
//
// Mechanism (per Step()):
//   q_err  = q_measured.conjugate() · q_setpoint   (body-frame rotation
//                                                   from measured to setpoint)
//   sign-flip q_err if q_err.w < 0    (shortest-arc convention)
//   err_vec = 2 · q_err.vec()         (axis-angle small-angle approx,
//                                      ≈ rotation vector for small errors)
//   rate_sp[i] = clamp(kp[i] · err_vec[i], ±rate_clamp[i])
//
// Per-axis gains because pitch+roll are often symmetric but yaw is not,
// and the rate-clamp lets the operator cap how aggressively the drone
// will *try* to slew toward a commanded attitude (separate from the
// rate controller's own output clamp).

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

class AttitudeController {
 public:
  struct AxisConfig {
    // P gain on the axis-angle error component (rad/s of commanded rate
    // per rad of attitude error). Typical operational value 6–8 for a
    // hover-tuned multirotor.
    float kp = 0.0f;
    // Symmetric clamp on the rate setpoint this axis (rad/s, magnitude).
    // <= 0 disables the clamp. Caps how fast the outer loop will ask
    // the rate loop to slew toward the desired attitude.
    float rate_clamp = 0.0f;
  };

  struct Config {
    AxisConfig roll;
    AxisConfig pitch;
    AxisConfig yaw;
  };

  AttitudeController() = default;

  // Apply cfg. Panics on invalid config. Stateless controller — no
  // integrator or filter state to clear, so Reset() is a no-op kept
  // for interface symmetry with the rate controller.
  void Init(const Config &cfg);
  void Reset() {}

  // Runtime config swap. Validates and Panics on invalid input.
  void SetConfig(const Config &cfg);
  const Config &GetConfig() const { return cfg_; }

  // Per-axis runtime setters for tuning UIs / EEPROM overrides /
  // autotune sweeps. Each replaces one axis atomically; the other two
  // axes keep their current config. No validation — runtime tuning
  // callers are responsible for sane inputs, matching the convention
  // of control::Pid::SetGains and RateController::SetRollGains. Use
  // SetConfig() when you want the strict validating path (e.g. when
  // loading freshly-parsed EEPROM data of unknown provenance).
  void SetRollAxis(const AxisConfig &a) { cfg_.roll = a; }
  void SetPitchAxis(const AxisConfig &a) { cfg_.pitch = a; }
  void SetYawAxis(const AxisConfig &a) { cfg_.yaw = a; }

  // Per-axis read-only accessors. Symmetric with the setters above —
  // useful for telemetry and for read-modify-write tuning ("bump roll
  // Kp by 10 %": new = controller.RollAxis(); new.kp *= 1.1; ...).
  const AxisConfig &RollAxis() const { return cfg_.roll; }
  const AxisConfig &PitchAxis() const { return cfg_.pitch; }
  const AxisConfig &YawAxis() const { return cfg_.yaw; }

  // Compute the body-frame angular-rate setpoint (rad/s) that drives
  // q_measured toward q_setpoint. Both quaternions must be unit; we
  // do not normalise here (AHRS produces unit quaternions and the
  // caller is expected to normalise q_setpoint when constructing it).
  Eigen::Vector3f Step(const Eigen::Quaternionf &q_setpoint,
                       const Eigen::Quaternionf &q_measured) const;

 private:
  Config cfg_{};
};
