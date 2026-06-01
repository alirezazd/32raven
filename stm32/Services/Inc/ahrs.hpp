// AHRS — Attitude estimator on the unit quaternion.
//
// Mahony complementary filter: fuses body-frame gyro (1 kHz prediction)
// with body-frame accelerometer (gravity reference) to produce a
// drift-bounded orientation estimate. Owned by System like the rate
// controller and mixer; runs once per fast tick from `OnFastTick`.
//
// Behaviour is gated by the gains in Config:
//   - kp_accel = 0 → accel correction off; quaternion drifts on gyro
//                    bias alone (matches a pure-integration AHRS).
//   - kp_accel > 0 → accel pulls the quaternion toward truth at a rate
//                    set by the gain.
//   - ki_bias  > 0 → gyro bias is learned alongside the quaternion.
//
// Trust attenuation (soft, two-sided):
//   - The accel correction is scaled by a smooth weight that's 1.0
//     when |accel|/g is within ±accel_trust_full_dev_g of 1.0, decays
//     smoothly to 0.0 as deviation reaches accel_trust_zero_dev_g, and
//     stays 0.0 beyond. Smoothstep ramp — no chatter at the boundary.
//   - The gyro-bias update is additionally scaled by a one-sided
//     smooth weight that's 1.0 when |gyro| <= gyro_quiescent_full_rad_s,
//     decays to 0.0 as |gyro| reaches gyro_quiescent_zero_rad_s. Stops
//     the bias estimator from absorbing transient errors during
//     maneuvers — matches the PX4 / ArduPilot / INAV spin-rate gate.
// Defaults with all four band fields at 0.0 disable the attenuation
// (weight is always 1) — same behaviour as a wide-open trust window.
//
// Convention (NED world, +Z = down):
//   q = attitude_world_to_body tracks the orientation of body in world,
//   propagated by right-multiplication with a body-frame angular
//   increment:
//     v_world = q · v_body
//     v_body  = q.conjugate() · v_world
//   World "up" is (0, 0, -1). Accel at rest reads specific force, which
//   points along "up" in body frame because the normal force opposes
//   gravity, so a normalised accel sample is the direct measurement of
//   "up" in body.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>

#include "icm42688p.hpp"
#include "vehicle_state.hpp"

class Ahrs {
 public:
  struct Config {
    // Proportional gain on the accel-derived tilt error (rad/s per rad
    // of mismatch). Zero disables the accel correction. Typical
    // operational value ~1.0.
    float kp_accel = 0.0f;
    // Integral gain on the accel-derived tilt error — drives the
    // gyro-bias estimator. Zero freezes the bias estimate at zero.
    // Typical operational value ~0.005-0.01.
    float ki_bias = 0.0f;

    // Soft accel-trust attenuation, expressed as deviations of |a|/g
    // from 1.0. Inside ±full → weight = 1; smoothly ramps to 0 at
    // ±zero; held at 0 beyond. When zero <= full, the attenuation is
    // disabled (weight is always 1) — preserves the wide-open default.
    float accel_trust_full_dev_g = 0.0f;
    float accel_trust_zero_dev_g = 0.0f;

    // Soft gyro-quiescence attenuation for the bias-update path,
    // expressed as |gyro| magnitudes in rad/s. Below full → weight = 1;
    // smoothly ramps to 0 at zero; held at 0 beyond. Stops bias from
    // absorbing wrong corrections during transient maneuvering. When
    // zero <= full, disabled (weight = 1).
    float gyro_quiescent_full_rad_s = 0.0f;
    float gyro_quiescent_zero_rad_s = 0.0f;
  };

  Ahrs() = default;

  // Apply cfg and Reset internal state. Panics on invalid config.
  // Called once by System::InitComponent.
  void Init(const Config &cfg);

  // Clear quaternion to identity, gyro bias to zero, internal state to
  // defaults. Call on arm transitions or after IMU re-init.
  void Reset();

  // Runtime config swap; preserves quaternion + bias state. Use for
  // tuning gains during a SIL session without restarting the filter.
  // Panics on invalid config.
  void SetConfig(const Config &cfg);

  const Config &GetConfig() const { return cfg_; }

  // Process one IMU sample burst. Aggregates the gyro for the rate
  // controller's measurement input and integrates the quaternion with
  // per-sample Mahony correction.
  ImuState Process(const Icm42688p::SampleBatch &batch);

  // Diagnostics.
  const Eigen::Quaternionf &Attitude() const { return q_; }
  const Eigen::Vector3f &GyroBiasEstimate() const { return bias_; }
  const ImuState &Current() const { return state_; }

 private:
  Config cfg_{};
  Eigen::Quaternionf q_ = Eigen::Quaternionf::Identity();
  Eigen::Vector3f bias_ = Eigen::Vector3f::Zero();
  ImuState state_{};
  uint64_t last_sample_ts_us_ = 0;
  bool has_last_sample_ts_ = false;
};
