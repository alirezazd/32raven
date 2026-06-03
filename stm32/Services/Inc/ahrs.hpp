// AHRS — Mahony complementary filter on the unit quaternion.
//
// Fuses body-frame gyro (1 kHz prediction) with body-frame accel
// (gravity reference) into a drift-bounded orientation. Runs once per
// fast tick.
//
// Gains (Config):
//   - kp_accel = 0 → accel correction off; pure gyro integration.
//   - kp_accel > 0 → accel pulls the quaternion toward truth.
//   - ki_bias  > 0 → gyro bias learned alongside the quaternion.
//
// Trust attenuation (smoothstep ramps, no boundary chatter):
//   - Accel correction scaled by a two-sided weight: 1.0 within
//     ±accel_trust_full_dev_g of |accel|/g == 1.0, → 0.0 by
//     accel_trust_zero_dev_g, 0.0 beyond.
//   - Bias update additionally scaled by a one-sided weight: 1.0 below
//     gyro_quiescent_full_rad_s, → 0.0 by gyro_quiescent_zero_rad_s.
//     Spin-rate gate (cf. PX4/ArduPilot/INAV) keeps the bias estimator
//     from absorbing transient maneuver errors.
//   All four band fields 0.0 disables attenuation (weight always 1).
//
// Convention (NED world, +Z = down):
//   q = attitude_world_to_body, propagated by right-multiply with a
//   body-frame angular increment:
//     v_world = q · v_body ; v_body = q.conjugate() · v_world
//   World "up" is (0, 0, -1). At rest accel reads specific force along
//   body "up" (normal force opposes gravity), so a normalised sample is
//   a direct measurement of "up" in body.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>

#include "icm42688p.hpp"
#include "vehicle_state.hpp"

class Ahrs {
 public:
  struct Config {
    // Tilt-error proportional gain (rad/s per rad of mismatch); 0
    // disables accel correction. Typical ~1.0.
    float kp_accel = 0.0f;
    // Tilt-error integral gain driving the bias estimator; 0 freezes
    // bias at zero. Typical ~0.005-0.01.
    float ki_bias = 0.0f;

    // Accel-trust band, as deviations of |a|/g from 1.0: weight 1
    // inside ±full, ramps to 0 by ±zero. zero <= full disables it.
    float accel_trust_full_dev_g = 0.0f;
    float accel_trust_zero_dev_g = 0.0f;

    // Gyro-quiescence band for the bias path, |gyro| in rad/s: weight 1
    // below full, ramps to 0 by zero. zero <= full disables it.
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
  // tuning gains at runtime without restarting the filter.
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
