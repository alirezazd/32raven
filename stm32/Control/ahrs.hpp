// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

// Mahony complementary filter on the unit quaternion: the gyro propagates it,
// the accel pulls it toward gravity. NED; q is attitude_world_to_body.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cstdint>
#include <optional>

#include "shared_state.hpp"

class Ahrs {
 public:
  struct Config {
    // rad/s per rad of tilt error; 0 is pure gyro integration.
    float kp_accel = 0.0f;
    // Integral gain of the gyro-bias estimator; 0 holds the bias at zero.
    float ki_bias = 0.0f;

    // |a|/g's distance from 1: accel fully trusted inside full, not at all
    // past zero. zero <= full disables the band.
    float accel_trust_full_dev_g = 0.0f;
    float accel_trust_zero_dev_g = 0.0f;

    // |gyro| in rad/s: the bias learns fully below full, not at all past
    // zero. zero <= full disables the band.
    float gyro_quiescent_full_rad_s = 0.0f;
    float gyro_quiescent_zero_rad_s = 0.0f;

    // |mag|/field_ut's distance from 1: disturbed past enter, clean again
    // inside clear, each only once held for the hold time.
    float mag_enter_band = 0.0f;
    float mag_clear_band = 0.0f;
    uint32_t mag_verdict_hold_us = 0;
  };

  Ahrs() = default;

  // Panics on an invalid config.
  void Init(const Config &cfg, SharedState &blackboard);

  // Consumes the IMU mailbox and hands the slot back to the interrupt.
  EstimatorState Process();

 private:
  // Timed on the samples' own stamps, so a sample held over ticks counts once.
  bool TrackMagInterference(float field_ratio, uint32_t sample_us);

  Config cfg_{};
  SharedState *blackboard_ = nullptr;
  Eigen::Quaternionf q_ = Eigen::Quaternionf::Identity();
  Eigen::Vector3f bias_ = Eigen::Vector3f::Zero();
  // Dates the gap before the next burst's first sample; nullopt until one.
  std::optional<uint64_t> last_imu_sample_us_;
  // The first sample that contradicted the verdict; nullopt while none does.
  std::optional<uint32_t> mag_contrary_since_us_;
};
