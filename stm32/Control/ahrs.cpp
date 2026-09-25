// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "ahrs.hpp"

#include <cmath>

#include "error_code.hpp"
#include "math/physics.hpp"
#include "panic.hpp"

namespace {

Eigen::Vector3f CorrectField(const MagCalibration &cal,
                             const MagnetometerData &mag) {
  const Eigen::Vector3f raw(mag.x - cal.offsets_ut[0],
                            mag.y - cal.offsets_ut[1],
                            mag.z - cal.offsets_ut[2]);
  return Eigen::Map<const Eigen::Matrix<float, 3, 3, Eigen::RowMajor>>(
             &cal.soft_iron[0][0]) *
         raw;
}

bool IsConfigValid(const Ahrs::Config &cfg) {
  if (cfg.kp_accel < 0.0f) return false;
  if (cfg.ki_bias < 0.0f) return false;
  if (cfg.accel_trust_full_dev_g < 0.0f) return false;
  if (cfg.accel_trust_zero_dev_g < 0.0f) return false;
  if (cfg.gyro_quiescent_full_rad_s < 0.0f) return false;
  if (cfg.gyro_quiescent_zero_rad_s < 0.0f) return false;
  if (!(cfg.mag_clear_band > 0.0f)) return false;
  if (cfg.mag_clear_band > cfg.mag_enter_band) return false;
  return true;
}

// Zero slope at both ends, so the weights it ramps have no kink to chatter on.
inline float SmoothStep01(float t) { return t * t * (3.0f - (2.0f * t)); }

float ComputeTwoSidedTrustWeight(float value, float centre, float full_dev,
                                 float zero_dev) {
  if (zero_dev <= full_dev) return 1.0f;
  const float dev = std::fabs(value - centre);
  if (dev <= full_dev) return 1.0f;
  if (dev >= zero_dev) return 0.0f;
  const float t = (dev - full_dev) / (zero_dev - full_dev);
  return 1.0f - SmoothStep01(t);
}

float ComputeOneSidedQuiescentWeight(float value, float full, float zero) {
  if (zero <= full) return 1.0f;
  if (value <= full) return 1.0f;
  if (value >= zero) return 0.0f;
  const float t = (value - full) / (zero - full);
  return 1.0f - SmoothStep01(t);
}

}  // namespace

void Ahrs::Init(const Config &cfg, SharedState &blackboard) {
  if (!IsConfigValid(cfg)) {
    Panic(ErrorCode::Stm32::kAhrsInvalidConfig);
  }
  cfg_ = cfg;
  blackboard_ = &blackboard;
}

EstimatorState Ahrs::Process() {
  const MagnetometerData &raw_mag = blackboard_->GetMagnetometer();
  const MagCalibration &mag_cal = blackboard_->GetMagCalibration();
  MagSample mag{.timestamp_us = raw_mag.timestamp_us,
                .body_ut = CorrectField(mag_cal, raw_mag)};
  if (mag_cal.calibrated && mag.timestamp_us != 0u) {
    mag.interference = TrackMagInterference(
        mag.body_ut.norm() / mag_cal.field_ut, mag.timestamp_us);
  }

  ImuBurstSlot &inbox = blackboard_->ImuBurstMailbox();
  const ImuBurst &burst = inbox.burst;

  // timestamp_us stays 0, so the zero rates read as no data, not stillness.
  // Nothing was read, so the slot is not released.
  if (!inbox.fresh || burst.count == 0) {
    EstimatorState idle{};
    idle.mag = mag;
    idle.attitude_world_to_body = q_;
    return idle;
  }

  Eigen::Vector3f gyro_accum = Eigen::Vector3f::Zero();
  Eigen::Vector3f accel_accum = Eigen::Vector3f::Zero();

  // Corrected here, not in the driver: a calibrator reading the burst must
  // see raw counts, and so must the log a fit is re-derived from.
  const GyroCalibration &gyro_cal = blackboard_->GetGyroCalibration();
  const Eigen::Vector3f gyro_offset{gyro_cal.offsets_rad_s[0],
                                    gyro_cal.offsets_rad_s[1],
                                    gyro_cal.offsets_rad_s[2]};
  const AccelCalibration &accel_cal = blackboard_->GetAccelCalibration();
  const Eigen::Vector3f accel_offset{accel_cal.offsets_mps2[0],
                                     accel_cal.offsets_mps2[1],
                                     accel_cal.offsets_mps2[2]};
  const Eigen::Vector3f accel_gain{accel_cal.gains[0], accel_cal.gains[1],
                                   accel_cal.gains[2]};
  // After the part's own corrections: it turns board vectors into airframe.
  const Eigen::Matrix3f &trim = blackboard_->GetBoardTrim().rotation;

  // One stamp, on the newest sample; the rest sit dt apart behind it, as a
  // ULog reader reconstructs them.
  const float dt_in_burst_s = burst.dt_us * 1e-6f;
  const uint64_t first_ts_us =
      burst.timestamp_us -
      static_cast<uint32_t>(static_cast<float>(burst.count - 1u) * burst.dt_us);

  for (uint8_t i = 0; i < burst.count; ++i) {
    const Eigen::Vector3f gyro_meas =
        trim * (Eigen::Vector3f{static_cast<float>(burst.gyro[0][i]),
                                static_cast<float>(burst.gyro[1][i]),
                                static_cast<float>(burst.gyro[2][i])} *
                    burst.gyro_scale -
                gyro_offset);
    const Eigen::Vector3f accel_raw =
        Eigen::Vector3f{static_cast<float>(burst.accel[0][i]),
                        static_cast<float>(burst.accel[1][i]),
                        static_cast<float>(burst.accel[2][i])} *
        burst.accel_scale;
    const Eigen::Vector3f accel =
        trim * (accel_raw - accel_offset).cwiseProduct(accel_gain);
    gyro_accum += gyro_meas;
    accel_accum += accel;

    // Sample 0 spans the gap back to the previous burst, which only the
    // stored stamp can date; with none yet it is not integrated.
    float dt_s = dt_in_burst_s;
    if (i == 0) {
      dt_s = 0.0f;
      if (last_imu_sample_us_ && first_ts_us > *last_imu_sample_us_) {
        dt_s = static_cast<float>(first_ts_us - *last_imu_sample_us_) * 1e-6f;
      }
    }
    if (dt_s <= 0.0f) continue;

    // At rest the accel measures body "up"; crossed with where q puts up, it
    // is the body rate that would rotate one onto the other.
    Eigen::Vector3f mes_err = Eigen::Vector3f::Zero();
    const float accel_norm = accel.norm();
    if (accel_norm > 1e-3f) {
      const float accel_norm_g = accel_norm / math::kGravityMps2;
      const float accel_trust =
          ComputeTwoSidedTrustWeight(accel_norm_g, 1.0f,
                                     cfg_.accel_trust_full_dev_g,
                                     cfg_.accel_trust_zero_dev_g);
      if (accel_trust > 0.0f) {
        const Eigen::Vector3f world_up(0.0f, 0.0f, -1.0f);
        const Eigen::Vector3f v_ref = q_.conjugate() * world_up;
        const Eigen::Vector3f v_meas = accel / accel_norm;
        mes_err = accel_trust * v_meas.cross(v_ref);
      }
    }

    // Mahony & Hamel's integral term, gated off again while the airframe
    // spins so a manoeuvre is not learned as bias.
    const float gyro_quiescent = ComputeOneSidedQuiescentWeight(
        gyro_meas.norm(), cfg_.gyro_quiescent_full_rad_s,
        cfg_.gyro_quiescent_zero_rad_s);
    bias_ -= cfg_.ki_bias * mes_err * dt_s * gyro_quiescent;

    const Eigen::Vector3f gyro_corrected =
        gyro_meas - bias_ + cfg_.kp_accel * mes_err;

    // First order: q ⊗ (1, ω·dt/2), renormalised.
    const Eigen::Quaternionf dq(1.0f, 0.5f * gyro_corrected.x() * dt_s,
                                0.5f * gyro_corrected.y() * dt_s,
                                0.5f * gyro_corrected.z() * dt_s);
    q_ = (q_ * dq).normalized();
  }

  const uint64_t last_ts_us = burst.timestamp_us;
  const float inv_count = 1.0f / static_cast<float>(burst.count);

  EstimatorState out{};
  out.timestamp_us = last_ts_us;
  out.gyro_body_rad_s = gyro_accum * inv_count;
  out.accel_body_mps2 = accel_accum * inv_count;
  out.mag = mag;
  out.attitude_world_to_body = q_;

  last_imu_sample_us_ = last_ts_us;

  // Keeps the reads above from sinking past the store that hands the slot
  // back to the interrupt.
  std::atomic_signal_fence(std::memory_order_release);
  inbox.fresh = false;
  return out;
}

bool Ahrs::TrackMagInterference(float field_ratio, uint32_t sample_us) {
  // The verdict the previous tick published.
  const bool interference = blackboard_->GetEstimate().mag.interference;
  const float band = interference ? cfg_.mag_clear_band : cfg_.mag_enter_band;
  const bool out = std::fabs(field_ratio - 1.0f) > band;
  if (out == interference) {
    mag_contrary_since_us_.reset();
    return interference;
  }
  if (!mag_contrary_since_us_) {
    mag_contrary_since_us_ = sample_us;
    return interference;
  }
  if (sample_us - *mag_contrary_since_us_ < cfg_.mag_verdict_hold_us) {
    return interference;
  }
  mag_contrary_since_us_.reset();
  return out;
}
