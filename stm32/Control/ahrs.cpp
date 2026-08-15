// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "ahrs.hpp"

#include <cmath>

#include "error_code.hpp"
#include "panic.hpp"

namespace {

constexpr float kGravityMps2 = 9.80665f;

bool IsConfigValid(const Ahrs::Config &cfg) {
  if (cfg.kp_accel < 0.0f) return false;
  if (cfg.ki_bias < 0.0f) return false;
  if (cfg.accel_trust_full_dev_g < 0.0f) return false;
  if (cfg.accel_trust_zero_dev_g < 0.0f) return false;
  if (cfg.gyro_quiescent_full_rad_s < 0.0f) return false;
  if (cfg.gyro_quiescent_zero_rad_s < 0.0f) return false;
  return true;
}

// Cubic-Hermite smoothstep on [0, 1]: zero derivative at both endpoints,
// branchless, ~3 FMAs on Cortex-M4.
inline float SmoothStep01(float t) { return t * t * (3.0f - (2.0f * t)); }

// Two-sided trust weight: 1.0 within ±full_dev of the centre, smooth
// ramp to 0.0 by ±zero_dev, 0.0 beyond. When zero_dev <= full_dev the
// attenuation is disabled (returns 1.0 unconditionally).
float TwoSidedTrustWeight(float value, float centre, float full_dev,
                          float zero_dev) {
  if (zero_dev <= full_dev) return 1.0f;
  const float dev = std::fabs(value - centre);
  if (dev <= full_dev) return 1.0f;
  if (dev >= zero_dev) return 0.0f;
  const float t = (dev - full_dev) / (zero_dev - full_dev);
  return 1.0f - SmoothStep01(t);
}

// One-sided low-pass weight: 1.0 below full, smooth ramp to 0.0 by
// zero, 0.0 above. When zero <= full the attenuation is disabled.
float OneSidedQuiescentWeight(float value, float full, float zero) {
  if (zero <= full) return 1.0f;
  if (value <= full) return 1.0f;
  if (value >= zero) return 0.0f;
  const float t = (value - full) / (zero - full);
  return 1.0f - SmoothStep01(t);
}

}  // namespace

void Ahrs::Init(const Config &cfg) {
  if (!IsConfigValid(cfg)) {
    Panic(ErrorCode::Stm32::kAhrsInvalidConfig);
  }
  cfg_ = cfg;
  Reset();
}

void Ahrs::Reset() {
  q_ = Eigen::Quaternionf::Identity();
  bias_ = Eigen::Vector3f::Zero();
  last_sample_ts_us_ = 0;
  has_last_sample_ts_ = false;
}

void Ahrs::SetConfig(const Config &cfg) {
  if (!IsConfigValid(cfg)) {
    Panic(ErrorCode::Stm32::kAhrsInvalidConfig);
  }
  cfg_ = cfg;
}

EstimatorState Ahrs::Process(SharedState &shared) {
  ImuSampleSlot &inbox = shared.ImuSampleMailbox();
  const ImuSampleBatch &batch = inbox.batch;

  // Nothing to integrate, so the attitude estimate stands and the kinematic
  // fields report zero rather than repeating the previous burst's rates.
  // timestamp_us stays 0, which is what tells a reader the zeros are an absence
  // and not a measurement of stillness. The slot is left alone: with nothing
  // published there is nothing to release.
  if (!inbox.fresh || batch.count == 0) {
    EstimatorState idle{};
    idle.attitude_world_to_body = q_;
    return idle;
  }

  Eigen::Vector3f gyro_accum = Eigen::Vector3f::Zero();
  Eigen::Vector3f accel_accum = Eigen::Vector3f::Zero();

  for (uint8_t i = 0; i < batch.count; ++i) {
    const ImuSample &s = batch.samples[i];
    const Eigen::Vector3f gyro_meas{s.gyro_rad_s[0], s.gyro_rad_s[1],
                                    s.gyro_rad_s[2]};
    const Eigen::Vector3f accel{s.accel_mps2[0], s.accel_mps2[1],
                                s.accel_mps2[2]};
    gyro_accum += gyro_meas;
    accel_accum += accel;

    // Per-sample dt. Sample 0 uses the previous burst's last timestamp;
    // sample i > 0 uses the previous sample within the burst. The very
    // first sample we ever see (no prior timestamp) skips integration.
    float dt_s = 0.0f;
    if (i == 0) {
      if (has_last_sample_ts_ && s.timestamp_us > last_sample_ts_us_) {
        dt_s = static_cast<float>(s.timestamp_us - last_sample_ts_us_) * 1e-6f;
      }
    } else {
      const uint64_t prev_ts = batch.samples[i - 1u].timestamp_us;
      if (s.timestamp_us > prev_ts) {
        dt_s = static_cast<float>(s.timestamp_us - prev_ts) * 1e-6f;
      }
    }
    if (dt_s <= 0.0f) continue;

    // Mahony correction term, attenuated by a smooth trust weight.
    //   v_ref  = q.conjugate() · world_up = where q thinks "up" is,
    //                                       in body frame
    //   v_meas = accel.normalized()       = where "up" actually is,
    //                                       in body frame (specific
    //                                       force at rest)
    //   err    = v_meas × v_ref           = body-frame angular velocity
    //                                       that would rotate ref onto
    //                                       meas (zero when q correct).
    // Accel-trust weight decays smoothly from 1.0 inside the
    // ±accel_trust_full_dev_g band to 0.0 at ±accel_trust_zero_dev_g, so
    // no chatter at the gate boundary.
    Eigen::Vector3f mes_err = Eigen::Vector3f::Zero();
    const float accel_norm = accel.norm();
    if (accel_norm > 1e-3f) {
      const float accel_norm_g = accel_norm / kGravityMps2;
      const float accel_trust =
          TwoSidedTrustWeight(accel_norm_g, 1.0f, cfg_.accel_trust_full_dev_g,
                              cfg_.accel_trust_zero_dev_g);
      if (accel_trust > 0.0f) {
        const Eigen::Vector3f world_up(0.0f, 0.0f, -1.0f);
        const Eigen::Vector3f v_ref = q_.conjugate() * world_up;
        const Eigen::Vector3f v_meas = accel / accel_norm;
        mes_err = accel_trust * v_meas.cross(v_ref);
      }
    }

    // PI gyro-bias update (Mahony & Hamel). mes_err is already accel-trust
    // weighted; a second |gyro| quiescence gate stops bias learning while
    // maneuvering. Both gates must be open for a full update.
    const float gyro_quiescent = OneSidedQuiescentWeight(
        gyro_meas.norm(), cfg_.gyro_quiescent_full_rad_s,
        cfg_.gyro_quiescent_zero_rad_s);
    bias_ -= cfg_.ki_bias * mes_err * dt_s * gyro_quiescent;

    // Corrected body rate. With Config defaults (kp_accel = 0, bias = 0)
    // this collapses to gyro_meas — pure gyro integration.
    const Eigen::Vector3f gyro_corrected =
        gyro_meas - bias_ + cfg_.kp_accel * mes_err;

    // First-order quaternion integration:
    //   q_dot = 0.5 · q ⊗ (0, ω)
    //   q_new ≈ q · (1, 0.5·ω·dt), normalized.
    const Eigen::Quaternionf dq(1.0f, 0.5f * gyro_corrected.x() * dt_s,
                                0.5f * gyro_corrected.y() * dt_s,
                                0.5f * gyro_corrected.z() * dt_s);
    q_ = (q_ * dq).normalized();
  }

  const uint64_t last_ts_us = batch.samples[batch.count - 1u].timestamp_us;
  const float inv_count = 1.0f / static_cast<float>(batch.count);

  EstimatorState out{};
  out.timestamp_us = last_ts_us;
  out.gyro_body_rad_s = gyro_accum * inv_count;
  out.accel_body_mps2 = accel_accum * inv_count;
  out.attitude_world_to_body = q_;

  last_sample_ts_us_ = last_ts_us;
  has_last_sample_ts_ = true;

  // Released only now, with nothing left to read from the slot. Release
  // ordering so the reads above cannot sink past the store that hands the slot
  // back to the interrupt.
  std::atomic_signal_fence(std::memory_order_release);
  inbox.fresh = false;
  return out;
}
