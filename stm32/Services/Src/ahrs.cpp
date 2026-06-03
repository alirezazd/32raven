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
inline float SmoothStep01(float t) { return t * t * (3.0f - 2.0f * t); }

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
  state_ = ImuState{};
  last_sample_ts_us_ = 0;
  has_last_sample_ts_ = false;
}

void Ahrs::SetConfig(const Config &cfg) {
  if (!IsConfigValid(cfg)) {
    Panic(ErrorCode::Stm32::kAhrsInvalidConfig);
  }
  cfg_ = cfg;
}

ImuState Ahrs::Process(const Icm42688p::SampleBatch &batch) {
  if (batch.count == 0) return state_;

  auto &imu = Icm42688p::GetInstance();
  Eigen::Vector3f gyro_accum = Eigen::Vector3f::Zero();

  for (uint8_t i = 0; i < batch.count; ++i) {
    const Icm42688p::Sample &raw = batch.samples[i];
    const Icm42688p::ScaledSample s = imu.ScaleSample(raw);
    const Eigen::Vector3f gyro_meas{s.gyro_rad_s[0], s.gyro_rad_s[1],
                                    s.gyro_rad_s[2]};
    const Eigen::Vector3f accel{s.accel_mps2[0], s.accel_mps2[1],
                                s.accel_mps2[2]};
    gyro_accum += gyro_meas;

    // Per-sample dt. Sample 0 uses the previous burst's last timestamp;
    // sample i > 0 uses the previous sample within the burst. The very
    // first sample we ever see (no prior timestamp) skips integration.
    float dt_s = 0.0f;
    if (i == 0) {
      if (has_last_sample_ts_ && raw.timestamp_us > last_sample_ts_us_) {
        dt_s =
            static_cast<float>(raw.timestamp_us - last_sample_ts_us_) * 1e-6f;
      }
    } else {
      const uint64_t prev_ts = batch.samples[i - 1u].timestamp_us;
      if (raw.timestamp_us > prev_ts) {
        dt_s = static_cast<float>(raw.timestamp_us - prev_ts) * 1e-6f;
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
    //
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

  const auto &last = batch.samples[batch.count - 1u];
  state_.timestamp_us = last.timestamp_us;
  state_.gyro_body_rad_s = gyro_accum / static_cast<float>(batch.count);
  state_.attitude_world_to_body = q_;
  // FIFO carries one raw temperature word per sample but die temp is
  // slow-varying (~90 s thermal τ) so the last sample is representative.
  state_.temperature_c = imu.ScaleSample(last).temperature_c;

  last_sample_ts_us_ = last.timestamp_us;
  has_last_sample_ts_ = true;
  return state_;
}
