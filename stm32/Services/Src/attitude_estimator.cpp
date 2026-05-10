#include "attitude_estimator.hpp"

#include "math/quaternion.hpp"
#include "math/vec3.hpp"

namespace attitude_estimator {

void AttitudeEstimator::Init(const Config &cfg) {
  cfg_ = cfg;
  Reset();
}

void AttitudeEstimator::Reset() {
  state_ = ImuState{};  // attitude → identity, gyro → 0, timestamp → 0
}

ImuState AttitudeEstimator::Process(const Icm42688p::SampleBatch &batch) {
  if (batch.count == 0) {
    return state_;
  }

  auto &imu = Icm42688p::GetInstance();
  math::Vec3 gyro_accum{};
  math::Quaternion q = state_.attitude_world_to_body;

  for (uint8_t i = 0; i < batch.count; ++i) {
    const Icm42688p::Sample &raw = batch.samples[i];
    const Icm42688p::ScaledSample scaled = imu.ScaleSample(raw);
    const math::Vec3 omega{scaled.gyro_rad_s[0], scaled.gyro_rad_s[1],
                           scaled.gyro_rad_s[2]};
    gyro_accum = gyro_accum + omega;

    // 7 in-burst sub-steps at the sample's own dt (≈125 µs nominal).
    // Skip the first sample of the burst because its dt-from-previous-
    // burst isn't tracked here: a missing 1/8 sub-step is sub-millidegree
    // per tick, well below what gyro-bias drift accumulates over the
    // same window — Phase C's MEKF will own the cross-burst gap.
    if (i > 0) {
      const uint64_t prev_ts = batch.samples[i - 1u].timestamp_us;
      if (raw.timestamp_us > prev_ts) {
        const float dt_s =
            static_cast<float>(raw.timestamp_us - prev_ts) * 1e-6f;
        q = math::IntegrateBodyRate(q, omega, dt_s);
      }
    }
  }

  state_.timestamp_us = batch.samples[batch.count - 1u].timestamp_us;
  state_.gyro_body_rad_s =
      gyro_accum * (1.0f / static_cast<float>(batch.count));
  state_.attitude_world_to_body = q;
  return state_;
}

}  // namespace attitude_estimator
