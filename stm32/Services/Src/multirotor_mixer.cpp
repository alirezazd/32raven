#include "multirotor_mixer.hpp"

#include "error_code.hpp"
#include "panic.hpp"

namespace multirotor_mixer {

namespace {

constexpr float Clamp(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

bool IsConfigValid(const Mixer::Config &cfg) {
  return cfg.idle >= 0.0f && cfg.idle <= 1.0f;
}

}  // namespace

void Mixer::Init(const Config &cfg) {
  if (!IsConfigValid(cfg)) {
    Panic(ErrorCode::Stm32::kMixerInvalidConfig);
  }
  cfg_ = cfg;
  armed_ = false;
}

MotorThrust Mixer::Mix(const Inputs &in) const {
  if (!armed_) {
    return MotorThrust{0.0f, 0.0f, 0.0f, 0.0f};
  }

  // Step 1: matrix multiply -> per-motor axis demand.
  float axis[4];
  for (int i = 0; i < 4; ++i) {
    axis[i] = QuadX::kFactors[i][0] * in.roll_torque +
              QuadX::kFactors[i][1] * in.pitch_torque +
              QuadX::kFactors[i][2] * in.yaw_torque;
  }

  // Step 2: Betaflight-style saturation rescale. If the motor spread
  // exceeds 1.0, scale all axis demands down so they fit. Preserves
  // relative axis torques; sacrifices absolute magnitude.
  float lo = axis[0];
  float hi = axis[0];
  for (int i = 1; i < 4; ++i) {
    if (axis[i] < lo) lo = axis[i];
    if (axis[i] > hi) hi = axis[i];
  }
  const float range = hi - lo;
  if (range > 1.0f) {
    const float scale = 1.0f / range;
    for (int i = 0; i < 4; ++i) {
      axis[i] *= scale;
    }
  }

  // Step 3: add throttle, clamp to [idle, 1]. No airmode (v1).
  MotorThrust out;
  for (int i = 0; i < 4; ++i) {
    out[i] = Clamp(axis[i] + in.thrust, cfg_.idle, 1.0f);
  }
  return out;
}

}  // namespace multirotor_mixer
