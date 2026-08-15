// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "rc_receiver.hpp"

#include <cstring>

#include "config_storage.hpp"
#include "error_code.hpp"
#include "panic.hpp"
#include "shared_state.hpp"
#include "stm32_config.hpp"
namespace {

constexpr uint16_t kCalibratedMinUs = 1000;
constexpr uint16_t kCalibratedTrimUs = 1500;
constexpr uint16_t kCalibratedMaxUs = 2000;

message::RcMapConfigMsg ToRcMapConfig(const RcReceiver::Config &cfg) {
  return {
      .roll = cfg.roll_channel,
      .pitch = cfg.pitch_channel,
      .yaw = cfg.yaw_channel,
      .throttle = cfg.throttle_channel,
  };
}

ee_schema::RcMap MakeRcMapBlob(const RcReceiver::Config &cfg) {
  ee_schema::RcMap map{};
  ee_schema::RcMap::PopulateHeader(map);
  map.roll_channel = cfg.roll_channel;
  map.pitch_channel = cfg.pitch_channel;
  map.yaw_channel = cfg.yaw_channel;
  map.throttle_channel = cfg.throttle_channel;
  return map;
}

void ApplyRcMapBlob(const ee_schema::RcMap &map, RcReceiver::Config &cfg) {
  cfg.roll_channel = map.roll_channel;
  cfg.pitch_channel = map.pitch_channel;
  cfg.yaw_channel = map.yaw_channel;
  cfg.throttle_channel = map.throttle_channel;
}

uint16_t ScaleSegment(uint16_t raw_us, uint16_t in_min_us, uint16_t in_max_us,
                      uint16_t out_min_us, uint16_t out_max_us) {
  const uint32_t input_span = (uint32_t)(in_max_us - in_min_us);
  const uint32_t output_span = (uint32_t)(out_max_us - out_min_us);
  const uint32_t scaled =
      (((uint32_t)(raw_us - in_min_us) * output_span) + (input_span / 2u)) /
      input_span;
  return (uint16_t)(out_min_us + scaled);
}

}  // namespace

RcReceiver &RcReceiver::GetInstance() {
  static RcReceiver instance;
  return instance;
}

float RcReceiver::NormalizedAxis(uint16_t us) {
  if (us == 0u) return 0.0f;
  const float v = (static_cast<float>(us) - kCalibratedTrimUs) /
                  static_cast<float>(kCalibratedTrimUs - kCalibratedMinUs);
  return v < -1.0f ? -1.0f : (v > 1.0f ? 1.0f : v);
}

float RcReceiver::NormalizedThrottle(uint16_t us) {
  if (us == 0u) return 0.0f;
  const float v = (static_cast<float>(us) - kCalibratedMinUs) /
                  static_cast<float>(kCalibratedMaxUs - kCalibratedMinUs);
  return v < 0.0f ? 0.0f : (v > 1.0f ? 1.0f : v);
}

void RcReceiver::SetThrottleMin(float v) {
  if (v < 0.0f || v > 1.0f) {
    Panic(ErrorCode::Stm32::kRcReceiverInvalidThrottleMin);
  }
  throttle_min_ = v;
}

void RcReceiver::Init(const Config &cfg, EE &ee, SharedState &blackboard) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kEepromReinit);
  }
  throttle_min_ = kPilotThrottleMin;

  if (!IsConfigValid(cfg)) {
    Panic(ErrorCode::Stm32::kRcReceiverInvalidConfig);
  }

  cfg_ = cfg;
  ee_ = &ee;
  blackboard_ = &blackboard;
  calibration_ = ConfigStorage::LoadOrInitRcCalibration(ee);
  const ee_schema::RcMap persisted_map =
      ConfigStorage::LoadOrInitRcMap(ee, MakeRcMapBlob(cfg_));
  Config candidate = cfg_;
  ApplyRcMapBlob(persisted_map, candidate);
  if (IsConfigValid(candidate)) {
    cfg_ = candidate;
  } else if (!ConfigStorage::SaveRcMap(ee, MakeRcMapBlob(cfg_))) {
    Panic(ErrorCode::Stm32::kEepromWriteFailed);
  }
  initialized_ = true;
}

bool RcReceiver::IsConfigValid(const Config &cfg) const {
  return message::IsRcMapConfigValid(ToRcMapConfig(cfg));
}

uint16_t RcReceiver::ApplyCalibration(uint16_t raw_us, uint16_t min_us,
                                      uint16_t trim_us, uint16_t max_us,
                                      int8_t rev) const {
  uint16_t calibrated = raw_us;
  if (max_us <= min_us) {
    calibrated = raw_us;
  } else if (raw_us <= min_us) {
    calibrated = kCalibratedMinUs;
  } else if (raw_us >= max_us) {
    calibrated = kCalibratedMaxUs;
  } else if (trim_us <= min_us || trim_us >= max_us) {
    calibrated = ScaleSegment(raw_us, min_us, max_us, kCalibratedMinUs,
                              kCalibratedMaxUs);
  } else if (raw_us <= trim_us) {
    calibrated = ScaleSegment(raw_us, min_us, trim_us, kCalibratedMinUs,
                              kCalibratedTrimUs);
  } else {
    calibrated = ScaleSegment(raw_us, trim_us, max_us, kCalibratedTrimUs,
                              kCalibratedMaxUs);
  }

  if (rev < 0) {
    calibrated = (uint16_t)(kCalibratedMinUs + kCalibratedMaxUs - calibrated);
  }
  return calibrated;
}

void RcReceiver::RecomputeFromRaw(RcData &out) const {
  out.roll_us = 0;
  out.pitch_us = 0;
  out.yaw_us = 0;
  out.throttle_us = 0;

  if (out.timestamp_us == 0) {
    return;
  }

  for (std::size_t i = 0; i < message::kRcChannelCount; ++i) {
    const uint16_t calibrated_value = ApplyCalibration(
        out.channels_raw[i], calibration_.min_us[i], calibration_.trim_us[i],
        calibration_.max_us[i], calibration_.rev[i]);

    const uint8_t source_channel = (uint8_t)(i + 1u);
    if (source_channel == cfg_.roll_channel) {
      out.roll_us = calibrated_value;
    }
    if (source_channel == cfg_.pitch_channel) {
      out.pitch_us = calibrated_value;
    }
    if (source_channel == cfg_.yaw_channel) {
      out.yaw_us = calibrated_value;
    }
    if (source_channel == cfg_.throttle_channel) {
      out.throttle_us = calibrated_value;
    }
  }
}

bool RcReceiver::PublishIfChanged(const RcData &next) {
  const RcData &published = blackboard_->GetRc();
  if (published.timestamp_us == next.timestamp_us &&
      published.channels_raw == next.channels_raw &&
      published.roll_us == next.roll_us &&
      published.pitch_us == next.pitch_us &&
      published.yaw_us == next.yaw_us &&
      published.throttle_us == next.throttle_us) {
    return false;
  }

  blackboard_->UpdateRc(next);
  return true;
}

void RcReceiver::ProcessRawState(const message::RcChannelsMsg &msg,
                                 uint32_t now_us) {
  if (blackboard_ == nullptr) {
    return;
  }

  RcData next = blackboard_->GetRc();
  next.timestamp_us = now_us;
  for (std::size_t i = 0; i < message::kRcChannelCount; ++i) {
    next.channels_raw[i] = msg.channels[i];
  }
  RecomputeFromRaw(next);
  (void)PublishIfChanged(next);
}

bool RcReceiver::SaveCalibration() {
  if (ee_ == nullptr) {
    return false;
  }
  return ConfigStorage::SaveRcCalibration(*ee_, calibration_);
}

bool RcReceiver::SaveRcMap(const Config &cfg) {
  if (ee_ == nullptr) {
    return false;
  }
  return ConfigStorage::SaveRcMap(*ee_, MakeRcMapBlob(cfg));
}

bool RcReceiver::SetRcMapConfig(const message::RcMapConfigMsg &cfg) {
  if (!message::IsRcMapConfigValid(cfg)) {
    return false;
  }

  Config candidate = cfg_;
  candidate.roll_channel = cfg.roll;
  candidate.pitch_channel = cfg.pitch;
  candidate.yaw_channel = cfg.yaw;
  candidate.throttle_channel = cfg.throttle;
  if (!IsConfigValid(candidate)) {
    return false;
  }

  if (!SaveRcMap(candidate)) {
    return false;
  }

  RcData next = blackboard_->GetRc();
  cfg_ = candidate;
  RecomputeFromRaw(next);
  (void)PublishIfChanged(next);
  return true;
}

bool RcReceiver::SetCalibrationConfig(
    const message::RcCalibrationConfigMsg &cfg) {
  if (!message::IsRcCalibrationConfigValid(cfg)) {
    return false;
  }

  ee_schema::RcCalibration updated = calibration_;
  static_assert(sizeof(updated.min_us) == sizeof(cfg.min_us));
  static_assert(sizeof(updated.max_us) == sizeof(cfg.max_us));
  static_assert(sizeof(updated.trim_us) == sizeof(cfg.trim_us));
  static_assert(sizeof(updated.rev) == sizeof(cfg.rev));
  std::memcpy(updated.min_us, cfg.min_us, sizeof(updated.min_us));
  std::memcpy(updated.max_us, cfg.max_us, sizeof(updated.max_us));
  std::memcpy(updated.trim_us, cfg.trim_us, sizeof(updated.trim_us));
  std::memcpy(updated.rev, cfg.rev, sizeof(updated.rev));

  if (!ConfigStorage::SaveRcCalibration(*ee_, updated)) {
    return false;
  }

  RcData next = blackboard_->GetRc();
  calibration_ = updated;
  RecomputeFromRaw(next);
  (void)PublishIfChanged(next);
  return true;
}
