// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "rc_receiver.hpp"

#include "ee_config_storage.hpp"
#include "error_code.hpp"
#include "panic.hpp"
#include "shared_state.hpp"
#include "stm32_config.hpp"

namespace {

// The CRSF range as CrsfTicksToUs lands it: 1500 at centre, 1000 and 2000 at
// the sticks' ends.
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
  const ee_schema::RcMap persisted_map =
      EeConfigStorage::LoadOrInitRcMap(ee, MakeRcMapBlob(cfg_));
  Config candidate = cfg_;
  ApplyRcMapBlob(persisted_map, candidate);
  if (IsConfigValid(candidate)) {
    cfg_ = candidate;
  } else if (!EeConfigStorage::SaveRcMap(ee, MakeRcMapBlob(cfg_))) {
    Panic(ErrorCode::Stm32::kEepromWriteFailed);
  }
  initialized_ = true;
}

bool RcReceiver::IsConfigValid(const Config &cfg) const {
  return message::IsRcMapConfigValid(ToRcMapConfig(cfg));
}

void RcReceiver::RecomputeFromRaw(RcData &out) const {
  out.roll_us = 0;
  out.pitch_us = 0;
  out.yaw_us = 0;
  out.throttle_us = 0;

  if (out.timestamp_us == 0u) {
    return;
  }

  for (std::size_t i = 0; i < message::kRcChannelCount; ++i) {
    const uint16_t us = out.channels_raw[i];
    const uint8_t source_channel = (uint8_t)(i + 1u);
    if (source_channel == cfg_.roll_channel) {
      out.roll_us = us;
    }
    if (source_channel == cfg_.pitch_channel) {
      out.pitch_us = us;
    }
    if (source_channel == cfg_.yaw_channel) {
      out.yaw_us = us;
    }
    if (source_channel == cfg_.throttle_channel) {
      out.throttle_us = us;
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
  // Local arrival time, not anything the transmitter sent, which is what
  // makes the published stamp ageable by a reader. Zero is reserved for "no
  // frame yet", so the one microsecond per wrap that lands on it borrows the
  // next and the sentinel stays exact.
  next.timestamp_us = (now_us == 0u) ? 1u : now_us;
  for (std::size_t i = 0; i < message::kRcChannelCount; ++i) {
    next.channels_raw[i] = msg.channels[i];
  }
  RecomputeFromRaw(next);
  (void)PublishIfChanged(next);
}

bool RcReceiver::SaveRcMap(const Config &cfg) {
  if (ee_ == nullptr) {
    return false;
  }
  return EeConfigStorage::SaveRcMap(*ee_, MakeRcMapBlob(cfg));
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
