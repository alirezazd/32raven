#include "rc_receiver.hpp"

#include <cstdio>
#include <cstring>

#include "config_storage.hpp"
#include "fc_link.hpp"
#include "panic.hpp"
#include "vehicle_state.hpp"

namespace {

constexpr uint16_t kCalibratedMinUs = 1000;
constexpr uint16_t kCalibratedTrimUs = 1500;
constexpr uint16_t kCalibratedMaxUs = 2000;
constexpr uint32_t kRxOnlineTimeoutUs = 1500000u;
constexpr uint8_t kTxOnMinRssi = 10u;
constexpr uint32_t kTxOnDebounceUs = 150000u;
constexpr uint32_t kTxOffDebounceUs = 500000u;

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
      ((uint32_t)(raw_us - in_min_us) * output_span + (input_span / 2u)) /
      input_span;
  return (uint16_t)(out_min_us + scaled);
}

}  // namespace

void RcReceiver::Init(const Config &cfg, EE &ee, VehicleState &vehicle_state,
                      FcLink &fc_link) {
  if (initialized_) {
    Panic(ErrorCode::kEepromReinit);
  }

  if (!IsConfigValid(cfg)) {
    Panic(ErrorCode::kRcReceiverInvalidConfig);
  }

  cfg_ = cfg;
  ee_ = &ee;
  vehicle_state_ = &vehicle_state;
  fc_link_ = &fc_link;
  calibration_ = ConfigStorage::LoadOrInitRcCalibration(ee);
  const ee_schema::RcMap persisted_map =
      ConfigStorage::LoadOrInitRcMap(ee, MakeRcMapBlob(cfg_));
  Config candidate = cfg_;
  ApplyRcMapBlob(persisted_map, candidate);
  if (IsConfigValid(candidate)) {
    cfg_ = candidate;
  } else if (!ConfigStorage::SaveRcMap(ee, MakeRcMapBlob(cfg_))) {
    Panic(ErrorCode::kEepromWriteFailed);
  }
  raw_ = RawData{};
  current_ = RcData{};
  rssi_ = 0;
  radio_status_update_us_ = 0;
  tx_candidate_since_us_ = 0;
  tx_candidate_online_ = false;
  initialized_ = true;
}

bool RcReceiver::IsConfigValid(const Config &cfg) const {
  const message::RcMapConfigMsg rc_map = ToRcMapConfig(cfg);
  if (!message::IsRcMapConfigValid(rc_map)) {
    return false;
  }

  return cfg.enabled_channels[cfg.roll_channel - 1u] &&
         cfg.enabled_channels[cfg.pitch_channel - 1u] &&
         cfg.enabled_channels[cfg.yaw_channel - 1u] &&
         cfg.enabled_channels[cfg.throttle_channel - 1u];
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

void RcReceiver::RecomputeCurrentFromRaw() {
  current_.channels.fill(0);
  current_.roll_us = 0;
  current_.pitch_us = 0;
  current_.yaw_us = 0;
  current_.throttle_us = 0;

  if (raw_.timestamp_us == 0) {
    return;
  }

  for (std::size_t i = 0; i < stm32_limits::kRcEnabledChannelCount; ++i) {
    const uint8_t source_index = stm32_limits::kRcEnabledChannelIndices[i];
    const uint16_t raw_value = raw_.channels[i];
    const uint16_t calibrated_value = ApplyCalibration(
        raw_value, calibration_.min_us[source_index],
        calibration_.trim_us[source_index], calibration_.max_us[source_index],
        calibration_.rev[source_index]);
    current_.channels[i] = calibrated_value;

    const uint8_t source_channel = (uint8_t)(source_index + 1u);
    if (source_channel == cfg_.roll_channel) {
      current_.roll_us = calibrated_value;
    }
    if (source_channel == cfg_.pitch_channel) {
      current_.pitch_us = calibrated_value;
    }
    if (source_channel == cfg_.yaw_channel) {
      current_.yaw_us = calibrated_value;
    }
    if (source_channel == cfg_.throttle_channel) {
      current_.throttle_us = calibrated_value;
    }
  }
}

void RcReceiver::RefreshLinkState(uint32_t now_us) {
  current_.rx_online =
      radio_status_update_us_ != 0 &&
      (uint32_t)(now_us - radio_status_update_us_) <= kRxOnlineTimeoutUs;

  const bool desired_tx_online = current_.rx_online && rssi_ >= kTxOnMinRssi;
  if (desired_tx_online != tx_candidate_online_) {
    tx_candidate_online_ = desired_tx_online;
    tx_candidate_since_us_ = now_us;
  }

  const uint32_t debounce_us =
      tx_candidate_online_ ? kTxOnDebounceUs : kTxOffDebounceUs;
  if (current_.tx_online != tx_candidate_online_ &&
      (uint32_t)(now_us - tx_candidate_since_us_) >= debounce_us) {
    current_.tx_online = tx_candidate_online_;
  }
}

bool RcReceiver::PublishIfChanged(const RcData &previous) {
  if (vehicle_state_ == nullptr) {
    return false;
  }

  if (previous.timestamp_us == current_.timestamp_us &&
      previous.channels == current_.channels &&
      previous.roll_us == current_.roll_us &&
      previous.pitch_us == current_.pitch_us &&
      previous.yaw_us == current_.yaw_us &&
      previous.throttle_us == current_.throttle_us &&
      previous.rx_online == current_.rx_online &&
      previous.tx_online == current_.tx_online) {
    return false;
  }

  vehicle_state_->UpdateRc(current_);
  return true;
}

void RcReceiver::ProcessRawState(const message::RcChannelsMsg &msg,
                                 uint32_t now_us) {
  if (!initialized_ || vehicle_state_ == nullptr) {
    return;
  }

  const RcData previous = current_;
  radio_status_update_us_ = now_us;
  rssi_ = msg.rssi;
  raw_.timestamp_us = now_us;
  current_.timestamp_us = now_us;
  for (std::size_t i = 0; i < stm32_limits::kRcEnabledChannelCount; ++i) {
    const uint8_t source_index = stm32_limits::kRcEnabledChannelIndices[i];
    raw_.channels[i] = msg.channels[source_index];
  }
  RecomputeCurrentFromRaw();

  RefreshLinkState(now_us);
  (void)PublishIfChanged(previous);
}

void RcReceiver::Poll(uint32_t now_us) {
  if (!initialized_ || vehicle_state_ == nullptr) {
    return;
  }

  const RcData previous = current_;
  RefreshLinkState(now_us);
  (void)PublishIfChanged(previous);
}

bool RcReceiver::SaveCalibration() {
  if (!initialized_ || ee_ == nullptr) {
    return false;
  }
  return ConfigStorage::SaveRcCalibration(*ee_, calibration_);
}

bool RcReceiver::SaveRcMap(const Config &cfg) {
  if (!initialized_ || ee_ == nullptr) {
    return false;
  }
  return ConfigStorage::SaveRcMap(*ee_, MakeRcMapBlob(cfg));
}

bool RcReceiver::SetRcMapConfig(const message::RcMapConfigMsg &cfg) {
  if (!initialized_ || !message::IsRcMapConfigValid(cfg)) {
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

  const RcData previous = current_;
  cfg_ = candidate;
  RecomputeCurrentFromRaw();
  (void)PublishIfChanged(previous);
  return true;
}

bool RcReceiver::SetCalibrationConfig(
    const message::RcCalibrationConfigMsg &cfg) {
  if (!initialized_ || !message::IsRcCalibrationConfigValid(cfg)) {
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

  const RcData previous = current_;
  calibration_ = updated;
  RecomputeCurrentFromRaw();
  (void)PublishIfChanged(previous);
  return true;
}
