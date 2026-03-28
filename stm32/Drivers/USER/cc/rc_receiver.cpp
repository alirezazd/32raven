#include "rc_receiver.hpp"

#include "config_storage.hpp"
#include "fc_link.hpp"
#include "panic.hpp"
#include "vehicle_state.hpp"
#include <cstdio>

namespace {

constexpr uint16_t kCalibratedMinUs = 1000;
constexpr uint16_t kCalibratedMaxUs = 2000;
constexpr uint32_t kRcLogPeriodUs = 1000000u;
constexpr uint32_t kRxOnlineTimeoutUs = 1500000u;
constexpr uint8_t kTxOnMinRssi = 10u;
constexpr uint32_t kTxOnDebounceUs = 150000u;
constexpr uint32_t kTxOffDebounceUs = 500000u;

} // namespace

void RcReceiver::Init(const Config &cfg, EE &ee, VehicleState &vehicle_state,
                      FcLink &fc_link) {
  if (initialized_) {
    Panic(ErrorCode::kEepromReinit);
  }

  (void)cfg;
  ee_ = &ee;
  vehicle_state_ = &vehicle_state;
  fc_link_ = &fc_link;
  calibration_ = ConfigStorage::LoadOrInitRcCalibration(ee);
  raw_ = RawData{};
  current_ = RcData{};
  rssi_ = 0;
  radio_status_update_us_ = 0;
  tx_candidate_since_us_ = 0;
  tx_candidate_online_ = false;
  initialized_ = true;
}

uint16_t RcReceiver::ApplyCalibration(uint16_t raw_us, uint16_t min_us,
                                      uint16_t max_us) const {
  if (max_us <= min_us) {
    return raw_us;
  }
  if (raw_us <= min_us) {
    return kCalibratedMinUs;
  }
  if (raw_us >= max_us) {
    return kCalibratedMaxUs;
  }

  const uint32_t input_span = (uint32_t)(max_us - min_us);
  const uint32_t scaled =
      ((uint32_t)(raw_us - min_us) * (kCalibratedMaxUs - kCalibratedMinUs) +
       (input_span / 2u)) /
      input_span;
  return (uint16_t)(kCalibratedMinUs + scaled);
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
      previous.rx_online == current_.rx_online &&
      previous.tx_online == current_.tx_online) {
    return false;
  }

  vehicle_state_->UpdateRc(current_);
  return true;
}

void RcReceiver::LogState(uint32_t now_us) {
  if (fc_link_ == nullptr ||
      (uint32_t)(now_us - last_log_us_) < kRcLogPeriodUs) {
    return;
  }
  last_log_us_ = now_us;

  char line[192];
  int written = std::snprintf(line, sizeof(line), "RC: rssi=%u rx=%u tx=%u",
                              (unsigned)rssi_, current_.rx_online ? 1u : 0u,
                              current_.tx_online ? 1u : 0u);
  const std::size_t log_count =
      (current_.channels.size() < 8u) ? current_.channels.size() : 8u;
  for (std::size_t i = 0;
       i < log_count && written > 0 && written < (int)sizeof(line); ++i) {
    written += std::snprintf(
        line + written, sizeof(line) - (std::size_t)written, " ch%u=%u",
        (unsigned)(stm32_limits::kRcEnabledChannelIndices[i] + 1u),
        (unsigned)current_.channels[i]);
  }
  fc_link_->SendLog("%s", line);
}

void RcReceiver::ProcessRawState(const message::RcChannelsMsg &msg,
                                 uint32_t now_us) {
  raw_.timestamp_us = now_us;

  if (!initialized_ || vehicle_state_ == nullptr) {
    return;
  }

  const RcData previous = current_;
  radio_status_update_us_ = now_us;
  rssi_ = msg.rssi;
  current_.timestamp_us = now_us;
  for (std::size_t i = 0; i < stm32_limits::kRcEnabledChannelCount; ++i) {
    const uint8_t source_index = stm32_limits::kRcEnabledChannelIndices[i];
    const uint16_t raw_value = msg.channels[source_index];
    raw_.channels[i] = raw_value;
    current_.channels[i] =
        (source_index < 4u)
            ? ApplyCalibration(raw_value, calibration_.min_us[source_index],
                               calibration_.max_us[source_index])
            : raw_value;
  }

  RefreshLinkState(now_us);
  (void)PublishIfChanged(previous);
  LogState(now_us);
}

void RcReceiver::Poll(uint32_t now_us) {
  if (!initialized_ || vehicle_state_ == nullptr) {
    return;
  }

  const RcData previous = current_;
  RefreshLinkState(now_us);
  (void)PublishIfChanged(previous);
  LogState(now_us);
}

bool RcReceiver::SaveCalibration() {
  if (!initialized_ || ee_ == nullptr) {
    return false;
  }
  return ConfigStorage::SaveRcCalibration(*ee_, calibration_);
}
