#include "crsf_link_service.hpp"

#include <cstring>

#include "board.h"
#include "fc_link.hpp"
#include "rc_receiver.hpp"
#include "stm32_config.hpp"
#include "uart.hpp"
#include "vehicle_state.hpp"

namespace {

constexpr uint8_t kCrsfSerialSyncByte = 0xC8u;
constexpr uint8_t kCrsfBroadcastAddress = 0x00u;
constexpr uint8_t kCrsfAddressRemoteControl = 0xEAu;
constexpr uint8_t kCrsfFrameTypeGps = 0x02u;
constexpr uint8_t kCrsfFrameTypeHeartbeat = 0x0Bu;
constexpr uint8_t kCrsfFrameTypeBattery = 0x08u;
constexpr uint8_t kCrsfFrameTypeLinkStatistics = 0x14u;
constexpr uint8_t kCrsfFrameTypeRcChannelsPacked = 0x16u;
constexpr uint8_t kCrsfFrameTypeDirectCommand = 0x32u;
constexpr uint8_t kCrsfAddressFlightController = 0xC8u;
constexpr uint8_t kCrsfAddressReceiver = 0xECu;
constexpr uint8_t kCrsfAddressTransmitter = 0xEEu;
constexpr uint8_t kCrsfCommandIdCrossfire = 0x10u;
constexpr uint8_t kCrsfCrossfireSubcmdBind = 0x01u;
constexpr uint8_t kCrsfCrossfireSubcmdCancelBind = 0x02u;
constexpr uint8_t kCrsfLinkStatisticsPayloadSize = 10u;
constexpr uint8_t kCrsfRcChannelsPayloadSize = 22u;
constexpr uint8_t kCrsfMinFrameLength = 2u;
constexpr uint8_t kCrsfMaxFrameLength = 62u;

constexpr uint32_t kFcLinkRcForwardIntervalUs =
    kFcLinkTelemetryIntervalMs * 1000u;

constexpr uint8_t kGpsPayloadSize = 15u;
constexpr uint8_t kHeartbeatPayloadSize = 2u;
constexpr uint8_t kBatteryPayloadSize = 8u;

uint8_t CrsfCrc8(const uint8_t *data, std::size_t len) {
  uint8_t crc = 0;
  for (std::size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8u; ++bit) {
      crc = (crc & 0x80u) != 0u ? (uint8_t)((crc << 1u) ^ 0xD5u)
                                : (uint8_t)(crc << 1u);
    }
  }
  return crc;
}

uint8_t CrsfCommandCrc8(const uint8_t *data, std::size_t len) {
  uint8_t crc = 0;
  for (std::size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8u; ++bit) {
      crc = (crc & 0x80u) != 0u ? (uint8_t)((crc << 1u) ^ 0xBAu)
                                : (uint8_t)(crc << 1u);
    }
  }
  return crc;
}

uint8_t ClampU8(uint32_t value) {
  return value > 0xFFu ? 0xFFu : (uint8_t)value;
}

uint16_t ClampU16(uint32_t value) {
  return value > 0xFFFFu ? 0xFFFFu : (uint16_t)value;
}

uint32_t ClampU24(uint32_t value) {
  return value > 0xFFFFFFu ? 0xFFFFFFu : value;
}

int32_t MmToMetersRounded(int32_t millimeters) {
  if (millimeters >= 0) {
    return (millimeters + 500) / 1000;
  }
  return -(((-millimeters) + 500) / 1000);
}

uint32_t EncodeGroundSpeedKphTenths(uint16_t speed_cm_s) {
  return ((uint32_t)speed_cm_s * 36u + 5u) / 10u;
}

uint32_t EncodeBatteryVoltageTenths(float voltage_v) {
  if (voltage_v <= 0.0f) {
    return 0;
  }
  return (uint32_t)(voltage_v * 10.0f + 0.5f);
}

uint32_t EncodeBatteryCurrentTenths(float current_a) {
  if (current_a <= 0.0f) {
    return 0;
  }
  return (uint32_t)(current_a * 10.0f + 0.5f);
}

uint32_t EncodeMah(float mah_drawn) {
  if (mah_drawn <= 0.0f) {
    return 0;
  }
  return (uint32_t)(mah_drawn + 0.5f);
}

void StoreBe16(uint8_t *dst, uint16_t value) {
  dst[0] = (uint8_t)(value >> 8u);
  dst[1] = (uint8_t)value;
}

void StoreBe24(uint8_t *dst, uint32_t value) {
  dst[0] = (uint8_t)(value >> 16u);
  dst[1] = (uint8_t)(value >> 8u);
  dst[2] = (uint8_t)value;
}

void StoreBe32(uint8_t *dst, uint32_t value) {
  dst[0] = (uint8_t)(value >> 24u);
  dst[1] = (uint8_t)(value >> 16u);
  dst[2] = (uint8_t)(value >> 8u);
  dst[3] = (uint8_t)value;
}

void EncodeGpsPayload(const GpsData &gps, uint8_t payload[kGpsPayloadSize]) {
  StoreBe32(payload + 0u, (uint32_t)gps.lat);
  StoreBe32(payload + 4u, (uint32_t)gps.lon);
  StoreBe16(payload + 8u, ClampU16(EncodeGroundSpeedKphTenths(gps.vel)));
  StoreBe16(payload + 10u, gps.hdg);
  const int32_t altitude_m = MmToMetersRounded(gps.alt);
  const int32_t altitude_offset_m = altitude_m + 1000;
  const uint16_t encoded_altitude =
      altitude_offset_m <= 0 ? 0u : ClampU16((uint32_t)altitude_offset_m);
  StoreBe16(payload + 12u, encoded_altitude);
  payload[14] = gps.num_sats;
}

void EncodeHeartbeatPayload(uint8_t payload[kHeartbeatPayloadSize]) {
  StoreBe16(payload, kCrsfAddressFlightController);
}

void EncodeBatteryPayload(const BatteryData &battery,
                          uint8_t payload[kBatteryPayloadSize]) {
  // Match the de-facto FC/EdgeTX CRSF battery encoding used in common stacks.
  StoreBe16(payload + 0u,
            ClampU16(EncodeBatteryVoltageTenths(battery.voltage)));
  StoreBe16(payload + 2u,
            ClampU16(EncodeBatteryCurrentTenths(battery.current)));
  StoreBe24(payload + 4u, ClampU24(EncodeMah(battery.mah_drawn)));
  payload[7] = ClampU8(battery.percentage);
}

bool IsCrsfSyncByte(uint8_t byte) {
  return byte == kCrsfSerialSyncByte || byte == kCrsfBroadcastAddress ||
         byte == kCrsfAddressFlightController ||
         byte == kCrsfAddressRemoteControl || byte == kCrsfAddressReceiver ||
         byte == kCrsfAddressTransmitter;
}

uint16_t CrsfTicksToUs(uint16_t ticks) {
  const int32_t centered_ticks = (int32_t)ticks - 992;
  const int32_t us = 1500 + ((centered_ticks * 5) / 8);
  if (us <= 0) {
    return 0;
  }
  return (uint16_t)us;
}

}  // namespace

void CrsfLinkService::Init(const Config &cfg, Uart6 &uart,
                           VehicleState &vehicle_state, RcReceiver &rc_receiver,
                           FcLink &fc_link) {
  if (initialized_) {
    ErrorHandler();
  }

  cfg_ = cfg;
  uart_ = &uart;
  vehicle_state_ = &vehicle_state;
  rc_receiver_ = &rc_receiver;
  fc_link_ = &fc_link;
  scheduler_started_ = false;
  last_forward_us_ = 0;
  last_forwarded_flags_ = 0;
  have_forwarded_state_ = false;
  last_link_quality_ = 0;
  pending_forward_ = false;
  crsf_have_length_ = false;
  crsf_frame_len_ = 0;
  crsf_frame_pos_ = 0;
  crsf_frame_.fill(0);
  telemetry_states_.fill(TopicState{});
  latest_rx_msg_ = {};
  pending_command_ = PendingCommand::kNone;
  initialized_ = true;
}

void CrsfLinkService::PollRx(uint32_t now_us, size_t byte_budget) {
  if (!initialized_ || uart_ == nullptr) {
    return;
  }

  uint8_t byte = 0;
  size_t count = 0;
  while (count < byte_budget && uart_->Read(byte)) {
    ++count;
    (void)ProcessCrsfByte(byte, now_us);
  }

}

void CrsfLinkService::PollTx(uint32_t now_us, size_t max_frames) {
  if (!initialized_ || uart_ == nullptr || vehicle_state_ == nullptr) {
    return;
  }

  ForwardLatestRawState(now_us);
  PrimeScheduler(now_us);

  const size_t frame_budget =
      max_frames != 0u ? max_frames : (size_t)cfg_.max_frames_per_poll;
  size_t sent = 0;
  while (sent < frame_budget) {
    if (TrySendPendingCommand() || SendScheduledTelemetry(now_us)) {
      ++sent;
      continue;
    }
    break;
  }
}

void CrsfLinkService::RequestReceiverBind() {
  pending_command_ = PendingCommand::kReceiverBind;
}

void CrsfLinkService::RequestReceiverCancelBind() {
  pending_command_ = PendingCommand::kReceiverCancelBind;
}

bool CrsfLinkService::SendScheduledTelemetry(uint32_t now_us) {
  // TODO(crsf): Extend this scheduler with flight mode, attitude, vario/baro,
  // temperature, voltage/cell data, and rpm once those blackboard sources are
  // available on STM32.
  std::array<bool, static_cast<size_t>(TelemetryTopic::kCount)> skipped{};
  size_t skipped_count = 0;

  while (skipped_count < telemetry_states_.size()) {
    bool have_candidate = false;
    TelemetryTopic best_topic = TelemetryTopic::kHeartbeat;
    uint8_t best_priority = 0;
    uint32_t best_overdue_us = 0;

    for (size_t i = 0; i < telemetry_states_.size(); ++i) {
      if (skipped[i]) {
        continue;
      }

      const auto topic = static_cast<TelemetryTopic>(i);
      const TopicConfig &topic_cfg = GetTelemetryTopicConfig(topic);
      TopicState &topic_state = telemetry_states_[i];

      if (topic_cfg.period_us == 0u) {
        skipped[i] = true;
        ++skipped_count;
        continue;
      }

      if ((int32_t)(now_us - topic_state.next_due_us) < 0) {
        skipped[i] = true;
        ++skipped_count;
        continue;
      }

      if (!IsTelemetryTopicReady(topic, now_us)) {
        topic_state.next_due_us = now_us + topic_cfg.period_us;
        skipped[i] = true;
        ++skipped_count;
        continue;
      }

      const uint32_t overdue_us = now_us - topic_state.next_due_us;
      if (!have_candidate || topic_cfg.priority > best_priority ||
          (topic_cfg.priority == best_priority &&
           overdue_us > best_overdue_us)) {
        have_candidate = true;
        best_topic = topic;
        best_priority = topic_cfg.priority;
        best_overdue_us = overdue_us;
      }
    }

    if (!have_candidate) {
      return false;
    }

    uint8_t type = 0;
    uint8_t payload[16] = {};
    uint8_t payload_len = 0;
    if (!PrepareTelemetryTopic(best_topic, now_us, type, payload, payload_len)) {
      skipped[static_cast<size_t>(best_topic)] = true;
      ++skipped_count;
      continue;
    }

    TopicState &topic_state = GetTelemetryTopicState(best_topic);
    if (!ShouldSendTelemetryTopic(best_topic, now_us, payload, payload_len)) {
      topic_state.next_due_us =
          ComputeDeferredDueTime(best_topic, now_us, payload, payload_len);
      skipped[static_cast<size_t>(best_topic)] = true;
      ++skipped_count;
      continue;
    }

    if (!TrySendTelemetryTopic(best_topic, now_us, payload, payload_len, type)) {
      return false;
    }

    topic_state.last_sent_us = now_us;
    topic_state.next_due_us = now_us + GetTelemetryTopicConfig(best_topic).period_us;
    topic_state.last_payload_len = payload_len;
    topic_state.have_last_payload = true;
    if (payload_len > 0u) {
      memcpy(topic_state.last_payload.data(), payload, payload_len);
    }
    return true;
  }

  return false;
}

void CrsfLinkService::PrimeScheduler(uint32_t now_us) {
  if (scheduler_started_) {
    return;
  }

  for (size_t i = 0; i < telemetry_states_.size(); ++i) {
    const auto topic = static_cast<TelemetryTopic>(i);
    telemetry_states_[i] = TopicState{
        .next_due_us = now_us + GetTelemetryTopicConfig(topic).start_delay_us,
        .last_sent_us = 0,
    };
  }
  scheduler_started_ = true;
}

const CrsfLinkService::TopicConfig &CrsfLinkService::GetTelemetryTopicConfig(
    TelemetryTopic topic) const {
  switch (topic) {
    case TelemetryTopic::kHeartbeat:
      return cfg_.heartbeat;
    case TelemetryTopic::kGps:
      return cfg_.gps;
    case TelemetryTopic::kBattery:
      return cfg_.battery;
    case TelemetryTopic::kCount:
    default:
      return cfg_.heartbeat;
  }
}

CrsfLinkService::TopicState &CrsfLinkService::GetTelemetryTopicState(
    TelemetryTopic topic) {
  return telemetry_states_[static_cast<size_t>(topic)];
}

bool CrsfLinkService::IsTelemetryTopicReady(TelemetryTopic topic,
                                            uint32_t now_us) const {
  if (vehicle_state_ == nullptr) {
    return false;
  }

  switch (topic) {
    case TelemetryTopic::kHeartbeat:
    case TelemetryTopic::kBattery:
      return true;
    case TelemetryTopic::kGps: {
      const GpsData &gps = vehicle_state_->GetGps();
      return gps.timestamp_us != 0 &&
             (uint32_t)(now_us - gps.timestamp_us) <= cfg_.gps_fresh_timeout_us;
    }
    case TelemetryTopic::kCount:
    default:
      return false;
  }
}

bool CrsfLinkService::PrepareTelemetryTopic(TelemetryTopic topic,
                                            uint32_t now_us, uint8_t &type,
                                            uint8_t *payload,
                                            uint8_t &payload_len) const {
  if (payload == nullptr) {
    return false;
  }

  switch (topic) {
    case TelemetryTopic::kHeartbeat:
      type = kCrsfFrameTypeHeartbeat;
      payload_len = kHeartbeatPayloadSize;
      EncodeHeartbeatPayload(payload);
      return true;
    case TelemetryTopic::kGps: {
      const GpsData &gps = vehicle_state_->GetGps();
      if (gps.timestamp_us == 0 ||
          (uint32_t)(now_us - gps.timestamp_us) > cfg_.gps_fresh_timeout_us) {
        return false;
      }
      type = kCrsfFrameTypeGps;
      payload_len = kGpsPayloadSize;
      EncodeGpsPayload(gps, payload);
      return true;
    }
    case TelemetryTopic::kBattery: {
      const BatteryData &battery = vehicle_state_->GetBattery();
      type = kCrsfFrameTypeBattery;
      payload_len = kBatteryPayloadSize;
      EncodeBatteryPayload(battery, payload);
      return true;
    }
    case TelemetryTopic::kCount:
    default:
      return false;
  }
}

bool CrsfLinkService::ShouldSendTelemetryTopic(TelemetryTopic topic,
                                               uint32_t now_us,
                                               const uint8_t *payload,
                                               uint8_t payload_len) const {
  const TopicConfig &topic_cfg = GetTelemetryTopicConfig(topic);
  const TopicState &topic_state =
      telemetry_states_[static_cast<size_t>(topic)];

  if (!topic_cfg.send_on_change || !topic_state.have_last_payload) {
    return true;
  }

  const bool changed =
      payload_len != topic_state.last_payload_len ||
      memcmp(topic_state.last_payload.data(), payload, payload_len) != 0;
  if (changed) {
    return true;
  }

  if (topic_cfg.max_silence_us == 0u) {
    return false;
  }

  return topic_state.last_sent_us == 0u ||
         (uint32_t)(now_us - topic_state.last_sent_us) >=
             topic_cfg.max_silence_us;
}

uint32_t CrsfLinkService::ComputeDeferredDueTime(TelemetryTopic topic,
                                                 uint32_t now_us,
                                                 const uint8_t *payload,
                                                 uint8_t payload_len) const {
  (void)payload;
  (void)payload_len;
  const TopicConfig &topic_cfg = GetTelemetryTopicConfig(topic);
  const TopicState &topic_state =
      telemetry_states_[static_cast<size_t>(topic)];
  const uint32_t period_due = now_us + topic_cfg.period_us;
  if (topic_cfg.max_silence_us == 0u || topic_state.last_sent_us == 0u) {
    return period_due;
  }

  const uint32_t silence_due = topic_state.last_sent_us + topic_cfg.max_silence_us;
  return (int32_t)(period_due - silence_due) < 0 ? period_due : silence_due;
}

bool CrsfLinkService::TrySendTelemetryTopic(TelemetryTopic topic,
                                            uint32_t now_us,
                                            const uint8_t *payload,
                                            uint8_t payload_len, uint8_t type) {
  (void)topic;
  (void)now_us;
  return SendBroadcastFrame(type, payload, payload_len);
}

bool CrsfLinkService::TrySendHeartbeatTelemetry() {
  uint8_t payload[kHeartbeatPayloadSize];
  EncodeHeartbeatPayload(payload);
  return SendBroadcastFrame(kCrsfFrameTypeHeartbeat, payload, sizeof(payload));
}

bool CrsfLinkService::TrySendGpsTelemetry(const uint8_t *payload,
                                          uint8_t payload_len) {
  return SendBroadcastFrame(kCrsfFrameTypeGps, payload, payload_len);
}

bool CrsfLinkService::TrySendBatteryTelemetry() {
  const BatteryData &battery = vehicle_state_->GetBattery();
  uint8_t payload[kBatteryPayloadSize];
  EncodeBatteryPayload(battery, payload);
  return SendBroadcastFrame(kCrsfFrameTypeBattery, payload, sizeof(payload));
}

bool CrsfLinkService::TrySendPendingCommand() {
  if (pending_command_ == PendingCommand::kNone) {
    return false;
  }

  // TODO(crsf): Bind/cancel-bind is currently fire-and-forget. Parse command
  // ACKs and enforce timeout/retry or Panic() policy once the CRSF ACK path is
  // implemented.
  uint8_t payload = 0;
  switch (pending_command_) {
    case PendingCommand::kReceiverBind:
      payload = kCrsfCrossfireSubcmdBind;
      break;
    case PendingCommand::kReceiverCancelBind:
      payload = kCrsfCrossfireSubcmdCancelBind;
      break;
    case PendingCommand::kNone:
    default:
      return false;
  }

  if (!SendDirectCommand(kCrsfAddressReceiver, kCrsfCommandIdCrossfire,
                         &payload, 1u)) {
    return false;
  }

  pending_command_ = PendingCommand::kNone;
  return true;
}

bool CrsfLinkService::ParseLinkStatisticsFrame(const uint8_t *payload,
                                               std::size_t len,
                                               uint32_t now_us) {
  if (payload == nullptr || len < kCrsfLinkStatisticsPayloadSize) {
    return false;
  }

  (void)now_us;
  last_link_quality_ = payload[2];
  return true;
}

bool CrsfLinkService::ParseRcChannelsFrame(const uint8_t *payload,
                                           std::size_t len,
                                           uint32_t now_us) {
  if (payload == nullptr || len < kCrsfRcChannelsPayloadSize ||
      rc_receiver_ == nullptr) {
    return false;
  }

  uint32_t bit_buffer = 0;
  uint8_t bits_available = 0;
  std::size_t byte_index = 0;
  message::RcChannelsMsg msg{};

  for (std::size_t channel = 0; channel < 16u; ++channel) {
    while (bits_available < 11u && byte_index < len) {
      bit_buffer |= (uint32_t)payload[byte_index++] << bits_available;
      bits_available = (uint8_t)(bits_available + 8u);
    }
    if (bits_available < 11u) {
      return false;
    }

    const uint16_t ticks = (uint16_t)(bit_buffer & 0x7FFu);
    bit_buffer >>= 11u;
    bits_available = (uint8_t)(bits_available - 11u);
    msg.channels[channel] = CrsfTicksToUs(ticks);
  }

  msg.rssi = last_link_quality_;
  msg.flags = 0;
  latest_rx_msg_ = msg;
  pending_forward_ = true;
  rc_receiver_->ProcessRawState(msg, now_us);
  return true;
}

bool CrsfLinkService::FinishCrsfFrame(uint32_t now_us) {
  if (!crsf_have_length_ ||
      crsf_frame_pos_ != (std::size_t)(crsf_frame_len_ + 2u) ||
      crsf_frame_len_ < kCrsfMinFrameLength) {
    return false;
  }

  const uint8_t type = crsf_frame_[2];
  const std::size_t payload_len = (std::size_t)(crsf_frame_len_ - 2u);
  const uint8_t *payload = crsf_frame_.data() + 3u;
  const uint8_t expected_crc = crsf_frame_[crsf_frame_pos_ - 1u];
  const uint8_t actual_crc =
      CrsfCrc8(crsf_frame_.data() + 2u, payload_len + 1u);
  if (actual_crc != expected_crc) {
    return false;
  }

  switch (type) {
    case kCrsfFrameTypeLinkStatistics:
      return ParseLinkStatisticsFrame(payload, payload_len, now_us);
    case kCrsfFrameTypeRcChannelsPacked:
      return ParseRcChannelsFrame(payload, payload_len, now_us);
    default:
      return false;
  }
}

bool CrsfLinkService::ProcessCrsfByte(uint8_t byte, uint32_t now_us) {
  if (crsf_frame_pos_ == 0u) {
    if (!IsCrsfSyncByte(byte)) {
      return false;
    }
    crsf_frame_[0] = byte;
    crsf_frame_pos_ = 1u;
    crsf_have_length_ = false;
    return false;
  }

  if (!crsf_have_length_) {
    if (byte < kCrsfMinFrameLength || byte > kCrsfMaxFrameLength) {
      crsf_frame_pos_ = 0u;
      return false;
    }

    crsf_frame_len_ = byte;
    crsf_frame_[1] = byte;
    crsf_frame_pos_ = 2u;
    crsf_have_length_ = true;
    return false;
  }

  if (crsf_frame_pos_ >= crsf_frame_.size()) {
    crsf_frame_pos_ = 0u;
    crsf_have_length_ = false;
    return false;
  }

  crsf_frame_[crsf_frame_pos_++] = byte;
  if (crsf_frame_pos_ < (std::size_t)(crsf_frame_len_ + 2u)) {
    return false;
  }

  const bool handled = FinishCrsfFrame(now_us);
  crsf_frame_pos_ = 0u;
  crsf_have_length_ = false;
  return handled;
}

void CrsfLinkService::ForwardLatestRawState(uint32_t now_us) {
  if (fc_link_ == nullptr || rc_receiver_ == nullptr) {
    return;
  }

  const RcData &rc = rc_receiver_->GetCurrentData();
  if (rc.timestamp_us == 0) {
    return;
  }

  uint8_t flags = 0;
  if (rc.rx_online) {
    flags |= message::kRcChannelsFlagRxOnline;
  }
  if (rc.tx_online) {
    flags |= message::kRcChannelsFlagTxOnline;
  }

  const bool flags_changed = !have_forwarded_state_ || flags != last_forwarded_flags_;
  if (!pending_forward_ && !flags_changed) {
    return;
  }

  if (!flags_changed && last_forward_us_ != 0 &&
      (uint32_t)(now_us - last_forward_us_) < kFcLinkRcForwardIntervalUs) {
    return;
  }

  message::RcChannelsMsg msg = latest_rx_msg_;
  msg.flags = flags;
  fc_link_->SendRcChannels(msg);
  last_forward_us_ = now_us;
  last_forwarded_flags_ = flags;
  have_forwarded_state_ = true;
  pending_forward_ = false;
}

bool CrsfLinkService::SendBroadcastFrame(uint8_t type, const uint8_t *payload,
                                         uint8_t payload_len) {
  if (uart_ == nullptr) {
    return false;
  }

  uint8_t frame[64];
  const uint8_t frame_len = (uint8_t)(payload_len + 2u);
  const size_t total_len = (size_t)frame_len + 2u;
  if (uart_->TxFree() < total_len) {
    return false;
  }
  frame[0] = kCrsfSerialSyncByte;
  frame[1] = frame_len;
  frame[2] = type;
  for (uint8_t i = 0; i < payload_len; ++i) {
    frame[3u + i] = payload[i];
  }
  frame[total_len - 1u] = CrsfCrc8(frame + 2u, (size_t)payload_len + 1u);
  uart_->Send(frame, total_len);
  return true;
}

bool CrsfLinkService::SendDirectCommand(uint8_t destination, uint8_t command_id,
                                        const uint8_t *payload,
                                        uint8_t payload_len) {
  if (uart_ == nullptr) {
    return false;
  }

  uint8_t frame[64];
  const uint8_t command_payload_len = (uint8_t)(1u + payload_len);
  const uint8_t frame_payload_len = (uint8_t)(3u + command_payload_len);
  const uint8_t frame_len = (uint8_t)(frame_payload_len + 2u);
  const size_t total_len = (size_t)frame_len + 2u;
  if (uart_->TxFree() < total_len) {
    return false;
  }

  frame[0] = kCrsfSerialSyncByte;
  frame[1] = frame_len;
  frame[2] = kCrsfFrameTypeDirectCommand;
  frame[3] = destination;
  frame[4] = kCrsfAddressFlightController;
  frame[5] = command_id;
  for (uint8_t i = 0; i < payload_len; ++i) {
    frame[6u + i] = payload[i];
  }

  frame[6u + payload_len] =
      CrsfCommandCrc8(frame + 2u, (size_t)command_payload_len + 3u);
  frame[total_len - 1u] =
      CrsfCrc8(frame + 2u, (size_t)frame_payload_len + 1u);

  uart_->Send(frame, total_len);
  return true;
}
