// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "crsf_link_service.hpp"

#include <cmath>
#include <cstring>
#include <span>

#include "checksum.hpp"
#include "error_code.hpp"
#include "panic.hpp"
#include "rc_receiver.hpp"
#include "shared_state.hpp"
#include "uart.hpp"

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

constexpr uint8_t kGpsPayloadSize = 15u;
constexpr uint8_t kHeartbeatPayloadSize = 2u;
constexpr uint8_t kBatteryPayloadSize = 8u;

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
  return (((uint32_t)speed_cm_s * 36u) + 5u) / 10u;
}

uint32_t EncodeBatteryVoltageTenths(float voltage_v) {
  if (voltage_v <= 0.0f) {
    return 0;
  }
  return static_cast<uint32_t>(std::lround(voltage_v * 10.0f));
}

uint32_t EncodeBatteryCurrentTenths(float current_a) {
  if (current_a <= 0.0f) {
    return 0;
  }
  return static_cast<uint32_t>(std::lround(current_a * 10.0f));
}

uint32_t EncodeMah(float mah_drawn) {
  if (mah_drawn <= 0.0f) {
    return 0;
  }
  return static_cast<uint32_t>(std::lround(mah_drawn));
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
  // De-facto EdgeTX CRSF battery encoding for cross-stack compatibility.
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
                           SharedState &blackboard, RcReceiver &rc_receiver) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kCrsfLinkInitFailed);
  }

  cfg_ = cfg;
  uart_ = &uart;
  blackboard_ = &blackboard;
  rc_receiver_ = &rc_receiver;
  crsf_have_length_ = false;
  crsf_frame_len_ = 0;
  crsf_frame_pos_ = 0;
  crsf_frame_.fill(0);
  payload_memos_.fill(PayloadMemo{});
  pending_command_ = PendingCommand::kNone;
  initialized_ = true;
}

void CrsfLinkService::PollRx(uint32_t now_us, size_t byte_budget) {
  if (!initialized_ || uart_ == nullptr) {
    return;
  }

  uint8_t byte = 0;
  size_t count = 0;
  while (count < byte_budget && uart_->ReadByte(byte)) {
    ++count;
    (void)ProcessCrsfByte(byte, now_us);
  }

}

void CrsfLinkService::PollCommands() {
  if (!initialized_ || uart_ == nullptr) {
    return;
  }

  // At most one command is ever pending, so this needs no frame budget.
  (void)TrySendPendingCommand();
}

void CrsfLinkService::RequestReceiverBind() {
  pending_command_ = PendingCommand::kReceiverBind;
}

void CrsfLinkService::RequestReceiverCancelBind() {
  pending_command_ = PendingCommand::kReceiverCancelBind;
}

CrsfLinkService::TelemetryResult CrsfLinkService::SendTelemetry(
    TelemetryTopic topic, bool silence_expired, uint32_t now_us) {
  if (!initialized_ || uart_ == nullptr || blackboard_ == nullptr) {
    return TelemetryResult::kBlocked;
  }

  const std::optional<TelemetryFrame> frame =
      PrepareTelemetryTopic(topic, now_us);
  if (!frame) {
    return TelemetryResult::kSkipped;
  }

  const size_t index = static_cast<size_t>(topic);
  if (cfg_.send_on_change[index] && !PayloadChanged(topic, *frame) &&
      !silence_expired) {
    return TelemetryResult::kSkipped;
  }

  if (!SendBroadcastFrame(frame->type, frame->payload.data(), frame->len)) {
    return TelemetryResult::kBlocked;
  }

  PayloadMemo &memo = payload_memos_[index];
  memo.len = frame->len;
  memo.valid = true;
  memo.bytes = frame->payload;
  return TelemetryResult::kSent;
}

std::optional<CrsfLinkService::TelemetryFrame>
CrsfLinkService::PrepareTelemetryTopic(TelemetryTopic topic,
                                       uint32_t now_us) const {
  // The encoders below write through a raw pointer, so the payload capacity is
  // only enforced here.
  static_assert(kHeartbeatPayloadSize <= kMaxTelemetryPayload);
  static_assert(kGpsPayloadSize <= kMaxTelemetryPayload);
  static_assert(kBatteryPayloadSize <= kMaxTelemetryPayload);

  if (blackboard_ == nullptr) {
    return std::nullopt;
  }

  TelemetryFrame frame{};
  switch (topic) {
    case TelemetryTopic::kHeartbeat:
      frame.type = kCrsfFrameTypeHeartbeat;
      frame.len = kHeartbeatPayloadSize;
      EncodeHeartbeatPayload(frame.payload.data());
      return frame;
    case TelemetryTopic::kGps: {
      const GpsData &gps = blackboard_->GetGps();
      if (gps.timestamp_us == 0 ||
          (now_us - gps.timestamp_us) > cfg_.gps_fresh_timeout_us) {
        return std::nullopt;
      }
      frame.type = kCrsfFrameTypeGps;
      frame.len = kGpsPayloadSize;
      EncodeGpsPayload(gps, frame.payload.data());
      return frame;
    }
    case TelemetryTopic::kBattery: {
      const BatteryData &battery = blackboard_->GetBattery();
      frame.type = kCrsfFrameTypeBattery;
      frame.len = kBatteryPayloadSize;
      EncodeBatteryPayload(battery, frame.payload.data());
      return frame;
    }
    case TelemetryTopic::kCount:
    default:
      return std::nullopt;
  }
}

bool CrsfLinkService::PayloadChanged(TelemetryTopic topic,
                                     const TelemetryFrame &frame) const {
  const PayloadMemo &memo = payload_memos_[static_cast<size_t>(topic)];
  if (!memo.valid) {
    return true;
  }
  return frame.len != memo.len ||
         memcmp(memo.bytes.data(), frame.payload.data(), frame.len) != 0;
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
  const BatteryData &battery = blackboard_->GetBattery();
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

  if (blackboard_ == nullptr) {
    return false;
  }

  // Field order is the CRSF LINK_STATISTICS layout; SNR values are signed dB
  // and the RSSI figures are positive magnitudes of a negative dBm.
  blackboard_->UpdateCrsfLink(CrsfLinkData{
      .timestamp_us = now_us,
      .uplink_rssi_ant1_dbm = payload[0],
      .uplink_rssi_ant2_dbm = payload[1],
      .uplink_link_quality = payload[2],
      .uplink_snr_db = static_cast<int8_t>(payload[3]),
      .active_antenna = payload[4],
      .rf_mode = payload[5],
      .uplink_tx_power_index = payload[6],
      .downlink_rssi_dbm = payload[7],
      .downlink_link_quality = payload[8],
      .downlink_snr_db = static_cast<int8_t>(payload[9]),
  });
  return true;
}

bool CrsfLinkService::ParseRcChannelsFrame(const uint8_t *payload,
                                           std::size_t len, uint32_t now_us) {
  if (payload == nullptr || len < kCrsfRcChannelsPayloadSize ||
      rc_receiver_ == nullptr) {
    return false;
  }

  uint32_t bit_buffer = 0;
  uint8_t bits_available = 0;
  std::size_t byte_index = 0;
  message::RcChannelsMsg msg{};

  for (std::size_t channel = 0; channel < message::kRcChannelCount; ++channel) {
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

  // Link quality rides its own frame and its own blackboard entry now, so the
  // channels carry nothing about the link; StatPublisher joins them on the way
  // out with each side's freshness judged separately.
  msg.link_quality = 0;
  msg.flags = 0;
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
      checksum::Dvbs2(std::span{crsf_frame_}.subspan(2u, payload_len + 1u));
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
  frame[total_len - 1u] =
      checksum::Dvbs2(std::span{frame}.subspan(2u, (size_t)payload_len + 1u));
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
  frame[total_len - 1u] = checksum::Dvbs2(
      std::span{frame}.subspan(2u, (size_t)frame_payload_len + 1u));

  uart_->Send(frame, total_len);
  return true;
}
