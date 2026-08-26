// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "crsf_link_service.hpp"

#include <cmath>
#include <cstring>
#include <optional>
#include <span>

#include "checksum.hpp"
#include "common_config.hpp"
#include "error_code.hpp"
#include "math/attitude_euler.hpp"
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
constexpr uint8_t kCrsfFrameTypeRpm = 0x0Cu;
constexpr uint8_t kCrsfFrameTypeTemperature = 0x0Du;
constexpr uint8_t kCrsfFrameTypeAttitude = 0x1Eu;
constexpr uint8_t kCrsfFrameTypeFlightMode = 0x21u;
constexpr uint8_t kCrsfFrameTypeGpsTime = 0x03u;
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
constexpr uint8_t kAttitudePayloadSize = 6u;
// A source byte, then one entry per motor: 24-bit RPM, 16-bit deci-Celsius.
constexpr uint8_t kRpmPayloadSize =
    1u + (3u * common_config::kAirframeMotorCount);
constexpr uint8_t kTemperaturePayloadSize =
    1u + (2u * common_config::kAirframeMotorCount);
// Longest name, the disarmed marker, and the terminator.
constexpr uint8_t kFlightModePayloadSize = 6u;
constexpr uint8_t kGpsTimePayloadSize = 9u;
// Source 0 is the airframe as a whole, which is what a per-motor list is --
// the alternative numbering identifies one physical sensor per frame.
constexpr uint8_t kCrsfSensorSourceAirframe = 0u;

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

// CRSF has no encoding for an unsensed pack, so both of these report zero.
uint32_t EncodeBatteryCurrentTenths(std::optional<float> current_a) {
  if (!current_a.has_value() || *current_a <= 0.0f) {
    return 0;
  }
  return static_cast<uint32_t>(std::lround(*current_a * 10.0f));
}

uint32_t EncodeMah(std::optional<float> mah_drawn) {
  if (!mah_drawn.has_value() || *mah_drawn <= 0.0f) {
    return 0;
  }
  return static_cast<uint32_t>(std::lround(*mah_drawn));
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

int16_t ClampI16(float value) {
  if (value > 32767.0f) return 32767;
  if (value < -32768.0f) return -32768;
  return static_cast<int16_t>(value);
}

void EncodeAttitudePayload(const EstimatorState &estimate,
                           uint8_t payload[kAttitudePayloadSize]) {
  constexpr float kRadToUnits = 10000.0f;
  const math::EulerZyx euler =
      math::EulerZyxFromQuaternion(estimate.attitude_world_to_body);
  // Pitch first, then roll: CRSF's order, not the roll-pitch-yaw the rest of
  // this codebase writes.
  StoreBe16(payload + 0u,
            static_cast<uint16_t>(ClampI16(euler.pitch * kRadToUnits)));
  StoreBe16(payload + 2u,
            static_cast<uint16_t>(ClampI16(euler.roll * kRadToUnits)));
  StoreBe16(payload + 4u,
            static_cast<uint16_t>(ClampI16(euler.yaw * kRadToUnits)));
}

// A motor the ESCs have not answered for reports zero rather than being left
// out: the list is positional, so a short one renumbers every motor after it.
void EncodeRpmPayload(const EscTelemetryData &esc,
                      uint8_t payload[kRpmPayloadSize]) {
  payload[0] = kCrsfSensorSourceAirframe;
  for (uint8_t i = 0; i < common_config::kAirframeMotorCount; ++i) {
    const bool valid = (esc.valid_mask & (1u << i)) != 0u;
    const uint32_t rpm = valid ? esc.motors[i].rpm : 0u;
    StoreBe24(payload + 1u + (3u * i), rpm & 0x00FFFFFFu);
  }
}

void EncodeTemperaturePayload(const EscTelemetryData &esc,
                              uint8_t payload[kTemperaturePayloadSize]) {
  payload[0] = kCrsfSensorSourceAirframe;
  for (uint8_t i = 0; i < common_config::kAirframeMotorCount; ++i) {
    const bool valid = (esc.valid_mask & (1u << i)) != 0u;
    // Deci-Celsius on the wire; AM32 reports whole degrees.
    const int16_t deci_c =
        valid ? static_cast<int16_t>(esc.motors[i].temperature_c * 10) : 0;
    StoreBe16(payload + 1u + (2u * i), static_cast<uint16_t>(deci_c));
  }
}

// EdgeTX renders this as the FM field, so it is the one status the pilot reads
// without looking away from the aircraft. Betaflight's vocabulary, because
// that is what the handset's users already know how to read.
uint8_t EncodeFlightModePayload(FlightMode mode, bool armed,
                                uint8_t payload[kFlightModePayloadSize]) {
  const char *name = (mode == FlightMode::kStabilize) ? "STAB" : "ACRO";
  uint8_t len = 0;
  while (name[len] != '\0') {
    payload[len] = static_cast<uint8_t>(name[len]);
    len++;
  }
  // Betaflight's disarmed markers are '*' ready, '!' arming blocked. Nothing
  // here can say blocked yet, so the honest answer is the one it can prove.
  if (!armed) {
    payload[len++] = static_cast<uint8_t>('*');
  }
  payload[len++] = 0u;
  return len;
}

// EdgeTX turns this into the handset's clock, not just a display field:
// crossfire.cpp splits the frame into a date and a time record, and
// telemetry_sensors.cpp calls rtcAdjust on the second when Adjust RTC is on.
// It decodes the year as `year - 2000` into a uint8, so a full four-digit year
// is required and an unresolved date must never reach it.
void EncodeGpsTimePayload(const GpsData &gps,
                          uint8_t payload[kGpsTimePayloadSize]) {
  StoreBe16(payload + 0u, gps.year);
  payload[2] = gps.month;
  payload[3] = gps.day;
  payload[4] = gps.hour;
  payload[5] = gps.min;
  payload[6] = gps.sec;
  // GpsData keeps whole seconds; the handset's decoder reads no further.
  StoreBe16(payload + 7u, 0u);
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

  size_t count = 0;
  while (count < byte_budget) {
    const std::optional<uint8_t> byte = uart_->ReadByte();
    if (!byte) {
      break;
    }
    ++count;
    (void)ProcessCrsfByte(byte.value(), now_us);
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
  static_assert(kAttitudePayloadSize <= kMaxTelemetryPayload);
  static_assert(kRpmPayloadSize <= kMaxTelemetryPayload);
  static_assert(kTemperaturePayloadSize <= kMaxTelemetryPayload);
  static_assert(kFlightModePayloadSize <= kMaxTelemetryPayload);
  static_assert(kGpsTimePayloadSize <= kMaxTelemetryPayload);

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
      // CRSF has no health bit to qualify a reading with, and a handset
      // holding a frozen voltage is one whose low-pack alarm cannot fire.
      if (battery.timestamp_us == 0 ||
          (now_us - battery.timestamp_us) > cfg_.battery_fresh_timeout_us) {
        return std::nullopt;
      }
      frame.type = kCrsfFrameTypeBattery;
      frame.len = kBatteryPayloadSize;
      EncodeBatteryPayload(battery, frame.payload.data());
      return frame;
    }
    case TelemetryTopic::kFlightMode: {
      frame.type = kCrsfFrameTypeFlightMode;
      frame.len = EncodeFlightModePayload(blackboard_->GetFlightMode(),
                                          blackboard_->IsArmed(),
                                          frame.payload.data());
      return frame;
    }
    case TelemetryTopic::kAttitude: {
      const EstimatorState &estimate = blackboard_->GetEstimate();
      if (estimate.timestamp_us == 0) {
        return std::nullopt;
      }
      frame.type = kCrsfFrameTypeAttitude;
      frame.len = kAttitudePayloadSize;
      EncodeAttitudePayload(estimate, frame.payload.data());
      return frame;
    }
    case TelemetryTopic::kRpm: {
      const EscTelemetryData &esc = blackboard_->GetEscTelemetry();
      // valid_mask, not the stamp: the mask is what says a motor answered,
      // and it clears per motor where the stamp is one number for all four.
      if (esc.valid_mask == 0u) {
        return std::nullopt;
      }
      frame.type = kCrsfFrameTypeRpm;
      frame.len = kRpmPayloadSize;
      EncodeRpmPayload(esc, frame.payload.data());
      return frame;
    }
    case TelemetryTopic::kTemperature: {
      const EscTelemetryData &esc = blackboard_->GetEscTelemetry();
      if (esc.valid_mask == 0u) {
        return std::nullopt;
      }
      frame.type = kCrsfFrameTypeTemperature;
      frame.len = kTemperaturePayloadSize;
      EncodeTemperaturePayload(esc, frame.payload.data());
      return frame;
    }
    case TelemetryTopic::kGpsTime: {
      const GpsData &gps = blackboard_->GetGps();
      // Setting a clock from a stale fix is worse than not setting it, and a
      // year of zero underflows the handset's `year - 2000`.
      if (gps.year == 0 || gps.timestamp_us == 0 ||
          (now_us - gps.timestamp_us) > cfg_.gps_fresh_timeout_us) {
        return std::nullopt;
      }
      frame.type = kCrsfFrameTypeGpsTime;
      frame.len = kGpsTimePayloadSize;
      EncodeGpsTimePayload(gps, frame.payload.data());
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

bool CrsfLinkService::TrySendPendingCommand() {
  // Fire-and-forget: the receive path parses RC channels and link statistics
  // only, so nothing here can observe whether the receiver acted on it.
  uint8_t payload = 0;
  switch (pending_command_) {
    case PendingCommand::kReceiverBind:
      payload = kCrsfCrossfireSubcmdBind;
      break;
    case PendingCommand::kReceiverCancelBind:
      payload = kCrsfCrossfireSubcmdCancelBind;
      break;
    case PendingCommand::kNone:
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
      .checksum_failures = checksum_failures_,
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
  // channels carry nothing about the link; TelemetryPublisher joins them on the
  // way out with each side's freshness judged separately.
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
    // A frame that arrived whole and failed its own checksum: the UART saw
    // nothing wrong, so this is the only place it can be counted.
    checksum_failures_ = checksum_failures_ + 1u;
    // Onto the published record rather than waiting for the next
    // LINK_STATISTICS frame, which a receiver failing every CRC never sends.
    if (blackboard_ != nullptr) {
      CrsfLinkData next = blackboard_->GetCrsfLink();
      next.checksum_failures = checksum_failures_;
      blackboard_->UpdateCrsfLink(next);
    }
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
  // TxFree was checked against total_len above, so this cannot be refused --
  // propagated rather than discarded so that stays true if the guard moves.
  return uart_->Send(frame, total_len) == Outcome::kOk;
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

  // TxFree was checked against total_len above, so this cannot be refused --
  // propagated rather than discarded so that stays true if the guard moves.
  return uart_->Send(frame, total_len) == Outcome::kOk;
}
