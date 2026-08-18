// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <type_traits>

#include "checksum.hpp"

namespace message {

// Alternating bit patterns, so a run of either value cannot look like sync.
inline constexpr uint8_t kMagic1 = 0xAA;
inline constexpr uint8_t kMagic2 = 0x55;

inline constexpr size_t kMaxPayload = 0xFFu;
inline constexpr uint8_t kMaxLogTextPayload = 200u;

enum class MsgId : uint8_t {
  kPing = 0x01,
  kLog = 0x02,
  kPong = 0x03,
  kReqRcMap = 0x04,
  kRcMapConfig = 0x05,
  kReqRcCalibration = 0x06,
  kRcCalibrationConfig = 0x07,
  kReqGyroCalibrationId = 0x08,
  kGyroCalibrationIdConfig = 0x09,
  kSetRcMapConfig = 0x0A,
  kSetRcCalibrationConfig = 0x0B,
  kReqReceiverBind = 0x0C,
  kRcChannels = 0x65,
  kGpsData = 0x10,
  kSystemStatus = 0x12,
  kVehicleStatus = 0x13,
  kPanic = 0x14,
  kEscTelemetry = 0x15,
  kPrivilegedArm = 0x16,  // Override the main arming state machine
  kSetUsbMode = 0x17,
  kUsbStatus = 0x18,
  kTone = 0x19,
  kLogList = 0x1B,
  kLogListReply = 0x1C,
  kLogRead = 0x1D,
  kLogData = 0x1E,
  kReboot = 0xC0,
  kBootload = 0xC1,
  kError = 0xEE
};

#pragma pack(push, 1)

struct Header {
  uint8_t magic[2];
  uint8_t id;
  uint8_t len;
};

// Serialize writes the header by reinterpreting the caller's byte buffer, and
// kPacketOverhead derives from this size. A toolchain that ignored the pragma
// would emit frames the peer cannot parse rather than fail here.
static_assert(sizeof(Header) == 4 && alignof(Header) == 1,
              "wire header must be 4 packed bytes");

// Fixed by CRSF, whose channels frame packs exactly this many at 11 bits
// each. Not a tunable: narrowing it would only move where the unused channels
// are dropped, at the cost of a wire width both firmwares have to agree on.
inline constexpr size_t kRcChannelCount = 16u;

struct RcChannelsMsg {
  uint16_t channels[kRcChannelCount];
  uint8_t link_quality;
  uint8_t flags;
} __attribute__((packed));

inline constexpr uint8_t kRcChannelsFlagRxOnline = 1u << 0;
inline constexpr uint8_t kRcChannelsFlagTxOnline = 1u << 1;

struct RcMapConfigMsg {
  uint8_t roll;
  uint8_t pitch;
  uint8_t yaw;
  uint8_t throttle;
} __attribute__((packed));

struct RcCalibrationConfigMsg {
  uint16_t min_us[16];
  uint16_t max_us[16];
  uint16_t trim_us[16];
  int8_t rev[16];
} __attribute__((packed));

struct GyroCalibrationIdConfigMsg {
  uint32_t cal_gyro0_id;
} __attribute__((packed));

struct GpsData {
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;

  uint8_t fixType;
  uint8_t numSV;

  int32_t lon;   // deg*1e7
  int32_t lat;   // deg*1e7
  int32_t hMSL;  // mm

  uint16_t vel;  // cm/s
  uint16_t hdg;  // cdeg

  uint32_t hAcc;  // mm
  uint32_t vAcc;  // mm

  uint16_t gDOP;  // [0.01]
  uint16_t pDOP;  // [0.01]
  uint16_t hDOP;  // [0.01]  TODO(fc): gate arming on this
  uint16_t vDOP;  // [0.01]

  uint8_t posCovValid;
  uint8_t velCovValid;
  float posCovNN;  // [m²]
  float posCovEE;  // [m²]
  float posCovDD;  // [m²]

  int16_t roll;   // cdeg
  int16_t pitch;  // cdeg
  int16_t yaw;    // cdeg

  uint16_t batt_voltage;  // mV
  int16_t batt_current;   // cA
  int8_t batt_remaining;  // %
} __attribute__((packed));

// Wire vocabularies: the enumerator is the transmitted byte -- append only,
// never renumber. Struct fields stay uint8_t: a peer can send an unknown one.
enum class BootState : uint8_t {
  kBooting = 0,
  kReady,
  kError,
};

inline constexpr uint8_t kSystemStatusFlagLoopAlive = 1u << 0;

inline constexpr uint32_t kSystemSensorFlagImu = 1u << 0;
inline constexpr uint32_t kSystemSensorFlagGps = 1u << 1;
inline constexpr uint32_t kSystemSensorFlagBattery = 1u << 2;
inline constexpr uint32_t kSystemSensorFlagRcReceiver = 1u << 3;
inline constexpr uint32_t kSystemSensorFlagEsc = 1u << 4;

struct SystemStatusMsg {
  uint32_t uptime_ms;
  uint32_t loop_counter;
  uint32_t error_code;
  uint32_t sensor_present_flags;
  uint32_t sensor_health_flags;
  uint16_t batt_voltage;
  int16_t batt_current;
  int8_t batt_remaining;
  uint8_t boot_state;
  uint8_t flags;
} __attribute__((packed));

enum class ArmedState : uint8_t {
  kDisarmed = 0,
  kArmed,
};

inline constexpr uint32_t kVehicleFailsafeFlagRcLoss = 1u << 0;
inline constexpr uint32_t kVehicleFailsafeFlagBattery = 1u << 1;
inline constexpr uint32_t kVehicleFailsafeFlagImu = 1u << 2;
inline constexpr uint32_t kVehicleFailsafeFlagGps = 1u << 3;

struct VehicleStatusMsg {
  uint8_t armed_state;
  uint32_t failsafe_flags;
  uint8_t flight_mode;
  uint8_t reserved[2];
} __attribute__((packed));

struct PanicMsg {
  uint32_t error_code;
} __attribute__((packed));

struct PrivilegedArmMsg {
  uint8_t armed;  // 0 = disarm, non-zero = arm
} __attribute__((packed));

// One USB port, one personality: a mode rather than a flag per dialect, so
// two grants cannot disagree about what the port currently is.
enum class UsbMode : uint8_t {
  kNone = 0,
  kEscConfig,
  kMsc,
};

struct SetUsbModeMsg {
  uint8_t mode;
} __attribute__((packed));

// Log retrieval: host-paced, and the STM32 serves it only while disarmed in
// Idle. Frames are fixed-size with an inner count, so length checks stay exact.

inline constexpr uint8_t kLogNameLen = 12u;  // 8.3, no terminator
inline constexpr uint8_t kLogListMaxEntries = 14u;
inline constexpr uint16_t kLogDataMaxBytes = 232u;

enum class LogStatus : uint8_t {
  kOk = 0,
  kBusy,
  kNotFound,
  kIoError,
};

struct LogListMsg {
  uint8_t first;
} __attribute__((packed));

struct LogListEntry {
  char name[kLogNameLen];
  uint32_t size_bytes;
} __attribute__((packed));

struct LogListReplyMsg {
  uint8_t status;
  uint8_t total;  // files on the card, so the reader knows to page
  uint8_t first;
  uint8_t count;  // valid entries below
  LogListEntry entries[kLogListMaxEntries];
} __attribute__((packed));

struct LogReadMsg {
  char name[kLogNameLen];
  uint32_t offset;
  uint16_t len;  // capped to kLogDataMaxBytes by the server
} __attribute__((packed));

struct LogDataMsg {
  uint32_t offset;
  uint16_t len;  // valid bytes below; 0 with status Ok means EOF
  uint8_t status;
  uint8_t data[kLogDataMaxBytes];
} __attribute__((packed));
static_assert(sizeof(LogDataMsg) <= kMaxPayload);
static_assert(sizeof(LogListReplyMsg) <= kMaxPayload);

inline constexpr uint8_t kUsbStatusAttached = 1u << 0;    // D+ pull-up driven
inline constexpr uint8_t kUsbStatusConfigured = 1u << 1;  // host enumerated us
inline constexpr uint8_t kUsbStatusPortOpen = 1u << 2;    // DTR asserted

// ToneMsg carries this value directly, so the enumerators are the wire
// format: append only, never renumber. kCount bounds the ESP32's score table
// and is never transmitted.
enum class Tone : uint8_t {
  kBeep = 0,
  kConfirm,
  kWarning,
  kError,
  kAm32Startup,
  kAm32Brushed,
  kAm32Dusking,
  kAm32Input,
  kAm32Input2,
  kAm32Default,
  kAm32Changed,
  kDoom,
  kDoomShort,
  kCount,
};

struct ToneMsg {
  uint8_t tone;
} __attribute__((packed));

struct UsbStatusMsg {
  uint8_t flags;
  uint8_t mode;       // UsbMode, the same one SetUsbModeMsg commands
  uint8_t rx_frames;  // configurator -> flight controller
  uint8_t tx_frames;  // flight controller -> configurator
} __attribute__((packed));

inline constexpr uint8_t kEscTelemetryMotorCount = 4u;

struct EscTelemetryMsg {
  uint64_t timestamp_us;
  uint32_t frame_count;
  uint32_t crc_error_count;
  uint32_t unassigned_frame_count;
  uint32_t rx_drop_bytes;
  uint32_t rx_dma_error_count;
  uint32_t uart_error_count;
  uint32_t rpm[kEscTelemetryMotorCount];
  uint32_t electrical_rpm[kEscTelemetryMotorCount];
  uint16_t voltage_centivolts[kEscTelemetryMotorCount];
  uint16_t current_centiamps[kEscTelemetryMotorCount];
  uint16_t consumption_mah[kEscTelemetryMotorCount];
  int16_t temperature_c[kEscTelemetryMotorCount];
  uint8_t valid_mask;
} __attribute__((packed));

struct Packet {
  Header header;
  uint8_t payload[kMaxPayload];
  uint16_t crc;  // CRC16-CCITT (XMODEM)
};

#pragma pack(pop)

inline constexpr size_t kPacketOverhead = sizeof(Header) + 2;

template <typename T>
inline constexpr uint8_t PayloadLength() {
  static_assert(sizeof(T) <= kMaxPayload, "Payload exceeds wire payload limit");
  return static_cast<uint8_t>(sizeof(T));
}

template <typename T>
using PacketBuffer = std::array<uint8_t, sizeof(T) + kPacketOverhead>;

inline constexpr bool IsKnownMsgId(MsgId id) {
  switch (id) {
    case MsgId::kPing:
    case MsgId::kLog:
    case MsgId::kPong:
    case MsgId::kReqRcMap:
    case MsgId::kRcMapConfig:
    case MsgId::kReqRcCalibration:
    case MsgId::kRcCalibrationConfig:
    case MsgId::kReqGyroCalibrationId:
    case MsgId::kGyroCalibrationIdConfig:
    case MsgId::kSetRcMapConfig:
    case MsgId::kSetRcCalibrationConfig:
    case MsgId::kReqReceiverBind:
    case MsgId::kRcChannels:
    case MsgId::kGpsData:
    case MsgId::kSystemStatus:
    case MsgId::kVehicleStatus:
    case MsgId::kPanic:
    case MsgId::kEscTelemetry:
    case MsgId::kPrivilegedArm:
    case MsgId::kSetUsbMode:
    case MsgId::kUsbStatus:
    case MsgId::kTone:
    case MsgId::kLogList:
    case MsgId::kLogListReply:
    case MsgId::kLogRead:
    case MsgId::kLogData:
    case MsgId::kReboot:
    case MsgId::kBootload:
    case MsgId::kError:
      return true;
    default:
      return false;
  }
}

inline constexpr bool IsPayloadLengthValid(MsgId id, uint8_t len) {
  switch (id) {
    case MsgId::kPing:
    case MsgId::kPong:
    case MsgId::kReqRcMap:
    case MsgId::kReqRcCalibration:
    case MsgId::kReqGyroCalibrationId:
    case MsgId::kReqReceiverBind:
    case MsgId::kReboot:
    case MsgId::kBootload:
    case MsgId::kError:
      return len == 0;
    case MsgId::kRcMapConfig:
    case MsgId::kSetRcMapConfig:
      return len == PayloadLength<RcMapConfigMsg>();
    case MsgId::kRcCalibrationConfig:
    case MsgId::kSetRcCalibrationConfig:
      return len == PayloadLength<RcCalibrationConfigMsg>();
    case MsgId::kGyroCalibrationIdConfig:
      return len == PayloadLength<GyroCalibrationIdConfigMsg>();
    case MsgId::kRcChannels:
      return len == PayloadLength<RcChannelsMsg>();
    case MsgId::kGpsData:
      return len == PayloadLength<GpsData>();
    case MsgId::kSystemStatus:
      return len == PayloadLength<SystemStatusMsg>();
    case MsgId::kVehicleStatus:
      return len == PayloadLength<VehicleStatusMsg>();
    case MsgId::kPanic:
      return len == PayloadLength<PanicMsg>();
    case MsgId::kEscTelemetry:
      return len == PayloadLength<EscTelemetryMsg>();
    case MsgId::kPrivilegedArm:
      return len == PayloadLength<PrivilegedArmMsg>();
    case MsgId::kSetUsbMode:
      return len == PayloadLength<SetUsbModeMsg>();
    case MsgId::kLogList:
      return len == PayloadLength<LogListMsg>();
    case MsgId::kLogListReply:
      return len == PayloadLength<LogListReplyMsg>();
    case MsgId::kLogRead:
      return len == PayloadLength<LogReadMsg>();
    case MsgId::kLogData:
      return len == PayloadLength<LogDataMsg>();
    case MsgId::kUsbStatus:
      return len == PayloadLength<UsbStatusMsg>();
    case MsgId::kTone:
      return len == PayloadLength<ToneMsg>();
    case MsgId::kLog:
      return len <= kMaxLogTextPayload;
    default:
      return false;
  }
}

inline bool IsPayloadValid(MsgId id, const uint8_t *payload, uint8_t len) {
  if (!IsPayloadLengthValid(id, len)) {
    return false;
  }

  if (len > 0 && payload == nullptr) {
    return false;
  }
  return true;
}

inline bool IsPacketValid(uint8_t raw_id, const uint8_t *payload, uint8_t len) {
  return IsPayloadValid(static_cast<MsgId>(raw_id), payload, len);
}

inline constexpr bool IsRcMapChannelValid(uint8_t channel) {
  return channel >= 1u && channel <= 4u;
}

inline constexpr bool IsRcMapConfigValid(const RcMapConfigMsg &cfg) {
  return IsRcMapChannelValid(cfg.roll) && IsRcMapChannelValid(cfg.pitch) &&
         IsRcMapChannelValid(cfg.yaw) && IsRcMapChannelValid(cfg.throttle) &&
         cfg.roll != cfg.pitch && cfg.roll != cfg.yaw &&
         cfg.roll != cfg.throttle && cfg.pitch != cfg.yaw &&
         cfg.pitch != cfg.throttle && cfg.yaw != cfg.throttle;
}

inline constexpr size_t kRcCalibrationChannelCount = 16u;

inline constexpr bool IsRcCalibrationRevValid(int8_t rev) {
  return rev == 1 || rev == -1;
}

inline constexpr bool IsRcCalibrationRangeValid(uint16_t min_us,
                                                uint16_t max_us,
                                                uint16_t trim_us) {
  return min_us < max_us && trim_us >= min_us && trim_us <= max_us;
}

inline bool IsRcCalibrationConfigValid(const RcCalibrationConfigMsg &cfg) {
  for (size_t i = 0; i < kRcCalibrationChannelCount; ++i) {
    if (!IsRcCalibrationRangeValid(cfg.min_us[i], cfg.max_us[i],
                                   cfg.trim_us[i]) ||
        !IsRcCalibrationRevValid(cfg.rev[i])) {
      return false;
    }
  }
  return true;
}

inline bool IsGyroCalibrationIdConfigValid(
    const GyroCalibrationIdConfigMsg &cfg) {
  return cfg.cal_gyro0_id != 0u;
}

// Takes the length from the payload type, so the declared length cannot
// disagree with the bytes actually copied.
template <typename T>
inline Packet MakePacket(MsgId id, const T &body) {
  Packet pkt{};
  pkt.header.id = static_cast<uint8_t>(id);
  pkt.header.len = PayloadLength<T>();
  std::memcpy(pkt.payload, &body, sizeof(T));
  return pkt;
}

// Callers must validate id and length first (both Dispatch paths do, via
// IsPacketValid); this is a view over the wire bytes, not a decoder.
template <typename T>
[[nodiscard]] const T &PayloadAs(const Packet &pkt) {
  static_assert(std::is_trivially_copyable_v<T>,
                "packet payload type must be trivially copyable");
  static_assert(alignof(T) == 1,
                "packet payload type must be packed for zero-copy decode");
  static_assert(sizeof(T) <= kMaxPayload,
                "packet payload type exceeds wire payload limit");
  return *reinterpret_cast<const T *>(pkt.payload);
}

[[nodiscard]] inline size_t Serialize(MsgId id,
                                      std::span<const uint8_t> payload,
                                      std::span<uint8_t> out) {
  if (payload.size() > kMaxPayload) {
    return 0;
  }
  const auto len = static_cast<uint8_t>(payload.size());
  if (out.size() < len + kPacketOverhead) {
    return 0;
  }
  if (!IsPayloadValid(id, payload.data(), len)) {
    return 0;
  }

  Header *h = reinterpret_cast<Header *>(out.data());
  h->magic[0] = kMagic1;
  h->magic[1] = kMagic2;
  h->id = static_cast<uint8_t>(id);
  h->len = len;

  if (len > 0) {
    std::memcpy(out.data() + sizeof(Header), payload.data(), len);
  }

  uint16_t crc = checksum::XModem(out.first(sizeof(Header) + len));

  // Little endian.
  out[sizeof(Header) + len] = static_cast<uint8_t>(crc & 0xFF);
  out[sizeof(Header) + len + 1] = static_cast<uint8_t>((crc >> 8) & 0xFF);

  return sizeof(Header) + len + 2;
}

}  // namespace message
