#pragma once

#include "error_code.hpp"
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

namespace message {

// ===================================
//   Inter-Processor Protocol
// ===================================

// Magic Bytes for Sync (Alt bits for robustness)
static constexpr uint8_t kMagic1 = 0xAA;
static constexpr uint8_t kMagic2 = 0x55;

// Maximum payload size
static constexpr size_t kMaxPayload = 0xFFu;
static constexpr uint8_t kMaxLogTextPayload = 200u;
static constexpr uint8_t kMaxLogBinaryArgs = 16u;
static constexpr uint8_t kLogBinaryFmtIdThreshold = 32u;

// Message Identifiers
enum class MsgId : uint8_t {
  kPing = 0x01,
  kLog = 0x02,
  kPong = 0x03,
  kRcChannels = 0x65,
  kGpsData = 0x10,
  kImuData = 0x11,
  kPanic = 0x14,

  // System
  kReboot = 0xC0,   // 192
  kBootload = 0xC1, // 193
  kError = 0xEE     // 238
};

#pragma pack(push, 1)

struct Header {
  uint8_t magic[2]; // {0xAA, 0x55}
  uint8_t id;       // MsgId
  uint8_t len;      // Payload Length
};

struct RcChannelsMsg {
  uint16_t channels[16];
  uint8_t rssi;
} __attribute__((packed));

struct GpsData {
  // Time
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;

  // Status
  uint8_t fixType;
  uint8_t numSV;

  // Position
  int32_t lon;  // deg*1e7
  int32_t lat;  // deg*1e7
  int32_t hMSL; // mm

  // Motion
  uint16_t vel; // cm/s
  uint16_t hdg; // cdeg

  // Accuracy
  uint32_t hAcc; // mm
  uint32_t vAcc; // mm

  // Quality Metrics (DOP)
  uint16_t gDOP; // Geometric DOP [0.01]
  uint16_t pDOP; // Position DOP [0.01]
  uint16_t hDOP; // Horizontal DOP [0.01] //TO DO: Use for arming safety
  uint16_t vDOP; // Vertical DOP [0.01]

  // Covariance (for Kalman filtering) - simplified
  uint8_t posCovValid; // Position covariance valid flag
  uint8_t velCovValid; // Velocity covariance valid flag
  float posCovNN;      // Position covariance North-North [m²]
  float posCovEE;      // Position covariance East-East [m²]
  float posCovDD;      // Position covariance Down-Down [m²]

  // Attitude (New)
  int16_t roll;  // cdeg
  int16_t pitch; // cdeg
  int16_t yaw;   // cdeg

  // Battery (New)
  uint16_t batt_voltage; // mV
  int16_t batt_current;  // cA
  int8_t batt_remaining; // %
} __attribute__((packed));

struct ImuData {
  uint64_t timestamp_us; // Local monotonic microseconds
  float accel[3];        // m/s²  (X, Y, Z)
  float gyro[3];         // rad/s (X, Y, Z)
} __attribute__((packed));

// Binary log: format string + raw args (ESP32 does vsnprintf)
struct LogBinary {
  uint8_t fmt_id;       // Format string ID
  uint8_t argc;         // Number of uint32_t arguments
  uint32_t args[16];    // Raw argument values
} __attribute__((packed));

struct PanicMsg {
  ErrorCode error_code; // Error code
} __attribute__((packed));

struct Packet {
  Header header;
  uint8_t payload[kMaxPayload];
  uint16_t crc; // CRC16-CCITT (XMODEM)
};

#pragma pack(pop)

// Overhead: Header(4) + CRC(2)
static constexpr size_t kPacketOverhead = sizeof(Header) + 2;

template <typename T> static constexpr uint8_t PayloadLength() {
  static_assert(sizeof(T) <= kMaxPayload, "Payload exceeds wire payload limit");
  return static_cast<uint8_t>(sizeof(T));
}

static constexpr bool IsKnownMsgId(MsgId id) {
  switch (id) {
  case MsgId::kPing:
  case MsgId::kLog:
  case MsgId::kPong:
  case MsgId::kRcChannels:
  case MsgId::kGpsData:
  case MsgId::kImuData:
  case MsgId::kPanic:
  case MsgId::kReboot:
  case MsgId::kBootload:
  case MsgId::kError:
    return true;
  default:
    return false;
  }
}

static constexpr bool IsPayloadLengthValid(MsgId id, uint8_t len) {
  switch (id) {
  case MsgId::kPing:
  case MsgId::kPong:
  case MsgId::kReboot:
  case MsgId::kBootload:
  case MsgId::kError:
    return len == 0;
  case MsgId::kRcChannels:
    return len == PayloadLength<RcChannelsMsg>();
  case MsgId::kGpsData:
    return len == PayloadLength<GpsData>();
  case MsgId::kImuData:
    return len == PayloadLength<ImuData>();
  case MsgId::kPanic:
    return len == PayloadLength<PanicMsg>();
  case MsgId::kLog:
    return len <= kMaxLogTextPayload;
  default:
    return false;
  }
}

static inline bool IsPayloadValid(MsgId id, const uint8_t *payload,
                                  uint8_t len) {
  if (!IsPayloadLengthValid(id, len)) {
    return false;
  }

  if (len > 0 && payload == nullptr) {
    return false;
  }

  if (id != MsgId::kLog || len == 0) {
    return true;
  }

  if (payload[0] >= kLogBinaryFmtIdThreshold) {
    return true;
  }

  if (len < 2u) {
    return false;
  }

  const uint8_t argc = payload[1];
  return argc <= kMaxLogBinaryArgs &&
         len == static_cast<uint8_t>(2u + (argc * sizeof(uint32_t)));
}

static inline bool IsPacketValid(uint8_t raw_id, const uint8_t *payload,
                                 uint8_t len) {
  return IsPayloadValid(static_cast<MsgId>(raw_id), payload, len);
}

// ---------------------------------------------------------
// Helper: Simple CRC16-CCITT (XMODEM)
// Poly: 0x1021
// ---------------------------------------------------------
static inline uint16_t Crc16(const uint8_t *data, size_t len) {
  uint16_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc ^= (uint16_t)data[i] << 8;
    for (int j = 0; j < 8; ++j) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// ---------------------------------------------------------
// Helper: Buffer Inference
// ---------------------------------------------------------
template <typename T>
static constexpr std::array<uint8_t, sizeof(T) + kPacketOverhead>
MakePacketBuffer(const T &) {
  return {};
}

// ---------------------------------------------------------
// Helper: Serialize
// ---------------------------------------------------------
// Fills the buffer 'out' with the formatted packet.
// 'out' must be at least 'len' + kPacketOverhead bytes (Header + CRC).
static inline size_t Serialize(MsgId id, const uint8_t *payload, uint8_t len,
                               uint8_t *out) {
  if (out == nullptr || !IsPayloadValid(id, payload, len)) {
    return 0;
  }

  Header *h = (Header *)out;
  h->magic[0] = kMagic1;
  h->magic[1] = kMagic2;
  h->id = (uint8_t)id;
  h->len = len;

  if (len > 0 && payload) {
    std::memcpy(out + sizeof(Header), payload, len);
  }

  uint16_t crc = Crc16(out, sizeof(Header) + len);

  // Append CRC (Little Endian)
  out[sizeof(Header) + len] = (uint8_t)(crc & 0xFF);
  out[sizeof(Header) + len + 1] = (uint8_t)((crc >> 8) & 0xFF);

  return sizeof(Header) + len + 2;
}

} // namespace message
