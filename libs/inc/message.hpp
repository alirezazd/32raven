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
static constexpr size_t kMaxPayload = 256;

// Message Identifiers
enum class MsgId : uint8_t {
  kPing = 0x01,
  kLog = 0x02,
  kPong = 0x03,
  kRcChannels = 0x65,
  kGpsData = 0x10,
  kImuData = 0x11,
  kTimeSync = 0x12,
  kConfig = 0x13,
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

struct TimeSyncMsg {
  uint32_t timestamp;   // Local timestamp (micros)
  int32_t drift_micros; // Drift from GPS PPS
  uint8_t synced;       // 1 if synced, 0 otherwise
} __attribute__((packed));

struct ConfigMsg {
  uint8_t telemetry_rate_hz;
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
  uint64_t timestamp_us; // GPS-corrected microseconds
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
// 'out' must be at least 'len' + 5 bytes (Header + CRC).
static inline size_t Serialize(MsgId id, const uint8_t *payload, uint8_t len,
                               uint8_t *out) {
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
