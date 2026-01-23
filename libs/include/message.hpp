#pragma once

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

struct GpsData {
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t valid;
  uint32_t tAcc; // ns
  int32_t lon;
  int32_t lat;
  int32_t height; // hMSL
  uint8_t fixType;
  uint8_t numSV;
} __attribute__((packed));

struct Packet {
  Header header;
  uint8_t payload[kMaxPayload];
  uint16_t crc; // CRC16-CCITT (XMODEM)
};

#pragma pack(pop)

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
