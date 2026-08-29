// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstdint>
#include <span>

// The checksums the wire protocols share. Each comes in an incremental form,
// for parsers that see one byte at a time, and a buffer form built from it.
namespace checksum {

// CRC-8/DVB-S2, poly 0xD5, init 0. Used by CRSF frames and MSP v2.
inline uint8_t Dvbs2Update(uint8_t crc, uint8_t byte) {
  crc ^= byte;
  for (uint8_t i = 0; i < 8u; ++i) {
    crc = (crc & 0x80u) ? static_cast<uint8_t>((crc << 1) ^ 0xD5u)
                        : static_cast<uint8_t>(crc << 1);
  }
  return crc;
}

[[nodiscard]] inline uint8_t Dvbs2(std::span<const uint8_t> bytes) {
  uint8_t crc = 0;
  for (const uint8_t byte : bytes) {
    crc = Dvbs2Update(crc, byte);
  }
  return crc;
}

// CRC-16/XMODEM, poly 0x1021, init 0. Used by the FC link packet trailer and
// by BLHeli four-way frames.
inline uint16_t XModemUpdate(uint16_t crc, uint8_t byte) {
  crc ^= static_cast<uint16_t>(byte) << 8;
  for (uint8_t i = 0; i < 8u; ++i) {
    crc = (crc & 0x8000u) ? static_cast<uint16_t>((crc << 1) ^ 0x1021u)
                          : static_cast<uint16_t>(crc << 1);
  }
  return crc;
}

[[nodiscard]] inline uint16_t XModem(std::span<const uint8_t> bytes) {
  uint16_t crc = 0;
  for (const uint8_t byte : bytes) {
    crc = XModemUpdate(crc, byte);
  }
  return crc;
}

// Fletcher-8, init 0. Used by UBX frames over everything between the sync
// pair and the checksum pair. The only two-byte result here, so it keeps a
// struct where the others return a scalar -- packing it into a uint16_t would
// make every caller unpack it again to compare the halves separately.
struct Fletcher8 {
  uint8_t ck_a;
  uint8_t ck_b;
};

inline void Fletcher8Update(Fletcher8 &ck, uint8_t byte) {
  ck.ck_a = static_cast<uint8_t>(ck.ck_a + byte);
  ck.ck_b = static_cast<uint8_t>(ck.ck_b + ck.ck_a);
}

[[nodiscard]] inline Fletcher8 Fletcher8Of(std::span<const uint8_t> bytes) {
  Fletcher8 ck{.ck_a = 0, .ck_b = 0};
  for (const uint8_t byte : bytes) {
    Fletcher8Update(ck, byte);
  }
  return ck;
}

// CRC-16/ARC, poly 0xA001 reflected, init 0. An ESC config session runs two
// CRC16s that are not interchangeable: XModem above for the host's four-way
// frames, this one for the bytes past the flight controller.
inline uint16_t Arc16Update(uint16_t crc, uint8_t byte) {
  for (uint8_t i = 0; i < 8u; ++i) {
    crc = ((crc ^ byte) & 1u) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001u)
                              : static_cast<uint16_t>(crc >> 1);
    byte = static_cast<uint8_t>(byte >> 1);
  }
  return crc;
}

}  // namespace checksum
