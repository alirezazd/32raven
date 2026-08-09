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

[[nodiscard]] inline uint8_t Dvbs2(std::span<const uint8_t> data) {
  uint8_t crc = 0;
  for (const uint8_t byte : data) {
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

[[nodiscard]] inline uint16_t XModem(std::span<const uint8_t> data) {
  uint16_t crc = 0;
  for (const uint8_t byte : data) {
    crc = XModemUpdate(crc, byte);
  }
  return crc;
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
