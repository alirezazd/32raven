/****************************************************************************
 *
 *   Copyright (c) 2016-2024 PX4 Pro Drone Autopilot. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace protocol {
static constexpr uint8_t SYNC1 = 0xB5;
static constexpr uint8_t SYNC2 = 0x62;
static constexpr uint8_t CLS_NAV = 0x01;
static constexpr uint8_t ID_NAV_PVT = 0x07;
static constexpr uint8_t CLS_CFG = 0x06;
static constexpr uint8_t ID_CFG_VALSET = 0x8A;
static constexpr uint8_t CLS_ACK = 0x05;
static constexpr uint8_t ID_ACK_ACK = 0x01;
static constexpr uint8_t ID_ACK_NAK = 0x00;
} // namespace protocol

struct PVTData {
  uint32_t iTOW; // 0 GPS Time of Week [ms]
  uint16_t year; // 4 Year
  uint8_t month; // 6 Month
  uint8_t day;   // 7 Day
  uint8_t hour;  // 8 Hour
  uint8_t min;   // 9 Minute
  uint8_t sec;   // 10 Second
  uint8_t valid; // 11 Validity flags
  uint32_t tAcc; // 12 Time accuracy estimate (UTC) [ns]
  int32_t nano;  // 16 Fractional part of iTOW [ns]

  uint8_t fixType; // 20 Fix type
  uint8_t flags;   // 21 Fix Status Flags
  uint8_t flags2;  // 22 Fix Status Flags 2
  uint8_t numSV;   // 23 Number of SVs used in Nav Solution

  int32_t lon;    // 24 Longitude [1e-7 deg]
  int32_t lat;    // 28 Latitude [1e-7 deg]
  int32_t height; // 32 Height above ellipsoid [mm]
  int32_t hMSL;   // 36 Height above mean sea level [mm]
  uint32_t hAcc;  // 40 Horizontal accuracy estimate [mm]
  uint32_t vAcc;  // 44 Vertical accuracy estimate [mm]

  int32_t velN;     // 48 NED north velocity [mm/s]
  int32_t velE;     // 52 NED east velocity [mm/s]
  int32_t velD;     // 56 NED down velocity [mm/s]
  int32_t gSpeed;   // 60 Ground Speed (2-D) [mm/s]
  int32_t headMot;  // 64 Heading of motion (2-D) [1e-5 deg]
  uint32_t sAcc;    // 68 Speed accuracy estimate [mm/s]
  uint32_t headAcc; // 72 Heading accuracy estimate [1e-5 deg]

  uint16_t pDOP;        // 76 Position DOP [0.01]
  uint8_t reserved1[6]; // 78..83 Reserved

  int32_t headVeh; // 84 Heading of vehicle (2-D) [1e-5 deg]
  int16_t magDec;  // 88 Magnetic Declination [1e-5 deg]
  uint16_t magAcc; // 90 Magnetic Declination accuracy [1e-5 deg]
} __attribute__((packed));

// PVTData matches full UBX NAV-PVT payload (92 bytes)
static_assert(sizeof(PVTData) == 92,
              "PVTData size must match UBX NAV-PVT payload");

class M9N {
public:
  static M9N &GetInstance() {
    static M9N instance;
    return instance;
  }

  /**
   * @brief Parse a single byte from the UART stream.
   *
   * @param b The byte to parse.
   * @return true if a complete NAV-PVT message has been received and updated.
   */
  bool parse(uint8_t b);

  /**
   * @brief Access the latest parsed PVT data.
   */
  const PVTData &pvt() const { return _pvt; }

private:
  friend class System;
  void Init();

  M9N() = default;
  ~M9N() = default;
  M9N(const M9N &) = delete;
  M9N &operator=(const M9N &) = delete;

  enum class State {
    SYNC1,
    SYNC2,
    CLASS,
    ID,
    LENGTH_L,
    LENGTH_H,
    PAYLOAD,
    CK_A,
    CK_B
  };

  State _state = State::SYNC1;
  uint8_t _cls = 0;
  uint8_t _id = 0;
  uint16_t _len = 0;
  uint16_t _payload_idx = 0;
  uint8_t _ck_a = 0;
  uint8_t _ck_b = 0;

  // Buffer for largest UBX message we parse (NAV-PVT is 92 bytes; 120 gives
  // headroom)
  static constexpr size_t MAX_PAYLOAD_SIZE = 120;
  uint8_t _payload_buf[MAX_PAYLOAD_SIZE];

  PVTData _pvt{};

  // Helper to send CFG-VALSET commands
  template <typename T> void sendCfgValSet(uint32_t key, T value);
};
