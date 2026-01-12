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

namespace protocol {
static constexpr uint8_t SYNC1 = 0xB5;
static constexpr uint8_t SYNC2 = 0x62;
static constexpr uint8_t CLS_NAV = 0x01;
static constexpr uint8_t ID_NAV_PVT = 0x07;
static constexpr uint8_t CLS_CFG = 0x06;
static constexpr uint8_t ID_CFG_VALSET = 0x8A;
} // namespace protocol

struct PVTData {
  uint32_t iTOW; // GPS Time of Week [ms]
  uint16_t year;
  uint8_t month;
  uint8_t day;
  uint8_t hour;
  uint8_t min;
  uint8_t sec;
  uint8_t valid; // Validity flags
  uint32_t tAcc; // Time accuracy estimate (UTC) [ns]
  int32_t nano;
  uint8_t fixType;  // GNSSfix type: 0=NoFix, 3=3D-Fix
  uint8_t flags;    // Fix Status Flags
  uint8_t flags2;   // Fix Status Flags 2
  uint8_t numSV;    // Number of SVs used in Nav Solution
  int32_t lon;      // [1e-7 deg]
  int32_t lat;      // [1e-7 deg]
  int32_t height;   // Height above ellipsoid [mm]
  int32_t hMSL;     // Height above mean sea level [mm]
  uint32_t hAcc;    // Horizontal accuracy estimate [mm]
  uint32_t vAcc;    // Vertical accuracy estimate [mm]
  int32_t velN;     // NED north velocity [mm/s]
  int32_t velE;     // NED east velocity [mm/s]
  int32_t velD;     // NED down velocity [mm/s]
  int32_t gSpeed;   // Ground Speed (2-D) [mm/s]
  int32_t headMot;  // Heading of motion (2-D) [1e-5 deg]
  uint32_t sAcc;    // Speed accuracy estimate [mm/s]
  uint32_t headAcc; // Heading accuracy estimate [1e-5 deg]
  uint16_t pDOP;    // Position DOP [0.01]
} __attribute__((packed));

struct M9NConfig {
  uint32_t baudrate = 115200;
};

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

  // We only need a buffer large enough for NAV-PVT (92 bytes + padding)
  static constexpr size_t MAX_PAYLOAD_SIZE = 120;
  uint8_t _payload_buf[MAX_PAYLOAD_SIZE];

  PVTData _pvt{};

  void calculateChecksum(const uint8_t *data, size_t len, uint8_t &ck_a,
                         uint8_t &ck_b);

  // Helper to send CFG-VALSET commands
  template <typename T> void sendCfgValSet(uint32_t key, T value);
};
