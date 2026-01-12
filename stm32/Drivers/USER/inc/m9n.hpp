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

class M9N {
public:
  static M9N &GetInstance() {
    static M9N instance;
    return instance;
  }

  // Read a byte from the GPS UART buffer
  bool Read(uint8_t &b);

private:
  friend class System;
  void Init();

  /**
   * @brief Flash golden config to M9N and set to 115200 baud.
   *
   * @param current_baud Current baud rate of M9N (default: 38400 factory
   * default)
   *
   * This function connects at the specified baud rate, sends the golden config
   * with persistent storage (flash), then switches to 115200 baud.
   */
  void FlashConfig(uint32_t current_baud = 38400);

  M9N() = default;
  ~M9N() = default;
  M9N(const M9N &) = delete;
  M9N &operator=(const M9N &) = delete;

  // Helper to send CFG-VALSET commands
  template <typename T>
  void sendCfgValSet(uint32_t key, T value, uint8_t layer = 0x01);
};
