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

#include "m9n.hpp"

#include "uart.hpp"

#include <cstring>
#include <type_traits>

namespace {

// UBX-CFG-VALSET fixed header fields
constexpr uint8_t kValsetVersion = 0x00;

// Layers bitfield in VALSET:
// 0x01 = RAM only (volatile). Keep minimal and safe.
// If later you want persistence, you can use 0x07 (RAM | BBR | Flash) on
// modules that support it, but do that only after you add ACK handling.
constexpr uint8_t kValsetLayerRam = 0x01;

// Key IDs you are using
constexpr uint32_t kKeyCfgRateMeasMs = 0x30210001; // CFG-RATE-MEAS
constexpr uint32_t kKeyCfgDynModel = 0x20110021;   // CFG-NAVSPG-DYNMODEL
constexpr uint32_t kKeyGpsEnable = 0x1031001f;     // CFG-SIGNAL-GPS_ENA
constexpr uint32_t kKeyGloEnable = 0x10310025;     // CFG-SIGNAL-GLO_ENA
constexpr uint32_t kKeyGalEnable = 0x10310021;     // CFG-SIGNAL-GAL_ENA
constexpr uint32_t kKeyBdsEnable = 0x10310022;     // CFG-SIGNAL-BDS_ENA

constexpr uint32_t kKeyMsgoutNavPvtUart1 =
    0x20910007; // CFG-MSGOUT-UBX_NAV_PVT_UART1
constexpr uint32_t kKeyUart1OutprotUbx = 0x10740001;  // CFG-UART1OUTPROT-UBX
constexpr uint32_t kKeyUart1OutprotNmea = 0x10740002; // CFG-UART1OUTPROT-NMEA

inline void UbxChecksum(const uint8_t *data, size_t len, uint8_t &ck_a,
                        uint8_t &ck_b) {
  ck_a = 0;
  ck_b = 0;
  for (size_t i = 0; i < len; i++) {
    ck_a = static_cast<uint8_t>(ck_a + data[i]);
    ck_b = static_cast<uint8_t>(ck_b + ck_a);
  }
}

} // namespace

template <typename T> void M9N::sendCfgValSet(uint32_t key, T value) {
  static_assert(std::is_trivially_copyable_v<T>,
                "CFG-VALSET value must be trivially copyable");

  constexpr size_t kValSize = sizeof(T);
  constexpr uint16_t kPayloadLen =
      static_cast<uint16_t>(4 + 4 + kValSize); // ver/layer/res + key + value
  constexpr size_t kPacketLen = 6 + kPayloadLen + 2;

  uint8_t buf[kPacketLen];

  // UBX header
  buf[0] = protocol::SYNC1;
  buf[1] = protocol::SYNC2;
  buf[2] = protocol::CLS_CFG;
  buf[3] = protocol::ID_CFG_VALSET;
  buf[4] = static_cast<uint8_t>(kPayloadLen & 0xFF);
  buf[5] = static_cast<uint8_t>((kPayloadLen >> 8) & 0xFF);

  // Payload: version, layer, reserved(2)
  buf[6] = kValsetVersion;
  buf[7] = kValsetLayerRam;
  buf[8] = 0x00;
  buf[9] = 0x00;

  // Key little-endian
  buf[10] = static_cast<uint8_t>(key & 0xFF);
  buf[11] = static_cast<uint8_t>((key >> 8) & 0xFF);
  buf[12] = static_cast<uint8_t>((key >> 16) & 0xFF);
  buf[13] = static_cast<uint8_t>((key >> 24) & 0xFF);

  // Value bytes (little-endian on STM32)
  std::memcpy(&buf[14], &value, kValSize);

  // Checksum over CLASS..payload
  uint8_t ck_a = 0, ck_b = 0;
  UbxChecksum(&buf[2], 4 + kPayloadLen, ck_a, ck_b);
  buf[kPacketLen - 2] = ck_a;
  buf[kPacketLen - 1] = ck_b;

  auto &uart = Uart<UartInstance::kUart2>::GetInstance();
  uart.Send(buf, kPacketLen);
}

// Keep template in this TU only (it’s only used here).
template void M9N::sendCfgValSet<uint8_t>(uint32_t, uint8_t);
template void M9N::sendCfgValSet<uint16_t>(uint32_t, uint16_t);

void M9N::Init() {
  // Best-practice baseline (Betaflight/PX4-style):
  // - UBX only (no NMEA)
  // - NAV-PVT enabled on UART1
  // - 10 Hz update
  // - dynamic model: airborne <4g
  // - multi-constellation enabled
  //
  // Note: we configure RAM layer only (volatile) to keep init simple and safe.

  // 1) Protocol output selection first (reduce spam / ambiguity)
  sendCfgValSet<uint8_t>(kKeyUart1OutprotUbx, 1);
  sendCfgValSet<uint8_t>(kKeyUart1OutprotNmea, 0);

  // 2) Enable NAV-PVT output on UART1 (1 message per navigation solution)
  sendCfgValSet<uint8_t>(kKeyMsgoutNavPvtUart1, 1);

  // 3) 10 Hz (100 ms)
  sendCfgValSet<uint16_t>(kKeyCfgRateMeasMs, 100);

  // 4) Dynamic model: airborne < 4g (8)
  sendCfgValSet<uint8_t>(kKeyCfgDynModel, 8);

  // 5) Constellations (common baseline)
  sendCfgValSet<uint8_t>(kKeyGpsEnable, 1);
  sendCfgValSet<uint8_t>(kKeyGloEnable, 1);
  sendCfgValSet<uint8_t>(kKeyGalEnable, 1);
  sendCfgValSet<uint8_t>(kKeyBdsEnable, 1);
}

bool M9N::parse(uint8_t b) {
  switch (_state) {
  case State::SYNC1:
    if (b == protocol::SYNC1) {
      _state = State::SYNC2;
    }
    break;

  case State::SYNC2:
    if (b == protocol::SYNC2) {
      _state = State::CLASS;
    } else if (b == protocol::SYNC1) {
      // stay in SYNC2 (handles B5 B5 62 ...)
      _state = State::SYNC2;
    } else {
      _state = State::SYNC1;
    }
    break;

  case State::CLASS:
    _cls = b;
    _ck_a = 0;
    _ck_b = 0;
    _ck_a = static_cast<uint8_t>(_ck_a + b);
    _ck_b = static_cast<uint8_t>(_ck_b + _ck_a);
    _state = State::ID;
    break;

  case State::ID:
    _id = b;
    _ck_a = static_cast<uint8_t>(_ck_a + b);
    _ck_b = static_cast<uint8_t>(_ck_b + _ck_a);
    _state = State::LENGTH_L;
    break;

  case State::LENGTH_L:
    _len = b;
    _ck_a = static_cast<uint8_t>(_ck_a + b);
    _ck_b = static_cast<uint8_t>(_ck_b + _ck_a);
    _state = State::LENGTH_H;
    break;

  case State::LENGTH_H:
    _len |= static_cast<uint16_t>(b) << 8;
    _ck_a = static_cast<uint8_t>(_ck_a + b);
    _ck_b = static_cast<uint8_t>(_ck_b + _ck_a);

    _payload_idx = 0;
    if (_len > MAX_PAYLOAD_SIZE) {
      _state = State::SYNC1;
    } else {
      _state = State::PAYLOAD;
    }
    break;

  case State::PAYLOAD:
    _payload_buf[_payload_idx++] = b;
    _ck_a = static_cast<uint8_t>(_ck_a + b);
    _ck_b = static_cast<uint8_t>(_ck_b + _ck_a);

    if (_payload_idx >= _len) {
      _state = State::CK_A;
    }
    break;

  case State::CK_A:
    _state = (b == _ck_a) ? State::CK_B : State::SYNC1;
    break;

  case State::CK_B:
    if (b == _ck_b) {
      if (_cls == protocol::CLS_NAV && _id == protocol::ID_NAV_PVT &&
          _len == sizeof(PVTData)) {
        std::memcpy(&_pvt, _payload_buf, sizeof(PVTData));
        _state = State::SYNC1;
        return true;
      }
    }
    _state = State::SYNC1;
    break;
  }

  return false;
}