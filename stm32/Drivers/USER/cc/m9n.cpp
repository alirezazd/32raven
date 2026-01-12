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

// Template implementation for sendCfgValSet
template <typename T> void M9N::sendCfgValSet(uint32_t key, T value) {
  constexpr size_t val_size = sizeof(T);
  constexpr size_t payload_len =
      4 + 4 + val_size; // Ver+Lay+Res(4) + Key(4) + Val
  constexpr size_t packet_len = 6 + payload_len + 2;

  uint8_t buf[packet_len];
  buf[0] = protocol::SYNC1;
  buf[1] = protocol::SYNC2;
  buf[2] = protocol::CLS_CFG;
  buf[3] = protocol::ID_CFG_VALSET;
  buf[4] = payload_len & 0xFF;
  buf[5] = (payload_len >> 8) & 0xFF;

  buf[6] = 0x00; // Version
  buf[7] = 0x01; // Layer RAM
  buf[8] = 0x00; // Reserved
  buf[9] = 0x00; // Reserved

  // Key (Little Endian)
  buf[10] = key & 0xFF;
  buf[11] = (key >> 8) & 0xFF;
  buf[12] = (key >> 16) & 0xFF;
  buf[13] = (key >> 24) & 0xFF;

  // Value (Little Endian)
  for (size_t i = 0; i < val_size; i++) {
    buf[14 + i] = (value >> (i * 8)) & 0xFF;
  }

  // Checksum
  uint8_t ck_a = 0, ck_b = 0;
  for (size_t i = 2; i < packet_len - 2; i++) {
    ck_a += buf[i];
    ck_b += ck_a;
  }
  buf[packet_len - 2] = ck_a;
  buf[packet_len - 1] = ck_b;

  Uart<UartInstance::kUart2>::GetInstance().Send(buf, packet_len);
}

void M9N::Init() {
  // Configure M9N with "Sweet Spot" settings
  // No delays needed - GPS processes commands asynchronously

  // 1. Set Rate to 10Hz (100ms)
  sendCfgValSet(0x30210001, (uint16_t)100);

  // 2. Set Dynamic Model to Airborne < 4g
  sendCfgValSet(0x20110021, (uint8_t)8);

  // 3. Enable All Constellations
  sendCfgValSet(0x1031001f, (uint8_t)1); // GPS
  sendCfgValSet(0x10310025, (uint8_t)1); // GLONASS
  sendCfgValSet(0x10310021, (uint8_t)1); // Galileo
  sendCfgValSet(0x10310022, (uint8_t)1); // BeiDou

  // 4. Enable NAV-PVT
  sendCfgValSet(0x20910007, (uint8_t)1);

  // 5. Disable NMEA
  sendCfgValSet(0x10740002, (uint8_t)0);

  // 6. Enable UBX
  sendCfgValSet(0x10740001, (uint8_t)1);
}

void M9N::calculateChecksum(const uint8_t *data, size_t len, uint8_t &ck_a,
                            uint8_t &ck_b) {
  ck_a = 0;
  ck_b = 0;
  for (size_t i = 0; i < len; i++) {
    ck_a += data[i];
    ck_b += ck_a;
  }
}

bool M9N::parse(uint8_t b) {
  switch (_state) {
  case State::SYNC1:
    if (b == protocol::SYNC1)
      _state = State::SYNC2;
    break;
  case State::SYNC2:
    if (b == protocol::SYNC2)
      _state = State::CLASS;
    else
      _state = State::SYNC1;
    break;
  case State::CLASS:
    _cls = b;
    _ck_a = 0;
    _ck_b = 0;
    _ck_a += b;
    _ck_b += _ck_a;
    _state = State::ID;
    break;
  case State::ID:
    _id = b;
    _ck_a += b;
    _ck_b += _ck_a;
    _state = State::LENGTH_L;
    break;
  case State::LENGTH_L:
    _len = b;
    _ck_a += b;
    _ck_b += _ck_a;
    _state = State::LENGTH_H;
    break;
  case State::LENGTH_H:
    _len |= (b << 8);
    _ck_a += b;
    _ck_b += _ck_a;
    _payload_idx = 0;
    if (_len > MAX_PAYLOAD_SIZE) {
      _state = State::SYNC1; // Payload too big for our buffer
    } else {
      _state = State::PAYLOAD;
    }
    break;
  case State::PAYLOAD:
    if (_payload_idx < _len) {
      _payload_buf[_payload_idx++] = b;
      _ck_a += b;
      _ck_b += _ck_a;
    }
    if (_payload_idx == _len) {
      _state = State::CK_A;
    }
    break;
  case State::CK_A:
    if (b == _ck_a)
      _state = State::CK_B;
    else
      _state = State::SYNC1;
    break;
  case State::CK_B:
    if (b == _ck_b) {
      // Packet Valid
      if (_cls == protocol::CLS_NAV && _id == protocol::ID_NAV_PVT) {
        if (_len >= 92) {
          // Copy to PVT struct; minimal padding safety check
          memcpy(&_pvt, _payload_buf, sizeof(PVTData));
          _state = State::SYNC1;
          return true;
        }
      }
    }
    _state = State::SYNC1;
    break;
  }
  return false;
}
