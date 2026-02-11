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

#include "system.hpp"
#include "time_base.hpp"
#include "uart.hpp"

#include <cstring>
#include <type_traits>

namespace {

// UBX-CFG-VALSET fixed header fields
constexpr uint8_t kValsetVersion = 0x00;

// Layers bitfield in VALSET:
// 0x01 = RAM only (volatile). Keep minimal and safe.
// 0x07 = RAM | BBR | Flash (persistent across reboots and battery removal)
constexpr uint8_t kValsetLayerRam = 0x01;
constexpr uint8_t kValsetLayerAll = 0x07;

// Key IDs you are using
constexpr uint32_t kKeyCfgRateMeasMs = 0x30210001; // CFG-RATE-MEAS
constexpr uint32_t kKeyCfgDynModel = 0x20110021;   // CFG-NAVSPG-DYNMODEL
constexpr uint32_t kKeyGpsEnable = 0x1031001f;     // CFG-SIGNAL-GPS_ENA
constexpr uint32_t kKeyGloEnable = 0x10310025;     // CFG-SIGNAL-GLO_ENA
constexpr uint32_t kKeyGalEnable = 0x10310021;     // CFG-SIGNAL-GAL_ENA
constexpr uint32_t kKeyBdsEnable = 0x10310022;     // CFG-SIGNAL-BDS_ENA
constexpr uint32_t kKeySbasEnable = 0x10310020;    // CFG-SIGNAL-SBAS_ENA

constexpr uint32_t kKeyItfmEnable = 0x10410013; // CFG-ITFM-ENABLE

constexpr uint32_t kKeyMsgoutNavPvtUart2 =
    0x20910009; // CFG-MSGOUT-UBX_NAV_PVT_UART2

constexpr uint32_t kKeyUart2Baudrate = 0x40530001;    // CFG-UART2-BAUDRATE
constexpr uint32_t kKeyUart2OutprotUbx = 0x10760001;  // CFG-UART2OUTPROT-UBX
constexpr uint32_t kKeyUart2OutprotNmea = 0x10760002; // CFG-UART2OUTPROT-NMEA

// Timepulse (TP1) Configuration
constexpr uint32_t kKeyCfgTp1Ena = 0x10050012;    // CFG-TP-TP1_ENA
constexpr uint32_t kKeyCfgTp1Period = 0x40050002; // CFG-TP-PERIOD_TP1 (us)
constexpr uint32_t kKeyCfgTp1Len = 0x40050004;    // CFG-TP-LEN_TP1 (us)
constexpr uint32_t kKeyCfgTp1TimeGrid =
    0x20050008; // CFG-TP-TIMEGRID_TP1 (1=GPS)

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

template <typename T>
void M9N::sendCfgValSet(uint32_t key, T value, uint8_t layer) {
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
  buf[7] = layer; // Use provided layer (RAM or persistent)
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

// Keep template in this TU only (it's only used here).
template void M9N::sendCfgValSet<uint8_t>(uint32_t, uint8_t, uint8_t);
template void M9N::sendCfgValSet<uint16_t>(uint32_t, uint16_t, uint8_t);
template void M9N::sendCfgValSet<uint32_t>(uint32_t, uint32_t, uint8_t);

void M9N::FlashConfig(uint32_t current_baud) {
  // ONE-TIME CONFIGURATION: Flash golden config to M9N.
  //
  // Steps:
  // 1. Connect at current_baud (e.g., 38400 factory default or existing config)
  // 2. Send all config with layer=0x07 (RAM+BBR+Flash) for persistence
  // 3. Wait for config to apply
  // 4. Reinit UART2 at 115200
  //
  // After this runs, M9N will boot at 115200 with golden config.

  auto &uart = Uart<UartInstance::kUart2>::GetInstance();

  // Temporarily reinit UART2 at current_baud to talk to M9N
  UartConfig temp_config = kUart2Config;
  temp_config.baudRate = current_baud;
  uart.ReInit(temp_config);

  // Small delay for UART to stabilize (100ms)
  auto &time = System::GetInstance().Time();
  uint64_t start = time.Micros();
  while ((time.Micros() - start) < MILLIS_TO_MICROS(100)) {
    // Busy wait
  }

  // Flush RX buffer to remove any NMEA/junk from before
  uart.FlushRx();

  // Send golden config with persistent layer (0x07 = RAM | BBR | Flash)
  // IMPORTANT: Baud rate change is done LAST to avoid race condition
  constexpr uint8_t kPersistent = kValsetLayerAll;

  // 1) Protocol output selection (must be first to reduce spam)
  sendCfgValSet<uint8_t>(kKeyUart2OutprotUbx, 1, kPersistent);
  sendCfgValSet<uint8_t>(kKeyUart2OutprotNmea, 0, kPersistent);

  // 2) Enable NAV-PVT output on UART2
  sendCfgValSet<uint8_t>(kKeyMsgoutNavPvtUart2, 1, kPersistent);

  // 3) 10 Hz (100 ms)
  sendCfgValSet<uint16_t>(kKeyCfgRateMeasMs, 100, kPersistent);

  // 4) Dynamic model: airborne < 2g (7)
  sendCfgValSet<uint8_t>(kKeyCfgDynModel, 7, kPersistent);

  // 5) Constellations
  sendCfgValSet<uint8_t>(kKeyGpsEnable, 1, kPersistent);
  sendCfgValSet<uint8_t>(kKeyGloEnable, 1, kPersistent);
  sendCfgValSet<uint8_t>(kKeyGalEnable, 1, kPersistent);
  sendCfgValSet<uint8_t>(kKeyBdsEnable, 1, kPersistent);
  sendCfgValSet<uint8_t>(kKeySbasEnable, 1, kPersistent);

  // 6) Interference mitigation (jamming/spoofing detection)
  sendCfgValSet<uint8_t>(kKeyItfmEnable, 1, kPersistent);

  // 7) Timepulse 1 (PPS) Configuration
  // Enable TP1, Periodic, 1Hz, 50ms active, GPS Timegrid
  sendCfgValSet<uint8_t>(kKeyCfgTp1Ena, 1, kPersistent);
  sendCfgValSet<uint32_t>(kKeyCfgTp1Period, 1000000,
                          kPersistent); // 1,000,000 us = 1s
  sendCfgValSet<uint32_t>(kKeyCfgTp1Len, 50000,
                          kPersistent); // 50,000 us = 50ms
  sendCfgValSet<uint8_t>(kKeyCfgTp1TimeGrid, 1, kPersistent); // 1 = GPS Time

  // 8) Baud rate change to 115200 (LAST to avoid race condition!)
  // All previous config is sent at current_baud, ensuring reliable delivery.
  // After this command, M9N may switch baud immediately.
  sendCfgValSet<uint32_t>(kKeyUart2Baudrate, 115200, kPersistent);

  // Wait for M9N to apply config and switch baud rate (500ms)
  start = time.Micros();
  while ((time.Micros() - start) < MILLIS_TO_MICROS(500)) {
    // Busy wait
  }

  // Reinit UART2 at 115200 to match M9N's new baud rate
  uart.ReInit(kUart2Config);

  // WARNING: No ACK handling implemented!
  // This function sends CFG-VALSET commands but does not parse ACK-ACK/NAK
  // responses. Flash writes (layer 0x07) can fail silently if:
  // - Keys are rejected by firmware
  // - Module is in wrong state
  // - Flash is write-protected
  //
  // TODO: Add ACK parsing to verify success, or at minimum:
  // - Parse parser output to verify UBX-only protocol (no NMEA)
  // - Verify NAV-PVT messages are received at 10Hz
  // - Check baud rate change succeeded (communication continues)
  //
  // For now, verify manually by observing GPS output after first boot.
}

void M9N::Init() {
  if (kFlashM9nConfig) {
    FlashConfig(38400);
  }

  // Runtime initialization (assumes M9N already configured via FlashConfig)
  // This sends config to RAM only for quick startup.
  //
  // NOTE: Baud rate is NOT set here. We assume M9N is already operating at
  // 115200 baud (set by FlashConfig and saved to BBR/Flash). Changing baud
  // rate requires coordinated UART re-initialization which is handled only
  // in FlashConfig.
  //
  // If M9N loses power/battery, it will revert to saved flash config,
  // which should already be 115200 + golden settings.

  // 1) Protocol output selection (reduce spam / ambiguity)
  sendCfgValSet<uint8_t>(kKeyUart2OutprotUbx, 1);
  sendCfgValSet<uint8_t>(kKeyUart2OutprotNmea, 0);

  // 2) Enable NAV-PVT output on UART2 (1 message per navigation solution)
  sendCfgValSet<uint8_t>(kKeyMsgoutNavPvtUart2, 1);

  // 3) 10 Hz (100 ms)
  sendCfgValSet<uint16_t>(kKeyCfgRateMeasMs, 100);

  // 4) Dynamic model: airborne < 2g (7)
  sendCfgValSet<uint8_t>(kKeyCfgDynModel, 7);

  // 5) Constellations (common baseline)
  sendCfgValSet<uint8_t>(kKeyGpsEnable, 1);
  sendCfgValSet<uint8_t>(kKeyGloEnable, 1);
  sendCfgValSet<uint8_t>(kKeyGalEnable, 1);
  sendCfgValSet<uint8_t>(kKeyBdsEnable, 1);
  sendCfgValSet<uint8_t>(kKeySbasEnable, 1);

  // 6) Interference mitigation (jamming/spoofing detection)
  sendCfgValSet<uint8_t>(kKeyItfmEnable, 1);

  // 7) Timepulse 1 (PPS) Configuration (RAM)
  sendCfgValSet<uint8_t>(kKeyCfgTp1Ena, 1);
  sendCfgValSet<uint32_t>(kKeyCfgTp1Period, 1000000);
  sendCfgValSet<uint32_t>(kKeyCfgTp1Len, 50000);
  sendCfgValSet<uint8_t>(kKeyCfgTp1TimeGrid, 1);
}

bool M9N::Read(uint8_t &b) {
  return Uart<UartInstance::kUart2>::GetInstance().Read(b);
}
