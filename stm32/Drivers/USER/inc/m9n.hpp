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

namespace ublox {

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
  M9N() = default;

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

  /**
   * @brief Configure the u-blox M9N with optimal settings.
   *        This template allows passing any callable that accepts (const
   * uint8_t*, size_t).
   *
   * @tparam Sender Callable type for sending data to UART.
   * @param config Configuration parameters.
   * @param sender Callback to send data: sender(data_ptr, length)
   */
  template <typename Sender>
  void configure(const M9NConfig &config, Sender &&sender) {
    // 1. Set baudrate (CFG-UART1-BAUDRATE)
    // 2. Set update rate (CFG-RATE-MEAS)
    // 3. Enable NAV-PVT
    // Implementation will generate the bytes and call sender()
    // Note: Actual implementation of constructing the packet needs to be in a
    // method we can call from here. We will implement a helper 'sendValSet'
    // that generates the bytes and calls the sender.

    // Example: Set Baudrate
    // uint32_t baud_key = 0x40520001; // CFG-UART1-BAUDRATE
    // sendCfgValSet(baud_key, config.baudrate, sender);

    // Helper delay
    auto simple_delay = [](uint32_t count) {
      volatile uint32_t i = 0;
      for (; i < count; i++) {
        __asm__("nop");
      }
    };

    // 1. Set Rate to 10Hz (100ms)
    // Key: CFG-RATE-MEAS (0x30210001)
    uint32_t rate_key = 0x30210001;
    sendCfgValSet(rate_key, (uint16_t)100, sender);
    simple_delay(1000000);

    // 2. Set Dynamic Model to Airborne < 4g
    // Key: CFG-NAVSPG-DYNMODEL (0x20110021). Value: 8 (Airborne < 4g)
    sendCfgValSet(0x20110021, (uint8_t)8, sender);
    simple_delay(1000000);

    // 3. Enable All Constellations
    // GPS (0x1031001f), GLONASS (0x10310025), Galileo (0x10310021), BeiDou
    // (0x10310022)
    sendCfgValSet(0x1031001f, (uint8_t)1, sender); // GPS
    simple_delay(500000);
    sendCfgValSet(0x10310025, (uint8_t)1, sender); // GLONASS
    simple_delay(500000);
    sendCfgValSet(0x10310021, (uint8_t)1, sender); // Galileo
    simple_delay(500000);
    sendCfgValSet(0x10310022, (uint8_t)1, sender); // BeiDou
    simple_delay(1000000);

    // 4. Enable NAV-PVT
    // Key: CFG-MSGOUT-UBX_NAV_PVT_UART1 (0x20910007)
    uint32_t msg_key = 0x20910007;
    sendCfgValSet(msg_key, (uint8_t)1, sender);
    simple_delay(1000000);

    // 5. Disable NMEA
    // Key: CFG-UART1OUTPROT-NMEA (0x10740002)
    uint32_t nmea_key = 0x10740002;
    sendCfgValSet(nmea_key, (uint8_t)0, sender);
    simple_delay(1000000);

    // 6. Enable UBX
    // Key: CFG-UART1OUTPROT-UBX (0x10740001)
    uint32_t ubx_key = 0x10740001;
    sendCfgValSet(ubx_key, (uint8_t)1, sender);
    simple_delay(1000000);
  }

private:
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

  // Helper to send CFG-VALSET for a single key-value pair
  // Used template here or move implementation to cpp if we want to hide it
  // But since configure is a template, helpers used by it must be accessible
  // if they are templates or generic. Simplifying: We can make a specialized
  // helper for internal use.

  template <typename T, typename Sender>
  void sendCfgValSet(uint32_t key, T value, Sender &&sender) {
    // Frame structure:
    // Sync1, Sync2, Cls(06), Id(8A), LenL, LenH,
    // Version(1), Layers(1=RAM), Reserved(2),
    // Key(4), Value(1,2,4,8)
    // CK_A, CK_B

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

    sender(buf, packet_len);
  }
};

} // namespace ublox
