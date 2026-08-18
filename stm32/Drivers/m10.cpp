// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "m10.hpp"

#include <cstring>
#include <type_traits>
#include <utility>

#include "error_code.hpp"
#include "m10_reg.hpp"
#include "panic.hpp"
#include "system.hpp"
#include "time_base.hpp"
#include "uart.hpp"

inline void UbxChecksum(const uint8_t *data, size_t len, uint8_t &ck_a,
                        uint8_t &ck_b) {
  ck_a = 0;
  ck_b = 0;
  for (size_t i = 0; i < len; i++) {
    ck_a = static_cast<uint8_t>(ck_a + data[i]);
    ck_b = static_cast<uint8_t>(ck_b + ck_a);
  }
}

M10 &M10::GetInstance() {
  static M10 instance;
  return instance;
}

void M10::WaitForReady() {
  auto &uart = Uart<UartInstance::kUart2>::GetInstance();
  auto &time = System::GetInstance().Time();
  const uint32_t start = time.Micros();

  while ((uint32_t)(time.Micros() - start) < MillisToMicros(1000)) {
    uart.FlushRx();

    SendCfgValSetRaw<uint8_t>(kKeyUart1OutprotUbx, 1, ValsetLayer::kRam);
    if (WaitForAck(UBX::kClsCfg, UBX::kIdCfgValset)) {
      return;
    }

    time.DelayMicros(MillisToMicros(50));
  }

  Panic(ErrorCode::Stm32::kGpsNotResponding);
}

template bool M10::SendCfgValSet<uint8_t>(uint32_t, uint8_t, M10::ValsetLayer);
template bool M10::SendCfgValSet<uint16_t>(uint32_t, uint16_t,
                                           M10::ValsetLayer);
template bool M10::SendCfgValSet<uint32_t>(uint32_t, uint32_t,
                                           M10::ValsetLayer);

template void M10::SendCfgValSetRaw<uint8_t>(uint32_t, uint8_t,
                                             M10::ValsetLayer);
template void M10::SendCfgValSetRaw<uint16_t>(uint32_t, uint16_t,
                                              M10::ValsetLayer);
template void M10::SendCfgValSetRaw<uint32_t>(uint32_t, uint32_t,
                                              M10::ValsetLayer);

template bool M10::WaitForValget<uint8_t>(uint32_t, uint8_t);
template bool M10::WaitForValget<uint16_t>(uint32_t, uint16_t);
template bool M10::WaitForValget<uint32_t>(uint32_t, uint32_t);

void M10::ApplyConfig(ValsetLayer layer) {
  // `auto value` keeps SendCfgValSet's template dispatch, so each key still
  // picks its own UBX storage width. A key that goes unacknowledged has to
  // panic; folding that in makes it impossible to add a key and forget.
  const auto set = [&](uint32_t key, auto value, ErrorCode::Stm32 err) {
    if (!SendCfgValSet(key, value, layer)) {
      Panic(err);
    }
  };

  // Error codes stay fully qualified: scripts/lint/check_error_codes.py finds
  // live enumerators by matching `ErrorCode::Stm32::k...`, so a `using enum`
  // here would report all eight below as dead and offer to delete them.
  const auto u8 = [](auto v) { return static_cast<uint8_t>(v); };

  set(kKeyUart1Baudrate, ToBaudRateValue(config_.baud_rate),
      ErrorCode::Stm32::kGpsVerifyProtocolFailed);
  set(kKeyUart1StopBits, u8(config_.uart1.stop_bits),
      ErrorCode::Stm32::kGpsVerifyProtocolFailed);
  set(kKeyUart1DataBits, u8(config_.uart1.data_bits),
      ErrorCode::Stm32::kGpsVerifyProtocolFailed);
  set(kKeyUart1Parity, u8(config_.uart1.parity),
      ErrorCode::Stm32::kGpsVerifyProtocolFailed);
  set(kKeyUart1InprotUbx, u8(true),
      ErrorCode::Stm32::kGpsVerifyProtocolFailed);
  set(kKeyUart1OutprotUbx, u8(config_.protocols.outprot_ubx),
      ErrorCode::Stm32::kGpsVerifyProtocolFailed);
  set(kKeyUart1OutprotNmea, u8(config_.protocols.outprot_nmea),
      ErrorCode::Stm32::kGpsVerifyProtocolFailed);

  set(kKeyMsgoutNavPvtUart1, u8(config_.messages.nav_pvt),
      ErrorCode::Stm32::kGpsVerifyNavPvtFailed);
  set(kKeyMsgoutNavDopUart1, u8(config_.messages.nav_dop),
      ErrorCode::Stm32::kGpsVerifyNavDopFailed);
  set(kKeyMsgoutNavCovUart1, u8(config_.messages.nav_cov),
      ErrorCode::Stm32::kGpsVerifyNavCovFailed);
  set(kKeyMsgoutNavEoeUart1, u8(config_.messages.nav_eoe),
      ErrorCode::Stm32::kGpsVerifyNavEoeFailed);

  set(kKeyCfgRateMeasMs, config_.nav.rate_meas_ms,
      ErrorCode::Stm32::kGpsVerifyRateFailed);
  set(kKeyCfgDynModel, u8(config_.nav.dyn_model),
      ErrorCode::Stm32::kGpsVerifyDynModelFailed);

  set(kKeyGpsEnable, u8(config_.gnss.gps_enable),
      ErrorCode::Stm32::kGpsVerifyConstellationFailed);
  set(kKeyGloEnable, u8(config_.gnss.glo_enable),
      ErrorCode::Stm32::kGpsVerifyConstellationFailed);
  set(kKeyGalEnable, u8(config_.gnss.gal_enable),
      ErrorCode::Stm32::kGpsVerifyConstellationFailed);
  set(kKeyBdsEnable, u8(config_.gnss.bds_enable),
      ErrorCode::Stm32::kGpsVerifyConstellationFailed);
  set(kKeySbasEnable, u8(config_.gnss.sbas_enable),
      ErrorCode::Stm32::kGpsVerifyConstellationFailed);
  set(kKeyItfmEnable, u8(config_.gnss.itfm_enable),
      ErrorCode::Stm32::kGpsVerifyItfmFailed);

  // The leading 4 is CFG-VALSET's own header -- version, layers and two
  // reserved bytes -- which the length field counts along with the key/value
  // pairs. SendCfgValSet and its valget counterpart include it too.
  constexpr uint16_t payload_len =
      4 + (4 + 1) + (4 + 4) + (4 + 4) + (4 + 1) + (4 + 1) + (4 + 1) + (4 + 1);
  constexpr size_t packet_len = 6 + payload_len + 2;
  uint8_t buf[packet_len];

  buf[0] = UBX::kSync1;
  buf[1] = UBX::kSync2;
  buf[2] = UBX::kClsCfg;
  buf[3] = UBX::kIdCfgValset;
  buf[4] = payload_len & 0xFF;
  buf[5] = (payload_len >> 8) & 0xFF;

  buf[6] = kValsetVersion;
  buf[7] = std::to_underlying(layer);
  buf[8] = 0;
  buf[9] = 0;

  size_t idx = 10;

  auto write_key_u1 = [&](uint32_t key, uint8_t v) {
    buf[idx++] = key & 0xFF;
    buf[idx++] = (key >> 8) & 0xFF;
    buf[idx++] = (key >> 16) & 0xFF;
    buf[idx++] = (key >> 24) & 0xFF;
    buf[idx++] = v;
  };

  auto write_key_u4 = [&](uint32_t key, uint32_t v) {
    buf[idx++] = key & 0xFF;
    buf[idx++] = (key >> 8) & 0xFF;
    buf[idx++] = (key >> 16) & 0xFF;
    buf[idx++] = (key >> 24) & 0xFF;
    std::memcpy(&buf[idx], &v, 4);
    idx += 4;
  };

  write_key_u1(kKeyCfgTp1Ena, static_cast<uint8_t>(config_.tp1.ena));
  write_key_u4(kKeyCfgTp1Period, config_.tp1.period);
  write_key_u4(kKeyCfgTp1Len, config_.tp1.len);
  write_key_u1(kKeyCfgTp1TimeGrid, static_cast<uint8_t>(config_.tp1.timegrid));
  write_key_u1(kKeyCfgTp1SyncGnss, static_cast<uint8_t>(config_.tp1.sync_gnss));
  write_key_u1(kKeyCfgTp1AlignToTow,
               static_cast<uint8_t>(config_.tp1.align_to_tow));
  write_key_u1(kKeyCfgTp1Pol, static_cast<uint8_t>(config_.tp1.pol_rising));

  // payload_len is measured from buf[6], where the CFG-VALSET payload starts.
  if (idx != 6 + payload_len) {
    Panic(ErrorCode::Stm32::kGpsConfigTimepulseBufferError);
  }

  uint8_t ck_a = 0;
  uint8_t ck_b = 0;
  UbxChecksum(&buf[2], 4 + payload_len, ck_a, ck_b);
  buf[packet_len - 2] = ck_a;
  buf[packet_len - 1] = ck_b;

  Uart<UartInstance::kUart2>::GetInstance().Send(buf, packet_len);
  if (!WaitForAck(UBX::kClsCfg, UBX::kIdCfgValset)) {
    Panic(ErrorCode::Stm32::kGpsConfigTimepulseFailed);
  }

  if (config_.uart1.enabled) {
    if (!SendCfgValSet(kKeyUart1Enabled, static_cast<uint8_t>(true), layer)) {
      Panic(ErrorCode::Stm32::kGpsVerifyProtocolFailed);
    }
  } else {
    SendCfgValSetRaw<uint8_t>(kKeyUart1Enabled, 0, layer);
    if (!WaitForAck(UBX::kClsCfg, UBX::kIdCfgValset)) {
      Panic(ErrorCode::Stm32::kGpsVerifyProtocolFailed);
    }
  }
}

void M10::Init(const Config &config) {
  config_ = config;

  WaitForReady();
  Uart<UartInstance::kUart2>::GetInstance().FlushRx();
  ApplyConfig(ValsetLayer::kRam);
}

bool M10::Read(uint8_t &b) {
  return Uart<UartInstance::kUart2>::GetInstance().ReadByte(b);
}

bool M10::WaitForAck(uint8_t want_cls, uint8_t want_id) {
  auto &uart = Uart<UartInstance::kUart2>::GetInstance();
  auto &time = System::GetInstance().Time();
  const uint32_t start = time.Micros();

  uint8_t frame[10];
  uint8_t idx = 0;

  while ((uint32_t)(time.Micros() - start) < config_.ack_timeout_us) {
    uint8_t b;
    if (!uart.ReadByte(b)) continue;

    if (idx == 0) {
      if (b != UBX::kSync1) continue;
      frame[idx++] = b;
      continue;
    }
    if (idx == 1) {
      if (b != UBX::kSync2) {
        idx = 0;
        continue;
      }
      frame[idx++] = b;
      continue;
    }

    frame[idx++] = b;
    if (idx < sizeof(frame)) continue;
    idx = 0;

    if (frame[2] != UBX::kClsAck) continue;

    const uint8_t id = frame[3];
    if (id != UBX::kIdAckAck && id != UBX::kIdAckNak) continue;

    if (frame[4] != 0x02 || frame[5] != 0x00) continue;

    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    uint8_t chk_buf[6] = {frame[2], frame[3], frame[4],
                          frame[5], frame[6], frame[7]};
    UbxChecksum(chk_buf, sizeof(chk_buf), ck_a, ck_b);
    if (ck_a != frame[8] || ck_b != frame[9]) continue;

    if (frame[6] == want_cls && frame[7] == want_id) {
      return id == UBX::kIdAckAck;
    }
  }

  return false;
}

template <typename T>
void M10::SendCfgValSetRaw(uint32_t key, T value, ValsetLayer layer) {
  static_assert(std::is_integral_v<T> || std::is_enum_v<T>);
  static_assert(sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4);

  constexpr uint16_t payload_len = 4 + 4 + sizeof(T);
  constexpr size_t packet_len = 6 + payload_len + 2;

  uint8_t buf[packet_len];

  buf[0] = UBX::kSync1;
  buf[1] = UBX::kSync2;
  buf[2] = UBX::kClsCfg;
  buf[3] = UBX::kIdCfgValset;
  buf[4] = payload_len & 0xFF;
  buf[5] = (payload_len >> 8) & 0xFF;

  buf[6] = kValsetVersion;
  buf[7] = std::to_underlying(layer);
  buf[8] = 0;
  buf[9] = 0;

  buf[10] = key & 0xFF;
  buf[11] = (key >> 8) & 0xFF;
  buf[12] = (key >> 16) & 0xFF;
  buf[13] = (key >> 24) & 0xFF;

  std::memcpy(&buf[14], &value, sizeof(T));

  uint8_t ck_a = 0;
  uint8_t ck_b = 0;
  UbxChecksum(&buf[2], 4 + payload_len, ck_a, ck_b);
  buf[packet_len - 2] = ck_a;
  buf[packet_len - 1] = ck_b;

  Uart<UartInstance::kUart2>::GetInstance().Send(buf, packet_len);
}

void M10::SendCfgValGet(uint32_t key, ValgetLayer layer) {
  constexpr uint8_t version = 0x00;
  constexpr uint16_t position = 0;
  constexpr uint16_t payload_len = 4 + 4;
  constexpr size_t packet_len = 6 + payload_len + 2;

  uint8_t buf[packet_len];

  buf[0] = UBX::kSync1;
  buf[1] = UBX::kSync2;
  buf[2] = UBX::kClsCfg;
  buf[3] = UBX::kIdCfgValget;
  buf[4] = payload_len & 0xFF;
  buf[5] = (payload_len >> 8) & 0xFF;

  buf[6] = version;
  buf[7] = std::to_underlying(layer);
  buf[8] = position & 0xFF;
  buf[9] = (position >> 8) & 0xFF;

  buf[10] = key & 0xFF;
  buf[11] = (key >> 8) & 0xFF;
  buf[12] = (key >> 16) & 0xFF;
  buf[13] = (key >> 24) & 0xFF;

  uint8_t ck_a = 0;
  uint8_t ck_b = 0;
  UbxChecksum(&buf[2], 4 + payload_len, ck_a, ck_b);
  buf[packet_len - 2] = ck_a;
  buf[packet_len - 1] = ck_b;

  Uart<UartInstance::kUart2>::GetInstance().Send(buf, packet_len);
}

template <typename T>
bool M10::WaitForValget(uint32_t key, T expected_value) {
  static_assert(std::is_integral_v<T> || std::is_enum_v<T>);
  static_assert(sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4);

  auto &uart = Uart<UartInstance::kUart2>::GetInstance();
  auto &time = System::GetInstance().Time();
  const uint32_t start = time.Micros();

  uint8_t frame[64];
  size_t idx = 0;
  uint16_t payload_len = 0;
  size_t frame_len = 0;

  while ((uint32_t)(time.Micros() - start) < config_.ack_timeout_us) {
    uint8_t b;
    if (!uart.ReadByte(b)) continue;

    if (idx == 0 && b != UBX::kSync1) continue;
    if (idx == 1 && b != UBX::kSync2) {
      idx = 0;
      continue;
    }

    if (idx < sizeof(frame)) frame[idx] = b;
    idx++;

    if (idx == 6) {
      payload_len = (uint16_t)frame[4] | ((uint16_t)frame[5] << 8);
      frame_len = 6 + payload_len + 2;
      if (frame_len > sizeof(frame)) {
        idx = 0;
        frame_len = 0;
        continue;
      }
    }

    if (frame_len == 0 || idx < frame_len) continue;

    idx = 0;
    frame_len = 0;

    if (frame[2] != UBX::kClsCfg || frame[3] != UBX::kIdCfgValget) continue;
    if (payload_len != 4 + 4 + sizeof(T)) continue;

    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    UbxChecksum(&frame[2], 4 + payload_len, ck_a, ck_b);
    if (ck_a != frame[6 + payload_len] || ck_b != frame[6 + payload_len + 1])
      continue;

    const uint32_t resp_key = (uint32_t)frame[10] | ((uint32_t)frame[11] << 8) |
                              ((uint32_t)frame[12] << 16) |
                              ((uint32_t)frame[13] << 24);
    if (resp_key != key) continue;

    T resp_value{};
    std::memcpy(&resp_value, &frame[14], sizeof(T));
    return resp_value == expected_value;
  }

  return false;
}

template <typename T>
bool M10::SendCfgValSet(uint32_t key, T value, ValsetLayer layer) {
  auto &uart = Uart<UartInstance::kUart2>::GetInstance();

  SendCfgValSetRaw(key, value, layer);
  if (!WaitForAck(UBX::kClsCfg, UBX::kIdCfgValset)) return false;

  uart.FlushRx();
  SendCfgValGet(key, ValgetLayer::kRam);
  return WaitForValget<T>(key, value);
}
