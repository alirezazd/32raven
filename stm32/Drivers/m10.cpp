// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "m10.hpp"

#include <cstring>
#include <optional>
#include <span>
#include <type_traits>
#include <utility>

#include "error_code.hpp"
#include "m10_reg.hpp"
#include "panic.hpp"
#include "system.hpp"
#include "time_base.hpp"
#include "uart.hpp"

struct UbxChecksum {
  uint8_t ck_a;
  uint8_t ck_b;
};

inline UbxChecksum ComputeUbxChecksum(const uint8_t *data, size_t len) {
  uint8_t ck_a = 0;
  uint8_t ck_b = 0;
  for (size_t i = 0; i < len; i++) {
    ck_a = static_cast<uint8_t>(ck_a + data[i]);
    ck_b = static_cast<uint8_t>(ck_b + ck_a);
  }
}

// One CFG-VALSET frame carrying several keys, for a group whose members have
// to move together: the receiver applies and acknowledges the whole frame
// once, where per-key writes apply one at a time. Sized for the largest group
// below rather than for the protocol's 64-key ceiling.
class ValsetFrame {
 public:
  static constexpr size_t kMaxPayload = 4 + (7 * (4 + 4));

  explicit ValsetFrame(uint8_t layer) {
    buf_[6] = kValsetVersion;
    buf_[7] = layer;
  }

  void AddU1(uint32_t key, uint8_t value) {
    Reserve(4 + 1);
    AddKey(key);
    buf_[idx_++] = value;
  }

  void AddU4(uint32_t key, uint32_t value) {
    Reserve(4 + 4);
    AddKey(key);
    std::memcpy(&buf_[idx_], &value, 4);
    idx_ += 4;
  }

  // The frame as bytes, header and checksum filled in. Valid until the next
  // Add; the caller sends it.
  std::span<const uint8_t> Finish() {
    const auto payload_len = static_cast<uint16_t>(idx_ - 6);
    buf_[0] = UBX::kSync1;
    buf_[1] = UBX::kSync2;
    buf_[2] = UBX::kClsCfg;
    buf_[3] = UBX::kIdCfgValset;
    buf_[4] = payload_len & 0xFF;
    buf_[5] = (payload_len >> 8) & 0xFF;
    const UbxChecksum ck = ComputeUbxChecksum(&buf_[2], 4 + payload_len);
    buf_[idx_] = ck.ck_a;
    buf_[idx_ + 1] = ck.ck_b;
    return {buf_, idx_ + 2};
  }

 private:
  // The buffer is sized for the largest group in this file, so overflow means
  // a group grew without the size following: caught here rather than left to
  // run past the frame.
  void Reserve(size_t bytes) {
    if (idx_ + bytes + 2 > sizeof(buf_)) {
      Panic(ErrorCode::Stm32::kGpsValsetFrameOverflow);
    }
  }

  void AddKey(uint32_t key) {
    buf_[idx_++] = key & 0xFF;
    buf_[idx_++] = (key >> 8) & 0xFF;
    buf_[idx_++] = (key >> 16) & 0xFF;
    buf_[idx_++] = (key >> 24) & 0xFF;
  }

  uint8_t buf_[6 + kMaxPayload + 2] = {};
  size_t idx_ = 10;  // past sync, class, id, length and the valset header
};

M10 &M10::GetInstance() {
  static M10 instance;
  return instance;
}

void M10::WaitForReady() {
  auto &uart = (*uart_);
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

  // One transaction for the whole CFG-SIGNAL group: every change to it resets
  // the GNSS subsystem, and the interface description wants the ACK plus 0.5 s
  // of settling before the next command -- per-key writes stacked five resets
  // with no settling at all. The read-back sits after the wait, where the
  // subsystem can answer for what it now runs.
  {
    const struct {
      uint32_t key;
      uint8_t value;
    } signals[] = {
        {kKeyGpsEnable, u8(config_.gnss.gps_enable)},
        {kKeyGloEnable, u8(config_.gnss.glo_enable)},
        {kKeyGalEnable, u8(config_.gnss.gal_enable)},
        {kKeyBdsEnable, u8(config_.gnss.bds_enable)},
        {kKeySbasEnable, u8(config_.gnss.sbas_enable)},
    };

    ValsetFrame frame(std::to_underlying(layer));
    for (const auto &signal : signals) {
      frame.AddU1(signal.key, signal.value);
    }
    const std::span<const uint8_t> bytes = frame.Finish();
    (*uart_).Send(bytes.data(), bytes.size());
    if (!WaitForAck(UBX::kClsCfg, UBX::kIdCfgValset)) {
      Panic(ErrorCode::Stm32::kGpsVerifyConstellationFailed);
    }

    System::GetInstance().Time().DelayMicros(MillisToMicros(500));

    for (const auto &signal : signals) {
      (*uart_).FlushRx();
      SendCfgValGet(signal.key, ValgetLayer::kRam);
      if (!WaitForValget<uint8_t>(signal.key, signal.value)) {
        Panic(ErrorCode::Stm32::kGpsVerifyConstellationFailed);
      }
    }
  }

  set(kKeyItfmEnable, u8(config_.gnss.itfm_enable),
      ErrorCode::Stm32::kGpsVerifyItfmFailed);

  // One frame for the timepulse group as well: its seven keys describe a
  // single pulse, and a receiver that took only some of them would emit one
  // nothing configured.
  {
    ValsetFrame frame(std::to_underlying(layer));
    frame.AddU1(kKeyCfgTp1Ena, static_cast<uint8_t>(config_.tp1.ena));
    frame.AddU4(kKeyCfgTp1Period, config_.tp1.period);
    frame.AddU4(kKeyCfgTp1Len, config_.tp1.len);
    frame.AddU1(kKeyCfgTp1TimeGrid, static_cast<uint8_t>(config_.tp1.timegrid));
    frame.AddU1(kKeyCfgTp1SyncGnss,
                static_cast<uint8_t>(config_.tp1.sync_gnss));
    frame.AddU1(kKeyCfgTp1AlignToTow,
                static_cast<uint8_t>(config_.tp1.align_to_tow));
    frame.AddU1(kKeyCfgTp1Pol, static_cast<uint8_t>(config_.tp1.pol_rising));
    const std::span<const uint8_t> bytes = frame.Finish();
    (*uart_).Send(bytes.data(), bytes.size());
    if (!WaitForAck(UBX::kClsCfg, UBX::kIdCfgValset)) {
      Panic(ErrorCode::Stm32::kGpsConfigTimepulseFailed);
    }
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

void M10::Init(Uart2 &uart, const Config &config) {
  uart_ = &uart;
  config_ = config;

  WaitForReady();
  (*uart_).FlushRx();
  ApplyConfig(ValsetLayer::kRam);
}

bool M10::WaitForAck(uint8_t want_cls, uint8_t want_id) {
  auto &uart = (*uart_);
  auto &time = System::GetInstance().Time();
  const uint32_t start = time.Micros();

  uint8_t frame[10];
  uint8_t idx = 0;

  while ((uint32_t)(time.Micros() - start) < config_.ack_timeout_us) {
    const std::optional<uint8_t> byte = uart.ReadByte();
    if (!byte) continue;
    const uint8_t b = byte.value();

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

    const uint8_t chk_buf[6] = {frame[2], frame[3], frame[4],
                                frame[5], frame[6], frame[7]};
    const UbxChecksum ck = ComputeUbxChecksum(chk_buf, sizeof(chk_buf));
    if (ck.ck_a != frame[8] || ck.ck_b != frame[9]) continue;

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

  const UbxChecksum ck = ComputeUbxChecksum(&buf[2], 4 + payload_len);
  buf[packet_len - 2] = ck.ck_a;
  buf[packet_len - 1] = ck.ck_b;

  (*uart_).Send(buf, packet_len);
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

  const UbxChecksum ck = ComputeUbxChecksum(&buf[2], 4 + payload_len);
  buf[packet_len - 2] = ck.ck_a;
  buf[packet_len - 1] = ck.ck_b;

  (*uart_).Send(buf, packet_len);
}

template <typename T>
bool M10::WaitForValget(uint32_t key, T expected_value) {
  static_assert(std::is_integral_v<T> || std::is_enum_v<T>);
  static_assert(sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4);

  auto &uart = (*uart_);
  auto &time = System::GetInstance().Time();
  const uint32_t start = time.Micros();

  uint8_t frame[64];
  size_t idx = 0;
  uint16_t payload_len = 0;
  size_t frame_len = 0;

  while ((uint32_t)(time.Micros() - start) < config_.ack_timeout_us) {
    const std::optional<uint8_t> byte = uart.ReadByte();
    if (!byte) continue;
    const uint8_t b = byte.value();

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

    const UbxChecksum ck = ComputeUbxChecksum(&frame[2], 4 + payload_len);
    if (ck.ck_a != frame[6 + payload_len] ||
        ck.ck_b != frame[6 + payload_len + 1])
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
  auto &uart = (*uart_);

  SendCfgValSetRaw(key, value, layer);
  if (!WaitForAck(UBX::kClsCfg, UBX::kIdCfgValset)) return false;

  uart.FlushRx();
  SendCfgValGet(key, ValgetLayer::kRam);
  return WaitForValget<T>(key, value);
}
