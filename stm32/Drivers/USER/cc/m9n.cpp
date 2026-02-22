#include "m9n.hpp"
#include "m9n_reg.hpp"

#include "panic.hpp"
#include "system.hpp"
#include "time_base.hpp"
#include "uart.hpp"

#include <cstring>
#include <type_traits>

inline void UbxChecksum(const uint8_t *data, size_t len, uint8_t &ck_a,
                        uint8_t &ck_b) {
  ck_a = 0;
  ck_b = 0;
  for (size_t i = 0; i < len; i++) {
    ck_a = static_cast<uint8_t>(ck_a + data[i]);
    ck_b = static_cast<uint8_t>(ck_b + ck_a);
  }
}

void M9N::WaitForReady() {
  auto &uart = Uart<UartInstance::kUart2>::GetInstance();
  auto &time = System::GetInstance().Time();
  const uint32_t kStart = time.Micros();

  while ((uint32_t)(time.Micros() - kStart) < MILLIS_TO_MICROS(1000)) {
    uart.FlushRx();

    // Any cheap VALSET that triggers an ACK is fine as "ready".
    SendCfgValSetRaw<uint8_t>(kKeyUart1OutprotUbx, 1, kValsetLayerRam);
    if (WaitForAck(UBX::kClsCfg, UBX::kIdCfgValset)) {
      return;
    }

    time.DelayMicros(MILLIS_TO_MICROS(50));
  }

  Panic(ErrorCode::kM9nNotResponding);
}

// Explicit instantiations
template bool M9N::SendCfgValSet<uint8_t>(uint32_t, uint8_t, uint8_t);
template bool M9N::SendCfgValSet<uint16_t>(uint32_t, uint16_t, uint8_t);
template bool M9N::SendCfgValSet<uint32_t>(uint32_t, uint32_t, uint8_t);

template void M9N::SendCfgValSetRaw<uint8_t>(uint32_t, uint8_t, uint8_t);
template void M9N::SendCfgValSetRaw<uint16_t>(uint32_t, uint16_t, uint8_t);
template void M9N::SendCfgValSetRaw<uint32_t>(uint32_t, uint32_t, uint8_t);

template bool M9N::WaitForValget<uint8_t>(uint32_t, uint8_t);
template bool M9N::WaitForValget<uint16_t>(uint32_t, uint16_t);
template bool M9N::WaitForValget<uint32_t>(uint32_t, uint32_t);

void M9N::ApplyConfig(uint8_t layer) {
  if (!SendCfgValSet(kKeyUart1OutprotUbx,
                     static_cast<uint8_t>(config_.protocols.outprot_ubx),
                     layer))
    Panic(ErrorCode::kM9nVerifyProtocolFailed);
  if (!SendCfgValSet(kKeyUart1OutprotNmea,
                     static_cast<uint8_t>(config_.protocols.outprot_nmea),
                     layer))
    Panic(ErrorCode::kM9nVerifyProtocolFailed);
  if (!SendCfgValSet(kKeyMsgoutNavPvtUart1,
                     static_cast<uint8_t>(config_.messages.nav_pvt), layer))
    Panic(ErrorCode::kM9nVerifyNavPvtFailed);
  if (!SendCfgValSet(kKeyMsgoutNavDopUart1,
                     static_cast<uint8_t>(config_.messages.nav_dop), layer))
    Panic(ErrorCode::kM9nVerifyNavDopFailed);
  if (!SendCfgValSet(kKeyMsgoutNavCovUart1,
                     static_cast<uint8_t>(config_.messages.nav_cov), layer))
    Panic(ErrorCode::kM9nVerifyNavCovFailed);
  if (!SendCfgValSet(kKeyMsgoutNavEoeUart1,
                     static_cast<uint8_t>(config_.messages.nav_eoe), layer))
    Panic(ErrorCode::kM9nVerifyNavEoeFailed);
  if (!SendCfgValSet(kKeyCfgRateMeasMs, config_.nav.rate_meas_ms, layer))
    Panic(ErrorCode::kM9nVerifyRateFailed);
  if (!SendCfgValSet(kKeyCfgDynModel, config_.nav.dyn_model, layer))
    Panic(ErrorCode::kM9nVerifyDynModelFailed);
  if (!SendCfgValSet(kKeyGpsEnable,
                     static_cast<uint8_t>(config_.gnss.gps_enable), layer))
    Panic(ErrorCode::kM9nVerifyConstellationFailed);
  if (!SendCfgValSet(kKeyGloEnable,
                     static_cast<uint8_t>(config_.gnss.glo_enable), layer))
    Panic(ErrorCode::kM9nVerifyConstellationFailed);
  if (!SendCfgValSet(kKeyGalEnable,
                     static_cast<uint8_t>(config_.gnss.gal_enable), layer))
    Panic(ErrorCode::kM9nVerifyConstellationFailed);
  if (!SendCfgValSet(kKeyBdsEnable,
                     static_cast<uint8_t>(config_.gnss.bds_enable), layer))
    Panic(ErrorCode::kM9nVerifyConstellationFailed);
  if (!SendCfgValSet(kKeySbasEnable,
                     static_cast<uint8_t>(config_.gnss.sbas_enable), layer))
    Panic(ErrorCode::kM9nVerifyConstellationFailed);
  if (!SendCfgValSet(kKeyItfmEnable,
                     static_cast<uint8_t>(config_.gnss.itfm_enable), layer))
    Panic(ErrorCode::kM9nVerifyItfmFailed);

  // Timepulse Config (Atomic - grouped)
  // Payload: Key(4) + Val(var)
  // Keys:
  // - TP1_ENA (U1)
  // - TP1_PERIOD (U4)
  // - TP1_LEN (U4)
  // - TP1_TIMEGRID (U1)
  // - TP1_SYNC_GNSS (U1)
  // - TP1_USE_LOCKED (U1)
  // - TP1_ALIGN_TO_TOW (U1)
  // - TP1_POL (U1)
  // - TP1_PERIOD_LOCK (U4)
  // - TP1_LEN_LOCK (U4)
  constexpr uint16_t kPayloadLen = (4 + 1) + (4 + 4) + (4 + 4) + (4 + 1) +
                                   (4 + 1) + (4 + 1) + (4 + 1) + (4 + 1) +
                                   (4 + 4) + (4 + 4);
  constexpr size_t kPacketLen = 6 + kPayloadLen + 2;
  uint8_t buf[kPacketLen];

  // Header
  buf[0] = UBX::kSync1;
  buf[1] = UBX::kSync2;
  buf[2] = UBX::kClsCfg;
  buf[3] = UBX::kIdCfgValset;
  buf[4] = kPayloadLen & 0xFF;
  buf[5] = (kPayloadLen >> 8) & 0xFF;

  // Header payload: Version, Layer, Reserved
  buf[6] = kValsetVersion;
  buf[7] = layer;
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
  write_key_u1(kKeyCfgTp1TimeGrid, config_.tp1.timegrid);
  write_key_u1(kKeyCfgTp1SyncGnss, static_cast<uint8_t>(config_.tp1.sync_gnss));
  write_key_u1(kKeyCfgTp1UseLocked,
               static_cast<uint8_t>(config_.tp1.use_locked));
  write_key_u1(kKeyCfgTp1AlignToTow,
               static_cast<uint8_t>(config_.tp1.align_to_tow));
  write_key_u1(kKeyCfgTp1Pol, static_cast<uint8_t>(config_.tp1.pol_rising));
  write_key_u4(kKeyCfgTp1PeriodLock, config_.tp1.period_lock);
  write_key_u4(kKeyCfgTp1LenLock, config_.tp1.len_lock);

  // Safety check: ensure we filled exactly the expected payload
  if (idx != 10 + kPayloadLen) {
    Panic(ErrorCode::kM9nConfigTimepulseBufferError);
  }

  // Checksum
  uint8_t ck_a = 0, ck_b = 0;
  UbxChecksum(&buf[2], 4 + kPayloadLen, ck_a, ck_b);
  buf[kPacketLen - 2] = ck_a;
  buf[kPacketLen - 1] = ck_b;

  // Send
  Uart<UartInstance::kUart2>::GetInstance().Send(buf, kPacketLen);
  // Wait for ACK
  if (!WaitForAck(UBX::kClsCfg, UBX::kIdCfgValset)) {
    Panic(ErrorCode::kM9nConfigTimepulseFailed);
  }
}

void M9N::FlashConfig(uint32_t current_baud) {
  auto &uart = Uart<UartInstance::kUart2>::GetInstance();
  auto &time = System::GetInstance().Time();

  uart.SetBaudRate(current_baud);

  time.DelayMicros(MILLIS_TO_MICROS(100));
  uart.FlushRx();

  ApplyConfig(kValsetLayerAll);

  // Baud change last, no ACK wait
  SendCfgValSetRaw<uint32_t>(kKeyUart1Baudrate, config_.baud_rate,
                             kValsetLayerAll);

  time.DelayMicros(MILLIS_TO_MICROS(200)); // settle time
  uart.SetBaudRate(config_.baud_rate);
  uart.FlushRx();
}

void M9N::Init(const Config &config) {
  config_ = config;

  // Wait up to 1s for GPS to boot and respond
  WaitForReady();

  if (config_.flash_config)
    FlashConfig(38400);

  auto &uart = Uart<UartInstance::kUart2>::GetInstance();
  uart.FlushRx();

  ApplyConfig(kValsetLayerRam);
}

bool M9N::Read(uint8_t &b) {
  return Uart<UartInstance::kUart2>::GetInstance().Read(b);
}

bool M9N::WaitForAck(uint8_t want_cls, uint8_t want_id) {
  auto &uart = Uart<UartInstance::kUart2>::GetInstance();
  auto &time = System::GetInstance().Time();
  const uint32_t kStart = time.Micros();

  uint8_t frame[10];
  uint8_t idx = 0;

  while ((uint32_t)(time.Micros() - kStart) < config_.ack_timeout_us) {
    uint8_t b;
    if (!uart.Read(b))
      continue;

    if (idx == 0) {
      if (b != UBX::kSync1)
        continue;
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
    if (idx < sizeof(frame))
      continue;
    idx = 0;

    if (frame[2] != UBX::kClsAck)
      continue;

    const uint8_t kId = frame[3];
    if (kId != UBX::kIdAckAck && kId != UBX::kIdAckNak)
      continue;

    if (frame[4] != 0x02 || frame[5] != 0x00)
      continue;

    uint8_t ck_a = 0, ck_b = 0;
    uint8_t chk_buf[6] = {frame[2], frame[3], frame[4],
                          frame[5], frame[6], frame[7]};
    UbxChecksum(chk_buf, sizeof(chk_buf), ck_a, ck_b);
    if (ck_a != frame[8] || ck_b != frame[9])
      continue;

    if (frame[6] == want_cls && frame[7] == want_id) {
      return kId == UBX::kIdAckAck;
    }
  }

  return false;
}

template <typename T>
void M9N::SendCfgValSetRaw(uint32_t key, T value, uint8_t layer) {
  static_assert(std::is_integral_v<T> || std::is_enum_v<T>);
  static_assert(sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4);

  constexpr uint16_t kPayloadLen = 4 + 4 + sizeof(T);
  constexpr size_t kPacketLen = 6 + kPayloadLen + 2;

  uint8_t buf[kPacketLen];

  buf[0] = UBX::kSync1;
  buf[1] = UBX::kSync2;
  buf[2] = UBX::kClsCfg;
  buf[3] = UBX::kIdCfgValset;
  buf[4] = kPayloadLen & 0xFF;
  buf[5] = (kPayloadLen >> 8) & 0xFF;

  buf[6] = kValsetVersion;
  buf[7] = layer;
  buf[8] = 0;
  buf[9] = 0;

  buf[10] = key & 0xFF;
  buf[11] = (key >> 8) & 0xFF;
  buf[12] = (key >> 16) & 0xFF;
  buf[13] = (key >> 24) & 0xFF;

  std::memcpy(&buf[14], &value, sizeof(T));

  uint8_t ck_a = 0, ck_b = 0;
  UbxChecksum(&buf[2], 4 + kPayloadLen, ck_a, ck_b);
  buf[kPacketLen - 2] = ck_a;
  buf[kPacketLen - 1] = ck_b;

  Uart<UartInstance::kUart2>::GetInstance().Send(buf, kPacketLen);
}

void M9N::SendCfgValGet(uint32_t key, uint8_t layer) {
  constexpr uint8_t kVersion = 0x00;
  constexpr uint16_t kPosition = 0;
  constexpr uint16_t kPayloadLen = 4 + 4; // version/layer/position + 1 key
  constexpr size_t kPacketLen = 6 + kPayloadLen + 2;

  uint8_t buf[kPacketLen];

  buf[0] = UBX::kSync1;
  buf[1] = UBX::kSync2;
  buf[2] = UBX::kClsCfg;
  buf[3] = UBX::kIdCfgValget;
  buf[4] = kPayloadLen & 0xFF;
  buf[5] = (kPayloadLen >> 8) & 0xFF;

  buf[6] = kVersion;
  buf[7] = layer;
  buf[8] = kPosition & 0xFF;
  buf[9] = (kPosition >> 8) & 0xFF;

  buf[10] = key & 0xFF;
  buf[11] = (key >> 8) & 0xFF;
  buf[12] = (key >> 16) & 0xFF;
  buf[13] = (key >> 24) & 0xFF;

  uint8_t ck_a = 0, ck_b = 0;
  UbxChecksum(&buf[2], 4 + kPayloadLen, ck_a, ck_b);
  buf[kPacketLen - 2] = ck_a;
  buf[kPacketLen - 1] = ck_b;

  Uart<UartInstance::kUart2>::GetInstance().Send(buf, kPacketLen);
}

template <typename T> bool M9N::WaitForValget(uint32_t key, T expected_value) {
  static_assert(std::is_integral_v<T> || std::is_enum_v<T>);
  static_assert(sizeof(T) == 1 || sizeof(T) == 2 || sizeof(T) == 4);

  auto &uart = Uart<UartInstance::kUart2>::GetInstance();
  auto &time = System::GetInstance().Time();
  const uint32_t kStart = time.Micros();

  uint8_t frame[64];
  size_t idx = 0;
  uint16_t payload_len = 0;
  size_t frame_len = 0;

  while ((uint32_t)(time.Micros() - kStart) < config_.ack_timeout_us) {
    uint8_t b;
    if (!uart.Read(b))
      continue;

    if (idx == 0 && b != UBX::kSync1)
      continue;
    if (idx == 1 && b != UBX::kSync2) {
      idx = 0;
      continue;
    }

    if (idx < sizeof(frame))
      frame[idx] = b;
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

    if (frame_len == 0 || idx < frame_len)
      continue;

    idx = 0;
    frame_len = 0;

    if (frame[2] != UBX::kClsCfg || frame[3] != UBX::kIdCfgValget)
      continue;

    // expect: version+layer+position (4) + key (4) + value
    if (payload_len != 4 + 4 + sizeof(T))
      continue;

    uint8_t ck_a = 0, ck_b = 0;
    UbxChecksum(&frame[2], 4 + payload_len, ck_a, ck_b);
    if (ck_a != frame[6 + payload_len] || ck_b != frame[6 + payload_len + 1])
      continue;

    const uint32_t kRespKey = (uint32_t)frame[10] | ((uint32_t)frame[11] << 8) |
                              ((uint32_t)frame[12] << 16) |
                              ((uint32_t)frame[13] << 24);
    if (kRespKey != key)
      continue;

    T resp_value{};
    std::memcpy(&resp_value, &frame[14], sizeof(T));
    return resp_value == expected_value;
  }

  return false;
}

template <typename T>
bool M9N::SendCfgValSet(uint32_t key, T value, uint8_t layer) {
  auto &uart = Uart<UartInstance::kUart2>::GetInstance();

  SendCfgValSetRaw(key, value, layer);
  if (!WaitForAck(UBX::kClsCfg, UBX::kIdCfgValset))
    return false;

  // Verify against active config (RAM) to keep it deterministic.
  uart.FlushRx();
  SendCfgValGet(key, kValgetLayerRam);
  return WaitForValget<T>(key, value);
}