#include "states.hpp"
#include "board.h"
#include "spi.hpp"
#include <cstdio>
#include <cstring>

// --- Protocol Handlers ---

static void OnPing(AppContext &ctx, const message::Packet &pkt) {
  (void)pkt;
  // Send Pong
  uint8_t tx_buf[32];
  size_t tx_len = message::Serialize(message::MsgId::kPong, nullptr, 0, tx_buf);
  ctx.sys->GetUart().Send(tx_buf, tx_len);
}

static void OnRcChannels(AppContext &ctx, const message::Packet &pkt) {
  (void)ctx;
  (void)pkt;
  // TODO: Handle RC Channels
  // For now, maybe toggle LED to show activity?
  // ctx.sys->Led().Toggle();
}

// --- Dispatch Table ---

static const Epistole::Dispatcher<AppContext>::Entry kHandlers[] = {
    {message::MsgId::kPing, OnPing},
    {message::MsgId::kRcChannels, OnRcChannels},
};

static const Epistole::Dispatcher<AppContext>
    kDispatcher(kHandlers, sizeof(kHandlers) / sizeof(kHandlers[0]));

// --- IdleState (Main State) ---

void IdleState::OnEnter(AppContext &ctx, SmTick now) {
  (void)now;
  ctx.sys->Led().Set(false);
  rx_state_ = RxState::kMagic1;
}

void IdleState::OnStep(AppContext &ctx, SmTick now) {
  auto &uart = ctx.sys->GetUart(); // Console

  // --- Process GPS (UART2) ---
  auto &gps_uart = ctx.sys->GetUart2();
  auto &gps_svc = ctx.sys->ServiceM9N();
  uint8_t c;
  while (gps_uart.Read(c)) {
    gps_svc.ProcessByte(c);
    if (gps_svc.NewDataAvailable()) {
      const auto &pvt = gps_svc.GetData();

      // Temporary: Send GPS RTC to ESP32
      message::GpsData t;
      t.year = pvt.year;
      t.month = pvt.month;
      t.day = pvt.day;
      t.hour = pvt.hour;
      t.min = pvt.min;
      t.sec = pvt.sec;
      t.valid = pvt.valid;
      t.tAcc = pvt.tAcc;
      t.lon = pvt.lon;
      t.lat = pvt.lat;
      t.height = pvt.hMSL;
      t.fixType = pvt.fixType;
      t.numSV = pvt.numSV;

      uint8_t tx_buf[64];
      size_t tx_len = message::Serialize(message::MsgId::kGpsData,
                                         (uint8_t *)&t, sizeof(t), tx_buf);
      uart.Send(tx_buf, tx_len);

      gps_svc.ClearNewDataFlag();
    }
  }

  // Process Console UART logic (Protocol Parser)
  while (uart.Read(c)) {
    switch (rx_state_) {
    case RxState::kMagic1:
      if (c == message::kMagic1)
        rx_state_ = RxState::kMagic2;
      break;
    case RxState::kMagic2:
      if (c == message::kMagic2)
        rx_state_ = RxState::kId;
      else
        rx_state_ = RxState::kMagic1;
      break;
    case RxState::kId:
      rx_pkt_internal_.id = c;
      rx_state_ = RxState::kLen;
      break;
    case RxState::kLen:
      rx_len_ = c;
      rx_idx_ = 0;
      rx_state_ = (rx_len_ > 0) ? RxState::kPayload : RxState::kCrc1;
      break;
    case RxState::kPayload:
      rx_pkt_internal_.payload[rx_idx_++] = c;
      if (rx_idx_ >= rx_len_)
        rx_state_ = RxState::kCrc1;
      break;
    case RxState::kCrc1:
      rx_pkt_internal_.crc = c;
      rx_state_ = RxState::kCrc2;
      break;
    case RxState::kCrc2:
      rx_pkt_internal_.crc |= ((uint16_t)c << 8);

      // Verify CRC
      {
        uint8_t check_buf[sizeof(message::Header) + 256];
        message::Header *h = (message::Header *)check_buf;
        h->magic[0] = message::kMagic1;
        h->magic[1] = message::kMagic2;
        h->id = rx_pkt_internal_.id;
        h->len = rx_len_;
        if (rx_len_ > 0)
          memcpy(check_buf + sizeof(message::Header), rx_pkt_internal_.payload,
                 rx_len_);

        if (message::Crc16(check_buf, sizeof(message::Header) + rx_len_) ==
            rx_pkt_internal_.crc) {

          // Construct Packet
          message::Packet pkt;
          pkt.header.id = rx_pkt_internal_.id;
          pkt.header.len = rx_len_;
          pkt.crc = rx_pkt_internal_.crc;
          if (rx_len_ > 0) {
            memcpy(pkt.payload, rx_pkt_internal_.payload, rx_len_);
          }

          // Dispatch
          kDispatcher.Dispatch(ctx, pkt);
        }
      }
      rx_state_ = RxState::kMagic1;
      break;
    }
  }
}

void IdleState::OnExit(AppContext &ctx, SmTick now) {
  (void)ctx;
  (void)now;
}

// --- NotIdleState (Unused) ---

void NotIdleState::OnEnter(AppContext &ctx, SmTick now) {
  (void)ctx;
  (void)now;
}
void NotIdleState::OnStep(AppContext &ctx, SmTick now) {
  (void)ctx;
  (void)now;
}
void NotIdleState::OnExit(AppContext &ctx, SmTick now) {
  (void)ctx;
  (void)now;
}
