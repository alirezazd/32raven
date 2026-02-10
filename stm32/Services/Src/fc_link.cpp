#include "fc_link.hpp"
#include "states.hpp" // For AppContext definition
#include "system.hpp"
#include "uart.hpp"
#include <cstdarg>
#include <cstdio>
#include <cstring>

void FcLink::Init(AppContext *ctx) { ctx_ = ctx; }

void FcLink::Poll() {
  if (!ctx_)
    return;

  auto &uart = ctx_->sys->GetUart1();

  // 1. RX Parsing
  uint8_t c;
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
          message::Packet pkt;
          pkt.header.id = rx_pkt_internal_.id;
          pkt.header.len = rx_len_;
          pkt.crc = rx_pkt_internal_.crc;
          if (rx_len_ > 0)
            memcpy(pkt.payload, rx_pkt_internal_.payload, rx_len_);

          // Dispatch
          CommandHandler::GetInstance().Dispatch(*ctx_, pkt);
        }
      }
      rx_state_ = RxState::kMagic1;
      break;
    }
  }

  // 2. TX Flushing (RingBuffer -> UART)
  // Send in chunks of e.g. 32 bytes to not hog DMA/UART if blocking
  uint8_t tx_chunk[64];
  size_t n = 0;
  while (tx_rb_.Pop(c)) {
    tx_chunk[n++] = c;
    if (n >= sizeof(tx_chunk))
      break;
  }
  if (n > 0) {
    uart.Send(tx_chunk, n);
  }
}

bool FcLink::Send(const message::Packet &pkt) {
  uint8_t buf[256 + sizeof(message::Header) + 2];
  // Serialize returns total length
  size_t len = message::Serialize((message::MsgId)pkt.header.id,
                                  (pkt.header.len > 0) ? pkt.payload : nullptr,
                                  pkt.header.len, buf);

  // Push to RingBuffer
  for (size_t i = 0; i < len; ++i) {
    if (!tx_rb_.Push(buf[i])) {
      return false; // Overflow
    }
  }
  return true;
}

void FcLink::SendGps(const GpsData &data, const BatteryData &bat) {
  message::GpsData t = {};
  // Map internal GpsData (blackboard) to Message GpsData
  // Warning: Ensure types match message definitions
  // message::GpsData in shared libs might differ from internal GpsData struct
  // Re-verification of libs/include/message.hpp might be needed if fields
  // differ significantly. Based on previous tasks, message::GpsData has: valid,
  // lat, lon, height, etc.

  t.fixType = data.fix_type;
  t.numSV = data.num_sats;
  t.hMSL = data.alt; // data.alt is in mm MSL
  t.vel = data.vel;  // cm/s
  t.hdg = data.hdg;  // cdeg
  t.lat = data.lat;
  t.lon = data.lon;

  t.year = data.year;
  t.month = data.month;
  t.day = data.day;
  t.hour = data.hour;
  t.min = data.min;
  t.sec = data.sec;

  t.hAcc = data.hAcc; // mm
  t.vAcc = data.vAcc; // mm

  // Populate new fields with dummy/placeholder data for now
  // Real attitude/battery data needs to come from `data` source
  t.roll = 0;
  t.pitch = 0;
  t.yaw = data.hdg; // Use GPS heading as yaw fallback

  t.batt_voltage = (uint16_t)(bat.voltage * 1000.0f); // V -> mV
  t.batt_current = (int16_t)(bat.current * 100.0f);   // A -> cA
  t.batt_remaining = (int8_t)bat.percentage;          // %

  message::Packet pkt;
  pkt.header.id = (uint8_t)message::MsgId::kGpsData;
  pkt.header.len = sizeof(t);
  memcpy(pkt.payload, &t, sizeof(t));

  Send(pkt);
}

void FcLink::SendLog(const char *format, ...) {
  char buf[128]; // Max payload size relative
  va_list args;
  va_start(args, format);
  int len = vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);

  if (len > 0) {
    if (len > 200)
      len = 200; // Cap to slightly less than max payload
    message::Packet pkt;
    pkt.header.id = (uint8_t)message::MsgId::kLog;
    pkt.header.len = (uint8_t)len;
    memcpy(pkt.payload, buf, len);
    Send(pkt);
  }
}

void FcLink::SendTimeSync(const message::TimeSyncMsg &msg) {
  message::Packet pkt;
  pkt.header.id = (uint8_t)message::MsgId::kTimeSync;
  pkt.header.len = sizeof(message::TimeSyncMsg);
  memcpy(pkt.payload, &msg, sizeof(message::TimeSyncMsg));
  Send(pkt);
}
