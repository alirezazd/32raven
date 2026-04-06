#include "fc_link.hpp"

#include <cstring>

#include "esp32_config.hpp"
#include "mavlink.hpp"
#include "panic.hpp"
#include "ring_buffer.hpp"
#include "system.hpp"

extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

static constexpr const char *kTag = "FC_Link";
static RingBuffer<message::Packet, esp32_limits::kFcLinkRxPacketQueueDepth + 1>
    g_packet_queue;

void FcLink::Init(const Config &cfg, UartFcLink *uart) {
  if (uart == nullptr) {
    Panic(ErrorCode::kFcLinkInitFailed);
  }
  cfg_ = cfg;
  if (cfg_.rc_forward_rate_hz == 0 || cfg_.handshake_attempts == 0 ||
      cfg_.handshake_retry_period_ms == 0) {
    Panic(ErrorCode::kFcLinkInitFailed);
  }
  uart_ = uart;
  g_packet_queue.Clear();
  next_rc_forward_ms_ = 0;
  ESP_LOGI(kTag, "Initialized");
}

void FcLink::PerformHandshake() {
  next_rc_forward_ms_ = 0;
  g_packet_queue.Clear();
  ESP_LOGI(kTag, "Handshake Start...");

  uint8_t tx_buf[message::kPacketOverhead];
  size_t tx_len = message::Serialize(message::MsgId::kPing, nullptr, 0, tx_buf);

  for (uint16_t i = 0; i < cfg_.handshake_attempts; ++i) {
    uart_->Write(tx_buf, tx_len);
    vTaskDelay(pdMS_TO_TICKS(cfg_.handshake_retry_period_ms));

    Poll();

    if (auto response = PopPacket()) {
      if (response->header.id == (uint8_t)message::MsgId::kPong) {
        ESP_LOGI(kTag, "Handshake Success!");
        return;
      }
    }
  }

  ESP_LOGW(kTag, "Handshake Failed");
  Panic(ErrorCode::kFcLinkHandshakeFailed);
}

bool FcLink::SendPacket(const message::Packet &pkt) {
  uint8_t tx[message::kMaxPayload +
             message::kPacketOverhead];  // Max payload + header + crc
  size_t len = message::Serialize((message::MsgId)pkt.header.id, pkt.payload,
                                  pkt.header.len, tx);
  if (len == 0) {
    return false;
  }
  uart_->Write(tx, len);
  return true;
}

void FcLink::ForwardRcState(const RcState &rc_state) {
  const TimeMs now = Sys().Timebase().NowMs();
  if (!TimeReached(now, next_rc_forward_ms_)) {
    return;
  }

  message::RcChannelsMsg msg{};
  std::memcpy(msg.channels, rc_state.channels, sizeof(msg.channels));
  msg.rssi = rc_state.rssi;

  message::Packet pkt{};
  pkt.header.id = (uint8_t)message::MsgId::kRcChannels;
  pkt.header.len = message::PayloadLength<message::RcChannelsMsg>();
  std::memcpy(pkt.payload, &msg, sizeof(msg));

  if (!SendPacket(pkt)) {
    return;
  }

  next_rc_forward_ms_ =
      TimeAfter(now, static_cast<TimeMs>((1000u + cfg_.rc_forward_rate_hz - 1) /
                                         cfg_.rc_forward_rate_hz));
}

std::optional<message::Packet> FcLink::PopPacket() {
  message::Packet packet;
  if (!g_packet_queue.Pop(packet)) {
    return std::nullopt;
  }
  return packet;
}

bool FcLink::FinishRxPacket() {
  if (!message::IsPacketValid(rx_pkt_internal_.id, rx_pkt_internal_.payload,
                              rx_len_)) {
    return false;
  }

  uint8_t check_buf[sizeof(message::Header) + message::kMaxPayload];
  message::Header *header = (message::Header *)check_buf;
  header->magic[0] = message::kMagic1;
  header->magic[1] = message::kMagic2;
  header->id = rx_pkt_internal_.id;
  header->len = rx_len_;
  if (rx_len_ > 0) {
    memcpy(check_buf + sizeof(message::Header), rx_pkt_internal_.payload,
           rx_len_);
  }

  if (message::Crc16(check_buf, sizeof(message::Header) + rx_len_) !=
      rx_pkt_internal_.crc) {
    return false;
  }

  message::Packet packet{};
  packet.header.id = rx_pkt_internal_.id;
  packet.header.len = rx_len_;
  if (rx_len_ > 0) {
    memcpy(packet.payload, rx_pkt_internal_.payload, rx_len_);
  }
  packet.crc = rx_pkt_internal_.crc;

  // TODO: Handle RX queue overflow explicitly instead of silently dropping
  // parsed FcLink packets here.
  (void)g_packet_queue.Push(packet);
  return true;
}

void FcLink::Poll() {
  const size_t pending_bytes = uart_->BufferedRxBytes();
  if (pending_bytes == 0) {
    return;
  }

  uint8_t buf[kMaxRxReadBufferSize];
  const size_t read_size =
      (pending_bytes < sizeof(buf)) ? pending_bytes : sizeof(buf);
  int n = uart_->Read(buf, read_size);

  for (int i = 0; i < n; ++i) {
    uint8_t b = buf[i];
    switch (rx_state_) {
      case RxState::kMagic1:
        if (b == message::kMagic1) rx_state_ = RxState::kMagic2;
        break;
      case RxState::kMagic2:
        if (b == message::kMagic2)
          rx_state_ = RxState::kId;
        else
          rx_state_ = RxState::kMagic1;
        break;
      case RxState::kId:
        rx_pkt_internal_.id = b;
        rx_state_ = RxState::kLen;
        break;
      case RxState::kLen:
        rx_len_ = b;
        rx_idx_ = 0;
        if (!message::IsPayloadLengthValid(
                static_cast<message::MsgId>(rx_pkt_internal_.id), rx_len_)) {
          rx_state_ = RxState::kMagic1;
          break;
        }
        rx_state_ = (rx_len_ > 0) ? RxState::kPayload : RxState::kCrc1;
        break;
      case RxState::kPayload:
        rx_pkt_internal_.payload[rx_idx_++] = b;
        if (rx_idx_ >= rx_len_) rx_state_ = RxState::kCrc1;
        break;
      case RxState::kCrc1:
        rx_pkt_internal_.crc = b;
        rx_state_ = RxState::kCrc2;
        break;
      case RxState::kCrc2:
        rx_pkt_internal_.crc |= ((uint16_t)b << 8);
        if (!FinishRxPacket()) {
          Sys().TonePlayer().PlayBuiltin(::TonePlayer::BuiltinTone::kError);
          ESP_LOGE(kTag, "Invalid FC packet id=0x%02X len=%u",
                   (unsigned)rx_pkt_internal_.id, (unsigned)rx_len_);
        }
        rx_state_ = RxState::kMagic1;
        break;
    }
  }
}
