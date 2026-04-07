#include "fc_link.hpp"

#include <cstring>

#include "error_code.hpp"
#include "esp32_limits.hpp"
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
      cfg_.handshake_retry_period_ms == 0 ||
      cfg_.invalid_packet_threshold == 0) {
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

void FcLink::SendPacket(const message::Packet &pkt) {
  uint8_t tx[message::kMaxPayload +
             message::kPacketOverhead];  // Max payload + header + crc
  size_t len = message::Serialize((message::MsgId)pkt.header.id, pkt.payload,
                                  pkt.header.len, tx);
  if (!len) {
    ESP_LOGE(kTag, "Failed to serialize packet id=0x%02X len=%u",
             (unsigned)pkt.header.id, (unsigned)pkt.header.len);
    Panic(ErrorCode::kFcLinkTxSerializeFailed);
    return;
  }
  uart_->Write(tx, len);
}

void FcLink::ForwardRcState(const RcState &rc_state) {
  const TimeMs now = Sys().Timebase().NowMs();
  if (!TimeReached(now, next_rc_forward_ms_)) {
    return;
  }

  message::Packet pkt{};
  pkt.header.id = (uint8_t)message::MsgId::kRcChannels;
  pkt.header.len = message::PayloadLength<message::RcChannelsMsg>();
  for (size_t i = 0; i < std::size(rc_state.channels); ++i) {
    const uint16_t channel = rc_state.channels[i];
    const size_t offset = i * sizeof(uint16_t);
    pkt.payload[offset] = (uint8_t)(channel & 0xFFu);
    pkt.payload[offset + 1] = (uint8_t)(channel >> 8);
  }
  pkt.payload[sizeof(rc_state.channels)] = rc_state.rssi;
  SendPacket(pkt);
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

// Returns true if packet is valid and successfully pushed to queue. Caller
// should alert on false return.
void FcLink::FinishRxPacket() {
  const auto should_alert_invalid = [this](ErrorCode code) {
    ++invalid_packet_count_;
    if (cfg_.invalid_packet_threshold > 0 &&
        invalid_packet_count_ >= cfg_.invalid_packet_threshold) {
      Panic(code);
    }
    return true;
  };

  if (!message::IsPacketValid(rx_pkt_internal_.id, rx_pkt_internal_.payload,
                              rx_len_)) {
    if (should_alert_invalid(ErrorCode::kFcLinkInvalidPacketLength)) {
      ESP_LOGW(kTag, "Invalid payload for packet id=0x%02X len=%u",
               (unsigned)rx_pkt_internal_.id, (unsigned)rx_len_);
      Sys().TonePlayer().PlayBuiltin(::TonePlayer::BuiltinTone::kError);
    }
    return;
  }

  const auto crc16_update = [](uint16_t crc, uint8_t byte) {
    crc ^= (uint16_t)byte << 8;
    for (int j = 0; j < 8; ++j) {
      crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                            : (uint16_t)(crc << 1);
    }
    return crc;
  };

  uint16_t crc = 0;
  crc = crc16_update(crc, message::kMagic1);
  crc = crc16_update(crc, message::kMagic2);
  crc = crc16_update(crc, rx_pkt_internal_.id);
  crc = crc16_update(crc, rx_len_);
  for (uint8_t i = 0; i < rx_len_; ++i) {
    crc = crc16_update(crc, rx_pkt_internal_.payload[i]);
  }

  if (crc != rx_pkt_internal_.crc) {
    if (should_alert_invalid(ErrorCode::kFcLinkInvalidPacketCrc)) {
      ESP_LOGW(kTag, "Invalid CRC: calculated=0x%04X received=0x%04X", crc,
               rx_pkt_internal_.crc);
      Sys().TonePlayer().PlayBuiltin(::TonePlayer::BuiltinTone::kError);
    }
    return;
  }

  message::Packet packet{};
  packet.header.id = rx_pkt_internal_.id;
  packet.header.len = rx_len_;
  if (rx_len_ > 0) {
    memcpy(packet.payload, rx_pkt_internal_.payload, rx_len_);
  }
  packet.crc = rx_pkt_internal_.crc;
  if (!g_packet_queue.Push(packet)) {
    Panic(ErrorCode::kFcLinkRxQueueFull);
  }
  invalid_packet_count_ = 0;
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

  const auto should_alert_invalid = [this](ErrorCode code) {
    ++invalid_packet_count_;
    if (cfg_.invalid_packet_threshold > 0 &&
        invalid_packet_count_ >= cfg_.invalid_packet_threshold) {
      Panic(code);
    }
    return true;
  };

  for (int i = 0; i < n; ++i) {
    uint8_t b = buf[i];
    switch (rx_state_) {
      case RxState::kMagic1:
        if (b == message::kMagic1) {
          rx_state_ = RxState::kMagic2;
        } else {
          if (should_alert_invalid(ErrorCode::kFcLinkInvalidPacketMagic1)) {
            ESP_LOGW(kTag, "Invalid magic byte[1]: 0x%02X", (unsigned)b);
            Sys().TonePlayer().PlayBuiltin(::TonePlayer::BuiltinTone::kError);
          }
        }
        break;
      case RxState::kMagic2:
        if (b == message::kMagic2) {
          rx_state_ = RxState::kId;
        } else {
          if (should_alert_invalid(ErrorCode::kFcLinkInvalidPacketMagic2)) {
            ESP_LOGW(kTag, "Invalid magic byte[2]: 0x%02X", (unsigned)b);
            Sys().TonePlayer().PlayBuiltin(::TonePlayer::BuiltinTone::kError);
          }
          rx_state_ = RxState::kMagic1;
        }
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
          if (should_alert_invalid(ErrorCode::kFcLinkInvalidPacketLength)) {
            ESP_LOGW(kTag, "Invalid payload length: %u for msg id=0x%02X",
                     (unsigned)rx_len_, (unsigned)rx_pkt_internal_.id);
            Sys().TonePlayer().PlayBuiltin(::TonePlayer::BuiltinTone::kError);
          }
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
        FinishRxPacket();
        rx_state_ = RxState::kMagic1;
        break;
    }
  }
}
