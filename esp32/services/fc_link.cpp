#include "fc_link.hpp"

#include <algorithm>
#include <cstring>

#include "error_code.hpp"
#include "panic.hpp"
#include "system.hpp"

extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

static constexpr const char *kTag = "FC_Link";

namespace {

void PanicFromHandshakePacket(const message::Packet &packet) {
  if (packet.header.id != static_cast<uint8_t>(message::MsgId::kPanic) ||
      packet.header.len != message::PayloadLength<message::PanicMsg>()) {
    return;
  }

  message::PanicMsg panic_msg{};
  std::memcpy(&panic_msg, packet.payload, sizeof(panic_msg));
  const auto error_code = static_cast<ErrorCode>(panic_msg.error_code);
  ESP_LOGE(kTag, "STM32 panic during handshake: %lu (%s)",
           static_cast<unsigned long>(panic_msg.error_code),
           GetMessage(error_code));
  Panic(error_code);
}

}  // namespace

void FcLink::Init(const Config &cfg, UartFcLink *uart) {
  if (uart == nullptr) {
    Panic(ErrorCode::kFcLinkInitFailed);
  }
  cfg_ = cfg;
  if (cfg_.handshake_attempts == 0 || cfg_.handshake_retry_period_ms == 0 ||
      cfg_.invalid_packet_threshold == 0) {
    Panic(ErrorCode::kFcLinkInitFailed);
  }

  uart_ = uart;
  ResetRxState();

  PerformHandshake();
  ESP_LOGI(kTag, "Initialized");
}

void FcLink::ResetRxState(bool flush_uart) {
  rx_packet_queue_.Clear();
  invalid_packet_count_ = 0;
  rx_synchronized_ = false;
  rx_state_ = RxState::kMagic1;
  rx_idx_ = 0;
  rx_len_ = 0;
  if (flush_uart && uart_ != nullptr) {
    uart_->Flush();
  }
}

void FcLink::PerformHandshake() {
  ResetRxState();

  ESP_LOGI(kTag, "Handshake Start...");
  message::Packet ping{};
  ping.header.id = static_cast<uint8_t>(message::MsgId::kPing);
  ping.header.len = 0;

  for (uint16_t i = 0; i < cfg_.handshake_attempts; ++i) {
    SendPacket(ping);
    vTaskDelay(pdMS_TO_TICKS(cfg_.handshake_retry_period_ms));

    Poll();

    const size_t queued_packet_count = PendingRxPacketCount();
    bool got_pong = false;
    for (size_t packet_idx = 0; packet_idx < queued_packet_count;
         ++packet_idx) {
      auto packet = PopPacket();
      if (!packet.has_value()) {
        Panic(ErrorCode::kFcLinkRxQueueFull);
      }

      if (packet->header.id == static_cast<uint8_t>(message::MsgId::kPing)) {
        continue;
      }

      if (packet->header.id == static_cast<uint8_t>(message::MsgId::kPong)) {
        got_pong = true;
        continue;
      }

      if (packet->header.id == static_cast<uint8_t>(message::MsgId::kPanic)) {
        PanicFromHandshakePacket(*packet);
      }

      QueueRxPacket(*packet);
    }

    if (got_pong) {
      ESP_LOGI(kTag, "Handshake Success!");
      return;
    }
  }

  ESP_LOGW(kTag, "Handshake Failed");
  Panic(ErrorCode::kFcLinkHandshakeFailed);
}

void FcLink::SendPacket(const message::Packet &pkt) {
  uint8_t tx[message::kMaxPayload + message::kPacketOverhead];
  const size_t len =
      message::Serialize(static_cast<message::MsgId>(pkt.header.id),
                         pkt.payload, pkt.header.len, tx);
  if (!len) {
    ESP_LOGE(kTag, "Failed to serialize packet id=0x%02X len=%u",
             static_cast<unsigned>(pkt.header.id),
             static_cast<unsigned>(pkt.header.len));
    Panic(ErrorCode::kFcLinkTxSerializeFailed);
    return;
  }

  uart_->Write(tx, len);
}

std::optional<message::Packet> FcLink::PopPacket() {
  message::Packet packet{};
  const bool have_packet = rx_packet_queue_.Pop(packet);
  if (!have_packet) {
    return std::nullopt;
  }
  return packet;
}

size_t FcLink::PendingRxPacketCount() const {
  return rx_packet_queue_.Available();
}

void FcLink::QueueRxPacket(const message::Packet &pkt) {
  const bool queued = rx_packet_queue_.Push(pkt);
  if (!queued) {
    Panic(ErrorCode::kFcLinkRxQueueFull);
  }
}

void FcLink::FinishRxPacket() {
  const auto should_alert_invalid = [this](ErrorCode code) {
    if (!rx_synchronized_) {
      return false;
    }
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
               static_cast<unsigned>(rx_pkt_internal_.id),
               static_cast<unsigned>(rx_len_));
      Sys().TonePlayer().PlayBuiltin(::TonePlayer::BuiltinTone::kError);
    }
    return;
  }

  const auto crc16_update = [](uint16_t crc, uint8_t byte) {
    crc ^= static_cast<uint16_t>(byte) << 8;
    for (int j = 0; j < 8; ++j) {
      crc = (crc & 0x8000u) ? static_cast<uint16_t>((crc << 1) ^ 0x1021u)
                            : static_cast<uint16_t>(crc << 1);
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
    std::memcpy(packet.payload, rx_pkt_internal_.payload, rx_len_);
  }
  packet.crc = rx_pkt_internal_.crc;
  if (!rx_packet_queue_.Push(packet)) {
    Panic(ErrorCode::kFcLinkRxQueueFull);
  }
  invalid_packet_count_ = 0;
  rx_synchronized_ = true;
}

void FcLink::Poll() {
  const size_t pending_bytes = uart_->BufferedRxBytes();
  if (pending_bytes == 0) {
    return;
  }

  uint8_t buf[kMaxRxReadBufferSize];
  const size_t read_size = std::min(pending_bytes, sizeof(buf));
  const int n = uart_->Read(buf, read_size);

  const auto should_alert_invalid = [this](ErrorCode code) {
    if (!rx_synchronized_) {
      return false;
    }
    ++invalid_packet_count_;
    if (cfg_.invalid_packet_threshold > 0 &&
        invalid_packet_count_ >= cfg_.invalid_packet_threshold) {
      Panic(code);
    }
    return true;
  };

  for (int i = 0; i < n; ++i) {
    const uint8_t b = buf[i];
    switch (rx_state_) {
      case RxState::kMagic1:
        if (b == message::kMagic1) {
          rx_state_ = RxState::kMagic2;
        } else {
          if (should_alert_invalid(ErrorCode::kFcLinkInvalidPacketMagic1)) {
            ESP_LOGW(kTag, "Invalid magic byte[1]: 0x%02X",
                     static_cast<unsigned>(b));
            Sys().TonePlayer().PlayBuiltin(::TonePlayer::BuiltinTone::kError);
          }
        }
        break;
      case RxState::kMagic2:
        if (b == message::kMagic2) {
          rx_state_ = RxState::kId;
        } else {
          if (should_alert_invalid(ErrorCode::kFcLinkInvalidPacketMagic2)) {
            ESP_LOGW(kTag, "Invalid magic byte[2]: 0x%02X",
                     static_cast<unsigned>(b));
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
                     static_cast<unsigned>(rx_len_),
                     static_cast<unsigned>(rx_pkt_internal_.id));
            Sys().TonePlayer().PlayBuiltin(::TonePlayer::BuiltinTone::kError);
          }
          rx_state_ = RxState::kMagic1;
          break;
        }
        rx_state_ = (rx_len_ > 0) ? RxState::kPayload : RxState::kCrc1;
        break;
      case RxState::kPayload:
        rx_pkt_internal_.payload[rx_idx_++] = b;
        if (rx_idx_ >= rx_len_) {
          rx_state_ = RxState::kCrc1;
        }
        break;
      case RxState::kCrc1:
        rx_pkt_internal_.crc = b;
        rx_state_ = RxState::kCrc2;
        break;
      case RxState::kCrc2:
        rx_pkt_internal_.crc |= (static_cast<uint16_t>(b) << 8);
        FinishRxPacket();
        rx_state_ = RxState::kMagic1;
        break;
    }
  }
}
