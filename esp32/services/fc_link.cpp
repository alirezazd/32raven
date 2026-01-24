#include "fc_link.hpp"
#include <cstring>

extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

static constexpr const char *kTag = "link";

void FcLink::Init(const Config &cfg, Uart *uart) {
  if (initialized_)
    return;
  cfg_ = cfg;
  uart_ = uart;
  initialized_ = true;
  ESP_LOGI(kTag, "Initialized");
}

bool FcLink::PerformHandshake() {
  if (!initialized_ || !uart_)
    return false;

  ESP_LOGI(kTag, "Handshake Start...");

  uint8_t tx_buf[32];
  size_t tx_len = message::Serialize(message::MsgId::kPing, nullptr, 0, tx_buf);
  const int kDurationMs = 1000;
  const int kPeriodMs = 20;
  const int kAttempts = kDurationMs / kPeriodMs;

  for (int i = 0; i < kAttempts; ++i) {
    uart_->Write(tx_buf, tx_len);
    vTaskDelay(pdMS_TO_TICKS(kPeriodMs));

    Poll(0);

    message::Packet response;
    if (GetPacket(response)) {
      if (response.header.id == (uint8_t)message::MsgId::kPong) {
        ESP_LOGI(kTag, "Handshake Success!");
        return true;
      }
    }
  }

  ESP_LOGW(kTag, "Handshake Failed");
  return false;
}

bool FcLink::SendPacket(const message::Packet &pkt) {
  if (!initialized_ || !uart_)
    return false;

  uint8_t tx[256 + 32]; // Max payload + header
  size_t len = message::Serialize((message::MsgId)pkt.header.id, pkt.payload,
                                  pkt.header.len, tx);
  uart_->Write(tx, len);
  return true;
}

bool FcLink::GetPacket(message::Packet &out_pkt) { return queue_.Pop(out_pkt); }

bool FcLink::IsConnected() const { return (rx_state_ != RxState::kMagic1); }

void FcLink::Poll(uint32_t now) {
  (void)now;
  if (!initialized_ || !uart_)
    return;

  if (!initialized_ || !uart_)
    return;

  uint8_t buf[128];
  int n = uart_->Read(buf, sizeof(buf));

  for (int i = 0; i < n; ++i) {
    uint8_t b = buf[i];
    switch (rx_state_) {
    case RxState::kMagic1:
      if (b == message::kMagic1)
        rx_state_ = RxState::kMagic2;
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
      rx_state_ = (rx_len_ > 0) ? RxState::kPayload : RxState::kCrc1;
      break;
    case RxState::kPayload:
      rx_pkt_internal_.payload[rx_idx_++] = b;
      if (rx_idx_ >= rx_len_)
        rx_state_ = RxState::kCrc1;
      break;
    case RxState::kCrc1:
      rx_pkt_internal_.crc = b;
      rx_state_ = RxState::kCrc2;
      break;
    case RxState::kCrc2:
      rx_pkt_internal_.crc |= ((uint16_t)b << 8);

      rx_pkt_internal_.crc |= ((uint16_t)b << 8);

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
          if (rx_len_ > 0)
            memcpy(pkt.payload, rx_pkt_internal_.payload, rx_len_);
          pkt.crc = rx_pkt_internal_.crc;

          if (!queue_.Push(pkt)) {
          }
        } else {
          ESP_LOGE(kTag, "CRC Fail");
        }
      }
      rx_state_ = RxState::kMagic1;
      break;
    }
  }
}
