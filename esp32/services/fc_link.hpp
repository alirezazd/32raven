// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <optional>

#include "esp32_limits.hpp"
#include "message.hpp"
#include "ring_buffer.hpp"
#include "uart.hpp"

class FcLink {
 public:
  struct Config {
    uint16_t handshake_attempts = 0;
    uint16_t handshake_retry_period_ms = 0;
    uint8_t invalid_packet_threshold = 0;
  };

  static FcLink &GetInstance() {
    static FcLink instance;
    return instance;
  }
  void Poll();
  void ResetRxState(bool flush_uart = true);
  void SendPacket(const message::Packet &pkt);

  template <typename T>
  void SendPacket(message::MsgId id, const T &body) {
    message::Packet pkt{};
    pkt.header.id = static_cast<uint8_t>(id);
    pkt.header.len = message::PayloadLength<T>();
    std::memcpy(pkt.payload, &body, sizeof(T));
    SendPacket(pkt);
  }

  std::optional<message::Packet> PopPacket();

 private:
  friend class System;
  void Init(const Config &cfg, UartFcLink *uart);
  void PerformHandshake();
  size_t PendingRxPacketCount() const;
  void QueueRxPacket(const message::Packet &pkt);
  void FinishRxPacket();
  FcLink() = default;
  ~FcLink() = default;
  FcLink(const FcLink &) = delete;
  FcLink &operator=(const FcLink &) = delete;

  UartFcLink *uart_ = nullptr;
  Config cfg_ = {};
  static constexpr size_t kMaxRxReadBufferSize =
      message::kMaxPayload + message::kPacketOverhead;
  RingBuffer<message::Packet, esp32_limits::kFcLinkRxPacketQueueDepth + 1>
      rx_packet_queue_;

  enum class RxState { kMagic1, kMagic2, kId, kLen, kPayload, kCrc1, kCrc2 };
  RxState rx_state_ = RxState::kMagic1;
  uint8_t rx_idx_ = 0;
  uint8_t rx_len_ = 0;
  uint8_t invalid_packet_count_ = 0;
  bool rx_synchronized_ = false;
  struct {
    uint8_t id;
    uint8_t len;
    uint8_t payload[message::kMaxPayload];
    uint16_t crc;
  } rx_pkt_internal_;
};
