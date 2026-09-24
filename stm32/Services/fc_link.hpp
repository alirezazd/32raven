// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <optional>

#include "message.hpp"
#include "ring_buffer.hpp"
#include "shared_state.hpp"
#include "uart.hpp"

class FcLink {
 public:
  static FcLink &GetInstance();

  // A tick is BeginRx, PopPacket until empty, then FlushTx. The caller
  // dispatches what it pops, so the link never sees the state machine.
  void BeginRx();
  std::optional<message::Packet> PopPacket(uint32_t now_us);
  void FlushTx();

  bool Send(const message::Packet &pkt);

  template <typename T>
  void SendPacket(message::MsgId id, const T &body) {
    Send(message::MakePacket(id, body));
  }

  void SendLog(const char *format, ...);

 private:
  friend class System;
  void Init(Uart1 &uart, SharedState &blackboard);

  FcLink() = default;
  ~FcLink() = default;
  FcLink(const FcLink &) = delete;
  FcLink &operator=(const FcLink &) = delete;

  // One max-sized frame per tick: a smaller budget falls behind a peer
  // sending full frames back to back.
  static constexpr size_t kRxByteBudget =
      message::kMaxPayload + message::kPacketOverhead;
  static constexpr size_t kTxBufSize = 512;

  enum class RxState { kMagic1, kMagic2, kId, kLen, kPayload, kCrc1, kCrc2 };

  Uart1 *uart_ = nullptr;
  SharedState *blackboard_ = nullptr;
  RxState rx_state_ = RxState::kMagic1;
  uint8_t rx_idx_ = 0;
  message::Packet rx_pkt_{};
  size_t rx_budget_left_ = 0;
  RingBuffer<uint8_t, kTxBufSize> tx_rb_;
  bool initialized_ = false;
  bool flushed_ = false;
};
