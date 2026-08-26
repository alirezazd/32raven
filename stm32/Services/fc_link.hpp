// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include "message.hpp"
#include "ring_buffer.hpp"
#include "shared_state.hpp"
#include "uart.hpp"

struct AppContext;

class FcLink {
 public:
  static FcLink &GetInstance();

  void Init(const AppContext *ctx, Uart1 &uart, SharedState &blackboard);

  // Min RX byte budget per Poll to drain one max-sized packet. Smaller
  // budgets stall: frames arriving faster than rx_budget/frame_size per
  // Poll pile up unprocessed.
  static constexpr size_t kMinRxByteBudget =
      message::kMaxPayload + message::kPacketOverhead;

  void Poll(size_t rx_budget = kMinRxByteBudget, size_t tx_budget = 64);

  // Sending Logic
  bool Send(const message::Packet &pkt);

  // Send one-shot RC channel mapping config.
  void SendRcChannels(const message::RcChannelsMsg &msg);
  void SendRcMapConfig(const message::RcMapConfigMsg &cfg);
  void SendRcCalibrationConfig(const message::RcCalibrationConfigMsg &cfg);
  void SendGyroCalibrationIdConfig(
      const message::GyroCalibrationIdConfigMsg &cfg);
  void SendEscTelemetry(const EscTelemetryData &data);
  void SendSystemStatus(const message::SystemStatusMsg &msg);
  void SendVehicleStatus(const message::VehicleStatusMsg &msg);

  template <typename T>
  void SendPacket(message::MsgId id, const T &body) {
    Send(message::MakePacket(id, body));
  }

  // Send Log Message
  void SendLog(const char *format, ...);

 private:
  FcLink() = default;
  ~FcLink() = default;
  FcLink(const FcLink &) = delete;
  FcLink &operator=(const FcLink &) = delete;

  const AppContext *ctx_ = nullptr;
  Uart1 *uart_ = nullptr;
  SharedState *blackboard_ = nullptr;

  // Both publish FcLinkData. Only a frame that passed its CRC moves the
  // stamp: it is the peer's heartbeat, and garbage is not the peer being
  // heard from.
  void NoteValidFrame();
  void NoteChecksumFailure();

  // RX Parsing State
  enum class RxState { kMagic1, kMagic2, kId, kLen, kPayload, kCrc1, kCrc2 };
  RxState rx_state_ = RxState::kMagic1;
  uint8_t rx_idx_ = 0;
  uint8_t rx_len_ = 0;
  struct {
    uint8_t id;
    uint8_t len;
    uint8_t payload[message::kMaxPayload];
    uint16_t crc;
  } rx_pkt_internal_;

  uint32_t checksum_failures_ = 0;

  // TX Buffer
  static constexpr size_t kTxBufSize = 512;
  RingBuffer<uint8_t, kTxBufSize> tx_rb_;

  bool initialized_ = false;
};
