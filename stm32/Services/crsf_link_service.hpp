// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "message.hpp"
#include "uart.hpp"

class RcReceiver;
class SharedState;

class CrsfLinkService {
 public:
  // TelemetryPublisher indexes its CRSF group by this order.
  enum class TelemetryTopic : uint8_t {
    kHeartbeat,
    kGps,
    kBattery,
    kCount,
  };

  static constexpr size_t kTopicCount =
      static_cast<size_t>(TelemetryTopic::kCount);

  enum class TelemetryResult : uint8_t {
    kSent,
    // No data to encode, or a send_on_change payload that has not moved.
    kSkipped,
    // The receiver's UART is backed up; nothing was written.
    kBlocked,
  };

  struct Config {
    uint32_t gps_fresh_timeout_us = 2000000u;
    uint32_t battery_fresh_timeout_us = 1000000u;
    // Indexed by TelemetryTopic. A topic with this clear always sends.
    std::array<bool, kTopicCount> send_on_change{};
  };

  void Init(const Config &cfg, Uart6 &uart, SharedState &blackboard,
            RcReceiver &rc_receiver);
  void PollRx(uint32_t now_us, size_t byte_budget = 128u);
  void PollCommands();

  // Encode one topic from the blackboard as it stands and put it on the wire.
  // When is TelemetryPublisher's call, so `silence_expired` -- its scheduler's
  // cue that an unchanged payload has been held back long enough -- arrives
  // here rather than being tracked twice.
  TelemetryResult SendTelemetry(TelemetryTopic topic, bool silence_expired,
                                uint32_t now_us);

  void RequestReceiverBind();
  void RequestReceiverCancelBind();

 private:
  friend class System;

  static constexpr uint8_t kMaxTelemetryPayload = 16u;

  struct TelemetryFrame {
    uint8_t type = 0;
    uint8_t len = 0;
    std::array<uint8_t, kMaxTelemetryPayload> payload{};
  };

  // What a send_on_change topic compares against. Separate from TopicState
  // because the scheduler is deliberately blind to payloads.
  struct PayloadMemo {
    uint8_t len = 0;
    bool valid = false;
    std::array<uint8_t, kMaxTelemetryPayload> bytes{};
  };

  enum class PendingCommand : uint8_t {
    kNone,
    kReceiverBind,
    kReceiverCancelBind,
  };

  // TODO(#11): add native telemetry frames as data lands and give each a
  // topic in TelemetryPublisher's CRSF group: flight mode 0x21, attitude 0x1E,
  // baro/vario 0x09/0x07, temps 0x0D, voltages/cell 0x0E, rpm 0x0C.
  std::optional<TelemetryFrame> PrepareTelemetryTopic(TelemetryTopic topic,
                                                      uint32_t now_us) const;
  bool PayloadChanged(TelemetryTopic topic, const TelemetryFrame &frame) const;
  bool TrySendPendingCommand();
  bool ProcessCrsfByte(uint8_t byte, uint32_t now_us);
  bool FinishCrsfFrame(uint32_t now_us);
  bool ParseLinkStatisticsFrame(const uint8_t *payload, std::size_t len,
                                uint32_t now_us);
  bool ParseRcChannelsFrame(const uint8_t *payload, std::size_t len,
                            uint32_t now_us);

  bool SendBroadcastFrame(uint8_t type, const uint8_t *payload,
                          uint8_t payload_len);
  bool SendDirectCommand(uint8_t destination, uint8_t command_id,
                         const uint8_t *payload, uint8_t payload_len);

  Uart6 *uart_ = nullptr;
  SharedState *blackboard_ = nullptr;
  RcReceiver *rc_receiver_ = nullptr;
  Config cfg_{};
  bool initialized_ = false;
  bool crsf_have_length_ = false;
  uint8_t crsf_frame_len_ = 0;
  std::size_t crsf_frame_pos_ = 0;
  std::array<uint8_t, 64> crsf_frame_{};
  std::array<PayloadMemo, kTopicCount> payload_memos_{};
  PendingCommand pending_command_ = PendingCommand::kNone;
  uint32_t checksum_failures_ = 0;
};
