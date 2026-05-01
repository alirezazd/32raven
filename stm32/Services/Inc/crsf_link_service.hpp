#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "message.hpp"
#include "uart.hpp"

class FcLink;
class RcReceiver;
class VehicleState;

class CrsfLinkService {
 public:
  struct TopicConfig {
    uint32_t period_us = 0;
    uint32_t start_delay_us = 0;
    uint32_t max_silence_us = 0;
    uint8_t priority = 0;
    bool send_on_change = false;
  };

  struct Config {
    uint8_t max_frames_per_poll = 1;
    uint32_t gps_fresh_timeout_us = 2000000u;
    TopicConfig heartbeat{};
    TopicConfig gps{};
    TopicConfig battery{};
  };

  void Init(const Config &cfg, Uart6 &uart, VehicleState &vehicle_state,
            RcReceiver &rc_receiver, FcLink &fc_link);
  void PollRx(uint32_t now_us, size_t byte_budget = 128u);
  void PollTx(uint32_t now_us, size_t max_frames = 0u);

  void RequestReceiverBind();
  void RequestReceiverCancelBind();

 private:
  friend class System;

  enum class TelemetryTopic : uint8_t {
    kHeartbeat,
    kGps,
    kBattery,
    kCount,
  };

  struct TopicState {
    uint32_t next_due_us = 0;
    uint32_t last_sent_us = 0;
    uint8_t last_payload_len = 0;
    bool have_last_payload = false;
    std::array<uint8_t, 16> last_payload{};
  };

  enum class PendingCommand : uint8_t {
    kNone,
    kReceiverBind,
    kReceiverCancelBind,
  };

  // TODO(crsf):
  // - Add more native telemetry frames as upstream data becomes available:
  //   flight mode (0x21), attitude (0x1E), baro/vario (0x09/0x07),
  //   temperatures (0x0D), voltages/cell data (0x0E), rpm (0x0C),
  //   richer link stats if the radio-side UI benefits.
  // - Slot future telemetry topics into the topic table and scheduler.
  bool SendScheduledTelemetry(uint32_t now_us);
  void PrimeScheduler(uint32_t now_us);
  const TopicConfig &GetTelemetryTopicConfig(TelemetryTopic topic) const;
  TopicState &GetTelemetryTopicState(TelemetryTopic topic);
  bool IsTelemetryTopicReady(TelemetryTopic topic, uint32_t now_us) const;
  bool PrepareTelemetryTopic(TelemetryTopic topic, uint32_t now_us,
                             uint8_t &type, uint8_t *payload,
                             uint8_t &payload_len) const;
  bool ShouldSendTelemetryTopic(TelemetryTopic topic, uint32_t now_us,
                                const uint8_t *payload,
                                uint8_t payload_len) const;
  uint32_t ComputeDeferredDueTime(TelemetryTopic topic, uint32_t now_us,
                                  const uint8_t *payload,
                                  uint8_t payload_len) const;
  bool TrySendTelemetryTopic(TelemetryTopic topic, uint32_t now_us,
                             const uint8_t *payload, uint8_t payload_len,
                             uint8_t type);
  bool TrySendHeartbeatTelemetry();
  bool TrySendGpsTelemetry(const uint8_t *payload, uint8_t payload_len);
  bool TrySendBatteryTelemetry();
  // TODO(crsf): Track command ACK/timeout state for bind/cancel-bind.
  // If the receiver does not ACK within the expected window, decide whether
  // to retry, surface an FcLink error, or escalate to Panic().
  bool TrySendPendingCommand();
  bool ProcessCrsfByte(uint8_t byte, uint32_t now_us);
  bool FinishCrsfFrame(uint32_t now_us);
  bool ParseLinkStatisticsFrame(const uint8_t *payload, std::size_t len,
                                uint32_t now_us);
  bool ParseRcChannelsFrame(const uint8_t *payload, std::size_t len,
                            uint32_t now_us);
  void ForwardLatestRawState(uint32_t now_us);

  bool SendBroadcastFrame(uint8_t type, const uint8_t *payload,
                          uint8_t payload_len);
  bool SendDirectCommand(uint8_t destination, uint8_t command_id,
                         const uint8_t *payload, uint8_t payload_len);

  Uart6 *uart_ = nullptr;
  VehicleState *vehicle_state_ = nullptr;
  RcReceiver *rc_receiver_ = nullptr;
  FcLink *fc_link_ = nullptr;
  Config cfg_{};
  bool initialized_ = false;
  bool scheduler_started_ = false;
  uint32_t last_forward_us_ = 0;
  uint8_t last_forwarded_flags_ = 0;
  bool have_forwarded_state_ = false;
  uint8_t last_link_quality_ = 0;
  bool pending_forward_ = false;
  bool crsf_have_length_ = false;
  uint8_t crsf_frame_len_ = 0;
  std::size_t crsf_frame_pos_ = 0;
  std::array<uint8_t, 64> crsf_frame_{};
  std::array<TopicState, static_cast<size_t>(TelemetryTopic::kCount)>
      telemetry_states_{};
  message::RcChannelsMsg latest_rx_msg_{};
  PendingCommand pending_command_ = PendingCommand::kNone;
};
