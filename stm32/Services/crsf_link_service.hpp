// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "common_config.hpp"
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
    kFlightMode,
    kAttitude,
    kRpm,
    kTemperature,
    kGpsTime,
    kCount,
  };

  static constexpr size_t kTopicCount =
      static_cast<size_t>(TelemetryTopic::kCount);

  // Payload bytes per topic, the longest each encodes; the wire adds four.
  // Indexed by TelemetryTopic, and public because the publisher costs a frame
  // against the link from it.
  static constexpr std::array<uint8_t, kTopicCount> kPayloadSize = {{
      2u,   // heartbeat: the origin address
      15u,  // gps
      8u,   // battery
      6u,  // flight mode: the longest name, the disarmed marker, the terminator
      6u,  // attitude
      // A source byte, then one entry per motor: 24-bit RPM, 16-bit deci-C.
      1u + (3u * common_config::kAirframeMotorCount),
      1u + (2u * common_config::kAirframeMotorCount),
      9u,  // gps time
  }};

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
    // The link the handset is set to: ELRS's own rate enumerator, and the
    // telemetry ratio's denominator with 0 standing for "Std", the ratio each
    // rate defaults to. Declared rather than learnt, because the receiver
    // reports the rate live but never the ratio.
    uint8_t expected_rf_mode = 0;
    uint8_t tlm_ratio_denom = 0;
    // Indexed by TelemetryTopic. A topic with this clear always sends.
    std::array<bool, kTopicCount> send_on_change{};
  };

  // One ELRS air rate as LinkStatistics.rf_mode names it: the over-the-air
  // packet rate, the denominator "Std" resolves to, and how many bytes of a
  // frame one telemetry packet carries.
  struct ElrsRate {
    uint16_t packet_hz;
    uint8_t std_denom;
    uint8_t bytes_per_call;
  };

  // What the link carries for telemetry at one rate under one ratio. Slots
  // are frame chunks, per kilosecond so an integer keeps three decimals of a
  // rate.
  struct TelemetryBudget {
    uint16_t packet_hz;
    uint8_t denom;
    uint8_t bytes_per_call;
    uint32_t slots_per_ks;
  };

  // Held back from the raw figure for resends on a lossy link and the
  // acknowledgement every chunk waits on. A safety factor on a model, so not
  // a knob.
  static constexpr uint32_t kBudgetMarginPct = 80u;

  static constexpr std::optional<ElrsRate> ElrsRateFor(uint8_t rf_mode) {
    for (const ElrsRateRow &row : kElrsRates) {
      if (row.rf_mode == rf_mode) {
        return row.rate;
      }
    }
    return std::nullopt;
  }

  // ExpressLRS runs telemetry in cycles of one link-statistics packet and
  // then a burst of data packets, the burst sized so link statistics never
  // fall more than 512 ms apart -- TLMBurstMaxForRateRatio in its common.cpp,
  // reproduced here. Every data packet carries one chunk of one frame.
  static constexpr uint32_t DataSlotsPerKs(uint16_t packet_hz, uint8_t denom) {
    const uint32_t telemetry_per_ks = (1000u * packet_hz) / denom;
    uint32_t burst = ((512u * packet_hz) / denom) / 1000u;
    burst = burst > 1u ? burst - 1u : 1u;
    return (telemetry_per_ks * burst) / (burst + 1u);
  }

  // A frame on the wire is sync, length, type, payload and CRC. The stubborn
  // sender folds its end marker into the last chunk of a multi-chunk frame
  // but spends a blank packet after a single-chunk one, so nothing costs
  // fewer than two.
  static constexpr uint8_t FrameSlots(TelemetryTopic topic,
                                      uint8_t bytes_per_call) {
    const uint32_t wire = kPayloadSize[static_cast<size_t>(topic)] + 4u;
    const uint32_t chunks = (wire + bytes_per_call - 1u) / bytes_per_call;
    return static_cast<uint8_t>(chunks < 2u ? 2u : chunks);
  }

  // The static form is what the generated config asserts the ladder's floors
  // against; the member reads the configured ratio for the live rate.
  static constexpr std::optional<TelemetryBudget> BudgetFor(
      uint8_t rf_mode, uint8_t denom_or_std) {
    const std::optional<ElrsRate> rate = ElrsRateFor(rf_mode);
    if (!rate) {
      return std::nullopt;
    }
    const uint8_t denom = denom_or_std == 0u ? rate->std_denom : denom_or_std;
    return TelemetryBudget{
        .packet_hz = rate->packet_hz,
        .denom = denom,
        .bytes_per_call = rate->bytes_per_call,
        .slots_per_ks =
            (DataSlotsPerKs(rate->packet_hz, denom) * kBudgetMarginPct) / 100u,
    };
  }
  std::optional<TelemetryBudget> BudgetFor(uint8_t rf_mode) const {
    return BudgetFor(rf_mode, cfg_.tlm_ratio_denom);
  }
  uint8_t ExpectedRfMode() const { return cfg_.expected_rf_mode; }

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

  struct ElrsRateRow {
    uint8_t rf_mode;
    ElrsRate rate;
  };

  // Transcribed from ExpressLRS src/src/common.cpp, one row per enumerator
  // that some hardware table implements; the rest are unknown. packet_hz is
  // 1e6 over the row's interval, which for the DVDA rates is the over-the-air
  // rate the telemetry cycle runs at rather than the RC rate in the name.
  static constexpr std::array<ElrsRateRow, 24> kElrsRates = {{
      {0, {25, 8, 5}},        // LORA_900_25HZ
      {1, {50, 16, 5}},       // LORA_900_50HZ
      {2, {100, 32, 5}},      // LORA_900_100HZ
      {3, {100, 32, 10}},     // LORA_900_100HZ_8CH
      {5, {200, 64, 5}},      // LORA_900_200HZ
      {6, {200, 64, 10}},     // LORA_900_200HZ_8CH
      {7, {250, 64, 5}},      // LORA_900_250HZ
      {10, {200, 64, 5}},     // LORA_900_50HZ_DVDA
      {11, {1000, 128, 10}},  // FSK_900_1000HZ_8CH
      {21, {50, 16, 5}},      // LORA_2G4_50HZ
      {23, {100, 32, 10}},    // LORA_2G4_100HZ_8CH
      {24, {150, 32, 5}},     // LORA_2G4_150HZ
      {27, {250, 64, 5}},     // LORA_2G4_250HZ
      {28, {333, 128, 10}},   // LORA_2G4_333HZ_8CH
      {29, {500, 128, 5}},    // LORA_2G4_500HZ
      {30, {1000, 128, 5}},   // FLRC_2G4_250HZ_DVDA
      {31, {1000, 128, 5}},   // FLRC_2G4_500HZ_DVDA
      {32, {500, 128, 5}},    // FLRC_2G4_500HZ
      {33, {1000, 128, 5}},   // FLRC_2G4_1000HZ
      {34, {1000, 128, 5}},   // FSK_2G4_250HZ_DVDA
      {35, {1000, 128, 5}},   // FSK_2G4_500HZ_DVDA
      {36, {1000, 128, 5}},   // FSK_2G4_1000HZ
      {100, {100, 32, 10}},   // LORA_DUAL_100HZ_8CH
      {101, {150, 32, 5}},    // LORA_DUAL_150HZ
  }};

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

  // TODO(#11): baro 0x09 and vario 0x07 join when the DPS310 lands. Cells
  // 0x0E stays out while only pack voltage is sensed: a per-cell split of it
  // reads as a balanced pack however far one cell has sagged.
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
