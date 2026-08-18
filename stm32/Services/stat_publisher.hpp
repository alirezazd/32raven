// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "crsf_link_service.hpp"
#include "topic_scheduler.hpp"

struct AppContext;
struct BatteryData;

namespace message {
struct SystemStatusMsg;
struct UsbStatusMsg;
struct VehicleStatusMsg;
}  // namespace message

// Every periodic publication on this board, each with a Kconfig period and
// priority. Poll() emits at most `max_frames_per_poll` of whichever are due, so
// what drops when the loop runs late is a stated policy rather than a
// consequence of where a call sits in the main tick.
//
// Only publications with a *rate* belong here. Replies, tones, logs and the
// panic packet fire at their trigger site, carrying data that exists only at
// that moment; GPS publishes on PVT arrival, where scheduling could only delay
// a sample that is already fresh.
class StatPublisher {
 public:
  struct Config {
    uint8_t fclink_max_frames_per_poll = 1;
    uint8_t crsf_max_frames_per_poll = 1;
    TopicConfig system_status{};
    TopicConfig vehicle_status{};
    TopicConfig esc_telemetry{};
    TopicConfig rc_channels{};
    TopicConfig usb_status{};
    TopicConfig crsf_heartbeat{};
    TopicConfig crsf_gps{};
    TopicConfig crsf_battery{};
  };

  // Public only so the .cpp's config tables can be sized by them.
  static constexpr size_t kFcLinkTopicCount = 5u;
  static constexpr size_t kCrsfTopicCount = CrsfLinkService::kTopicCount;

  static StatPublisher &GetInstance();

  // `loop_counter` reaches SystemStatus verbatim. An argument rather than a
  // member because it describes the caller's loop, not this publisher.
  void Poll(const AppContext &ctx, uint32_t now_us, uint32_t loop_counter);

 private:
  friend class System;

  void Init(const Config &cfg, uint32_t now_us);

  // The .cpp's config array is built in this order and indexed by it.
  enum class FcLinkTopic : uint8_t {
    kSystemStatus,
    kVehicleStatus,
    kEscTelemetry,
    kRcChannels,
    kUsbStatus,
    kCount,
  };

  static_assert(static_cast<size_t>(FcLinkTopic::kCount) == kFcLinkTopicCount);

  // kBlocked ends the group's poll rather than trying the next topic: a
  // refusal means that group's link is full, which the next topic on it would
  // only meet as well. Groups are polled separately, so it says nothing about
  // the other link.
  enum class Outcome : uint8_t {
    kSent,
    kSkipped,
    kBlocked,
  };

  using Publish = Outcome (*)(StatPublisher &self, const AppContext &ctx,
                              uint32_t now_us);

  // A scheduler and the deadlines it owns. Groups are independent by
  // construction: separate ladders, separate staggers, separate budgets, so
  // one link backing up cannot hold the other's slot.
  template <size_t N>
  struct Group {
    TopicScheduler scheduler{};
    std::array<TopicState, N> states{};
  };

  StatPublisher() = default;
  ~StatPublisher() = default;
  StatPublisher(const StatPublisher &) = delete;
  StatPublisher &operator=(const StatPublisher &) = delete;

  static uint16_t BatteryVoltageMv(const BatteryData &battery);
  static int16_t BatteryCurrentCa(const BatteryData &battery);
  static int8_t BatteryRemainingPct(const BatteryData &battery);
  static message::SystemStatusMsg BuildSystemStatusMsg(const AppContext &ctx,
                                                       uint32_t now_us,
                                                       uint32_t loop_counter);
  static message::VehicleStatusMsg BuildVehicleStatusMsg(const AppContext &ctx);
  static message::UsbStatusMsg BuildUsbStatusMsg(const AppContext &ctx);

  static Outcome PublishSystemStatus(StatPublisher &self, const AppContext &ctx,
                                     uint32_t now_us);
  static Outcome PublishVehicleStatus(StatPublisher &self, const AppContext &ctx,
                                      uint32_t now_us);
  static Outcome PublishEscTelemetry(StatPublisher &self, const AppContext &ctx,
                                     uint32_t now_us);
  static Outcome PublishRcChannels(StatPublisher &self, const AppContext &ctx,
                                   uint32_t now_us);
  static Outcome PublishUsbStatus(StatPublisher &self, const AppContext &ctx,
                                  uint32_t now_us);

  // CrsfLinkService owns the payloads and the change detection; the silence
  // bound is the scheduler's, so it is passed in rather than duplicated there.
  static Outcome PublishCrsfTopic(StatPublisher &self, const AppContext &ctx,
                                  uint32_t now_us,
                                  CrsfLinkService::TelemetryTopic topic);
  static Outcome PublishCrsfHeartbeat(StatPublisher &self, const AppContext &ctx,
                                      uint32_t now_us);
  static Outcome PublishCrsfGps(StatPublisher &self, const AppContext &ctx,
                                uint32_t now_us);
  static Outcome PublishCrsfBattery(StatPublisher &self, const AppContext &ctx,
                                    uint32_t now_us);

  // Emit whichever topics of one group are due, up to `budget` frames.
  template <size_t N>
  void PollGroup(Group<N> &group, const std::array<Publish, N> &publishers,
                 uint8_t budget, const AppContext &ctx, uint32_t now_us);

  Config cfg_{};
  Group<kFcLinkTopicCount> fclink_{};  // -> UART1, the ESP32
  Group<kCrsfTopicCount> crsf_{};      // -> UART6, the receiver
  bool initialized_ = false;

  uint32_t loop_counter_ = 0;

  // Last-published values, so a publisher can tell an unchanged payload from a
  // new one. Per-publisher rather than on the Blackboard: a shared flag would
  // serve exactly one reader.
  uint32_t usb_sent_timestamp_us_ = 0;
  bool have_usb_status_ = false;
  uint32_t rc_sent_timestamp_us_ = 0;
  uint8_t rc_sent_flags_ = 0;
  bool have_rc_channels_ = false;
};
