// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>

#include "crsf_link_service.hpp"
#include "topic_scheduler.hpp"

class FcLink;
class SharedState;
struct BatteryData;

namespace message {
struct AttitudeMsg;
struct GpsData;
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
class TelemetryPublisher {
 public:
  struct Config {
    uint8_t fclink_max_frames_per_poll = 1;
    uint8_t crsf_max_frames_per_poll = 1;
    TopicConfig system_status{};
    TopicConfig vehicle_status{};
    TopicConfig esc_telemetry{};
    TopicConfig rc_channels{};
    TopicConfig usb_status{};
    TopicConfig gps{};
    TopicConfig attitude{};
    TopicConfig crsf_heartbeat{};
    TopicConfig crsf_gps{};
    TopicConfig crsf_battery{};
    TopicConfig crsf_flight_mode{};
    TopicConfig crsf_attitude{};
    TopicConfig crsf_rpm{};
    TopicConfig crsf_temperature{};
  };

  // Public only so the .cpp's config tables can be sized by them.
  static constexpr size_t kFcLinkTopicCount = 7u;
  static constexpr size_t kCrsfTopicCount = CrsfLinkService::kTopicCount;

  static TelemetryPublisher &GetInstance();

  void Poll(uint32_t now_us);

 private:
  friend class System;

  void Init(const Config &cfg, SharedState &blackboard, FcLink &fclink,
            CrsfLinkService &crsf, uint32_t now_us);

  // Faults belong to the sensor whose path suffered them, so each folds into
  // that sensor's health bit rather than getting a field of its own. FcLink's
  // own UART is absent on purpose: a report about FcLink's transport only
  // arrives over FcLink when it was not needed.
  //
  // Every counter behind these is a since-boot total that nothing resets, so
  // the window is what keeps `onboard_control_sensors_health` meaning what
  // MAVLink says it means -- an error now, not an error ever. A ground station
  // that wants "degraded this session" holds that itself; it sees every frame,
  // where the vehicle would have to lie in all of them to say the same thing.
  struct FaultWindow {
    uint32_t last_total = 0;
    bool healthy = true;
  };

  // Indexes fault_windows_. Every sensor that counts a fault appears; the
  // window is only half of each bit, which also has to be fresh.
  enum class FaultSource : uint8_t {
    kImu,
    kGps,
    kRc,
    kEsc,
    kBattery,
    kCount,
  };

  // Long enough that a single retried DMA error does not hold a sensor
  // unhealthy across a whole GCS refresh, short enough to still catch a
  // repeating fault.
  static constexpr uint32_t kLinkErrorWindowUs = 1000000u;

  void UpdateFaultWindows(uint32_t now_us);
  // Only the window: the sensor's own bit also has to be fresh.
  bool IsHealthy(FaultSource source) const {
    return fault_windows_[std::to_underlying(source)].healthy;
  }

  // The .cpp's config array is built in this order and indexed by it.
  enum class FcLinkTopic : uint8_t {
    kSystemStatus,
    kVehicleStatus,
    kEscTelemetry,
    kRcChannels,
    kUsbStatus,
    kGps,
    kAttitude,
    kCount,
  };

  static_assert(static_cast<size_t>(FcLinkTopic::kCount) == kFcLinkTopicCount);

  // Parallel to CrsfLinkService::TelemetryResult, and named apart from the
  // shared ::Outcome that reaches this header through uart.hpp.
  // kBlocked ends the group's poll rather than trying the next topic: a
  // refusal means that group's link is full, which the next topic on it would
  // only meet as well. Groups are polled separately, so it says nothing about
  // the other link.
  enum class PublishResult : uint8_t {
    kSent,
    kSkipped,
    kBlocked,
  };

  using Publish = PublishResult (*)(TelemetryPublisher &self, uint32_t now_us);

  // A scheduler and the deadlines it owns. Groups are independent by
  // construction: separate ladders, separate staggers, separate budgets, so
  // one link backing up cannot hold the other's slot.
  template <size_t N>
  struct Group {
    TopicScheduler scheduler{};
    std::array<TopicState, N> states{};
  };

  TelemetryPublisher() = default;
  ~TelemetryPublisher() = default;
  TelemetryPublisher(const TelemetryPublisher &) = delete;
  TelemetryPublisher &operator=(const TelemetryPublisher &) = delete;

  static uint16_t BatteryVoltageMv(const BatteryData &battery);
  static int16_t BatteryCurrentCa(const BatteryData &battery);
  static int8_t BatteryRemainingPct(const BatteryData &battery);
  uint16_t ComputeControlLoopLoad();
  message::SystemStatusMsg BuildSystemStatusMsg(uint32_t now_us,
                                                uint16_t load) const;
  message::VehicleStatusMsg BuildVehicleStatusMsg() const;
  message::UsbStatusMsg BuildUsbStatusMsg() const;
  message::GpsData BuildGpsMsg() const;
  message::AttitudeMsg BuildAttitudeMsg() const;

  static PublishResult PublishSystemStatus(TelemetryPublisher &self,
                                           uint32_t now_us);
  static PublishResult PublishVehicleStatus(TelemetryPublisher &self,
                                            uint32_t now_us);
  static PublishResult PublishEscTelemetry(TelemetryPublisher &self,
                                           uint32_t now_us);
  static PublishResult PublishRcChannels(TelemetryPublisher &self,
                                         uint32_t now_us);
  static PublishResult PublishUsbStatus(TelemetryPublisher &self,
                                        uint32_t now_us);
  static PublishResult PublishGps(TelemetryPublisher &self, uint32_t now_us);
  static PublishResult PublishAttitude(TelemetryPublisher &self,
                                       uint32_t now_us);

  // CrsfLinkService owns the payloads and the change detection; the silence
  // bound is the scheduler's, so it is passed in rather than duplicated there.
  static PublishResult PublishCrsfTopic(TelemetryPublisher &self,
                                        uint32_t now_us,
                                        CrsfLinkService::TelemetryTopic topic);
  static PublishResult PublishCrsfHeartbeat(TelemetryPublisher &self,
                                            uint32_t now_us);
  static PublishResult PublishCrsfGps(TelemetryPublisher &self,
                                      uint32_t now_us);
  static PublishResult PublishCrsfFlightMode(TelemetryPublisher &self,
                                    uint32_t now_us);
  static PublishResult PublishCrsfAttitude(TelemetryPublisher &self,
                                    uint32_t now_us);
  static PublishResult PublishCrsfRpm(TelemetryPublisher &self,
                                    uint32_t now_us);
  static PublishResult PublishCrsfTemperature(TelemetryPublisher &self,
                                    uint32_t now_us);
  static PublishResult PublishCrsfBattery(TelemetryPublisher &self,
                                          uint32_t now_us);

  // Emit whichever topics of one group are due, up to `budget` frames.
  template <size_t N>
  void PollGroup(Group<N> &group, const std::array<Publish, N> &publishers,
                 uint8_t budget, uint32_t now_us);

  Config cfg_{};
  SharedState *blackboard_ = nullptr;
  FcLink *fclink_svc_ = nullptr;
  CrsfLinkService *crsf_svc_ = nullptr;
  uint32_t last_link_window_us_ = 0;
  std::array<FaultWindow, std::to_underlying(FaultSource::kCount)>
      fault_windows_{};
  Group<kFcLinkTopicCount> fclink_{};  // -> UART1, the ESP32
  Group<kCrsfTopicCount> crsf_{};      // -> UART6, the receiver
  bool initialized_ = false;

  uint32_t load_last_busy_cycles_ = 0;
  uint32_t load_window_start_cycles_ = 0;

  // Last-published values, so a publisher can tell an unchanged payload from a
  // new one. Per-publisher rather than on the Blackboard: a shared flag would
  // serve exactly one reader.
  uint32_t usb_sent_timestamp_us_ = 0;
  bool have_usb_status_ = false;
  uint32_t rc_sent_timestamp_us_ = 0;
  uint8_t rc_sent_flags_ = 0;
  bool have_rc_channels_ = false;
  uint32_t gps_sent_timestamp_us_ = 0;
  bool have_gps_ = false;
  uint64_t attitude_sent_timestamp_us_ = 0;
  bool have_attitude_ = false;
};
