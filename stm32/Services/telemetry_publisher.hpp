// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>

#include "crsf_link_service.hpp"
#include "fc_link.hpp"
#include "message.hpp"
#include "shared_state.hpp"
#include "topic_scheduler.hpp"

// Every periodic publication on this board. The FcLink group's cadences are
// fixed in the .cpp: both ends of that link are this project, each consumer's
// need is known, and the build checks the ladder against the configured baud.
// It emits at most a fixed count of frames per poll, so what drops when the
// loop runs late is a stated policy rather than a consequence of where a call
// sits in the main tick. The CRSF group faces a link of varying capacity, so
// its periods are Kconfig's and are fitted to the link: the receiver's
// telemetry budget follows from the air rate and the handset's ratio, and
// every period is stretched to fit it, evenly, each topic no further than its
// max_silence while those floors fit. Nothing drops there; a slow link is a
// slow ladder.
//
// Only publications with a *rate* belong here. Replies, tones, logs and the
// panic packet fire at their trigger site, carrying data that exists only at
// that moment; GPS publishes on PVT arrival, where scheduling could only delay
// a sample that is already fresh.
class TelemetryPublisher {
 public:
  struct Config {
    TopicConfig crsf_heartbeat{};
    TopicConfig crsf_gps{};
    TopicConfig crsf_battery{};
    TopicConfig crsf_flight_mode{};
    TopicConfig crsf_attitude{};
    TopicConfig crsf_rpm{};
    TopicConfig crsf_temperature{};
    TopicConfig crsf_gps_time{};
    TopicConfig crsf_baro_altitude{};
    // Declination and the module's yaw off the nose, as the one angle the
    // compass bearing is short of true. East positive.
    float heading_offset_rad = 0.0f;
  };

  // Public only so the .cpp's config tables can be sized by them.
  static constexpr size_t kFcLinkTopicCount = 7u;
  static constexpr size_t kCrsfTopicCount = CrsfLinkService::kTopicCount;

  static TelemetryPublisher &GetInstance();

  void Poll(uint32_t now_us);

 private:
  friend class System;

  void Init(SharedState &blackboard, FcLink &fclink, CrsfLinkService &crsf,
            uint32_t now_us);

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
  message::EscTelemetryMsg BuildEscTelemetryMsg() const;
  message::UsbStatusMsg BuildUsbStatusMsg() const;
  message::GpsData BuildGpsMsg() const;
  message::AttitudeMsg BuildAttitudeMsg() const;
  // Tilt-compensated bearing of the corrected field, true; NaN while the
  // field is too short to point.
  float CompassHeading() const;

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
  static PublishResult PublishCrsfGpsTime(TelemetryPublisher &self,
                                    uint32_t now_us);
  static PublishResult PublishCrsfBaroAltitude(TelemetryPublisher &self,
                                               uint32_t now_us);
  static PublishResult PublishCrsfTemperature(TelemetryPublisher &self,
                                    uint32_t now_us);
  static PublishResult PublishCrsfBattery(TelemetryPublisher &self,
                                          uint32_t now_us);

  // Emit whichever topics of one group are due, up to `budget` frames.
  template <size_t N>
  void PollGroup(Group<N> &group, const std::array<Publish, N> &publishers,
                 uint8_t budget, uint32_t now_us);

  void FitCrsfLadder();

  // The CRSF group's periods as fitted to the link. A runtime copy of the
  // config table: the scheduler reads its span live, so rewriting a period
  // here is the whole mechanism.
  std::array<TopicConfig, kCrsfTopicCount> crsf_configs_{};
  // The rf_mode the ladder was last fitted to, -1 before any.
  int16_t applied_rf_mode_ = -1;
  SharedState *blackboard_ = nullptr;
  FcLink *fclink_svc_ = nullptr;
  CrsfLinkService *crsf_svc_ = nullptr;
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
