// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "telemetry_publisher.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "common_config.hpp"
#include "crsf_link_service.hpp"
#include "error_code.hpp"
#include "fc_link.hpp"
#include "message.hpp"
#include "shared_state.hpp"
#include "slot_stagger.hpp"
#include "stm32_config.hpp"

namespace {

// Was RcData::rx_online, recomputed on a tick. Derived at read time instead:
// silence needs no code, and each consumer is free to pick its own bound --
// this one only has to answer "is the GCS being shown a live link".
constexpr uint32_t kRcFreshTimeoutUs = 1500000u;
// ~200 control ticks at any configured rate. Generous on purpose: this only
// answers "is the ground station being shown a live loop", and an indicator
// that flickers on scheduling jitter is worse than one that lags.
constexpr uint32_t kControlLoopAliveTimeoutUs = 200000u;

// The sample interrupt stamps ImuHealth every burst, so this is ~100 burst
// periods. Deliberately well past Sentinel's stall window: a path that stalls
// and is recovered should not blink the health bit on its way back.
constexpr uint32_t kImuFreshTimeoutUs = 100000u;

// AM32 emits telemetry per commutation, so a running motor stamps this far
// faster. Sized for an idle disarmed ESC that still answers, not for the
// frame rate: below the fault window there would be nothing to compare.
constexpr uint32_t kEscFreshTimeoutUs = 1000000u;

// The FcLink ladder, fixed here rather than configured: the link is this
// project at both ends, and each cadence is what its consumer on the ESP32
// needs. Flattened in FcLinkTopic order -- the scheduler indexes this by the
// enumerator, so a row inserted out of order silently reschedules the wrong
// stream.
constexpr std::array<TopicConfig, TelemetryPublisher::kFcLinkTopicCount>
    kFcLinkTopicConfigs = {{
        // SystemStatus: the ESP32's heartbeat takes MAV_STATE from it and
        // holds a sample fresh for three seconds, so a second leaves a lost
        // frame inside that window.
        {.period = 1000000u, .max_silence = 0u, .priority = 10u},
        // VehicleStatus: armed state and failsafe flags, the one thing a
        // ground station must not learn late -- hence the highest priority,
        // and the heartbeat's own second.
        {.period = 1000000u, .max_silence = 0u, .priority = 12u},
        {.period = 1000000u, .max_silence = 0u, .priority = 5u},
        // RcChannels: the MAVLink RC_CHANNELS stream's own 40 ms. Sent on
        // change, so a quiet receiver costs nothing until the silence bound,
        // which is what lets the ESP32 tell held channels from a dead link.
        {.period = 40000u, .max_silence = 1000000u, .priority = 9u},
        // UsbStatus: sent when the MSP or four-way counters move; the silence
        // bound keeps the configurator UI from flapping on an idle port.
        {.period = 100000u, .max_silence = 300000u, .priority = 4u},
        // GpsData: sent on a new fix, so this only caps the rate, at the
        // M10's own measurement interval -- any slower would drop fixes.
        {.period = kM10Config.nav.rate_meas_ms * 1000u,
         .max_silence = 5000000u,
         .priority = 8u},
        // Attitude: a decimation of the control tick. The silence bound is
        // only reached with the loop suspended, which is when the ground
        // station most needs telling the estimate is stale.
        {.period = 100000u, .max_silence = 1000000u, .priority = 7u},
    }};

constexpr std::array<uint32_t, TelemetryPublisher::kFcLinkTopicCount>
    kFcLinkFrameBytes = {{
        sizeof(message::SystemStatusMsg) + message::kPacketOverhead,
        sizeof(message::VehicleStatusMsg) + message::kPacketOverhead,
        sizeof(message::EscTelemetryMsg) + message::kPacketOverhead,
        sizeof(message::RcChannelsMsg) + message::kPacketOverhead,
        sizeof(message::UsbStatusMsg) + message::kPacketOverhead,
        sizeof(message::GpsData) + message::kPacketOverhead,
        sizeof(message::AttitudeMsg) + message::kPacketOverhead,
    }};

// The ladder at full rate.
constexpr uint64_t FcLinkLadderBytesPerKs() {
  uint64_t bytes_per_ks = 0;
  for (size_t i = 0; i < TelemetryPublisher::kFcLinkTopicCount; ++i) {
    bytes_per_ks +=
        (static_cast<uint64_t>(kFcLinkFrameBytes[i]) * 1000000000ull) /
        kFcLinkTopicConfigs[i].period;
  }
  return bytes_per_ks;
}

// Held back from the line for what the ladder does not count -- logs,
// replies, the RC map.
constexpr uint64_t kFcLinkMarginPct = 80u;
static_assert(
    FcLinkLadderBytesPerKs() <=
        (static_cast<uint64_t>(common_config::kFcLinkBaud) / 10u) * 1000u *
            kFcLinkMarginPct / 100u,
    "the FcLink telemetry ladder needs more than the configured line carries "
    "-- raise COMMON_FCLINK_BAUD");

// The CRSF periods' ceiling: Kconfig's own, and what keeps a stretched
// period inside uint32_t and the scheduler's wrap-safe compare.
constexpr uint32_t kCrsfPeriodCeilingUs = 60000000u;

// How far the link budget may stretch a topic: to its max_silence, which
// already promises the handset a frame at least that often, or to the
// ceiling where it promises nothing.
constexpr uint32_t CrsfFloorUs(const TopicConfig &topic) {
  if (topic.max_silence == 0u) {
    return kCrsfPeriodCeilingUs;
  }
  return topic.max_silence > topic.period ? topic.max_silence : topic.period;
}

// Frame chunks per kilosecond a topic costs at one period.
constexpr uint64_t CrsfSlotsPerKs(uint32_t slots, uint32_t period_us) {
  return (static_cast<uint64_t>(slots) * 1000000000ull) / period_us;
}

// In CrsfLinkService::TelemetryTopic order, which is the order the scheduler
// indexes. The base ladder: what the link is fitted from, never what the
// scheduler reads.
constexpr std::array<TopicConfig, TelemetryPublisher::kCrsfTopicCount>
    kCrsfTopicConfigs = {{
        kTelemetryPublisherConfig.crsf_heartbeat,
        kTelemetryPublisherConfig.crsf_gps,
        kTelemetryPublisherConfig.crsf_battery,
        kTelemetryPublisherConfig.crsf_flight_mode,
        kTelemetryPublisherConfig.crsf_attitude,
        kTelemetryPublisherConfig.crsf_rpm,
        kTelemetryPublisherConfig.crsf_temperature,
        kTelemetryPublisherConfig.crsf_gps_time,
    }};

// Whether the ladder driven as slowly as its floors allow still fits a link.
// If not, no stretch can save it, so the declared link is refused at build
// time rather than discovered on the handset.
constexpr bool CrsfFloorsFit(const CrsfLinkService::TelemetryBudget &budget) {
  uint64_t needed_per_ks = 0;
  for (size_t i = 0; i < TelemetryPublisher::kCrsfTopicCount; ++i) {
    if (kCrsfTopicConfigs[i].period == 0u) {
      continue;
    }
    const uint8_t slots = CrsfLinkService::FrameSlots(
        static_cast<CrsfLinkService::TelemetryTopic>(i), budget.bytes_per_call);
    needed_per_ks += CrsfSlotsPerKs(slots, CrsfFloorUs(kCrsfTopicConfigs[i]));
  }
  return needed_per_ks <= budget.slots_per_ks;
}

constexpr std::optional<CrsfLinkService::TelemetryBudget> kDeclaredCrsfBudget =
    CrsfLinkService::BudgetFor(kCrsfLinkConfig.expected_rf_mode,
                               kCrsfLinkConfig.tlm_ratio_denom);
static_assert(
    kDeclaredCrsfBudget && CrsfFloorsFit(*kDeclaredCrsfBudget),
    "the CRSF ladder's floors need more than the configured link carries -- "
    "raise STM32_RC_RECEIVER_CRSF_TLM_RATIO to what the handset can give, "
    "loosen the STM32_RC_RECEIVER_CRSF_*_MAX_SILENCE_MS floors, or disable "
    "topics");

template <size_t N>
constexpr std::array<uint32_t, N> PeriodsOf(
    const std::array<TopicConfig, N> &topics) {
  std::array<uint32_t, N> periods{};
  for (size_t i = 0; i < N; ++i) {
    periods[i] = topics[i].period;
  }
  return periods;
}

constexpr std::array<uint32_t, TelemetryPublisher::kFcLinkTopicCount>
    kFcLinkPeriodsUs = PeriodsOf(kFcLinkTopicConfigs);
constexpr std::array<uint32_t, TelemetryPublisher::kCrsfTopicCount>
    kCrsfPeriodsUs = PeriodsOf(kCrsfTopicConfigs);

// Each group staggers within itself only -- they drive different UARTs, so a
// shared tick between groups costs nothing. Pick keeps the target while it
// clears every topic and moves to the next value up when a period is retuned
// onto it, so a knob change cannot quietly reinstate the collision.
constexpr uint32_t kFcLinkSpacingTargetUs = 7000u;
constexpr uint32_t kFcLinkStaggerUs =
    slot_stagger::Pick(kFcLinkSpacingTargetUs, kFcLinkPeriodsUs);
static_assert(kFcLinkStaggerUs != 0,
              "no spacing near the target clears every configured FcLink "
              "period, so a topic would open in phase with the ladder");

constexpr uint32_t kCrsfSpacingTargetUs = 200000u;
constexpr uint32_t kCrsfStaggerUs =
    slot_stagger::Pick(kCrsfSpacingTargetUs, kCrsfPeriodsUs);
static_assert(kCrsfStaggerUs != 0,
              "no spacing near the target clears every configured CRSF "
              "period, so a topic would open in phase with the ladder");

// Caps the burst when several FcLink topics come due together, at the cost
// of deferring the losers to the next poll: two lets a status pair leave in
// one tick while a late loop still cannot flood the line.
constexpr uint8_t kFcLinkFramesPerPoll = 2u;

// The CRSF link carries single-digit frames a second and the tick is 1 kHz,
// so a per-poll count configures nothing for that group; one keeps a stagger
// collision -- which stretching can create, the proof above being for the
// base ladder -- to a tick of latency.
constexpr uint8_t kCrsfFramesPerPoll = 1u;

// The CRSF ladder fitted to a budget. Every enabled topic stretches by one
// factor until it meets its floor, where it holds and its share comes off the
// budget the rest divide -- water-filling, at most one pass per topic. When
// the floors alone exceed the link the budget still wins and the floors
// stretch by one factor too: the build refuses that for the declared link, so
// it means a handset changed underneath the config, and a ladder over budget
// hands scheduling back to the receiver's FIFO, which is what this exists to
// prevent.
template <size_t N>
constexpr std::array<uint32_t, N> StretchPeriods(
    const std::array<TopicConfig, N> &base, const std::array<uint8_t, N> &slots,
    uint32_t budget_per_ks) {
  std::array<uint32_t, N> out{};
  std::array<bool, N> capped{};
  for (size_t i = 0; i < N; ++i) {
    out[i] = base[i].period;
  }
  if (budget_per_ks == 0u) {
    for (size_t i = 0; i < N; ++i) {
      if (base[i].period != 0u) {
        out[i] = kCrsfPeriodCeilingUs;
      }
    }
    return out;
  }

  for (size_t pass = 0; pass < N; ++pass) {
    uint64_t required = 0;
    uint64_t held = 0;
    for (size_t i = 0; i < N; ++i) {
      if (base[i].period == 0u) {
        continue;
      }
      if (capped[i]) {
        held += CrsfSlotsPerKs(slots[i], CrsfFloorUs(base[i]));
      } else {
        required += CrsfSlotsPerKs(slots[i], base[i].period);
      }
    }

    if (held >= budget_per_ks) {
      uint64_t floors = 0;
      for (size_t i = 0; i < N; ++i) {
        if (base[i].period != 0u) {
          floors += CrsfSlotsPerKs(slots[i], CrsfFloorUs(base[i]));
        }
      }
      for (size_t i = 0; i < N; ++i) {
        if (base[i].period == 0u) {
          continue;
        }
        const uint64_t stretched =
            (static_cast<uint64_t>(CrsfFloorUs(base[i])) * floors) /
            budget_per_ks;
        out[i] = stretched > kCrsfPeriodCeilingUs
                     ? kCrsfPeriodCeilingUs
                     : static_cast<uint32_t>(stretched);
      }
      return out;
    }

    const uint64_t room = budget_per_ks - held;
    if (required <= room) {
      for (size_t i = 0; i < N; ++i) {
        if (!capped[i]) {
          out[i] = base[i].period;
        }
      }
      return out;
    }

    bool moved = false;
    for (size_t i = 0; i < N; ++i) {
      if (base[i].period == 0u || capped[i]) {
        continue;
      }
      const uint64_t stretched =
          (static_cast<uint64_t>(base[i].period) * required) / room;
      const uint32_t floor = CrsfFloorUs(base[i]);
      if (stretched > floor) {
        capped[i] = true;
        moved = true;
        out[i] = floor;
      } else {
        out[i] = static_cast<uint32_t>(stretched);
      }
    }
    if (!moved) {
      return out;
    }
  }
  return out;
}

// A three-topic ladder: 2 slots every 200 ms with a 2 s floor, 4 every 500 ms
// with a 2 s floor, 2 every second with none. 20 slots/s at base.
constexpr std::array<TopicConfig, 3> kStretchLadder = {{
    {.period = 200000u, .max_silence = 2000000u, .priority = 0u},
    {.period = 500000u, .max_silence = 2000000u, .priority = 0u},
    {.period = 1000000u, .max_silence = 0u, .priority = 0u},
}};
constexpr std::array<uint8_t, 3> kStretchSlots = {2u, 4u, 2u};
static_assert(StretchPeriods(kStretchLadder, kStretchSlots, 20000u) ==
                  std::array<uint32_t, 3>{200000u, 500000u, 1000000u},
              "a ladder that fits is left alone");
static_assert(StretchPeriods(kStretchLadder, kStretchSlots, 10000u) ==
                  std::array<uint32_t, 3>{400000u, 1000000u, 2000000u},
              "half the budget doubles every period, inside every floor");
static_assert(StretchPeriods(kStretchLadder, kStretchSlots, 4000u) ==
                  std::array<uint32_t, 3>{1200000u, 2000000u, 6000000u},
              "a topic at its floor holds, and the rest absorb its share");
static_assert(StretchPeriods(kStretchLadder, kStretchSlots, 1000u) ==
                  std::array<uint32_t, 3>{6066000u, 6066000u, 60000000u},
              "below the floors' own need they stretch too, the floorless "
              "topic to the ceiling");

}  // namespace

TelemetryPublisher &TelemetryPublisher::GetInstance() {
  static TelemetryPublisher instance;
  return instance;
}

uint16_t TelemetryPublisher::BatteryVoltageMv(const BatteryData &battery) {
  if (battery.voltage <= 0.0f) {
    return 0;
  }

  const uint32_t voltage_mv =
      static_cast<uint32_t>(std::lround(battery.voltage * 1000.0f));
  return voltage_mv > UINT16_MAX ? UINT16_MAX
                                 : static_cast<uint16_t>(voltage_mv);
}

int16_t TelemetryPublisher::BatteryCurrentCa(const BatteryData &battery) {
  // -1 already means "no pack" here and is also MAVLink's unknown, which the
  // ESP32 forwards into SYS_STATUS and BATTERY_STATUS unchanged.
  if (battery.voltage <= 0.0f || !battery.current.has_value()) {
    return -1;
  }

  const int32_t current_ca = static_cast<int32_t>(*battery.current * 100.0f);
  if (current_ca > INT16_MAX) {
    return INT16_MAX;
  }
  if (current_ca < INT16_MIN) {
    return INT16_MIN;
  }
  return static_cast<int16_t>(current_ca);
}

int8_t TelemetryPublisher::BatteryRemainingPct(const BatteryData &battery) {
  if (battery.voltage <= 0.0f) {
    return -1;
  }
  return battery.percentage > 100u ? 100
                                   : static_cast<int8_t>(battery.percentage);
}

// A duty cycle over the window since the last publish, not a per-tick figure:
// the reader owns the window, so a missed publish widens it rather than
// corrupting it. The counter only grows, so this never writes what the control
// tick is concurrently adding to.
uint16_t TelemetryPublisher::ComputeControlLoopLoad() {
  const uint32_t busy = blackboard_->GetControlLoopLoad().busy_cycles;
  const uint32_t now = TimeBase::Cycles();
  const uint32_t window = now - load_window_start_cycles_;
  const uint32_t spent = busy - load_last_busy_cycles_;
  load_window_start_cycles_ = now;
  load_last_busy_cycles_ = busy;
  if (window == 0u) {
    return 0u;
  }
  const uint64_t per_mille =
      (static_cast<uint64_t>(spent) * 1000u) / static_cast<uint64_t>(window);
  return static_cast<uint16_t>(per_mille > 1000u ? 1000u : per_mille);
}

message::SystemStatusMsg TelemetryPublisher::BuildSystemStatusMsg(
    uint32_t now_us, uint16_t load) const {
  const SharedState &blackboard = *blackboard_;
  const GpsData &gps = blackboard.GetGps();
  const BatteryData &battery = blackboard.GetBattery();
  const RcData &rc = blackboard.GetRc();

  uint32_t sensors_present = 0;
  uint32_t sensors_health = 0;

  const ImuHealth &imu = blackboard.GetImuHealth();
  if (imu.timestamp_us != 0u) {
    sensors_present |= message::kSystemSensorFlagImu;
    if (IsHealthy(FaultSource::kImu) &&
        (now_us - imu.timestamp_us) <= kImuFreshTimeoutUs) {
      sensors_health |= message::kSystemSensorFlagImu;
    }
  }

  if (gps.timestamp_us != 0u) {
    sensors_present |= message::kSystemSensorFlagGps;
    if ((now_us - gps.timestamp_us) <= kGpsFreshTimeoutUs &&
        gps.fix_type >= 2u && IsHealthy(FaultSource::kGps)) {
      sensors_health |= message::kSystemSensorFlagGps;
    }
  }

  if (battery.voltage > 0.0f) {
    sensors_present |= message::kSystemSensorFlagBattery;
    if ((now_us - battery.timestamp_us) <= kBatteryFreshTimeoutUs &&
        IsHealthy(FaultSource::kBattery)) {
      sensors_health |= message::kSystemSensorFlagBattery;
    }
  }

  if (rc.timestamp_us != 0u) {
    sensors_present |= message::kSystemSensorFlagRcReceiver;
    if ((now_us - rc.timestamp_us) <= kRcFreshTimeoutUs &&
        IsHealthy(FaultSource::kRc)) {
      sensors_health |= message::kSystemSensorFlagRcReceiver;
    }
  }

  const EscTelemetryData &esc = blackboard.GetEscTelemetry();
  if (esc.valid_mask != 0u) {
    sensors_present |= message::kSystemSensorFlagEsc;
    if ((now_us - esc.timestamp_us) <= kEscFreshTimeoutUs &&
        IsHealthy(FaultSource::kEsc)) {
      sensors_health |= message::kSystemSensorFlagEsc;
    }
  }

  message::SystemStatusMsg msg{};
  msg.uptime_ms = blackboard_->UptimeMs();
  // Zeroed with the loop suspended: the last flight's count would otherwise
  // read as this bench session's.
  const bool loop_running = blackboard.IsControlLoopRunning();
  msg.loop_counter = loop_running ? blackboard.MainTickCount() : 0u;
  msg.control_loop_load = load;
  msg.error_code = static_cast<uint32_t>(ErrorCode::Common::kOk);
  msg.sensor_present_flags = sensors_present;
  msg.sensor_health_flags = sensors_health;
  msg.batt_voltage = BatteryVoltageMv(battery);
  msg.batt_current = BatteryCurrentCa(battery);
  msg.batt_remaining = BatteryRemainingPct(battery);
  msg.boot_state = static_cast<uint8_t>(
      loop_running ? message::BootState::kReady : message::BootState::kBooting);
  // Read rather than asserted: the ESP32 gates MAV_STATE_CRITICAL on this,
  // and a bit set unconditionally told every ground indicator that a stopped
  // control loop was fine.
  const bool loop_alive =
      loop_running && (now_us - blackboard.GetControlLoopLoad().timestamp_us) <
                          kControlLoopAliveTimeoutUs;
  msg.flags = loop_alive ? message::kSystemStatusFlagLoopAlive : 0u;
  return msg;
}

message::VehicleStatusMsg TelemetryPublisher::BuildVehicleStatusMsg() const {
  message::VehicleStatusMsg msg{};
  msg.armed_state = static_cast<uint8_t>(blackboard_->IsArmed()
                                             ? message::ArmedState::kArmed
                                             : message::ArmedState::kDisarmed);
  msg.failsafe_flags = blackboard_->FailsafeFlags();
  msg.flight_mode = static_cast<uint8_t>(blackboard_->GetFlightMode());
  return msg;
}

message::UsbStatusMsg TelemetryPublisher::BuildUsbStatusMsg() const {
  const UsbStatusData &usb = blackboard_->GetUsbStatus();

  uint8_t flags = 0u;
  if (usb.attached) flags |= message::kUsbStatusAttached;
  if (usb.configured) flags |= message::kUsbStatusConfigured;
  if (usb.port_open) flags |= message::kUsbStatusPortOpen;

  // The states that set these two flags are mutually exclusive, so the pair
  // cannot both be true.
  const message::UsbMode mode = usb.esc_config_granted
                                    ? message::UsbMode::kEscConfig
                                : usb.msc_active ? message::UsbMode::kMsc
                                                 : message::UsbMode::kNone;

  // Both dialects share the port, so their frames share one count -- the
  // reader is drawing "data moved", not attributing it to a protocol. They
  // stay apart on the Blackboard; merging is this message's view of them.
  return message::UsbStatusMsg{
      .flags = flags,
      .mode = static_cast<uint8_t>(mode),
      .rx_frames =
          static_cast<uint8_t>(usb.msp_requests + usb.four_way_requests),
      .tx_frames = static_cast<uint8_t>(usb.msp_replies + usb.four_way_replies),
  };
}

TelemetryPublisher::PublishResult TelemetryPublisher::PublishSystemStatus(
    TelemetryPublisher &self, uint32_t now_us) {
  self.fclink_svc_->SendSystemStatus(
      self.BuildSystemStatusMsg(now_us, self.ComputeControlLoopLoad()));
  return PublishResult::kSent;
}

TelemetryPublisher::PublishResult TelemetryPublisher::PublishVehicleStatus(
    TelemetryPublisher &self, uint32_t now_us) {
  (void)self;
  (void)now_us;
  self.fclink_svc_->SendVehicleStatus(self.BuildVehicleStatusMsg());
  return PublishResult::kSent;
}

TelemetryPublisher::PublishResult TelemetryPublisher::PublishEscTelemetry(
    TelemetryPublisher &self, uint32_t now_us) {
  (void)self;
  (void)now_us;
  const EscTelemetryData &esc = self.blackboard_->GetEscTelemetry();
  if (esc.valid_mask == 0u) {
    return PublishResult::kSkipped;
  }
  self.fclink_svc_->SendEscTelemetry(esc);
  return PublishResult::kSent;
}

TelemetryPublisher::PublishResult TelemetryPublisher::PublishRcChannels(
    TelemetryPublisher &self, uint32_t now_us) {
  const RcData &rc = self.blackboard_->GetRc();
  if (rc.timestamp_us == 0u) {
    return PublishResult::kSkipped;
  }

  // The two frames age independently, so each is judged against its own
  // stamp -- channels can be current while the link report has gone quiet.
  const CrsfLinkData &link = self.blackboard_->GetCrsfLink();
  const bool rc_fresh = (now_us - rc.timestamp_us) <= kRcFreshTimeoutUs;
  const bool link_fresh = link.timestamp_us != 0u &&
                          (now_us - link.timestamp_us) <= kRcFreshTimeoutUs;

  uint8_t flags = 0u;
  if (rc_fresh) {
    flags |= message::kRcChannelsFlagRxOnline;
  }

  // A receiver that stopped producing frames leaves the channels frozen, and
  // resending them would read as a live link. The stream stops until the flags
  // move or the silence window forces a keepalive.
  const bool unchanged = self.have_rc_channels_ &&
                         rc.timestamp_us == self.rc_sent_timestamp_us_ &&
                         flags == self.rc_sent_flags_;
  if (unchanged && !self.fclink_.scheduler.SilenceExpired(
                       static_cast<size_t>(FcLinkTopic::kRcChannels), now_us)) {
    return PublishResult::kSkipped;
  }

  message::RcChannelsMsg msg{};
  // Raw, not calibrated: MAVLink RC_CHANNELS names these fields chanN_raw and
  // a ground station reads them as what the receiver sent. Calibrated values
  // would hide a reversed channel or a mis-set endpoint behind the correction.
  for (std::size_t i = 0; i < message::kRcChannelCount; ++i) {
    msg.channels[i] = rc.channels_raw[i];
  }
  msg.link_quality = link_fresh ? link.uplink_link_quality : 0u;
  msg.flags = flags;
  self.fclink_svc_->SendRcChannels(msg);

  self.rc_sent_timestamp_us_ = rc.timestamp_us;
  self.rc_sent_flags_ = flags;
  self.have_rc_channels_ = true;
  return PublishResult::kSent;
}

TelemetryPublisher::PublishResult TelemetryPublisher::PublishUsbStatus(
    TelemetryPublisher &self, uint32_t now_us) {
  // Only a USB bench session has anything to report -- outside one the
  // port is detached and every field reads zero. The silence is load bearing:
  // it is what the ESP32 times out on to learn the session never opened, so
  // the stream's presence is the grant and no flag has to carry it.
  const UsbStatusData &granted = self.blackboard_->GetUsbStatus();
  if (!granted.esc_config_granted && !granted.msc_active) {
    // Drop the snapshot with it, or a session that reopens onto identical
    // values would have its first report suppressed as unchanged.
    self.have_usb_status_ = false;
    return PublishResult::kSkipped;
  }

  // One compare: MspService stamps the Blackboard only when something moved.
  const UsbStatusData &usb = self.blackboard_->GetUsbStatus();
  const bool unchanged =
      self.have_usb_status_ && usb.timestamp_us == self.usb_sent_timestamp_us_;
  if (unchanged && !self.fclink_.scheduler.SilenceExpired(
                       static_cast<size_t>(FcLinkTopic::kUsbStatus), now_us)) {
    return PublishResult::kSkipped;
  }

  self.fclink_svc_->SendPacket(message::MsgId::kUsbStatus,
                               self.BuildUsbStatusMsg());
  self.usb_sent_timestamp_us_ = usb.timestamp_us;
  self.have_usb_status_ = true;
  return PublishResult::kSent;
}

message::GpsData TelemetryPublisher::BuildGpsMsg() const {
  const GpsData &gps = blackboard_->GetGps();
  message::GpsData msg{};
  msg.year = gps.year;
  msg.month = gps.month;
  msg.day = gps.day;
  msg.hour = gps.hour;
  msg.min = gps.min;
  msg.sec = gps.sec;
  msg.fixType = gps.fix_type;
  msg.numSV = gps.num_sats;
  msg.lon = gps.lon;
  msg.lat = gps.lat;
  msg.hMSL = gps.alt;
  msg.vel = gps.vel;
  msg.hdg = gps.hdg;
  msg.hAcc = gps.hAcc;
  msg.vAcc = gps.vAcc;
  msg.gDOP = gps.gDOP;
  msg.pDOP = gps.pDOP;
  msg.hDOP = gps.hDOP;
  msg.vDOP = gps.vDOP;
  msg.posCovValid = gps.posCovValid;
  msg.velCovValid = gps.velCovValid;
  msg.posCovNN = gps.posCovNN;
  msg.posCovEE = gps.posCovEE;
  msg.posCovDD = gps.posCovDD;
  return msg;
}

message::AttitudeMsg TelemetryPublisher::BuildAttitudeMsg() const {
  const EstimatorState &estimate = blackboard_->GetEstimate();
  const Eigen::Quaternionf &q = estimate.attitude_world_to_body;
  return message::AttitudeMsg{
      .timestamp_us = estimate.timestamp_us,
      .qw = q.w(),
      .qx = q.x(),
      .qy = q.y(),
      .qz = q.z(),
  };
}

TelemetryPublisher::PublishResult TelemetryPublisher::PublishGps(
    TelemetryPublisher &self, uint32_t now_us) {
  const GpsData &gps = self.blackboard_->GetGps();
  if (gps.timestamp_us == 0u) {
    return PublishResult::kSkipped;
  }

  // The receiver stamps only on a new PVT, so a resent fix would read as a
  // position re-observed rather than re-read.
  const bool unchanged =
      self.have_gps_ && gps.timestamp_us == self.gps_sent_timestamp_us_;
  if (unchanged && !self.fclink_.scheduler.SilenceExpired(
                       static_cast<size_t>(FcLinkTopic::kGps), now_us)) {
    return PublishResult::kSkipped;
  }

  self.fclink_svc_->SendPacket(message::MsgId::kGpsData, self.BuildGpsMsg());
  self.gps_sent_timestamp_us_ = gps.timestamp_us;
  self.have_gps_ = true;
  return PublishResult::kSent;
}

TelemetryPublisher::PublishResult TelemetryPublisher::PublishAttitude(
    TelemetryPublisher &self, uint32_t now_us) {
  const EstimatorState &estimate = self.blackboard_->GetEstimate();
  if (estimate.timestamp_us == 0u) {
    return PublishResult::kSkipped;
  }

  // The control tick writes this faster than the link can carry it, so the
  // period is a decimation rather than a rate. A suspended loop freezes the
  // stamp, which the silence window then reports.
  const bool unchanged =
      self.have_attitude_ &&
      estimate.timestamp_us == self.attitude_sent_timestamp_us_;
  if (unchanged && !self.fclink_.scheduler.SilenceExpired(
                       static_cast<size_t>(FcLinkTopic::kAttitude), now_us)) {
    return PublishResult::kSkipped;
  }

  self.fclink_svc_->SendPacket(message::MsgId::kAttitude,
                               self.BuildAttitudeMsg());
  self.attitude_sent_timestamp_us_ = estimate.timestamp_us;
  self.have_attitude_ = true;
  return PublishResult::kSent;
}

TelemetryPublisher::PublishResult TelemetryPublisher::PublishCrsfTopic(
    TelemetryPublisher &self, uint32_t now_us,
    CrsfLinkService::TelemetryTopic topic) {
  const bool silence_expired =
      self.crsf_.scheduler.SilenceExpired(static_cast<size_t>(topic), now_us);
  switch (self.crsf_svc_->SendTelemetry(topic, silence_expired, now_us)) {
    case CrsfLinkService::TelemetryResult::kSent:
      return PublishResult::kSent;
    case CrsfLinkService::TelemetryResult::kSkipped:
      return PublishResult::kSkipped;
    case CrsfLinkService::TelemetryResult::kBlocked:
    default:
      return PublishResult::kBlocked;
  }
}

TelemetryPublisher::PublishResult TelemetryPublisher::PublishCrsfHeartbeat(
    TelemetryPublisher &self, uint32_t now_us) {
  return PublishCrsfTopic(self, now_us,
                          CrsfLinkService::TelemetryTopic::kHeartbeat);
}

TelemetryPublisher::PublishResult TelemetryPublisher::PublishCrsfGps(
    TelemetryPublisher &self, uint32_t now_us) {
  return PublishCrsfTopic(self, now_us, CrsfLinkService::TelemetryTopic::kGps);
}

TelemetryPublisher::PublishResult TelemetryPublisher::PublishCrsfBattery(
    TelemetryPublisher &self, uint32_t now_us) {
  return PublishCrsfTopic(self, now_us,
                          CrsfLinkService::TelemetryTopic::kBattery);
}

TelemetryPublisher::PublishResult TelemetryPublisher::PublishCrsfFlightMode(
    TelemetryPublisher &self, uint32_t now_us) {
  return PublishCrsfTopic(self, now_us,
                          CrsfLinkService::TelemetryTopic::kFlightMode);
}

TelemetryPublisher::PublishResult TelemetryPublisher::PublishCrsfAttitude(
    TelemetryPublisher &self, uint32_t now_us) {
  return PublishCrsfTopic(self, now_us,
                          CrsfLinkService::TelemetryTopic::kAttitude);
}

TelemetryPublisher::PublishResult TelemetryPublisher::PublishCrsfRpm(
    TelemetryPublisher &self, uint32_t now_us) {
  return PublishCrsfTopic(self, now_us,
                          CrsfLinkService::TelemetryTopic::kRpm);
}

TelemetryPublisher::PublishResult TelemetryPublisher::PublishCrsfTemperature(
    TelemetryPublisher &self, uint32_t now_us) {
  return PublishCrsfTopic(self, now_us,
                          CrsfLinkService::TelemetryTopic::kTemperature);
}

TelemetryPublisher::PublishResult TelemetryPublisher::PublishCrsfGpsTime(
    TelemetryPublisher &self, uint32_t now_us) {
  return PublishCrsfTopic(self, now_us,
                          CrsfLinkService::TelemetryTopic::kGpsTime);
}

void TelemetryPublisher::Init(SharedState &blackboard, FcLink &fclink,
                              CrsfLinkService &crsf, uint32_t now_us) {
  blackboard_ = &blackboard;
  fclink_svc_ = &fclink;
  crsf_svc_ = &crsf;
  last_link_window_us_ = now_us;
  fclink_.scheduler.Init(kFcLinkTopicConfigs, fclink_.states, kFcLinkStaggerUs,
                         now_us);
  crsf_configs_ = kCrsfTopicConfigs;
  crsf_.scheduler.Init(crsf_configs_, crsf_.states, kCrsfStaggerUs, now_us);
  initialized_ = true;
  FitCrsfLadder();
}

template <size_t N>
void TelemetryPublisher::PollGroup(Group<N> &group,
                                   const std::array<Publish, N> &publishers,
                                   uint8_t budget, uint32_t now_us) {
  size_t sent = 0;
  while (sent < budget) {
    const std::optional<size_t> due = group.scheduler.NextDue(now_us);
    if (!due) {
      return;
    }

    switch (publishers[*due](*this, now_us)) {
      case PublishResult::kSent:
        group.scheduler.MarkSent(*due, now_us);
        ++sent;
        break;
      case PublishResult::kSkipped:
        group.scheduler.Skip(*due, now_us);
        break;
      case PublishResult::kBlocked:
        return;
    }
  }
}

void TelemetryPublisher::UpdateFaultWindows(uint32_t now_us) {
  if ((now_us - last_link_window_us_) < kLinkErrorWindowUs) {
    return;
  }
  last_link_window_us_ = now_us;

  const SystemHealth &health = blackboard_->GetSystemHealth();

  // Ordered by FaultSource. Transport plus the parser above it, because a peer
  // emitting garbage over a flawless UART is not healthy. The last three come
  // pre-summed: path_faults already folds the IMU's bus in beside its parser,
  // the ESC owns its transport, and nothing frames an ADC reading.
  const std::array<uint32_t, std::to_underlying(FaultSource::kCount)> totals = {
      blackboard_->GetImuHealth().path_faults,
      health.gps_uart.Total() + blackboard_->GetGps().checksum_failures,
      health.rc_uart.Total() + blackboard_->GetCrsfLink().checksum_failures,
      blackboard_->GetEscTelemetry().Total(),
      health.batt_adc.Total(),
  };

  for (size_t i = 0; i < totals.size(); ++i) {
    fault_windows_[i].healthy = totals[i] == fault_windows_[i].last_total;
    fault_windows_[i].last_total = totals[i];
  }
}

void TelemetryPublisher::Poll(uint32_t now_us) {
  UpdateFaultWindows(now_us);
  FitCrsfLadder();

  static constexpr std::array<Publish, kFcLinkTopicCount> kFcLinkPublishers = {
      PublishSystemStatus, PublishVehicleStatus, PublishEscTelemetry,
      PublishRcChannels,   PublishUsbStatus,     PublishGps,
      PublishAttitude,
  };
  static constexpr std::array<Publish, kCrsfTopicCount> kCrsfPublishers = {
      PublishCrsfHeartbeat,   PublishCrsfGps,      PublishCrsfBattery,
      PublishCrsfFlightMode,  PublishCrsfAttitude, PublishCrsfRpm,
      PublishCrsfTemperature, PublishCrsfGpsTime,
  };

  PollGroup(fclink_, kFcLinkPublishers, kFcLinkFramesPerPoll, now_us);
  PollGroup(crsf_, kCrsfPublishers, kCrsfFramesPerPoll, now_us);
}

// The declared rate stands in until the receiver reports one -- nothing is
// listening before then, and it is the best knowledge there is. A rate the
// table does not know leaves the ladder as configured: a guess would be
// worse than honesty, and the log says which rate it was.
void TelemetryPublisher::FitCrsfLadder() {
  const CrsfLinkData &link = blackboard_->GetCrsfLink();
  const uint8_t expected = crsf_svc_->ExpectedRfMode();
  const uint8_t rf_mode = link.timestamp_us != 0u ? link.rf_mode : expected;
  if (rf_mode == applied_rf_mode_) {
    return;
  }
  applied_rf_mode_ = rf_mode;

  const std::optional<CrsfLinkService::TelemetryBudget> budget =
      crsf_svc_->BudgetFor(rf_mode);
  if (!budget) {
    for (size_t i = 0; i < kCrsfTopicCount; ++i) {
      crsf_configs_[i].period = kCrsfTopicConfigs[i].period;
    }
    fclink_svc_->SendLog("crsf: rf_mode %u unknown, ladder unshed", rf_mode);
    return;
  }

  std::array<uint8_t, kCrsfTopicCount> slots{};
  uint64_t needed = 0;
  uint64_t floors = 0;
  for (size_t i = 0; i < kCrsfTopicCount; ++i) {
    slots[i] = CrsfLinkService::FrameSlots(
        static_cast<CrsfLinkService::TelemetryTopic>(i),
        budget->bytes_per_call);
    if (kCrsfTopicConfigs[i].period == 0u) {
      continue;
    }
    needed += CrsfSlotsPerKs(slots[i], kCrsfTopicConfigs[i].period);
    floors += CrsfSlotsPerKs(slots[i], CrsfFloorUs(kCrsfTopicConfigs[i]));
  }
  const std::array<uint32_t, kCrsfTopicCount> periods =
      StretchPeriods(kCrsfTopicConfigs, slots, budget->slots_per_ks);
  for (size_t i = 0; i < kCrsfTopicCount; ++i) {
    crsf_configs_[i].period = periods[i];
  }

  // Slots per second to one decimal, from thousandths.
  fclink_svc_->SendLog(
      "crsf: rf_mode %u (%u Hz 1:%u) ladder %lu.%lu of %lu.%lu slots/s%s%s",
      rf_mode, budget->packet_hz, budget->denom,
      static_cast<unsigned long>(needed / 1000u),
      static_cast<unsigned long>((needed % 1000u) / 100u),
      static_cast<unsigned long>(budget->slots_per_ks / 1000u),
      static_cast<unsigned long>((budget->slots_per_ks % 1000u) / 100u),
      floors > budget->slots_per_ks ? ", floors over budget" : "",
      rf_mode != expected ? ", not the configured rate" : "");
}
