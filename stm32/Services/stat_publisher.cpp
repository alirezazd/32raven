// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "stat_publisher.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

#include "crsf_link_service.hpp"
#include "ctx.hpp"
#include "error_code.hpp"
#include "message.hpp"
#include "shared_state.hpp"
#include "slot_stagger.hpp"
#include "state_machine.hpp"
// EscConfigState has to be complete to convert to IState<AppContext>* for the
// state compare below; ctx.hpp only forward-declares it.
#include "states.hpp"  // IWYU pragma: keep
#include "stm32_config.hpp"
#include "system.hpp"

namespace {

// Was RcData::rx_online, recomputed on a tick. Derived at read time instead:
// silence needs no code, and each consumer is free to pick its own bound --
// this one only has to answer "is the GCS being shown a live link".
constexpr uint32_t kRcFreshTimeoutUs = 1500000u;

// The sample interrupt stamps ImuHealth every burst, so this is ~100 burst
// periods. Deliberately well past Sentinel's stall window: a path that stalls
// and is recovered should not blink the health bit on its way back.
constexpr uint32_t kImuFreshTimeoutUs = 100000u;

// Flattened in FcLinkTopic order -- the scheduler indexes this by the
// enumerator, so a row inserted out of order silently reschedules the wrong
// stream.
constexpr std::array<TopicConfig, StatPublisher::kFcLinkTopicCount>
    kFcLinkTopicConfigs = {{
        kStatPublisherConfig.system_status,
        kStatPublisherConfig.vehicle_status,
        kStatPublisherConfig.esc_telemetry,
        kStatPublisherConfig.rc_channels,
        kStatPublisherConfig.usb_status,
    }};

// In CrsfLinkService::TelemetryTopic order, on the same terms.
constexpr std::array<TopicConfig, StatPublisher::kCrsfTopicCount>
    kCrsfTopicConfigs = {{
        kStatPublisherConfig.crsf_heartbeat,
        kStatPublisherConfig.crsf_gps,
        kStatPublisherConfig.crsf_battery,
    }};

constexpr uint32_t kFcLinkPeriodsUs[] = {
    kStatPublisherConfig.system_status.period,
    kStatPublisherConfig.vehicle_status.period,
    kStatPublisherConfig.esc_telemetry.period,
    kStatPublisherConfig.rc_channels.period,
    kStatPublisherConfig.usb_status.period,
};

constexpr uint32_t kCrsfPeriodsUs[] = {
    kStatPublisherConfig.crsf_heartbeat.period,
    kStatPublisherConfig.crsf_gps.period,
    kStatPublisherConfig.crsf_battery.period,
};

static_assert(std::size(kFcLinkPeriodsUs) == StatPublisher::kFcLinkTopicCount,
              "a FcLinkTopic has no period here, so its offset goes unchecked");
static_assert(std::size(kCrsfPeriodsUs) == StatPublisher::kCrsfTopicCount,
              "a TelemetryTopic has no period here, so its offset goes "
              "unchecked");

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

}  // namespace

StatPublisher &StatPublisher::GetInstance() {
  static StatPublisher instance;
  return instance;
}

uint16_t StatPublisher::BatteryVoltageMv(const BatteryData &battery) {
  if (battery.voltage <= 0.0f) {
    return 0;
  }

  const uint32_t voltage_mv =
      static_cast<uint32_t>(std::lround(battery.voltage * 1000.0f));
  return voltage_mv > UINT16_MAX ? UINT16_MAX
                                 : static_cast<uint16_t>(voltage_mv);
}

int16_t StatPublisher::BatteryCurrentCa(const BatteryData &battery) {
  if (battery.voltage <= 0.0f) {
    return -1;
  }

  const int32_t current_ca = static_cast<int32_t>(battery.current * 100.0f);
  if (current_ca > INT16_MAX) {
    return INT16_MAX;
  }
  if (current_ca < INT16_MIN) {
    return INT16_MIN;
  }
  return static_cast<int16_t>(current_ca);
}

int8_t StatPublisher::BatteryRemainingPct(const BatteryData &battery) {
  if (battery.voltage <= 0.0f) {
    return -1;
  }
  return battery.percentage > 100u ? 100
                                   : static_cast<int8_t>(battery.percentage);
}

message::SystemStatusMsg StatPublisher::BuildSystemStatusMsg(
    const AppContext &ctx, uint32_t now_us, uint32_t loop_counter) {
  const SharedState &blackboard = ctx.sys->Blackboard();
  const GpsData &gps = blackboard.GetGps();
  const BatteryData &battery = blackboard.GetBattery();
  const RcData &rc = blackboard.GetRc();

  uint32_t sensors_present = 0;
  uint32_t sensors_health = 0;

  const ImuHealth &imu = blackboard.GetImuHealth();
  if (imu.timestamp_us != 0u) {
    sensors_present |= message::kSystemSensorFlagImu;
    if (imu.path_faults == 0u &&
        (now_us - imu.timestamp_us) <= kImuFreshTimeoutUs) {
      sensors_health |= message::kSystemSensorFlagImu;
    }
  }

  if (gps.timestamp_us != 0u) {
    sensors_present |= message::kSystemSensorFlagGps;
    if ((now_us - gps.timestamp_us) <= kGpsFreshTimeoutUs &&
        gps.fix_type >= 2u) {
      sensors_health |= message::kSystemSensorFlagGps;
    }
  }

  if (battery.voltage > 0.0f) {
    sensors_present |= message::kSystemSensorFlagBattery;
    sensors_health |= message::kSystemSensorFlagBattery;
  }

  if (rc.timestamp_us != 0u) {
    sensors_present |= message::kSystemSensorFlagRcReceiver;
    if ((now_us - rc.timestamp_us) <= kRcFreshTimeoutUs) {
      sensors_health |= message::kSystemSensorFlagRcReceiver;
    }
  }

  const EscTelemetryData &esc = blackboard.GetEscTelemetry();
  if (esc.valid_mask != 0u) {
    sensors_present |= message::kSystemSensorFlagEsc;
    if (esc.rx_dma_error_count == 0u && esc.uart_error_count == 0u) {
      sensors_health |= message::kSystemSensorFlagEsc;
    }
  }

  message::SystemStatusMsg msg{};
  msg.uptime_ms = ctx.sys->Blackboard().UptimeMs();
  msg.loop_counter = loop_counter;
  msg.error_code = static_cast<uint32_t>(ErrorCode::Common::kOk);
  msg.sensor_present_flags = sensors_present;
  msg.sensor_health_flags = sensors_health;
  msg.batt_voltage = BatteryVoltageMv(battery);
  msg.batt_current = BatteryCurrentCa(battery);
  msg.batt_remaining = BatteryRemainingPct(battery);
  msg.boot_state = static_cast<uint8_t>(message::BootState::kReady);
  msg.flags = message::kSystemStatusFlagLoopAlive;
  return msg;
}

message::VehicleStatusMsg StatPublisher::BuildVehicleStatusMsg(
    const AppContext &ctx) {
  message::VehicleStatusMsg msg{};
  msg.armed_state = static_cast<uint8_t>(ctx.sys->Blackboard().IsArmed()
                                             ? message::ArmedState::kArmed
                                             : message::ArmedState::kDisarmed);
  msg.failsafe_flags = ctx.sys->Blackboard().FailsafeFlags();
  msg.flight_mode = static_cast<uint8_t>(ctx.sys->Blackboard().GetFlightMode());
  return msg;
}

message::UsbStatusMsg StatPublisher::BuildUsbStatusMsg(const AppContext &ctx) {
  const UsbStatusData &usb = ctx.sys->Blackboard().GetUsbStatus();

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

StatPublisher::Outcome StatPublisher::PublishSystemStatus(StatPublisher &self,
                                                          const AppContext &ctx,
                                                          uint32_t now_us) {
  ctx.sys->FcLinkSvc().SendSystemStatus(
      BuildSystemStatusMsg(ctx, now_us, self.loop_counter_));
  return Outcome::kSent;
}

StatPublisher::Outcome StatPublisher::PublishVehicleStatus(StatPublisher &self,
                                                           const AppContext &ctx,
                                                           uint32_t now_us) {
  (void)self;
  (void)now_us;
  ctx.sys->FcLinkSvc().SendVehicleStatus(BuildVehicleStatusMsg(ctx));
  return Outcome::kSent;
}

StatPublisher::Outcome StatPublisher::PublishEscTelemetry(StatPublisher &self,
                                                          const AppContext &ctx,
                                                          uint32_t now_us) {
  (void)self;
  (void)now_us;
  const EscTelemetryData &esc = ctx.sys->Blackboard().GetEscTelemetry();
  if (esc.valid_mask == 0u) {
    return Outcome::kSkipped;
  }
  ctx.sys->FcLinkSvc().SendEscTelemetry(esc);
  return Outcome::kSent;
}

StatPublisher::Outcome StatPublisher::PublishRcChannels(StatPublisher &self,
                                                        const AppContext &ctx,
                                                        uint32_t now_us) {
  const RcData &rc = ctx.sys->Blackboard().GetRc();
  if (rc.timestamp_us == 0u) {
    return Outcome::kSkipped;
  }

  // The two frames age independently, so each is judged against its own
  // stamp -- channels can be current while the link report has gone quiet.
  const CrsfLinkData &link = ctx.sys->Blackboard().GetCrsfLink();
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
    return Outcome::kSkipped;
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
  ctx.sys->FcLinkSvc().SendRcChannels(msg);

  self.rc_sent_timestamp_us_ = rc.timestamp_us;
  self.rc_sent_flags_ = flags;
  self.have_rc_channels_ = true;
  return Outcome::kSent;
}

StatPublisher::Outcome StatPublisher::PublishUsbStatus(StatPublisher &self,
                                                       const AppContext &ctx,
                                                       uint32_t now_us) {
  // Only a USB bench session has anything to report -- outside one the
  // port is detached and every field reads zero. The silence is load bearing:
  // it is what the ESP32 times out on to learn the session never opened, so
  // the stream's presence is the grant and no flag has to carry it.
  if (ctx.sm->CurrentState() != ctx.esc_config_state &&
      ctx.sm->CurrentState() != ctx.msc_state) {
    // Drop the snapshot with it, or a session that reopens onto identical
    // values would have its first report suppressed as unchanged.
    self.have_usb_status_ = false;
    return Outcome::kSkipped;
  }

  // One compare: MspService stamps the Blackboard only when something moved.
  const UsbStatusData &usb = ctx.sys->Blackboard().GetUsbStatus();
  const bool unchanged =
      self.have_usb_status_ && usb.timestamp_us == self.usb_sent_timestamp_us_;
  if (unchanged && !self.fclink_.scheduler.SilenceExpired(
                       static_cast<size_t>(FcLinkTopic::kUsbStatus), now_us)) {
    return Outcome::kSkipped;
  }

  ctx.sys->FcLinkSvc().SendPacket(message::MsgId::kUsbStatus,
                                  BuildUsbStatusMsg(ctx));
  self.usb_sent_timestamp_us_ = usb.timestamp_us;
  self.have_usb_status_ = true;
  return Outcome::kSent;
}

StatPublisher::Outcome StatPublisher::PublishCrsfTopic(
    StatPublisher &self, const AppContext &ctx, uint32_t now_us,
    CrsfLinkService::TelemetryTopic topic) {
  const bool silence_expired =
      self.crsf_.scheduler.SilenceExpired(static_cast<size_t>(topic), now_us);
  switch (
      ctx.sys->CrsfLinkSvc().SendTelemetry(topic, silence_expired, now_us)) {
    case CrsfLinkService::TelemetryResult::kSent:
      return Outcome::kSent;
    case CrsfLinkService::TelemetryResult::kSkipped:
      return Outcome::kSkipped;
    case CrsfLinkService::TelemetryResult::kBlocked:
    default:
      return Outcome::kBlocked;
  }
}

StatPublisher::Outcome StatPublisher::PublishCrsfHeartbeat(StatPublisher &self,
                                                           const AppContext &ctx,
                                                           uint32_t now_us) {
  return PublishCrsfTopic(self, ctx, now_us,
                          CrsfLinkService::TelemetryTopic::kHeartbeat);
}

StatPublisher::Outcome StatPublisher::PublishCrsfGps(StatPublisher &self,
                                                     const AppContext &ctx,
                                                     uint32_t now_us) {
  return PublishCrsfTopic(self, ctx, now_us,
                          CrsfLinkService::TelemetryTopic::kGps);
}

StatPublisher::Outcome StatPublisher::PublishCrsfBattery(StatPublisher &self,
                                                         const AppContext &ctx,
                                                         uint32_t now_us) {
  return PublishCrsfTopic(self, ctx, now_us,
                          CrsfLinkService::TelemetryTopic::kBattery);
}

void StatPublisher::Init(const Config &cfg, uint32_t now_us) {
  cfg_ = cfg;
  fclink_.scheduler.Init(kFcLinkTopicConfigs, fclink_.states, kFcLinkStaggerUs,
                         now_us);
  crsf_.scheduler.Init(kCrsfTopicConfigs, crsf_.states, kCrsfStaggerUs, now_us);
  initialized_ = true;
}

template <size_t N>
void StatPublisher::PollGroup(Group<N> &group,
                              const std::array<Publish, N> &publishers,
                              uint8_t budget, const AppContext &ctx,
                              uint32_t now_us) {
  size_t sent = 0;
  while (sent < budget) {
    const std::optional<size_t> due = group.scheduler.NextDue(now_us);
    if (!due) {
      return;
    }

    switch (publishers[*due](*this, ctx, now_us)) {
      case Outcome::kSent:
        group.scheduler.MarkSent(*due, now_us);
        ++sent;
        break;
      case Outcome::kSkipped:
        group.scheduler.Skip(*due, now_us);
        break;
      case Outcome::kBlocked:
        return;
    }
  }
}

void StatPublisher::Poll(const AppContext &ctx, uint32_t now_us,
                         uint32_t loop_counter) {
  if (!initialized_) {
    return;
  }

  static constexpr std::array<Publish, kFcLinkTopicCount> kFcLinkPublishers = {
      PublishSystemStatus, PublishVehicleStatus, PublishEscTelemetry,
      PublishRcChannels,   PublishUsbStatus,
  };
  static constexpr std::array<Publish, kCrsfTopicCount> kCrsfPublishers = {
      PublishCrsfHeartbeat,
      PublishCrsfGps,
      PublishCrsfBattery,
  };

  loop_counter_ = loop_counter;

  PollGroup(fclink_, kFcLinkPublishers, cfg_.fclink_max_frames_per_poll, ctx,
            now_us);
  PollGroup(crsf_, kCrsfPublishers, cfg_.crsf_max_frames_per_poll, ctx, now_us);
}
