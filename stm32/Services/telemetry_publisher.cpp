// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "telemetry_publisher.hpp"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

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

// The sample interrupt stamps ImuHealth every burst, so this is ~100 burst
// periods. Deliberately well past Sentinel's stall window: a path that stalls
// and is recovered should not blink the health bit on its way back.
constexpr uint32_t kImuFreshTimeoutUs = 100000u;

// AM32 emits telemetry per commutation, so a running motor stamps this far
// faster. Sized for an idle disarmed ESC that still answers, not for the
// frame rate: below the fault window there would be nothing to compare.
constexpr uint32_t kEscFreshTimeoutUs = 1000000u;

// Flattened in FcLinkTopic order -- the scheduler indexes this by the
// enumerator, so a row inserted out of order silently reschedules the wrong
// stream.
constexpr std::array<TopicConfig, TelemetryPublisher::kFcLinkTopicCount>
    kFcLinkTopicConfigs = {{
        kTelemetryPublisherConfig.system_status,
        kTelemetryPublisherConfig.vehicle_status,
        kTelemetryPublisherConfig.esc_telemetry,
        kTelemetryPublisherConfig.rc_channels,
        kTelemetryPublisherConfig.usb_status,
        kTelemetryPublisherConfig.gps,
        kTelemetryPublisherConfig.attitude,
    }};

// In CrsfLinkService::TelemetryTopic order, on the same terms.
constexpr std::array<TopicConfig, TelemetryPublisher::kCrsfTopicCount>
    kCrsfTopicConfigs = {{
        kTelemetryPublisherConfig.crsf_heartbeat,
        kTelemetryPublisherConfig.crsf_gps,
        kTelemetryPublisherConfig.crsf_battery,
        kTelemetryPublisherConfig.crsf_flight_mode,
        kTelemetryPublisherConfig.crsf_attitude,
        kTelemetryPublisherConfig.crsf_rpm,
        kTelemetryPublisherConfig.crsf_temperature,
    }};

constexpr uint32_t kFcLinkPeriodsUs[] = {
    kTelemetryPublisherConfig.system_status.period,
    kTelemetryPublisherConfig.vehicle_status.period,
    kTelemetryPublisherConfig.esc_telemetry.period,
    kTelemetryPublisherConfig.rc_channels.period,
    kTelemetryPublisherConfig.usb_status.period,
    kTelemetryPublisherConfig.gps.period,
    kTelemetryPublisherConfig.attitude.period,
};

constexpr uint32_t kCrsfPeriodsUs[] = {
    kTelemetryPublisherConfig.crsf_heartbeat.period,
    kTelemetryPublisherConfig.crsf_gps.period,
    kTelemetryPublisherConfig.crsf_battery.period,
    kTelemetryPublisherConfig.crsf_flight_mode.period,
    kTelemetryPublisherConfig.crsf_attitude.period,
    kTelemetryPublisherConfig.crsf_rpm.period,
    kTelemetryPublisherConfig.crsf_temperature.period,
};

static_assert(std::size(kFcLinkPeriodsUs) ==
                  TelemetryPublisher::kFcLinkTopicCount,
              "a FcLinkTopic has no period here, so its offset goes unchecked");
static_assert(std::size(kCrsfPeriodsUs) == TelemetryPublisher::kCrsfTopicCount,
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
  msg.flags = message::kSystemStatusFlagLoopAlive;
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

void TelemetryPublisher::Init(const Config &cfg, SharedState &blackboard,
                              FcLink &fclink, CrsfLinkService &crsf,
                              uint32_t now_us) {
  cfg_ = cfg;
  blackboard_ = &blackboard;
  fclink_svc_ = &fclink;
  crsf_svc_ = &crsf;
  last_link_window_us_ = now_us;
  fclink_.scheduler.Init(kFcLinkTopicConfigs, fclink_.states, kFcLinkStaggerUs,
                         now_us);
  crsf_.scheduler.Init(kCrsfTopicConfigs, crsf_.states, kCrsfStaggerUs, now_us);
  initialized_ = true;
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
  if (!initialized_) {
    return;
  }

  UpdateFaultWindows(now_us);

  static constexpr std::array<Publish, kFcLinkTopicCount> kFcLinkPublishers = {
      PublishSystemStatus, PublishVehicleStatus, PublishEscTelemetry,
      PublishRcChannels,   PublishUsbStatus,     PublishGps,
      PublishAttitude,
  };
  static constexpr std::array<Publish, kCrsfTopicCount> kCrsfPublishers = {
      PublishCrsfHeartbeat,   PublishCrsfGps,      PublishCrsfBattery,
      PublishCrsfFlightMode,  PublishCrsfAttitude, PublishCrsfRpm,
      PublishCrsfTemperature,
  };

  PollGroup(fclink_, kFcLinkPublishers, cfg_.fclink_max_frames_per_poll,
            now_us);
  PollGroup(crsf_, kCrsfPublishers, cfg_.crsf_max_frames_per_poll, now_us);
}
