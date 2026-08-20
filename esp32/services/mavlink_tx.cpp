// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include <mavlink.h>

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <limits>
#include <type_traits>

#include "../../third_party/mavlink/standard/mavlink_msg_autopilot_version.h"
#include "error_code.hpp"
#include "esp32_config.hpp"
#include "esp_log.h"
#include "flight_mode.hpp"
#include "mavlink.hpp"
#include "panic.hpp"
#include "slot_stagger.hpp"
#include "system.hpp"

namespace {

// MAVLink reads NaN as "not known" in a float field, which zero cannot say --
// an ESC sitting at rest reports zero amps and means it.
constexpr float kUnknownFloat = std::numeric_limits<float>::quiet_NaN();

// PX4's main_mode byte, from src/modules/commander/px4_custom_mode.h. That
// header is not vendored, so these enumerators are the only record here.
enum class Px4MainMode : uint8_t {
  kUnset = 0,
  kAcro = 5,
  kStabilized = 7,
};

int64_t DaysFromCivil(int64_t year, unsigned month, unsigned day) {
  year -= month <= 2 ? 1 : 0;
  const int64_t era = (year >= 0 ? year : year - 399) / 400;
  const unsigned yoe = static_cast<unsigned>(year - (era * 400));
  const unsigned adjusted_month = month > 2 ? month - 3u : month + 9u;
  const unsigned doy = (((153u * adjusted_month) + 2u) / 5u) + day - 1u;
  const unsigned doe = (yoe * 365u) + (yoe / 4u) - (yoe / 100u) + doy;
  return (era * 146097) + static_cast<int64_t>(doe) - 719468;
}

std::optional<uint64_t> TryBuildGpsUnixUsec(const message::GpsData &gps) {
  if (gps.year < 1970 || gps.month < 1 || gps.month > 12 || gps.day < 1 ||
      gps.day > 31 || gps.hour > 23 || gps.min > 59 || gps.sec > 59) {
    return std::nullopt;
  }

  const int64_t days = DaysFromCivil(gps.year, gps.month, gps.day);
  if (days < 0) {
    return std::nullopt;
  }

  const int64_t seconds =
      (days * 86400) + (static_cast<int64_t>(gps.hour) * 3600) +
      (static_cast<int64_t>(gps.min) * 60) + static_cast<int64_t>(gps.sec);
  if (seconds < 0) {
    return std::nullopt;
  }

  return static_cast<uint64_t>(seconds) * 1000000ull;
}

uint32_t MapSystemSensorFlagsToMavlink(uint32_t flags) {
  uint32_t mavlink_flags = 0;
  if ((flags & message::kSystemSensorFlagImu) != 0u) {
    mavlink_flags |= MAV_SYS_STATUS_SENSOR_3D_GYRO;
    mavlink_flags |= MAV_SYS_STATUS_SENSOR_3D_ACCEL;
  }
  if ((flags & message::kSystemSensorFlagGps) != 0u) {
    mavlink_flags |= MAV_SYS_STATUS_SENSOR_GPS;
  }
  if ((flags & message::kSystemSensorFlagBattery) != 0u) {
    mavlink_flags |= MAV_SYS_STATUS_SENSOR_BATTERY;
  }
  if ((flags & message::kSystemSensorFlagRcReceiver) != 0u) {
    mavlink_flags |= MAV_SYS_STATUS_SENSOR_RC_RECEIVER;
  }
  if ((flags & message::kSystemSensorFlagEsc) != 0u) {
    mavlink_flags |= MAV_SYS_STATUS_SENSOR_PROPULSION;
  }
  return mavlink_flags;
}

int8_t NormalizeBatteryRemaining(int8_t battery_remaining) {
  if (battery_remaining < 0) {
    return -1;
  }
  return battery_remaining > 100 ? 100 : battery_remaining;
}
}  // namespace

void Mavlink::QueueTxItem(const TxQueueItem &item) {
  if (!tx_work_queue_.Push(item)) {
    constexpr const char *tag = "mavlink";

    TxQueueItem dropped{};
    (void)tx_work_queue_.Pop(dropped);
    (void)tx_work_queue_.Push(item);
    ESP_LOGW(tag, "TX work queue full; dropped oldest item");
    Sys().TonePlayer().PlayBuiltin(TonePlayer::BuiltinTone::kWarning);
  }
}

void Mavlink::QueueStatusText(const char *text, uint8_t severity) {
  if (text == nullptr || text[0] == '\0') {
    return;
  }

  StatusText status{};
  status.severity = severity;
  std::strncpy(status.text, text, sizeof(status.text) - 1);
  status.text[sizeof(status.text) - 1] = '\0';

  QueueTxItem(status);
}

void Mavlink::NotifyGcsIssue(const char *text, uint8_t severity) {
  if (text == nullptr || text[0] == '\0') {
    return;
  }

  QueueStatusText(text, severity);
  Sys().TonePlayer().PlayBuiltin(::TonePlayer::BuiltinTone::kWarning);
}

void Mavlink::ReportPanic(PanicSource source, uint32_t error_code) {
  const char *source_name = "ESP32";
  if (source == PanicSource::kStm32) {
    source_name = "STM32";
  }

  StatusText status{};
  status.severity = MAV_SEVERITY_CRITICAL;
  std::snprintf(status.text, sizeof(status.text), "%s PANIC: 0x%lX %s",
                source_name, static_cast<unsigned long>(error_code),
                GetMessage(error_code));

  const bool have_transport = transport_ != nullptr && transport_->IsReady();
  if (!SendStatusTextFrameNow(status, false) && have_transport &&
      error_code !=
          static_cast<uint32_t>(ErrorCode::Esp32::kMavlinkPanicSendFailed)) {
    Panic(ErrorCode::Esp32::kMavlinkPanicSendFailed);
  }
}

bool Mavlink::SendStatusTextFrameNow(const StatusText &status,
                                     bool require_link_enabled) {
  if (transport_ == nullptr || (require_link_enabled && !link_enabled_) ||
      !transport_->IsReady()) {
    return false;
  }

  const TxFrameState frame = StartStatusTextFrame(status);
  if (frame.Empty()) {
    return false;
  }

  const int sent = transport_->Send(frame.Bytes());
  if (sent > 0) {
    udp_tx_packet_count_.fetch_add(1, std::memory_order_relaxed);
    return true;
  }
  return false;
}

void Mavlink::QueueCommandAck(uint16_t command, uint8_t result,
                              uint8_t target_system, uint8_t target_component) {
  QueueTxItem(CommandAck{command, result, target_system, target_component});
}

void Mavlink::QueueAutopilotVersion() { QueueTxItem(AutopilotVersion{}); }

void Mavlink::QueueMissionCount(uint8_t target_system, uint8_t target_component,
                                uint8_t mission_type) {
  QueueTxItem(MissionCount{target_system, target_component, mission_type});
}

std::optional<Mavlink::TxFrameState> Mavlink::StartQueuedTxWorkFrame() {
  TxQueueItem work{};
  if (!tx_work_queue_.Pop(work)) {
    return std::nullopt;
  }

  return std::visit(
      [&](const auto &item) -> std::optional<TxFrameState> {
        using Item = std::decay_t<decltype(item)>;
        if constexpr (std::is_same_v<Item, std::monostate>) {
          return std::nullopt;
        } else if constexpr (std::is_same_v<Item, CommandAck>) {
          return StartCommandAckFrame(item);
        } else if constexpr (std::is_same_v<Item, AutopilotVersion>) {
          return StartAutopilotVersionFrame(item);
        } else if constexpr (std::is_same_v<Item, MissionCount>) {
          return StartMissionCountFrame(item);
        } else if constexpr (std::is_same_v<Item, StatusText>) {
          return StartStatusTextFrame(item);
        }
      },
      work);
}

Mavlink::TxFrameState Mavlink::StartCommandAckFrame(const CommandAck &ack) {
  mavlink_message_t m{};
  mavlink_msg_command_ack_pack(cfg_.identity.sysid, cfg_.identity.compid, &m,
                               ack.command, ack.result, UINT8_MAX, 0,
                               ack.target_system, ack.target_component);

  return TxFrameState{m, /*is_heartbeat=*/false};
}

Mavlink::TxFrameState Mavlink::StartAutopilotVersionFrame(
    const AutopilotVersion &work) {
  (void)work;

  constexpr uint64_t mav_protocol_capability_param_float = 1ull << 1;
  constexpr uint64_t mav_protocol_capability_mavlink2 = 1ull << 13;
  constexpr uint64_t mav_protocol_capability_param_encode_c_cast = 1ull << 17;
  const uint64_t capabilities = mav_protocol_capability_param_float |
                                mav_protocol_capability_mavlink2 |
                                mav_protocol_capability_param_encode_c_cast;
  static constexpr uint8_t kZeroHash[8] = {};
  static constexpr uint8_t kZeroUid2[18] = {};

  mavlink_message_t m{};
  mavlink_msg_autopilot_version_pack(cfg_.identity.sysid, cfg_.identity.compid,
                                     &m, capabilities, kMavlinkFlightSwVersion,
                                     0, 0, 0, kZeroHash, kZeroHash, kZeroHash,
                                     0, 0, 0, kZeroUid2);

  return TxFrameState{m, /*is_heartbeat=*/false};
}

Mavlink::TxFrameState Mavlink::StartMissionCountFrame(
    const MissionCount &work) {
  mavlink_message_t m{};
  mavlink_msg_mission_count_pack(cfg_.identity.sysid, cfg_.identity.compid, &m,
                                 work.target_system, work.target_component, 0,
                                 work.mission_type, 0);

  return TxFrameState{m, /*is_heartbeat=*/false};
}

Mavlink::TxFrameState Mavlink::StartStatusTextFrame(const StatusText &work) {
  mavlink_message_t m{};
  mavlink_msg_statustext_pack(cfg_.identity.sysid, cfg_.identity.compid, &m,
                              work.severity, work.text, 0, 0);

  return TxFrameState{m, /*is_heartbeat=*/false};
}

namespace {

// Only one frame leaves per tick, so streams that come due together do not
// interleave -- they serialise into a burst, and because rescheduling advances
// by period rather than from now, that opening phase persists for the life of
// the link. Giving each stream its own slot spaces the first frames out, and
// deriving the offset from the slot rather than configuring it per stream
// keeps a collision from being introduced by hand.
//
// The heartbeat is a slot like any other. Its deadline can pull it earlier, but
// it still reschedules by period, so leaving it out of the ladder only means
// its offset is fixed at zero and whatever lands there collides with it.
constexpr uint16_t kSysStatusPeriodMs = 1000;

constexpr uint32_t kSlotPeriodsMs[] = {
    kMavlinkConfig.tx.periods.hb_ms,   kSysStatusPeriodMs,
    kMavlinkConfig.tx.periods.gps_ms,  kMavlinkConfig.tx.periods.att_ms,
    kMavlinkConfig.tx.periods.gpos_ms, kMavlinkConfig.tx.periods.batt_ms,
    kMavlinkConfig.tx.periods.rc_ms,   kMavlinkConfig.tx.periods.esc_ms,
};
static_assert(std::size(kSlotPeriodsMs) == Mavlink::kTxSlotCount,
              "a TxSlot has no period here, so its offset goes unchecked");

// Equal priority throughout leaves the tie-break as the whole rule: of the
// streams due, the one waiting longest goes. Raising the heartbeat's would not
// help -- its precedence comes from being dispatched before these at all.
constexpr std::array<TopicConfig, Mavlink::kTxSlotCount> kTxSlotConfigs = [] {
  std::array<TopicConfig, Mavlink::kTxSlotCount> configs{};
  for (size_t i = 0; i < configs.size(); ++i) {
    configs[i] = TopicConfig{.period = kSlotPeriodsMs[i]};
  }
  return configs;
}();

// 210 ms measured best against today's stream set. Pick keeps it while it
// clears every slot and moves to the next value up when a period is retuned
// onto it, so adding a stream cannot quietly reinstate the collision.
constexpr uint32_t kTxSlotSpacingTargetMs = 210;
constexpr uint32_t kTxSlotStaggerMs =
    slot_stagger::Pick(kTxSlotSpacingTargetMs, kSlotPeriodsMs);
static_assert(kTxSlotStaggerMs != 0,
              "no spacing near the target clears every configured TX period, "
              "so some stream would sit permanently in phase with the ladder");

constexpr size_t SlotIndex(Mavlink::TxSlot slot) {
  return static_cast<size_t>(slot);
}

}  // namespace

// The offsets come from a ladder solved at compile time against kMavlinkConfig,
// so this schedules that configuration and no other. It takes no config for
// that reason: a parameter would suggest the offsets follow whatever is passed.
void Mavlink::InitTxSchedule(uint32_t now_ms, bool force_heartbeat_due) {
  last_hb_done_ms_ =
      force_heartbeat_due ? (now_ms - cfg_.tx.schedule.hb_deadline_ms) : now_ms;
  tx_scheduler_.Init(kTxSlotConfigs, tx_slots_, kTxSlotStaggerMs, now_ms);
}

bool Mavlink::ShouldSendHbNow(const Config::Tx &cfg_tx, uint32_t now_ms) const {
  if (cfg_tx.periods.hb_ms == 0 || cfg_tx.schedule.hb_deadline_ms == 0) {
    return false;
  }

  const int32_t since_done = static_cast<int32_t>(now_ms - last_hb_done_ms_);
  if (since_done >= static_cast<int32_t>(cfg_tx.schedule.hb_deadline_ms)) {
    return true;
  }
  return tx_scheduler_.IsDue(SlotIndex(TxSlot::kHb), now_ms);
}

void Mavlink::ServiceTx(uint32_t now_ms) {
  constexpr uint32_t tx_poll_period_ms = 10;

  if (static_cast<int32_t>(now_ms - next_tx_poll_ms_) < 0) {
    return;
  }
  next_tx_poll_ms_ = now_ms + tx_poll_period_ms;

  if (!transport_->IsReady()) {
    // Drop any remembered peer state as soon as the transport reports not
    // ready so TX does not keep targeting a stale endpoint across reconnects.
    transport_->ClearPeer();
  }
  if (!link_enabled_) {
    return;
  }
  ServiceUdpTx(now_ms);
}

bool Mavlink::StartNextFrameIfIdle(TxState &tx, const Config::Tx &cfg_tx,
                                   uint32_t now_ms) {
  if (!tx_frame_.Empty()) {
    return true;
  }

  // Priority order matters here: heartbeats and command replies preempt the
  // periodic telemetry streams so the link stays responsive to the GCS.
  if (ShouldSendHbNow(cfg_tx, now_ms)) {
    tx_frame_ = StartHeartbeatFrame(cfg_tx, now_ms);
  } else if (const std::optional<TxFrameState> queued_frame =
                 StartQueuedTxWorkFrame()) {
    tx_frame_ = *queued_frame;
  } else if (const auto param_frame = StartNextParamFrame(
                 tx, cfg_.identity.sysid, cfg_.identity.compid)) {
    tx_frame_ = *param_frame;
  } else if (const std::optional<TxFrameState> scheduled_frame =
                 StartNextScheduledFrame(cfg_tx, now_ms)) {
    tx_frame_ = *scheduled_frame;
  }

  return !tx_frame_.Empty();
}

void Mavlink::CompleteFrame(TxFrameState &frame, uint32_t now_ms) {
  if (frame.IsHeartbeat()) {
    last_hb_done_ms_ = now_ms;
  }
  frame.Clear();
}

Mavlink::TxFrameState Mavlink::StartHeartbeatFrame(const Config::Tx &cfg_tx,
                                                   uint32_t now_ms) {
  constexpr uint8_t mav_autopilot_32raven = 200;

  // `have_data` latches on the first report and never clears, so without this
  // a silent STM32 would keep the ground station asserting what it last saw.
  const bool vehicle_fresh =
      vehicle_status_.have_data &&
      (uint32_t)(now_ms - vehicle_status_.update_ms) <= peer_timeout_ms_;

  const bool vehicle_armed =
      vehicle_fresh &&
      static_cast<message::ArmedState>(vehicle_status_.value.armed_state) ==
          message::ArmedState::kArmed;

  uint8_t base_mode =
      MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
  if (vehicle_armed) {
    base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
  }
  if (vehicle_fresh &&
      static_cast<FlightMode>(vehicle_status_.value.flight_mode) ==
          FlightMode::kStabilize) {
    base_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
  }

  uint8_t system_status = MAV_STATE_BOOT;
  if (system_status_.have_data) {
    const message::SystemStatusMsg &status = system_status_.value;
    const bool fresh = SystemStatusFresh(now_ms);
    const bool loop_alive =
        (status.flags & message::kSystemStatusFlagLoopAlive) != 0u;
    const bool system_error =
        status.error_code != static_cast<uint32_t>(ErrorCode::Common::kOk) ||
        static_cast<message::BootState>(status.boot_state) ==
            message::BootState::kError;
    // Not gated on freshness: a stale failsafe can only hold MAV_STATE at
    // CRITICAL, and a link that died with one raised is not the time to stop.
    const bool vehicle_failsafe =
        vehicle_status_.have_data && vehicle_status_.value.failsafe_flags != 0u;

    if (!fresh || !loop_alive || system_error || vehicle_failsafe) {
      system_status = MAV_STATE_CRITICAL;
    } else if (static_cast<message::BootState>(status.boot_state) !=
               message::BootState::kReady) {
      system_status = MAV_STATE_BOOT;
    } else if (vehicle_armed) {
      system_status = MAV_STATE_ACTIVE;
    } else {
      system_status = MAV_STATE_STANDBY;
    }
  }

  // PX4 custom_mode layout: bytes [reserved(2), main_mode, sub_mode].
  Px4MainMode main_mode = Px4MainMode::kUnset;
  if (vehicle_fresh) {
    switch (static_cast<FlightMode>(vehicle_status_.value.flight_mode)) {
      case FlightMode::kAcro:
        main_mode = Px4MainMode::kAcro;
        break;
      case FlightMode::kStabilize:
        main_mode = Px4MainMode::kStabilized;
        break;
    }
  }
  const uint32_t custom_mode = static_cast<uint32_t>(main_mode) << 16;

  mavlink_message_t m{};
  mavlink_msg_heartbeat_pack(cfg_.identity.sysid, cfg_.identity.compid, &m,
                             MAV_TYPE_QUADROTOR, mav_autopilot_32raven,
                             base_mode, custom_mode, system_status);

  // Also what stops the slot staying due and refiring next poll, since the
  // deadline can pull the heartbeat out ahead of it.
  tx_scheduler_.MarkSent(SlotIndex(TxSlot::kHb), now_ms);
  return TxFrameState{m, /*is_heartbeat=*/true};
}

bool Mavlink::SystemStatusFresh(uint32_t now_ms) const {
  return system_status_.have_data &&
         static_cast<uint32_t>(now_ms - system_status_.update_ms) <=
             kMavlinkSystemStatusFreshMs;
}

Mavlink::TxFrameState Mavlink::StartSysStatusFrame(uint32_t now_ms) {
  uint32_t sensors_present = 0;
  uint32_t sensors_enabled = 0;
  uint32_t sensors_health = 0;
  uint16_t voltage_battery = 0;
  int16_t current_battery = -1;
  int8_t battery_remaining = -1;
  uint16_t load = 0;
  if (system_status_.have_data) {
    const message::SystemStatusMsg &status = system_status_.value;
    sensors_present =
        MapSystemSensorFlagsToMavlink(status.sensor_present_flags);
    sensors_enabled = sensors_present;

    // A stale cache still names which sensors exist, which does not go out of
    // date. What they were last doing does: health clears and the readings go
    // to their unknowns, rather than a link that died being reported as a
    // vehicle whose every sensor is well.
    if (SystemStatusFresh(now_ms)) {
      sensors_health = MapSystemSensorFlagsToMavlink(
          status.sensor_health_flags & status.sensor_present_flags);
      voltage_battery = status.batt_voltage;
      current_battery = status.batt_current;
      battery_remaining = NormalizeBatteryRemaining(status.batt_remaining);
      load = status.control_loop_load;
    } else {
      voltage_battery = UINT16_MAX;
    }
  } else if (const std::optional<message::GpsData> latest =
                 GetCachedValue(gps_)) {
    // Without SystemStatusMsg a fix is all there is to report.
    sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
    sensors_enabled |= MAV_SYS_STATUS_SENSOR_GPS;
    if (latest->fixType >= 2) {
      sensors_health |= MAV_SYS_STATUS_SENSOR_GPS;
    }
  }

  mavlink_message_t m{};
  mavlink_msg_sys_status_pack(cfg_.identity.sysid, cfg_.identity.compid, &m,
                              sensors_present, sensors_enabled, sensors_health,
                              load, voltage_battery, current_battery,
                              battery_remaining, 0, 0, 0, 0, 0, 0, 0, 0, 0);

  return TxFrameState{m, /*is_heartbeat=*/false};
}

std::optional<Mavlink::TxFrameState> Mavlink::StartGpsRawIntFrame(
    const Config::Tx &cfg_tx) {
  const std::optional<message::GpsData> latest = GetCachedValue(gps_);
  if (!latest.has_value()) {
    return std::nullopt;
  }

  mavlink_message_t m{};
  uint64_t time_usec = 0;
  if (const std::optional<uint64_t> gps_time_usec =
          TryBuildGpsUnixUsec(*latest)) {
    time_usec = *gps_time_usec;
  }

  const uint16_t eph = (latest->hDOP > 0) ? latest->hDOP : UINT16_MAX;
  const uint16_t epv = (latest->vDOP > 0) ? latest->vDOP : UINT16_MAX;

  mavlink_msg_gps_raw_int_pack(
      cfg_.identity.sysid, cfg_.identity.compid, &m, time_usec,
      static_cast<uint8_t>(latest->fixType), latest->lat, latest->lon,
      static_cast<int32_t>(latest->hMSL), eph, epv,
      static_cast<uint16_t>(latest->vel), static_cast<uint16_t>(latest->hdg),
      static_cast<uint8_t>(latest->numSV), 0, latest->hAcc, latest->vAcc, 0, 0,
      0);

  return TxFrameState{m, /*is_heartbeat=*/false};
}

std::optional<Mavlink::TxFrameState> Mavlink::StartAttitudeFrame(
    const Config::Tx &cfg_tx) {
  const std::optional<message::AttitudeMsg> latest = GetCachedValue(attitude_);
  if (!latest.has_value()) {
    return std::nullopt;
  }

  // ZYX Tait-Bryan, the order MAVLink ATTITUDE names its fields in.
  const float qw = latest->qw;
  const float qx = latest->qx;
  const float qy = latest->qy;
  const float qz = latest->qz;

  const float roll = std::atan2(2.0f * ((qw * qx) + (qy * qz)),
                                1.0f - (2.0f * ((qx * qx) + (qy * qy))));
  float sin_pitch = 2.0f * ((qw * qy) - (qz * qx));
  sin_pitch = sin_pitch > 1.0f ? 1.0f : (sin_pitch < -1.0f ? -1.0f : sin_pitch);
  const float pitch = std::asin(sin_pitch);
  const float yaw = std::atan2(2.0f * ((qw * qz) + (qx * qy)),
                               1.0f - (2.0f * ((qy * qy) + (qz * qz))));

  mavlink_message_t m{};

  mavlink_msg_attitude_pack(cfg_.identity.sysid, cfg_.identity.compid, &m, 0,
                            roll, pitch, yaw, 0.0f, 0.0f, 0.0f);

  return TxFrameState{m, /*is_heartbeat=*/false};
}

std::optional<Mavlink::TxFrameState> Mavlink::StartGlobalPositionIntFrame(
    const Config::Tx &cfg_tx) {
  const std::optional<message::GpsData> latest = GetCachedValue(gps_);
  if (!latest.has_value()) {
    return std::nullopt;
  }

  mavlink_message_t m{};
  mavlink_msg_global_position_int_pack(
      cfg_.identity.sysid, cfg_.identity.compid, &m, 0, latest->lat,
      latest->lon, latest->hMSL, latest->hMSL, 0, 0, 0, latest->hdg);

  return TxFrameState{m, /*is_heartbeat=*/false};
}

std::optional<Mavlink::TxFrameState> Mavlink::StartBatteryStatusFrame(
    const Config::Tx &cfg_tx, uint32_t now_ms) {
  // Nothing in BATTERY_STATUS can qualify a reading the way SYS_STATUS's
  // health bit does, so a stale one is withheld instead of dressed up.
  if (!SystemStatusFresh(now_ms)) {
    return std::nullopt;
  }
  const message::SystemStatusMsg &status = system_status_.value;
  const uint16_t voltage_mv = status.batt_voltage;
  const int16_t current_ca = status.batt_current;
  const int8_t remaining_pct = status.batt_remaining;

  const bool have_battery = voltage_mv > 0;
  const int8_t battery_remaining =
      have_battery ? NormalizeBatteryRemaining(remaining_pct)
                   : static_cast<int8_t>(-1);

  uint16_t voltages[10];
  for (uint16_t &voltage : voltages) {
    voltage = UINT16_MAX;
  }
  voltages[0] = have_battery ? voltage_mv : UINT16_MAX;

  uint16_t voltages_ext[4] = {};

  mavlink_message_t m{};
  mavlink_msg_battery_status_pack(
      cfg_.identity.sysid, cfg_.identity.compid, &m, 0,
      MAV_BATTERY_FUNCTION_ALL, MAV_BATTERY_TYPE_LIPO, INT16_MAX, voltages,
      have_battery ? current_ca : static_cast<int16_t>(-1), -1, -1,
      battery_remaining, 0,
      static_cast<uint8_t>(MAV_BATTERY_CHARGE_STATE_UNDEFINED), voltages_ext, 0,
      0);

  return TxFrameState{m, /*is_heartbeat=*/false};
}

std::optional<Mavlink::TxFrameState> Mavlink::StartRcChannelsFrame(
    const Config::Tx &cfg_tx) {
  if (!rc_channels_.have_data) {
    return std::nullopt;
  }

  const message::RcChannelsMsg &channels = rc_channels_.value;
  // RC_CHANNELS is fixed at eighteen slots whatever the aircraft carries, so
  // the ones past our channel count are filled with the value the field
  // documents as "not present" rather than with zeros a ground station would
  // draw as live sticks at minimum.
  constexpr uint16_t mavlink_unused_channel_value = UINT16_MAX;
  constexpr size_t mavlink_rc_channel_slots = 18u;
  static_assert(message::kRcChannelCount <= mavlink_rc_channel_slots,
                "more channels enabled than RC_CHANNELS can carry");

  const bool rx_online =
      (channels.flags & message::kRcChannelsFlagRxOnline) != 0;
  const uint8_t channel_count =
      rx_online ? static_cast<uint8_t>(message::kRcChannelCount) : 0u;
  // CRSF reports link quality as a percentage; RC_CHANNELS.rssi spreads its
  // range over 0-254 and keeps UINT8_MAX for unknown, so a percentage passed
  // through unscaled would read as roughly two fifths of the real figure.
  const auto to_mavlink_rssi = [](uint8_t link_quality) -> uint8_t {
    const uint32_t percent = link_quality > 100u ? 100u : link_quality;
    return static_cast<uint8_t>((percent * 254u) / 100u);
  };
  const uint8_t rssi =
      rx_online ? to_mavlink_rssi(channels.link_quality) : UINT8_MAX;

  std::array<uint16_t, mavlink_rc_channel_slots> slots{};
  slots.fill(mavlink_unused_channel_value);
  if (rx_online) {
    for (size_t i = 0; i < message::kRcChannelCount; ++i) {
      slots[i] = channels.channels[i];
    }
  }

  mavlink_message_t m{};
  mavlink_msg_rc_channels_pack(
      cfg_.identity.sysid, cfg_.identity.compid, &m, rc_channels_.update_ms,
      channel_count, slots[0], slots[1], slots[2], slots[3], slots[4], slots[5],
      slots[6], slots[7], slots[8], slots[9], slots[10], slots[11], slots[12],
      slots[13], slots[14], slots[15], slots[16], slots[17], rssi);

  return TxFrameState{m, /*is_heartbeat=*/false};
}

std::optional<Mavlink::TxFrameState> Mavlink::StartEscStatusFrame(
    const Config::Tx &cfg_tx) {
  if (!esc_telemetry_.have_data) {
    return std::nullopt;
  }

  const message::EscTelemetryMsg &esc = esc_telemetry_.value;
  int32_t rpm[message::kEscTelemetryMotorCount] = {};
  float voltage[message::kEscTelemetryMotorCount];
  float current[message::kEscTelemetryMotorCount];

  for (uint8_t i = 0; i < message::kEscTelemetryMotorCount; ++i) {
    voltage[i] = kUnknownFloat;
    current[i] = kUnknownFloat;

    const bool valid = (esc.valid_mask & (1u << i)) != 0u;
    if (!valid) {
      continue;
    }
    rpm[i] = esc.rpm[i] > static_cast<uint32_t>(INT32_MAX)
                 ? INT32_MAX
                 : static_cast<int32_t>(esc.rpm[i]);
    voltage[i] = static_cast<float>(esc.voltage_centivolts[i]) * 0.01f;
    // Negative is the FC saying the ESC has no shunt, not a reading.
    if (esc.current_centiamps[i] >= 0) {
      current[i] = static_cast<float>(esc.current_centiamps[i]) * 0.01f;
    }
  }

  mavlink_message_t m{};
  mavlink_msg_esc_status_pack(cfg_.identity.sysid, cfg_.identity.compid, &m, 0,
                              esc.timestamp_us, rpm, voltage, current);

  return TxFrameState{m, /*is_heartbeat=*/false};
}

std::optional<Mavlink::TxFrameState> Mavlink::StartNextScheduledFrame(
    const Config::Tx &cfg_tx, uint32_t now_ms) {
  // The heartbeat holds a ladder slot for its offset but is dispatched by
  // ShouldSendHbNow before this runs; skipping it here is what keeps it from
  // going out twice.
  while (const std::optional<size_t> due = tx_scheduler_.NextDue(now_ms)) {
    const auto slot = static_cast<TxSlot>(*due);
    if (slot == TxSlot::kHb) {
      tx_scheduler_.Skip(*due, now_ms);
      continue;
    }

    std::optional<TxFrameState> frame;
    switch (slot) {
      case TxSlot::kSys:
        frame = StartSysStatusFrame(now_ms);
        break;
      case TxSlot::kGps:
        frame = StartGpsRawIntFrame(cfg_tx);
        break;
      case TxSlot::kAtt:
        frame = StartAttitudeFrame(cfg_tx);
        break;
      case TxSlot::kGpos:
        frame = StartGlobalPositionIntFrame(cfg_tx);
        break;
      case TxSlot::kBatt:
        frame = StartBatteryStatusFrame(cfg_tx, now_ms);
        break;
      case TxSlot::kRc:
        frame = StartRcChannelsFrame(cfg_tx);
        break;
      case TxSlot::kEsc:
        frame = StartEscStatusFrame(cfg_tx);
        break;
      case TxSlot::kHb:
      case TxSlot::kCount:
        break;
    }

    // A stream with nothing to say still consumes its turn: the deadline moves
    // either way, so a source that never has data cannot spin here.
    if (!frame) {
      tx_scheduler_.Skip(*due, now_ms);
      continue;
    }

    tx_scheduler_.MarkSent(*due, now_ms);
    return frame;
  }

  return std::nullopt;
}

void Mavlink::ServiceUdpTx(uint32_t now_ms) {
  if (!transport_->IsReady()) {
    return;
  }

  if (!StartNextFrameIfIdle(udp_tx_, cfg_.tx, now_ms)) {
    return;
  }

  const int sent = transport_->Send(tx_frame_.Bytes());
  if (sent > 0) {
    if (tx_frame_.IsHeartbeat()) {
      udp_tx_heartbeat_count_.fetch_add(1, std::memory_order_relaxed);
    }
    udp_tx_packet_count_.fetch_add(1, std::memory_order_relaxed);
  }
  CompleteFrame(tx_frame_, now_ms);
}
