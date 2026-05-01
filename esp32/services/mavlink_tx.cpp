#include <mavlink.h>

#include <cstdio>
#include <cstring>
#include <type_traits>

#include "../../third_party/mavlink/standard/mavlink_msg_autopilot_version.h"
#include "error_code.hpp"
#include "esp32_limits.hpp"
#include "esp_log.h"
#include "mavlink.hpp"
#include "panic.hpp"
#include "system.hpp"

namespace {

int64_t DaysFromCivil(int64_t year, unsigned month, unsigned day) {
  year -= month <= 2 ? 1 : 0;
  const int64_t era = (year >= 0 ? year : year - 399) / 400;
  const unsigned yoe = static_cast<unsigned>(year - era * 400);
  const unsigned adjusted_month = month > 2 ? month - 3u : month + 9u;
  const unsigned doy = (153u * adjusted_month + 2u) / 5u + day - 1u;
  const unsigned doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
  return era * 146097 + static_cast<int64_t>(doe) - 719468;
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

  const int64_t seconds = days * 86400 + static_cast<int64_t>(gps.hour) * 3600 +
                          static_cast<int64_t>(gps.min) * 60 +
                          static_cast<int64_t>(gps.sec);
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

void Mavlink::ReportPanic(PanicSource source, ErrorCode error_code) {
  const char *source_name = "ESP32";
  if (source == PanicSource::kStm32) {
    source_name = "STM32";
  }

  StatusText status{};
  status.severity = MAV_SEVERITY_CRITICAL;
  std::snprintf(status.text, sizeof(status.text), "%s PANIC: 0x%lX %s",
                source_name,
                static_cast<unsigned long>(static_cast<uint32_t>(error_code)),
                GetMessage(error_code));

  if (!SendStatusTextFrameNow(status, false) &&
      error_code != ErrorCode::kMavlinkPanicSendFailed) {
    Panic(ErrorCode::kMavlinkPanicSendFailed);
  }
}

bool Mavlink::SendStatusTextFrameNow(const StatusText &status,
                                     bool require_link_enabled) {
  if (udp_ == nullptr || (require_link_enabled && !link_enabled_) ||
      !Sys().Wifi().HasAssociatedStations()) {
    return false;
  }

  const TxFrameState frame = StartStatusTextFrame(status);
  if (frame.len == 0) {
    return false;
  }

  const int sent = udp_->Send(frame.buf, frame.len);
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

  TxFrameState frame{};
  frame.len = static_cast<uint16_t>(mavlink_msg_to_send_buffer(frame.buf, &m));
  frame.is_hb = false;
  return frame;
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
  mavlink_msg_autopilot_version_pack(
      cfg_.identity.sysid, cfg_.identity.compid, &m, capabilities,
      esp32_limits::kMavlinkFlightSwVersion, 0, 0, 0, kZeroHash, kZeroHash,
      kZeroHash, 0, 0, 0, kZeroUid2);

  TxFrameState frame{};
  frame.len = static_cast<uint16_t>(mavlink_msg_to_send_buffer(frame.buf, &m));
  frame.is_hb = false;
  return frame;
}

Mavlink::TxFrameState Mavlink::StartMissionCountFrame(
    const MissionCount &work) {
  mavlink_message_t m{};
  mavlink_msg_mission_count_pack(cfg_.identity.sysid, cfg_.identity.compid, &m,
                                 work.target_system, work.target_component, 0,
                                 work.mission_type, 0);

  TxFrameState frame{};
  frame.len = static_cast<uint16_t>(mavlink_msg_to_send_buffer(frame.buf, &m));
  frame.is_hb = false;
  return frame;
}

Mavlink::TxFrameState Mavlink::StartStatusTextFrame(const StatusText &work) {
  mavlink_message_t m{};
  mavlink_msg_statustext_pack(cfg_.identity.sysid, cfg_.identity.compid, &m,
                              work.severity, work.text, 0, 0);

  TxFrameState frame{};
  frame.len = static_cast<uint16_t>(mavlink_msg_to_send_buffer(frame.buf, &m));
  frame.is_hb = false;
  return frame;
}

void Mavlink::InitTxSchedule(const Config::Tx &cfg_tx, uint32_t now_ms,
                             bool force_heartbeat_due) {
  constexpr uint16_t sys_status_start_delay_ms = 250;

  tx_schedule_.last_hb_done_ms =
      force_heartbeat_due ? (now_ms - cfg_tx.schedule.hb_deadline_ms) : now_ms;
  tx_schedule_.next_hb_ms = now_ms;
  tx_schedule_.next_sys_ms = now_ms + sys_status_start_delay_ms;
  tx_schedule_.next_gps_ms = now_ms + cfg_tx.schedule.gps_start_delay_ms;
  tx_schedule_.next_att_ms = now_ms + cfg_tx.schedule.att_start_delay_ms;
  tx_schedule_.next_gpos_ms = now_ms + cfg_tx.schedule.gpos_start_delay_ms;
  tx_schedule_.next_batt_ms = now_ms + cfg_tx.schedule.batt_start_delay_ms;
  tx_schedule_.next_rc_ms = now_ms + cfg_tx.schedule.rc_start_delay_ms;
}

bool Mavlink::ShouldSendHbNow(const Config::Tx &cfg_tx, uint32_t now_ms) const {
  if (cfg_tx.periods.hb_ms == 0 || cfg_tx.schedule.hb_deadline_ms == 0) {
    return false;
  }

  const int32_t since_done =
      static_cast<int32_t>(now_ms - tx_schedule_.last_hb_done_ms);
  if (since_done >= static_cast<int32_t>(cfg_tx.schedule.hb_deadline_ms)) {
    return true;
  }
  return static_cast<int32_t>(now_ms - tx_schedule_.next_hb_ms) >= 0;
}

void Mavlink::ServiceTx(uint32_t now_ms) {
  constexpr uint32_t tx_poll_period_ms = 10;

  if (static_cast<int32_t>(now_ms - next_tx_poll_ms_) < 0) {
    return;
  }
  next_tx_poll_ms_ = now_ms + tx_poll_period_ms;

  if (!Sys().Wifi().HasAssociatedStations()) {
    // Drop the remembered UDP peer as soon as the AP has no stations so TX
    // does not keep targeting a stale address across reconnects.
    udp_->ClearPeer();
  }
  if (!link_enabled_) {
    return;
  }
  ServiceUdpTx(now_ms);
}

bool Mavlink::StartNextFrameIfIdle(TxState &tx, const Config::Tx &cfg_tx,
                                   uint32_t now_ms) {
  if (tx_frame_.len > 0) {
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

  return tx_frame_.len > 0;
}

void Mavlink::CompleteFrame(TxFrameState &frame, uint32_t now_ms) {
  if (frame.is_hb) {
    tx_schedule_.last_hb_done_ms = now_ms;
  }
  frame.len = 0;
  frame.is_hb = false;
}

Mavlink::TxFrameState Mavlink::StartHeartbeatFrame(const Config::Tx &cfg_tx,
                                                   uint32_t now_ms) {
  constexpr uint8_t mav_autopilot_32raven = 200;

  uint8_t base_mode = 0;
  uint8_t system_status = MAV_STATE_BOOT;
  if (system_status_.have_data) {
    const message::SystemStatusMsg &status = system_status_.value;
    const bool fresh = (uint32_t)(now_ms - system_status_.update_ms) <=
                       esp32_limits::kMavlinkSystemStatusFreshMs;
    const bool loop_alive =
        (status.flags & message::kSystemStatusFlagLoopAlive) != 0u;
    const bool system_error =
        status.error_code != ErrorCode::kOk ||
        status.boot_state == message::kSystemBootStateError;
    const bool vehicle_armed =
        vehicle_status_.have_data &&
        vehicle_status_.value.armed_state == message::kVehicleArmedStateArmed;
    const bool vehicle_failsafe =
        vehicle_status_.have_data && vehicle_status_.value.failsafe_flags != 0u;

    if (vehicle_armed) {
      base_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
    }

    if (!fresh || !loop_alive || system_error || vehicle_failsafe) {
      system_status = MAV_STATE_CRITICAL;
    } else if (status.boot_state != message::kSystemBootStateReady) {
      system_status = MAV_STATE_BOOT;
    } else if (vehicle_armed) {
      system_status = MAV_STATE_ACTIVE;
    } else {
      system_status = MAV_STATE_STANDBY;
    }
  }

  mavlink_message_t m{};
  mavlink_msg_heartbeat_pack(cfg_.identity.sysid, cfg_.identity.compid, &m,
                             MAV_TYPE_QUADROTOR, mav_autopilot_32raven,
                             base_mode, 0, system_status);

  TxFrameState frame{};
  frame.len = static_cast<uint16_t>(mavlink_msg_to_send_buffer(frame.buf, &m));
  frame.is_hb = true;
  tx_schedule_.next_hb_ms += cfg_tx.periods.hb_ms;
  return frame;
}

Mavlink::TxFrameState Mavlink::StartSysStatusFrame() {
  constexpr uint16_t sys_status_ms = 1000;

  tx_schedule_.next_sys_ms += sys_status_ms;

  uint32_t sensors_present = 0;
  uint32_t sensors_enabled = 0;
  uint32_t sensors_health = 0;
  uint16_t voltage_battery = 0;
  int16_t current_battery = -1;
  int8_t battery_remaining = -1;
  if (system_status_.have_data) {
    const message::SystemStatusMsg &status = system_status_.value;
    sensors_present =
        MapSystemSensorFlagsToMavlink(status.sensor_present_flags);
    sensors_enabled = sensors_present;
    sensors_health = MapSystemSensorFlagsToMavlink(status.sensor_health_flags &
                                                   status.sensor_present_flags);
    voltage_battery = status.batt_voltage;
    current_battery = status.batt_current;
    battery_remaining = NormalizeBatteryRemaining(status.batt_remaining);
  } else if (const std::optional<message::GpsData> latest =
                 GetCachedValue(gps_)) {
    sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
    sensors_enabled |= MAV_SYS_STATUS_SENSOR_GPS;
    if (latest->fixType >= 2) {
      sensors_health |= MAV_SYS_STATUS_SENSOR_GPS;
    }

    if (latest->batt_voltage > 0) {
      sensors_present |= MAV_SYS_STATUS_SENSOR_BATTERY;
      sensors_enabled |= MAV_SYS_STATUS_SENSOR_BATTERY;
      sensors_health |= MAV_SYS_STATUS_SENSOR_BATTERY;
      voltage_battery = latest->batt_voltage;
      current_battery = latest->batt_current;
      battery_remaining = latest->batt_remaining;
    }
  }

  mavlink_message_t m{};
  mavlink_msg_sys_status_pack(cfg_.identity.sysid, cfg_.identity.compid, &m,
                              sensors_present, sensors_enabled, sensors_health,
                              0, voltage_battery, current_battery,
                              battery_remaining, 0, 0, 0, 0, 0, 0, 0, 0, 0);

  TxFrameState frame{};
  frame.len = static_cast<uint16_t>(mavlink_msg_to_send_buffer(frame.buf, &m));
  frame.is_hb = false;
  return frame;
}

std::optional<Mavlink::TxFrameState> Mavlink::StartGpsRawIntFrame(
    const Config::Tx &cfg_tx) {
  tx_schedule_.next_gps_ms += cfg_tx.periods.gps_ms;

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

  TxFrameState frame{};
  frame.len = static_cast<uint16_t>(mavlink_msg_to_send_buffer(frame.buf, &m));
  frame.is_hb = false;
  return frame;
}

std::optional<Mavlink::TxFrameState> Mavlink::StartAttitudeFrame(
    const Config::Tx &cfg_tx) {
  tx_schedule_.next_att_ms += cfg_tx.periods.att_ms;

  const std::optional<message::GpsData> latest = GetCachedValue(gps_);
  if (!latest.has_value()) {
    return std::nullopt;
  }

  mavlink_message_t m{};
  const float roll =
      (static_cast<float>(latest->roll) * 0.01f) * 0.017453292519943295f;
  const float pitch =
      (static_cast<float>(latest->pitch) * 0.01f) * 0.017453292519943295f;
  const float yaw =
      (static_cast<float>(latest->yaw) * 0.01f) * 0.017453292519943295f;

  mavlink_msg_attitude_pack(cfg_.identity.sysid, cfg_.identity.compid, &m, 0,
                            roll, pitch, yaw, 0.0f, 0.0f, 0.0f);

  TxFrameState frame{};
  frame.len = static_cast<uint16_t>(mavlink_msg_to_send_buffer(frame.buf, &m));
  frame.is_hb = false;
  return frame;
}

std::optional<Mavlink::TxFrameState> Mavlink::StartGlobalPositionIntFrame(
    const Config::Tx &cfg_tx) {
  tx_schedule_.next_gpos_ms += cfg_tx.periods.gpos_ms;

  const std::optional<message::GpsData> latest = GetCachedValue(gps_);
  if (!latest.has_value()) {
    return std::nullopt;
  }

  mavlink_message_t m{};
  mavlink_msg_global_position_int_pack(
      cfg_.identity.sysid, cfg_.identity.compid, &m, 0, latest->lat,
      latest->lon, latest->hMSL, latest->hMSL, 0, 0, 0, latest->hdg);

  TxFrameState frame{};
  frame.len = static_cast<uint16_t>(mavlink_msg_to_send_buffer(frame.buf, &m));
  frame.is_hb = false;
  return frame;
}

std::optional<Mavlink::TxFrameState> Mavlink::StartBatteryStatusFrame(
    const Config::Tx &cfg_tx) {
  tx_schedule_.next_batt_ms += cfg_tx.periods.batt_ms;

  const std::optional<message::GpsData> latest = GetCachedValue(gps_);
  if (!latest.has_value()) {
    return std::nullopt;
  }

  const bool have_battery = latest->batt_voltage > 0;
  int8_t battery_remaining = -1;
  if (have_battery && latest->batt_remaining >= 0) {
    battery_remaining =
        (latest->batt_remaining > 100) ? 100 : latest->batt_remaining;
  }

  uint16_t voltages[10];
  for (uint16_t &voltage : voltages) {
    voltage = UINT16_MAX;
  }
  voltages[0] = have_battery ? latest->batt_voltage : UINT16_MAX;

  uint16_t voltages_ext[4] = {};

  mavlink_message_t m{};
  mavlink_msg_battery_status_pack(
      cfg_.identity.sysid, cfg_.identity.compid, &m, 0,
      MAV_BATTERY_FUNCTION_ALL, MAV_BATTERY_TYPE_LIPO, INT16_MAX, voltages,
      have_battery ? latest->batt_current : static_cast<int16_t>(-1), -1, -1,
      battery_remaining, 0,
      static_cast<uint8_t>(MAV_BATTERY_CHARGE_STATE_UNDEFINED), voltages_ext, 0,
      0);

  TxFrameState frame{};
  frame.len = static_cast<uint16_t>(mavlink_msg_to_send_buffer(frame.buf, &m));
  frame.is_hb = false;
  return frame;
}

std::optional<Mavlink::TxFrameState> Mavlink::StartRcChannelsFrame(
    const Config::Tx &cfg_tx) {
  tx_schedule_.next_rc_ms += cfg_tx.periods.rc_ms;

  if (!rc_channels_.have_data) {
    return std::nullopt;
  }

  const message::RcChannelsMsg &channels = rc_channels_.value;
  constexpr uint16_t invalid_channel_value = UINT16_MAX;
  constexpr uint16_t mavlink_unused_channel_value = UINT16_MAX;
  constexpr uint8_t rc_channel_count = static_cast<uint8_t>(
      sizeof(channels.channels) / sizeof(channels.channels[0]));

  const bool rx_online =
      (channels.flags & message::kRcChannelsFlagRxOnline) != 0;
  const uint8_t channel_count = rx_online ? rc_channel_count : 0u;
  const uint8_t rssi = rx_online ? channels.rssi : UINT8_MAX;

  mavlink_message_t m{};
  mavlink_msg_rc_channels_pack(
      cfg_.identity.sysid, cfg_.identity.compid, &m, rc_channels_.update_ms,
      channel_count, rx_online ? channels.channels[0] : invalid_channel_value,
      rx_online ? channels.channels[1] : invalid_channel_value,
      rx_online ? channels.channels[2] : invalid_channel_value,
      rx_online ? channels.channels[3] : invalid_channel_value,
      rx_online ? channels.channels[4] : invalid_channel_value,
      rx_online ? channels.channels[5] : invalid_channel_value,
      rx_online ? channels.channels[6] : invalid_channel_value,
      rx_online ? channels.channels[7] : invalid_channel_value,
      rx_online ? channels.channels[8] : invalid_channel_value,
      rx_online ? channels.channels[9] : invalid_channel_value,
      rx_online ? channels.channels[10] : invalid_channel_value,
      rx_online ? channels.channels[11] : invalid_channel_value,
      rx_online ? channels.channels[12] : invalid_channel_value,
      rx_online ? channels.channels[13] : invalid_channel_value,
      rx_online ? channels.channels[14] : invalid_channel_value,
      rx_online ? channels.channels[15] : invalid_channel_value,
      mavlink_unused_channel_value, mavlink_unused_channel_value, rssi);

  TxFrameState frame{};
  frame.len = static_cast<uint16_t>(mavlink_msg_to_send_buffer(frame.buf, &m));
  frame.is_hb = false;
  return frame;
}

std::optional<Mavlink::TxFrameState> Mavlink::StartNextScheduledFrame(
    const Config::Tx &cfg_tx, uint32_t now_ms) {
  // Among periodic streams, send whichever frame has been due the longest.
  // This avoids permanently favoring a stream that just happens to be checked
  // first in the list below.
  enum class Pick : uint8_t {
    kNone,
    kSys,
    kGps,
    kAtt,
    kGpos,
    kBatt,
    kRc,
  } pick = Pick::kNone;

  uint32_t best_age = 0;
  const auto consider = [&](bool enabled, uint32_t due_ms, Pick candidate) {
    if (!enabled) {
      return;
    }

    const int32_t age = static_cast<int32_t>(now_ms - due_ms);
    if (age < 0) {
      return;
    }

    if (pick == Pick::kNone || static_cast<uint32_t>(age) > best_age) {
      best_age = static_cast<uint32_t>(age);
      pick = candidate;
    }
  };

  consider(true, tx_schedule_.next_sys_ms, Pick::kSys);
  consider(cfg_tx.periods.gps_ms > 0, tx_schedule_.next_gps_ms, Pick::kGps);
  consider(cfg_tx.periods.att_ms > 0, tx_schedule_.next_att_ms, Pick::kAtt);
  consider(cfg_tx.periods.gpos_ms > 0, tx_schedule_.next_gpos_ms, Pick::kGpos);
  consider(cfg_tx.periods.batt_ms > 0, tx_schedule_.next_batt_ms, Pick::kBatt);
  consider(cfg_tx.periods.rc_ms > 0, tx_schedule_.next_rc_ms, Pick::kRc);

  switch (pick) {
    case Pick::kSys:
      return StartSysStatusFrame();
    case Pick::kGps:
      return StartGpsRawIntFrame(cfg_tx);
    case Pick::kAtt:
      return StartAttitudeFrame(cfg_tx);
    case Pick::kGpos:
      return StartGlobalPositionIntFrame(cfg_tx);
    case Pick::kBatt:
      return StartBatteryStatusFrame(cfg_tx);
    case Pick::kRc:
      return StartRcChannelsFrame(cfg_tx);
    case Pick::kNone:
    default:
      break;
  }

  return std::nullopt;
}

void Mavlink::ServiceUdpTx(uint32_t now_ms) {
  if (!Sys().Wifi().HasAssociatedStations()) {
    return;
  }

  if (!StartNextFrameIfIdle(udp_tx_, cfg_.tx, now_ms)) {
    return;
  }

  const int sent = udp_->Send(tx_frame_.buf, tx_frame_.len);
  if (sent > 0) {
    udp_tx_packet_count_.fetch_add(1, std::memory_order_relaxed);
  }
  CompleteFrame(tx_frame_, now_ms);
}
