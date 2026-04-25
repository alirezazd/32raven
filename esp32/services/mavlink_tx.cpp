#include <mavlink.h>

#include <cstring>
#include <type_traits>

#include "../../third_party/mavlink/standard/mavlink_msg_autopilot_version.h"
#include "esp32_limits.hpp"
#include "esp_log.h"
#include "mavlink.hpp"
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

void Mavlink::QueueCommandAck(uint16_t command, uint8_t result,
                              uint8_t target_system, uint8_t target_component) {
  QueueTxItem(CommandAck{command, result, target_system, target_component});
}

void Mavlink::QueueAutopilotVersion() { QueueTxItem(AutopilotVersion{}); }

void Mavlink::QueueMissionCount(uint8_t target_system, uint8_t target_component,
                                uint8_t mission_type) {
  QueueTxItem(MissionCount{target_system, target_component, mission_type});
}

bool Mavlink::StartQueuedTxWorkFrame(TxFrameState &frame) {
  TxQueueItem work{};
  if (!tx_work_queue_.Pop(work)) {
    return false;
  }

  std::visit(
      [&](const auto &item) {
        using Item = std::decay_t<decltype(item)>;
        if constexpr (std::is_same_v<Item, std::monostate>) {
          return;
        } else if constexpr (std::is_same_v<Item, CommandAck>) {
          StartCommandAckFrame(item, frame);
        } else if constexpr (std::is_same_v<Item, AutopilotVersion>) {
          StartAutopilotVersionFrame(item, frame);
        } else if constexpr (std::is_same_v<Item, MissionCount>) {
          StartMissionCountFrame(item, frame);
        } else if constexpr (std::is_same_v<Item, StatusText>) {
          StartStatusTextFrame(item, frame);
        }
      },
      work);
  return true;
}

void Mavlink::StartCommandAckFrame(const CommandAck &ack, TxFrameState &frame) {
  mavlink_message_t m{};
  mavlink_msg_command_ack_pack(cfg_.identity.sysid, cfg_.identity.compid, &m,
                               ack.command, ack.result, UINT8_MAX, 0,
                               ack.target_system, ack.target_component);

  frame.len = static_cast<uint16_t>(mavlink_msg_to_send_buffer(frame.buf, &m));
  frame.is_hb = false;
}

void Mavlink::StartAutopilotVersionFrame(const AutopilotVersion &work,
                                         TxFrameState &frame) {
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

  frame.len = static_cast<uint16_t>(mavlink_msg_to_send_buffer(frame.buf, &m));
  frame.is_hb = false;
}

void Mavlink::StartMissionCountFrame(const MissionCount &work,
                                     TxFrameState &frame) {
  mavlink_message_t m{};
  mavlink_msg_mission_count_pack(cfg_.identity.sysid, cfg_.identity.compid, &m,
                                 work.target_system, work.target_component, 0,
                                 work.mission_type, 0);

  frame.len = static_cast<uint16_t>(mavlink_msg_to_send_buffer(frame.buf, &m));
  frame.is_hb = false;
}

void Mavlink::StartStatusTextFrame(const StatusText &work,
                                   TxFrameState &frame) {
  mavlink_message_t m{};
  mavlink_msg_statustext_pack(cfg_.identity.sysid, cfg_.identity.compid, &m,
                              work.severity, work.text, 0, 0);

  frame.len = static_cast<uint16_t>(mavlink_msg_to_send_buffer(frame.buf, &m));
  frame.is_hb = false;
}

void Mavlink::InitTxSchedule(const TxConfig &cfg_tx, uint32_t now_ms,
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

bool Mavlink::ShouldSendHbNow(const TxConfig &cfg_tx, uint32_t now_ms) const {
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

bool Mavlink::StartNextFrameIfIdle(TxState &tx, const TxConfig &cfg_tx,
                                   uint32_t now_ms) {
  if (tx_frame_.len > 0) {
    return true;
  }

  // Priority order matters here: heartbeats and command replies preempt the
  // periodic telemetry streams so the link stays responsive to the GCS.
  if (ShouldSendHbNow(cfg_tx, now_ms)) {
    StartHeartbeatFrame(tx_frame_, cfg_tx);
  } else if (StartQueuedTxWorkFrame(tx_frame_)) {
  } else if (HasPendingParamWork(tx) &&
             StartNextParamFrame(tx, tx_frame_, cfg_.identity.sysid,
                                 cfg_.identity.compid)) {
  } else {
    StartNextScheduledFrame(tx_frame_, cfg_tx, now_ms);
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

void Mavlink::StartHeartbeatFrame(TxFrameState &frame, const TxConfig &cfg_tx) {
  constexpr uint8_t mav_autopilot_32raven = 200;

  mavlink_message_t m{};
  mavlink_msg_heartbeat_pack(cfg_.identity.sysid, cfg_.identity.compid, &m,
                             MAV_TYPE_QUADROTOR, mav_autopilot_32raven, 0, 0,
                             MAV_STATE_ACTIVE);

  frame.len = static_cast<uint16_t>(mavlink_msg_to_send_buffer(frame.buf, &m));
  frame.is_hb = true;
  tx_schedule_.next_hb_ms += cfg_tx.periods.hb_ms;
}

void Mavlink::StartSysStatusFrame(TxFrameState &frame) {
  constexpr uint16_t sys_status_ms = 1000;

  tx_schedule_.next_sys_ms += sys_status_ms;

  uint32_t sensors_present = 0;
  uint32_t sensors_enabled = 0;
  uint32_t sensors_health = 0;
  uint16_t voltage_battery = 0;
  int16_t current_battery = -1;
  int8_t battery_remaining = -1;
  if (const std::optional<message::GpsData> latest = GetLatestGpsData()) {
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

  frame.len = static_cast<uint16_t>(mavlink_msg_to_send_buffer(frame.buf, &m));
  frame.is_hb = false;
}

void Mavlink::StartGpsRawIntFrame(TxFrameState &frame, const TxConfig &cfg_tx) {
  tx_schedule_.next_gps_ms += cfg_tx.periods.gps_ms;

  const std::optional<message::GpsData> latest = GetLatestGpsData();
  if (!latest.has_value()) {
    return;
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

  frame.len = static_cast<uint16_t>(mavlink_msg_to_send_buffer(frame.buf, &m));
  frame.is_hb = false;
}

void Mavlink::StartAttitudeFrame(TxFrameState &frame, const TxConfig &cfg_tx) {
  tx_schedule_.next_att_ms += cfg_tx.periods.att_ms;

  const std::optional<message::GpsData> latest = GetLatestGpsData();
  if (!latest.has_value()) {
    return;
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

  frame.len = static_cast<uint16_t>(mavlink_msg_to_send_buffer(frame.buf, &m));
  frame.is_hb = false;
}

void Mavlink::StartGlobalPositionIntFrame(TxFrameState &frame,
                                          const TxConfig &cfg_tx) {
  tx_schedule_.next_gpos_ms += cfg_tx.periods.gpos_ms;

  const std::optional<message::GpsData> latest = GetLatestGpsData();
  if (!latest.has_value()) {
    return;
  }

  mavlink_message_t m{};
  mavlink_msg_global_position_int_pack(
      cfg_.identity.sysid, cfg_.identity.compid, &m, 0, latest->lat,
      latest->lon, latest->hMSL, latest->hMSL, 0, 0, 0, latest->hdg);

  frame.len = static_cast<uint16_t>(mavlink_msg_to_send_buffer(frame.buf, &m));
  frame.is_hb = false;
}

void Mavlink::StartBatteryStatusFrame(TxFrameState &frame,
                                      const TxConfig &cfg_tx) {
  tx_schedule_.next_batt_ms += cfg_tx.periods.batt_ms;

  const std::optional<message::GpsData> latest = GetLatestGpsData();
  if (!latest.has_value()) {
    return;
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

  frame.len = static_cast<uint16_t>(mavlink_msg_to_send_buffer(frame.buf, &m));
  frame.is_hb = false;
}

void Mavlink::StartRcChannelsFrame(TxFrameState &frame, const TxConfig &cfg_tx) {
  tx_schedule_.next_rc_ms += cfg_tx.periods.rc_ms;

  const std::optional<RcChannelsSample> sample = GetLatestRcChannelsData();
  if (!sample.has_value()) {
    return;
  }

  constexpr uint16_t kInvalidChannelValue = UINT16_MAX;
  constexpr uint16_t kMavlinkUnusedChannelValue = UINT16_MAX;
  constexpr uint8_t kRcChannelCount =
      static_cast<uint8_t>(sizeof(sample->msg.channels) /
                           sizeof(sample->msg.channels[0]));

  const bool rx_online =
      (sample->msg.flags & message::kRcChannelsFlagRxOnline) != 0;
  const uint8_t channel_count = rx_online ? kRcChannelCount : 0u;
  const uint8_t rssi = rx_online ? sample->msg.rssi : UINT8_MAX;

  mavlink_message_t m{};
  mavlink_msg_rc_channels_pack(
      cfg_.identity.sysid, cfg_.identity.compid, &m, sample->update_ms,
      channel_count,
      rx_online ? sample->msg.channels[0] : kInvalidChannelValue,
      rx_online ? sample->msg.channels[1] : kInvalidChannelValue,
      rx_online ? sample->msg.channels[2] : kInvalidChannelValue,
      rx_online ? sample->msg.channels[3] : kInvalidChannelValue,
      rx_online ? sample->msg.channels[4] : kInvalidChannelValue,
      rx_online ? sample->msg.channels[5] : kInvalidChannelValue,
      rx_online ? sample->msg.channels[6] : kInvalidChannelValue,
      rx_online ? sample->msg.channels[7] : kInvalidChannelValue,
      rx_online ? sample->msg.channels[8] : kInvalidChannelValue,
      rx_online ? sample->msg.channels[9] : kInvalidChannelValue,
      rx_online ? sample->msg.channels[10] : kInvalidChannelValue,
      rx_online ? sample->msg.channels[11] : kInvalidChannelValue,
      rx_online ? sample->msg.channels[12] : kInvalidChannelValue,
      rx_online ? sample->msg.channels[13] : kInvalidChannelValue,
      rx_online ? sample->msg.channels[14] : kInvalidChannelValue,
      rx_online ? sample->msg.channels[15] : kInvalidChannelValue,
      kMavlinkUnusedChannelValue, kMavlinkUnusedChannelValue, rssi);

  frame.len = static_cast<uint16_t>(mavlink_msg_to_send_buffer(frame.buf, &m));
  frame.is_hb = false;
}

void Mavlink::StartNextScheduledFrame(TxFrameState &frame,
                                      const TxConfig &cfg_tx, uint32_t now_ms) {
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
      StartSysStatusFrame(frame);
      break;
    case Pick::kGps:
      StartGpsRawIntFrame(frame, cfg_tx);
      break;
    case Pick::kAtt:
      StartAttitudeFrame(frame, cfg_tx);
      break;
    case Pick::kGpos:
      StartGlobalPositionIntFrame(frame, cfg_tx);
      break;
    case Pick::kBatt:
      StartBatteryStatusFrame(frame, cfg_tx);
      break;
    case Pick::kRc:
      StartRcChannelsFrame(frame, cfg_tx);
      break;
    case Pick::kNone:
    default:
      break;
  }
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
