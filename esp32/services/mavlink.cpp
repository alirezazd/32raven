#include "mavlink.hpp"

#include <cstring>

#include "../../third_party/mavlink/standard/mavlink_msg_autopilot_version.h"
#include "../drivers/uart.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "panic.hpp"
#include "system.hpp"

static constexpr const char *kTag = "mavlink";

namespace {

int64_t DaysFromCivil(int64_t year, unsigned month, unsigned day) {
  year -= month <= 2 ? 1 : 0;
  const int64_t era = (year >= 0 ? year : year - 399) / 400;
  const unsigned yoe = (unsigned)(year - era * 400);
  const unsigned doy =
      (153u * (month + (month > 2 ? (unsigned)-3 : 9u)) + 2u) / 5u + day - 1u;
  const unsigned doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
  return era * 146097 + (int64_t)doe - 719468;
}

bool TryBuildGpsUnixUsec(const message::GpsData &gps, uint64_t &unix_usec) {
  if (gps.year < 1970 || gps.month < 1 || gps.month > 12 || gps.day < 1 ||
      gps.day > 31 || gps.hour > 23 || gps.min > 59 || gps.sec > 59) {
    return false;
  }

  const int64_t days = DaysFromCivil(gps.year, gps.month, gps.day);
  if (days < 0) {
    return false;
  }

  const int64_t seconds =
      days * 86400 + (int64_t)gps.hour * 3600 + (int64_t)gps.min * 60 +
      (int64_t)gps.sec;
  if (seconds < 0) {
    return false;
  }

  unix_usec = (uint64_t)seconds * 1000000ull;
  return true;
}

enum class ParamKey : uint8_t {
  kSysId,
  kCompId,
  kHeartbeatMs,
  kGpsMs,
  kAttMs,
  kGposMs,
  kBattMs,
};

struct ParamDef {
  const char *id;
  uint8_t type;
  ParamKey key;
};

constexpr ParamDef kParamTable[] = {
    {"SYSID_THISMAV", MAV_PARAM_TYPE_UINT8, ParamKey::kSysId},
    {"SYS_COMP_ID", MAV_PARAM_TYPE_UINT8, ParamKey::kCompId},
    {"MAV_HB_MS", MAV_PARAM_TYPE_UINT16, ParamKey::kHeartbeatMs},
    {"MAV_GPS_MS", MAV_PARAM_TYPE_UINT16, ParamKey::kGpsMs},
    {"MAV_ATT_MS", MAV_PARAM_TYPE_UINT16, ParamKey::kAttMs},
    {"MAV_GPOS_MS", MAV_PARAM_TYPE_UINT16, ParamKey::kGposMs},
    {"MAV_BATT_MS", MAV_PARAM_TYPE_UINT16, ParamKey::kBattMs},
};

constexpr uint64_t kMavProtocolCapabilityParamFloat = 1ull << 1;
constexpr uint64_t kMavProtocolCapabilityMavlink2 = 1ull << 13;
constexpr uint64_t kMavProtocolCapabilityParamEncodeCCast = 1ull << 17;
constexpr uint16_t kSysStatusMs = 1000;
constexpr uint16_t kSysStatusStartDelayMs = 250;
constexpr size_t kCommandQueueDepth = 8;

}  // namespace

Mavlink::Mavlink() {}
Mavlink::~Mavlink() {}

Mavlink &Mavlink::GetInstance() {
  static Mavlink instance;
  return instance;
}

void Mavlink::ArmFirstSchedule(TxState &tx, uint32_t now_ms) {
  // Stagger streams to avoid bursts on ELRS limited bandwidth
  tx.next_hb_ms = now_ms;
  tx.next_sys_ms = now_ms + kSysStatusStartDelayMs;
  tx.next_gps_ms = now_ms + cfg_.tx.schedule.gps_start_delay_ms;
  tx.next_att_ms = now_ms + cfg_.tx.schedule.att_start_delay_ms;
  tx.next_gpos_ms = now_ms + cfg_.tx.schedule.gpos_start_delay_ms;
  tx.next_batt_ms = now_ms + cfg_.tx.schedule.batt_start_delay_ms;
}

bool Mavlink::ShouldSendHbNow(const TxState &tx, uint32_t now_ms) const {
  // Deadline guarantee: never exceed the configured gap between completed HBs.
  // Also honor the nominal cadence using next_hb_ms_.
  if (cfg_.tx.periods.hb_ms == 0 || cfg_.tx.schedule.hb_deadline_ms == 0) {
    return false;
  }
  int32_t since_done = (int32_t)(now_ms - tx.last_hb_done_ms);
  if (since_done >= (int32_t)cfg_.tx.schedule.hb_deadline_ms) {
    return true;
  }
  return (int32_t)(now_ms - tx.next_hb_ms) >= 0;
}

void Mavlink::Init(const Config &cfg, UartRcRx *uart, UdpServer *udp) {
  static StaticQueue_t command_queue_buffer;
  static uint8_t command_queue_storage[kCommandQueueDepth *
                                       sizeof(CommandLongEvent)];
  if (uart == nullptr || udp == nullptr) {
    Panic(ErrorCode::kMavlinkInitFailed);
  }
  if (cfg.identity.sysid == 0 || cfg.rx.read_chunk_size == 0 ||
      cfg.tx.periods.hb_ms == 0 || cfg.tx.schedule.hb_deadline_ms == 0 ||
      cfg.rx.read_chunk_size > Mavlink::kMaxRxReadChunkSize) {
    Panic(ErrorCode::kMavlinkInitFailed);
  }

  cfg_ = cfg;
  uart_ = uart;
  udp_ = udp;
  rc_state_ = RcState{};
  rx_msg_ = mavlink_message_t{};
  rx_status_ = mavlink_status_t{};
  command_queue_ =
      xQueueCreateStatic(kCommandQueueDepth, sizeof(CommandLongEvent),
                         command_queue_storage, &command_queue_buffer);
  if (command_queue_ == nullptr) {
    Panic(ErrorCode::kMavlinkInitFailed);
  }

  // Clear TX state
  rc_tx_ = TxState{};
  udp_tx_ = TxState{};

  // Heartbeat timing: allow immediate first HB
  have_latest_ = false;
  latest_update_ms_ = 0;
  ArmFirstSchedule(rc_tx_, 0);
  ArmFirstSchedule(udp_tx_, 0);

  ESP_LOGI(kTag, "Initialized (RX RC + TX ELRS set)");

  extern void StartMavlinkRcTask();
  StartMavlinkRcTask();
}

void Mavlink::Poll() {
  size_t read_chunk_size = cfg_.rx.read_chunk_size;
  if (read_chunk_size > rx_buf_.size()) {
    read_chunk_size = rx_buf_.size();
  }

  int n = uart_->Read(rx_buf_.data(), read_chunk_size, 0);
  if (n > 0) {
    for (int i = 0; i < n; i++) {
      HandleRxByte(rx_buf_[i]);
    }
  }
}

void Mavlink::HandleRxByte(uint8_t b) {
  if (mavlink_parse_char(MAVLINK_COMM_1, b, &rx_msg_, &rx_status_)) {
    HandleMessage(rx_msg_, RxSource::kRc);
  }
}

void Mavlink::HandleMessage(const mavlink_message_t &msg, RxSource source) {
  TxState &tx = (source == RxSource::kUdp) ? udp_tx_ : rc_tx_;

  switch (msg.msgid) {
    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {
      mavlink_rc_channels_override_t rc{};
      mavlink_msg_rc_channels_override_decode(&msg, &rc);

      rc_state_.channels[0] = rc.chan1_raw;
      rc_state_.channels[1] = rc.chan2_raw;
      rc_state_.channels[2] = rc.chan3_raw;
      rc_state_.channels[3] = rc.chan4_raw;
      rc_state_.channels[4] = rc.chan5_raw;
      rc_state_.channels[5] = rc.chan6_raw;
      rc_state_.channels[6] = rc.chan7_raw;
      rc_state_.channels[7] = rc.chan8_raw;
      rc_state_.channels[8] = rc.chan9_raw;
      rc_state_.channels[9] = rc.chan10_raw;
      rc_state_.channels[10] = rc.chan11_raw;
      rc_state_.channels[11] = rc.chan12_raw;
      rc_state_.channels[12] = rc.chan13_raw;
      rc_state_.channels[13] = rc.chan14_raw;
      rc_state_.channels[14] = rc.chan15_raw;
      rc_state_.channels[15] = rc.chan16_raw;
      break;
    }
    case MAVLINK_MSG_ID_RADIO_STATUS: {
      mavlink_radio_status_t radio{};
      mavlink_msg_radio_status_decode(&msg, &radio);
      rc_state_.rssi = radio.rssi;
      break;
    }
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
      mavlink_param_request_list_t req{};
      mavlink_msg_param_request_list_decode(&msg, &req);
      if (req.target_system == cfg_.identity.sysid &&
          (req.target_component == cfg_.identity.compid ||
           req.target_component == 0 ||
           req.target_component == MAV_COMP_ID_ALL)) {
        tx.param_stream_active = true;
        tx.next_param_index = 0;
      }
      break;
    }
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
      mavlink_mission_request_list_t req{};
      mavlink_msg_mission_request_list_decode(&msg, &req);
      if (req.target_system == cfg_.identity.sysid &&
          (req.target_component == cfg_.identity.compid ||
           req.target_component == 0 ||
           req.target_component == MAV_COMP_ID_ALL)) {
        tx.pending_mission_count = true;
        tx.pending_mission_type = req.mission_type;
      }
      break;
    }
    case MAVLINK_MSG_ID_COMMAND_LONG: {
      mavlink_command_long_t cmd{};
      mavlink_msg_command_long_decode(&msg, &cmd);
      if (cmd.target_system != cfg_.identity.sysid ||
          (cmd.target_component != cfg_.identity.compid &&
           cmd.target_component != 0 &&
           cmd.target_component != MAV_COMP_ID_ALL)) {
        break;
      }

      if (command_queue_ != nullptr) {
        const CommandLongEvent ev = {
            .link = (source == RxSource::kUdp) ? Link::kUdp : Link::kRc,
            .msg = msg,
        };
        if (xQueueSend((QueueHandle_t)command_queue_, &ev, 0) != pdPASS) {
          ESP_LOGW(kTag, "dropping COMMAND_LONG: queue full");
        }
      }
      break;
    }

    default:
      break;
  }
}

void Mavlink::MaybeLogUdpMessage(const mavlink_message_t &msg) {
  for (uint8_t i = 0; i < udp_logged_msgid_count_; ++i) {
    if (udp_logged_msgids_[i] == msg.msgid) {
      return;
    }
  }

  if (udp_logged_msgid_count_ < udp_logged_msgids_.size()) {
    udp_logged_msgids_[udp_logged_msgid_count_++] = (uint16_t)msg.msgid;
  }

  const char *name = "UNKNOWN";
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
      name = "HEARTBEAT";
      break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
      name = "PARAM_REQUEST_LIST";
      break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
      name = "PARAM_REQUEST_READ";
      break;
    case MAVLINK_MSG_ID_COMMAND_LONG:
      name = "COMMAND_LONG";
      break;
    case MAVLINK_MSG_ID_COMMAND_INT:
      name = "COMMAND_INT";
      break;
    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
      name = "REQUEST_DATA_STREAM";
      break;
    case MAVLINK_MSG_ID_TIMESYNC:
      name = "TIMESYNC";
      break;
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
      name = "MISSION_REQUEST_LIST";
      break;
    case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE:
      name = "RC_CHANNELS_OVERRIDE";
      break;
    default:
      break;
  }

  if (msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG) {
    mavlink_command_long_t cmd{};
    mavlink_msg_command_long_decode(&msg, &cmd);
    if (cmd.command == MAV_CMD_REQUEST_MESSAGE) {
      ESP_LOGI(kTag,
               "UDP RX COMMAND_LONG cmd=%lu req_msg=%lu target_sys=%u "
               "target_comp=%u",
               (unsigned long)cmd.command, (unsigned long)cmd.param1,
               (unsigned)cmd.target_system, (unsigned)cmd.target_component);
    } else {
      ESP_LOGI(kTag,
               "UDP RX COMMAND_LONG cmd=%lu target_sys=%u target_comp=%u",
               (unsigned long)cmd.command, (unsigned)cmd.target_system,
               (unsigned)cmd.target_component);
    }
    return;
  }

  ESP_LOGI(kTag, "UDP RX msgid=%lu (%s)", (unsigned long)msg.msgid, name);
}

void Mavlink::QueueCommandAck(TxState &tx, uint16_t command, uint8_t result,
                              uint8_t target_system,
                              uint8_t target_component) {
  tx.pending_command_ack = true;
  tx.pending_command = command;
  tx.pending_command_result = result;
  tx.pending_command_target_system = target_system;
  tx.pending_command_target_component = target_component;
}

Mavlink::TxState &Mavlink::TxForLink(Link link) {
  return (link == Link::kUdp) ? udp_tx_ : rc_tx_;
}

bool Mavlink::PopCommandLongEvent(CommandLongEvent &event) {
  if (command_queue_ == nullptr) {
    return false;
  }
  return xQueueReceive((QueueHandle_t)command_queue_, &event, 0) == pdPASS;
}

uint32_t Mavlink::UdpRxPacketCount() const {
  return udp_rx_packet_count_.load(std::memory_order_relaxed);
}

uint32_t Mavlink::UdpTxPacketCount() const {
  return udp_tx_packet_count_.load(std::memory_order_relaxed);
}

void Mavlink::QueueCommandAck(Link link, uint16_t command, uint8_t result,
                              uint8_t target_system,
                              uint8_t target_component) {
  QueueCommandAck(TxForLink(link), command, result, target_system,
                  target_component);
}

void Mavlink::QueueAutopilotVersion(Link link) {
  TxForLink(link).pending_autopilot_version = true;
}

void Mavlink::StartCommandAckFrame(TxState &tx) {
  if (!tx.pending_command_ack) {
    return;
  }

  mavlink_message_t m{};
  mavlink_msg_command_ack_pack(
      cfg_.identity.sysid, cfg_.identity.compid, &m, tx.pending_command,
      tx.pending_command_result, UINT8_MAX, 0, tx.pending_command_target_system,
      tx.pending_command_target_component);

  tx.len = (uint16_t)mavlink_msg_to_send_buffer(tx.buf, &m);
  tx.sent = 0;
  tx.is_hb = false;
  tx.pending_command_ack = false;
}

void Mavlink::StartAutopilotVersionFrame(TxState &tx) {
  if (!tx.pending_autopilot_version) {
    return;
  }

  const uint64_t capabilities = kMavProtocolCapabilityParamFloat |
                                kMavProtocolCapabilityMavlink2 |
                                kMavProtocolCapabilityParamEncodeCCast;
  static constexpr uint8_t kZeroHash[8] = {};
  static constexpr uint8_t kZeroUid2[18] = {};

  mavlink_message_t m{};
  // TODO: Populate real firmware/build version fields once version metadata is
  // available in the firmware.
  mavlink_msg_autopilot_version_pack(
      cfg_.identity.sysid, cfg_.identity.compid, &m, capabilities,
      0,  // flight_sw_version
      0,  // middleware_sw_version
      0,  // os_sw_version
      0,  // board_version
      kZeroHash, kZeroHash, kZeroHash,
      0,  // vendor_id
      0,  // product_id
      0,  // uid
      kZeroUid2);

  tx.len = (uint16_t)mavlink_msg_to_send_buffer(tx.buf, &m);
  tx.sent = 0;
  tx.is_hb = false;
  tx.pending_autopilot_version = false;
}

void Mavlink::StartMissionCountFrame(TxState &tx) {
  if (!tx.pending_mission_count) {
    return;
  }

  mavlink_message_t m{};
  mavlink_msg_mission_count_pack(cfg_.identity.sysid, cfg_.identity.compid, &m,
                                 cfg_.identity.sysid, cfg_.identity.compid,
                                 0, tx.pending_mission_type, 0);

  tx.len = (uint16_t)mavlink_msg_to_send_buffer(tx.buf, &m);
  tx.sent = 0;
  tx.is_hb = false;
  tx.pending_mission_count = false;
}

void Mavlink::StartStatusTextFrame(TxState &tx) {
  if (!tx.pending_statustext) {
    return;
  }

  char text[51] = {};
  std::strncpy(text, tx.pending_statustext_text, sizeof(text) - 1);

  mavlink_message_t m{};
  mavlink_msg_statustext_pack(cfg_.identity.sysid, cfg_.identity.compid, &m,
                              tx.pending_statustext_severity, text,
                              0,  // id
                              0   // chunk_seq
  );

  tx.len = (uint16_t)mavlink_msg_to_send_buffer(tx.buf, &m);
  tx.sent = 0;
  tx.is_hb = false;
  tx.pending_statustext = false;
  tx.pending_statustext_text[0] = '\0';
}

void Mavlink::StartParamValueFrame(TxState &tx) {
  if (!tx.param_stream_active) {
    return;
  }

  const uint16_t param_count =
      static_cast<uint16_t>(sizeof(kParamTable) / sizeof(kParamTable[0]));
  if (tx.next_param_index >= param_count) {
    tx.param_stream_active = false;
    tx.next_param_index = 0;
    return;
  }

  const ParamDef &def = kParamTable[tx.next_param_index];
  float param_value = 0.0f;
  switch (def.key) {
    case ParamKey::kSysId:
      param_value = static_cast<float>(cfg_.identity.sysid);
      break;
    case ParamKey::kCompId:
      param_value = static_cast<float>(cfg_.identity.compid);
      break;
    case ParamKey::kHeartbeatMs:
      param_value = static_cast<float>(cfg_.tx.periods.hb_ms);
      break;
    case ParamKey::kGpsMs:
      param_value = static_cast<float>(cfg_.tx.periods.gps_ms);
      break;
    case ParamKey::kAttMs:
      param_value = static_cast<float>(cfg_.tx.periods.att_ms);
      break;
    case ParamKey::kGposMs:
      param_value = static_cast<float>(cfg_.tx.periods.gpos_ms);
      break;
    case ParamKey::kBattMs:
      param_value = static_cast<float>(cfg_.tx.periods.batt_ms);
      break;
  }

  char param_id[17] = {};
  std::strncpy(param_id, def.id, 16);

  mavlink_message_t m{};
  mavlink_msg_param_value_pack(cfg_.identity.sysid, cfg_.identity.compid, &m,
                               param_id, param_value, def.type, param_count,
                               tx.next_param_index);

  tx.len = (uint16_t)mavlink_msg_to_send_buffer(tx.buf, &m);
  tx.sent = 0;
  tx.is_hb = false;
  tx.next_param_index++;
  if (tx.next_param_index >= param_count) {
    tx.param_stream_active = false;
    tx.next_param_index = 0;
  }
}

// ---------------- Telemetry input (from STM32) ----------------

void Mavlink::OfferTelemetry(const message::GpsData &d, uint32_t now_ms) {
  latest_ = d;
  have_latest_ = true;
  latest_update_ms_ = now_ms;
}

void Mavlink::QueueStatusText(const char *text, uint8_t severity) {
  if (text == nullptr || text[0] == '\0') {
    return;
  }

  auto queue = [&](TxState &tx) {
    tx.pending_statustext = true;
    tx.pending_statustext_severity = severity;
    std::strncpy(tx.pending_statustext_text, text,
                 sizeof(tx.pending_statustext_text) - 1);
    tx.pending_statustext_text[sizeof(tx.pending_statustext_text) - 1] = '\0';
  };

  queue(rc_tx_);
  queue(udp_tx_);
}

// ---------------- TX core ----------------

void Mavlink::ServiceInFlightTx(TxState &tx) {
  if (!uart_ || tx.len == 0) {
    return;
  }

  uint16_t remaining = (uint16_t)(tx.len - tx.sent);
  if (remaining == 0) {
    return;
  }

  int wrote = uart_->Write(tx.buf + tx.sent, remaining);
  if (wrote > 0) {
    tx.sent = (uint16_t)(tx.sent + (uint16_t)wrote);
  }
}

void Mavlink::StartHeartbeatFrame(TxState &tx) {
  mavlink_message_t m{};

  // TODO: Make vehicle type configurable instead of hardcoding quadrotor.
  mavlink_msg_heartbeat_pack(cfg_.identity.sysid, cfg_.identity.compid, &m,
                             MAV_TYPE_QUADROTOR, MAV_AUTOPILOT_GENERIC, 0, 0,
                             MAV_STATE_ACTIVE);

  tx.len = (uint16_t)mavlink_msg_to_send_buffer(tx.buf, &m);
  tx.sent = 0;
  tx.is_hb = true;

  tx.next_hb_ms += cfg_.tx.periods.hb_ms;
}

void Mavlink::StartSysStatusFrame(TxState &tx) {
  tx.next_sys_ms += kSysStatusMs;

  uint32_t sensors_present = 0;
  uint32_t sensors_enabled = 0;
  uint32_t sensors_health = 0;
  uint16_t voltage_battery = 0;
  int16_t current_battery = -1;
  int8_t battery_remaining = -1;

  if (have_latest_) {
    sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
    sensors_enabled |= MAV_SYS_STATUS_SENSOR_GPS;
    if (latest_.fixType >= 2) {
      sensors_health |= MAV_SYS_STATUS_SENSOR_GPS;
    }

    if (latest_.batt_voltage > 0) {
      sensors_present |= MAV_SYS_STATUS_SENSOR_BATTERY;
      sensors_enabled |= MAV_SYS_STATUS_SENSOR_BATTERY;
      sensors_health |= MAV_SYS_STATUS_SENSOR_BATTERY;
      voltage_battery = latest_.batt_voltage;
      current_battery = latest_.batt_current;
      battery_remaining = latest_.batt_remaining;
    }
  }

  mavlink_message_t m{};
  mavlink_msg_sys_status_pack(
      cfg_.identity.sysid, cfg_.identity.compid, &m, sensors_present,
      sensors_enabled, sensors_health,
      0,  // load unknown
      voltage_battery, current_battery, battery_remaining,
      0,  // drop_rate_comm
      0,  // errors_comm
      0,  // errors_count1
      0,  // errors_count2
      0,  // errors_count3
      0,  // errors_count4
      0,  // sensors_present_extended
      0,  // sensors_enabled_extended
      0   // sensors_health_extended
  );

  tx.len = (uint16_t)mavlink_msg_to_send_buffer(tx.buf, &m);
  tx.sent = 0;
  tx.is_hb = false;
}

void Mavlink::StartGpsRawIntFrame(TxState &tx) {
  tx.next_gps_ms += cfg_.tx.periods.gps_ms;

  // Only send if we have telemetry
  if (!have_latest_) {
    return;
  }

  mavlink_message_t m{};
  uint64_t time_usec = 0;
  (void)TryBuildGpsUnixUsec(latest_, time_usec);

  // MAVLink expects: lat/lon 1e7 deg, alt mm, vel cm/s, cog cdeg
  // fixType maps well to MAVLink fix_type.

  mavlink_msg_gps_raw_int_pack(
      cfg_.identity.sysid, cfg_.identity.compid, &m,
      time_usec,
      (uint8_t)latest_.fixType, latest_.lat, latest_.lon,
      (int32_t)latest_.hMSL,           // alt (mm)
      (uint16_t)(latest_.hAcc / 10u),  // eph (cm) rough
      (uint16_t)(latest_.vAcc / 10u),  // epv (cm) rough
      (uint16_t)latest_.vel,           // vel (cm/s)
      (uint16_t)latest_.hdg,           // cog (cdeg)
      (uint8_t)latest_.numSV,
      0,  // alt_ellipsoid
      0,  // h_acc
      0,  // v_acc
      0,  // vel_acc
      0,  // hdg_acc
      0   // yaw
  );

  tx.len = (uint16_t)mavlink_msg_to_send_buffer(tx.buf, &m);
  tx.sent = 0;
  tx.is_hb = false;
}

void Mavlink::StartAttitudeFrame(TxState &tx) {
  tx.next_att_ms += cfg_.tx.periods.att_ms;

  if (!have_latest_) {
    return;
  }

  mavlink_message_t m{};

  // latest_.roll/pitch/yaw are cdeg -> convert to rad
  // rad = (cdeg / 100.0) * pi/180
  float roll = ((float)latest_.roll * 0.01f) * 0.017453292519943295f;
  float pitch = ((float)latest_.pitch * 0.01f) * 0.017453292519943295f;
  float yaw = ((float)latest_.yaw * 0.01f) * 0.017453292519943295f;

  mavlink_msg_attitude_pack(cfg_.identity.sysid, cfg_.identity.compid, &m,
                            0,  // time_boot_ms unknown here
                            roll, pitch, yaw, 0.0f, 0.0f, 0.0f);

  tx.len = (uint16_t)mavlink_msg_to_send_buffer(tx.buf, &m);
  tx.sent = 0;
  tx.is_hb = false;
}

void Mavlink::StartGlobalPositionIntFrame(TxState &tx) {
  tx.next_gpos_ms += cfg_.tx.periods.gpos_ms;

  if (!have_latest_) {
    return;
  }

  mavlink_message_t m{};

  // ELRS uses relative_alt and vz for vario. We don't have relative_alt/vz in
  // GpsData. Use hMSL as a placeholder for alt and set vz=0 unless you later
  // add vertical speed in your STM32 packet.
  int32_t lat = latest_.lat;
  int32_t lon = latest_.lon;
  int32_t alt = latest_.hMSL;      // mm
  int32_t rel_alt = latest_.hMSL;  // mm (placeholder for relative)
  int16_t vx = 0;                  // cm/s
  int16_t vy = 0;                  // cm/s
  int16_t vz = 0;                  // cm/s
  uint16_t hdg = latest_.hdg;      // cdeg

  mavlink_msg_global_position_int_pack(cfg_.identity.sysid,
                                       cfg_.identity.compid, &m,
                                       0,  // time_boot_ms unknown
                                       lat, lon, alt, rel_alt, vx, vy, vz, hdg);

  tx.len = (uint16_t)mavlink_msg_to_send_buffer(tx.buf, &m);
  tx.sent = 0;
  tx.is_hb = false;
}

void Mavlink::StartBatteryStatusFrame(TxState &tx) {
  tx.next_batt_ms += cfg_.tx.periods.batt_ms;

  if (!have_latest_) {
    return;
  }

  mavlink_message_t m{};

  // MAVLink BATTERY_STATUS:
  // - voltages[] is mV per cell, 0xFFFF unknown
  // We only have pack voltage (mV). Put it in voltages[0] and leave rest
  // unknown.
  uint16_t voltages[10];
  for (int i = 0; i < 10; ++i) {
    voltages[i] = 0xFFFF;
  }
  voltages[0] = latest_.batt_voltage;

  mavlink_msg_battery_status_pack(
      cfg_.identity.sysid, cfg_.identity.compid, &m,
      0,  // id
      MAV_BATTERY_FUNCTION_ALL, MAV_BATTERY_TYPE_LIPO,
      0,  // temperature unknown
      voltages,
      (int16_t)latest_.batt_current,  // cA in your struct; MAVLink expects cA
      (int32_t)latest_
          .batt_current,  // current_consumed: you have cA, not mAh.
                          // You should replace this once STM32 provides mAh.
      0,                  // energy_consumed unknown
      (int8_t)latest_.batt_remaining,
      0,         // time_remaining
      0,         // charge_state
      voltages,  // voltages_ext (safe re-use, ignores extra)
      0,         // mode
      0          // fault_bitmask
  );

  tx.len = (uint16_t)mavlink_msg_to_send_buffer(tx.buf, &m);
  tx.sent = 0;
  tx.is_hb = false;
}

void Mavlink::StartNextScheduledFrame(TxState &tx, uint32_t now_ms) {
  // Heartbeat is handled outside as hard priority.
  // Here we select the next due stream among sys/gps/att/gpos/batt.
  // Keep this simple and deterministic.

  // Find earliest due among enabled streams.
  // If multiple are due, pick the one with the oldest deadline (smallest
  // next_*).
  uint32_t best_due = 0xFFFFFFFFu;
  enum class Pick : uint8_t {
    kNone,
    kSys,
    kGps,
    kAtt,
    kGpos,
    kBatt
  } pick = Pick::kNone;

  if ((int32_t)(now_ms - tx.next_sys_ms) >= 0) {
    best_due = tx.next_sys_ms;
    pick = Pick::kSys;
  }
  if (cfg_.tx.periods.gps_ms > 0 && (int32_t)(now_ms - tx.next_gps_ms) >= 0) {
    if (tx.next_gps_ms < best_due) {
      best_due = tx.next_gps_ms;
      pick = Pick::kGps;
    }
  }
  if (cfg_.tx.periods.att_ms > 0 && (int32_t)(now_ms - tx.next_att_ms) >= 0) {
    if (tx.next_att_ms < best_due) {
      best_due = tx.next_att_ms;
      pick = Pick::kAtt;
    }
  }
  if (cfg_.tx.periods.gpos_ms > 0 &&
      (int32_t)(now_ms - tx.next_gpos_ms) >= 0) {
    if (tx.next_gpos_ms < best_due) {
      best_due = tx.next_gpos_ms;
      pick = Pick::kGpos;
    }
  }
  if (cfg_.tx.periods.batt_ms > 0 &&
      (int32_t)(now_ms - tx.next_batt_ms) >= 0) {
    if (tx.next_batt_ms < best_due) {
      best_due = tx.next_batt_ms;
      pick = Pick::kBatt;
    }
  }

  switch (pick) {
    case Pick::kSys:
      StartSysStatusFrame(tx);
      break;
    case Pick::kGps:
      StartGpsRawIntFrame(tx);
      break;
    case Pick::kAtt:
      StartAttitudeFrame(tx);
      break;
    case Pick::kGpos:
      StartGlobalPositionIntFrame(tx);
      break;
    case Pick::kBatt:
      StartBatteryStatusFrame(tx);
      break;
    case Pick::kNone:
    default:
      break;
  }
}

void Mavlink::TxTick(uint32_t now_ms) {
  TxState &tx = rc_tx_;
  // Arm schedule on first real tick
  if (!tx.schedule_armed) {
    ArmFirstSchedule(tx, now_ms);
    // Force immediate heartbeat
    tx.last_hb_done_ms = now_ms - cfg_.tx.schedule.hb_deadline_ms;
    tx.schedule_armed = true;
  }

  // 1) Finish writing any in-flight frame (atomic frame output)
  if (tx.len > 0 && tx.sent < tx.len) {
    ServiceInFlightTx(tx);
  }

  // 2) If we finished pushing bytes to driver, wait for TX done then clear
  // state
  if (tx.len > 0 && tx.sent >= tx.len) {
    if (tx.is_hb) {
      tx.last_hb_done_ms = now_ms;
      tx.is_hb = false;
    }
    tx.len = 0;
    tx.sent = 0;
    return;
  }

  // 3) Nothing in flight: heartbeat has hard priority
  if (ShouldSendHbNow(tx, now_ms)) {
    StartHeartbeatFrame(tx);
    ServiceInFlightTx(tx);
    return;
  }

  // 4) Send one scheduled stream frame if due
  if (tx.pending_command_ack) {
    StartCommandAckFrame(tx);
  } else if (tx.pending_autopilot_version) {
    StartAutopilotVersionFrame(tx);
  } else if (tx.pending_mission_count) {
    StartMissionCountFrame(tx);
  } else if (tx.pending_statustext) {
    StartStatusTextFrame(tx);
  } else if (tx.param_stream_active) {
    StartParamValueFrame(tx);
  } else {
    StartNextScheduledFrame(tx, now_ms);
  }
  if (tx.len > 0) {
    ServiceInFlightTx(tx);
  }
}

void Mavlink::UdpTick(uint32_t now_ms) {
  if (udp_ == nullptr) {
    return;
  }

  if (!Sys().Wifi().HasAssociatedStations()) {
    udp_->ClearPeer();
    return;
  }

  const bool had_peer = udp_->HasPeer();
  uint8_t rx_buf[512];
  const int received = udp_->Receive(rx_buf, sizeof(rx_buf));
  if (received > 0) {
    if (!had_peer && udp_->HasPeer()) {
      QueueStatusText("MAVLink WiFi ready", MAV_SEVERITY_INFO);
    }
    mavlink_message_t msg{};
    mavlink_status_t status{};
    for (int i = 0; i < received; ++i) {
      if (mavlink_parse_char(MAVLINK_COMM_2, rx_buf[i], &msg, &status)) {
        udp_rx_packet_count_.fetch_add(1, std::memory_order_relaxed);
        MaybeLogUdpMessage(msg);
        HandleMessage(msg, RxSource::kUdp);
      }
    }
  }

  TxState &tx = udp_tx_;
  if (!tx.schedule_armed) {
    ArmFirstSchedule(tx, now_ms);
    tx.last_hb_done_ms = now_ms - cfg_.tx.schedule.hb_deadline_ms;
    tx.schedule_armed = true;
  }

  if (tx.len == 0) {
    if (ShouldSendHbNow(tx, now_ms)) {
      StartHeartbeatFrame(tx);
    } else if (tx.pending_command_ack) {
      StartCommandAckFrame(tx);
    } else if (tx.pending_autopilot_version) {
      StartAutopilotVersionFrame(tx);
    } else if (tx.pending_mission_count) {
      StartMissionCountFrame(tx);
    } else if (tx.pending_statustext) {
      StartStatusTextFrame(tx);
    } else if (tx.param_stream_active) {
      StartParamValueFrame(tx);
    } else {
      StartNextScheduledFrame(tx, now_ms);
    }
  }

  if (tx.len == 0) {
    return;
  }

  const int sent = udp_->Send(tx.buf, tx.len);
  tx.len = 0;
  tx.sent = 0;
  if (sent > 0) {
    udp_tx_packet_count_.fetch_add(1, std::memory_order_relaxed);
    if (tx.is_hb) {
      tx.last_hb_done_ms = now_ms;
    }
  }
  tx.is_hb = false;
}
