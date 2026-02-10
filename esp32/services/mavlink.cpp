#include "mavlink.hpp"

#include "../drivers/uart.hpp"
#include "esp_log.h"

static constexpr const char *kTag = "mavlink";

Mavlink::Mavlink() {}
Mavlink::~Mavlink() {}

Mavlink &Mavlink::GetInstance() {
  static Mavlink instance;
  return instance;
}

void Mavlink::ArmFirstSchedule(uint32_t now_ms) {
  // Stagger streams to avoid bursts on ELRS limited bandwidth
  next_hb_ms_ = now_ms;
  next_gps_ms_ = now_ms + 200;
  next_att_ms_ = now_ms + 400;
  next_gpos_ms_ = now_ms + 600;
  next_batt_ms_ = now_ms + 800;
}

bool Mavlink::ShouldSendHbNow(uint32_t now_ms) const {
  // Deadline guarantee: never exceed kHbDeadlineMs gap between completed HBs.
  // Also honor the nominal cadence using next_hb_ms_.
  int32_t since_done = (int32_t)(now_ms - last_hb_done_ms_);
  if (since_done >= (int32_t)kHbDeadlineMs) {
    return true;
  }
  return (int32_t)(now_ms - next_hb_ms_) >= 0;
}

void Mavlink::Init(const Config &cfg, Uart *uart) {
  cfg_ = cfg;
  uart_ = uart;
  initialized_ = true;

  // Clear TX state
  tx_len_ = 0;
  tx_sent_ = 0;
  tx_is_hb_ = false;

  // Heartbeat timing: allow immediate first HB
  last_hb_done_ms_ = 0;
  have_latest_ = false;
  latest_update_ms_ = 0;

  schedule_armed_ = false;
  ArmFirstSchedule(0);

  ESP_LOGI(kTag, "Initialized (RX RC + TX ELRS set)");

  extern void StartMavlinkTxTask();
  StartMavlinkTxTask();
}

void Mavlink::Poll(uint32_t now_ms) {
  if (!initialized_ || !uart_) {
    return;
  }

  uint8_t buf[256];
  int n = uart_->Read(buf, sizeof(buf), 0);
  if (n <= 0) {
    return;
  }

  for (int i = 0; i < n; i++) {
    HandleRxByte(buf[i], now_ms);
  }
}

void Mavlink::HandleRxByte(uint8_t b, uint32_t now_ms) {
  if (mavlink_parse_char(MAVLINK_COMM_1, b, &rx_msg_, &rx_status_)) {
    HandleMessage(rx_msg_, now_ms);
  }
}

void Mavlink::HandleMessage(const mavlink_message_t &msg, uint32_t now_ms) {
  switch (msg.msgid) {
  case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {
    mavlink_rc_channels_override_t rc{};
    mavlink_msg_rc_channels_override_decode(&msg, &rc);

    rc_.channels[0] = rc.chan1_raw;
    rc_.channels[1] = rc.chan2_raw;
    rc_.channels[2] = rc.chan3_raw;
    rc_.channels[3] = rc.chan4_raw;
    rc_.channels[4] = rc.chan5_raw;
    rc_.channels[5] = rc.chan6_raw;
    rc_.channels[6] = rc.chan7_raw;
    rc_.channels[7] = rc.chan8_raw;
    rc_.channels[8] = rc.chan9_raw;
    rc_.channels[9] = rc.chan10_raw;
    rc_.channels[10] = rc.chan11_raw;
    rc_.channels[11] = rc.chan12_raw;
    rc_.channels[12] = rc.chan13_raw;
    rc_.channels[13] = rc.chan14_raw;
    rc_.channels[14] = rc.chan15_raw;
    rc_.channels[15] = rc.chan16_raw;
    rc_.channels[16] = rc.chan17_raw;
    rc_.channels[17] = rc.chan18_raw;

    rc_.count = 18;
    rc_.last_update = now_ms;
    break;
  }

  default:
    break;
  }
}

// ---------------- Telemetry input (from STM32) ----------------

void Mavlink::OfferTelemetry(const message::GpsData &d, uint32_t now_ms) {
  latest_ = d;
  have_latest_ = true;
  latest_update_ms_ = now_ms;
}

// ---------------- TX core ----------------

void Mavlink::ServiceInFlightTx() {
  if (!uart_ || tx_len_ == 0) {
    return;
  }

  uint16_t remaining = (uint16_t)(tx_len_ - tx_sent_);
  if (remaining == 0) {
    return;
  }

  int wrote = uart_->Write(tx_buf_ + tx_sent_, remaining);
  if (wrote > 0) {
    tx_sent_ = (uint16_t)(tx_sent_ + (uint16_t)wrote);
  }
}

void Mavlink::StartHeartbeatFrame() {
  mavlink_message_t m{};

  mavlink_msg_heartbeat_pack(cfg_.sysid, cfg_.compid, &m, MAV_TYPE_GENERIC,
                             MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE);

  tx_len_ = (uint16_t)mavlink_msg_to_send_buffer(tx_buf_, &m);
  tx_sent_ = 0;
  tx_is_hb_ = true;

  next_hb_ms_ += cfg_.hb_period_ms;
}

void Mavlink::StartGpsRawIntFrame() {
  next_gps_ms_ += cfg_.gps_period_ms;

  // Only send if we have telemetry
  if (!have_latest_) {
    return;
  }

  mavlink_message_t m{};

  // MAVLink expects: lat/lon 1e7 deg, alt mm, vel cm/s, cog cdeg
  // fixType maps well to MAVLink fix_type.

  mavlink_msg_gps_raw_int_pack(cfg_.sysid, cfg_.compid, &m,
                               0, // time_usec (unknown here)
                               (uint8_t)latest_.fixType, latest_.lat,
                               latest_.lon,
                               (int32_t)latest_.hMSL,          // alt (mm)
                               (uint16_t)(latest_.hAcc / 10u), // eph (cm) rough
                               (uint16_t)(latest_.vAcc / 10u), // epv (cm) rough
                               (uint16_t)latest_.vel,          // vel (cm/s)
                               (uint16_t)latest_.hdg,          // cog (cdeg)
                               (uint8_t)latest_.numSV,
                               0, // alt_ellipsoid
                               0, // h_acc
                               0, // v_acc
                               0, // vel_acc
                               0, // hdg_acc
                               0  // yaw
  );

  tx_len_ = (uint16_t)mavlink_msg_to_send_buffer(tx_buf_, &m);
  tx_sent_ = 0;
  tx_is_hb_ = false;
}

void Mavlink::StartAttitudeFrame() {
  next_att_ms_ += cfg_.att_period_ms;

  if (!have_latest_) {
    return;
  }

  mavlink_message_t m{};

  // latest_.roll/pitch/yaw are cdeg -> convert to rad
  // rad = (cdeg / 100.0) * pi/180
  float roll = ((float)latest_.roll * 0.01f) * 0.017453292519943295f;
  float pitch = ((float)latest_.pitch * 0.01f) * 0.017453292519943295f;
  float yaw = ((float)latest_.yaw * 0.01f) * 0.017453292519943295f;

  mavlink_msg_attitude_pack(cfg_.sysid, cfg_.compid, &m,
                            0, // time_boot_ms unknown here
                            roll, pitch, yaw, 0.0f, 0.0f, 0.0f);

  tx_len_ = (uint16_t)mavlink_msg_to_send_buffer(tx_buf_, &m);
  tx_sent_ = 0;
  tx_is_hb_ = false;
}

void Mavlink::StartGlobalPositionIntFrame() {
  next_gpos_ms_ += cfg_.gpos_period_ms;

  if (!have_latest_) {
    return;
  }

  mavlink_message_t m{};

  // ELRS uses relative_alt and vz for vario. We don't have relative_alt/vz in
  // GpsData. Use hMSL as a placeholder for alt and set vz=0 unless you later
  // add vertical speed in your STM32 packet.
  int32_t lat = latest_.lat;
  int32_t lon = latest_.lon;
  int32_t alt = latest_.hMSL;     // mm
  int32_t rel_alt = latest_.hMSL; // mm (placeholder for relative)
  int16_t vx = 0;                 // cm/s
  int16_t vy = 0;                 // cm/s
  int16_t vz = 0;                 // cm/s
  uint16_t hdg = latest_.hdg;     // cdeg

  mavlink_msg_global_position_int_pack(cfg_.sysid, cfg_.compid, &m,
                                       0, // time_boot_ms unknown
                                       lat, lon, alt, rel_alt, vx, vy, vz, hdg);

  tx_len_ = (uint16_t)mavlink_msg_to_send_buffer(tx_buf_, &m);
  tx_sent_ = 0;
  tx_is_hb_ = false;
}

void Mavlink::StartBatteryStatusFrame() {
  next_batt_ms_ += cfg_.batt_period_ms;

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
      cfg_.sysid, cfg_.compid, &m,
      0, // id
      MAV_BATTERY_FUNCTION_ALL, MAV_BATTERY_TYPE_LIPO,
      0, // temperature unknown
      voltages,
      (int16_t)latest_.batt_current, // cA in your struct; MAVLink expects cA
      (int32_t)latest_
          .batt_current, // current_consumed: you have cA, not mAh.
                         // You should replace this once STM32 provides mAh.
      0,                 // energy_consumed unknown
      (int8_t)latest_.batt_remaining,
      0,        // time_remaining
      0,        // charge_state
      voltages, // voltages_ext (safe re-use, ignores extra)
      0,        // mode
      0         // fault_bitmask
  );

  tx_len_ = (uint16_t)mavlink_msg_to_send_buffer(tx_buf_, &m);
  tx_sent_ = 0;
  tx_is_hb_ = false;
}

void Mavlink::StartNextScheduledFrame(uint32_t now_ms) {
  // Heartbeat is handled outside as hard priority.
  // Here we select the next due stream among gps/att/gpos/batt.
  // Keep this simple and deterministic.

  // If no telemetry yet, only HB can be sent.
  if (!have_latest_) {
    return;
  }

  // Find earliest due among enabled streams.
  // If multiple are due, pick the one with the oldest deadline (smallest
  // next_*).
  uint32_t best_due = 0xFFFFFFFFu;
  enum class Pick : uint8_t {
    kNone,
    kGps,
    kAtt,
    kGpos,
    kBatt
  } pick = Pick::kNone;

  if (cfg_.gps_period_ms > 0 && (int32_t)(now_ms - next_gps_ms_) >= 0) {
    best_due = next_gps_ms_;
    pick = Pick::kGps;
  }
  if (cfg_.att_period_ms > 0 && (int32_t)(now_ms - next_att_ms_) >= 0) {
    if (next_att_ms_ < best_due) {
      best_due = next_att_ms_;
      pick = Pick::kAtt;
    }
  }
  if (cfg_.gpos_period_ms > 0 && (int32_t)(now_ms - next_gpos_ms_) >= 0) {
    if (next_gpos_ms_ < best_due) {
      best_due = next_gpos_ms_;
      pick = Pick::kGpos;
    }
  }
  if (cfg_.batt_period_ms > 0 && (int32_t)(now_ms - next_batt_ms_) >= 0) {
    if (next_batt_ms_ < best_due) {
      best_due = next_batt_ms_;
      pick = Pick::kBatt;
    }
  }

  switch (pick) {
  case Pick::kGps:
    StartGpsRawIntFrame();
    break;
  case Pick::kAtt:
    StartAttitudeFrame();
    break;
  case Pick::kGpos:
    StartGlobalPositionIntFrame();
    break;
  case Pick::kBatt:
    StartBatteryStatusFrame();
    break;
  case Pick::kNone:
  default:
    break;
  }
}

void Mavlink::TxTick(uint32_t now_ms) {
  if (!initialized_ || !uart_) {
    return;
  }

  // Arm schedule on first real tick
  if (!schedule_armed_) {
    ArmFirstSchedule(now_ms);
    // Force immediate heartbeat
    last_hb_done_ms_ = now_ms - kHbDeadlineMs;
    schedule_armed_ = true;
  }

  // 1) Finish writing any in-flight frame (atomic frame output)
  if (tx_len_ > 0 && tx_sent_ < tx_len_) {
    ServiceInFlightTx();
  }

  // 2) If we finished pushing bytes to driver, wait for TX done then clear
  // state
  if (tx_len_ > 0 && tx_sent_ >= tx_len_) {
    if (uart_->DrainTx(0)) {
      if (tx_is_hb_) {
        last_hb_done_ms_ = now_ms;
        tx_is_hb_ = false;
      }
      tx_len_ = 0;
      tx_sent_ = 0;
    }
    return;
  }

  // 3) Nothing in flight: heartbeat has hard priority
  if (ShouldSendHbNow(now_ms)) {
    StartHeartbeatFrame();
    ServiceInFlightTx();
    return;
  }

  // 4) Send one scheduled stream frame if due
  StartNextScheduledFrame(now_ms);
  if (tx_len_ > 0) {
    ServiceInFlightTx();
  }
}
