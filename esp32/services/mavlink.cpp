#include "mavlink.hpp"
extern "C" {
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}
#include <ctime>

static constexpr const char *kTag = "mavlink";

void Mavlink::Init(const Config &cfg, Uart *uart) {
  if (initialized_)
    return;
  cfg_ = cfg;
  uart_ = uart;
  initialized_ = true;
  ESP_LOGI(kTag, "Initialized (RC Receiver)");
}

void Mavlink::Poll(uint32_t now) {
  (void)now;
  if (!initialized_ || !uart_)
    return;

  uint8_t buf[128];
  int n = uart_->Read(buf, sizeof(buf));
  if (n > 0) {
    // ESP_LOGI(kTag, "EP2 Rx: %d bytes", n);
  }

  for (int i = 0; i < n; ++i) {
    if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &rx_msg_, &rx_status_)) {
      // ESP_LOGI(kTag, "Parsed Msg %d", rx_msg_.msgid);
      // Packet Received
      if (handler_) {
        handler_(rx_msg_);
      }

      switch (rx_msg_.msgid) {
      case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE: {
        mavlink_rc_channels_override_t rc;
        mavlink_msg_rc_channels_override_decode(&rx_msg_, &rc);

        static uint32_t last_log = 0;
        if (now - last_log > 1000) {
          // ESP_LOGI(kTag, "RC Ovr: Ch1=%d Ch2=%d", rc.chan1_raw,
          // rc.chan2_raw);
          last_log = now;
        }

        // Store
        rc_.channels[0] = rc.chan1_raw; // Roll
        rc_.channels[1] = rc.chan2_raw; // Pitch
        rc_.channels[2] = rc.chan3_raw; // Throttle
        rc_.channels[3] = rc.chan4_raw; // Yaw
        rc_.channels[4] = rc.chan5_raw; // Arm/Disarm
        rc_.channels[5] = rc.chan6_raw; // Arm/Disarm/Buzzer
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
        rc_.last_update = now;
        break;
      }
      default:
        // Ignore others
        break;
      }
    }
  }

  // Heartbeat (1Hz)
  if (now - last_heartbeat_ >= 1000) {
    SendHeartbeat();
    last_heartbeat_ = now;
  }
}

void Mavlink::SendHeartbeat() {
  mavlink_message_t msg;
  mavlink_heartbeat_t hb = {};
  hb.type = MAV_TYPE_QUADROTOR;
  hb.autopilot = MAV_AUTOPILOT_GENERIC;
  hb.base_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
  hb.system_status = MAV_STATE_ACTIVE;

  mavlink_msg_heartbeat_encode(1, 1, &msg, &hb);

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  if (uart_) {
    uart_->Write(buf, len);
  }
}

void Mavlink::SendSystemTime(const message::GpsData &t) {
  uint64_t unix_time_us = 0;

  if (t.year > 0) {
    struct tm timeinfo = {};
    timeinfo.tm_year = t.year - 1900;
    timeinfo.tm_mon = t.month - 1;
    timeinfo.tm_mday = t.day;
    timeinfo.tm_hour = t.hour;
    timeinfo.tm_min = t.min;
    timeinfo.tm_sec = t.sec;
    timeinfo.tm_isdst = 0;

    time_t uts = mktime(&timeinfo);
    if (uts != -1) {
      unix_time_us = (uint64_t)uts * 1000000ULL;
    }
  }

  mavlink_message_t msg;
  mavlink_system_time_t st = {};
  st.time_unix_usec = unix_time_us;
  st.time_boot_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

  mavlink_msg_system_time_encode(1, 1, &msg, &st);

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  if (uart_) {
    uart_->Write(buf, len);
  }
}

void Mavlink::SendGpsRawInt(const message::GpsData &t) {
  mavlink_message_t msg;
  mavlink_gps_raw_int_t gps = {};

  // Time since boot in us (not absolute time)
  gps.time_usec = (uint64_t)xTaskGetTickCount() * portTICK_PERIOD_MS * 1000;
  gps.fix_type = t.fixType; // 0-1: no fix, 2: 2D, 3: 3D
  gps.lat = t.lat;          // deg * 1E7
  gps.lon = t.lon;          // deg * 1E7
  gps.alt = t.height;       // mm (MSL)
  gps.eph = UINT16_MAX;     // Unknown
  gps.epv = UINT16_MAX;     // Unknown
  gps.vel = UINT16_MAX;     // Unknown
  gps.cog = UINT16_MAX;     // Unknown
  gps.satellites_visible = t.numSV;

  mavlink_msg_gps_raw_int_encode(1, 1, &msg, &gps);

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  if (uart_) {
    uart_->Write(buf, len);
  }
}

void Mavlink::SendGlobalPositionInt(const message::GpsData &t) {
  mavlink_message_t msg;
  mavlink_global_position_int_t pos = {};

  pos.time_boot_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
  pos.lat = t.lat;    // deg * 1E7
  pos.lon = t.lon;    // deg * 1E7
  pos.alt = t.height; // mm (MSL)
  pos.relative_alt =
      t.height;         // mm (Height above home - simplified as MSL for now)
  pos.vx = 0;           // Unknown
  pos.vy = 0;           // Unknown
  pos.vz = 0;           // Unknown
  pos.hdg = UINT16_MAX; // Unknown

  mavlink_msg_global_position_int_encode(1, 1, &msg, &pos);

  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

  if (uart_) {
    uart_->Write(buf, len);
  }
}
