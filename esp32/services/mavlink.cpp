#include "mavlink.hpp"
extern "C" {
#include "esp_log.h"
}

static const char *kTag = "mavlink";

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
}
