#include <array>
#include <cstdio>

#include "mavlink.hpp"
#include "system.hpp"

void Mavlink::ServiceUdpRx() {
  // Parse at most one full MAVLink packet per poll so RX work stays bounded and
  // cannot monopolize the state-machine task under sustained traffic.
  std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> rx_buf{};
  const int received = udp_->Receive(rx_buf.data(), rx_buf.size());
  if (received <= 0) {
    return;
  }

  mavlink_message_t msg{};
  for (int i = 0; i < received; ++i) {
    // Parse byte stream into complete MAVLink messages and dispatch each one.
    if (mavlink_parse_char(MAVLINK_COMM_2, rx_buf[i], &msg, nullptr)) {
      udp_rx_packet_count_.fetch_add(1, std::memory_order_relaxed);
      HandleMessage(msg);
    }
  }
}

// Incoming MAVLink dispatch: decode requests, mutate local state, and queue
// deferred responses for the TX scheduler.
void Mavlink::HandleMessage(const mavlink_message_t &msg) {
  // Incoming MAVLink only mutates local state or queues replies. Actual
  // transmission is deferred to the worker-side TX scheduler.
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
      Sys().Led().SetPattern(LED::Pattern::kDoubleBlink, 300, 1);
      break;
    }
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
    case MAVLINK_MSG_ID_PARAM_SET:
    case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST:
    case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ:
    case MAVLINK_MSG_ID_PARAM_EXT_SET: {
      HandleParamMessage(msg);
      break;
    }
    case MAVLINK_MSG_ID_MISSION_ACK:
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
      HandleMissionMessage(msg);
      break;
    }
    case MAVLINK_MSG_ID_SYSTEM_TIME:
      break;
    case MAVLINK_MSG_ID_COMMAND_LONG: {
      HandleCommandMessage(msg);
      break;
    }
    default:
      LogUnhandledMessageOnce(msg);
      break;
  }
}

void Mavlink::LogUnhandledMessageOnce(const mavlink_message_t &msg) {
  // Keep the warning path one-shot per msgid so noisy GCS traffic does not
  // flood the UI status queue.
  for (uint8_t i = 0; i < unhandled_logged_msgid_count_; ++i) {
    if (unhandled_logged_msgids_[i] == msg.msgid) {
      return;
    }
  }

  if (unhandled_logged_msgid_count_ < unhandled_logged_msgids_.size()) {
    unhandled_logged_msgids_[unhandled_logged_msgid_count_++] =
        static_cast<uint16_t>(msg.msgid);
  }

  char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1] = {};
  std::snprintf(text, sizeof(text), "Unhandled MAVLink msgid=%lu src=UDP",
                (unsigned long)msg.msgid);
  QueueStatusText(text, MAV_SEVERITY_WARNING);
}
