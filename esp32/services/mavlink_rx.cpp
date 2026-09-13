// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include <array>
#include <cstdio>

#include "esp_log.h"
#include "mavlink.hpp"
#include "system.hpp"

static constexpr const char *kTag = "mavlink";

void Mavlink::ServiceRx() {
  // Drain whatever the transport has buffered this tick. Bounded so a flooding
  // host cannot starve the rest of the state-machine loop.
  static constexpr int kMaxDrainIterations = 16;
  std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> rx_buf{};
  mavlink_message_t msg{};

  for (int iter = 0; iter < kMaxDrainIterations; ++iter) {
    const int received = transport_->Receive(rx_buf);
    if (received <= 0) {
      return;
    }
    for (int i = 0; i < received; ++i) {
      if (mavlink_parse_char(MAVLINK_COMM_2, rx_buf[i], &msg, nullptr)) {
        if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
          rx_heartbeat_count_.fetch_add(1, std::memory_order_relaxed);
        }
        rx_packet_count_.fetch_add(1, std::memory_order_relaxed);
        HandleMessage(msg);
      }
    }
  }
}

// Nothing here transmits: handlers mutate local state or queue a reply, and
// the TX scheduler sends it later.
void Mavlink::HandleMessage(const mavlink_message_t &msg) {
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
  char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1] = {};
  std::snprintf(text, sizeof(text), "Unhandled MAVLink msgid=%lu src=UDP",
                (unsigned long)msg.msgid);
  NotifyGcsIssueOnce(text, MAV_SEVERITY_WARNING);
}

void Mavlink::HandleParamMessage(const mavlink_message_t &msg) {
  char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1] = {};
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
      mavlink_param_request_list_t req{};
      mavlink_msg_param_request_list_decode(&msg, &req);
      if (IsTargetedToThisComponent(req.target_system, req.target_component)) {
        params_.StartStream();
      }
      break;
    }
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
      mavlink_param_request_read_t req{};
      mavlink_msg_param_request_read_decode(&msg, &req);
      if (!IsTargetedToThisComponent(req.target_system, req.target_component) ||
          params_.QueueRead(req.param_index, req.param_id)) {
        break;
      }
      if (req.param_index >= 0) {
        ESP_LOGW(kTag, "PARAM_REQUEST_READ unresolved index=%d",
                 static_cast<int>(req.param_index));
        std::snprintf(text, sizeof(text), "Unhandled PARAM_READ index=%d",
                      static_cast<int>(req.param_index));
      } else {
        ESP_LOGW(kTag, "PARAM_REQUEST_READ unresolved param_id=%.16s",
                 req.param_id);
        std::snprintf(text, sizeof(text), "Unhandled PARAM_READ %.16s",
                      req.param_id);
      }
      NotifyGcsIssueOnce(text, MAV_SEVERITY_WARNING);
      break;
    }
    case MAVLINK_MSG_ID_PARAM_SET: {
      mavlink_param_set_t req{};
      mavlink_msg_param_set_decode(&msg, &req);
      if (!IsTargetedToThisComponent(req.target_system, req.target_component)) {
        break;
      }
      const MavlinkParamServer::SetResult result =
          params_.Set(req.param_id, req.param_value, req.param_type);
      if (result == MavlinkParamServer::SetResult::kUnknownParam) {
        ESP_LOGW(kTag, "PARAM_SET unresolved param_id=%.16s", req.param_id);
        std::snprintf(text, sizeof(text), "Unhandled PARAM_SET %.16s",
                      req.param_id);
        NotifyGcsIssueOnce(text, MAV_SEVERITY_WARNING);
      } else if (result != MavlinkParamServer::SetResult::kAccepted) {
        ESP_LOGW(kTag, "PARAM_SET rejected param_id=%.16s reason=%s",
                 req.param_id, MavlinkParamServer::SetResultName(result));
        std::snprintf(text, sizeof(text), "Rejected PARAM_SET %.16s",
                      req.param_id);
        NotifyGcsIssueOnce(text, MAV_SEVERITY_WARNING);
      }
      break;
    }
    case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST: {
      mavlink_param_ext_request_list_t req{};
      mavlink_msg_param_ext_request_list_decode(&msg, &req);
      if (!IsTargetedToThisComponent(req.target_system, req.target_component)) {
        break;
      }
      ESP_LOGW(kTag, "PARAM_EXT_REQUEST_LIST unsupported");
      NotifyGcsIssueOnce("Unsupported PARAM_EXT_LIST", MAV_SEVERITY_WARNING);
      break;
    }
    case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ: {
      mavlink_param_ext_request_read_t req{};
      mavlink_msg_param_ext_request_read_decode(&msg, &req);
      if (!IsTargetedToThisComponent(req.target_system, req.target_component)) {
        break;
      }
      ESP_LOGW(kTag, "PARAM_EXT_REQUEST_READ unsupported");
      NotifyGcsIssueOnce("Unsupported PARAM_EXT_READ", MAV_SEVERITY_WARNING);
      break;
    }
    case MAVLINK_MSG_ID_PARAM_EXT_SET: {
      mavlink_param_ext_set_t req{};
      mavlink_msg_param_ext_set_decode(&msg, &req);
      if (!IsTargetedToThisComponent(req.target_system, req.target_component)) {
        break;
      }
      ESP_LOGW(kTag,
               "PARAM_EXT_SET unsupported target_sys=%u target_comp=%u "
               "param_id=%.16s value=%.128s type=%u",
               static_cast<unsigned>(req.target_system),
               static_cast<unsigned>(req.target_component), req.param_id,
               req.param_value, static_cast<unsigned>(req.param_type));
      std::snprintf(text, sizeof(text), "Unsupported PARAM_EXT_SET %.16s",
                    req.param_id);
      NotifyGcsIssueOnce(text, MAV_SEVERITY_WARNING);
      break;
    }
    default: {
      std::snprintf(text, sizeof(text), "Unhandled param msgid=%lu",
                    (unsigned long)msg.msgid);
      NotifyGcsIssueOnce(text, MAV_SEVERITY_WARNING);
      break;
    }
  }
}
