#include <cstdio>

#include "mavlink.hpp"
#include "system.hpp"

void Mavlink::LogUnhandledCommandOnce(uint16_t command, const char *reason) {
  for (uint8_t i = 0; i < unhandled_logged_command_count_; ++i) {
    if (unhandled_logged_commands_[i] == command) {
      return;
    }
  }

  if (unhandled_logged_command_count_ < unhandled_logged_commands_.size()) {
    unhandled_logged_commands_[unhandled_logged_command_count_++] = command;
  }

  char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1] = {};
  if (reason != nullptr && reason[0] != '\0') {
    std::snprintf(text, sizeof(text), "Unhandled MAV_CMD=%u %s",
                  static_cast<unsigned>(command), reason);
  } else {
    std::snprintf(text, sizeof(text), "Unhandled MAV_CMD=%u",
                  static_cast<unsigned>(command));
  }
  NotifyGcsIssue(text, MAV_SEVERITY_WARNING);
}

void Mavlink::HandleCommandMessage(const mavlink_message_t &msg) {
  mavlink_command_long_t cmd{};
  mavlink_msg_command_long_decode(&msg, &cmd);
  if (!IsTargetedToThisComponent(cmd.target_system, cmd.target_component)) {
    return;
  }

  HandleCommandLong(msg, cmd);
}

void Mavlink::HandleCommandLong(const mavlink_message_t &msg,
                                const mavlink_command_long_t &cmd) {
  const uint8_t source_system = msg.sysid;
  const uint8_t source_component = msg.compid;

  switch (cmd.command) {
    case MAV_CMD_START_RX_PAIR: {
      if (static_cast<uint32_t>(cmd.param1) == RC_TYPE_CRSF) {
        message::Packet req_pkt{};
        req_pkt.header.id =
            static_cast<uint8_t>(message::MsgId::kReqReceiverBind);
        req_pkt.header.len = 0;
        Sys().FcLink().SendPacket(req_pkt);
        QueueCommandAck(static_cast<uint16_t>(cmd.command), MAV_RESULT_ACCEPTED,
                        source_system, source_component);
      } else {
        QueueCommandAck(static_cast<uint16_t>(cmd.command),
                        MAV_RESULT_UNSUPPORTED, source_system,
                        source_component);
        LogUnhandledCommandOnce(static_cast<uint16_t>(cmd.command),
                                "rx-type");
      }
      break;
    }
    case MAV_CMD_REQUEST_MESSAGE:
      if (static_cast<uint32_t>(cmd.param1) ==
          MAVLINK_MSG_ID_AUTOPILOT_VERSION) {
        QueueCommandAck(static_cast<uint16_t>(cmd.command), MAV_RESULT_ACCEPTED,
                        source_system, source_component);
        QueueAutopilotVersion();
      } else {
        QueueCommandAck(static_cast<uint16_t>(cmd.command),
                        MAV_RESULT_UNSUPPORTED, source_system,
                        source_component);
        LogUnhandledCommandOnce(static_cast<uint16_t>(cmd.command),
                                "request-msg");
      }
      break;
    case MAV_CMD_PREFLIGHT_CALIBRATION:
      QueueCommandAck(static_cast<uint16_t>(cmd.command), MAV_RESULT_ACCEPTED,
                      source_system, source_component);
      break;
    default:
      QueueCommandAck(static_cast<uint16_t>(cmd.command),
                      MAV_RESULT_UNSUPPORTED, source_system, source_component);
      LogUnhandledCommandOnce(static_cast<uint16_t>(cmd.command),
                              "unsupported");
      break;
  }
}
