#include "mavlink.hpp"

#include "system.hpp"

void Mavlink::HandleCommandMessage(const mavlink_message_t &msg) {
  mavlink_command_long_t cmd{};
  mavlink_msg_command_long_decode(&msg, &cmd);
  if (cmd.target_system != cfg_.identity.sysid ||
      (cmd.target_component != cfg_.identity.compid &&
       cmd.target_component != 0 &&
       cmd.target_component != MAV_COMP_ID_ALL)) {
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
        QueueCommandAck(static_cast<uint16_t>(cmd.command),
                        MAV_RESULT_ACCEPTED, source_system, source_component);
      } else {
        QueueCommandAck(static_cast<uint16_t>(cmd.command),
                        MAV_RESULT_UNSUPPORTED, source_system,
                        source_component);
      }
      break;
    }
    case MAV_CMD_REQUEST_MESSAGE:
      if (static_cast<uint32_t>(cmd.param1) == MAVLINK_MSG_ID_AUTOPILOT_VERSION) {
        QueueCommandAck(static_cast<uint16_t>(cmd.command),
                        MAV_RESULT_ACCEPTED, source_system, source_component);
        QueueAutopilotVersion();
      } else {
        QueueCommandAck(static_cast<uint16_t>(cmd.command),
                        MAV_RESULT_UNSUPPORTED, source_system,
                        source_component);
      }
      break;
    case MAV_CMD_PREFLIGHT_CALIBRATION:
      QueueCommandAck(static_cast<uint16_t>(cmd.command), MAV_RESULT_ACCEPTED,
                      source_system, source_component);
      break;
    default:
      QueueCommandAck(static_cast<uint16_t>(cmd.command),
                      MAV_RESULT_UNSUPPORTED, source_system, source_component);
      break;
  }
}
