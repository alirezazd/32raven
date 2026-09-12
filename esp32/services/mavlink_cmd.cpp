// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include <cstdio>

#include "mavlink.hpp"

// `detail` is the command's param1, which for MAV_CMD_REQUEST_MESSAGE is the
// message being asked for -- the whole content of the request, and the only
// thing that says which gap this is. It reaches the text, and through the text
// the one-shot key, so two messages refused under the same command number are
// two reports rather than one.
void Mavlink::LogUnhandledCommandOnce(uint16_t command, uint32_t detail,
                                      const char *reason) {
  char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1] = {};
  if (reason != nullptr && reason[0] != '\0') {
    std::snprintf(text, sizeof(text), "Unhandled MAV_CMD=%u %s param1=%lu",
                  static_cast<unsigned>(command), reason,
                  (unsigned long)detail);
  } else {
    std::snprintf(text, sizeof(text), "Unhandled MAV_CMD=%u param1=%lu",
                  static_cast<unsigned>(command), (unsigned long)detail);
  }
  NotifyGcsIssueOnce(text, MAV_SEVERITY_WARNING);
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
        // MAVLink defines no cancel for this command, and no RC sub-type for
        // CRSF -- so param2 is the one slot a later spec revision cannot
        // collide with. 0 keeps the plain "start pairing" meaning.
        constexpr uint32_t kSubTypeCancelBind = 1u;
        const bool cancel =
            static_cast<uint32_t>(cmd.param2) == kSubTypeCancelBind;
        message::Packet req_pkt{};
        req_pkt.header.id =
            static_cast<uint8_t>(cancel ? message::MsgId::kReqReceiverCancelBind
                                        : message::MsgId::kReqReceiverBind);
        req_pkt.header.len = 0;
        fc_link_->SendPacket(req_pkt);
        QueueCommandAck(static_cast<uint16_t>(cmd.command), MAV_RESULT_ACCEPTED,
                        source_system, source_component);
      } else {
        QueueCommandAck(static_cast<uint16_t>(cmd.command),
                        MAV_RESULT_UNSUPPORTED, source_system,
                        source_component);
        LogUnhandledCommandOnce(static_cast<uint16_t>(cmd.command),
                                static_cast<uint32_t>(cmd.param1), "rx-type");
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
                                static_cast<uint32_t>(cmd.param1),
                                "request-msg");
      }
      break;
    case MAV_CMD_PREFLIGHT_CALIBRATION:
      // param1 is the gyro slot and param5 the accel, as the MAVLink command
      // defines them. ACCEPTED means the request reached the flight controller,
      // not that the run finished -- the gyro ends in a tone, and the accel in
      // a STATUSTEXT per pose.
      if (static_cast<uint32_t>(cmd.param1) != 0u ||
          static_cast<uint32_t>(cmd.param5) != 0u) {
        message::Packet req_pkt{};
        req_pkt.header.id =
            static_cast<uint8_t>(static_cast<uint32_t>(cmd.param5) != 0u
                                     ? message::MsgId::kCalibrateAccel
                                     : message::MsgId::kCalibrateGyro);
        req_pkt.header.len = 0;
        fc_link_->SendPacket(req_pkt);
        QueueCommandAck(static_cast<uint16_t>(cmd.command), MAV_RESULT_ACCEPTED,
                        source_system, source_component);
      } else {
        QueueCommandAck(static_cast<uint16_t>(cmd.command),
                        MAV_RESULT_UNSUPPORTED, source_system,
                        source_component);
        LogUnhandledCommandOnce(static_cast<uint16_t>(cmd.command),
                                static_cast<uint32_t>(cmd.param1),
                                "cal-slot");
      }
      break;
    default:
      QueueCommandAck(static_cast<uint16_t>(cmd.command),
                      MAV_RESULT_UNSUPPORTED, source_system, source_component);
      LogUnhandledCommandOnce(static_cast<uint16_t>(cmd.command),
                              static_cast<uint32_t>(cmd.param1),
                              "unsupported");
      break;
  }
}
