// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include <cmath>
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

// A message this firmware has decided not to produce, as opposed to one
// nobody has looked at yet. Both are answered UNSUPPORTED -- the ack is the
// protocol's whole answer -- but only the second is worth telling the operator
// about, because only the second names something still to be built.
bool Mavlink::IsDeclinedMessage(uint32_t message_id) {
  switch (message_id) {
    // Metadata describing this vehicle: served by the ground station's own
    // build instead, so both the current message and its deprecated
    // predecessor are refused on purpose.
    case MAVLINK_MSG_ID_COMPONENT_METADATA:
    case MAVLINK_MSG_ID_COMPONENT_INFORMATION:
    // No gimbal is fitted. The capability bit for one is left clear, and this
    // is the answer for a ground station that asks regardless.
    case MAVLINK_MSG_ID_GIMBAL_MANAGER_INFORMATION:
      return true;
    default:
      return false;
  }
}

void Mavlink::HandleRequestMessage(const mavlink_command_long_t &cmd,
                                   uint8_t source_system,
                                   uint8_t source_component) {
  const auto command = static_cast<uint16_t>(cmd.command);
  const auto message_id = static_cast<uint32_t>(cmd.param1);

  if (message_id == MAVLINK_MSG_ID_AUTOPILOT_VERSION) {
    QueueCommandAck(command, MAV_RESULT_ACCEPTED, source_system,
                    source_component);
    QueueAutopilotVersion();
    return;
  }

  if (message_id == MAVLINK_MSG_ID_AVAILABLE_MODES) {
    // param2 is the 1-based index, and zero is the ground station asking for
    // the whole list in one go -- which this message cannot carry, so it is
    // answered from the top and walked by number_modes like any other.
    const long requested = std::lround(cmd.param2);
    const long count = static_cast<long>(kFlightModes.size());
    if (requested > count || requested < 0) {
      QueueCommandAck(command, MAV_RESULT_DENIED, source_system,
                      source_component);
      return;
    }
    QueueCommandAck(command, MAV_RESULT_ACCEPTED, source_system,
                    source_component);
    QueueAvailableModes(static_cast<uint8_t>(requested < 1 ? 1 : requested));
    return;
  }

  QueueCommandAck(command, MAV_RESULT_UNSUPPORTED, source_system,
                  source_component);
  if (!IsDeclinedMessage(message_id)) {
    LogUnhandledCommandOnce(command, message_id, "request-msg");
  }
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
      HandleRequestMessage(cmd, source_system, source_component);
      break;
    case MAV_CMD_PREFLIGHT_CALIBRATION: {
      // param1 is the gyro slot and param5 the accel, as the MAVLink command
      // defines them, and only the value 1 asks for a run. param5 = 2 is the
      // level-horizon trim, a different routine this board does not have, so
      // treating every non-zero as "accel" answered one button with another.
      // ACCEPTED means the request reached the flight controller, not that the
      // run finished -- the gyro ends in a tone, and the accel in a STATUSTEXT
      // per pose.
      const uint32_t gyro_slot = static_cast<uint32_t>(cmd.param1);
      const uint32_t accel_slot = static_cast<uint32_t>(cmd.param5);
      // Every field zero is the cancel, as PX4 reads it and QGC sends it.
      const bool cancel = gyro_slot == 0u && accel_slot == 0u &&
                          static_cast<uint32_t>(cmd.param2) == 0u &&
                          static_cast<uint32_t>(cmd.param3) == 0u &&
                          static_cast<uint32_t>(cmd.param4) == 0u &&
                          static_cast<uint32_t>(cmd.param6) == 0u &&
                          static_cast<uint32_t>(cmd.param7) == 0u;
      message::MsgId id = message::MsgId::kCalibrateGyro;
      if (cancel) {
        id = message::MsgId::kCancelCalibration;
      } else if (gyro_slot != 1u) {
        if (accel_slot != 1u) {
          QueueCommandAck(static_cast<uint16_t>(cmd.command),
                          MAV_RESULT_UNSUPPORTED, source_system,
                          source_component);
          LogUnhandledCommandOnce(static_cast<uint16_t>(cmd.command),
                                  accel_slot != 0u ? accel_slot : gyro_slot,
                                  "cal-slot");
          break;
        }
        id = message::MsgId::kCalibrateAccel;
      }
      message::Packet req_pkt{};
      req_pkt.header.id = static_cast<uint8_t>(id);
      fc_link_->SendPacket(req_pkt);
      QueueCommandAck(static_cast<uint16_t>(cmd.command), MAV_RESULT_ACCEPTED,
                      source_system, source_component);
      break;
    }
    default:
      QueueCommandAck(static_cast<uint16_t>(cmd.command),
                      MAV_RESULT_UNSUPPORTED, source_system, source_component);
      LogUnhandledCommandOnce(static_cast<uint16_t>(cmd.command),
                              static_cast<uint32_t>(cmd.param1), "unsupported");
      break;
  }
}
