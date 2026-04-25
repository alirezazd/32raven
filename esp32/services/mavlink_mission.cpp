#include "mavlink.hpp"

void Mavlink::HandleMissionMessage(const mavlink_message_t &msg) {
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
      mavlink_mission_request_list_t req{};
      mavlink_msg_mission_request_list_decode(&msg, &req);
      // Mission inventory requests are queued so RX only records the request
      // metadata if it is actually addressed to this endpoint.
      if (req.target_system == cfg_.identity.sysid &&
          (req.target_component == cfg_.identity.compid ||
           req.target_component == 0 ||
           req.target_component == MAV_COMP_ID_ALL)) {
        QueueMissionCount(msg.sysid, msg.compid, req.mission_type);
      }
      break;
    }
    case MAVLINK_MSG_ID_MISSION_ACK:
      break;
    default:
      break;
  }
}
