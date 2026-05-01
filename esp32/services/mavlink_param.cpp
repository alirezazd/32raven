#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "esp_log.h"
#include "mavlink.hpp"
#include "panic.hpp"
#include "system.hpp"

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
}

namespace {

constexpr const char *kTag = "mavlink";
constexpr uint16_t kFcConfigRequestAttempts = 100;
constexpr uint16_t kFcConfigRequestRetryPeriodMs = 50;
constexpr std::size_t kMavParamIdCStringLen =
    MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1u;

namespace param_detail {

template <size_t kSrcLen>
inline std::array<char, kSrcLen + 1> MavTextToCString(
    const char (&src)[kSrcLen]) {
  std::array<char, kSrcLen + 1> dst{};
  std::memcpy(dst.data(), src, kSrcLen);
  dst[kSrcLen] = '\0';
  return dst;
}

enum class ParamKey : uint8_t {
  kSysId,
  kMavSysId,
  kCompId,
  kCalAcc0Id,
  kCalGyro0Id,
  kCalMag0Id,
  kCalMag1Id,
  kCalMag2Id,
  kCalMag0Rot,
  kCalMag1Rot,
  kCalMag2Rot,
  kSensBoardRot,
  kSensDpresOff,
  kSysHasMag,
  kSysHasNumAspd,
  kSysAutostart,
  kComRcInMode,
  kRcChanCnt,
  kRcMapRoll,
  kRcMapPitch,
  kRcMapYaw,
  kRcMapThrottle,
  kRcMapFlaps,
  kRcMapAux1,
  kRcMapAux2,
  kRcMapParam1,
  kRcMapParam2,
  kRcMapParam3,
  kRcMapPaySw,
  kHeartbeatMs,
  kGpsMs,
  kAttMs,
  kGposMs,
  kBattMs,
  kRcMs,
  kComRcLossT,
};

struct ParamDef {
  const char *id;
  uint8_t type;
  ParamKey key;
};

inline constexpr ParamDef kParamTable[] = {
    {"SYSID_THISMAV", MAV_PARAM_TYPE_UINT8, ParamKey::kSysId},
    {"MAV_SYS_ID", MAV_PARAM_TYPE_UINT8, ParamKey::kMavSysId},
    {"SYS_COMP_ID", MAV_PARAM_TYPE_UINT8, ParamKey::kCompId},
    {"CAL_ACC0_ID", MAV_PARAM_TYPE_INT32, ParamKey::kCalAcc0Id},
    {"CAL_GYRO0_ID", MAV_PARAM_TYPE_INT32, ParamKey::kCalGyro0Id},
    {"CAL_MAG0_ID", MAV_PARAM_TYPE_INT32, ParamKey::kCalMag0Id},
    {"CAL_MAG1_ID", MAV_PARAM_TYPE_INT32, ParamKey::kCalMag1Id},
    {"CAL_MAG2_ID", MAV_PARAM_TYPE_INT32, ParamKey::kCalMag2Id},
    {"CAL_MAG0_ROT", MAV_PARAM_TYPE_INT32, ParamKey::kCalMag0Rot},
    {"CAL_MAG1_ROT", MAV_PARAM_TYPE_INT32, ParamKey::kCalMag1Rot},
    {"CAL_MAG2_ROT", MAV_PARAM_TYPE_INT32, ParamKey::kCalMag2Rot},
    {"SENS_BOARD_ROT", MAV_PARAM_TYPE_INT32, ParamKey::kSensBoardRot},
    {"SENS_DPRES_OFF", MAV_PARAM_TYPE_REAL32, ParamKey::kSensDpresOff},
    {"SYS_HAS_MAG", MAV_PARAM_TYPE_INT32, ParamKey::kSysHasMag},
    {"SYS_HAS_NUM_ASPD", MAV_PARAM_TYPE_INT32, ParamKey::kSysHasNumAspd},
    {"SYS_AUTOSTART", MAV_PARAM_TYPE_INT32, ParamKey::kSysAutostart},
    {"COM_RC_IN_MODE", MAV_PARAM_TYPE_UINT8, ParamKey::kComRcInMode},
    {"RC_CHAN_CNT", MAV_PARAM_TYPE_UINT8, ParamKey::kRcChanCnt},
    {"RC_MAP_ROLL", MAV_PARAM_TYPE_UINT8, ParamKey::kRcMapRoll},
    {"RC_MAP_PITCH", MAV_PARAM_TYPE_UINT8, ParamKey::kRcMapPitch},
    {"RC_MAP_YAW", MAV_PARAM_TYPE_UINT8, ParamKey::kRcMapYaw},
    {"RC_MAP_THROTTLE", MAV_PARAM_TYPE_UINT8, ParamKey::kRcMapThrottle},
    {"RC_MAP_FLAPS", MAV_PARAM_TYPE_INT32, ParamKey::kRcMapFlaps},
    {"RC_MAP_AUX1", MAV_PARAM_TYPE_INT32, ParamKey::kRcMapAux1},
    {"RC_MAP_AUX2", MAV_PARAM_TYPE_INT32, ParamKey::kRcMapAux2},
    {"RC_MAP_PARAM1", MAV_PARAM_TYPE_INT32, ParamKey::kRcMapParam1},
    {"RC_MAP_PARAM2", MAV_PARAM_TYPE_INT32, ParamKey::kRcMapParam2},
    {"RC_MAP_PARAM3", MAV_PARAM_TYPE_INT32, ParamKey::kRcMapParam3},
    {"RC_MAP_PAY_SW", MAV_PARAM_TYPE_INT32, ParamKey::kRcMapPaySw},
    {"MAV_HB_MS", MAV_PARAM_TYPE_UINT16, ParamKey::kHeartbeatMs},
    {"MAV_GPS_MS", MAV_PARAM_TYPE_UINT16, ParamKey::kGpsMs},
    {"MAV_ATT_MS", MAV_PARAM_TYPE_UINT16, ParamKey::kAttMs},
    {"MAV_GPOS_MS", MAV_PARAM_TYPE_UINT16, ParamKey::kGposMs},
    {"MAV_BATT_MS", MAV_PARAM_TYPE_UINT16, ParamKey::kBattMs},
    {"MAV_RC_MS", MAV_PARAM_TYPE_UINT16, ParamKey::kRcMs},
    {"COM_RC_LOSS_T", MAV_PARAM_TYPE_REAL32, ParamKey::kComRcLossT},
};

inline constexpr uint8_t kRcCalibrationParamCountPerChannel = 4u;
inline constexpr uint8_t kTotalRcCalibrationParamCount =
    message::kRcCalibrationChannelCount * kRcCalibrationParamCountPerChannel;
inline constexpr uint16_t kBaseParamCount =
    static_cast<uint16_t>(sizeof(kParamTable) / sizeof(kParamTable[0]));
inline constexpr uint16_t kTotalParamCount =
    kBaseParamCount + kTotalRcCalibrationParamCount;

float EncodeParamValue(float value, uint8_t param_type) {
  mavlink_param_union_t param{};
  param.type = static_cast<mavlink_message_type_t>(param_type);

  switch (param_type) {
    case MAV_PARAM_TYPE_UINT8:
      param.param_uint8 = static_cast<uint8_t>(std::lround(value));
      break;
    case MAV_PARAM_TYPE_INT8:
      param.param_int8 = static_cast<int8_t>(std::lround(value));
      break;
    case MAV_PARAM_TYPE_UINT16:
      param.param_uint16 = static_cast<uint16_t>(std::lround(value));
      break;
    case MAV_PARAM_TYPE_INT16:
      param.param_int16 = static_cast<int16_t>(std::lround(value));
      break;
    case MAV_PARAM_TYPE_UINT32:
      param.param_uint32 = static_cast<uint32_t>(std::llround(value));
      break;
    case MAV_PARAM_TYPE_INT32:
      param.param_int32 = static_cast<int32_t>(std::llround(value));
      break;
    case MAV_PARAM_TYPE_REAL32:
    default:
      param.param_float = value;
      break;
  }

  return param.param_float;
}

float DecodeParamValue(float encoded_value, uint8_t param_type) {
  mavlink_param_union_t param{};
  param.param_float = encoded_value;

  switch (param_type) {
    case MAV_PARAM_TYPE_UINT8:
      return static_cast<float>(param.param_uint8);
    case MAV_PARAM_TYPE_INT8:
      return static_cast<float>(param.param_int8);
    case MAV_PARAM_TYPE_UINT16:
      return static_cast<float>(param.param_uint16);
    case MAV_PARAM_TYPE_INT16:
      return static_cast<float>(param.param_int16);
    case MAV_PARAM_TYPE_UINT32:
      return static_cast<float>(param.param_uint32);
    case MAV_PARAM_TYPE_INT32:
      return static_cast<float>(param.param_int32);
    case MAV_PARAM_TYPE_REAL32:
    default:
      return param.param_float;
  }
}

}  // namespace param_detail

}  // namespace

void Mavlink::HandleParamMessage(const mavlink_message_t &msg) {
  switch (msg.msgid) {
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
      mavlink_param_request_list_t req{};
      mavlink_msg_param_request_list_decode(&msg, &req);
      if (IsTargetedToThisComponent(req.target_system, req.target_component)) {
        udp_tx_.param_stream_ = TxState::ParamStreamActive{};
      }
      break;
    }
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
      mavlink_param_request_read_t req{};
      mavlink_msg_param_request_read_decode(&msg, &req);
      if (!IsTargetedToThisComponent(req.target_system, req.target_component)) {
        break;
      }

      const std::optional<ParamRef> param =
          TryResolveParam(req.param_index, req.param_id);
      if (param.has_value()) {
        QueueParamValue(udp_tx_, ParamMavlinkIndex(*param));
        break;
      }

      if (req.param_index >= 0) {
        ESP_LOGW(kTag, "PARAM_REQUEST_READ unresolved index=%d",
                 static_cast<int>(req.param_index));
        char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1] = {};
        std::snprintf(text, sizeof(text), "Unhandled PARAM_READ index=%d",
                      static_cast<int>(req.param_index));
        NotifyGcsIssue(text, MAV_SEVERITY_WARNING);
      } else {
        const auto param_id = param_detail::MavTextToCString(req.param_id);
        ESP_LOGW(kTag, "PARAM_REQUEST_READ unresolved param_id=%s",
                 param_id.data());
        char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1] = {};
        std::snprintf(text, sizeof(text), "Unhandled PARAM_READ %s",
                      param_id.data());
        NotifyGcsIssue(text, MAV_SEVERITY_WARNING);
      }
      break;
    }
    case MAVLINK_MSG_ID_PARAM_SET: {
      mavlink_param_set_t req{};
      mavlink_msg_param_set_decode(&msg, &req);
      if (!IsTargetedToThisComponent(req.target_system, req.target_component)) {
        break;
      }

      const std::optional<ParamRef> param = TryResolveParam(-1, req.param_id);
      if (!param.has_value()) {
        const auto param_id = param_detail::MavTextToCString(req.param_id);
        ESP_LOGW(kTag, "PARAM_SET unresolved param_id=%s", param_id.data());
        char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1] = {};
        std::snprintf(text, sizeof(text), "Unhandled PARAM_SET %s",
                      param_id.data());
        NotifyGcsIssue(text, MAV_SEVERITY_WARNING);
        break;
      }

      const float decoded_value =
          param_detail::DecodeParamValue(req.param_value, req.param_type);
      const ParamSetResult set_result = TrySetParam(*param, decoded_value);
      if (set_result != ParamSetResult::kAccepted) {
        const auto param_id = param_detail::MavTextToCString(req.param_id);
        ESP_LOGW(kTag, "PARAM_SET rejected param_id=%s reason=%s",
                 param_id.data(), ParamSetResultName(set_result));
        char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1] = {};
        std::snprintf(text, sizeof(text), "Rejected PARAM_SET %s",
                      param_id.data());
        NotifyGcsIssue(text, MAV_SEVERITY_WARNING);
      }
      QueueParamValue(udp_tx_, ParamMavlinkIndex(*param));
      break;
    }
    case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST: {
      mavlink_param_ext_request_list_t req{};
      mavlink_msg_param_ext_request_list_decode(&msg, &req);
      if (!IsTargetedToThisComponent(req.target_system, req.target_component)) {
        break;
      }
      ESP_LOGW(kTag, "PARAM_EXT_REQUEST_LIST unsupported");
      NotifyGcsIssue("Unsupported PARAM_EXT_LIST", MAV_SEVERITY_WARNING);
      break;
    }
    case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ: {
      mavlink_param_ext_request_read_t req{};
      mavlink_msg_param_ext_request_read_decode(&msg, &req);
      if (!IsTargetedToThisComponent(req.target_system, req.target_component)) {
        break;
      }
      ESP_LOGW(kTag, "PARAM_EXT_REQUEST_READ unsupported");
      NotifyGcsIssue("Unsupported PARAM_EXT_READ", MAV_SEVERITY_WARNING);
      break;
    }
    case MAVLINK_MSG_ID_PARAM_EXT_SET: {
      mavlink_param_ext_set_t req{};
      mavlink_msg_param_ext_set_decode(&msg, &req);
      if (!IsTargetedToThisComponent(req.target_system, req.target_component)) {
        break;
      }

      const auto param_id = param_detail::MavTextToCString(req.param_id);
      const auto param_value = param_detail::MavTextToCString(req.param_value);

      ESP_LOGW(kTag,
               "PARAM_EXT_SET unsupported target_sys=%u target_comp=%u "
               "param_id=%s value=%s type=%u",
               static_cast<unsigned>(req.target_system),
               static_cast<unsigned>(req.target_component), param_id.data(),
               param_value.data(), static_cast<unsigned>(req.param_type));
      char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1] = {};
      std::snprintf(text, sizeof(text), "Unsupported PARAM_EXT_SET %s",
                    param_id.data());
      NotifyGcsIssue(text, MAV_SEVERITY_WARNING);
      break;
    }
    default:
      NotifyGcsIssue("Unhandled MAVLink param message", MAV_SEVERITY_WARNING);
      break;
  }
}

void Mavlink::ResetParamState() {
  rc_map_config_ = {};
  rc_calibration_config_ = {};
  gyro_calibration_id_config_ = {};
  rc_map_request_ = {};
  rc_calibration_request_ = {};
  gyro_calibration_id_request_ = {};
  rc_map_apply_.Reset();
  rc_calibration_apply_.Reset();
}

uint16_t Mavlink::ParamMavlinkIndex(const ParamRef &param) {
  return std::visit([](const auto &ref) { return ref.mavlink_index; }, param);
}

const char *Mavlink::ParamSetResultName(ParamSetResult result) {
  switch (result) {
    case ParamSetResult::kAccepted:
      return "accepted";
    case ParamSetResult::kUnsupported:
      return "unsupported";
    case ParamSetResult::kMissingBaseConfig:
      return "missing-base-config";
    case ParamSetResult::kInvalidValue:
      return "invalid-value";
    case ParamSetResult::kInvalidResultingConfig:
      return "invalid-resulting-config";
  }
  return "unknown";
}

std::optional<Mavlink::ParamRef> Mavlink::TryResolveRcCalibrationParam(
    const char *param_id) {
  unsigned channel = 0;
  char suffix[6] = {};
  if (std::sscanf(param_id, "RC%u_%5s", &channel, suffix) != 2) {
    return std::nullopt;
  }
  if (channel < 1u || channel > message::kRcCalibrationChannelCount) {
    return std::nullopt;
  }

  const uint8_t channel_index = static_cast<uint8_t>(channel - 1u);
  const uint16_t channel_offset =
      channel_index * param_detail::kRcCalibrationParamCountPerChannel;
  if (std::strcmp(suffix, "MIN") == 0) {
    return RcCalibrationParamRef{
        static_cast<uint16_t>(param_detail::kBaseParamCount + channel_offset),
        channel_index, 0u};
  }
  if (std::strcmp(suffix, "MAX") == 0) {
    return RcCalibrationParamRef{
        static_cast<uint16_t>(param_detail::kBaseParamCount + channel_offset +
                              1u),
        channel_index, 1u};
  }
  if (std::strcmp(suffix, "TRIM") == 0) {
    return RcCalibrationParamRef{
        static_cast<uint16_t>(param_detail::kBaseParamCount + channel_offset +
                              2u),
        channel_index, 2u};
  }
  if (std::strcmp(suffix, "REV") == 0) {
    return RcCalibrationParamRef{
        static_cast<uint16_t>(param_detail::kBaseParamCount + channel_offset +
                              3u),
        channel_index, 3u};
  }
  return std::nullopt;
}

std::optional<Mavlink::ParamRef> Mavlink::TryResolveParam(
    int16_t requested_index, const char *requested_id) const {
  if (requested_index >= 0) {
    return TryResolveParamByIndex(static_cast<uint16_t>(requested_index));
  }

  if (requested_id == nullptr) {
    return std::nullopt;
  }

  std::array<char, kMavParamIdCStringLen> param_id{};
  std::memcpy(param_id.data(), requested_id,
              MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
  param_id[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';

  for (uint16_t param_index = 0; param_index < param_detail::kBaseParamCount;
      ++param_index) {
    if (std::strncmp(param_id.data(), param_detail::kParamTable[param_index].id,
                     MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN) == 0) {
      return FixedParamRef{param_index};
    }
  }

  if (std::strncmp(param_id.data(), "RC", 2) != 0) {
    return std::nullopt;
  }

  return TryResolveRcCalibrationParam(param_id.data());
}

std::optional<Mavlink::ParamRef> Mavlink::TryResolveParamByIndex(
    uint16_t param_index) const {
  if (param_index < param_detail::kBaseParamCount) {
    return FixedParamRef{param_index};
  }
  if (param_index >= param_detail::kTotalParamCount) {
    return std::nullopt;
  }

  const uint16_t rc_param_index = param_index - param_detail::kBaseParamCount;
  const uint8_t channel_index = static_cast<uint8_t>(
      rc_param_index / param_detail::kRcCalibrationParamCountPerChannel);
  const uint8_t field_index = static_cast<uint8_t>(
      rc_param_index % param_detail::kRcCalibrationParamCountPerChannel);
  return RcCalibrationParamRef{param_index, channel_index, field_index};
}

std::optional<Mavlink::EncodedParam> Mavlink::TryEncodeParam(
    const ParamRef &param) const {
  if (const auto *fixed = std::get_if<FixedParamRef>(&param)) {
    return TryEncodeFixedParam(*fixed);
  }

  return TryEncodeRcCalibrationParam(std::get<RcCalibrationParamRef>(param));
}

std::optional<Mavlink::EncodedParam> Mavlink::TryEncodeFixedParam(
    const FixedParamRef &param) const {
  const param_detail::ParamDef &def =
      param_detail::kParamTable[param.mavlink_index];
  EncodedParam encoded{};
  encoded.type = def.type;
  std::strncpy(encoded.id, def.id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
  encoded.id[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';

  switch (def.key) {
    case param_detail::ParamKey::kSysId:
    case param_detail::ParamKey::kMavSysId:
      encoded.value = static_cast<float>(cfg_.identity.sysid);
      return encoded;
    case param_detail::ParamKey::kCompId:
      encoded.value = static_cast<float>(cfg_.identity.compid);
      return encoded;
    case param_detail::ParamKey::kCalAcc0Id:
    case param_detail::ParamKey::kCalGyro0Id: {
      const std::optional<float> value = TryEncodeGyroCalibrationIdParam();
      if (!value.has_value()) {
        return std::nullopt;
      }
      encoded.value = *value;
      return encoded;
    }
    case param_detail::ParamKey::kCalMag0Id:
    case param_detail::ParamKey::kCalMag1Id:
    case param_detail::ParamKey::kCalMag2Id:
      encoded.value = 0.0f;
      return encoded;
    case param_detail::ParamKey::kCalMag0Rot:
    case param_detail::ParamKey::kCalMag1Rot:
    case param_detail::ParamKey::kCalMag2Rot:
      encoded.value = -1.0f;
      return encoded;
    case param_detail::ParamKey::kSensBoardRot:
    case param_detail::ParamKey::kSensDpresOff:
    case param_detail::ParamKey::kSysHasMag:
    case param_detail::ParamKey::kSysHasNumAspd:
    case param_detail::ParamKey::kComRcInMode:
    case param_detail::ParamKey::kComRcLossT:
      encoded.value = 0.0f;
      return encoded;
    case param_detail::ParamKey::kSysAutostart:
      encoded.value = static_cast<float>(esp32_limits::kMavlinkSysAutostart);
      return encoded;
    case param_detail::ParamKey::kRcChanCnt:
      encoded.value = static_cast<float>(message::kRcCalibrationChannelCount);
      return encoded;
    case param_detail::ParamKey::kRcMapRoll:
    case param_detail::ParamKey::kRcMapPitch:
    case param_detail::ParamKey::kRcMapYaw:
    case param_detail::ParamKey::kRcMapThrottle: {
      const std::optional<float> value = TryEncodeRcMapParam(param);
      if (!value.has_value()) {
        return std::nullopt;
      }
      encoded.value = *value;
      return encoded;
    }
    case param_detail::ParamKey::kRcMapFlaps:
    case param_detail::ParamKey::kRcMapAux1:
    case param_detail::ParamKey::kRcMapAux2:
    case param_detail::ParamKey::kRcMapParam1:
    case param_detail::ParamKey::kRcMapParam2:
    case param_detail::ParamKey::kRcMapParam3:
    case param_detail::ParamKey::kRcMapPaySw:
      encoded.value = 0.0f;
      return encoded;
    case param_detail::ParamKey::kHeartbeatMs:
      encoded.value = static_cast<float>(cfg_.tx.periods.hb_ms);
      return encoded;
    case param_detail::ParamKey::kGpsMs:
      encoded.value = static_cast<float>(cfg_.tx.periods.gps_ms);
      return encoded;
    case param_detail::ParamKey::kAttMs:
      encoded.value = static_cast<float>(cfg_.tx.periods.att_ms);
      return encoded;
    case param_detail::ParamKey::kGposMs:
      encoded.value = static_cast<float>(cfg_.tx.periods.gpos_ms);
      return encoded;
    case param_detail::ParamKey::kBattMs:
      encoded.value = static_cast<float>(cfg_.tx.periods.batt_ms);
      return encoded;
    case param_detail::ParamKey::kRcMs:
      encoded.value = static_cast<float>(cfg_.tx.periods.rc_ms);
      return encoded;
  }

  return std::nullopt;
}

std::optional<float> Mavlink::TryEncodeGyroCalibrationIdParam() const {
  const std::optional<message::GyroCalibrationIdConfigMsg> gyro_cfg =
      GetCachedValue(gyro_calibration_id_config_);
  if (!gyro_cfg.has_value()) {
    return std::nullopt;
  }

  return static_cast<float>(gyro_cfg->cal_gyro0_id);
}

std::optional<float> Mavlink::TryEncodeRcMapParam(
    const FixedParamRef &param) const {
  const param_detail::ParamDef &def =
      param_detail::kParamTable[param.mavlink_index];
  const std::optional<message::RcMapConfigMsg> rc_map =
      GetCachedValue(rc_map_config_);
  if (!rc_map.has_value()) {
    return std::nullopt;
  }

  switch (def.key) {
    case param_detail::ParamKey::kRcMapRoll:
      return static_cast<float>(rc_map->roll);
    case param_detail::ParamKey::kRcMapPitch:
      return static_cast<float>(rc_map->pitch);
    case param_detail::ParamKey::kRcMapYaw:
      return static_cast<float>(rc_map->yaw);
    case param_detail::ParamKey::kRcMapThrottle:
      return static_cast<float>(rc_map->throttle);
    default:
      return std::nullopt;
  }
}

std::optional<Mavlink::EncodedParam> Mavlink::TryEncodeRcCalibrationParam(
    const RcCalibrationParamRef &param) const {
  const std::optional<message::RcCalibrationConfigMsg> rc_calibration =
      GetCachedValue(rc_calibration_config_);
  if (!rc_calibration.has_value()) {
    return std::nullopt;
  }

  EncodedParam encoded{};
  switch (param.field_index) {
    case 0:
      std::snprintf(encoded.id, sizeof(encoded.id), "RC%u_MIN",
                    static_cast<unsigned>(param.channel_index + 1u));
      encoded.type = MAV_PARAM_TYPE_UINT16;
      encoded.value =
          static_cast<float>(rc_calibration->min_us[param.channel_index]);
      break;
    case 1:
      std::snprintf(encoded.id, sizeof(encoded.id), "RC%u_MAX",
                    static_cast<unsigned>(param.channel_index + 1u));
      encoded.type = MAV_PARAM_TYPE_UINT16;
      encoded.value =
          static_cast<float>(rc_calibration->max_us[param.channel_index]);
      break;
    case 2:
      std::snprintf(encoded.id, sizeof(encoded.id), "RC%u_TRIM",
                    static_cast<unsigned>(param.channel_index + 1u));
      encoded.type = MAV_PARAM_TYPE_UINT16;
      encoded.value =
          static_cast<float>(rc_calibration->trim_us[param.channel_index]);
      break;
    case 3:
    default:
      std::snprintf(encoded.id, sizeof(encoded.id), "RC%u_REV",
                    static_cast<unsigned>(param.channel_index + 1u));
      encoded.type = MAV_PARAM_TYPE_INT8;
      encoded.value =
          static_cast<float>(rc_calibration->rev[param.channel_index]);
      break;
  }
  return encoded;
}

Mavlink::ParamSetResult Mavlink::TrySetParam(const ParamRef &param,
                                             float param_value) {
  if (const auto *fixed = std::get_if<FixedParamRef>(&param)) {
    return TrySetFixedParam(*fixed, param_value);
  }

  return TrySetRcCalibrationParam(std::get<RcCalibrationParamRef>(param),
                                  param_value);
}

Mavlink::ParamSetResult Mavlink::TrySetFixedParam(const FixedParamRef &param,
                                                  float param_value) {
  const param_detail::ParamDef &def =
      param_detail::kParamTable[param.mavlink_index];
  switch (def.key) {
    case param_detail::ParamKey::kRcMapRoll:
    case param_detail::ParamKey::kRcMapPitch:
    case param_detail::ParamKey::kRcMapYaw:
    case param_detail::ParamKey::kRcMapThrottle:
      return TrySetRcMapParam(param, param_value);
    case param_detail::ParamKey::kRcMapFlaps:
    case param_detail::ParamKey::kRcMapAux1:
    case param_detail::ParamKey::kRcMapAux2:
    case param_detail::ParamKey::kRcMapParam1:
    case param_detail::ParamKey::kRcMapParam2:
    case param_detail::ParamKey::kRcMapParam3:
    case param_detail::ParamKey::kRcMapPaySw:
      if (std::lround(param_value) == 0) {
        return ParamSetResult::kAccepted;
      }
      return ParamSetResult::kInvalidValue;
    default:
      return ParamSetResult::kUnsupported;
  }
}

Mavlink::ParamSetResult Mavlink::TrySetRcMapParam(const FixedParamRef &param,
                                                  float param_value) {
  const param_detail::ParamDef &def =
      param_detail::kParamTable[param.mavlink_index];
  message::RcMapConfigMsg updated{};
  bool have_base = false;
  if (const message::RcMapConfigMsg *desired = rc_map_apply_.Desired()) {
    updated = *desired;
    have_base = true;
  } else if (rc_map_config_.have_data) {
    updated = rc_map_config_.value;
    have_base = true;
  }
  if (!have_base) {
    ESP_LOGW(kTag, "RC_MAP write rejected: current map unavailable");
    return ParamSetResult::kMissingBaseConfig;
  }

  const long value = std::lround(param_value);
  if (value < 1 || value > 4) {
    ESP_LOGW(kTag, "RC_MAP write rejected: param=%s value=%ld", def.id, value);
    return ParamSetResult::kInvalidValue;
  }

  switch (def.key) {
    case param_detail::ParamKey::kRcMapRoll:
      updated.roll = static_cast<uint8_t>(value);
      break;
    case param_detail::ParamKey::kRcMapPitch:
      updated.pitch = static_cast<uint8_t>(value);
      break;
    case param_detail::ParamKey::kRcMapYaw:
      updated.yaw = static_cast<uint8_t>(value);
      break;
    case param_detail::ParamKey::kRcMapThrottle:
      updated.throttle = static_cast<uint8_t>(value);
      break;
    default:
      return ParamSetResult::kUnsupported;
  }

  if (!message::IsRcMapConfigValid(updated)) {
    ESP_LOGW(kTag, "RC_MAP write rejected: invalid map r=%u p=%u y=%u t=%u",
             static_cast<unsigned>(updated.roll),
             static_cast<unsigned>(updated.pitch),
             static_cast<unsigned>(updated.yaw),
             static_cast<unsigned>(updated.throttle));
    return ParamSetResult::kInvalidResultingConfig;
  }

  const uint32_t now_ms = Sys().Timebase().NowMs();
  rc_map_apply_.Start(updated, now_ms, kFcConfigRequestAttempts);
  return ParamSetResult::kAccepted;
}

Mavlink::ParamSetResult Mavlink::TrySetRcCalibrationParam(
    const RcCalibrationParamRef &param, float param_value) {
  message::RcCalibrationConfigMsg updated{};
  bool have_base = false;
  if (const message::RcCalibrationConfigMsg *desired =
          rc_calibration_apply_.Desired()) {
    updated = *desired;
    have_base = true;
  } else if (rc_calibration_config_.have_data) {
    updated = rc_calibration_config_.value;
    have_base = true;
  }
  if (!have_base) {
    ESP_LOGW(kTag, "RC_CAL write rejected: current calibration unavailable");
    return ParamSetResult::kMissingBaseConfig;
  }
  const char *field_name = "MIN";

  switch (param.field_index) {
    case 0:
      field_name = "MIN";
      updated.min_us[param.channel_index] =
          static_cast<uint16_t>(std::lround(param_value));
      break;
    case 1:
      field_name = "MAX";
      updated.max_us[param.channel_index] =
          static_cast<uint16_t>(std::lround(param_value));
      break;
    case 2:
      field_name = "TRIM";
      updated.trim_us[param.channel_index] =
          static_cast<uint16_t>(std::lround(param_value));
      break;
    case 3:
      field_name = "REV";
      updated.rev[param.channel_index] = (param_value < 0.0f) ? -1 : 1;
      break;
    default:
      return ParamSetResult::kUnsupported;
  }

  if (!message::IsRcCalibrationConfigValid(updated)) {
    ESP_LOGW(kTag, "RC_CAL write rejected: ch=%u field=%s result=%u/%u/%u/%d",
             static_cast<unsigned>(param.channel_index + 1u), field_name,
             static_cast<unsigned>(updated.min_us[param.channel_index]),
             static_cast<unsigned>(updated.trim_us[param.channel_index]),
             static_cast<unsigned>(updated.max_us[param.channel_index]),
             static_cast<int>(updated.rev[param.channel_index]));
    return ParamSetResult::kInvalidResultingConfig;
  }

  const uint32_t now_ms = Sys().Timebase().NowMs();
  rc_calibration_apply_.Start(updated, now_ms, kFcConfigRequestAttempts);
  return ParamSetResult::kAccepted;
}

Mavlink::ParamDependency Mavlink::DependencyForParam(
    const ParamRef &param) const {
  const auto *fixed = std::get_if<FixedParamRef>(&param);
  if (fixed == nullptr) {
    return ParamDependency::kRcCalibration;
  }

  const param_detail::ParamDef &def =
      param_detail::kParamTable[fixed->mavlink_index];
  switch (def.key) {
    case param_detail::ParamKey::kCalAcc0Id:
    case param_detail::ParamKey::kCalGyro0Id:
      return ParamDependency::kGyroCalibrationId;
    case param_detail::ParamKey::kRcMapRoll:
    case param_detail::ParamKey::kRcMapPitch:
    case param_detail::ParamKey::kRcMapYaw:
    case param_detail::ParamKey::kRcMapThrottle:
      return ParamDependency::kRcMap;
    default:
      return ParamDependency::kNone;
  }
}

bool Mavlink::IsParamDependencyApplyPending(ParamDependency dependency) const {
  bool pending = false;
  switch (dependency) {
    case ParamDependency::kRcMap:
      pending = rc_map_apply_.IsActive();
      break;
    case ParamDependency::kRcCalibration:
      pending = rc_calibration_apply_.IsActive();
      break;
    case ParamDependency::kGyroCalibrationId:
    case ParamDependency::kNone:
    default:
      break;
  }
  return pending;
}

bool Mavlink::EnsureParamDependencyRequested(ParamDependency dependency) {
  if (dependency == ParamDependency::kNone) {
    return false;
  }

  message::MsgId request_id = message::MsgId::kPing;
  const char *description = "";
  ParamRequestState *request = nullptr;
  switch (dependency) {
    case ParamDependency::kRcMap:
      request_id = message::MsgId::kReqRcMap;
      description = "RC map";
      request = &rc_map_request_;
      break;
    case ParamDependency::kRcCalibration:
      request_id = message::MsgId::kReqRcCalibration;
      description = "RC calibration";
      request = &rc_calibration_request_;
      break;
    case ParamDependency::kGyroCalibrationId:
      request_id = message::MsgId::kReqGyroCalibrationId;
      description = "gyro calibration ID";
      request = &gyro_calibration_id_request_;
      break;
    case ParamDependency::kNone:
    default:
      return false;
  }

  const uint32_t now_ms = Sys().Timebase().NowMs();
  bool should_send = false;
  if (request != nullptr &&
      (!request->waiting ||
       static_cast<int32_t>(now_ms - request->next_request_ms) >= 0)) {
    request->waiting = true;
    request->next_request_ms = now_ms + kFcConfigRequestRetryPeriodMs;
    should_send = true;
  }

  if (!should_send) {
    return true;
  }

  message::Packet req_pkt{};
  req_pkt.header.id = static_cast<uint8_t>(request_id);
  req_pkt.header.len = 0;
  ESP_LOGI(kTag, "Requesting STM32 %s on demand...", description);
  Sys().FcLink().SendPacket(req_pkt);
  return true;
}

template <typename T>
Mavlink::PendingApplyAction<T> Mavlink::PreparePendingParamApply(
    PendingParamApplyState<T> &apply, uint32_t now_ms) {
  auto *active = apply.GetActive();
  if (active == nullptr ||
      static_cast<int32_t>(now_ms - active->next_retry_ms) < 0) {
    return PendingApplyNone{};
  }

  if (active->attempts_remaining == 0) {
    apply.Reset();
    return PendingApplyFailed{};
  }

  PendingApplySend<T> action{active->desired};
  active->attempts_remaining--;
  active->next_retry_ms = now_ms + kFcConfigRequestRetryPeriodMs;
  return action;
}

void Mavlink::ServicePendingParamApplies(uint32_t now_ms) {
  const PendingApplyAction<message::RcMapConfigMsg> rc_map_action =
      PreparePendingParamApply(rc_map_apply_, now_ms);
  const PendingApplyAction<message::RcCalibrationConfigMsg>
      rc_calibration_action =
          PreparePendingParamApply(rc_calibration_apply_, now_ms);

  if (std::holds_alternative<PendingApplyFailed>(rc_map_action)) {
    Panic(ErrorCode::kFcLinkRcMapSetFailed);
  }
  if (std::holds_alternative<PendingApplyFailed>(rc_calibration_action)) {
    Panic(ErrorCode::kFcLinkRcCalibrationSetFailed);
  }

  if (const auto *send = std::get_if<PendingApplySend<message::RcMapConfigMsg>>(
          &rc_map_action)) {
    message::Packet req_pkt{};
    req_pkt.header.id = static_cast<uint8_t>(message::MsgId::kSetRcMapConfig);
    req_pkt.header.len = message::PayloadLength<message::RcMapConfigMsg>();
    std::memcpy(req_pkt.payload, &send->value, sizeof(send->value));
    Sys().FcLink().SendPacket(req_pkt);
  }

  if (const auto *send =
          std::get_if<PendingApplySend<message::RcCalibrationConfigMsg>>(
              &rc_calibration_action)) {
    message::Packet req_pkt{};
    req_pkt.header.id =
        static_cast<uint8_t>(message::MsgId::kSetRcCalibrationConfig);
    req_pkt.header.len =
        message::PayloadLength<message::RcCalibrationConfigMsg>();
    std::memcpy(req_pkt.payload, &send->value, sizeof(send->value));
    Sys().FcLink().SendPacket(req_pkt);
  }
}

void Mavlink::QueueParamValue(TxState &tx, uint16_t param_index) const {
  if (!tx.pending_param_queue_.Push(param_index)) {
    ESP_LOGW(kTag, "dropping PARAM_VALUE reply: queue full");
  }
}

std::optional<Mavlink::TxFrameState> Mavlink::StartQueuedParamValueFrame(
    TxState &tx, uint8_t sysid, uint8_t compid) {
  uint16_t param_index = 0;
  if (!tx.pending_param_queue_.Peek(param_index)) {
    return std::nullopt;
  }

  const std::optional<ParamRef> param = TryResolveParamByIndex(param_index);
  if (!param.has_value()) {
    ESP_LOGW(kTag, "dropping queued PARAM_VALUE index=%u",
             static_cast<unsigned>(param_index));
    (void)tx.pending_param_queue_.Pop(param_index);
    return std::nullopt;
  }

  const ParamDependency dependency = DependencyForParam(*param);
  if (IsParamDependencyApplyPending(dependency)) {
    return std::nullopt;
  }

  const std::optional<TxFrameState> frame =
      StartParamValueFrame(*param, sysid, compid);
  if (!frame.has_value()) {
    (void)EnsureParamDependencyRequested(dependency);
    return std::nullopt;
  }

  (void)tx.pending_param_queue_.Pop(param_index);
  return frame;
}

std::optional<Mavlink::TxFrameState> Mavlink::StartStreamParamValueFrame(
    TxState &tx, uint8_t sysid, uint8_t compid) {
  auto *stream = std::get_if<TxState::ParamStreamActive>(&tx.param_stream_);
  if (stream == nullptr) {
    return std::nullopt;
  }

  if (stream->next_param_index >= param_detail::kTotalParamCount) {
    tx.param_stream_ = TxState::ParamStreamIdle{};
    return std::nullopt;
  }

  const std::optional<ParamRef> param =
      TryResolveParamByIndex(stream->next_param_index);
  if (!param.has_value()) {
    ESP_LOGW(kTag, "stopping PARAM stream at index=%u",
             static_cast<unsigned>(stream->next_param_index));
    tx.param_stream_ = TxState::ParamStreamIdle{};
    return std::nullopt;
  }

  const ParamDependency dependency = DependencyForParam(*param);
  if (IsParamDependencyApplyPending(dependency)) {
    return std::nullopt;
  }

  const std::optional<TxFrameState> frame =
      StartParamValueFrame(*param, sysid, compid);
  if (!frame.has_value()) {
    (void)EnsureParamDependencyRequested(dependency);
    return std::nullopt;
  }

  stream->next_param_index++;
  if (stream->next_param_index >= param_detail::kTotalParamCount) {
    tx.param_stream_ = TxState::ParamStreamIdle{};
  }
  return frame;
}

std::optional<Mavlink::TxFrameState> Mavlink::StartParamValueFrame(
    const ParamRef &param, uint8_t sysid, uint8_t compid) {
  const std::optional<EncodedParam> encoded = TryEncodeParam(param);
  if (!encoded.has_value()) {
    return std::nullopt;
  }

  mavlink_message_t m{};
  mavlink_msg_param_value_pack(
      sysid, compid, &m, encoded->id,
      param_detail::EncodeParamValue(encoded->value, encoded->type),
      encoded->type, param_detail::kTotalParamCount, ParamMavlinkIndex(param));

  TxFrameState frame{};
  frame.len = static_cast<uint16_t>(mavlink_msg_to_send_buffer(frame.buf, &m));
  frame.is_hb = false;
  return frame;
}

std::optional<Mavlink::TxFrameState> Mavlink::StartNextParamFrame(
    TxState &tx, uint8_t sysid, uint8_t compid) {
  if (const std::optional<TxFrameState> queued_frame =
          StartQueuedParamValueFrame(tx, sysid, compid)) {
    return queued_frame;
  }
  return StartStreamParamValueFrame(tx, sysid, compid);
}
