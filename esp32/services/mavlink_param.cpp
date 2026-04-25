#include <algorithm>
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

uint32_t IncrementGeneration(uint32_t generation) {
  return (generation == UINT32_MAX) ? 1u : (generation + 1u);
}

namespace param_detail {

inline void CopyMavTextField(const char *src, size_t src_len, char *dst,
                             size_t dst_len) {
  if (dst == nullptr || dst_len == 0) {
    return;
  }

  const size_t copy_len = std::min(src_len, dst_len - 1u);
  std::memcpy(dst, src, copy_len);
  dst[copy_len] = '\0';
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

enum class RcCalibrationField : uint8_t {
  kMin,
  kMax,
  kTrim,
  kRev,
};

inline int8_t QuantizeSignedUnit(float value) {
  return (value < 0.0f) ? -1 : 1;
}

inline const char *RcCalibrationFieldName(RcCalibrationField field) {
  switch (field) {
    case RcCalibrationField::kMin:
      return "MIN";
    case RcCalibrationField::kMax:
      return "MAX";
    case RcCalibrationField::kTrim:
      return "TRIM";
    case RcCalibrationField::kRev:
      return "REV";
  }
  return "?";
}

inline float EncodeParamValue(float value, uint8_t param_type) {
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

inline float DecodeParamValue(float encoded_value, uint8_t param_type) {
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
      if (req.target_system == cfg_.identity.sysid &&
          (req.target_component == cfg_.identity.compid ||
           req.target_component == 0 ||
           req.target_component == MAV_COMP_ID_ALL)) {
        BeginParamStream(udp_tx_);
      }
      break;
    }
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
      mavlink_param_request_read_t req{};
      mavlink_msg_param_request_read_decode(&msg, &req);
      if (req.target_system != cfg_.identity.sysid ||
          (req.target_component != cfg_.identity.compid &&
           req.target_component != 0 &&
           req.target_component != MAV_COMP_ID_ALL)) {
        break;
      }

      if (const std::optional<ParamRef> param =
              TryResolveParam(req.param_index, req.param_id)) {
        QueueParamValue(udp_tx_, param->index);
      }
      break;
    }
    case MAVLINK_MSG_ID_PARAM_SET: {
      mavlink_param_set_t req{};
      mavlink_msg_param_set_decode(&msg, &req);
      if (req.target_system != cfg_.identity.sysid ||
          (req.target_component != cfg_.identity.compid &&
           req.target_component != 0 &&
           req.target_component != MAV_COMP_ID_ALL)) {
        break;
      }

      const std::optional<ParamRef> param = TryResolveParam(-1, req.param_id);
      if (!param.has_value()) {
        ESP_LOGW(kTag, "PARAM_SET unresolved param_id=%.*s", 16, req.param_id);
        break;
      }

      const float decoded_value =
          param_detail::DecodeParamValue(req.param_value, req.param_type);
      (void)TrySetParam(*param, decoded_value, req.param_type);
      QueueParamValue(udp_tx_, param->index);
      break;
    }
    case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST:
      ESP_LOGW(kTag, "PARAM_EXT_REQUEST_LIST unsupported");
      break;
    case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ:
      ESP_LOGW(kTag, "PARAM_EXT_REQUEST_READ unsupported");
      break;
    case MAVLINK_MSG_ID_PARAM_EXT_SET: {
      mavlink_param_ext_set_t req{};
      mavlink_msg_param_ext_set_decode(&msg, &req);

      char param_id[17] = {};
      char param_value[129] = {};
      param_detail::CopyMavTextField(req.param_id, sizeof(req.param_id),
                                     param_id, sizeof(param_id));
      param_detail::CopyMavTextField(req.param_value, sizeof(req.param_value),
                                     param_value, sizeof(param_value));

      ESP_LOGW(kTag,
               "PARAM_EXT_SET unsupported target_sys=%u target_comp=%u "
               "param_id=%s value=%s type=%u",
               static_cast<unsigned>(req.target_system),
               static_cast<unsigned>(req.target_component), param_id,
               param_value, static_cast<unsigned>(req.param_type));
      break;
    }
    default:
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
  rc_map_apply_ = {};
  rc_calibration_apply_ = {};
}

void Mavlink::UpdateRcMapConfigCache(const message::RcMapConfigMsg &cfg,
                                     uint32_t now_ms) {
  bool confirmed_apply = false;
  if (!message::IsRcMapConfigValid(cfg)) {
    Panic(ErrorCode::kFcLinkInvalidRcMapConfig);
  }
  rc_map_config_.value = cfg;
  rc_map_config_.have_data = true;
  rc_map_config_.update_ms = now_ms;
  rc_map_config_.generation = IncrementGeneration(rc_map_config_.generation);
  rc_map_request_ = {};
  confirmed_apply = rc_map_apply_.active &&
                    std::memcmp(&rc_map_apply_.desired, &cfg, sizeof(cfg)) == 0;
  if (confirmed_apply) {
    rc_map_apply_ = {};
  }
  if (confirmed_apply) {
    Sys().TonePlayer().PlayBuiltin(::TonePlayer::BuiltinTone::kConfirm);
  }
}

void Mavlink::UpdateRcCalibrationConfigCache(
    const message::RcCalibrationConfigMsg &cfg, uint32_t now_ms) {
  bool confirmed_apply = false;
  if (!message::IsRcCalibrationConfigValid(cfg)) {
    Panic(ErrorCode::kFcLinkInvalidRcCalibrationConfig);
  }
  rc_calibration_config_.value = cfg;
  rc_calibration_config_.have_data = true;
  rc_calibration_config_.update_ms = now_ms;
  rc_calibration_config_.generation =
      IncrementGeneration(rc_calibration_config_.generation);
  rc_calibration_request_ = {};
  confirmed_apply =
      rc_calibration_apply_.active &&
      std::memcmp(&rc_calibration_apply_.desired, &cfg, sizeof(cfg)) == 0;
  if (confirmed_apply) {
    rc_calibration_apply_ = {};
  }
  if (confirmed_apply) {
    Sys().TonePlayer().PlayBuiltin(::TonePlayer::BuiltinTone::kConfirm);
  }
}

void Mavlink::UpdateGyroCalibrationIdConfigCache(
    const message::GyroCalibrationIdConfigMsg &cfg, uint32_t now_ms) {
  if (!message::IsGyroCalibrationIdConfigValid(cfg)) {
    Panic(ErrorCode::kFcLinkInvalidGyroCalibrationIdConfig);
  }
  gyro_calibration_id_config_.value = cfg;
  gyro_calibration_id_config_.have_data = true;
  gyro_calibration_id_config_.update_ms = now_ms;
  gyro_calibration_id_config_.generation =
      IncrementGeneration(gyro_calibration_id_config_.generation);
  gyro_calibration_id_request_ = {};
}

bool Mavlink::TryGetRcMapConfig(message::RcMapConfigMsg &cfg) const {
  const bool have_data = rc_map_config_.have_data;
  if (have_data) {
    cfg = rc_map_config_.value;
  }
  return have_data;
}

bool Mavlink::TryGetRcCalibrationConfig(
    message::RcCalibrationConfigMsg &cfg) const {
  const bool have_data = rc_calibration_config_.have_data;
  if (have_data) {
    cfg = rc_calibration_config_.value;
  }
  return have_data;
}

bool Mavlink::TryGetGyroCalibrationIdConfig(
    message::GyroCalibrationIdConfigMsg &cfg) const {
  const bool have_data = gyro_calibration_id_config_.have_data;
  if (have_data) {
    cfg = gyro_calibration_id_config_.value;
  }
  return have_data;
}

std::array<char, 17> Mavlink::CopyMavParamId(const char *src) {
  std::array<char, 17> dst{};
  std::memcpy(dst.data(), src, 16);
  dst[16] = '\0';
  return dst;
}

Mavlink::ParamRef Mavlink::MakeFixedParamRef(uint16_t param_index) {
  ParamRef param{};
  param.family = ParamRef::Family::kFixed;
  param.index = param_index;
  return param;
}

Mavlink::ParamRef Mavlink::MakeRcCalibrationParamRef(uint16_t param_index,
                                                     uint8_t channel_index,
                                                     uint8_t field_index) {
  ParamRef param{};
  param.family = ParamRef::Family::kRcCalibration;
  param.index = param_index;
  param.channel_index = channel_index;
  param.field_index = field_index;
  return param;
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
    return MakeRcCalibrationParamRef(
        static_cast<uint16_t>(param_detail::kBaseParamCount + channel_offset +
                              0u),
        channel_index, 0u);
  }
  if (std::strcmp(suffix, "MAX") == 0) {
    return MakeRcCalibrationParamRef(
        static_cast<uint16_t>(param_detail::kBaseParamCount + channel_offset +
                              1u),
        channel_index, 1u);
  }
  if (std::strcmp(suffix, "TRIM") == 0) {
    return MakeRcCalibrationParamRef(
        static_cast<uint16_t>(param_detail::kBaseParamCount + channel_offset +
                              2u),
        channel_index, 2u);
  }
  if (std::strcmp(suffix, "REV") == 0) {
    return MakeRcCalibrationParamRef(
        static_cast<uint16_t>(param_detail::kBaseParamCount + channel_offset +
                              3u),
        channel_index, 3u);
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

  const auto param_id = CopyMavParamId(requested_id);

  for (uint16_t param_index = 0; param_index < param_detail::kBaseParamCount;
       ++param_index) {
    if (std::strncmp(param_id.data(), param_detail::kParamTable[param_index].id,
                     16) == 0) {
      return MakeFixedParamRef(param_index);
    }
  }

  return TryResolveRcCalibrationParam(param_id.data());
}

std::optional<Mavlink::ParamRef> Mavlink::TryResolveParamByIndex(
    uint16_t param_index) const {
  if (param_index < param_detail::kBaseParamCount) {
    return MakeFixedParamRef(param_index);
  }
  if (param_index >= param_detail::kTotalParamCount) {
    return std::nullopt;
  }

  const uint16_t rc_param_index = param_index - param_detail::kBaseParamCount;
  const uint8_t channel_index = static_cast<uint8_t>(
      rc_param_index / param_detail::kRcCalibrationParamCountPerChannel);
  const uint8_t field_index = static_cast<uint8_t>(
      rc_param_index % param_detail::kRcCalibrationParamCountPerChannel);
  return MakeRcCalibrationParamRef(param_index, channel_index, field_index);
}

bool Mavlink::TryEncodeParam(const ParamRef &param, char (&param_id)[17],
                             uint8_t &param_type, float &param_value) const {
  if (param.family == ParamRef::Family::kFixed) {
    return TryEncodeFixedParam(param, param_id, param_type, param_value);
  }

  return TryEncodeRcCalibrationParam(param, param_id, param_type, param_value);
}

bool Mavlink::TryEncodeFixedParam(const ParamRef &param, char (&param_id)[17],
                                  uint8_t &param_type,
                                  float &param_value) const {
  const param_detail::ParamDef &def = param_detail::kParamTable[param.index];
  param_type = def.type;
  std::strncpy(param_id, def.id, 16);

  switch (def.key) {
    case param_detail::ParamKey::kSysId:
    case param_detail::ParamKey::kMavSysId:
      param_value = static_cast<float>(cfg_.identity.sysid);
      return true;
    case param_detail::ParamKey::kCompId:
      param_value = static_cast<float>(cfg_.identity.compid);
      return true;
    case param_detail::ParamKey::kCalAcc0Id:
    case param_detail::ParamKey::kCalGyro0Id:
      return TryEncodeGyroCalibrationIdParam(param_value);
    case param_detail::ParamKey::kCalMag0Id:
    case param_detail::ParamKey::kCalMag1Id:
    case param_detail::ParamKey::kCalMag2Id:
      param_value = 0.0f;
      return true;
    case param_detail::ParamKey::kCalMag0Rot:
    case param_detail::ParamKey::kCalMag1Rot:
    case param_detail::ParamKey::kCalMag2Rot:
      param_value = -1.0f;
      return true;
    case param_detail::ParamKey::kSensBoardRot:
    case param_detail::ParamKey::kSensDpresOff:
    case param_detail::ParamKey::kSysHasMag:
    case param_detail::ParamKey::kSysHasNumAspd:
    case param_detail::ParamKey::kComRcInMode:
    case param_detail::ParamKey::kComRcLossT:
      param_value = 0.0f;
      return true;
    case param_detail::ParamKey::kSysAutostart:
      param_value = static_cast<float>(esp32_limits::kMavlinkSysAutostart);
      return true;
    case param_detail::ParamKey::kRcChanCnt:
      param_value = static_cast<float>(message::kRcCalibrationChannelCount);
      return true;
    case param_detail::ParamKey::kRcMapRoll:
    case param_detail::ParamKey::kRcMapPitch:
    case param_detail::ParamKey::kRcMapYaw:
    case param_detail::ParamKey::kRcMapThrottle:
      return TryEncodeRcMapParam(param, param_value);
    case param_detail::ParamKey::kRcMapFlaps:
    case param_detail::ParamKey::kRcMapAux1:
    case param_detail::ParamKey::kRcMapAux2:
    case param_detail::ParamKey::kRcMapParam1:
    case param_detail::ParamKey::kRcMapParam2:
    case param_detail::ParamKey::kRcMapParam3:
    case param_detail::ParamKey::kRcMapPaySw:
      param_value = 0.0f;
      return true;
    case param_detail::ParamKey::kHeartbeatMs:
      param_value = static_cast<float>(cfg_.tx.periods.hb_ms);
      return true;
    case param_detail::ParamKey::kGpsMs:
      param_value = static_cast<float>(cfg_.tx.periods.gps_ms);
      return true;
    case param_detail::ParamKey::kAttMs:
      param_value = static_cast<float>(cfg_.tx.periods.att_ms);
      return true;
    case param_detail::ParamKey::kGposMs:
      param_value = static_cast<float>(cfg_.tx.periods.gpos_ms);
      return true;
    case param_detail::ParamKey::kBattMs:
      param_value = static_cast<float>(cfg_.tx.periods.batt_ms);
      return true;
    case param_detail::ParamKey::kRcMs:
      param_value = static_cast<float>(cfg_.tx.periods.rc_ms);
      return true;
  }

  return false;
}

bool Mavlink::TryEncodeGyroCalibrationIdParam(float &param_value) const {
  message::GyroCalibrationIdConfigMsg gyro_cfg{};
  if (!TryGetGyroCalibrationIdConfig(gyro_cfg)) {
    return false;
  }

  param_value = static_cast<float>(gyro_cfg.cal_gyro0_id);
  return true;
}

bool Mavlink::TryEncodeRcMapParam(const ParamRef &param,
                                  float &param_value) const {
  const param_detail::ParamDef &def = param_detail::kParamTable[param.index];
  message::RcMapConfigMsg rc_map{};
  if (!TryGetRcMapConfig(rc_map)) {
    return false;
  }

  switch (def.key) {
    case param_detail::ParamKey::kRcMapRoll:
      param_value = static_cast<float>(rc_map.roll);
      return true;
    case param_detail::ParamKey::kRcMapPitch:
      param_value = static_cast<float>(rc_map.pitch);
      return true;
    case param_detail::ParamKey::kRcMapYaw:
      param_value = static_cast<float>(rc_map.yaw);
      return true;
    case param_detail::ParamKey::kRcMapThrottle:
      param_value = static_cast<float>(rc_map.throttle);
      return true;
    default:
      return false;
  }
}

bool Mavlink::TryEncodeRcCalibrationParam(const ParamRef &param,
                                          char (&param_id)[17],
                                          uint8_t &param_type,
                                          float &param_value) const {
  message::RcCalibrationConfigMsg rc_calibration{};
  if (!TryGetRcCalibrationConfig(rc_calibration)) {
    return false;
  }

  switch (param.field_index) {
    case 0:
      std::snprintf(param_id, sizeof(param_id), "RC%u_MIN",
                    static_cast<unsigned>(param.channel_index + 1u));
      param_type = MAV_PARAM_TYPE_UINT16;
      param_value =
          static_cast<float>(rc_calibration.min_us[param.channel_index]);
      break;
    case 1:
      std::snprintf(param_id, sizeof(param_id), "RC%u_MAX",
                    static_cast<unsigned>(param.channel_index + 1u));
      param_type = MAV_PARAM_TYPE_UINT16;
      param_value =
          static_cast<float>(rc_calibration.max_us[param.channel_index]);
      break;
    case 2:
      std::snprintf(param_id, sizeof(param_id), "RC%u_TRIM",
                    static_cast<unsigned>(param.channel_index + 1u));
      param_type = MAV_PARAM_TYPE_UINT16;
      param_value =
          static_cast<float>(rc_calibration.trim_us[param.channel_index]);
      break;
    case 3:
    default:
      std::snprintf(param_id, sizeof(param_id), "RC%u_REV",
                    static_cast<unsigned>(param.channel_index + 1u));
      param_type = MAV_PARAM_TYPE_INT8;
      param_value = static_cast<float>(rc_calibration.rev[param.channel_index]);
      break;
  }
  return true;
}

bool Mavlink::TrySetParam(const ParamRef &param, float param_value,
                          uint8_t param_type) {
  (void)param_type;
  if (param.family == ParamRef::Family::kFixed) {
    return TrySetFixedParam(param, param_value);
  }

  return TrySetRcCalibrationParam(param, param_value);
}

bool Mavlink::TrySetFixedParam(const ParamRef &param, float param_value) {
  const param_detail::ParamDef &def = param_detail::kParamTable[param.index];
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
      return std::lround(param_value) == 0;
    default:
      return false;
  }
}

bool Mavlink::TrySetRcMapParam(const ParamRef &param, float param_value) {
  const param_detail::ParamDef &def = param_detail::kParamTable[param.index];
  message::RcMapConfigMsg updated{};
  bool have_base = false;
  if (rc_map_apply_.active) {
    updated = rc_map_apply_.desired;
    have_base = true;
  } else if (rc_map_config_.have_data) {
    updated = rc_map_config_.value;
    have_base = true;
  }
  if (!have_base) {
    ESP_LOGW(kTag, "RC_MAP write rejected: current map unavailable");
    return false;
  }

  const long value = std::lround(param_value);
  if (value < 1 || value > 4) {
    ESP_LOGW(kTag, "RC_MAP write rejected: param=%s value=%ld", def.id, value);
    return false;
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
      return false;
  }

  if (!message::IsRcMapConfigValid(updated)) {
    ESP_LOGW(kTag, "RC_MAP write rejected: invalid map r=%u p=%u y=%u t=%u",
             static_cast<unsigned>(updated.roll),
             static_cast<unsigned>(updated.pitch),
             static_cast<unsigned>(updated.yaw),
             static_cast<unsigned>(updated.throttle));
    return false;
  }

  const uint32_t now_ms = Sys().Timebase().NowMs();
  rc_map_apply_.desired = updated;
  rc_map_apply_.next_retry_ms = now_ms;
  rc_map_apply_.attempts_remaining = kFcConfigRequestAttempts;
  rc_map_apply_.active = true;
  return true;
}

bool Mavlink::TrySetRcCalibrationParam(const ParamRef &param,
                                       float param_value) {
  message::RcCalibrationConfigMsg updated{};
  bool have_base = false;
  if (rc_calibration_apply_.active) {
    updated = rc_calibration_apply_.desired;
    have_base = true;
  } else if (rc_calibration_config_.have_data) {
    updated = rc_calibration_config_.value;
    have_base = true;
  }
  if (!have_base) {
    ESP_LOGW(kTag, "RC_CAL write rejected: current calibration unavailable");
    return false;
  }
  param_detail::RcCalibrationField field =
      param_detail::RcCalibrationField::kMin;

  switch (param.field_index) {
    case 0:
      field = param_detail::RcCalibrationField::kMin;
      updated.min_us[param.channel_index] =
          static_cast<uint16_t>(std::lround(param_value));
      break;
    case 1:
      field = param_detail::RcCalibrationField::kMax;
      updated.max_us[param.channel_index] =
          static_cast<uint16_t>(std::lround(param_value));
      break;
    case 2:
      field = param_detail::RcCalibrationField::kTrim;
      updated.trim_us[param.channel_index] =
          static_cast<uint16_t>(std::lround(param_value));
      break;
    case 3:
      field = param_detail::RcCalibrationField::kRev;
      updated.rev[param.channel_index] =
          param_detail::QuantizeSignedUnit(param_value);
      break;
    default:
      return false;
  }

  if (!message::IsRcCalibrationConfigValid(updated)) {
    ESP_LOGW(kTag, "RC_CAL write rejected: ch=%u field=%s result=%u/%u/%u/%d",
             static_cast<unsigned>(param.channel_index + 1u),
             param_detail::RcCalibrationFieldName(field),
             static_cast<unsigned>(updated.min_us[param.channel_index]),
             static_cast<unsigned>(updated.trim_us[param.channel_index]),
             static_cast<unsigned>(updated.max_us[param.channel_index]),
             static_cast<int>(updated.rev[param.channel_index]));
    return false;
  }

  const uint32_t now_ms = Sys().Timebase().NowMs();
  rc_calibration_apply_.desired = updated;
  rc_calibration_apply_.next_retry_ms = now_ms;
  rc_calibration_apply_.attempts_remaining = kFcConfigRequestAttempts;
  rc_calibration_apply_.active = true;
  return true;
}

Mavlink::ParamDependency Mavlink::DependencyForParam(
    const ParamRef &param) const {
  if (param.family == ParamRef::Family::kRcCalibration) {
    return ParamDependency::kRcCalibration;
  }

  const param_detail::ParamDef &def = param_detail::kParamTable[param.index];
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
      pending = rc_map_apply_.active;
      break;
    case ParamDependency::kRcCalibration:
      pending = rc_calibration_apply_.active;
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
Mavlink::PendingApplyAction Mavlink::PreparePendingParamApply(
    PendingParamApplyState<T> &apply, T &value, uint32_t now_ms) {
  PendingApplyAction action{};
  if (!apply.active || static_cast<int32_t>(now_ms - apply.next_retry_ms) < 0) {
    return action;
  }

  if (apply.attempts_remaining == 0) {
    apply = {};
    action.failed = true;
    return action;
  }

  value = apply.desired;
  apply.attempts_remaining--;
  apply.next_retry_ms = now_ms + kFcConfigRequestRetryPeriodMs;
  action.send = true;
  return action;
}

void Mavlink::ServicePendingParamApplies(uint32_t now_ms) {
  message::RcMapConfigMsg rc_map{};
  message::RcCalibrationConfigMsg rc_calibration{};
  PendingApplyAction rc_map_action{};
  PendingApplyAction rc_calibration_action{};

  rc_map_action = PreparePendingParamApply(rc_map_apply_, rc_map, now_ms);
  rc_calibration_action =
      PreparePendingParamApply(rc_calibration_apply_, rc_calibration, now_ms);

  if (rc_map_action.failed) {
    Panic(ErrorCode::kFcLinkRcMapSetFailed);
  }
  if (rc_calibration_action.failed) {
    Panic(ErrorCode::kFcLinkRcCalibrationSetFailed);
  }

  if (rc_map_action.send) {
    message::Packet req_pkt{};
    req_pkt.header.id = static_cast<uint8_t>(message::MsgId::kSetRcMapConfig);
    req_pkt.header.len = message::PayloadLength<message::RcMapConfigMsg>();
    std::memcpy(req_pkt.payload, &rc_map, sizeof(rc_map));
    Sys().FcLink().SendPacket(req_pkt);
  }

  if (rc_calibration_action.send) {
    message::Packet req_pkt{};
    req_pkt.header.id =
        static_cast<uint8_t>(message::MsgId::kSetRcCalibrationConfig);
    req_pkt.header.len =
        message::PayloadLength<message::RcCalibrationConfigMsg>();
    std::memcpy(req_pkt.payload, &rc_calibration, sizeof(rc_calibration));
    Sys().FcLink().SendPacket(req_pkt);
  }
}

void Mavlink::BeginParamStream(TxState &tx) const {
  tx.param_stream_active = true;
  tx.next_param_index = 0;
}

bool Mavlink::HasPendingParamWork(const TxState &tx) const {
  return !tx.pending_param_queue_.IsEmpty() || tx.param_stream_active;
}

void Mavlink::QueueParamValue(TxState &tx, uint16_t param_index) const {
  if (!PushPendingParamValue(tx, param_index)) {
    ESP_LOGW(kTag, "dropping PARAM_VALUE reply: queue full");
  }
}

bool Mavlink::PushPendingParamValue(TxState &tx, uint16_t param_index) {
  return tx.pending_param_queue_.Push(param_index);
}

std::optional<uint16_t> Mavlink::PopPendingParamValue(TxState &tx) {
  uint16_t param_index = 0;
  if (!tx.pending_param_queue_.Pop(param_index)) {
    return std::nullopt;
  }
  return param_index;
}

std::optional<uint16_t> Mavlink::PeekPendingParamValue(const TxState &tx) {
  uint16_t param_index = 0;
  if (!tx.pending_param_queue_.Peek(param_index)) {
    return std::nullopt;
  }
  return param_index;
}

bool Mavlink::StartQueuedParamValueFrame(TxState &tx, TxFrameState &frame,
                                         uint8_t sysid, uint8_t compid) {
  const std::optional<uint16_t> param_index = PeekPendingParamValue(tx);
  if (!param_index.has_value()) {
    return false;
  }

  const std::optional<ParamRef> param = TryResolveParamByIndex(*param_index);
  if (!param.has_value()) {
    return false;
  }

  const ParamDependency dependency = DependencyForParam(*param);
  if (IsParamDependencyApplyPending(dependency)) {
    return false;
  }

  if (!StartParamValueFrame(*param, frame, sysid, compid)) {
    (void)EnsureParamDependencyRequested(dependency);
    return false;
  }

  (void)PopPendingParamValue(tx);
  return true;
}

bool Mavlink::StartStreamParamValueFrame(TxState &tx, TxFrameState &frame,
                                         uint8_t sysid, uint8_t compid) {
  if (!tx.param_stream_active) {
    return false;
  }

  if (tx.next_param_index >= param_detail::kTotalParamCount) {
    tx.param_stream_active = false;
    tx.next_param_index = 0;
    return false;
  }

  const std::optional<ParamRef> param =
      TryResolveParamByIndex(tx.next_param_index);
  if (!param.has_value()) {
    tx.param_stream_active = false;
    tx.next_param_index = 0;
    return false;
  }

  const ParamDependency dependency = DependencyForParam(*param);
  if (IsParamDependencyApplyPending(dependency)) {
    return false;
  }

  if (!StartParamValueFrame(*param, frame, sysid, compid)) {
    (void)EnsureParamDependencyRequested(dependency);
    return false;
  }

  tx.next_param_index++;
  if (tx.next_param_index >= param_detail::kTotalParamCount) {
    tx.param_stream_active = false;
    tx.next_param_index = 0;
  }
  return true;
}

bool Mavlink::StartParamValueFrame(const ParamRef &param, TxFrameState &frame,
                                   uint8_t sysid, uint8_t compid) {
  char param_id[17] = {};
  uint8_t param_type = 0;
  float param_value = 0.0f;
  if (!TryEncodeParam(param, param_id, param_type, param_value)) {
    return false;
  }

  mavlink_message_t m{};
  mavlink_msg_param_value_pack(
      sysid, compid, &m, param_id,
      param_detail::EncodeParamValue(param_value, param_type), param_type,
      param_detail::kTotalParamCount, param.index);

  frame.len = static_cast<uint16_t>(mavlink_msg_to_send_buffer(frame.buf, &m));
  frame.is_hb = false;
  return true;
}

bool Mavlink::StartNextParamFrame(TxState &tx, TxFrameState &frame,
                                  uint8_t sysid, uint8_t compid) {
  if (StartQueuedParamValueFrame(tx, frame, sysid, compid)) {
    return true;
  }
  return StartStreamParamValueFrame(tx, frame, sysid, compid);
}
