// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "mavlink_param.hpp"

#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <span>

#include "checksum.hpp"
#include "common_config.hpp"
#include "esp_log.h"
#include "message.hpp"

namespace {

constexpr const char *kTag = "mavlink";

namespace param_detail {

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
  kEscMs,
  kComRcLossT,
};

struct ParamDef {
  const char *id;
  uint8_t type;
  ParamKey key;
  // Left out of the parameter hash. QGC resolves our names against PX4's
  // metadata and skips whatever that marks `volatile`, so a name PX4 calls
  // volatile has to be skipped here too or the two CRCs can never agree.
  bool px4_volatile = false;
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
    {"SENS_DPRES_OFF", MAV_PARAM_TYPE_REAL32, ParamKey::kSensDpresOff,
     /*px4_volatile=*/true},
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
    {"MAV_ESC_MS", MAV_PARAM_TYPE_UINT16, ParamKey::kEscMs},
    {"COM_RC_LOSS_T", MAV_PARAM_TYPE_REAL32, ParamKey::kComRcLossT},
};

inline constexpr uint8_t kRcCalibrationParamCountPerChannel = 4u;
inline constexpr uint8_t kTotalRcCalibrationParamCount =
    message::kRcCalibrationChannelCount * kRcCalibrationParamCountPerChannel;
inline constexpr uint16_t kBaseParamCount =
    static_cast<uint16_t>(sizeof(kParamTable) / sizeof(kParamTable[0]));
inline constexpr uint16_t kTotalParamCount =
    kBaseParamCount + kTotalRcCalibrationParamCount;

// QGC asks any px4Firmware() vehicle for this before downloading parameters:
// a matching hash lets it load its own cache instead. Virtual, so it is not in
// kTotalParamCount and rides the reply queue on an index no real one uses.
inline constexpr char kHashCheckParamId[] = "_HASH_CHECK";
inline constexpr uint16_t kHashCheckQueueIndex = 0xFFFFu;

// The value as QGC caches it, whose width and representation are what it feeds
// its own CRC -- one byte for a uint8 parameter, four for an int32 or a float.
size_t ParamValueBytes(float value, uint8_t param_type, uint8_t out[4]) {
  switch (param_type) {
    case MAV_PARAM_TYPE_UINT8: {
      const auto v = static_cast<uint8_t>(std::lround(value));
      out[0] = v;
      return 1;
    }
    case MAV_PARAM_TYPE_INT8: {
      const auto v = static_cast<int8_t>(std::lround(value));
      std::memcpy(out, &v, 1);
      return 1;
    }
    case MAV_PARAM_TYPE_UINT16: {
      const auto v = static_cast<uint16_t>(std::lround(value));
      std::memcpy(out, &v, 2);
      return 2;
    }
    case MAV_PARAM_TYPE_INT16: {
      const auto v = static_cast<int16_t>(std::lround(value));
      std::memcpy(out, &v, 2);
      return 2;
    }
    case MAV_PARAM_TYPE_UINT32: {
      const auto v = static_cast<uint32_t>(std::llround(value));
      std::memcpy(out, &v, 4);
      return 4;
    }
    case MAV_PARAM_TYPE_INT32: {
      const auto v = static_cast<int32_t>(std::llround(value));
      std::memcpy(out, &v, 4);
      return 4;
    }
    case MAV_PARAM_TYPE_REAL32:
    default:
      std::memcpy(out, &value, 4);
      return 4;
  }
}

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

const char *MavlinkParamServer::SetResultName(SetResult result) {
  switch (result) {
    case SetResult::kAccepted:
      return "accepted";
    case SetResult::kUnknownParam:
      return "unknown-param";
    case SetResult::kUnsupported:
      return "unsupported";
    case SetResult::kMissingBaseConfig:
      return "missing-base-config";
    case SetResult::kInvalidValue:
      return "invalid-value";
    case SetResult::kInvalidResultingConfig:
      return "invalid-resulting-config";
  }
  return "unknown";
}

void MavlinkParamServer::Init(const MavlinkConfig &cfg,
                              FcConfigCache &fc_config) {
  cfg_ = &cfg;
  fc_config_ = &fc_config;
  Reset();
}

void MavlinkParamServer::Reset() {
  reply_queue_.Clear();
  stream_ = StreamIdle{};
}

void MavlinkParamServer::StartStream() { stream_ = StreamActive{}; }

bool MavlinkParamServer::QueueRead(int16_t param_index, const char *param_id) {
  if (const std::optional<ParamRef> param =
          TryResolveParam(param_index, param_id)) {
    QueueReply(ParamMavlinkIndex(*param));
    return true;
  }
  if (param_index < 0 &&
      std::strncmp(param_id, param_detail::kHashCheckParamId,
                   sizeof(param_detail::kHashCheckParamId)) == 0) {
    QueueReply(param_detail::kHashCheckQueueIndex);
    return true;
  }
  return false;
}

MavlinkParamServer::SetResult MavlinkParamServer::Set(const char *param_id,
                                                      float param_value,
                                                      uint8_t param_type) {
  const std::optional<ParamRef> param = TryResolveParam(-1, param_id);
  if (!param.has_value()) {
    return SetResult::kUnknownParam;
  }
  const SetResult result = TrySetParam(
      *param, param_detail::DecodeParamValue(param_value, param_type));
  QueueReply(ParamMavlinkIndex(*param));
  return result;
}

uint16_t MavlinkParamServer::ParamMavlinkIndex(const ParamRef &param) {
  return std::visit([](const auto &ref) { return ref.mavlink_index; }, param);
}

std::optional<MavlinkParamServer::ParamRef>
MavlinkParamServer::TryResolveRcCalibrationParam(const char *param_id) {
  // param_id arrives from the network. sscanf("%u") is undefined on a value
  // too large for the type, where strtoul is defined to saturate at ULONG_MAX,
  // which the range check below then rejects.
  if (param_id[0] != 'R' || param_id[1] != 'C') {
    return std::nullopt;
  }
  const char *digits = param_id + 2;
  char *end = nullptr;
  const unsigned long channel = std::strtoul(digits, &end, 10);
  if (end == digits || *end != '_') {
    return std::nullopt;
  }
  if (channel < 1u || channel > message::kRcCalibrationChannelCount) {
    return std::nullopt;
  }
  const char *const suffix = end + 1;

  const uint8_t channel_index = static_cast<uint8_t>(channel - 1u);
  const uint16_t channel_offset =
      channel_index * param_detail::kRcCalibrationParamCountPerChannel;
  if (std::strcmp(suffix, "MIN") == 0) {
    return RcCalibrationParamRef{
        static_cast<uint16_t>(param_detail::kBaseParamCount + channel_offset),
        channel_index, RcCalField::kMin};
  }
  if (std::strcmp(suffix, "MAX") == 0) {
    return RcCalibrationParamRef{
        static_cast<uint16_t>(param_detail::kBaseParamCount + channel_offset +
                              1u),
        channel_index, RcCalField::kMax};
  }
  if (std::strcmp(suffix, "TRIM") == 0) {
    return RcCalibrationParamRef{
        static_cast<uint16_t>(param_detail::kBaseParamCount + channel_offset +
                              2u),
        channel_index, RcCalField::kTrim};
  }
  if (std::strcmp(suffix, "REV") == 0) {
    return RcCalibrationParamRef{
        static_cast<uint16_t>(param_detail::kBaseParamCount + channel_offset +
                              3u),
        channel_index, RcCalField::kRev};
  }
  return std::nullopt;
}

std::optional<MavlinkParamServer::ParamRef> MavlinkParamServer::TryResolveParam(
    int16_t requested_index, const char *requested_id) const {
  if (requested_index >= 0) {
    return TryResolveParamByIndex(static_cast<uint16_t>(requested_index));
  }

  if (requested_id == nullptr) {
    return std::nullopt;
  }

  std::array<char, kParamIdCStringLen> param_id{};
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

std::optional<MavlinkParamServer::ParamRef>
MavlinkParamServer::TryResolveParamByIndex(uint16_t param_index) const {
  if (param_index < param_detail::kBaseParamCount) {
    return FixedParamRef{param_index};
  }
  if (param_index >= param_detail::kTotalParamCount) {
    return std::nullopt;
  }

  // Keeps the modulo below total over RcCalField.
  static_assert(param_detail::kRcCalibrationParamCountPerChannel ==
                static_cast<uint8_t>(RcCalField::kRev) + 1u);

  const uint16_t rc_param_index = param_index - param_detail::kBaseParamCount;
  const uint8_t channel_index = static_cast<uint8_t>(
      rc_param_index / param_detail::kRcCalibrationParamCountPerChannel);
  const auto field = static_cast<RcCalField>(
      rc_param_index % param_detail::kRcCalibrationParamCountPerChannel);
  return RcCalibrationParamRef{param_index, channel_index, field};
}

std::optional<MavlinkParamServer::EncodedParam>
MavlinkParamServer::TryEncodeParam(const ParamRef &param) const {
  if (const auto *fixed = std::get_if<FixedParamRef>(&param)) {
    return TryEncodeFixedParam(*fixed);
  }

  return TryEncodeRcCalibrationParam(std::get<RcCalibrationParamRef>(param));
}

std::optional<MavlinkParamServer::EncodedParam>
MavlinkParamServer::TryEncodeFixedParam(const FixedParamRef &param) const {
  const param_detail::ParamDef &def =
      param_detail::kParamTable[param.mavlink_index];
  EncodedParam encoded{};
  encoded.type = def.type;
  std::strncpy(encoded.id, def.id, MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN);
  encoded.id[MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN] = '\0';

  switch (def.key) {
    case param_detail::ParamKey::kSysId:
    case param_detail::ParamKey::kMavSysId:
      encoded.value = static_cast<float>(cfg_->sysid);
      return encoded;
    case param_detail::ParamKey::kCompId:
      encoded.value = static_cast<float>(kMavlinkComponentId);
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
      encoded.value = static_cast<float>(common_config::kAirframeSysAutostart);
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
      encoded.value = static_cast<float>(cfg_->tx.periods.hb_ms);
      return encoded;
    case param_detail::ParamKey::kGpsMs:
      encoded.value = static_cast<float>(cfg_->tx.periods.gps_ms);
      return encoded;
    case param_detail::ParamKey::kAttMs:
      encoded.value = static_cast<float>(cfg_->tx.periods.att_ms);
      return encoded;
    case param_detail::ParamKey::kGposMs:
      encoded.value = static_cast<float>(cfg_->tx.periods.gpos_ms);
      return encoded;
    case param_detail::ParamKey::kBattMs:
      encoded.value = static_cast<float>(cfg_->tx.periods.batt_ms);
      return encoded;
    case param_detail::ParamKey::kRcMs:
      encoded.value = static_cast<float>(cfg_->tx.periods.rc_ms);
      return encoded;
    case param_detail::ParamKey::kEscMs:
      encoded.value = static_cast<float>(cfg_->tx.periods.esc_ms);
      return encoded;
  }

  return std::nullopt;
}

std::optional<float> MavlinkParamServer::TryEncodeGyroCalibrationIdParam()
    const {
  const std::optional<message::GyroCalibrationIdConfigMsg> gyro_cfg =
      fc_config_->GyroCalibrationId();
  if (!gyro_cfg.has_value()) {
    return std::nullopt;
  }

  return static_cast<float>(gyro_cfg->cal_gyro0_id);
}

std::optional<float> MavlinkParamServer::TryEncodeRcMapParam(
    const FixedParamRef &param) const {
  const param_detail::ParamDef &def =
      param_detail::kParamTable[param.mavlink_index];
  const std::optional<message::RcMapConfigMsg> rc_map = fc_config_->RcMap();
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

std::optional<MavlinkParamServer::EncodedParam>
MavlinkParamServer::TryEncodeRcCalibrationParam(
    const RcCalibrationParamRef &param) const {
  const std::optional<message::RcCalibrationConfigMsg> rc_calibration =
      fc_config_->RcCalibration();
  if (!rc_calibration.has_value()) {
    return std::nullopt;
  }

  EncodedParam encoded{};
  switch (param.field) {
    case RcCalField::kMin:
      std::snprintf(encoded.id, sizeof(encoded.id), "RC%u_MIN",
                    static_cast<unsigned>(param.channel_index + 1u));
      encoded.type = MAV_PARAM_TYPE_UINT16;
      encoded.value =
          static_cast<float>(rc_calibration->min_us[param.channel_index]);
      break;
    case RcCalField::kMax:
      std::snprintf(encoded.id, sizeof(encoded.id), "RC%u_MAX",
                    static_cast<unsigned>(param.channel_index + 1u));
      encoded.type = MAV_PARAM_TYPE_UINT16;
      encoded.value =
          static_cast<float>(rc_calibration->max_us[param.channel_index]);
      break;
    case RcCalField::kTrim:
      std::snprintf(encoded.id, sizeof(encoded.id), "RC%u_TRIM",
                    static_cast<unsigned>(param.channel_index + 1u));
      encoded.type = MAV_PARAM_TYPE_UINT16;
      encoded.value =
          static_cast<float>(rc_calibration->trim_us[param.channel_index]);
      break;
    case RcCalField::kRev:
      std::snprintf(encoded.id, sizeof(encoded.id), "RC%u_REV",
                    static_cast<unsigned>(param.channel_index + 1u));
      encoded.type = MAV_PARAM_TYPE_INT8;
      encoded.value =
          static_cast<float>(rc_calibration->rev[param.channel_index]);
      break;
  }
  return encoded;
}

MavlinkParamServer::SetResult MavlinkParamServer::TrySetParam(
    const ParamRef &param, float param_value) {
  if (const auto *fixed = std::get_if<FixedParamRef>(&param)) {
    return TrySetFixedParam(*fixed, param_value);
  }

  return TrySetRcCalibrationParam(std::get<RcCalibrationParamRef>(param),
                                  param_value);
}

MavlinkParamServer::SetResult MavlinkParamServer::TrySetFixedParam(
    const FixedParamRef &param, float param_value) {
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
        return SetResult::kAccepted;
      }
      return SetResult::kInvalidValue;
    default:
      return SetResult::kUnsupported;
  }
}

MavlinkParamServer::SetResult MavlinkParamServer::TrySetRcMapParam(
    const FixedParamRef &param, float param_value) {
  const param_detail::ParamDef &def =
      param_detail::kParamTable[param.mavlink_index];
  const std::optional<message::RcMapConfigMsg> base =
      fc_config_->RcMapWriteBase();
  if (!base.has_value()) {
    ESP_LOGW(kTag, "RC_MAP write rejected: current map unavailable");
    return SetResult::kMissingBaseConfig;
  }
  message::RcMapConfigMsg updated = *base;

  const long value = std::lround(param_value);
  if (value < 1 || value > 4) {
    ESP_LOGW(kTag, "RC_MAP write rejected: param=%s value=%ld", def.id, value);
    return SetResult::kInvalidValue;
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
      return SetResult::kUnsupported;
  }

  if (!message::IsRcMapConfigValid(updated)) {
    ESP_LOGW(kTag, "RC_MAP write rejected: invalid map r=%u p=%u y=%u t=%u",
             static_cast<unsigned>(updated.roll),
             static_cast<unsigned>(updated.pitch),
             static_cast<unsigned>(updated.yaw),
             static_cast<unsigned>(updated.throttle));
    return SetResult::kInvalidResultingConfig;
  }

  fc_config_->WriteRcMap(updated);
  return SetResult::kAccepted;
}

MavlinkParamServer::SetResult MavlinkParamServer::TrySetRcCalibrationParam(
    const RcCalibrationParamRef &param, float param_value) {
  const std::optional<message::RcCalibrationConfigMsg> base =
      fc_config_->RcCalibrationWriteBase();
  if (!base.has_value()) {
    ESP_LOGW(kTag, "RC_CAL write rejected: current calibration unavailable");
    return SetResult::kMissingBaseConfig;
  }
  message::RcCalibrationConfigMsg updated = *base;
  // Overwritten by every arm below, but a scoped enum can still hold a value
  // outside its enumerators, and the log call further down would then read an
  // uninitialised pointer.
  // NOLINTNEXTLINE(clang-analyzer-deadcode.DeadStores)
  const char *field_name = "MIN";

  switch (param.field) {
    case RcCalField::kMin:
      field_name = "MIN";
      updated.min_us[param.channel_index] =
          static_cast<uint16_t>(std::lround(param_value));
      break;
    case RcCalField::kMax:
      field_name = "MAX";
      updated.max_us[param.channel_index] =
          static_cast<uint16_t>(std::lround(param_value));
      break;
    case RcCalField::kTrim:
      field_name = "TRIM";
      updated.trim_us[param.channel_index] =
          static_cast<uint16_t>(std::lround(param_value));
      break;
    case RcCalField::kRev:
      field_name = "REV";
      updated.rev[param.channel_index] = (param_value < 0.0f) ? -1 : 1;
      break;
  }

  if (!message::IsRcCalibrationConfigValid(updated)) {
    ESP_LOGW(kTag, "RC_CAL write rejected: ch=%u field=%s result=%u/%u/%u/%d",
             static_cast<unsigned>(param.channel_index + 1u), field_name,
             static_cast<unsigned>(updated.min_us[param.channel_index]),
             static_cast<unsigned>(updated.trim_us[param.channel_index]),
             static_cast<unsigned>(updated.max_us[param.channel_index]),
             static_cast<int>(updated.rev[param.channel_index]));
    return SetResult::kInvalidResultingConfig;
  }

  fc_config_->WriteRcCalibration(updated);
  return SetResult::kAccepted;
}

std::optional<FcConfigCache::Record> MavlinkParamServer::RecordFor(
    const ParamRef &param) const {
  const auto *fixed = std::get_if<FixedParamRef>(&param);
  if (fixed == nullptr) {
    return FcConfigCache::Record::kRcCalibration;
  }

  const param_detail::ParamDef &def =
      param_detail::kParamTable[fixed->mavlink_index];
  switch (def.key) {
    case param_detail::ParamKey::kCalAcc0Id:
    case param_detail::ParamKey::kCalGyro0Id:
      return FcConfigCache::Record::kGyroCalibrationId;
    case param_detail::ParamKey::kRcMapRoll:
    case param_detail::ParamKey::kRcMapPitch:
    case param_detail::ParamKey::kRcMapYaw:
    case param_detail::ParamKey::kRcMapThrottle:
      return FcConfigCache::Record::kRcMap;
    default:
      return std::nullopt;
  }
}

// Mirrors ParameterManager::_tryCacheHashLoad: CRC-32 over each parameter's
// name bytes then its value bytes, walked in name order, seeded at zero and
// left uncomplemented. A hash that disagrees costs only the full download QGC
// would have done anyway, so a mismatch degrades rather than breaks.
uint32_t MavlinkParamServer::ComputeParamHash() const {
  uint32_t crc = 0;
  const char *previous = nullptr;

  for (uint16_t emitted = 0; emitted < param_detail::kTotalParamCount;
       ++emitted) {
    // Selection over the table rather than a sorted copy: QGC walks a QMap,
    // which is ordered by name, and nothing here may allocate.
    const char *best_id = nullptr;
    float best_value = 0.0F;
    uint8_t best_type = 0;

    for (uint16_t i = 0; i < param_detail::kTotalParamCount; ++i) {
      const std::optional<ParamRef> param = TryResolveParamByIndex(i);
      if (!param.has_value()) {
        continue;
      }
      const std::optional<EncodedParam> encoded = TryEncodeParam(*param);
      if (!encoded.has_value()) {
        continue;
      }
      if (const auto *fixed = std::get_if<FixedParamRef>(&*param);
          fixed != nullptr &&
          param_detail::kParamTable[fixed->mavlink_index].px4_volatile) {
        continue;
      }
      if (previous != nullptr && std::strcmp(encoded->id, previous) <= 0) {
        continue;
      }
      if (best_id != nullptr && std::strcmp(encoded->id, best_id) >= 0) {
        continue;
      }
      best_id = encoded->id;
      best_value = encoded->value;
      best_type = encoded->type;
    }

    if (best_id == nullptr) {
      break;
    }

    const auto *const name_bytes = reinterpret_cast<const uint8_t *>(best_id);
    crc = checksum::Crc32Update(
        crc, std::span<const uint8_t>(name_bytes, std::strlen(best_id)));
    uint8_t bytes[4] = {};
    const size_t width =
        param_detail::ParamValueBytes(best_value, best_type, bytes);
    crc = checksum::Crc32Update(crc, std::span<const uint8_t>(bytes, width));
    previous = best_id;
  }

  return crc;
}

void MavlinkParamServer::QueueReply(uint16_t param_index) {
  if (!reply_queue_.Push(param_index)) {
    ESP_LOGW(kTag, "dropping PARAM_VALUE reply: queue full");
  }
}

std::optional<mavlink_message_t> MavlinkParamServer::NextQueuedMessage(
    uint32_t now_ms) {
  uint16_t param_index = 0;
  if (!reply_queue_.Peek(param_index)) {
    return std::nullopt;
  }

  if (param_index == param_detail::kHashCheckQueueIndex) {
    mavlink_param_union_t value{};
    value.type = MAV_PARAM_TYPE_UINT32;
    value.param_uint32 = ComputeParamHash();

    mavlink_message_t m{};
    // count 0 and index -1, as PX4 sends it: the parameter is virtual, and
    // QGC returns before either reaches its bookkeeping.
    mavlink_msg_param_value_pack(
        cfg_->sysid, kMavlinkComponentId, &m, param_detail::kHashCheckParamId,
        value.param_float, MAV_PARAM_TYPE_UINT32, 0, UINT16_MAX);
    (void)reply_queue_.Pop(param_index);
    return m;
  }

  const std::optional<ParamRef> param = TryResolveParamByIndex(param_index);
  if (!param.has_value()) {
    ESP_LOGW(kTag, "dropping queued PARAM_VALUE index=%u",
             static_cast<unsigned>(param_index));
    (void)reply_queue_.Pop(param_index);
    return std::nullopt;
  }

  const std::optional<FcConfigCache::Record> record = RecordFor(*param);
  if (record.has_value() && !fc_config_->Available(*record, now_ms)) {
    return std::nullopt;
  }

  const std::optional<mavlink_message_t> m = PackParamValue(*param);
  if (!m.has_value()) {
    return std::nullopt;
  }

  (void)reply_queue_.Pop(param_index);
  return m;
}

std::optional<mavlink_message_t> MavlinkParamServer::NextStreamMessage(
    uint32_t now_ms) {
  auto *stream = std::get_if<StreamActive>(&stream_);
  if (stream == nullptr) {
    return std::nullopt;
  }

  if (stream->next_param_index >= param_detail::kTotalParamCount) {
    stream_ = StreamIdle{};
    return std::nullopt;
  }

  const std::optional<ParamRef> param =
      TryResolveParamByIndex(stream->next_param_index);
  if (!param.has_value()) {
    ESP_LOGW(kTag, "stopping PARAM stream at index=%u",
             static_cast<unsigned>(stream->next_param_index));
    stream_ = StreamIdle{};
    return std::nullopt;
  }

  const std::optional<FcConfigCache::Record> record = RecordFor(*param);
  if (record.has_value() && !fc_config_->Available(*record, now_ms)) {
    return std::nullopt;
  }

  const std::optional<mavlink_message_t> m = PackParamValue(*param);
  if (!m.has_value()) {
    return std::nullopt;
  }

  stream->next_param_index++;
  if (stream->next_param_index >= param_detail::kTotalParamCount) {
    stream_ = StreamIdle{};
  }
  return m;
}

std::optional<mavlink_message_t> MavlinkParamServer::PackParamValue(
    const ParamRef &param) const {
  const std::optional<EncodedParam> encoded = TryEncodeParam(param);
  if (!encoded.has_value()) {
    return std::nullopt;
  }

  mavlink_message_t m{};
  mavlink_msg_param_value_pack(
      cfg_->sysid, kMavlinkComponentId, &m, encoded->id,
      param_detail::EncodeParamValue(encoded->value, encoded->type),
      encoded->type, param_detail::kTotalParamCount, ParamMavlinkIndex(param));
  return m;
}

std::optional<mavlink_message_t> MavlinkParamServer::NextMessage(
    uint32_t now_ms) {
  if (const std::optional<mavlink_message_t> queued =
          NextQueuedMessage(now_ms)) {
    return queued;
  }
  return NextStreamMessage(now_ms);
}
