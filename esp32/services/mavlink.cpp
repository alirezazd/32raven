#include "mavlink.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "../../third_party/mavlink/standard/mavlink_msg_autopilot_version.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "panic.hpp"
#include "system.hpp"

static constexpr const char *kTag = "mavlink";
portMUX_TYPE g_mavlink_cache_lock = portMUX_INITIALIZER_UNLOCKED;

namespace {

namespace mi {

inline constexpr uint64_t kMavProtocolCapabilityParamFloat = 1ull << 1;
inline constexpr uint64_t kMavProtocolCapabilityMavlink2 = 1ull << 13;
inline constexpr uint64_t kMavProtocolCapabilityParamEncodeCCast = 1ull << 17;

inline void CopyMavParamId(const char *src, char (&dst)[17]) {
  std::memcpy(dst, src, 16);
  dst[16] = '\0';
}

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
  kHeartbeatMs,
  kGpsMs,
  kAttMs,
  kGposMs,
  kBattMs,
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
    {"MAV_HB_MS", MAV_PARAM_TYPE_UINT16, ParamKey::kHeartbeatMs},
    {"MAV_GPS_MS", MAV_PARAM_TYPE_UINT16, ParamKey::kGpsMs},
    {"MAV_ATT_MS", MAV_PARAM_TYPE_UINT16, ParamKey::kAttMs},
    {"MAV_GPOS_MS", MAV_PARAM_TYPE_UINT16, ParamKey::kGposMs},
    {"MAV_BATT_MS", MAV_PARAM_TYPE_UINT16, ParamKey::kBattMs},
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

inline bool ParamIdEquals(const char *lhs, const char *rhs) {
  return std::strncmp(lhs, rhs, 16) == 0;
}

inline bool TryParseRcCalibrationParamId(const char *param_id,
                                         uint8_t &channel_index,
                                         RcCalibrationField &field) {
  unsigned channel = 0;
  char suffix[6] = {};
  if (std::sscanf(param_id, "RC%u_%5s", &channel, suffix) != 2) {
    return false;
  }
  if (channel < 1u || channel > message::kRcCalibrationChannelCount) {
    return false;
  }

  if (std::strcmp(suffix, "MIN") == 0) {
    field = RcCalibrationField::kMin;
  } else if (std::strcmp(suffix, "MAX") == 0) {
    field = RcCalibrationField::kMax;
  } else if (std::strcmp(suffix, "TRIM") == 0) {
    field = RcCalibrationField::kTrim;
  } else if (std::strcmp(suffix, "REV") == 0) {
    field = RcCalibrationField::kRev;
  } else {
    return false;
  }

  channel_index = static_cast<uint8_t>(channel - 1u);
  return true;
}

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

}  // namespace mi

constexpr uint16_t kSysStatusStartDelayMs = 250;
constexpr MAV_AUTOPILOT kMavAutopilot32Raven = static_cast<MAV_AUTOPILOT>(200);
constexpr uint16_t kSysStatusMs = 1000;
constexpr uint16_t kFcConfigRequestAttempts = 100;
constexpr uint16_t kFcConfigRequestRetryPeriodMs = 50;
constexpr TickType_t kMavlinkWorkerPeriodTicks = pdMS_TO_TICKS(10);
constexpr UBaseType_t kMavlinkTaskPrio = 10;
constexpr uint32_t kMavlinkTaskStackDepthWords =
    static_cast<uint32_t>(esp32_limits::kMavlinkTaskStackDepthWords);

StaticTask_t s_mavlink_task_buffer;
StackType_t s_mavlink_task_stack[kMavlinkTaskStackDepthWords];
TaskHandle_t s_mavlink_task_handle = nullptr;

int64_t DaysFromCivil(int64_t year, unsigned month, unsigned day) {
  year -= month <= 2 ? 1 : 0;
  const int64_t era = (year >= 0 ? year : year - 399) / 400;
  const unsigned yoe = (unsigned)(year - era * 400);
  const unsigned doy =
      (153u * (month + (month > 2 ? (unsigned)-3 : 9u)) + 2u) / 5u + day - 1u;
  const unsigned doe = yoe * 365u + yoe / 4u - yoe / 100u + doy;
  return era * 146097 + (int64_t)doe - 719468;
}

bool TryBuildGpsUnixUsec(const message::GpsData &gps, uint64_t &unix_usec) {
  if (gps.year < 1970 || gps.month < 1 || gps.month > 12 || gps.day < 1 ||
      gps.day > 31 || gps.hour > 23 || gps.min > 59 || gps.sec > 59) {
    return false;
  }

  const int64_t days = DaysFromCivil(gps.year, gps.month, gps.day);
  if (days < 0) {
    return false;
  }

  const int64_t seconds = days * 86400 + (int64_t)gps.hour * 3600 +
                          (int64_t)gps.min * 60 + (int64_t)gps.sec;
  if (seconds < 0) {
    return false;
  }

  unix_usec = (uint64_t)seconds * 1000000ull;
  return true;
}

void MavlinkTaskEntry(void *arg) {
  auto *mavlink = static_cast<Mavlink *>(arg);
  TickType_t last_wake = xTaskGetTickCount();

  while (true) {
    mavlink->WorkerTick(Sys().Timebase().NowMs());
    vTaskDelayUntil(&last_wake, kMavlinkWorkerPeriodTicks);

    const size_t stack_high_water_bytes =
        uxTaskGetStackHighWaterMark(nullptr) * sizeof(StackType_t);
    if (stack_high_water_bytes <
        esp32_limits::kMavlinkStackPanicThresholdBytes) {
      Panic(ErrorCode::kMavlinkStackOverflow);
    }
  }
}

template <typename MatchHandler>
bool ConsumeFcLinkPackets(uint8_t match_id, MatchHandler &&handle_match) {
  const size_t packet_count = Sys().FcLink().PendingRxPacketCount();
  bool matched = false;

  for (size_t packet_idx = 0; packet_idx < packet_count; ++packet_idx) {
    auto packet = Sys().FcLink().PopPacket();
    if (!packet.has_value()) {
      Panic(ErrorCode::kFcLinkRxQueueFull);
    }

    if (packet->header.id == match_id) {
      matched = handle_match(*packet) || matched;
      continue;
    }

    Sys().FcLink().QueueRxPacket(*packet);
  }

  return matched;
}

}  // namespace

Mavlink::Mavlink() {}
Mavlink::~Mavlink() {}

Mavlink &Mavlink::GetInstance() {
  static Mavlink instance;
  return instance;
}

void Mavlink::ArmFirstSchedule(TxState &tx, const TxConfig &cfg_tx,
                               uint32_t now_ms) {
  tx.next_hb_ms = now_ms;
  tx.next_sys_ms = now_ms + kSysStatusStartDelayMs;
  tx.next_gps_ms = now_ms + cfg_tx.schedule.gps_start_delay_ms;
  tx.next_att_ms = now_ms + cfg_tx.schedule.att_start_delay_ms;
  tx.next_gpos_ms = now_ms + cfg_tx.schedule.gpos_start_delay_ms;
  tx.next_batt_ms = now_ms + cfg_tx.schedule.batt_start_delay_ms;
}

bool Mavlink::ShouldSendHbNow(const TxState &tx, const TxConfig &cfg_tx,
                              uint32_t now_ms) const {
  if (cfg_tx.periods.hb_ms == 0 || cfg_tx.schedule.hb_deadline_ms == 0) {
    return false;
  }
  int32_t since_done = (int32_t)(now_ms - tx.last_hb_done_ms);
  if (since_done >= (int32_t)cfg_tx.schedule.hb_deadline_ms) {
    return true;
  }
  return (int32_t)(now_ms - tx.next_hb_ms) >= 0;
}

void Mavlink::Init(const Config &cfg, UdpServer *udp) {
  if (udp == nullptr) {
    Panic(ErrorCode::kMavlinkInitFailed);
  }
  if (cfg.identity.sysid == 0 || cfg.tx.periods.hb_ms == 0 ||
      cfg.tx.schedule.hb_deadline_ms == 0) {
    Panic(ErrorCode::kMavlinkInitFailed);
  }

  cfg_ = cfg;
  udp_ = udp;
  udp_tx_ = TxState{};

  have_latest_ = false;
  latest_update_ms_ = 0;
  latest_rc_channels_ = {};
  have_latest_rc_channels_ = false;
  latest_rc_channels_update_ms_ = 0;
  rc_map_config_ = {};
  have_rc_map_config_ = false;
  rc_calibration_config_ = {};
  have_rc_calibration_config_ = false;
  gyro_calibration_id_config_ = {};
  have_gyro_calibration_id_config_ = false;
  unhandled_logged_msgid_count_ = 0;
  rc_chan_count_ = message::kRcCalibrationChannelCount;
  rc_config_confirm_pending_ = false;
  rc_config_confirm_due_ms_ = 0;
  pending_rc_map_request_ = PendingFcConfigRequest{};
  pending_rc_calibration_request_ = PendingFcConfigRequest{};
  pending_gyro_calibration_id_request_ = PendingFcConfigRequest{};
  primary_link_enabled_.store(false, std::memory_order_relaxed);
  pending_primary_link_heartbeat_led_pulse_.store(false,
                                                  std::memory_order_relaxed);
  primary_link_heartbeat_led_clear_ms_ = 0;
  ArmFirstSchedule(udp_tx_, cfg_.tx, 0);

  ESP_LOGI(kTag, "Initialized (primary MAVLink UDP link)");

  StartFcConfigRequest(message::MsgId::kReqRcMap,
                       ErrorCode::kFcLinkRcMapRequestFailed);
  StartFcConfigRequest(message::MsgId::kReqRcCalibration,
                       ErrorCode::kFcLinkRcCalibrationRequestFailed);
  StartFcConfigRequest(message::MsgId::kReqGyroCalibrationId,
                       ErrorCode::kFcLinkGyroCalibrationIdRequestFailed);

  StartMavlinkTask();
}

void Mavlink::StartMavlinkTask() {
  if (s_mavlink_task_handle != nullptr) {
    Panic(ErrorCode::kMavlinkTaskAlreadyRunning);
  }

  s_mavlink_task_handle = xTaskCreateStaticPinnedToCore(
      MavlinkTaskEntry, "mavlink", kMavlinkTaskStackDepthWords, this,
      kMavlinkTaskPrio, s_mavlink_task_stack, &s_mavlink_task_buffer, 0);
  if (s_mavlink_task_handle == nullptr) {
    Panic(ErrorCode::kMavlinkInitFailed);
  }
}

void Mavlink::StopMavlinkTask() {
  if (s_mavlink_task_handle == nullptr) {
    return;
  }

  if (s_mavlink_task_handle == xTaskGetCurrentTaskHandle()) {
    return;
  }

  vTaskSuspend(s_mavlink_task_handle);
}

void Mavlink::Poll() {
  static constexpr uint32_t kPrimaryLinkHeartbeatLedPatternMs = 300;
  const uint32_t now_ms = Sys().Timebase().NowMs();

  if (pending_primary_link_heartbeat_led_pulse_.exchange(
          false, std::memory_order_relaxed)) {
    Sys().Led().SetPattern(LED::Pattern::kDoubleBlink,
                           kPrimaryLinkHeartbeatLedPatternMs);
    primary_link_heartbeat_led_clear_ms_ =
        now_ms + kPrimaryLinkHeartbeatLedPatternMs;
  } else if (primary_link_heartbeat_led_clear_ms_ != 0 &&
             static_cast<int32_t>(now_ms -
                                  primary_link_heartbeat_led_clear_ms_) >= 0) {
    Sys().Led().Off();
    primary_link_heartbeat_led_clear_ms_ = 0;
  }
}

void Mavlink::SetPrimaryLinkEnabled(bool enabled) {
  primary_link_enabled_.store(enabled, std::memory_order_relaxed);
  pending_primary_link_heartbeat_led_pulse_.store(false,
                                                  std::memory_order_relaxed);
  primary_link_heartbeat_led_clear_ms_ = 0;
  if (enabled) {
    Sys().Led().Off();
  }
  if (!enabled && udp_ != nullptr) {
    udp_->ClearPeer();
  }
}

void Mavlink::StartFcConfigRequest(message::MsgId request_id,
                                   ErrorCode failure_code) {
  PendingFcConfigRequest *request = nullptr;
  const char *request_log = nullptr;

  switch (request_id) {
    case message::MsgId::kReqRcMap:
      ClearRcMapConfig();
      request = &pending_rc_map_request_;
      request_log = "Requesting STM32 RC map...";
      break;
    case message::MsgId::kReqRcCalibration:
      ClearRcCalibrationConfig();
      request = &pending_rc_calibration_request_;
      request_log = "Requesting STM32 RC calibration...";
      break;
    case message::MsgId::kReqGyroCalibrationId:
      ClearGyroCalibrationIdConfig();
      request = &pending_gyro_calibration_id_request_;
      request_log = "Requesting STM32 gyro calibration ID...";
      break;
    default:
      Panic(ErrorCode::kMavlinkInitFailed);
      return;
  }

  ESP_LOGI(kTag, "%s", request_log);
  request->request_id = request_id;
  request->failure_code = failure_code;
  request->next_retry_ms = 0;
  request->attempts_sent = 0;
  request->active = true;
  ServiceFcConfigRequests(Sys().Timebase().NowMs());
}

void Mavlink::ServiceFcConfigRequests(uint32_t now_ms) {
  const auto service_request = [&](PendingFcConfigRequest &request,
                                   bool satisfied, const char *success_log,
                                   const char *failure_log) {
    if (!request.active) {
      return;
    }

    if (satisfied) {
      request.active = false;
      ESP_LOGI(kTag, "%s", success_log);
      return;
    }

    if ((int32_t)(now_ms - request.next_retry_ms) < 0) {
      return;
    }

    if (request.attempts_sent >= kFcConfigRequestAttempts) {
      ESP_LOGW(kTag, "%s", failure_log);
      Panic(request.failure_code);
    }

    message::Packet req_pkt{};
    req_pkt.header.id = static_cast<uint8_t>(request.request_id);
    req_pkt.header.len = 0;
    Sys().FcLink().SendPacket(req_pkt);
    ++request.attempts_sent;
    request.next_retry_ms = now_ms + kFcConfigRequestRetryPeriodMs;
  };

  service_request(pending_rc_map_request_, have_rc_map_config_,
                  "STM32 RC map received", "STM32 RC map request failed");
  service_request(pending_rc_calibration_request_, have_rc_calibration_config_,
                  "STM32 RC calibration received",
                  "STM32 RC calibration request failed");
  service_request(pending_gyro_calibration_id_request_,
                  have_gyro_calibration_id_config_,
                  "STM32 gyro calibration ID received",
                  "STM32 gyro calibration ID request failed");
}

uint32_t Mavlink::UdpRxPacketCount() const {
  return udp_rx_packet_count_.load(std::memory_order_relaxed);
}

uint32_t Mavlink::UdpTxPacketCount() const {
  return udp_tx_packet_count_.load(std::memory_order_relaxed);
}

void Mavlink::OfferTelemetry(const message::GpsData &d, uint32_t now_ms) {
  latest_ = d;
  have_latest_ = true;
  latest_update_ms_ = now_ms;
}

void Mavlink::OfferRcChannels(const message::RcChannelsMsg &msg,
                              uint32_t now_ms) {
  taskENTER_CRITICAL(&g_mavlink_cache_lock);
  latest_rc_channels_ = msg;
  have_latest_rc_channels_ = true;
  latest_rc_channels_update_ms_ = now_ms;
  taskEXIT_CRITICAL(&g_mavlink_cache_lock);
}

Mavlink::RcChannelsSnapshot Mavlink::GetRcChannelsSnapshot(
    uint32_t now_ms) const {
  RcChannelsSnapshot snapshot{};
  taskENTER_CRITICAL(&g_mavlink_cache_lock);
  snapshot.msg = latest_rc_channels_;
  snapshot.update_ms = latest_rc_channels_update_ms_;
  snapshot.have_data = have_latest_rc_channels_;
  taskEXIT_CRITICAL(&g_mavlink_cache_lock);

  if (snapshot.have_data) {
    snapshot.age_ms = now_ms - snapshot.update_ms;
    snapshot.live = snapshot.age_ms <= 500u;
  }
  return snapshot;
}

void Mavlink::SetRcMapConfig(const message::RcMapConfigMsg &cfg) {
  rc_map_config_ = cfg;
  have_rc_map_config_ = true;
}

void Mavlink::SetRcCalibrationConfig(
    const message::RcCalibrationConfigMsg &cfg) {
  rc_calibration_config_ = cfg;
  have_rc_calibration_config_ = true;
}

void Mavlink::SetGyroCalibrationIdConfig(
    const message::GyroCalibrationIdConfigMsg &cfg) {
  gyro_calibration_id_config_ = cfg;
  have_gyro_calibration_id_config_ = true;
}

void Mavlink::ClearRcMapConfig() {
  rc_map_config_ = {};
  have_rc_map_config_ = false;
}

void Mavlink::ClearRcCalibrationConfig() {
  rc_calibration_config_ = {};
  have_rc_calibration_config_ = false;
}

void Mavlink::ClearGyroCalibrationIdConfig() {
  gyro_calibration_id_config_ = {};
  have_gyro_calibration_id_config_ = false;
}

bool Mavlink::ApplyRcMapConfigToFcLink(const message::RcMapConfigMsg &cfg) {
  if (!message::IsRcMapConfigValid(cfg)) {
    ESP_LOGW(kTag, "Rejecting invalid STM32 RC map set request");
    return false;
  }

  message::Packet req_pkt{};
  req_pkt.header.id = static_cast<uint8_t>(message::MsgId::kSetRcMapConfig);
  req_pkt.header.len = message::PayloadLength<message::RcMapConfigMsg>();
  std::memcpy(req_pkt.payload, &cfg, sizeof(cfg));

  const auto drain_stale_responses = [&]() {
    Sys().FcLink().Poll();
    (void)ConsumeFcLinkPackets(
        static_cast<uint8_t>(message::MsgId::kRcMapConfig),
        [&](const message::Packet &response) {
          if (!message::IsPayloadLengthValid(message::MsgId::kRcMapConfig,
                                             response.header.len)) {
            Panic(ErrorCode::kFcLinkInvalidPacketLength);
          }

          const auto *applied =
              reinterpret_cast<const message::RcMapConfigMsg *>(
                  response.payload);
          if (!message::IsRcMapConfigValid(*applied)) {
            Panic(ErrorCode::kFcLinkInvalidRcMapConfig);
          }

          SetRcMapConfig(*applied);
          return false;
        });
  };

  drain_stale_responses();

  for (uint16_t i = 0; i < kFcConfigRequestAttempts; ++i) {
    Sys().FcLink().SendPacket(req_pkt);
    const TickType_t deadline =
        xTaskGetTickCount() + pdMS_TO_TICKS(kFcConfigRequestRetryPeriodMs);

    while (xTaskGetTickCount() < deadline) {
      vTaskDelay(pdMS_TO_TICKS(10));
      Sys().FcLink().Poll();

      if (ConsumeFcLinkPackets(
              static_cast<uint8_t>(message::MsgId::kRcMapConfig),
              [&](const message::Packet &response) {
                if (!message::IsPayloadLengthValid(message::MsgId::kRcMapConfig,
                                                   response.header.len)) {
                  Panic(ErrorCode::kFcLinkInvalidPacketLength);
                }

                const auto *applied =
                    reinterpret_cast<const message::RcMapConfigMsg *>(
                        response.payload);
                if (!message::IsRcMapConfigValid(*applied)) {
                  Panic(ErrorCode::kFcLinkInvalidRcMapConfig);
                }

                SetRcMapConfig(*applied);
                return std::memcmp(applied, &cfg, sizeof(cfg)) == 0;
              })) {
        return true;
      }
    }

    drain_stale_responses();
  }

  Panic(ErrorCode::kFcLinkRcMapSetFailed);
}

bool Mavlink::ApplyRcCalibrationConfigToFcLink(
    const message::RcCalibrationConfigMsg &cfg) {
  if (!message::IsRcCalibrationConfigValid(cfg)) {
    ESP_LOGW(kTag, "Rejecting invalid STM32 RC calibration set request");
    return false;
  }

  message::Packet req_pkt{};
  req_pkt.header.id =
      static_cast<uint8_t>(message::MsgId::kSetRcCalibrationConfig);
  req_pkt.header.len =
      message::PayloadLength<message::RcCalibrationConfigMsg>();
  std::memcpy(req_pkt.payload, &cfg, sizeof(cfg));

  const auto drain_stale_responses = [&]() {
    Sys().FcLink().Poll();
    (void)ConsumeFcLinkPackets(
        static_cast<uint8_t>(message::MsgId::kRcCalibrationConfig),
        [&](const message::Packet &response) {
          if (!message::IsPayloadLengthValid(
                  message::MsgId::kRcCalibrationConfig, response.header.len)) {
            Panic(ErrorCode::kFcLinkInvalidPacketLength);
          }

          const auto *applied =
              reinterpret_cast<const message::RcCalibrationConfigMsg *>(
                  response.payload);
          if (!message::IsRcCalibrationConfigValid(*applied)) {
            Panic(ErrorCode::kFcLinkInvalidRcCalibrationConfig);
          }

          SetRcCalibrationConfig(*applied);
          return false;
        });
  };

  drain_stale_responses();

  for (uint16_t i = 0; i < kFcConfigRequestAttempts; ++i) {
    Sys().FcLink().SendPacket(req_pkt);
    const TickType_t deadline =
        xTaskGetTickCount() + pdMS_TO_TICKS(kFcConfigRequestRetryPeriodMs);

    while (xTaskGetTickCount() < deadline) {
      vTaskDelay(pdMS_TO_TICKS(10));
      Sys().FcLink().Poll();

      if (ConsumeFcLinkPackets(
              static_cast<uint8_t>(message::MsgId::kRcCalibrationConfig),
              [&](const message::Packet &response) {
                if (!message::IsPayloadLengthValid(
                        message::MsgId::kRcCalibrationConfig,
                        response.header.len)) {
                  Panic(ErrorCode::kFcLinkInvalidPacketLength);
                }

                const auto *applied =
                    reinterpret_cast<const message::RcCalibrationConfigMsg *>(
                        response.payload);
                if (!message::IsRcCalibrationConfigValid(*applied)) {
                  Panic(ErrorCode::kFcLinkInvalidRcCalibrationConfig);
                }

                SetRcCalibrationConfig(*applied);
                return std::memcmp(applied, &cfg, sizeof(cfg)) == 0;
              })) {
        return true;
      }
    }

    drain_stale_responses();
  }

  Panic(ErrorCode::kFcLinkRcCalibrationSetFailed);
}

void Mavlink::QueueStatusText(const char *text, uint8_t severity) {
  if (text == nullptr || text[0] == '\0') {
    return;
  }

  auto queue = [&](TxState &tx) {
    tx.pending_statustext = true;
    tx.pending_statustext_severity = severity;
    std::strncpy(tx.pending_statustext_text, text,
                 sizeof(tx.pending_statustext_text) - 1);
    tx.pending_statustext_text[sizeof(tx.pending_statustext_text) - 1] = '\0';
  };

  queue(udp_tx_);
}

void Mavlink::WorkerTick(uint32_t now_ms) {
  ServiceFcConfigRequests(now_ms);
  ServicePrimaryLink(now_ms);
  ServiceRcConfigConfirm(now_ms);
}

void Mavlink::ScheduleRcConfigConfirm(uint32_t now_ms) {
  static constexpr uint32_t kRcConfigConfirmDelayMs = 1000;
  rc_config_confirm_pending_ = true;
  rc_config_confirm_due_ms_ = now_ms + kRcConfigConfirmDelayMs;
}

void Mavlink::ServiceRcConfigConfirm(uint32_t now_ms) {
  if (!rc_config_confirm_pending_) {
    return;
  }
  if ((int32_t)(now_ms - rc_config_confirm_due_ms_) < 0) {
    return;
  }

  rc_config_confirm_pending_ = false;
  Sys().TonePlayer().PlayBuiltin(::TonePlayer::BuiltinTone::kConfirm);
}

void Mavlink::HandleMessage(const mavlink_message_t &msg) {
  TxState &tx = udp_tx_;

  switch (msg.msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT: {
      if (primary_link_enabled_.load(std::memory_order_relaxed)) {
        pending_primary_link_heartbeat_led_pulse_.store(
            true, std::memory_order_relaxed);
      }
      break;
    }
    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
      mavlink_param_request_list_t req{};
      mavlink_msg_param_request_list_decode(&msg, &req);
      if (req.target_system == cfg_.identity.sysid &&
          (req.target_component == cfg_.identity.compid ||
           req.target_component == 0 ||
           req.target_component == MAV_COMP_ID_ALL)) {
        tx.param_stream_active = true;
        tx.next_param_index = 0;
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

      uint16_t param_index = 0;
      if (TryResolveParamIndex(req.param_index, req.param_id, param_index)) {
        QueueParamValue(param_index);
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

      char param_id[17] = {};
      mi::CopyMavParamId(req.param_id, param_id);

      uint16_t param_index = 0;
      if (!TryResolveParamIndex(-1, req.param_id, param_index)) {
        ESP_LOGW(kTag, "PARAM_SET unresolved param_id=%s", param_id);
        break;
      }

      const float decoded_value =
          mi::DecodeParamValue(req.param_value, req.param_type);
      (void)TrySetParamByIndex(param_index, decoded_value, req.param_type);
      QueueParamValue(param_index);
      break;
    }
    case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST: {
      ESP_LOGW(kTag, "PARAM_EXT_REQUEST_LIST unsupported");
      break;
    }
    case MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ: {
      ESP_LOGW(kTag, "PARAM_EXT_REQUEST_READ unsupported");
      break;
    }
    case MAVLINK_MSG_ID_PARAM_EXT_SET: {
      mavlink_param_ext_set_t req{};
      mavlink_msg_param_ext_set_decode(&msg, &req);

      char param_id[17] = {};
      char param_value[129] = {};
      mi::CopyMavTextField(req.param_id, sizeof(req.param_id), param_id,
                           sizeof(param_id));
      mi::CopyMavTextField(req.param_value, sizeof(req.param_value),
                           param_value, sizeof(param_value));

      ESP_LOGW(kTag,
               "PARAM_EXT_SET unsupported target_sys=%u target_comp=%u "
               "param_id=%s value=%s type=%u",
               (unsigned)req.target_system, (unsigned)req.target_component,
               param_id, param_value, (unsigned)req.param_type);
      break;
    }
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
      mavlink_mission_request_list_t req{};
      mavlink_msg_mission_request_list_decode(&msg, &req);
      if (req.target_system == cfg_.identity.sysid &&
          (req.target_component == cfg_.identity.compid ||
           req.target_component == 0 ||
           req.target_component == MAV_COMP_ID_ALL)) {
        tx.pending_mission_count = true;
        tx.pending_mission_type = req.mission_type;
      }
      break;
    }
    case MAVLINK_MSG_ID_MISSION_ACK: {
      break;
    }
    case MAVLINK_MSG_ID_SYSTEM_TIME: {
      break;
    }
    case MAVLINK_MSG_ID_COMMAND_LONG: {
      mavlink_command_long_t cmd{};
      mavlink_msg_command_long_decode(&msg, &cmd);
      if (cmd.target_system != cfg_.identity.sysid ||
          (cmd.target_component != cfg_.identity.compid &&
           cmd.target_component != 0 &&
           cmd.target_component != MAV_COMP_ID_ALL)) {
        break;
      }
      HandleCommandLong(msg, cmd);
      break;
    }

    default:
      LogUnhandledMessageOnce(msg);
      break;
  }
}

void Mavlink::HandleCommandLong(const mavlink_message_t &msg,
                                const mavlink_command_long_t &cmd) {
  const uint8_t source_system = msg.sysid;
  const uint8_t source_component = msg.compid;

  switch (cmd.command) {
    case MAV_CMD_REQUEST_MESSAGE:
      if ((uint32_t)cmd.param1 == MAVLINK_MSG_ID_AUTOPILOT_VERSION) {
        QueueCommandAck((uint16_t)cmd.command, MAV_RESULT_ACCEPTED,
                        source_system, source_component);
        QueueAutopilotVersion();
      } else {
        QueueCommandAck((uint16_t)cmd.command, MAV_RESULT_UNSUPPORTED,
                        source_system, source_component);
      }
      break;
    case MAV_CMD_PREFLIGHT_CALIBRATION:
      QueueCommandAck((uint16_t)cmd.command, MAV_RESULT_ACCEPTED, source_system,
                      source_component);
      break;
    default:
      QueueCommandAck((uint16_t)cmd.command, MAV_RESULT_UNSUPPORTED,
                      source_system, source_component);
      break;
  }
}

void Mavlink::LogUnhandledMessageOnce(const mavlink_message_t &msg) {
  for (uint8_t i = 0; i < unhandled_logged_msgid_count_; ++i) {
    if (unhandled_logged_msgids_[i] == msg.msgid) {
      return;
    }
  }

  if (unhandled_logged_msgid_count_ < unhandled_logged_msgids_.size()) {
    unhandled_logged_msgids_[unhandled_logged_msgid_count_++] =
        static_cast<uint16_t>(msg.msgid);
  }

  char text[51] = {};
  std::snprintf(text, sizeof(text), "Unhandled MAVLink msgid=%lu src=UDP",
                (unsigned long)msg.msgid);
  QueueStatusText(text, MAV_SEVERITY_WARNING);
}

bool Mavlink::TryResolveParamIndex(int16_t requested_index,
                                   const char *requested_id,
                                   uint16_t &resolved_index) const {
  if (requested_index >= 0) {
    const uint16_t index = static_cast<uint16_t>(requested_index);
    if (index < mi::kTotalParamCount) {
      resolved_index = index;
      return true;
    }
    return false;
  }

  if (requested_id == nullptr) {
    return false;
  }

  char param_id[17] = {};
  mi::CopyMavParamId(requested_id, param_id);

  for (uint16_t i = 0; i < mi::kBaseParamCount; ++i) {
    if (mi::ParamIdEquals(param_id, mi::kParamTable[i].id)) {
      resolved_index = i;
      return true;
    }
  }

  uint8_t channel_index = 0;
  mi::RcCalibrationField field = mi::RcCalibrationField::kMin;
  if (!mi::TryParseRcCalibrationParamId(param_id, channel_index, field)) {
    return false;
  }

  uint16_t field_offset = 0;
  switch (field) {
    case mi::RcCalibrationField::kMin:
      field_offset = 0;
      break;
    case mi::RcCalibrationField::kMax:
      field_offset = 1;
      break;
    case mi::RcCalibrationField::kTrim:
      field_offset = 2;
      break;
    case mi::RcCalibrationField::kRev:
      field_offset = 3;
      break;
  }

  resolved_index = static_cast<uint16_t>(
      mi::kBaseParamCount +
      channel_index * mi::kRcCalibrationParamCountPerChannel + field_offset);
  return true;
}

bool Mavlink::TryEncodeParamByIndex(uint16_t param_index, char (&param_id)[17],
                                    uint8_t &param_type,
                                    float &param_value) const {
  if (param_index < mi::kBaseParamCount) {
    const mi::ParamDef &def = mi::kParamTable[param_index];
    param_type = def.type;
    std::strncpy(param_id, def.id, 16);
    switch (def.key) {
      case mi::ParamKey::kSysId:
      case mi::ParamKey::kMavSysId:
        param_value = static_cast<float>(cfg_.identity.sysid);
        break;
      case mi::ParamKey::kCompId:
        param_value = static_cast<float>(cfg_.identity.compid);
        break;
      case mi::ParamKey::kCalAcc0Id:
      case mi::ParamKey::kCalGyro0Id:
        param_value =
            static_cast<float>(gyro_calibration_id_config_.cal_gyro0_id);
        break;
      case mi::ParamKey::kCalMag0Id:
      case mi::ParamKey::kCalMag1Id:
      case mi::ParamKey::kCalMag2Id:
        param_value = 0.0f;
        break;
      case mi::ParamKey::kCalMag0Rot:
      case mi::ParamKey::kCalMag1Rot:
      case mi::ParamKey::kCalMag2Rot:
        param_value = -1.0f;
        break;
      case mi::ParamKey::kSensBoardRot:
        param_value = 0.0f;
        break;
      case mi::ParamKey::kSensDpresOff:
        param_value = 0.0f;
        break;
      case mi::ParamKey::kSysHasMag:
        param_value = 0.0f;
        break;
      case mi::ParamKey::kSysHasNumAspd:
        param_value = 0.0f;
        break;
      case mi::ParamKey::kSysAutostart:
        param_value = static_cast<float>(esp32_limits::kMavlinkSysAutostart);
        break;
      case mi::ParamKey::kComRcInMode:
        param_value = 0.0f;
        break;
      case mi::ParamKey::kRcChanCnt:
        param_value = static_cast<float>(rc_chan_count_);
        break;
      case mi::ParamKey::kRcMapRoll:
        param_value = static_cast<float>(rc_map_config_.roll);
        break;
      case mi::ParamKey::kRcMapPitch:
        param_value = static_cast<float>(rc_map_config_.pitch);
        break;
      case mi::ParamKey::kRcMapYaw:
        param_value = static_cast<float>(rc_map_config_.yaw);
        break;
      case mi::ParamKey::kRcMapThrottle:
        param_value = static_cast<float>(rc_map_config_.throttle);
        break;
      case mi::ParamKey::kHeartbeatMs:
        param_value = static_cast<float>(cfg_.tx.periods.hb_ms);
        break;
      case mi::ParamKey::kGpsMs:
        param_value = static_cast<float>(cfg_.tx.periods.gps_ms);
        break;
      case mi::ParamKey::kAttMs:
        param_value = static_cast<float>(cfg_.tx.periods.att_ms);
        break;
      case mi::ParamKey::kGposMs:
        param_value = static_cast<float>(cfg_.tx.periods.gpos_ms);
        break;
      case mi::ParamKey::kBattMs:
        param_value = static_cast<float>(cfg_.tx.periods.batt_ms);
        break;
      case mi::ParamKey::kComRcLossT:
        param_value = 0.0f;
        break;
    }
    return true;
  }

  if (param_index >= mi::kTotalParamCount || !have_rc_calibration_config_) {
    return false;
  }

  const uint16_t rc_param_index = param_index - mi::kBaseParamCount;
  const uint8_t channel_index = static_cast<uint8_t>(
      rc_param_index / mi::kRcCalibrationParamCountPerChannel);
  const uint8_t field_index = static_cast<uint8_t>(
      rc_param_index % mi::kRcCalibrationParamCountPerChannel);

  switch (field_index) {
    case 0:
      std::snprintf(param_id, sizeof(param_id), "RC%u_MIN",
                    (unsigned)(channel_index + 1u));
      param_type = MAV_PARAM_TYPE_UINT16;
      param_value =
          static_cast<float>(rc_calibration_config_.min_us[channel_index]);
      break;
    case 1:
      std::snprintf(param_id, sizeof(param_id), "RC%u_MAX",
                    (unsigned)(channel_index + 1u));
      param_type = MAV_PARAM_TYPE_UINT16;
      param_value =
          static_cast<float>(rc_calibration_config_.max_us[channel_index]);
      break;
    case 2:
      std::snprintf(param_id, sizeof(param_id), "RC%u_TRIM",
                    (unsigned)(channel_index + 1u));
      param_type = MAV_PARAM_TYPE_UINT16;
      param_value =
          static_cast<float>(rc_calibration_config_.trim_us[channel_index]);
      break;
    case 3:
    default:
      std::snprintf(param_id, sizeof(param_id), "RC%u_REV",
                    (unsigned)(channel_index + 1u));
      param_type = MAV_PARAM_TYPE_INT8;
      param_value =
          static_cast<float>(rc_calibration_config_.rev[channel_index]);
      break;
  }
  return true;
}

bool Mavlink::TrySetParamByIndex(uint16_t param_index, float param_value,
                                 uint8_t param_type) {
  (void)param_type;
  if (param_index >= mi::kTotalParamCount) {
    ESP_LOGW(kTag, "PARAM_SET rejected: index=%u out of range",
             (unsigned)param_index);
    return false;
  }

  if (param_index < mi::kBaseParamCount) {
    const mi::ParamDef &def = mi::kParamTable[param_index];
    switch (def.key) {
      case mi::ParamKey::kRcChanCnt: {
        const long value = std::lround(param_value);
        if (value >= 1 &&
            value <= static_cast<long>(message::kRcCalibrationChannelCount)) {
          rc_chan_count_ = static_cast<uint8_t>(value);
          return true;
        }
        return false;
      }
      case mi::ParamKey::kRcMapRoll:
      case mi::ParamKey::kRcMapPitch:
      case mi::ParamKey::kRcMapYaw:
      case mi::ParamKey::kRcMapThrottle: {
        if (!have_rc_map_config_) {
          ESP_LOGW(kTag, "RC_MAP write rejected: current map unavailable");
          return false;
        }

        message::RcMapConfigMsg updated = rc_map_config_;
        const long value = std::lround(param_value);
        if (value < 1 || value > 4) {
          ESP_LOGW(kTag, "RC_MAP write rejected: param=%s value=%ld", def.id,
                   value);
          return false;
        }

        switch (def.key) {
          case mi::ParamKey::kRcMapRoll:
            updated.roll = static_cast<uint8_t>(value);
            break;
          case mi::ParamKey::kRcMapPitch:
            updated.pitch = static_cast<uint8_t>(value);
            break;
          case mi::ParamKey::kRcMapYaw:
            updated.yaw = static_cast<uint8_t>(value);
            break;
          case mi::ParamKey::kRcMapThrottle:
            updated.throttle = static_cast<uint8_t>(value);
            break;
          default:
            break;
        }

        if (!message::IsRcMapConfigValid(updated)) {
          ESP_LOGW(kTag,
                   "RC_MAP write rejected: invalid map r=%u p=%u y=%u t=%u",
                   (unsigned)updated.roll, (unsigned)updated.pitch,
                   (unsigned)updated.yaw, (unsigned)updated.throttle);
          return false;
        }
        const bool applied = ApplyRcMapConfigToFcLink(updated);
        if (applied) {
          ScheduleRcConfigConfirm(Sys().Timebase().NowMs());
        }
        return applied;
      }
      default:
        return false;
    }
  }

  if (!have_rc_calibration_config_) {
    ESP_LOGW(kTag, "RC_CAL write rejected: current calibration unavailable");
    return false;
  }

  message::RcCalibrationConfigMsg updated = rc_calibration_config_;
  const uint16_t rc_param_index = param_index - mi::kBaseParamCount;
  const uint8_t channel_index = static_cast<uint8_t>(
      rc_param_index / mi::kRcCalibrationParamCountPerChannel);
  const uint8_t field_index = static_cast<uint8_t>(
      rc_param_index % mi::kRcCalibrationParamCountPerChannel);
  mi::RcCalibrationField field = mi::RcCalibrationField::kMin;

  switch (field_index) {
    case 0:
      field = mi::RcCalibrationField::kMin;
      updated.min_us[channel_index] =
          static_cast<uint16_t>(std::lround(param_value));
      break;
    case 1:
      field = mi::RcCalibrationField::kMax;
      updated.max_us[channel_index] =
          static_cast<uint16_t>(std::lround(param_value));
      break;
    case 2:
      field = mi::RcCalibrationField::kTrim;
      updated.trim_us[channel_index] =
          static_cast<uint16_t>(std::lround(param_value));
      break;
    case 3:
      field = mi::RcCalibrationField::kRev;
      updated.rev[channel_index] = mi::QuantizeSignedUnit(param_value);
      break;
    default:
      return false;
  }

  if (!message::IsRcCalibrationConfigValid(updated)) {
    ESP_LOGW(kTag, "RC_CAL write rejected: ch=%u field=%s result=%u/%u/%u/%d",
             (unsigned)(channel_index + 1u), mi::RcCalibrationFieldName(field),
             (unsigned)updated.min_us[channel_index],
             (unsigned)updated.trim_us[channel_index],
             (unsigned)updated.max_us[channel_index],
             (int)updated.rev[channel_index]);
    return false;
  }

  const bool applied = ApplyRcCalibrationConfigToFcLink(updated);
  if (applied) {
    ScheduleRcConfigConfirm(Sys().Timebase().NowMs());
  }
  return applied;
}

void Mavlink::QueueCommandAck(TxState &tx, uint16_t command, uint8_t result,
                              uint8_t target_system, uint8_t target_component) {
  tx.pending_command_ack = true;
  tx.pending_command = command;
  tx.pending_command_result = result;
  tx.pending_command_target_system = target_system;
  tx.pending_command_target_component = target_component;
}

void Mavlink::QueueCommandAck(uint16_t command, uint8_t result,
                              uint8_t target_system,
                              uint8_t target_component) {
  QueueCommandAck(udp_tx_, command, result, target_system, target_component);
}

void Mavlink::QueueAutopilotVersion() {
  udp_tx_.pending_autopilot_version = true;
}

void Mavlink::QueueParamValue(uint16_t param_index) {
  if (!PushPendingParamValue(udp_tx_, param_index)) {
    ESP_LOGW(kTag, "dropping PARAM_VALUE reply: queue full");
  }
}

bool Mavlink::PushPendingParamValue(TxState &tx, uint16_t param_index) {
  if (tx.pending_param_count >= tx.pending_param_indices.size()) {
    return false;
  }

  const uint8_t slot =
      static_cast<uint8_t>((tx.pending_param_head + tx.pending_param_count) %
                           tx.pending_param_indices.size());
  tx.pending_param_indices[slot] = param_index;
  ++tx.pending_param_count;
  return true;
}

bool Mavlink::PopPendingParamValue(TxState &tx, uint16_t &param_index) {
  if (tx.pending_param_count == 0) {
    return false;
  }

  param_index = tx.pending_param_indices[tx.pending_param_head];
  tx.pending_param_head = static_cast<uint8_t>((tx.pending_param_head + 1) %
                                               tx.pending_param_indices.size());
  --tx.pending_param_count;
  return true;
}

void Mavlink::StartCommandAckFrame(TxState &tx) {
  if (!tx.pending_command_ack) {
    return;
  }

  mavlink_message_t m{};
  mavlink_msg_command_ack_pack(cfg_.identity.sysid, cfg_.identity.compid, &m,
                               tx.pending_command, tx.pending_command_result,
                               UINT8_MAX, 0, tx.pending_command_target_system,
                               tx.pending_command_target_component);

  tx.len = (uint16_t)mavlink_msg_to_send_buffer(tx.buf, &m);
  tx.sent = 0;
  tx.is_hb = false;
  tx.pending_command_ack = false;
}

void Mavlink::StartAutopilotVersionFrame(TxState &tx) {
  if (!tx.pending_autopilot_version) {
    return;
  }

  const uint64_t capabilities = mi::kMavProtocolCapabilityParamFloat |
                                mi::kMavProtocolCapabilityMavlink2 |
                                mi::kMavProtocolCapabilityParamEncodeCCast;
  static constexpr uint8_t kZeroHash[8] = {};
  static constexpr uint8_t kZeroUid2[18] = {};

  mavlink_message_t m{};
  mavlink_msg_autopilot_version_pack(
      cfg_.identity.sysid, cfg_.identity.compid, &m, capabilities,
      esp32_limits::kMavlinkFlightSwVersion, 0, 0, 0, kZeroHash, kZeroHash,
      kZeroHash, 0, 0, 0, kZeroUid2);

  tx.len = (uint16_t)mavlink_msg_to_send_buffer(tx.buf, &m);
  tx.sent = 0;
  tx.is_hb = false;
  tx.pending_autopilot_version = false;
}

void Mavlink::StartMissionCountFrame(TxState &tx) {
  if (!tx.pending_mission_count) {
    return;
  }

  mavlink_message_t m{};
  mavlink_msg_mission_count_pack(cfg_.identity.sysid, cfg_.identity.compid, &m,
                                 cfg_.identity.sysid, cfg_.identity.compid, 0,
                                 tx.pending_mission_type, 0);

  tx.len = (uint16_t)mavlink_msg_to_send_buffer(tx.buf, &m);
  tx.sent = 0;
  tx.is_hb = false;
  tx.pending_mission_count = false;
}

void Mavlink::StartStatusTextFrame(TxState &tx) {
  if (!tx.pending_statustext) {
    return;
  }

  char text[51] = {};
  std::strncpy(text, tx.pending_statustext_text, sizeof(text) - 1);

  mavlink_message_t m{};
  mavlink_msg_statustext_pack(cfg_.identity.sysid, cfg_.identity.compid, &m,
                              tx.pending_statustext_severity, text, 0, 0);

  tx.len = (uint16_t)mavlink_msg_to_send_buffer(tx.buf, &m);
  tx.sent = 0;
  tx.is_hb = false;
  tx.pending_statustext = false;
  tx.pending_statustext_text[0] = '\0';
}

void Mavlink::StartSingleParamValueFrame(TxState &tx) {
  uint16_t param_index = 0;
  if (!PopPendingParamValue(tx, param_index)) {
    return;
  }

  char param_id[17] = {};
  uint8_t param_type = 0;
  float param_value = 0.0f;
  if (!TryEncodeParamByIndex(param_index, param_id, param_type, param_value)) {
    return;
  }

  mavlink_message_t m{};
  mavlink_msg_param_value_pack(cfg_.identity.sysid, cfg_.identity.compid, &m,
                               param_id,
                               mi::EncodeParamValue(param_value, param_type),
                               param_type, mi::kTotalParamCount, param_index);

  tx.len = (uint16_t)mavlink_msg_to_send_buffer(tx.buf, &m);
  tx.sent = 0;
  tx.is_hb = false;
}

void Mavlink::StartParamValueFrame(TxState &tx) {
  if (!tx.param_stream_active) {
    return;
  }

  if (tx.next_param_index >= mi::kTotalParamCount) {
    tx.param_stream_active = false;
    tx.next_param_index = 0;
    return;
  }

  char param_id[17] = {};
  uint8_t param_type = 0;
  float param_value = 0.0f;
  if (!TryEncodeParamByIndex(tx.next_param_index, param_id, param_type,
                             param_value)) {
    tx.param_stream_active = false;
    tx.next_param_index = 0;
    return;
  }

  mavlink_message_t m{};
  mavlink_msg_param_value_pack(
      cfg_.identity.sysid, cfg_.identity.compid, &m, param_id,
      mi::EncodeParamValue(param_value, param_type), param_type,
      mi::kTotalParamCount, tx.next_param_index);

  tx.len = (uint16_t)mavlink_msg_to_send_buffer(tx.buf, &m);
  tx.sent = 0;
  tx.is_hb = false;
  tx.next_param_index++;
  if (tx.next_param_index >= mi::kTotalParamCount) {
    tx.param_stream_active = false;
    tx.next_param_index = 0;
  }
}

void Mavlink::EnsureScheduleArmed(TxState &tx, const TxConfig &cfg_tx,
                                  uint32_t now_ms) {
  if (tx.schedule_armed) {
    return;
  }

  ArmFirstSchedule(tx, cfg_tx, now_ms);
  tx.last_hb_done_ms = now_ms - cfg_tx.schedule.hb_deadline_ms;
  tx.schedule_armed = true;
}

bool Mavlink::StartNextFrameIfIdle(TxState &tx, const TxConfig &cfg_tx,
                                   uint32_t now_ms) {
  if (tx.len > 0) {
    return true;
  }

  if (ShouldSendHbNow(tx, cfg_tx, now_ms)) {
    StartHeartbeatFrame(tx, cfg_tx);
  } else if (tx.pending_command_ack) {
    StartCommandAckFrame(tx);
  } else if (tx.pending_autopilot_version) {
    StartAutopilotVersionFrame(tx);
  } else if (tx.pending_mission_count) {
    StartMissionCountFrame(tx);
  } else if (tx.pending_statustext) {
    StartStatusTextFrame(tx);
  } else if (tx.pending_param_count > 0) {
    StartSingleParamValueFrame(tx);
  } else if (tx.param_stream_active) {
    StartParamValueFrame(tx);
  } else {
    StartNextScheduledFrame(tx, cfg_tx, now_ms);
  }

  return tx.len > 0;
}

void Mavlink::CompleteFrame(TxState &tx, uint32_t now_ms) {
  if (tx.is_hb) {
    tx.last_hb_done_ms = now_ms;
  }
  tx.len = 0;
  tx.sent = 0;
  tx.is_hb = false;
}

void Mavlink::StartHeartbeatFrame(TxState &tx, const TxConfig &cfg_tx) {
  mavlink_message_t m{};

  mavlink_msg_heartbeat_pack(cfg_.identity.sysid, cfg_.identity.compid, &m,
                             MAV_TYPE_QUADROTOR, kMavAutopilot32Raven, 0, 0,
                             MAV_STATE_ACTIVE);

  tx.len = (uint16_t)mavlink_msg_to_send_buffer(tx.buf, &m);
  tx.sent = 0;
  tx.is_hb = true;
  tx.next_hb_ms += cfg_tx.periods.hb_ms;
}

void Mavlink::StartSysStatusFrame(TxState &tx) {
  tx.next_sys_ms += kSysStatusMs;

  uint32_t sensors_present = 0;
  uint32_t sensors_enabled = 0;
  uint32_t sensors_health = 0;
  uint16_t voltage_battery = 0;
  int16_t current_battery = -1;
  int8_t battery_remaining = -1;

  if (have_latest_) {
    sensors_present |= MAV_SYS_STATUS_SENSOR_GPS;
    sensors_enabled |= MAV_SYS_STATUS_SENSOR_GPS;
    if (latest_.fixType >= 2) {
      sensors_health |= MAV_SYS_STATUS_SENSOR_GPS;
    }

    if (latest_.batt_voltage > 0) {
      sensors_present |= MAV_SYS_STATUS_SENSOR_BATTERY;
      sensors_enabled |= MAV_SYS_STATUS_SENSOR_BATTERY;
      sensors_health |= MAV_SYS_STATUS_SENSOR_BATTERY;
      voltage_battery = latest_.batt_voltage;
      current_battery = latest_.batt_current;
      battery_remaining = latest_.batt_remaining;
    }
  }

  mavlink_message_t m{};
  mavlink_msg_sys_status_pack(cfg_.identity.sysid, cfg_.identity.compid, &m,
                              sensors_present, sensors_enabled, sensors_health,
                              0, voltage_battery, current_battery,
                              battery_remaining, 0, 0, 0, 0, 0, 0, 0, 0, 0);

  tx.len = (uint16_t)mavlink_msg_to_send_buffer(tx.buf, &m);
  tx.sent = 0;
  tx.is_hb = false;
}

void Mavlink::StartGpsRawIntFrame(TxState &tx, const TxConfig &cfg_tx) {
  tx.next_gps_ms += cfg_tx.periods.gps_ms;

  if (!have_latest_) {
    return;
  }

  mavlink_message_t m{};
  uint64_t time_usec = 0;
  (void)TryBuildGpsUnixUsec(latest_, time_usec);

  mavlink_msg_gps_raw_int_pack(
      cfg_.identity.sysid, cfg_.identity.compid, &m, time_usec,
      (uint8_t)latest_.fixType, latest_.lat, latest_.lon, (int32_t)latest_.hMSL,
      (uint16_t)(latest_.hAcc / 10u), (uint16_t)(latest_.vAcc / 10u),
      (uint16_t)latest_.vel, (uint16_t)latest_.hdg, (uint8_t)latest_.numSV, 0,
      0, 0, 0, 0, 0);

  tx.len = (uint16_t)mavlink_msg_to_send_buffer(tx.buf, &m);
  tx.sent = 0;
  tx.is_hb = false;
}

void Mavlink::StartAttitudeFrame(TxState &tx, const TxConfig &cfg_tx) {
  tx.next_att_ms += cfg_tx.periods.att_ms;

  if (!have_latest_) {
    return;
  }

  mavlink_message_t m{};
  float roll = ((float)latest_.roll * 0.01f) * 0.017453292519943295f;
  float pitch = ((float)latest_.pitch * 0.01f) * 0.017453292519943295f;
  float yaw = ((float)latest_.yaw * 0.01f) * 0.017453292519943295f;

  mavlink_msg_attitude_pack(cfg_.identity.sysid, cfg_.identity.compid, &m, 0,
                            roll, pitch, yaw, 0.0f, 0.0f, 0.0f);

  tx.len = (uint16_t)mavlink_msg_to_send_buffer(tx.buf, &m);
  tx.sent = 0;
  tx.is_hb = false;
}

void Mavlink::StartGlobalPositionIntFrame(TxState &tx, const TxConfig &cfg_tx) {
  tx.next_gpos_ms += cfg_tx.periods.gpos_ms;

  if (!have_latest_) {
    return;
  }

  mavlink_message_t m{};
  int32_t lat = latest_.lat;
  int32_t lon = latest_.lon;
  int32_t alt = latest_.hMSL;
  int32_t rel_alt = latest_.hMSL;
  int16_t vx = 0;
  int16_t vy = 0;
  int16_t vz = 0;
  uint16_t hdg = latest_.hdg;

  mavlink_msg_global_position_int_pack(cfg_.identity.sysid,
                                       cfg_.identity.compid, &m, 0, lat, lon,
                                       alt, rel_alt, vx, vy, vz, hdg);

  tx.len = (uint16_t)mavlink_msg_to_send_buffer(tx.buf, &m);
  tx.sent = 0;
  tx.is_hb = false;
}

void Mavlink::StartBatteryStatusFrame(TxState &tx, const TxConfig &cfg_tx) {
  tx.next_batt_ms += cfg_tx.periods.batt_ms;

  if (!have_latest_) {
    return;
  }

  mavlink_message_t m{};
  uint16_t voltages[10];
  for (int i = 0; i < 10; ++i) {
    voltages[i] = 0xFFFF;
  }
  voltages[0] = latest_.batt_voltage;

  mavlink_msg_battery_status_pack(
      cfg_.identity.sysid, cfg_.identity.compid, &m, 0,
      MAV_BATTERY_FUNCTION_ALL, MAV_BATTERY_TYPE_LIPO, 0, voltages,
      (int16_t)latest_.batt_current, (int32_t)latest_.batt_current, 0,
      (int8_t)latest_.batt_remaining, 0, 0, voltages, 0, 0);

  tx.len = (uint16_t)mavlink_msg_to_send_buffer(tx.buf, &m);
  tx.sent = 0;
  tx.is_hb = false;
}

void Mavlink::StartNextScheduledFrame(TxState &tx, const TxConfig &cfg_tx,
                                      uint32_t now_ms) {
  uint32_t best_due = 0xFFFFFFFFu;
  enum class Pick : uint8_t {
    kNone,
    kSys,
    kGps,
    kAtt,
    kGpos,
    kBatt,
  } pick = Pick::kNone;

  if ((int32_t)(now_ms - tx.next_sys_ms) >= 0) {
    best_due = tx.next_sys_ms;
    pick = Pick::kSys;
  }
  if (cfg_tx.periods.gps_ms > 0 && (int32_t)(now_ms - tx.next_gps_ms) >= 0) {
    if (tx.next_gps_ms < best_due) {
      best_due = tx.next_gps_ms;
      pick = Pick::kGps;
    }
  }
  if (cfg_tx.periods.att_ms > 0 && (int32_t)(now_ms - tx.next_att_ms) >= 0) {
    if (tx.next_att_ms < best_due) {
      best_due = tx.next_att_ms;
      pick = Pick::kAtt;
    }
  }
  if (cfg_tx.periods.gpos_ms > 0 && (int32_t)(now_ms - tx.next_gpos_ms) >= 0) {
    if (tx.next_gpos_ms < best_due) {
      best_due = tx.next_gpos_ms;
      pick = Pick::kGpos;
    }
  }
  if (cfg_tx.periods.batt_ms > 0 && (int32_t)(now_ms - tx.next_batt_ms) >= 0) {
    if (tx.next_batt_ms < best_due) {
      best_due = tx.next_batt_ms;
      pick = Pick::kBatt;
    }
  }

  switch (pick) {
    case Pick::kSys:
      StartSysStatusFrame(tx);
      break;
    case Pick::kGps:
      StartGpsRawIntFrame(tx, cfg_tx);
      break;
    case Pick::kAtt:
      StartAttitudeFrame(tx, cfg_tx);
      break;
    case Pick::kGpos:
      StartGlobalPositionIntFrame(tx, cfg_tx);
      break;
    case Pick::kBatt:
      StartBatteryStatusFrame(tx, cfg_tx);
      break;
    case Pick::kNone:
    default:
      break;
  }
}

void Mavlink::ServicePrimaryLink(uint32_t now_ms) {
  if (udp_ == nullptr) {
    return;
  }

  if (!primary_link_enabled_.load(std::memory_order_relaxed) ||
      !Sys().Wifi().HasAssociatedStations()) {
    udp_->ClearPeer();
    return;
  }

  uint8_t rx_buf[512];
  const int received = udp_->Receive(rx_buf, sizeof(rx_buf));
  if (received > 0) {
    mavlink_message_t msg{};
    mavlink_status_t status{};
    for (int i = 0; i < received; ++i) {
      if (mavlink_parse_char(MAVLINK_COMM_2, rx_buf[i], &msg, &status)) {
        udp_rx_packet_count_.fetch_add(1, std::memory_order_relaxed);
        HandleMessage(msg);
      }
    }
  }

  TxState &tx = udp_tx_;
  const TxConfig &cfg_tx = cfg_.tx;
  EnsureScheduleArmed(tx, cfg_tx, now_ms);
  if (!StartNextFrameIfIdle(tx, cfg_tx, now_ms)) {
    return;
  }

  const int sent = udp_->Send(tx.buf, tx.len);
  tx.len = 0;
  tx.sent = 0;
  if (sent > 0) {
    udp_tx_packet_count_.fetch_add(1, std::memory_order_relaxed);
  }
  CompleteFrame(tx, now_ms);
}
