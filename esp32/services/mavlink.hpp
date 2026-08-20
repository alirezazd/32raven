// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <mavlink.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <cstring>
#include <optional>
#include <span>
#include <type_traits>
#include <variant>

#include "error_code.hpp"
#include "mavlink_transport.hpp"
#include "fc_link.hpp"
#include "message.hpp"  // for message::GpsData
#include "panic.hpp"
#include "ring_buffer.hpp"
#include "topic_scheduler.hpp"

class Mavlink {
 public:
  enum class PanicSource : uint8_t {
    kEsp32,
    kStm32,
  };

  // mavlink_tx.cpp's period and state arrays are built in this order and
  // indexed by it. Public only because those tables sit at file scope.
  enum class TxSlot : uint8_t {
    kHb,
    kSys,
    kGps,
    kAtt,
    kGpos,
    kBatt,
    kRc,
    kEsc,
    kCount,
  };

  static constexpr size_t kTxSlotCount = static_cast<size_t>(TxSlot::kCount);

  struct Config {
    struct Identity {
      uint8_t sysid = 0;
      uint8_t compid = 0;
    } identity;

    struct Tx {
      struct Periods {
        uint16_t hb_ms = 0;
        uint16_t gps_ms = 0;
        uint16_t att_ms = 0;
        uint16_t gpos_ms = 0;
        uint16_t batt_ms = 0;
        uint16_t rc_ms = 0;
        uint16_t esc_ms = 0;
      } periods;

      struct Schedule {
        uint16_t hb_deadline_ms = 0;
      } schedule;
    } tx;
  };

  struct LatestRcChannelsData {
    message::RcChannelsMsg msg{};
    uint32_t update_ms = 0;
  };

  static Mavlink &GetInstance();

  // Swaps the link without re-initializing the service. Clears the in-flight
  // frame and the previous transport's peer, so a stale UDP address cannot be
  // carried into a serial swap.
  void SetTransport(IMavlinkTransport *transport);

  void Poll(uint32_t now_ms);
  void SetTelemetryLink(bool enabled);
  template <typename T>
  void UpdateTelemetryCache(const T &value, uint32_t now_ms) {
    if constexpr (std::is_same_v<T, message::GpsData>) {
      UpdateCache(gps_, value, now_ms);
    } else if constexpr (std::is_same_v<T, message::RcChannelsMsg>) {
      UpdateCache(rc_channels_, value, now_ms);
    } else if constexpr (std::is_same_v<T, message::SystemStatusMsg>) {
      UpdateCache(system_status_, value, now_ms);
    } else if constexpr (std::is_same_v<T, message::VehicleStatusMsg>) {
      UpdateCache(vehicle_status_, value, now_ms);
    } else if constexpr (std::is_same_v<T, message::EscTelemetryMsg>) {
      UpdateCache(esc_telemetry_, value, now_ms);
    } else if constexpr (std::is_same_v<T, message::AttitudeMsg>) {
      UpdateCache(attitude_, value, now_ms);
    } else {
      static_assert(sizeof(T) == 0, "unsupported MAVLink telemetry cache type");
    }
  }
  template <typename T>
  void UpdateConfigCache(const T &cfg, uint32_t now_ms) {
    if constexpr (std::is_same_v<T, message::RcMapConfigMsg>) {
      if (!message::IsRcMapConfigValid(cfg)) {
        Panic(ErrorCode::Esp32::kFcLinkInvalidRcMapConfig);
      }
      UpdateCache(rc_map_config_, cfg, now_ms);
      rc_map_request_ = {};
      const T *desired = rc_map_apply_.Desired();
      if (desired != nullptr && std::memcmp(desired, &cfg, sizeof(T)) == 0) {
        rc_map_apply_.Reset();
      }
    } else if constexpr (std::is_same_v<T, message::RcCalibrationConfigMsg>) {
      if (!message::IsRcCalibrationConfigValid(cfg)) {
        Panic(ErrorCode::Common::kFcLinkInvalidRcCalibrationConfig);
      }
      UpdateCache(rc_calibration_config_, cfg, now_ms);
      rc_calibration_request_ = {};
      const T *desired = rc_calibration_apply_.Desired();
      if (desired != nullptr && std::memcmp(desired, &cfg, sizeof(T)) == 0) {
        rc_calibration_apply_.Reset();
      }
    } else if constexpr (std::is_same_v<T,
                                        message::GyroCalibrationIdConfigMsg>) {
      if (!message::IsGyroCalibrationIdConfigValid(cfg)) {
        Panic(ErrorCode::Common::kFcLinkInvalidGyroCalibrationIdConfig);
      }
      UpdateCache(gyro_calibration_id_config_, cfg, now_ms);
      gyro_calibration_id_request_ = {};
    } else {
      static_assert(sizeof(T) == 0, "unsupported MAVLink config cache type");
    }
  }
  void ReportPanic(PanicSource source, uint32_t error_code);
  uint32_t GetUdpRxPacketCount() const;
  uint32_t GetUdpTxPacketCount() const;
  uint32_t GetUdpRxHeartbeatCount() const;
  uint32_t GetUdpTxHeartbeatCount() const;
  std::optional<LatestRcChannelsData> GetLatestRcChannelsData() const;
  std::optional<bool> PeerArmed(uint32_t now_ms) const;

 private:
  friend class System;
  void Init(const Config &cfg, IMavlinkTransport *transport,
            FcLink &fc_link);
  template <typename T>
  struct CachedValue {
    T value{};
    bool have_data = false;
    uint32_t update_ms = 0;
    uint32_t generation = 0;
  };

  template <typename T>
  static void UpdateCache(CachedValue<T> &cache, const T &value,
                          uint32_t now_ms) {
    cache.value = value;
    cache.have_data = true;
    cache.update_ms = now_ms;
    cache.generation =
        (cache.generation == UINT32_MAX) ? 1u : (cache.generation + 1u);
  }

  template <typename T>
  static std::optional<T> GetCachedValue(const CachedValue<T> &cache) {
    if (!cache.have_data) {
      return std::nullopt;
    }
    return cache.value;
  }

  static constexpr std::size_t kMavParamIdCStringLen =
      MAVLINK_MSG_PARAM_VALUE_FIELD_PARAM_ID_LEN + 1u;

  struct FixedParamRef {
    uint16_t mavlink_index = 0;
  };

  // Values are the per-channel parameter offsets; mavlink_index arithmetic and
  // the index->field modulo both depend on them.
  enum class RcCalField : uint8_t { kMin = 0, kMax = 1, kTrim = 2, kRev = 3 };

  struct RcCalibrationParamRef {
    uint16_t mavlink_index = 0;
    uint8_t channel_index = 0;
    RcCalField field = RcCalField::kMin;
  };

  using ParamRef = std::variant<FixedParamRef, RcCalibrationParamRef>;

  struct EncodedParam {
    char id[kMavParamIdCStringLen]{};
    uint8_t type = 0;
    float value = 0.0f;
  };

  enum class ParamSetResult : uint8_t {
    kAccepted,
    kUnsupported,
    kMissingBaseConfig,
    kInvalidValue,
    kInvalidResultingConfig,
  };

  struct ParamRequestState {
    uint32_t next_request_ms = 0;
    bool waiting = false;
  };

  template <typename T>
  struct PendingParamApplyState {
    struct Idle {};

    struct Active {
      T desired{};
      uint32_t next_retry_ms = 0;
      uint16_t attempts_remaining = 0;
    };

    bool IsActive() const { return std::holds_alternative<Active>(state); }

    const T *Desired() const {
      const Active *active = GetActive();
      return (active == nullptr) ? nullptr : &active->desired;
    }

    void Start(const T &value, uint32_t now_ms, uint16_t attempts) {
      state = Active{value, now_ms, attempts};
    }

    void Reset() { state = Idle{}; }

    Active *GetActive() { return std::get_if<Active>(&state); }
    const Active *GetActive() const { return std::get_if<Active>(&state); }

    std::variant<Idle, Active> state{};
  };

  struct PendingApplyNone {};
  struct PendingApplyFailed {};

  template <typename T>
  struct PendingApplySend {
    T value{};
  };

  template <typename T>
  using PendingApplyAction =
      std::variant<PendingApplyNone, PendingApplySend<T>, PendingApplyFailed>;

  enum class ParamDependency : uint8_t {
    kNone = 0,
    kRcMap = 1 << 0,
    kRcCalibration = 1 << 1,
    kGyroCalibrationId = 1 << 2,
  };

  struct TxState {
    static constexpr uint8_t kPendingParamValueQueueDepth = 64;

    struct ParamStreamIdle {};
    struct ParamStreamActive {
      uint16_t next_param_index = 0;
    };

    RingBuffer<uint16_t, kPendingParamValueQueueDepth + 1>
        pending_param_queue_{};
    std::variant<ParamStreamIdle, ParamStreamActive> param_stream_{};
  };

  struct CommandAck {
    uint16_t command = 0;
    uint8_t result = 0;
    uint8_t target_system = 0;
    uint8_t target_component = 0;
  };

  struct AutopilotVersion {};

  struct MissionCount {
    uint8_t target_system = 0;
    uint8_t target_component = 0;
    uint8_t mission_type = 0;
  };

  struct StatusText {
    uint8_t severity = MAV_SEVERITY_INFO;
    char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN + 1]{};
  };

  using TxQueueItem = std::variant<std::monostate, CommandAck, AutopilotVersion,
                                   MissionCount, StatusText>;

  // The buffer is private so Bytes() is the only way to reach the frame: a
  // span over the raw array would cover all MAVLINK_MAX_PACKET_LEN bytes and
  // put whatever the previous frame left behind on the wire.
  class TxFrameState {
   public:
    TxFrameState() = default;

    TxFrameState(const mavlink_message_t &msg, bool is_heartbeat)
        : is_hb_(is_heartbeat) {
      len_ =
          static_cast<uint16_t>(mavlink_msg_to_send_buffer(buf_.data(), &msg));
    }

    void Clear() {
      len_ = 0;
      is_hb_ = false;
    }

    [[nodiscard]] bool Empty() const { return len_ == 0; }
    [[nodiscard]] bool IsHeartbeat() const { return is_hb_; }
    [[nodiscard]] std::span<const uint8_t> Bytes() const {
      return {buf_.data(), len_};
    }

   private:
    std::array<uint8_t, MAVLINK_MAX_PACKET_LEN> buf_{};
    uint16_t len_ = 0;
    bool is_hb_ = false;
  };

  Mavlink();
  ~Mavlink();
  Mavlink(const Mavlink &) = delete;
  Mavlink &operator=(const Mavlink &) = delete;

  IMavlinkTransport *transport_ = nullptr;
  FcLink *fc_link_ = nullptr;
  Config cfg_{};

  std::array<uint16_t, 64> unhandled_logged_msgids_{};
  std::atomic<uint32_t> udp_rx_packet_count_{0};
  std::atomic<uint32_t> udp_tx_packet_count_{0};
  // Bumped before the totals above, so a reader that sees a total move already
  // sees whether that packet was a heartbeat.
  std::atomic<uint32_t> udp_rx_heartbeat_count_{0};
  std::atomic<uint32_t> udp_tx_heartbeat_count_{0};
  uint8_t unhandled_logged_msgid_count_ = 0;
  std::array<uint16_t, 64> unhandled_logged_commands_{};
  uint8_t unhandled_logged_command_count_ = 0;
  bool link_enabled_ = false;
  CachedValue<message::GpsData> gps_{};
  CachedValue<message::AttitudeMsg> attitude_{};
  CachedValue<message::RcChannelsMsg> rc_channels_{};
  CachedValue<message::SystemStatusMsg> system_status_{};
  CachedValue<message::VehicleStatusMsg> vehicle_status_{};
  CachedValue<message::EscTelemetryMsg> esc_telemetry_{};

  void HandleMessage(const mavlink_message_t &msg);
  void HandleParamMessage(const mavlink_message_t &msg);
  void HandleMissionMessage(const mavlink_message_t &msg);
  void HandleCommandMessage(const mavlink_message_t &msg);
  void HandleCommandLong(const mavlink_message_t &msg,
                         const mavlink_command_long_t &cmd);
  bool IsTargetedToThisComponent(uint8_t target_system,
                                 uint8_t target_component) const {
    return target_system == cfg_.identity.sysid &&
           (target_component == cfg_.identity.compid || target_component == 0 ||
            target_component == MAV_COMP_ID_ALL);
  }
  void ServiceUdpRx();
  void LogUnhandledMessageOnce(const mavlink_message_t &msg);
  void LogUnhandledCommandOnce(uint16_t command, const char *reason);
  void QueueTxItem(const TxQueueItem &item);
  void QueueCommandAck(uint16_t command, uint8_t result, uint8_t target_system,
                       uint8_t target_component);
  void QueueAutopilotVersion();
  void QueueMissionCount(uint8_t target_system, uint8_t target_component,
                         uint8_t mission_type);
  void QueueStatusText(const char *text, uint8_t severity = MAV_SEVERITY_INFO);
  void NotifyGcsIssue(const char *text,
                      uint8_t severity = MAV_SEVERITY_WARNING);
  bool SendStatusTextFrameNow(const StatusText &status,
                              bool require_link_enabled);
  std::optional<TxFrameState> StartQueuedTxWorkFrame();
  TxFrameState StartCommandAckFrame(const CommandAck &ack);
  TxFrameState StartAutopilotVersionFrame(const AutopilotVersion &work);
  TxFrameState StartMissionCountFrame(const MissionCount &work);
  TxFrameState StartStatusTextFrame(const StatusText &work);
  std::optional<TxFrameState> StartRcChannelsFrame(const Config::Tx &cfg_tx);
  void ResetParamState();
  std::optional<ParamRef> TryResolveParam(int16_t requested_index,
                                          const char *requested_id) const;
  std::optional<ParamRef> TryResolveParamByIndex(uint16_t param_index) const;
  void QueueParamValue(TxState &tx, uint16_t param_index) const;
  std::optional<TxFrameState> StartNextParamFrame(TxState &tx, uint8_t sysid,
                                                  uint8_t compid);
  std::optional<EncodedParam> TryEncodeParam(const ParamRef &param) const;
  std::optional<EncodedParam> TryEncodeFixedParam(
      const FixedParamRef &param) const;
  std::optional<float> TryEncodeGyroCalibrationIdParam() const;
  std::optional<float> TryEncodeRcMapParam(const FixedParamRef &param) const;
  std::optional<EncodedParam> TryEncodeRcCalibrationParam(
      const RcCalibrationParamRef &param) const;
  ParamSetResult TrySetParam(const ParamRef &param, float param_value);
  ParamSetResult TrySetFixedParam(const FixedParamRef &param,
                                  float param_value);
  ParamSetResult TrySetRcMapParam(const FixedParamRef &param,
                                  float param_value);
  ParamSetResult TrySetRcCalibrationParam(const RcCalibrationParamRef &param,
                                          float param_value);
  std::optional<TxFrameState> StartParamValueFrame(const ParamRef &param,
                                                   uint8_t sysid,
                                                   uint8_t compid);
  static uint16_t ParamMavlinkIndex(const ParamRef &param);
  static const char *ParamSetResultName(ParamSetResult result);
  static std::optional<ParamRef> TryResolveRcCalibrationParam(
      const char *param_id);
  ParamDependency DependencyForParam(const ParamRef &param) const;
  bool IsParamDependencyApplyPending(ParamDependency dependency) const;
  bool EnsureParamDependencyRequested(ParamDependency dependency);
  void ServicePendingParamApplies(uint32_t now_ms);
  template <typename T>
  PendingApplyAction<T> PreparePendingParamApply(
      PendingParamApplyState<T> &apply, uint32_t now_ms);
  std::optional<TxFrameState> StartQueuedParamValueFrame(TxState &tx,
                                                         uint8_t sysid,
                                                         uint8_t compid);
  std::optional<TxFrameState> StartStreamParamValueFrame(TxState &tx,
                                                         uint8_t sysid,
                                                         uint8_t compid);

  static constexpr uint8_t kTxWorkQueueDepth = 8;
  TxState udp_tx_{};
  RingBuffer<TxQueueItem, kTxWorkQueueDepth + 1> tx_work_queue_{};
  TxFrameState tx_frame_{};
  TopicScheduler tx_scheduler_{};
  std::array<TopicState, kTxSlotCount> tx_slots_{};
  // Outside the scheduler because it measures from the frame actually leaving,
  // not from when the slot came due.
  uint32_t last_hb_done_ms_ = 0;
  uint32_t next_tx_poll_ms_ = 0;
  uint32_t peer_timeout_ms_ = 500;
  void ServiceTx(uint32_t now_ms);
  void ServiceUdpTx(uint32_t now_ms);
  bool StartNextFrameIfIdle(TxState &tx, const Config::Tx &cfg_tx,
                            uint32_t now_ms);
  void CompleteFrame(TxFrameState &frame, uint32_t now_ms);
  CachedValue<message::RcMapConfigMsg> rc_map_config_{};
  CachedValue<message::RcCalibrationConfigMsg> rc_calibration_config_{};
  CachedValue<message::GyroCalibrationIdConfigMsg>
      gyro_calibration_id_config_{};
  ParamRequestState rc_map_request_{};
  ParamRequestState rc_calibration_request_{};
  ParamRequestState gyro_calibration_id_request_{};
  PendingParamApplyState<message::RcMapConfigMsg> rc_map_apply_{};
  PendingParamApplyState<message::RcCalibrationConfigMsg>
      rc_calibration_apply_{};

  bool ShouldSendHbNow(const Config::Tx &cfg_tx, uint32_t now_ms) const;
  void InitTxSchedule(uint32_t now_ms, bool force_heartbeat_due = false);

  TxFrameState StartHeartbeatFrame(const Config::Tx &cfg_tx, uint32_t now_ms);
  // The ESP32 emits every one of these whether or not the FC is talking, so
  // a frame arriving says nothing about the peer's health. Each of the three
  // built from this cache has to ask its age itself.
  bool SystemStatusFresh(uint32_t now_ms) const;

  TxFrameState StartSysStatusFrame(uint32_t now_ms);
  std::optional<TxFrameState> StartGpsRawIntFrame(const Config::Tx &cfg_tx);
  std::optional<TxFrameState> StartAttitudeFrame(const Config::Tx &cfg_tx);
  std::optional<TxFrameState> StartGlobalPositionIntFrame(
      const Config::Tx &cfg_tx);
  std::optional<TxFrameState> StartBatteryStatusFrame(const Config::Tx &cfg_tx,
                                                      uint32_t now_ms);
  std::optional<TxFrameState> StartEscStatusFrame(const Config::Tx &cfg_tx);

  std::optional<TxFrameState> StartNextScheduledFrame(const Config::Tx &cfg_tx,
                                                      uint32_t now_ms);
};
