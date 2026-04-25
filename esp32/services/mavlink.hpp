#pragma once

#include <mavlink.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <optional>
#include <variant>

#include "message.hpp"  // for message::GpsData
#include "ring_buffer.hpp"
#include "udp_server.hpp"

class Mavlink {
 public:
  struct RcChannelsSample {
    message::RcChannelsMsg msg{};
    uint32_t update_ms = 0;
  };

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
      } periods;

      struct Schedule {
        uint16_t hb_deadline_ms = 0;
        uint16_t gps_start_delay_ms = 0;
        uint16_t att_start_delay_ms = 0;
        uint16_t gpos_start_delay_ms = 0;
        uint16_t batt_start_delay_ms = 0;
        uint16_t rc_start_delay_ms = 0;
      } schedule;
    } tx;
  };

  static Mavlink &GetInstance();

  void Init(const Config &cfg, UdpServer *udp);

  // Main task: services incoming MAVLink control traffic, pending FC-link side
  // effects, and rate-limited outbound MAVLink frames.
  void Poll(uint32_t now_ms);
  void EnableTelemetryLink();
  void DisableTelemetryLink();
  void UpdateGpsCache(const message::GpsData &gps, uint32_t now_ms);
  void UpdateRcChannelsCache(const message::RcChannelsMsg &channels,
                             uint32_t now_ms);
  void UpdateRcMapConfigCache(const message::RcMapConfigMsg &cfg,
                              uint32_t now_ms);
  void UpdateRcCalibrationConfigCache(
      const message::RcCalibrationConfigMsg &cfg, uint32_t now_ms);
  void UpdateGyroCalibrationIdConfigCache(
      const message::GyroCalibrationIdConfigMsg &cfg, uint32_t now_ms);
  std::optional<message::GpsData> GetLatestGpsData() const;
  std::optional<RcChannelsSample> GetLatestRcChannelsData() const;
  uint32_t UdpRxPacketCount() const;
  uint32_t UdpTxPacketCount() const;

 private:
  using TxConfig = Config::Tx;
  struct TxState;

  template <typename T>
  struct CachedValue {
    T value{};
    bool have_data = false;
    uint32_t update_ms = 0;
    uint32_t generation = 0;
  };

  struct ParamRef {
    enum class Family : uint8_t {
      kFixed,
      kRcCalibration,
    };

    Family family = Family::kFixed;
    uint16_t index = 0;
    uint8_t channel_index = 0;
    uint8_t field_index = 0;
  };

  struct ParamRequestState {
    uint32_t next_request_ms = 0;
    bool waiting = false;
  };

  template <typename T>
  struct PendingParamApplyState {
    T desired{};
    uint32_t next_retry_ms = 0;
    uint16_t attempts_remaining = 0;
    bool active = false;
  };

  struct PendingApplyAction {
    bool send = false;
    bool failed = false;
  };

  enum class ParamDependency : uint8_t {
    kNone = 0,
    kRcMap = 1 << 0,
    kRcCalibration = 1 << 1,
    kGyroCalibrationId = 1 << 2,
  };

  struct TxState {
    static constexpr uint8_t kPendingParamValueQueueDepth = 64;

    RingBuffer<uint16_t, kPendingParamValueQueueDepth + 1>
        pending_param_queue_{};
    bool param_stream_active = false;
    uint16_t next_param_index = 0;
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

  struct TxFrameState {
    uint8_t buf[MAVLINK_MAX_PACKET_LEN]{};
    uint16_t len = 0;
    bool is_hb = false;
  };

  struct TxScheduleState {
    uint32_t last_hb_done_ms = 0;
    uint32_t next_hb_ms = 0;
    uint32_t next_sys_ms = 0;
    uint32_t next_gps_ms = 0;
    uint32_t next_att_ms = 0;
    uint32_t next_gpos_ms = 0;
    uint32_t next_batt_ms = 0;
    uint32_t next_rc_ms = 0;
  };

  Mavlink();
  ~Mavlink();
  Mavlink(const Mavlink &) = delete;
  Mavlink &operator=(const Mavlink &) = delete;

  UdpServer *udp_ = nullptr;
  Config cfg_{};

  std::array<uint16_t, 64> unhandled_logged_msgids_{};
  uint8_t unhandled_logged_msgid_count_ = 0;
  std::atomic<uint32_t> udp_rx_packet_count_{0};
  std::atomic<uint32_t> udp_tx_packet_count_{0};
  bool link_enabled_ = false;
  CachedValue<message::GpsData> latest_gps_{};
  CachedValue<message::RcChannelsMsg> latest_rc_channels_{};

  void HandleMessage(const mavlink_message_t &msg);
  void HandleParamMessage(const mavlink_message_t &msg);
  void HandleMissionMessage(const mavlink_message_t &msg);
  void HandleCommandMessage(const mavlink_message_t &msg);
  void HandleCommandLong(const mavlink_message_t &msg,
                         const mavlink_command_long_t &cmd);
  void ServiceUdpRx();
  void LogUnhandledMessageOnce(const mavlink_message_t &msg);
  void QueueTxItem(const TxQueueItem &item);
  void QueueCommandAck(uint16_t command, uint8_t result, uint8_t target_system,
                       uint8_t target_component);
  void QueueAutopilotVersion();
  void QueueMissionCount(uint8_t target_system, uint8_t target_component,
                         uint8_t mission_type);
  void QueueStatusText(const char *text, uint8_t severity = MAV_SEVERITY_INFO);
  bool StartQueuedTxWorkFrame(TxFrameState &frame);
  void StartCommandAckFrame(const CommandAck &ack, TxFrameState &frame);
  void StartAutopilotVersionFrame(const AutopilotVersion &work,
                                  TxFrameState &frame);
  void StartMissionCountFrame(const MissionCount &work, TxFrameState &frame);
  void StartStatusTextFrame(const StatusText &work, TxFrameState &frame);
  void StartRcChannelsFrame(TxFrameState &frame, const TxConfig &cfg_tx);
  void ResetParamState();
  std::optional<ParamRef> TryResolveParam(int16_t requested_index,
                                          const char *requested_id) const;
  std::optional<ParamRef> TryResolveParamByIndex(uint16_t param_index) const;
  void BeginParamStream(TxState &tx) const;
  bool HasPendingParamWork(const TxState &tx) const;
  void QueueParamValue(TxState &tx, uint16_t param_index) const;
  bool StartNextParamFrame(TxState &tx, TxFrameState &frame, uint8_t sysid,
                           uint8_t compid);
  bool TryEncodeParam(const ParamRef &param, char (&param_id)[17],
                      uint8_t &param_type, float &param_value) const;
  bool TryEncodeFixedParam(const ParamRef &param, char (&param_id)[17],
                           uint8_t &param_type, float &param_value) const;
  bool TryEncodeGyroCalibrationIdParam(float &param_value) const;
  bool TryEncodeRcMapParam(const ParamRef &param, float &param_value) const;
  bool TryEncodeRcCalibrationParam(const ParamRef &param, char (&param_id)[17],
                                   uint8_t &param_type,
                                   float &param_value) const;
  bool TrySetParam(const ParamRef &param, float param_value,
                   uint8_t param_type);
  bool TrySetFixedParam(const ParamRef &param, float param_value);
  bool TrySetRcMapParam(const ParamRef &param, float param_value);
  bool TrySetRcCalibrationParam(const ParamRef &param, float param_value);
  bool StartParamValueFrame(const ParamRef &param, TxFrameState &frame,
                            uint8_t sysid, uint8_t compid);
  static std::array<char, 17> CopyMavParamId(const char *src);
  static ParamRef MakeFixedParamRef(uint16_t param_index);
  static ParamRef MakeRcCalibrationParamRef(uint16_t param_index,
                                            uint8_t channel_index,
                                            uint8_t field_index);
  static std::optional<ParamRef> TryResolveRcCalibrationParam(
      const char *param_id);
  static std::optional<uint16_t> PeekPendingParamValue(const TxState &tx);
  static bool PushPendingParamValue(TxState &tx, uint16_t param_index);
  static std::optional<uint16_t> PopPendingParamValue(TxState &tx);
  ParamDependency DependencyForParam(const ParamRef &param) const;
  bool IsParamDependencyApplyPending(ParamDependency dependency) const;
  bool EnsureParamDependencyRequested(ParamDependency dependency);
  void ServicePendingParamApplies(uint32_t now_ms);
  template <typename T>
  PendingApplyAction PreparePendingParamApply(PendingParamApplyState<T> &apply,
                                              T &value, uint32_t now_ms);
  bool StartQueuedParamValueFrame(TxState &tx, TxFrameState &frame,
                                  uint8_t sysid, uint8_t compid);
  bool StartStreamParamValueFrame(TxState &tx, TxFrameState &frame,
                                  uint8_t sysid, uint8_t compid);
  bool TryGetRcMapConfig(message::RcMapConfigMsg &cfg) const;
  bool TryGetRcCalibrationConfig(message::RcCalibrationConfigMsg &cfg) const;
  bool TryGetGyroCalibrationIdConfig(
      message::GyroCalibrationIdConfigMsg &cfg) const;

  static constexpr uint8_t kTxWorkQueueDepth = 8;
  TxState udp_tx_{};
  RingBuffer<TxQueueItem, kTxWorkQueueDepth + 1> tx_work_queue_{};
  TxFrameState tx_frame_{};
  TxScheduleState tx_schedule_{};
  uint32_t next_tx_poll_ms_ = 0;
  void ServiceTx(uint32_t now_ms);
  void ServiceUdpTx(uint32_t now_ms);
  bool StartNextFrameIfIdle(TxState &tx, const TxConfig &cfg_tx,
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

  bool ShouldSendHbNow(const TxConfig &cfg_tx, uint32_t now_ms) const;
  void InitTxSchedule(const TxConfig &cfg_tx, uint32_t now_ms,
                      bool force_heartbeat_due = false);

  // Frame builders (only the ELRS-supported set)
  void StartHeartbeatFrame(TxFrameState &frame, const TxConfig &cfg_tx);
  void StartSysStatusFrame(TxFrameState &frame);
  void StartGpsRawIntFrame(TxFrameState &frame, const TxConfig &cfg_tx);
  void StartAttitudeFrame(TxFrameState &frame, const TxConfig &cfg_tx);
  void StartGlobalPositionIntFrame(TxFrameState &frame, const TxConfig &cfg_tx);
  void StartBatteryStatusFrame(TxFrameState &frame, const TxConfig &cfg_tx);

  // Pick next stream to send when idle (hb has priority)
  void StartNextScheduledFrame(TxFrameState &frame, const TxConfig &cfg_tx,
                               uint32_t now_ms);
};
