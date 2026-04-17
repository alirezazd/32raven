#pragma once

#include <mavlink.h>

#include <array>
#include <atomic>
#include <cstdint>

#include "error_code.hpp"
#include "esp32_limits.hpp"
#include "message.hpp"  // for message::GpsData
#include "udp_server.hpp"

class Mavlink {
 public:
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
      } periods;

      struct Schedule {
        uint16_t hb_deadline_ms = 0;
        uint16_t gps_start_delay_ms = 0;
        uint16_t att_start_delay_ms = 0;
        uint16_t gpos_start_delay_ms = 0;
        uint16_t batt_start_delay_ms = 0;
      } schedule;
    } tx;
  };

  static Mavlink &GetInstance();

  void Init(const Config &cfg, UdpServer *udp);

  // Poll services the primary-link heartbeat LED outside the worker task.
  void Poll();
  void StartMavlinkTask();
  void StopMavlinkTask();

  // Worker: called from one dedicated RTOS task at a steady tick (e.g. 10ms).
  void WorkerTick(uint32_t now_ms);
  void SetPrimaryLinkEnabled(bool enabled);
  void OfferTelemetry(const message::GpsData &d, uint32_t now_ms);
  void SetRcMapConfig(const message::RcMapConfigMsg &cfg);
  void ClearRcMapConfig();
  void SetRcCalibrationConfig(const message::RcCalibrationConfigMsg &cfg);
  void ClearRcCalibrationConfig();
  void SetGyroCalibrationIdConfig(
      const message::GyroCalibrationIdConfigMsg &cfg);
  void ClearGyroCalibrationIdConfig();
  bool HasRcMapConfig() const { return have_rc_map_config_; }
  bool HasRcCalibrationConfig() const { return have_rc_calibration_config_; }
  bool HasGyroCalibrationIdConfig() const {
    return have_gyro_calibration_id_config_;
  }
  void QueueStatusText(const char *text, uint8_t severity = MAV_SEVERITY_INFO);
  uint32_t UdpRxPacketCount() const;
  uint32_t UdpTxPacketCount() const;

 private:
  using TxConfig = Config::Tx;

  struct TxState {
    static constexpr uint8_t kPendingParamValueQueueDepth = 64;

    uint8_t buf[MAVLINK_MAX_PACKET_LEN]{};
    uint16_t len = 0;
    uint16_t sent = 0;
    bool is_hb = false;
    bool pending_command_ack = false;
    uint16_t pending_command = 0;
    uint8_t pending_command_result = 0;
    uint8_t pending_command_target_system = 0;
    uint8_t pending_command_target_component = 0;
    bool pending_autopilot_version = false;
    bool pending_mission_count = false;
    uint8_t pending_mission_type = 0;
    bool pending_statustext = false;
    uint8_t pending_statustext_severity = MAV_SEVERITY_INFO;
    char pending_statustext_text[51]{};
    std::array<uint16_t, kPendingParamValueQueueDepth> pending_param_indices{};
    uint8_t pending_param_head = 0;
    uint8_t pending_param_count = 0;
    bool param_stream_active = false;
    uint16_t next_param_index = 0;
    uint32_t last_hb_done_ms = 0;
    bool schedule_armed = false;
    uint32_t next_hb_ms = 0;
    uint32_t next_sys_ms = 0;
    uint32_t next_gps_ms = 0;
    uint32_t next_att_ms = 0;
    uint32_t next_gpos_ms = 0;
    uint32_t next_batt_ms = 0;
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
  std::atomic<bool> primary_link_enabled_{false};
  std::atomic<bool> pending_primary_link_heartbeat_led_pulse_{false};
  uint32_t primary_link_heartbeat_led_clear_ms_ = 0;

  void HandleMessage(const mavlink_message_t &msg);
  void HandleCommandLong(const mavlink_message_t &msg,
                         const mavlink_command_long_t &cmd);
  void LogUnhandledMessageOnce(const mavlink_message_t &msg);
  void QueueCommandAck(TxState &tx, uint16_t command, uint8_t result,
                       uint8_t target_system, uint8_t target_component);
  void QueueCommandAck(uint16_t command, uint8_t result, uint8_t target_system,
                       uint8_t target_component);
  void QueueAutopilotVersion();
  void QueueParamValue(uint16_t param_index);
  bool PushPendingParamValue(TxState &tx, uint16_t param_index);
  bool PopPendingParamValue(TxState &tx, uint16_t &param_index);
  void StartCommandAckFrame(TxState &tx);
  void StartAutopilotVersionFrame(TxState &tx);
  void StartMissionCountFrame(TxState &tx);
  void StartStatusTextFrame(TxState &tx);
  void StartSingleParamValueFrame(TxState &tx);
  void StartParamValueFrame(TxState &tx);
  bool TryResolveParamIndex(int16_t requested_index, const char *requested_id,
                            uint16_t &resolved_index) const;
  bool TryEncodeParamByIndex(uint16_t param_index, char (&param_id)[17],
                             uint8_t &param_type, float &param_value) const;
  bool TrySetParamByIndex(uint16_t param_index, float param_value,
                          uint8_t param_type);

  TxState udp_tx_{};
  void ServicePrimaryLink(uint32_t now_ms);
  void EnsureScheduleArmed(TxState &tx, const TxConfig &cfg_tx,
                           uint32_t now_ms);
  bool StartNextFrameIfIdle(TxState &tx, const TxConfig &cfg_tx,
                            uint32_t now_ms);
  void CompleteFrame(TxState &tx, uint32_t now_ms);

  // ---------- Latest-only telemetry cache ----------
  message::GpsData latest_{};
  bool have_latest_ = false;
  uint32_t latest_update_ms_ = 0;
  message::RcMapConfigMsg rc_map_config_{};
  bool have_rc_map_config_ = false;
  message::RcCalibrationConfigMsg rc_calibration_config_{};
  bool have_rc_calibration_config_ = false;
  message::GyroCalibrationIdConfigMsg gyro_calibration_id_config_{};
  bool have_gyro_calibration_id_config_ = false;
  uint8_t rc_chan_count_ = message::kRcCalibrationChannelCount;
  bool rc_config_confirm_pending_ = false;
  uint32_t rc_config_confirm_due_ms_ = 0;
  struct PendingFcConfigRequest {
    message::MsgId request_id = message::MsgId::kPing;
    ErrorCode failure_code = ErrorCode::kFcLinkInitFailed;
    uint32_t next_retry_ms = 0;
    uint16_t attempts_sent = 0;
    bool active = false;
  };
  PendingFcConfigRequest pending_rc_map_request_{};
  PendingFcConfigRequest pending_rc_calibration_request_{};
  PendingFcConfigRequest pending_gyro_calibration_id_request_{};

  // Helpers
  void ScheduleRcConfigConfirm(uint32_t now_ms);
  void ServiceRcConfigConfirm(uint32_t now_ms);

  bool ShouldSendHbNow(const TxState &tx, const TxConfig &cfg_tx,
                       uint32_t now_ms) const;
  void ArmFirstSchedule(TxState &tx, const TxConfig &cfg_tx, uint32_t now_ms);
  void StartFcConfigRequest(message::MsgId request_id, ErrorCode failure_code);
  void ServiceFcConfigRequests(uint32_t now_ms);
  bool ApplyRcMapConfigToFcLink(const message::RcMapConfigMsg &cfg);
  bool ApplyRcCalibrationConfigToFcLink(
      const message::RcCalibrationConfigMsg &cfg);

  // Frame builders (only the ELRS-supported set)
  void StartHeartbeatFrame(TxState &tx, const TxConfig &cfg_tx);
  void StartSysStatusFrame(TxState &tx);
  void StartGpsRawIntFrame(TxState &tx, const TxConfig &cfg_tx);
  void StartAttitudeFrame(TxState &tx, const TxConfig &cfg_tx);
  void StartGlobalPositionIntFrame(TxState &tx, const TxConfig &cfg_tx);
  void StartBatteryStatusFrame(TxState &tx, const TxConfig &cfg_tx);

  // Pick next stream to send when idle (hb has priority)
  void StartNextScheduledFrame(TxState &tx, const TxConfig &cfg_tx,
                               uint32_t now_ms);
};
