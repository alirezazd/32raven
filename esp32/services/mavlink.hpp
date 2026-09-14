// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <mavlink.h>

#include <array>
#include <atomic>
#include <cstdint>
#include <optional>
#include <span>
#include <type_traits>
#include <variant>

#include "fc_config_cache.hpp"
#include "fc_link.hpp"
#include "flight_mode.hpp"
#include "mavlink_config.hpp"
#include "mavlink_param.hpp"
#include "mavlink_transport.hpp"
#include "message.hpp"
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
      // Before the cache, which is what the comparison is against. Driven by
      // arrival rather than by the SYS_STATUS schedule: a transition the GCS
      // link was down for is still worth queueing, and one frame in must not
      // become one announcement per frame out.
      ReportSensorHealthChanges(value);
      UpdateCache(system_status_, value, now_ms);
    } else if constexpr (std::is_same_v<T, message::VehicleStatusMsg>) {
      UpdateCache(vehicle_status_, value, now_ms);
    } else if constexpr (std::is_same_v<T, message::EscTelemetryMsg>) {
      UpdateCache(esc_telemetry_, value, now_ms);
    } else if constexpr (std::is_same_v<T, message::AttitudeMsg>) {
      UpdateCache(attitude_, value, now_ms);
    } else if constexpr (std::is_same_v<T, message::MagnetometerMsg>) {
      UpdateCache(magnetometer_, value, now_ms);
    } else {
      static_assert(sizeof(T) == 0, "unsupported MAVLink telemetry cache type");
    }
  }
  template <typename T>
  void UpdateConfigCache(const T &cfg) {
    fc_config_.Adopt(cfg);
  }
  void ReportPanic(PanicSource source, uint32_t error_code);
  // Each edge of a calibration run, as the line an operator reads.
  void ReportCalProgress(const message::CalStatusMsg &msg);
  uint32_t GetRxPacketCount() const;
  uint32_t GetTxPacketCount() const;
  uint32_t GetRxHeartbeatCount() const;
  uint32_t GetTxHeartbeatCount() const;
  std::optional<LatestRcChannelsData> GetLatestRcChannelsData() const;
  std::optional<bool> PeerArmed(uint32_t now_ms) const;

 private:
  friend class System;
  void Init(const MavlinkConfig &cfg, IMavlinkTransport *transport,
            FcLink &fc_link);

  Mavlink();
  ~Mavlink();
  Mavlink(const Mavlink &) = delete;
  Mavlink &operator=(const Mavlink &) = delete;

  template <typename T>
  struct CachedValue {
    T value{};
    bool have_data = false;
    uint32_t update_ms = 0;
  };

  template <typename T>
  static void UpdateCache(CachedValue<T> &cache, const T &value,
                          uint32_t now_ms) {
    cache.value = value;
    cache.have_data = true;
    cache.update_ms = now_ms;
  }

  template <typename T>
  static std::optional<T> GetCachedValue(const CachedValue<T> &cache) {
    if (!cache.have_data) {
      return std::nullopt;
    }
    return cache.value;
  }

  // The ESP32 emits every one of these whether or not the FC is talking, so
  // a frame arriving says nothing about the peer's health. Each of the three
  // built from this cache has to ask its age itself.
  bool SystemStatusFresh(uint32_t now_ms) const;

  CachedValue<message::GpsData> gps_{};
  CachedValue<message::AttitudeMsg> attitude_{};
  CachedValue<message::RcChannelsMsg> rc_channels_{};
  CachedValue<message::SystemStatusMsg> system_status_{};
  CachedValue<message::VehicleStatusMsg> vehicle_status_{};
  CachedValue<message::MagnetometerMsg> magnetometer_{};
  CachedValue<message::EscTelemetryMsg> esc_telemetry_{};
  // Three of the STM32's VehicleStatus periods: at or below one, a heartbeat
  // lands on an expired report and names a mode the aircraft is not in.
  uint32_t peer_timeout_ms_ = 750;

  void ServiceRx();
  void HandleMessage(const mavlink_message_t &msg);
  void HandleParamMessage(const mavlink_message_t &msg);
  void HandleMissionMessage(const mavlink_message_t &msg);
  void HandleCommandMessage(const mavlink_message_t &msg);
  void HandleCommandLong(const mavlink_message_t &msg,
                         const mavlink_command_long_t &cmd);
  void HandleRequestMessage(const mavlink_command_long_t &cmd,
                            uint8_t source_system, uint8_t source_component);
  static bool IsDeclinedMessage(uint32_t message_id);
  bool IsTargetedToThisComponent(uint8_t target_system,
                                 uint8_t target_component) const {
    return target_system == cfg_.sysid &&
           (target_component == kMavlinkComponentId || target_component == 0 ||
            target_component == MAV_COMP_ID_ALL);
  }

  void NotifyGcsIssue(const char *text,
                      uint8_t severity = MAV_SEVERITY_WARNING);
  // For a gap in what this firmware implements: the same text is announced
  // once and then stays quiet, so a ground station that retries a request
  // does not warn and sound per attempt.
  void NotifyGcsIssueOnce(const char *text,
                          uint8_t severity = MAV_SEVERITY_WARNING);
  void LogUnhandledMessageOnce(const mavlink_message_t &msg);
  void LogUnhandledCommandOnce(uint16_t command, uint32_t detail,
                               const char *reason);
  // SYS_STATUS carries health as a live bitmap, so a fault the GCS blinked
  // past leaves no trace there. This turns each edge into a STATUSTEXT, which
  // a ground station keeps and timestamps -- the record the bitmap cannot be
  // without lying about what it means.
  void ReportSensorHealthChanges(const message::SystemStatusMsg &status);
  // CRC-32 of every gap already announced, so each distinct one is announced
  // once a boot. Keyed on the rendered line rather than on a msgid or command
  // number: the line already carries the whole identity of what is missing --
  // the parameter's name, the message a MAV_CMD asked for -- where the number
  // alone would fold two unrelated gaps into one report.
  std::array<uint32_t, 64> announced_gap_hashes_{};
  uint8_t announced_gap_count_ = 0;
  // Edge detection for ReportSensorHealthChanges. Held apart from the cache
  // because that one is also written by staleness, which is a change in what
  // is known rather than in what the vehicle reported.
  uint32_t last_sensor_health_ = 0;
  uint32_t last_sensor_present_ = 0;
  bool sensor_health_seen_ = false;
  // What the page has been told so far. It reads edges, not state: a repeated
  // line shows a pose twice. One set, because the flight computer runs one
  // calibration at a time.
  bool cal_running_ = false;
  uint8_t cal_sides_ = 0;
  uint8_t cal_announced_side_ = message::kAccelSideCount;
  uint8_t cal_last_progress_ = 0;

  struct CommandAck {
    uint16_t command = 0;
    uint8_t result = 0;
    uint8_t target_system = 0;
    uint8_t target_component = 0;
  };

  // Every mode this vehicle flies, and the only list of them: the heartbeat's
  // custom_mode and the AVAILABLE_MODES answer are both built from here, so a
  // mode added to one cannot go missing from the other.
  struct FlightModeInfo {
    FlightMode mode;
    // PX4's main_mode byte, from src/modules/commander/px4_custom_mode.h.
    // That header is not vendored, so these values are the only record of it
    // here -- and 32Raven declares a PX4 firmware class, so a ground station
    // reads the mode out of custom_mode exactly the way PX4 writes it.
    uint8_t px4_main_mode;
    // Reported verbatim in AVAILABLE_MODES. A ground station shows this only
    // for a non-standard mode, which both of these are: MAV_STANDARD_MODE has
    // no entry for a rate or attitude mode.
    const char *name;
    uint32_t properties;  // MAV_MODE_PROPERTY bits
  };

  // No MAV_MODE_PROPERTY_ADVANCED on either: a ground station hides an
  // advanced mode behind its own toggle, and Acro is the mode this aircraft
  // is flown in rather than an expert corner of the list.
  static constexpr std::array<FlightModeInfo, 2> kFlightModes = {{
      {FlightMode::kAcro, 5, "Acro", 0},
      {FlightMode::kStabilize, 7, "Stabilized", 0},
  }};

  struct AutopilotVersion {};

  // One mode index, 1-based as AVAILABLE_MODES counts them.
  struct AvailableModes {
    uint8_t mode_index = 1;
  };

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
                                   AvailableModes, MissionCount, StatusText>;

  // The buffer is private so Bytes() is the only way to reach the frame: a
  // span over the raw array would cover all MAVLINK_MAX_PACKET_LEN bytes and
  // put whatever the previous frame left behind on the wire.
  class TxFrameState {
   public:
    TxFrameState() = default;

    TxFrameState(const mavlink_message_t &msg, bool is_heartbeat) {
      Load(msg, is_heartbeat);
    }

    void Load(const mavlink_message_t &msg, bool is_heartbeat) {
      len_ =
          static_cast<uint16_t>(mavlink_msg_to_send_buffer(buf_.data(), &msg));
      is_hb_ = is_heartbeat;
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

  void QueueTxItem(const TxQueueItem &item);
  void QueueCommandAck(uint16_t command, uint8_t result, uint8_t target_system,
                       uint8_t target_component);
  void QueueAutopilotVersion();
  void QueueAvailableModes(uint8_t mode_index);
  void QueueMissionCount(uint8_t target_system, uint8_t target_component,
                         uint8_t mission_type);
  void QueueStatusText(const char *text, uint8_t severity = MAV_SEVERITY_INFO);
  bool SendStatusTextFrameNow(const StatusText &status,
                              bool require_link_enabled);
  std::optional<TxFrameState> StartQueuedTxWorkFrame();
  TxFrameState StartCommandAckFrame(const CommandAck &ack);
  TxFrameState StartAutopilotVersionFrame(const AutopilotVersion &work);
  TxFrameState StartAvailableModesFrame(const AvailableModes &work);
  TxFrameState StartMissionCountFrame(const MissionCount &work);
  TxFrameState StartStatusTextFrame(const StatusText &work);
  static constexpr uint8_t kTxWorkQueueDepth = 8;
  RingBuffer<TxQueueItem, kTxWorkQueueDepth + 1> tx_work_queue_{};
  TxFrameState tx_frame_{};

  void ServiceTx(uint32_t now_ms);
  void TransmitNextFrame(uint32_t now_ms);
  bool StartNextFrameIfIdle(uint32_t now_ms);
  void CompleteFrame(TxFrameState &frame, uint32_t now_ms);
  bool ShouldSendHbNow(uint32_t now_ms) const;
  void InitTxSchedule(uint32_t now_ms, bool force_heartbeat_due = false);
  std::optional<TxFrameState> StartNextScheduledFrame(uint32_t now_ms);
  TopicScheduler tx_scheduler_{};
  std::array<TopicState, kTxSlotCount> tx_slots_{};
  // Outside the scheduler because it measures from the frame actually leaving,
  // not from when the slot came due.
  uint32_t last_hb_done_ms_ = 0;
  uint32_t next_tx_poll_ms_ = 0;
  bool link_enabled_ = false;

  TxFrameState StartHeartbeatFrame(uint32_t now_ms);
  TxFrameState StartSysStatusFrame(uint32_t now_ms);
  std::optional<TxFrameState> StartGpsRawIntFrame();
  // Absent until a sample arrives, or the vector is too short to point.
  std::optional<float> MagneticHeading(float roll, float pitch) const;
  std::optional<TxFrameState> StartAttitudeFrame();
  std::optional<TxFrameState> StartGlobalPositionIntFrame();
  std::optional<TxFrameState> StartBatteryStatusFrame(uint32_t now_ms);
  std::optional<TxFrameState> StartRcChannelsFrame();
  std::optional<TxFrameState> StartEscStatusFrame();

  IMavlinkTransport *transport_ = nullptr;
  FcLink *fc_link_ = nullptr;
  MavlinkConfig cfg_{};
  FcConfigCache fc_config_{};
  MavlinkParamServer params_{};
  std::atomic<uint32_t> rx_packet_count_{0};
  std::atomic<uint32_t> tx_packet_count_{0};
  // Bumped before the totals above, so a reader that sees a total move already
  // sees whether that packet was a heartbeat.
  std::atomic<uint32_t> rx_heartbeat_count_{0};
  std::atomic<uint32_t> tx_heartbeat_count_{0};
};
