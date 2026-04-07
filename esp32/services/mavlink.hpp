#pragma once

#include <mavlink.h>

#include <atomic>
#include <array>
#include <cstdint>

#include "esp32_limits.hpp"
#include "../drivers/uart.hpp"
#include "message.hpp"  // for message::GpsData
#include "udp_server.hpp"

struct RcState {
  uint16_t channels[16]{};
  uint8_t rssi = 0;
};

class Mavlink {
 public:
  enum class Link : uint8_t {
    kRc,
    kUdp,
  };

  struct CommandLongEvent {
    Link link = Link::kUdp;
    mavlink_message_t msg{};
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
      } periods;

      struct Schedule {
        uint16_t hb_deadline_ms = 0;
        uint16_t gps_start_delay_ms = 0;
        uint16_t att_start_delay_ms = 0;
        uint16_t gpos_start_delay_ms = 0;
        uint16_t batt_start_delay_ms = 0;
      } schedule;
    } tx;

    struct Rc {
      struct Rx {
        uint16_t read_chunk_size = 0;
      } rx;

      Tx tx;
    } rc;
  };

  static Mavlink &GetInstance();

  void Init(const Config &cfg, UartRcRx *uart, UdpServer *udp);

  // RX: reads UART and parses inbound MAVLink (RC override only, for now)
  void Poll();

  // TX: called from a dedicated RTOS task at a steady tick (e.g. 10ms)
  void TxTick(uint32_t now_ms);

  // UDP: called from a dedicated RTOS task when MAVLink-over-Wi-Fi is enabled.
  void UdpTick(uint32_t now_ms);

  // Latest telemetry from STM32 (25Hz input ok; we downsample on TX)
  void OfferTelemetry(const message::GpsData &d, uint32_t now_ms);
  void QueueStatusText(const char *text, uint8_t severity = MAV_SEVERITY_INFO);
  bool PopCommandLongEvent(CommandLongEvent &event);
  void QueueCommandAck(Link link, uint16_t command, uint8_t result,
                       uint8_t target_system, uint8_t target_component);
  void QueueAutopilotVersion(Link link);
  uint32_t UdpRxPacketCount() const;
  uint32_t UdpTxPacketCount() const;

  const RcState &GetRcState() const { return rc_state_; }

 private:
  using TxConfig = Config::Tx;

  enum class RxSource : uint8_t {
    kRc,
    kUdp,
  };

  struct TxState {
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

  // ---------- Common ----------
  UartRcRx *uart_ = nullptr;
  UdpServer *udp_ = nullptr;
  Config cfg_{};
  void *command_queue_ = nullptr;  // QueueHandle_t

  // ---------- RX (RC only) ----------
  RcState rc_state_{};
  mavlink_message_t rx_msg_{};
  mavlink_status_t rx_status_{};
  static constexpr size_t kMaxRxReadChunkSize =
      esp32_limits::kMavlinkMaxRxReadChunkSize;
  std::array<uint8_t, kMaxRxReadChunkSize> rx_buf_{};
  std::array<uint16_t, 32> udp_logged_msgids_{};
  uint8_t udp_logged_msgid_count_ = 0;
  std::atomic<uint32_t> udp_rx_packet_count_{0};
  std::atomic<uint32_t> udp_tx_packet_count_{0};

  void HandleRxByte(uint8_t b);
  void HandleMessage(const mavlink_message_t &msg, RxSource source);
  void MaybeLogUdpMessage(const mavlink_message_t &msg);
  void QueueCommandAck(TxState &tx, uint16_t command, uint8_t result,
                       uint8_t target_system, uint8_t target_component);
  TxState &TxForLink(Link link);
  void StartCommandAckFrame(TxState &tx);
  void StartAutopilotVersionFrame(TxState &tx);
  void StartMissionCountFrame(TxState &tx);
  void StartStatusTextFrame(TxState &tx);
  void StartParamValueFrame(TxState &tx);

  // ---------- TX: atomic frame writes ----------
  TxState rc_tx_{};
  TxState udp_tx_{};
  void ServiceInFlightTx(TxState &tx);

  // ---------- Latest-only telemetry cache ----------
  message::GpsData latest_{};
  bool have_latest_ = false;
  uint32_t latest_update_ms_ = 0;

  // Helpers

  bool ShouldSendHbNow(const TxState &tx, const TxConfig &cfg_tx,
                       uint32_t now_ms) const;
  void ArmFirstSchedule(TxState &tx, const TxConfig &cfg_tx, uint32_t now_ms);

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
