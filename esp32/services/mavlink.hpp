#pragma once

#include "../drivers/uart.hpp"
#include "message.hpp" // for message::GpsData
#include <array>
#include <cstdint>
#include <mavlink.h>

struct RcData {
  uint16_t channels[16]{};
  uint32_t last_update = 0;
};

struct RadioStatusData {
  uint32_t last_update = 0;
  uint8_t rssi = 0;
};

struct RcState {
  RcData rc{};
  RadioStatusData radio_status{};
};

class Mavlink {
public:
  struct Config {
    struct Identity {
      uint8_t sysid = 0;
      uint8_t compid = 0;
    } identity;

    struct Rx {
      uint16_t read_chunk_size = 0;
    } rx;

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

  void Init(const Config &cfg, UartEp2 *uart);

  // RX: reads UART and parses inbound MAVLink (RC override only, for now)
  void Poll(uint32_t now_ms);

  // TX: called from a dedicated RTOS task at a steady tick (e.g. 10ms)
  void TxTick(uint32_t now_ms);

  // Latest telemetry from STM32 (25Hz input ok; we downsample on TX)
  void OfferTelemetry(const message::GpsData &d, uint32_t now_ms);

  const RcState &GetRcState() const { return rc_state_; }

private:
  Mavlink();
  ~Mavlink();
  Mavlink(const Mavlink &) = delete;
  Mavlink &operator=(const Mavlink &) = delete;

  // ---------- Common ----------
  UartEp2 *uart_ = nullptr;
  bool initialized_ = false;
  Config cfg_{};

  // ---------- RX (RC only) ----------
  RcState rc_state_{};
  mavlink_message_t rx_msg_{};
  mavlink_status_t rx_status_{};
  static constexpr size_t kMaxRxReadChunkSize = 256;
  std::array<uint8_t, kMaxRxReadChunkSize> rx_buf_{};

  void HandleRxByte(uint8_t b, uint32_t now_ms);
  void HandleMessage(const mavlink_message_t &msg, uint32_t now_ms);

  // ---------- TX: atomic frame writes ----------
  uint8_t tx_buf_[MAVLINK_MAX_PACKET_LEN]{};
  uint16_t tx_len_ = 0;
  uint16_t tx_sent_ = 0;

  bool tx_is_hb_ = false; // only to stamp hb timing
  void ServiceInFlightTx();

  // ---------- TX: Heartbeat timing ----------
  uint32_t last_hb_done_ms_ = 0;

  // ---------- Latest-only telemetry cache ----------
  message::GpsData latest_{};
  bool have_latest_ = false;
  uint32_t latest_update_ms_ = 0;

  // ---------- Stream scheduler ----------
  bool schedule_armed_ = false;
  uint32_t next_hb_ms_ = 0;
  uint32_t next_gps_ms_ = 0;
  uint32_t next_att_ms_ = 0;
  uint32_t next_gpos_ms_ = 0;
  uint32_t next_batt_ms_ = 0;

  // Helpers

  bool ShouldSendHbNow(uint32_t now_ms) const;
  void ArmFirstSchedule(uint32_t now_ms);

  // Frame builders (only the ELRS-supported set)
  void StartHeartbeatFrame();
  void StartGpsRawIntFrame();
  void StartAttitudeFrame();
  void StartGlobalPositionIntFrame();
  void StartBatteryStatusFrame();

  // Pick next stream to send when idle (hb has priority)
  void StartNextScheduledFrame(uint32_t now_ms);
};
