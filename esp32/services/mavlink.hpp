#pragma once

#include "../drivers/uart.hpp"
#include "message.hpp" // for message::GpsData
#include <cstdint>
#include <mavlink.h>

struct RcData {
  uint16_t channels[18]{};
  uint8_t count = 0;
  uint32_t last_update = 0;
};

class Mavlink {
public:
  struct Config {
    uint8_t sysid = 0;
    uint8_t compid = 0;

    // Telemetry periods in ms (0 = disabled)
    // Budget: ~125 bytes/sec for ELRS 4-channel mode
    uint16_t hb_period_ms = 0;   // HEARTBEAT
    uint16_t gps_period_ms = 0;  // GPS_RAW_INT
    uint16_t att_period_ms = 0;  // ATTITUDE
    uint16_t gpos_period_ms = 0; // GLOBAL_POSITION_INT
    uint16_t batt_period_ms = 0; // BATTERY_STATUS
  };

  static Mavlink &GetInstance();

  void Init(const Config &cfg, Uart *uart);

  // RX: reads UART and parses inbound MAVLink (RC override only, for now)
  void Poll(uint32_t now_ms);

  // TX: called from a dedicated RTOS task at a steady tick (e.g. 10ms)
  void TxTick(uint32_t now_ms);

  // Latest telemetry from STM32 (25Hz input ok; we downsample on TX)
  void OfferTelemetry(const message::GpsData &d, uint32_t now_ms);

  const RcData &GetRc() const { return rc_; }

private:
  Mavlink();
  ~Mavlink();
  Mavlink(const Mavlink &) = delete;
  Mavlink &operator=(const Mavlink &) = delete;

  // ---------- Common ----------
  Uart *uart_ = nullptr;
  bool initialized_ = false;
  Config cfg_{};

  // ---------- RX (RC only) ----------
  RcData rc_{};
  mavlink_message_t rx_msg_{};
  mavlink_status_t rx_status_{};

  void HandleRxByte(uint8_t b, uint32_t now_ms);
  void HandleMessage(const mavlink_message_t &msg, uint32_t now_ms);

  // ---------- TX: atomic frame writes ----------
  uint8_t tx_buf_[MAVLINK_MAX_PACKET_LEN]{};
  uint16_t tx_len_ = 0;
  uint16_t tx_sent_ = 0;

  bool tx_is_hb_ = false; // only to stamp hb timing
  void ServiceInFlightTx();

  // ---------- TX: Heartbeat timing ----------
  static constexpr uint32_t kHbDeadlineMs = 1500; // max allowed gap
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
