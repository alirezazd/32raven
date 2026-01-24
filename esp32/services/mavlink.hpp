#pragma once

#include "message.hpp"
#include "ring_buffer.hpp"
#include "uart.hpp"
#include <mavlink.h>

class Mavlink {
public:
  struct Config {};

  struct RcData {
    uint16_t channels[18];
    uint8_t count;
    uint32_t last_update;
  };

  static Mavlink &GetInstance() {
    static Mavlink instance;
    return instance;
  }

  void Init(const Config &cfg, Uart *uart);
  void Poll(uint32_t now);

  // Pop a message from the internal buffer. Returns true if msg valid.
  bool GetMessage(mavlink_message_t &msg);

  const RcData &GetRc() const { return rc_; }

private:
  static constexpr size_t kQueueSize = 10;
  RingBuffer<mavlink_message_t, kQueueSize> queue_;
  Mavlink() = default;
  ~Mavlink() = default;
  Mavlink(const Mavlink &) = delete;
  Mavlink &operator=(const Mavlink &) = delete;

  // UART Interface (Exclusive ownership)
  Uart *uart_ = nullptr;
  Config cfg_;
  bool initialized_ = false;

  // MAVLink Parser State
  mavlink_message_t rx_msg_;
  mavlink_status_t rx_status_;

  // Data
  RcData rc_{};

  // Heartbeat
  uint32_t last_heartbeat_ = 0;
  void SendHeartbeat();

public:
  void SendSystemTime(const message::GpsData &t);
  void SendGpsRawInt(const message::GpsData &t);
  void SendGlobalPositionInt(const message::GpsData &t);
};
