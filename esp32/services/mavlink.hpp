#pragma once

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

  using MsgHandler = void (*)(const mavlink_message_t &msg);
  void SetHandler(MsgHandler handler) { handler_ = handler; }

  const RcData &GetRc() const { return rc_; }

private:
  MsgHandler handler_ = nullptr;
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
};
