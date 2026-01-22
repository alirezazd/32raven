#pragma once

#include "message.hpp"
#include "uart.hpp"

class FlightController {
public:
  struct Config {};

  static FlightController &GetInstance() {
    static FlightController instance;
    return instance;
  }

  void Init(const Config &cfg, Uart *uart);
  void Poll(uint32_t now);

  // API
  bool SendPacket(const message::Packet &pkt);
  bool PerformHandshake(); // Blocking handshake (Ping/Pong)

  // Checks if a new packet was received and retrieves it.
  // Clears the "packet ready" flag.
  bool GetPacket(message::Packet &out_pkt);

  // Status
  bool IsConnected() const;

private:
  FlightController() = default;
  ~FlightController() = default;
  FlightController(const FlightController &) = delete;
  FlightController &operator=(const FlightController &) = delete;

  Uart *uart_ = nullptr;
  Config cfg_;
  bool initialized_ = false;

  // RX Data
  message::Packet rx_pkt_out_{};
  bool rx_pkt_ready_ = false;

  // Internal Logic State
  enum class LogicState {
    kInit,
    kWaitSync,
    kActive
  } state_ = LogicState::kInit;
  uint32_t last_activity_ = 0;

  // Epistole Parser State
  enum class RxState { kMagic1, kMagic2, kId, kLen, kPayload, kCrc1, kCrc2 };
  RxState rx_state_ = RxState::kMagic1;
  uint8_t rx_idx_ = 0;
  uint8_t rx_len_ = 0;
  struct {
    uint8_t id;
    uint8_t len;
    uint8_t payload[255];
    uint16_t crc;
  } rx_pkt_internal_;
};
