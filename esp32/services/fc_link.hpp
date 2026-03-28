#pragma once

#include <cstddef>
#include <cstdint>

#include "mavlink.hpp"
#include "message.hpp"
#include "uart.hpp"

class FcLink {
public:
  struct Config {
    uint8_t telemetry_rate_hz = 25;
    uint16_t handshake_timeout_ms = 2000;
    uint16_t handshake_retry_period_ms = 20;
    uint16_t rx_read_chunk_size = 128;
    uint8_t rx_queue_depth = 10;
  };

  static FcLink &GetInstance() {
    static FcLink instance;
    return instance;
  }

  void Init(const Config &cfg, UartFcLink *uart);
  void Poll(uint32_t now);

  // API
  bool SendPacket(const message::Packet &pkt);
  bool SendRcState(const RcState &state);
  bool Dispatch(const message::Packet &pkt);
  bool PerformHandshake(); // Blocking handshake (Ping/Pong)

  // Checks if a new packet was received and retrieves it.
  // Clears the "packet ready" flag.
  bool GetPacket(message::Packet &out_pkt);

  // Status
  bool IsConnected() const;

private:
  FcLink() = default;
  ~FcLink() = default;
  FcLink(const FcLink &) = delete;
  FcLink &operator=(const FcLink &) = delete;

  UartFcLink *uart_ = nullptr;
  Config cfg_;
  bool initialized_ = false;

  static constexpr size_t kMaxRxReadChunkSize = message::kMaxPayload;
  void *packet_queue_ = nullptr; // QueueHandle_t

  // Internal Logic State
  enum class LogicState {
    kInit,
    kWaitSync,
    kActive
  } state_ = LogicState::kInit;
  uint32_t last_activity_ = 0;
  // TODO: use fixed smaple rate and get rid of these
  uint32_t last_rc_forward_ms_ = 0;
  uint32_t last_radio_status_forward_ms_ = 0;

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
