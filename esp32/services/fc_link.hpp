#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>

#include "mavlink.hpp"
#include "message.hpp"
#include "timebase.hpp"
#include "uart.hpp"

class FcLink {
 public:
  struct Config {
    uint8_t rc_forward_rate_hz = 0;
    uint16_t handshake_attempts = 0;
    uint16_t handshake_retry_period_ms = 0;
  };

  static FcLink &GetInstance() {
    static FcLink instance;
    return instance;
  }

  void Init(const Config &cfg, UartFcLink *uart);
  void Poll();
  void ForwardRcState(const RcState &rc_state);
  void PerformHandshake();  // Blocking handshake (Ping/Pong)

  std::optional<message::Packet> PopPacket();

 private:
  bool FinishRxPacket();
  FcLink() = default;
  ~FcLink() = default;
  FcLink(const FcLink &) = delete;
  FcLink &operator=(const FcLink &) = delete;

  UartFcLink *uart_ = nullptr;
  Config cfg_;
  static constexpr size_t kMaxRxReadBufferSize =
      message::kMaxPayload + message::kPacketOverhead;
  TimeMs next_rc_forward_ms_ = 0;

  enum class RxState { kMagic1, kMagic2, kId, kLen, kPayload, kCrc1, kCrc2 };
  RxState rx_state_ = RxState::kMagic1;
  uint8_t rx_idx_ = 0;
  uint8_t rx_len_ = 0;
  struct {
    uint8_t id;
    uint8_t len;
    uint8_t payload[message::kMaxPayload];
    uint16_t crc;
  } rx_pkt_internal_;
  bool SendPacket(const message::Packet &pkt);
};
