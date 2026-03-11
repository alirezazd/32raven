#pragma once

#include "command_handler.hpp"
#include "message.hpp"
#include "ring_buffer.hpp"
#include "vehicle_state.hpp"

struct AppContext;

class FcLink {
public:
  static FcLink &GetInstance() {
    static FcLink instance;
    return instance;
  }

  void Init(AppContext *ctx);
  void Poll(size_t rx_budget = 128, size_t tx_budget = 64);

  // Sending Logic
  bool Send(const message::Packet &pkt);

  // Convenience for GpsData from Blackboard
  // Convenience for GpsData from Blackboard
  void SendGps(const GpsData &data, const BatteryData &bat);

  // Convenience for one IMU sample.
  void SendImu(uint64_t timestamp_us, const float accel[3], const float gyro[3]);

  // Send Log Message
  void SendLog(const char *format, ...);
  
  // Send Binary Log (format string + raw args, ESP32 does formatting)
  void SendLogBinary(uint8_t fmt_id, uint8_t argc, const uint32_t *args);

private:
  FcLink() = default;
  ~FcLink() = default;
  FcLink(const FcLink &) = delete;
  FcLink &operator=(const FcLink &) = delete;

  AppContext *ctx_ = nullptr;

  // RX Parsing State
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

  // TX Buffer
  static constexpr size_t kTxBufSize = 512;
  RingBuffer<uint8_t, kTxBufSize> tx_rb_;
};
