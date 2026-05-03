#pragma once

#include <array>
#include <cstdint>

class DShotCodec {
 public:
  static constexpr uint8_t kMotorCount = 4;
  static constexpr uint16_t kCommandMax = 47;
  static constexpr uint16_t kMotorStop = 0;
  static constexpr uint16_t kThrottleMin = 48;
  static constexpr uint16_t kThrottleMax = 2047;

  using MotorValues = std::array<uint16_t, kMotorCount>;
  using TelemetryRequests = std::array<bool, kMotorCount>;

  struct Config {
    uint8_t gap_bits;
  };

  static DShotCodec &GetInstance() {
    static DShotCodec instance;
    return instance;
  }

  void Init(const Config &cfg);
  bool Write(const MotorValues &motor, bool telemetry = false);
  bool Write(const MotorValues &motor, const TelemetryRequests &telemetry);
  bool IsInitialized() const { return initialized_; }

 private:
  DShotCodec() = default;
  ~DShotCodec() = default;
  DShotCodec(const DShotCodec &) = delete;
  DShotCodec &operator=(const DShotCodec &) = delete;

  static uint16_t MakePacket(uint16_t value, bool telemetry);

  static constexpr uint16_t kFrameBits = 16;
  static constexpr uint16_t kMaxGapBits = 8;
  static constexpr uint16_t kMaxBits = kFrameBits + kMaxGapBits;

  uint16_t buf_[kMaxBits * kMotorCount] = {};
  Config cfg_{};
  bool initialized_ = false;
};
