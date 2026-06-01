#pragma once

#include <array>
#include <cstdint>

#include "ring_buffer.hpp"

class EscTelemetry {
 public:
  static constexpr uint8_t kMotorCount = 4;

  struct Config {
    uint32_t baud_rate;
    uint8_t motor_pole_count;
    uint32_t response_timeout_us;
  };

  struct Sample {
    uint32_t timestamp_us = 0;
    uint16_t voltage_centivolts = 0;
    uint16_t current_centiamps = 0;
    uint16_t consumption_mah = 0;
    uint16_t erpm_hundreds = 0;
    uint32_t electrical_rpm = 0;
    uint32_t rpm = 0;
    int16_t temperature_c = 0;
    bool valid = false;
  };

  struct Snapshot {
    std::array<Sample, kMotorCount> motors{};
    uint8_t valid_mask = 0;
    uint32_t frame_count = 0;
    uint32_t crc_error_count = 0;
    uint32_t unassigned_frame_count = 0;
    uint32_t rx_drop_bytes = 0;
    uint32_t rx_dma_error_count = 0;
    uint32_t uart_error_count = 0;
  };

  static EscTelemetry &GetInstance();

  void Init(const Config &cfg);
  void ExpectMotor(uint8_t motor_index, uint32_t now_us);
  void Poll(uint32_t now_us);
  Snapshot GetSnapshot() const;
  bool HasValidSample() const { return valid_mask_ != 0u; }
  bool IsInitialized() const { return initialized_; }

  void OnUartInterrupt();
  void OnRxHalfCplt();
  void OnRxCplt();
  void HandleRxDmaError(uint32_t isr_flags);

 private:
  EscTelemetry() = default;
  ~EscTelemetry() = default;
  EscTelemetry(const EscTelemetry &) = delete;
  EscTelemetry &operator=(const EscTelemetry &) = delete;

  static constexpr uint16_t kRxDmaSize = 64;
  static constexpr uint16_t kRxRingSize = 256;
  static constexpr uint8_t kKissFrameSize = 10;
  static constexpr uint8_t kNoMotor = 0xFF;

  void ConfigureUart();
  void StartRxDma();
  void DrainRx();
  void ProcessByte(uint8_t byte, uint32_t now_us);
  void PublishFrame(uint32_t now_us);
  bool ExpectedMotorActive(uint32_t now_us) const;

  Config cfg_{};
  bool initialized_ = false;

  uint8_t rx_dma_buf_[kRxDmaSize] = {};
  uint16_t rx_last_pos_ = 0;
  RingBuffer<uint8_t, kRxRingSize> rx_ring_{};

  uint8_t frame_buf_[kKissFrameSize] = {};
  uint8_t frame_len_ = 0;

  std::array<Sample, kMotorCount> samples_{};
  uint8_t valid_mask_ = 0;
  uint8_t expected_motor_ = kNoMotor;
  uint32_t expected_since_us_ = 0;

  volatile uint32_t rx_drop_bytes_ = 0;
  volatile uint32_t rx_dma_error_count_ = 0;
  volatile uint32_t uart_ore_error_count_ = 0;
  volatile uint32_t uart_fe_error_count_ = 0;
  volatile uint32_t uart_ne_error_count_ = 0;
  volatile uint32_t uart_pe_error_count_ = 0;

  uint32_t frame_count_ = 0;
  uint32_t crc_error_count_ = 0;
  uint32_t unassigned_frame_count_ = 0;
};
