// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <array>
#include <cstdint>
#include <optional>

#include "ring_buffer.hpp"
#include "shared_state.hpp"

class EscTelemetry {
 public:
  static constexpr uint8_t kMotorCount = 4;

  // Fixed by the KISS protocol, not by this board. 8N1, so ten bit times per
  // byte; a timeout at or below one frame time expires mid-reply every time.
  static constexpr uint32_t kBaudRate = 115200;
  static constexpr uint8_t kKissFrameSize = 10;
  static constexpr uint32_t kMinResponseTimeoutUs =
      ((kKissFrameSize * 10u * 1000000u) + kBaudRate - 1u) / kBaudRate;

  struct Config {
    // An AM32 target that names no current pin still reports one, scaled from
    // an unrelated ADC input -- so off, both it and the consumption derived
    // from it are published as absent.
    bool has_current;
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

  // The settings an ESC reports about itself, decoded only once eeprom_version
  // names a layout whose offsets we have checked.
  struct Info {
    uint8_t eeprom_version = 0;
    uint8_t firmware_major = 0;
    uint8_t firmware_minor = 0;
    uint8_t motor_poles = 0;
    uint8_t motor_kv_raw = 0;
    // AM32's inputType enum: 0 auto, 1 dshot, 2 servo, 3 serial, 4 dshot with
    // EDT arming, 5 DroneCAN.
    uint8_t input_type = 0;
    bool reversed = false;
    bool bidirectional = false;
    bool valid = false;
  };

  static EscTelemetry &GetInstance();

  void ExpectMotor(uint8_t motor_index, uint32_t now_us);
  void ExpectInfo(uint8_t motor_index, uint32_t now_us);
  Info GetInfo(uint8_t motor_index) const;
  void Poll(uint32_t now_us);
  bool HasValidSample() const { return valid_mask_ != 0u; }
  uint8_t ValidMask() const { return valid_mask_; }
  bool IsExpectingInfo() const {
    return expected_frame_size_ == kInfoFrameSize;
  }
  bool IsInitialized() const { return initialized_; }

  void OnUartInterrupt();
  void OnRxHalfCplt();
  void OnRxCplt();
  void HandleRxDmaError(uint32_t isr_flags);

 private:
  friend class System;
  void Init(const Config &cfg, SharedState &blackboard);

  // Wire units convert here rather than at parse time, so `samples_` stays the
  // shape the frames arrive in and only the published view carries volts.
  EscTelemetryData BuildEscTelemetryData() const;
  void PublishIfChanged();

  EscTelemetry() = default;
  ~EscTelemetry() = default;
  EscTelemetry(const EscTelemetry &) = delete;
  EscTelemetry &operator=(const EscTelemetry &) = delete;

  static constexpr uint16_t kRxDmaSize = 64;
  static constexpr uint16_t kRxRingSize = 256;
  // 48 settings bytes plus a CRC8, the whole of AM32's makeInfoPacket.
  static constexpr uint8_t kInfoFrameSize = 49;
  static constexpr uint8_t kNoMotor = 0xFF;
  // Spans the whole command burst, because the window opens with the first
  // frame and AM32 replies on the sixth of ten. Only bounds how long a lost
  // reply keeps the parser mis-sized.
  static constexpr uint32_t kInfoTimeoutUs = 60000;

  void ConfigureUart();
  void StartRxDma();
  void DrainRx();
  void ProcessByte(uint8_t byte, uint32_t now_us);
  void PublishFrame(uint32_t now_us);
  void PublishInfo();
  bool ExpectedMotorActive(uint32_t now_us) const;

  Config cfg_{};
  bool initialized_ = false;

  uint8_t rx_dma_buf_[kRxDmaSize] = {};
  uint16_t rx_last_pos_ = 0;
  RingBuffer<uint8_t, kRxRingSize> rx_ring_{};

  uint8_t frame_buf_[kInfoFrameSize] = {};
  uint8_t frame_len_ = 0;
  uint8_t expected_frame_size_ = kKissFrameSize;

  SharedState *blackboard_ = nullptr;
  std::array<Sample, kMotorCount> samples_{};
  std::array<Info, kMotorCount> info_{};
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
