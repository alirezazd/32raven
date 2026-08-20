// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstdint>
#include <optional>

#include "shared_state.hpp"

class Battery {
 public:
  // The F407 ADC is 12-bit and the driver never programs a lower resolution,
  // so full scale is fixed rather than configured.
  static constexpr uint8_t kAdcResolutionBits = 12;
  static constexpr uint16_t kAdcMaxRaw =
      (uint16_t{1} << kAdcResolutionBits) - uint16_t{1};

  // A scale for a pin nobody reads calibrates nothing, so these travel
  // together or not at all.
  struct CurrentSense {
    uint8_t adc_channel;
    uint32_t scale_ma_per_v;
    int32_t offset_mv;
    uint16_t deadband_ma;
  };

  struct Config {
    uint32_t sample_period_us;
    uint16_t adc_reference_mv;
    // ADC_CCR.ADCPRE field value, derived from PCLK2 by the config generator.
    uint8_t adc_prescaler_bits;
    uint8_t voltage_adc_channel;
    uint8_t oversample_count;
    float filter_alpha;
    uint16_t adc_timeout_us;
    uint32_t voltage_multiplier_milli;
    int32_t voltage_offset_mv;
    // Absent on a board with no sensor, which is what makes the driver sample
    // the voltage channel alone and publish no current at all.
    std::optional<CurrentSense> current_sense;
    uint8_t cell_count;
    uint16_t cell_empty_mv;
    uint16_t cell_full_mv;
    uint32_t initial_mah_drawn;
  };

  static Battery &GetInstance();

  // Non-blocking: starts one conversion per call and collects it on a later
  // one, so a sample spans two calls per oversample rather than completing
  // inside this one.
  void Poll(uint32_t now_us);

 private:
  friend class System;
  void Init(const Config &cfg, SharedState &blackboard);

  Battery() = default;
  ~Battery() = default;
  Battery(const Battery &) = delete;
  Battery &operator=(const Battery &) = delete;

  void InitAdc();
  void StartConversion(uint32_t now_us);
  bool CollectConversion(uint32_t now_us);
  void PublishSample(uint32_t now_us, uint16_t voltage_raw,
                     uint16_t current_raw);
  uint8_t EstimatePercentage(float voltage_v) const;
  bool IsVoltageConversion() const;
  uint8_t ConversionsPerSample() const;

  std::optional<float> Sensed(float value) const {
    return cfg_.current_sense ? std::optional<float>{value} : std::nullopt;
  }

  Config cfg_{};
  SharedState *blackboard_ = nullptr;
  uint32_t last_sample_us_ = 0;
  uint32_t last_integrator_us_ = 0;
  uint32_t conversion_start_us_ = 0;
  uint32_t voltage_acc_ = 0;
  uint32_t current_acc_ = 0;
  float filtered_voltage_v_ = 0.0f;
  float filtered_current_a_ = 0.0f;
  float mah_drawn_ = 0.0f;
  // Counts conversions within a sample, not oversample iterations: even is the
  // voltage channel, odd is current. With no current sense every one is
  // voltage, so a sample is half as many.
  uint8_t conversion_index_ = 0;
  bool conversion_in_flight_ = false;
  bool sample_active_ = false;
  bool filter_valid_ = false;
  bool initialized_ = false;
};
