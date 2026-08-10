// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once

#include <cstdint>

#include "vehicle_state.hpp"

class Battery {
 public:
  struct Config {
    uint32_t sample_period_us;
    uint16_t adc_reference_mv;
    // ADC_CCR.ADCPRE field value, derived from PCLK2 by the config generator.
    uint8_t adc_prescaler_bits;
    uint8_t voltage_adc_channel;
    uint8_t current_adc_channel;
    uint8_t oversample_count;
    float filter_alpha;
    uint16_t adc_timeout_us;
    uint32_t voltage_multiplier_milli;
    int32_t voltage_offset_mv;
    uint32_t current_scale_ma_per_v;
    int32_t current_offset_mv;
    uint16_t current_deadband_ma;
    uint8_t cell_count;
    uint16_t cell_empty_mv;
    uint16_t cell_full_mv;
    uint32_t initial_mah_drawn;
  };

  static Battery &GetInstance();

  void Poll(uint32_t now_us);
  const BatteryData &GetData() const { return data_; }

  // Last raw ADC counts, before any scaling. Calibrating the voltage
  // multiplier / current scale means comparing the pin against a known
  // reference, and every other output is already derived through the
  // constants under test.
  uint16_t LastVoltageRaw() const { return last_voltage_raw_; }
  uint16_t LastCurrentRaw() const { return last_current_raw_; }
  uint32_t AdcErrorCount() const { return adc_error_count_; }

 private:
  friend class System;
  void Init(const Config &cfg);

  Battery() = default;
  ~Battery() = default;
  Battery(const Battery &) = delete;
  Battery &operator=(const Battery &) = delete;

  struct AdcPair {
    uint16_t voltage_raw;
    uint16_t current_raw;
    bool valid;
  };

  void InitAdc();
  AdcPair ReadAdcPair();
  void PublishSample(uint32_t now_us, uint16_t voltage_raw,
                     uint16_t current_raw);
  uint8_t EstimatePercentage(float voltage_v) const;

  Config cfg_{};
  BatteryData data_{};
  uint32_t last_sample_us_ = 0;
  uint32_t last_integrator_us_ = 0;
  float filtered_voltage_v_ = 0.0f;
  float filtered_current_a_ = 0.0f;
  float mah_drawn_ = 0.0f;
  uint16_t last_voltage_raw_ = 0;
  uint16_t last_current_raw_ = 0;
  bool filter_valid_ = false;
  bool initialized_ = false;
  uint32_t adc_error_count_ = 0;
};
