// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "battery.hpp"

#include "error_code.hpp"
#include "panic.hpp"
#include "shared_state.hpp"
#include "stm32f4xx.h"

namespace {

constexpr float kMilli = 1000.0f;
constexpr float kMicrosecondsPerMahAtOneAmp = 3600000.0f;
// SMPx encoding. Both inputs are low-impedance -- a resistive divider and an
// op-amp current sense -- so 84 cycles settles them with margin. 480, the
// maximum, is for high-impedance sources and buys nothing here.
constexpr uint32_t kAdcSampleTime84Cycles = 4u;

void SetAdcSampleTime(uint8_t channel, uint32_t sample_time) {
  const uint32_t shift = static_cast<uint32_t>(channel % 10u) * 3u;
  if (channel <= 9u) {
    ADC1->SMPR2 &= ~(0x7u << shift);
    ADC1->SMPR2 |= sample_time << shift;
  } else {
    ADC1->SMPR1 &= ~(0x7u << shift);
    ADC1->SMPR1 |= sample_time << shift;
  }
}

float ClampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

}  // namespace

Battery &Battery::GetInstance() {
  static Battery instance;
  return instance;
}

void Battery::Init(const Config &cfg, SharedState &blackboard) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kAdcInitFailed);
  }

  if (cfg.sample_period_us == 0u || cfg.adc_reference_mv == 0u ||
      cfg.oversample_count == 0u || cfg.filter_alpha <= 0.0f ||
      cfg.filter_alpha > 1.0f || cfg.adc_timeout_us == 0u ||
      cfg.voltage_multiplier_milli == 0u || cfg.cell_count == 0u ||
      cfg.cell_empty_mv >= cfg.cell_full_mv) {
    Panic(ErrorCode::Stm32::kAdcInitFailed);
  }

  if (cfg.current_sense &&
      (cfg.current_sense->scale_ma_per_v == 0u ||
       cfg.voltage_adc_channel == cfg.current_sense->adc_channel)) {
    Panic(ErrorCode::Stm32::kAdcInitFailed);
  }

  cfg_ = cfg;
  blackboard_ = &blackboard;

  mah_drawn_ = static_cast<float>(cfg_.initial_mah_drawn);
  // Seeded before the first conversion lands so a reader between here and then
  // sees the mAh carried over rather than zero.
  blackboard_->UpdateBattery(BatteryData{.voltage = 0.0f,
                                         .current = Sensed(0.0f),
                                         .mah_drawn = Sensed(mah_drawn_),
                                         .percentage = 0u});

  InitAdc();
  initialized_ = true;
}

// One conversion in flight at a time, started here and collected on a later
// call. A conversion takes ~5 us and the main tick comes back in ~1 ms, so the
// result is always waiting by the next call and nothing ever spins on EOC.
void Battery::Poll(uint32_t now_us) {
  if (conversion_in_flight_) {
    if (!CollectConversion(now_us)) {
      return;
    }
  } else if (!sample_active_) {
    if (last_sample_us_ != 0u &&
        static_cast<uint32_t>(now_us - last_sample_us_) <
            cfg_.sample_period_us) {
      return;
    }

    // Stamped at the start rather than the publish, so the period measures
    // sample to sample and stays the one filter_alpha was solved against.
    last_sample_us_ = now_us;
    sample_active_ = true;
    conversion_index_ = 0;
    voltage_acc_ = 0;
    current_acc_ = 0;
  }

  StartConversion(now_us);
}

void Battery::InitAdc() {
  // GPIO/port clocks for kEscVba/kEscCur are set by GPIO::Init(); only the
  // ADC1 clock + peripheral programming belong here.
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  (void)RCC->APB2ENR;

  ADC->CCR =
      (ADC->CCR & ~ADC_CCR_ADCPRE) |
      (static_cast<uint32_t>(cfg_.adc_prescaler_bits) << ADC_CCR_ADCPRE_Pos);

  // Single conversion per trigger, channel selected per conversion. A two
  // channel scan would start the second the instant the first finished, giving
  // DR a read deadline of one conversion; Poll collects a whole main tick
  // later, so it could never meet one. Missing it latches OVR, which stops EOC
  // for good.
  ADC1->CR1 = 0;
  ADC1->CR2 = ADC_CR2_EOCS;
  ADC1->SQR1 = 0;  // L = 0: one regular conversion.
  ADC1->SQR2 = 0;
  ADC1->SQR3 = static_cast<uint32_t>(cfg_.voltage_adc_channel);

  SetAdcSampleTime(cfg_.voltage_adc_channel, kAdcSampleTime84Cycles);
  if (cfg_.current_sense) {
    SetAdcSampleTime(cfg_.current_sense->adc_channel, kAdcSampleTime84Cycles);
  }

  ADC1->SR = 0;
  ADC1->CR2 |= ADC_CR2_ADON;
}

bool Battery::IsVoltageConversion() const {
  return !cfg_.current_sense || (conversion_index_ % 2u) == 0u;
}

uint8_t Battery::ConversionsPerSample() const {
  return static_cast<uint8_t>(cfg_.oversample_count *
                              (cfg_.current_sense ? 2u : 1u));
}

void Battery::StartConversion(uint32_t now_us) {
  uint8_t channel = cfg_.voltage_adc_channel;
  if (cfg_.current_sense && !IsVoltageConversion()) {
    channel = cfg_.current_sense->adc_channel;
  }

  ADC1->SQR3 = static_cast<uint32_t>(channel);
  // Clears EOC and OVR together, so a conversion abandoned by a previous
  // sample cannot leave the peripheral latched.
  ADC1->SR = 0;
  ADC1->CR2 |= ADC_CR2_SWSTART;
  conversion_start_us_ = now_us;
  conversion_in_flight_ = true;
}

// True when the result landed and the next conversion may start.
bool Battery::CollectConversion(uint32_t now_us) {
  if ((ADC1->SR & ADC_SR_EOC) == 0u) {
    // A conversion is ~5 us against a ~1 ms call period, so an unfinished one
    // means a stopped ADC rather than a slow one.
    if (static_cast<uint32_t>(now_us - conversion_start_us_) >=
        cfg_.adc_timeout_us) {
      conversion_in_flight_ = false;
      sample_active_ = false;
    }
    return false;
  }

  const auto raw = static_cast<uint16_t>(ADC1->DR & ADC_DR_DATA);
  conversion_in_flight_ = false;
  if (IsVoltageConversion()) {
    voltage_acc_ += raw;
  } else {
    current_acc_ += raw;
  }
  conversion_index_++;

  if (conversion_index_ < ConversionsPerSample()) {
    return true;
  }

  sample_active_ = false;
  // Init rejects a zero oversample_count, which the analyzer cannot carry in
  // from there.
  // NOLINTBEGIN(clang-analyzer-core.DivideZero)
  PublishSample(now_us,
                static_cast<uint16_t>(voltage_acc_ / cfg_.oversample_count),
                static_cast<uint16_t>(current_acc_ / cfg_.oversample_count));
  // NOLINTEND(clang-analyzer-core.DivideZero)
  return false;
}

void Battery::PublishSample(uint32_t now_us, uint16_t voltage_raw,
                            uint16_t current_raw) {
  const float voltage_adc_mv =
      (static_cast<float>(voltage_raw) * cfg_.adc_reference_mv) /
      Battery::kAdcMaxRaw;

  float measured_voltage_v =
      (((voltage_adc_mv * cfg_.voltage_multiplier_milli) / kMilli) +
       cfg_.voltage_offset_mv) /
      kMilli;
  if (measured_voltage_v < 0.0f) {
    measured_voltage_v = 0.0f;
  }

  // Stays zero with no sense path, which keeps the filter and the integrator
  // below at rest rather than needing a branch of their own.
  float measured_current_a = 0.0f;
  if (cfg_.current_sense) {
    const CurrentSense &sense = *cfg_.current_sense;
    const float current_adc_mv =
        (static_cast<float>(current_raw) * cfg_.adc_reference_mv) /
        Battery::kAdcMaxRaw;

    measured_current_a =
        ((current_adc_mv - static_cast<float>(sense.offset_mv)) *
         sense.scale_ma_per_v) /
        (kMilli * kMilli);

    if (measured_current_a < 0.0f) {
      measured_current_a = 0.0f;
    }
    if (measured_current_a < static_cast<float>(sense.deadband_ma) / kMilli) {
      measured_current_a = 0.0f;
    }
  }

  if (!filter_valid_) {
    filtered_voltage_v_ = measured_voltage_v;
    filtered_current_a_ = measured_current_a;
    filter_valid_ = true;
  } else {
    const float alpha = cfg_.filter_alpha;
    filtered_voltage_v_ += alpha * (measured_voltage_v - filtered_voltage_v_);
    filtered_current_a_ += alpha * (measured_current_a - filtered_current_a_);
  }

  if (last_integrator_us_ != 0u && filtered_current_a_ > 0.0f) {
    const uint32_t dt_us = static_cast<uint32_t>(now_us - last_integrator_us_);
    mah_drawn_ += (filtered_current_a_ * static_cast<float>(dt_us)) /
                  kMicrosecondsPerMahAtOneAmp;
  }
  last_integrator_us_ = now_us;

  blackboard_->UpdateBattery(
      BatteryData{.timestamp_us = last_sample_us_,
                  .voltage = filtered_voltage_v_,
                  .current = Sensed(filtered_current_a_),
                  .mah_drawn = Sensed(mah_drawn_),
                  .percentage = EstimatePercentage(filtered_voltage_v_)});
}

uint8_t Battery::EstimatePercentage(float voltage_v) const {
  const float cell_voltage_mv =
      (voltage_v * kMilli) / static_cast<float>(cfg_.cell_count);
  const float pct = ((cell_voltage_mv - cfg_.cell_empty_mv) * 100.0f) /
                    static_cast<float>(cfg_.cell_full_mv - cfg_.cell_empty_mv);

  return static_cast<uint8_t>(ClampFloat(pct, 0.0f, 100.0f));
}
