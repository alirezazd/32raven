#include "battery.hpp"

#include "board.hpp"
#include "error_code.hpp"
#include "panic.hpp"
#include "stm32_limits.hpp"
#include "stm32f4xx.h"

namespace {

constexpr float kMilli = 1000.0f;
constexpr float kMicrosecondsPerMahAtOneAmp = 3600000.0f;
constexpr uint32_t kAdcSampleTime480Cycles = 7u;
constexpr uint8_t kEscVbaAdcChannel = 10u;
constexpr uint8_t kEscCurAdcChannel = 11u;
constexpr uint32_t kGpioModeMask = 0x3u;
constexpr uint32_t kGpioAnalogMode = 0x3u;

void ValidateAdcChannel(uint8_t channel) {
  if (channel <= 15u) {
    return;
  }
  Panic(ErrorCode::Stm32::kAdcInitFailed);
}

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

uint8_t PinIndex(uint16_t pin) {
  if (pin == 0u || (pin & (pin - 1u)) != 0u) {
    Panic(ErrorCode::Stm32::kGpioConfigFailed);
  }

  for (uint8_t index = 0; index < 16u; ++index) {
    if ((pin & (1u << index)) != 0u) {
      return index;
    }
  }

  Panic(ErrorCode::Stm32::kGpioConfigFailed);
  return 0u;
}

void ConfigureAnalogPin(GPIO_TypeDef *port, uint16_t pin) {
  const uint32_t shift = static_cast<uint32_t>(PinIndex(pin)) * 2u;
  port->MODER =
      (port->MODER & ~(kGpioModeMask << shift)) | (kGpioAnalogMode << shift);
  port->PUPDR &= ~(kGpioModeMask << shift);
}

void EnableBatteryPeripheralClocks() {
  RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
  (void)RCC->AHB1ENR;

  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
  (void)RCC->APB2ENR;
}

bool WaitForEoc(uint16_t timeout_us) {
  const uint32_t start_us = TIM2->CNT;
  while ((ADC1->SR & ADC_SR_EOC) == 0u) {
    if (static_cast<uint32_t>(TIM2->CNT - start_us) >= timeout_us) {
      return false;
    }
  }
  return true;
}

float ClampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

}  // namespace

void Battery::Init(const Config &cfg) {
  if (initialized_) {
    Panic(ErrorCode::Stm32::kAdcInitFailed);
  }

  if (cfg.sample_period_us == 0u || cfg.adc_reference_mv == 0u ||
      cfg.oversample_count == 0u || cfg.filter_alpha_permille == 0u ||
      cfg.filter_alpha_permille > 1000u || cfg.adc_timeout_us == 0u ||
      cfg.voltage_multiplier_milli == 0u || cfg.current_scale_ma_per_v == 0u ||
      cfg.cell_count == 0u || cfg.cell_empty_mv >= cfg.cell_full_mv ||
      cfg.voltage_adc_channel == cfg.current_adc_channel) {
    Panic(ErrorCode::Stm32::kAdcInitFailed);
  }

  cfg_ = cfg;
  ValidateAdcChannel(cfg_.voltage_adc_channel);
  ValidateAdcChannel(cfg_.current_adc_channel);
  if (cfg_.voltage_adc_channel != kEscVbaAdcChannel ||
      cfg_.current_adc_channel != kEscCurAdcChannel) {
    Panic(ErrorCode::Stm32::kAdcInitFailed);
  }

  mah_drawn_ = static_cast<float>(cfg_.initial_mah_drawn);
  data_ = {};
  data_.mah_drawn = mah_drawn_;

  InitAdc();
  initialized_ = true;
}

void Battery::Poll(uint32_t now_us) {
  if (!initialized_) {
    Panic(ErrorCode::Stm32::kAdcInitFailed);
  }

  if (last_sample_us_ != 0u &&
      static_cast<uint32_t>(now_us - last_sample_us_) < cfg_.sample_period_us) {
    return;
  }

  last_sample_us_ = now_us;
  const AdcPair sample = ReadAdcPair();
  if (!sample.valid) {
    adc_error_count_++;
    return;
  }

  PublishSample(now_us, sample.voltage_raw, sample.current_raw);
}

void Battery::InitAdc() {
  if (board::kEscVba.port != GPIOC || board::kEscCur.port != GPIOC) {
    Panic(ErrorCode::Stm32::kGpioConfigFailed);
  }

  EnableBatteryPeripheralClocks();
  ConfigureAnalogPin(board::kEscVba.port, board::kEscVba.pin);
  ConfigureAnalogPin(board::kEscCur.port, board::kEscCur.pin);

  ADC->CCR &= ~ADC_CCR_ADCPRE;
  ADC->CCR |= ADC_CCR_ADCPRE_0;  // PCLK2 / 4 = 21 MHz on this clock tree.

  ADC1->CR1 = ADC_CR1_SCAN;
  ADC1->CR2 = ADC_CR2_EOCS;
  ADC1->SQR1 = ADC_SQR1_L_0;  // 2 regular conversions.
  ADC1->SQR2 = 0;
  ADC1->SQR3 = static_cast<uint32_t>(cfg_.voltage_adc_channel) |
               (static_cast<uint32_t>(cfg_.current_adc_channel) << 5u);

  SetAdcSampleTime(cfg_.voltage_adc_channel, kAdcSampleTime480Cycles);
  SetAdcSampleTime(cfg_.current_adc_channel, kAdcSampleTime480Cycles);

  ADC1->SR = 0;
  ADC1->CR2 |= ADC_CR2_ADON;
}

Battery::AdcPair Battery::ReadAdcPair() {
  uint32_t voltage_acc = 0;
  uint32_t current_acc = 0;

  for (uint8_t i = 0; i < cfg_.oversample_count; ++i) {
    ADC1->SR = 0;
    ADC1->CR2 |= ADC_CR2_SWSTART;

    if (!WaitForEoc(cfg_.adc_timeout_us)) {
      return AdcPair{0, 0, false};
    }
    voltage_acc += ADC1->DR & ADC_DR_DATA;

    if (!WaitForEoc(cfg_.adc_timeout_us)) {
      return AdcPair{0, 0, false};
    }
    current_acc += ADC1->DR & ADC_DR_DATA;
  }

  return AdcPair{
      static_cast<uint16_t>(voltage_acc / cfg_.oversample_count),
      static_cast<uint16_t>(current_acc / cfg_.oversample_count),
      true,
  };
}

void Battery::PublishSample(uint32_t now_us, uint16_t voltage_raw,
                            uint16_t current_raw) {
  const float voltage_adc_mv =
      (static_cast<float>(voltage_raw) * cfg_.adc_reference_mv) /
      stm32_limits::kBatteryAdcMaxRaw;
  const float current_adc_mv =
      (static_cast<float>(current_raw) * cfg_.adc_reference_mv) /
      stm32_limits::kBatteryAdcMaxRaw;

  float measured_voltage_v =
      ((voltage_adc_mv * cfg_.voltage_multiplier_milli) / kMilli +
       cfg_.voltage_offset_mv) /
      kMilli;
  if (measured_voltage_v < 0.0f) {
    measured_voltage_v = 0.0f;
  }

  float measured_current_a =
      ((current_adc_mv - static_cast<float>(cfg_.current_offset_mv)) *
       cfg_.current_scale_ma_per_v) /
      (kMilli * kMilli);

  if (measured_current_a < 0.0f) {
    measured_current_a = 0.0f;
  }
  if (measured_current_a <
      static_cast<float>(cfg_.current_deadband_ma) / kMilli) {
    measured_current_a = 0.0f;
  }

  if (!filter_valid_) {
    filtered_voltage_v_ = measured_voltage_v;
    filtered_current_a_ = measured_current_a;
    filter_valid_ = true;
  } else {
    const float alpha = static_cast<float>(cfg_.filter_alpha_permille) / kMilli;
    filtered_voltage_v_ += alpha * (measured_voltage_v - filtered_voltage_v_);
    filtered_current_a_ += alpha * (measured_current_a - filtered_current_a_);
  }

  if (last_integrator_us_ != 0u && filtered_current_a_ > 0.0f) {
    const uint32_t dt_us = static_cast<uint32_t>(now_us - last_integrator_us_);
    mah_drawn_ += (filtered_current_a_ * static_cast<float>(dt_us)) /
                  kMicrosecondsPerMahAtOneAmp;
  }
  last_integrator_us_ = now_us;

  data_.voltage = filtered_voltage_v_;
  data_.current = filtered_current_a_;
  data_.mah_drawn = mah_drawn_;
  data_.percentage = EstimatePercentage(filtered_voltage_v_);
}

uint8_t Battery::EstimatePercentage(float voltage_v) const {
  const float cell_voltage_mv =
      (voltage_v * kMilli) / static_cast<float>(cfg_.cell_count);
  const float pct = ((cell_voltage_mv - cfg_.cell_empty_mv) * 100.0f) /
                    static_cast<float>(cfg_.cell_full_mv - cfg_.cell_empty_mv);

  return static_cast<uint8_t>(ClampFloat(pct, 0.0f, 100.0f));
}
