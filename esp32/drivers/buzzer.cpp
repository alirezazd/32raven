// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "buzzer.hpp"

#include <algorithm>
#include <cmath>

#include "error_code.hpp"
#include "panic.hpp"

extern "C" {
#include "driver/ledc.h"
#include "esp_log.h"
#include "soc/soc.h"       // IWYU pragma: keep
#include "soc/soc_caps.h"  // IWYU pragma: keep
}

namespace {
constexpr const char *kTag = "buzzer";
}  // namespace

Buzzer &Buzzer::GetInstance() {
  static Buzzer instance;
  return instance;
}

uint32_t Buzzer::NoteHz(uint32_t freq_hz) const {
  return std::min(freq_hz, cfg_.ledc.max_note_hz);
}

uint32_t Buzzer::ComputeDutyTicks(float duty_cycle) const {
  duty_cycle = std::clamp(duty_cycle, 0.0f, 1.0f);

  const uint32_t duty_max =
      (UINT32_C(1) << static_cast<uint32_t>(cfg_.ledc.duty_resolution)) -
      UINT32_C(1);
  const float ticks_f = duty_cycle * static_cast<float>(duty_max);
  return static_cast<uint32_t>(std::lround(ticks_f));
}

void Buzzer::Init(const Config &cfg) {
  if (cfg.output.pin == GPIO_NUM_NC) {
    Panic(ErrorCode::Esp32::kBuzzerInvalidConfig);
  }

  cfg_ = cfg;
  // ledc_timer_config() will not configure a timer without a frequency, and
  // the timer has to exist before the channel does. The highest note is the
  // one frequency the build has already proven this resolution can carry, and
  // it is never heard: the channel comes up at zero duty and every note sets
  // its own.
  freq_hz_ = cfg_.ledc.max_note_hz;

  ledc_timer_config_t timer_cfg = {};
  timer_cfg.speed_mode = cfg_.ledc.speed_mode;
  timer_cfg.duty_resolution = cfg_.ledc.duty_resolution;
  timer_cfg.timer_num = cfg_.ledc.timer;
  timer_cfg.freq_hz = freq_hz_;
  timer_cfg.clk_cfg = LEDC_AUTO_CLK;
  timer_cfg.deconfigure = false;

  if (ledc_timer_config(&timer_cfg) != ESP_OK) {
    Panic(ErrorCode::Esp32::kBuzzerTimerInitFailed);
  }

  ledc_channel_config_t channel_cfg = {};
  channel_cfg.gpio_num = cfg.output.pin;
  channel_cfg.speed_mode = cfg_.ledc.speed_mode;
  channel_cfg.channel = cfg_.ledc.channel;
  channel_cfg.intr_type = LEDC_INTR_DISABLE;
  channel_cfg.timer_sel = cfg_.ledc.timer;
  channel_cfg.duty = 0;
  channel_cfg.hpoint = 0;
  channel_cfg.sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD;
  channel_cfg.flags.output_invert =
      static_cast<unsigned int>(cfg.output.active_low ? 1U : 0U);

  if (ledc_channel_config(&channel_cfg) != ESP_OK) {
    Panic(ErrorCode::Esp32::kBuzzerChannelInitFailed);
  }
  running_ = false;

  ESP_LOGI(kTag, "init pin=%d active_low=%d freq=%lu duty_ticks=%lu",
           static_cast<int>(cfg.output.pin),
           static_cast<int>(cfg.output.active_low),
           static_cast<unsigned long>(freq_hz_),
           static_cast<unsigned long>(duty_ticks_));
}

void Buzzer::Start(uint32_t freq_hz) {
  if (freq_hz == 0) {
    Panic(ErrorCode::Esp32::kBuzzerInvalidArg);
  }
  freq_hz = NoteHz(freq_hz);
  if (ledc_set_freq(cfg_.ledc.speed_mode, cfg_.ledc.timer, freq_hz) != ESP_OK) {
    Panic(ErrorCode::Esp32::kBuzzerSetFreqFailed);
  }
  freq_hz_ = freq_hz;
  if (ledc_set_duty_and_update(cfg_.ledc.speed_mode, cfg_.ledc.channel, duty_ticks_, 0) !=
      ESP_OK) {
    Panic(ErrorCode::Esp32::kBuzzerSetDutyFailed);
  }
  running_ = true;
}

void Buzzer::SetFrequency(uint32_t freq_hz) {
  if (freq_hz == 0) {
    Panic(ErrorCode::Esp32::kBuzzerInvalidArg);
  }
  freq_hz = NoteHz(freq_hz);
  if (ledc_set_freq(cfg_.ledc.speed_mode, cfg_.ledc.timer, freq_hz) != ESP_OK) {
    Panic(ErrorCode::Esp32::kBuzzerSetFreqFailed);
  }
  freq_hz_ = freq_hz;
}

void Buzzer::SetDutyCycle(float duty_cycle) {
  duty_ticks_ = ComputeDutyTicks(duty_cycle);
  if (!running_) {
    return;
  }
  if (ledc_set_duty_and_update(cfg_.ledc.speed_mode, cfg_.ledc.channel, duty_ticks_, 0) !=
      ESP_OK) {
    Panic(ErrorCode::Esp32::kBuzzerSetDutyFailed);
  }
}

void Buzzer::Stop() {
  if (ledc_set_duty_and_update(cfg_.ledc.speed_mode, cfg_.ledc.channel, 0, 0) != ESP_OK) {
    Panic(ErrorCode::Esp32::kBuzzerSetDutyFailed);
  }
  running_ = false;
}
