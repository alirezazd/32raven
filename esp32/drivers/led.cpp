// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#include "led.hpp"

#include <cstddef>  // for size_t

#include "error_code.hpp"
#include "panic.hpp"

extern "C" {
#include "driver/gpio.h"  // IWYU pragma: keep
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/task.h"
}

void LED::Init(const Config &cfg) {
  static constexpr uint32_t kTaskStackBytes = 2048;
  static StaticTask_t task_buffer;
  static StackType_t task_stack[kTaskStackBytes];
  pin_ = cfg.pin;
  active_low_ = cfg.active_low;
  ledc_ = cfg.ledc;
  duty_max_ =
      (UINT32_C(1) << static_cast<uint32_t>(ledc_.duty_resolution)) - UINT32_C(1);

  // Configure LEDC Timer
  ledc_timer_config_t timer_conf = {};
  timer_conf.speed_mode = ledc_.speed_mode;
  timer_conf.timer_num = ledc_.timer;
  timer_conf.duty_resolution = ledc_.duty_resolution;
  timer_conf.freq_hz = ledc_.freq_hz;
  timer_conf.clk_cfg = LEDC_AUTO_CLK;
  if (ledc_timer_config(&timer_conf) != ESP_OK) {
    Panic(ErrorCode::Esp32::kLedTimerInitFailed);
  }

  // Configure LEDC Channel
  ledc_channel_config_t channel_conf = {};
  channel_conf.gpio_num = pin_;
  channel_conf.speed_mode = ledc_.speed_mode;
  channel_conf.channel = ledc_.channel;
  channel_conf.intr_type = LEDC_INTR_DISABLE;
  channel_conf.timer_sel = ledc_.timer;
  channel_conf.duty = 0;  // Start off
  channel_conf.hpoint = 0;
  if (ledc_channel_config(&channel_conf) != ESP_OK) {
    Panic(ErrorCode::Esp32::kLedChannelInitFailed);
  }

  if (ledc_fade_func_install(0) != ESP_OK) {
    Panic(ErrorCode::Esp32::kLedFadeInstallFailed);
  }

  // Start OFF deterministically
  Off();

  task_handle_ = xTaskCreateStatic(TaskEntry, "led_task", kTaskStackBytes, this,
                                   1, task_stack, &task_buffer);
  if (task_handle_ == nullptr) {
    Panic(ErrorCode::Esp32::kLedTaskCreateFailed);
  }
}

void LED::SetPattern(const Step *steps, size_t count,
                     std::optional<int> repeat_count) {
  if (repeat_count.has_value() && *repeat_count <= 0) {
    Off();
    return;
  }

  current_steps_ = steps;
  current_step_count_ = count;
  repeat_count_ = repeat_count;
  if (task_handle_) {
    xTaskNotifyGive((TaskHandle_t)task_handle_);
  }
}

void LED::SetPattern(Pattern p, uint32_t period_ms,
                     std::optional<int> repeat_count) {
  switch (p) {
    case Pattern::kBlink:
      // 50% duty
      dynamic_steps_[0] = {100, 0, static_cast<uint16_t>(period_ms / 2)};
      dynamic_steps_[1] = {0, 0, static_cast<uint16_t>(period_ms / 2)};
      SetPattern(dynamic_steps_, 2, repeat_count);
      break;

    case Pattern::kBreathe:
      // Fade In/Out
      dynamic_steps_[0] = {100, static_cast<uint16_t>(period_ms / 2), 0};
      dynamic_steps_[1] = {0, static_cast<uint16_t>(period_ms / 2), 0};
      SetPattern(dynamic_steps_, 2, repeat_count);
      break;

    case Pattern::kDoubleBlink: {
      // On 10%, Off 10%, On 10%, Off 70%
      const uint16_t t_on = static_cast<uint16_t>(period_ms / 10);
      const uint16_t t_off_short = static_cast<uint16_t>(period_ms / 10);
      const uint16_t t_off_long =
          static_cast<uint16_t>(period_ms - (2 * t_on) - t_off_short);

      dynamic_steps_[0] = {100, 0, t_on};
      dynamic_steps_[1] = {0, 0, t_off_short};
      dynamic_steps_[2] = {100, 0, t_on};
      dynamic_steps_[3] = {0, 0, t_off_long};
      SetPattern(dynamic_steps_, 4, repeat_count);
      break;
    }
  }
}

void LED::On() {
  static const Step kOnStep = {100, 0, 1000};
  is_on_ = true;
  SetPattern(&kOnStep, 1);
}

void LED::Off() {
  static const Step kOffStep = {0, 0, 1000};
  is_on_ = false;
  SetPattern(&kOffStep, 1);
}

void LED::Toggle() {
  if (is_on_)
    Off();
  else
    On();
}

void LED::TaskEntry(void *param) { static_cast<LED *>(param)->Task(); }

void LED::Task() {
  static const Step kOffStep = {0, 0, 1000};

  size_t step_idx = 0;
  const Step *steps = current_steps_;
  size_t count = current_step_count_;
  std::optional<int> repeat_count = repeat_count_;

  while (true) {
    const auto load_pattern = [&]() {
      steps = current_steps_;
      count = current_step_count_;
      repeat_count = repeat_count_;
      step_idx = 0;
    };

    if (ulTaskNotifyTake(pdTRUE, 0) > 0) {
      load_pattern();
    }

    if (!steps || count == 0) {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      load_pattern();
      continue;
    }

    const Step &s = steps[step_idx];

    // Calculate target duty
    // duty_percent 0-100
    uint32_t target_duty = (s.duty_percent * duty_max_) / 100;
    if (active_low_) {
      target_duty = duty_max_ - target_duty;
    }

    if (s.fade_ms > 0) {
      ledc_set_fade_with_time(ledc_.speed_mode, ledc_.channel, target_duty, s.fade_ms);
      ledc_fade_start(ledc_.speed_mode, ledc_.channel, LEDC_FADE_NO_WAIT);
      // Wait for fade to complete or new pattern
      if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(s.fade_ms)) > 0) {
        ledc_stop(ledc_.speed_mode, ledc_.channel, target_duty);  // Stop at current
        load_pattern();
        continue;
      }
    } else {
      ledc_set_duty(ledc_.speed_mode, ledc_.channel, target_duty);
      ledc_update_duty(ledc_.speed_mode, ledc_.channel);
    }

    if (s.hold_ms > 0) {
      if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(s.hold_ms)) > 0) {
        load_pattern();
        continue;
      }
    }

    step_idx = (step_idx + 1) % count;
    if (step_idx == 0 && repeat_count.has_value()) {
      --(*repeat_count);
      if (*repeat_count <= 0) {
        is_on_ = false;
        current_steps_ = &kOffStep;
        current_step_count_ = 1;
        repeat_count_ = std::nullopt;
        load_pattern();
      }
    }
  }
}
