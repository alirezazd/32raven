#include "led.hpp"
#include "driver/gpio.h" // IWYU pragma: keep
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h" // IWYU pragma: keep
#include "freertos/task.h"
#include <cstddef> // for size_t

// LEDC configuration
static constexpr ledc_mode_t kLedcMode = LEDC_LOW_SPEED_MODE;
static constexpr ledc_timer_t kLedcTimer = LEDC_TIMER_0;
static constexpr ledc_channel_t kLedcChannel = LEDC_CHANNEL_0;
static constexpr ledc_timer_bit_t kLedcRes = LEDC_TIMER_13_BIT;
static constexpr uint32_t kLedcFreq = 5000;
// 13-bit => max duty is 8191
static constexpr uint32_t kDutyMax = 8191;

// Internal definitions for static steps if needed, but we use dynamic
// generation now.

void LED::Init(const Config &cfg, ErrorHandler error_handler) {
  pin_ = cfg.pin;
  active_low_ = cfg.active_low;

  // Configure LEDC Timer
  ledc_timer_config_t timer_conf = {};
  timer_conf.speed_mode = kLedcMode;
  timer_conf.timer_num = kLedcTimer;
  timer_conf.duty_resolution = kLedcRes;
  timer_conf.freq_hz = kLedcFreq;
  timer_conf.clk_cfg = LEDC_AUTO_CLK;
  if (ledc_timer_config(&timer_conf) != ESP_OK) {
    error_handler("LEDC Timer Init Failed");
    return;
  }

  // Configure LEDC Channel
  ledc_channel_config_t channel_conf = {};
  channel_conf.gpio_num = pin_;
  channel_conf.speed_mode = kLedcMode;
  channel_conf.channel = kLedcChannel;
  channel_conf.intr_type = LEDC_INTR_DISABLE;
  channel_conf.timer_sel = kLedcTimer;
  channel_conf.duty = 0; // Start off
  channel_conf.hpoint = 0;
  if (ledc_channel_config(&channel_conf) != ESP_OK) {
    error_handler("LEDC Channel Init Failed");
    return;
  }

  if (ledc_fade_func_install(0) != ESP_OK) {
    error_handler("LEDC Fade Install Failed");
    return;
  }

  // Start OFF deterministically
  Off();

  xTaskCreate(TaskEntry, "led_task", 2048, this, 1,
              (TaskHandle_t *)&task_handle_);

  initialized_ = true;
}

void LED::SetPattern(const Step *steps, size_t count) {
  current_steps_ = steps;
  current_step_count_ = count;
  if (task_handle_) {
    xTaskNotifyGive((TaskHandle_t)task_handle_);
  }
}

void LED::SetPattern(Pattern p, uint32_t period_ms) {
  if (p == Pattern::kBlink) {
    // 50% duty
    dynamic_steps_[0] = {100, 0, (uint16_t)(period_ms / 2)};
    dynamic_steps_[1] = {0, 0, (uint16_t)(period_ms / 2)};
    SetPattern(dynamic_steps_, 2);
  } else if (p == Pattern::kBreathe) {
    // Fade In/Out
    dynamic_steps_[0] = {100, (uint16_t)(period_ms / 2), 0};
    dynamic_steps_[1] = {0, (uint16_t)(period_ms / 2), 0};
    SetPattern(dynamic_steps_, 2);
  } else if (p == Pattern::kDoubleBlink) {
    // On 10%, Off 10%, On 10%, Off 70%
    uint16_t t_on = period_ms / 10;
    uint16_t t_off_short = period_ms / 10;
    uint16_t t_off_long = period_ms - (2 * t_on) - t_off_short;

    dynamic_steps_[0] = {100, 0, t_on};
    dynamic_steps_[1] = {0, 0, t_off_short};
    dynamic_steps_[2] = {100, 0, t_on};
    dynamic_steps_[3] = {0, 0, t_off_long};
    SetPattern(dynamic_steps_, 4);
  }
}

void LED::On() {
  if (!initialized_)
    return;
  static const Step kOnStep = {100, 0, 1000};
  is_on_ = true;
  SetPattern(&kOnStep, 1);
}

void LED::Off() {
  if (!initialized_)
    return;
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
  size_t step_idx = 0;
  while (true) {
    const Step *steps = current_steps_;
    size_t count = current_step_count_;

    if (!steps || count == 0) {
      ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
      continue;
    }

    const Step &s = steps[step_idx];

    // Calculate target duty
    // duty_percent 0-100
    uint32_t target_duty = (s.duty_percent * kDutyMax) / 100;
    if (active_low_) {
      target_duty = kDutyMax - target_duty;
    }

    if (s.fade_ms > 0) {
      ledc_set_fade_with_time(kLedcMode, kLedcChannel, target_duty, s.fade_ms);
      ledc_fade_start(kLedcMode, kLedcChannel, LEDC_FADE_NO_WAIT);
      // Wait for fade to complete or new pattern
      if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(s.fade_ms)) > 0) {
        ledc_stop(kLedcMode, kLedcChannel, target_duty); // Stop at current
        step_idx = 0;
        continue; // Restart loop with new pattern
      }
    } else {
      ledc_set_duty(kLedcMode, kLedcChannel, target_duty);
      ledc_update_duty(kLedcMode, kLedcChannel);
    }

    if (s.hold_ms > 0) {
      if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(s.hold_ms)) > 0) {
        step_idx = 0;
        continue;
      }
    }

    step_idx = (step_idx + 1) % count;
  }
}
