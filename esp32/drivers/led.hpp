// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>

extern "C" {
#include "hal/gpio_types.h"  // IWYU pragma: keep
#include "hal/ledc_types.h"
}

class LED {
 public:
  static LED &GetInstance() {
    static LED instance;
    return instance;
  }

  // Which LEDC timer and channel this driver claims. The generator checks the
  // claims against each other, so nothing here has to.
  struct Ledc {
    ledc_mode_t speed_mode;
    ledc_timer_t timer;
    ledc_channel_t channel;
    ledc_timer_bit_t duty_resolution;
    uint32_t freq_hz;
  };

  struct Config {
    gpio_num_t pin = GPIO_NUM_8;
    bool active_low = true;
    Ledc ledc{};
  };

  enum class Pattern : uint8_t {
    kBlink,
    kBreathe,
    kDoubleBlink,
  };

  struct Step {
    uint8_t duty_percent;
    uint16_t fade_ms;
    uint16_t hold_ms;
  };

  // Set a specific pattern type with a period
  void SetPattern(Pattern p, uint32_t period_ms,
                  std::optional<int> repeat_count = std::nullopt);

  // Set a custom sequence of steps
  void SetPattern(const Step *steps, size_t count,
                  std::optional<int> repeat_count = std::nullopt);
  void On();
  void Off();
  void Toggle();

 private:
  friend class System;

  void Init(const Config &cfg);

  LED() = default;
  ~LED() = default;
  LED(const LED &) = delete;
  LED &operator=(const LED &) = delete;

  static void TaskEntry(void *param);
  void Task();

  // Installs a sequence and wakes the task. SetPattern and Off both end here
  // rather than one calling the other: esp32_stack_check scores a back edge as
  // zero, so a cycle between them would shrink the LED task's measured depth
  // instead of failing it.
  void Apply(const Step *steps, size_t count, std::optional<int> repeat_count);

  gpio_num_t pin_ = GPIO_NUM_NC;
  bool active_low_ = false;
  Ledc ledc_{};
  uint32_t duty_max_ = 0;
  bool is_on_ = false;

  void *task_handle_ = nullptr;  // TaskHandle_t

  const Step *current_steps_ = nullptr;
  size_t current_step_count_ = 0;
  std::optional<int> repeat_count_ = std::nullopt;

  // Buffer for dynamic patterns (max 4 steps for double blink)
  Step dynamic_steps_[4];
};
