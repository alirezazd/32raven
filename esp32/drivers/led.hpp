#pragma once
#include <cstddef>

#include "error_code.hpp"

extern "C" {
#include "hal/gpio_types.h" // IWYU pragma: keep
}

class LED {
public:
  static LED &GetInstance() {
    static LED instance;
    return instance;
  }

  struct Config {
    gpio_num_t pin = GPIO_NUM_8;
    bool active_low = true;
  };

  enum class Pattern {
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
  void SetPattern(Pattern p, uint32_t period_ms);

  // Set a custom sequence of steps
  void SetPattern(const Step *steps, size_t count);

  void On();
  void Off();
  void Toggle();

  bool IsInitialized() const { return initialized_; }

private:
  friend class System;

  ErrorCode Init(const Config &cfg);

  LED() = default;
  ~LED() = default;
  LED(const LED &) = delete;
  LED &operator=(const LED &) = delete;

  static void TaskEntry(void *param);
  void Task();

  gpio_num_t pin_ = GPIO_NUM_NC;
  bool active_low_ = false;
  bool is_on_ = false;
  bool initialized_ = false;

  void *task_handle_ = nullptr; // TaskHandle_t

  const Step *current_steps_ = nullptr;
  size_t current_step_count_ = 0;

  // Buffer for dynamic patterns (max 4 steps for double blink)
  Step dynamic_steps_[4];
};
