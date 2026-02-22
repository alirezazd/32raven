#pragma once

#include "error_code.hpp"
#include "timebase.hpp"

extern "C" {
#include "driver/gpio.h" // IWYU pragma: keep
}

class Button {
public:
  static Button &GetInstance() {
    static Button instance;
    return instance;
  }

  struct Config {
    gpio_num_t pin;
    bool active_low;
    bool pullup;
    bool pulldown;

    TimeMs debounce_ms;
    TimeMs long_press_ms;
  };

  // Polling API
  void Poll(TimeMs now_ms);

  // Edge-like events (latched until consumed)
  bool ConsumeLongPress();
  bool ConsumePress();
  bool ConsumeRelease();

  bool IsPressed() const { return pressed_; }
  bool IsInitialized() const { return initialized_; }

private:
  friend class System;

  ErrorCode Init(const Config &cfg);

  bool ReadRawPressed() const;

  // config
  gpio_num_t pin_ = GPIO_NUM_NC;
  bool active_low_ = true;
  TimeMs debounce_ms_ = 30;
  TimeMs long_press_ms_ = 500;

  // debouncer
  bool raw_last_ = false;
  bool stable_ = false;
  TimeMs raw_last_change_ms_ = 0;

  // state
  bool pressed_ = false;
  TimeMs press_start_ms_ = 0;
  bool long_fired_ = false;

  // event latches
  bool ev_press_ = false;
  bool ev_release_ = false;
  bool ev_long_ = false;

  bool initialized_ = false;

  Button() = default;
  ~Button() = default;
  Button(const Button &) = delete;
  Button &operator=(const Button &) = delete;
};
