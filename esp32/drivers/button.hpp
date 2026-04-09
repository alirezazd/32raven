#pragma once

#include "timebase.hpp"

extern "C" {
#include "driver/gpio.h"  // IWYU pragma: keep
}

class Button {
 public:
  static Button &GetInstance() {
    static Button instance;
    return instance;
  }
  struct Config {
    struct Input {
      gpio_num_t pin = GPIO_NUM_NC;
      bool active_low = true;
      bool pullup = true;
      bool pulldown = false;
    };

    struct Timing {
      TimeMs debounce_ms = 30;
      TimeMs long_press_ms = 500;
      TimeMs long_long_press_ms = 1500;
    };

    Input input{};
    Timing timing{};
  };
  // Polling API
  void Poll();
  // Semantic events (latched until consumed)
  bool ConsumePress();
  bool ConsumeLongPress();
  bool ConsumeLongLongPress();
  void FlushEvents();

 private:
  enum class Phase : uint8_t {
    kReleased,
    kDebouncingPress,
    kPressed,
    kDebouncingRelease,
  };

  friend class System;
  void Init(const Config &cfg);
  bool ReadRawPressed() const;
  void EnterPressed(TimeMs now_ms);
  void EnterReleased(TimeMs now_ms);
  void UpdateHoldEvents(TimeMs now_ms);
  // config
  gpio_num_t pin_ = GPIO_NUM_NC;
  bool active_low_ = true;
  TimeMs debounce_ms_ = 30;
  TimeMs long_press_ms_ = 500;
  TimeMs long_long_press_ms_ = 1500;
  Phase phase_ = Phase::kReleased;
  bool raw_pressed_ = false;
  TimeMs debounce_started_ms_ = 0;
  TimeMs press_started_ms_ = 0;
  bool long_fired_ = false;
  bool long_long_fired_ = false;
  // event latches
  bool ev_press_ = false;
  bool ev_long_ = false;
  bool ev_long_long_ = false;
  Button() = default;
  ~Button() = default;
  Button(const Button &) = delete;
  Button &operator=(const Button &) = delete;
};
