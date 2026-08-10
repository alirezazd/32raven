// SPDX-License-Identifier: GPL-3.0-only
// Copyright (C) 2026 Alireza Azadi

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
    };

    Input input{};
    Timing timing{};
  };
  // Polling API
  void Poll();
  // Semantic events (latched until consumed)
  [[nodiscard]] bool ConsumePress();
  [[nodiscard]] bool ConsumeLongPress();
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
  Config cfg_{};
  Phase phase_ = Phase::kReleased;
  bool raw_pressed_ = false;
  TimeMs debounce_started_ms_ = 0;
  TimeMs press_started_ms_ = 0;
  bool long_fired_ = false;
  // event latches
  bool ev_press_ = false;
  bool ev_long_ = false;
  Button() = default;
  ~Button() = default;
  Button(const Button &) = delete;
  Button &operator=(const Button &) = delete;
};
