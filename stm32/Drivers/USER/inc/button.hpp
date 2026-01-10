#pragma once

#include "board.h"
#include <cstdint>

class Button {
public:
  static Button &GetInstance() {
    static Button instance;
    return instance;
  }

  struct Config {
    uint16_t pin;
    bool active_low;
    bool pullup;
    bool pulldown;
    uint32_t debounce_ms;
    uint32_t long_press_ms;
  };

  void Init(const Config &config);
  void Poll(uint32_t now_ms);

  // Event consumption
  bool ConsumePress();
  bool ConsumeLongPress();
  bool ConsumeRelease();

  // State access
  bool IsPressed() const { return pressed_; }
  bool IsInitialized() const { return initialized_; }

private:
  Button() = default;
  ~Button() = default;
  Button(const Button &) = delete;
  Button &operator=(const Button &) = delete;

  bool ReadRawPressed() const;

  // Configuration
  uint16_t pin_ = 0;
  bool active_low_ = true;
  uint32_t debounce_ms_ = 50;
  uint32_t long_press_ms_ = 500;

  // Debouncer state
  bool raw_last_ = false;
  bool stable_ = false;
  uint32_t raw_last_change_ms_ = 0;

  // Button state
  bool pressed_ = false;
  uint32_t press_start_ms_ = 0;
  bool long_fired_ = false;
  bool initialized_ = false;

  // Event latches
  bool ev_press_ = false;
  bool ev_release_ = false;
  bool ev_long_ = false;
};
