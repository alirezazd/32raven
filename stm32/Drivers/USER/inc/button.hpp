#pragma once
#include "board.h" // for USER_BTN_GPIO_PORT/Pin and ErrorHandler()
#include "stm32f4xx.h"
#include <cstdint>

class Button {
public:
  struct Config {
    GPIO_TypeDef *port;
    uint16_t pin;           // GPIO_PIN_x bitmask
    bool active_low;        // for your schematic: false
    uint32_t debounce_ms;   // e.g. 50
    uint32_t long_press_ms; // e.g. 500
  };

  static Button &GetInstance() {
    static Button instance;
    return instance;
  }

  void Poll(uint32_t now_ms);

  bool ConsumePress();
  bool ConsumeRelease();
  bool ConsumeLongPress();

  bool IsPressed() const { return pressed_; }

private:
  friend class System;
  void Init(const Config &cfg);

  Button() = default;
  ~Button() = default;
  Button(const Button &) = delete;
  Button &operator=(const Button &) = delete;

  bool ReadRawPressed() const;

  // Config
  GPIO_TypeDef *port_ = nullptr;
  uint16_t pin_ = 0;
  bool active_low_ = false;
  uint32_t debounce_ms_ = 50;
  uint32_t long_press_ms_ = 500;

  // Debounce tracking
  bool raw_last_ = false;
  bool stable_ = false;
  uint32_t raw_last_change_ms_ = 0;

  // State
  bool pressed_ = false;
  uint32_t press_start_ms_ = 0;
  bool long_fired_ = false;

  // Latched events
  bool ev_press_ = false;
  bool ev_release_ = false;
  bool ev_long_ = false;

  bool initialized_ = false;
};
