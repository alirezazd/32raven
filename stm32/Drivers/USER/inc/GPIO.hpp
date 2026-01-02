// GPIO.hpp
#pragma once

#include "stm32f4xx_hal.h"

class GPIO {
public:
  // Board-level pin description
  struct PinConfig {
    GPIO_TypeDef *port;
    GPIO_InitTypeDef init;
  };

  // Board GPIO layout
  struct Config {
    PinConfig led;
    PinConfig button;
    PinConfig pb10;
  };

  // One-time init entry point
  // static void init(const Config &config) { instance()._init(config); }

  static GPIO &getInstance() {
    static GPIO instance;
    return instance;
  }

private:
  friend class System;
  static void init(const Config &config) { getInstance()._init(config); }

  GPIO() = default;
  ~GPIO() = default;

  GPIO(const GPIO &) = delete;
  GPIO &operator=(const GPIO &) = delete;

  void _init(const Config &config);
  bool initialized_ = false;
};
