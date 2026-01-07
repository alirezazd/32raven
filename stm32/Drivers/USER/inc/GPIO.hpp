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
  void Init(const Config &config);

private:
  friend class System;

  static GPIO &GetInstance() {
    static GPIO instance;
    return instance;
  }

  GPIO() = default;
  ~GPIO() = default;

  GPIO(const GPIO &) = delete;
  GPIO &operator=(const GPIO &) = delete;

  bool initialized_ = false;
};
