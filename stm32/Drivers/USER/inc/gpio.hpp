#pragma once
#include "stm32f4xx_hal.h"
#include <cstddef>

class GPIO {
public:
  struct PinConfig {
    GPIO_TypeDef *port;
    GPIO_InitTypeDef init;
  };

  struct Config {
    const PinConfig *pins;
    size_t pin_count;
  };

private:
  friend class System;
  void Init(const Config &cfg);

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
