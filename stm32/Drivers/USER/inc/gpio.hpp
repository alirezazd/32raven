#pragma once
#include "panic.hpp"
#include "stm32f4xx_hal.h"
#include <array>
#include <cstddef>

class GPIO {
public:
  struct PinConfig {
    GPIO_TypeDef *port;
    GPIO_InitTypeDef init;
    bool active_low;
  };

  void WritePin(GPIO_TypeDef *port, uint16_t pin, bool state);
  bool ReadPin(GPIO_TypeDef *port, uint16_t pin);

private:
  friend class System;
  template <size_t N> void Init(const std::array<PinConfig, N> &pins) {
    Init(pins.data(), N);
  }

  void Init(const PinConfig *pins, size_t count);

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
