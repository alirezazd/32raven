#pragma once
#include "gpio.hpp"
#include "stm32f4xx.h"
#include <cstdint>

class LED {
public:
  struct Config {
    struct Pin {
      GPIO_TypeDef *port;
      uint16_t number; // GPIO_PIN_x bitmask
    } pin;
    bool active_low;
  };

  void Set(bool on);
  void Toggle();
  bool IsOn() const;

private:
  friend class System;
  void Init(GPIO &gpio, const Config &cfg);

  static LED &GetInstance() {
    static LED instance;
    return instance;
  }

  LED() = default;
  ~LED() = default;

  LED(const LED &) = delete;
  LED &operator=(const LED &) = delete;

  GPIO *gpio_ = nullptr;
  GPIO_TypeDef *port_ = nullptr;
  uint16_t pin_ = 0;
  bool active_low_ = false;
  bool initialized_ = false;
};
