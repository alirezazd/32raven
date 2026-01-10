#ifndef USER_DRIVERS_LED_HPP
#define USER_DRIVERS_LED_HPP

#include "stm32f4xx_hal.h"

class LED {
public:
  struct Config {
    GPIO_TypeDef *port;
    uint16_t pin;
    bool active_low;
  };

  void Init(const Config &config);

  void On();
  void Off();
  void Toggle();

private:
  friend class System;

  static LED &GetInstance() {
    static LED instance;
    return instance;
  }

  LED() = default;
  ~LED() = default;

  LED(const LED &) = delete;
  LED &operator=(const LED &) = delete;

  GPIO_TypeDef *port_ = nullptr;
  uint16_t pin_ = 0;
  bool active_low_ = false;
  bool initialized_ = false;
};

#endif // USER_DRIVERS_LED_HPP
