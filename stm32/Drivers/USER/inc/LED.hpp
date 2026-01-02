#ifndef USER_DRIVERS_LED_HPP
#define USER_DRIVERS_LED_HPP

#include "stm32f4xx_hal.h"

class LED {
public:
  static LED &getInstance() {
    static LED instance;
    return instance;
  }

  struct Config {
    GPIO_TypeDef *port;
    uint16_t pin;
  };

  static void on() { getInstance()._on(); }
  static void off() { getInstance()._off(); }
  static void toggle() { getInstance()._toggle(); }

  void _init(const Config &config);

  void _on();
  void _off();
  void _toggle();

private:
  friend class System;
  static void init(const Config &config) { getInstance()._init(config); }

  LED() = default;
  ~LED() = default;

  LED(const LED &) = delete;
  LED &operator=(const LED &) = delete;

  GPIO_TypeDef *port_ = nullptr;
  uint16_t pin_ = 0;
  bool initialized_ = false;
};

#endif // USER_DRIVERS_LED_HPP
