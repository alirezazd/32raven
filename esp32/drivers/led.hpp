#pragma once

extern "C" {
#include "hal/gpio_types.h" // IWYU pragma: keep
}

class LED {
public:
  static LED &GetInstance() {
    static LED instance;
    return instance;
  }

  struct Config {
    gpio_num_t pin = GPIO_NUM_8;
    bool active_low = true;
  };

  void Init(const Config &cfg);

  void On();
  void Off();
  void Toggle();

  bool IsInitialized() const { return initialized_; }

private:
  friend class System;

  LED() = default;
  ~LED() = default;
  LED(const LED &) = delete;
  LED &operator=(const LED &) = delete;

  gpio_num_t pin_ = GPIO_NUM_NC;
  bool active_low_ = false;
  bool is_on_ = false;
  bool initialized_ = false;
};
