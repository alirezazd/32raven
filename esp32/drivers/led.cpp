#include "led.hpp"
#include "driver/gpio.h"

void LED::Init(const Config &cfg) {
  pin_ = cfg.pin;
  active_low_ = cfg.active_low;

  gpio_config_t io{};
  io.pin_bit_mask = (1ULL << pin_);
  io.mode = GPIO_MODE_OUTPUT;
  io.pull_up_en = GPIO_PULLUP_DISABLE;
  io.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io.intr_type = GPIO_INTR_DISABLE;

  gpio_config(&io);

  // Start OFF deterministically
  is_on_ = false;
  const int kLevel = active_low_ ? 1 : 0;
  gpio_set_level(pin_, kLevel);

  initialized_ = true;
}

void LED::On() {
  if (!initialized_)
    return;

  is_on_ = true;
  const int kLevel = active_low_ ? 0 : 1;
  gpio_set_level(pin_, kLevel);
}

void LED::Off() {
  if (!initialized_)
    return;

  is_on_ = false;
  const int kLevel = active_low_ ? 1 : 0;
  gpio_set_level(pin_, kLevel);
}

void LED::Toggle() {
  if (!initialized_)
    return;

  is_on_ = !is_on_;
  const int kLevel = is_on_ ? (active_low_ ? 0 : 1) : (active_low_ ? 1 : 0);
  gpio_set_level(pin_, kLevel);
}
