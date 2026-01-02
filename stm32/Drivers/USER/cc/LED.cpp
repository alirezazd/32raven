#include "LED.hpp"

#include "board.h"

void LED::_init(const Config &config) {
  if (initialized_) {
    Error_Handler();
  }
  initialized_ = true;

  port_ = config.port;
  pin_ = config.pin;

  // Default state is OFF, GPIO already configured
  HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_RESET);
}

void LED::_on() { HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_SET); }

void LED::_off() { HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_RESET); }

void LED::_toggle() { HAL_GPIO_TogglePin(port_, pin_); }
