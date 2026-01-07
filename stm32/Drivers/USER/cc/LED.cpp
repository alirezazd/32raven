#include "LED.hpp"

#include "board.h"

void LED::Init(const Config &config) {
  if (initialized_) {
    ErrorHandler();
  }
  initialized_ = true;

  port_ = config.port;
  pin_ = config.pin;

  // Default state is OFF, GPIO already configured
  HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_RESET);
}

void LED::On() { HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_SET); }

void LED::Off() { HAL_GPIO_WritePin(port_, pin_, GPIO_PIN_RESET); }

void LED::Toggle() { HAL_GPIO_TogglePin(port_, pin_); }
