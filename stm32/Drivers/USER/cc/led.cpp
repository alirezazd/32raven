#include "led.hpp"

#include "board.h"

void LED::Init(const Config &config) {
  if (initialized_) {
    ErrorHandler();
  }
  initialized_ = true;

  port_ = config.port;
  pin_ = config.pin;
  active_low_ = config.active_low;

  // Default state is OFF, GPIO already configured.
  // We use HAL_GPIO_WritePin to ensure state.
  // Assuming Active Low: OFF = High (SET), ON = Low (RESET)
  // Assuming Active High: OFF = Low (RESET), ON = High (SET)

  // The user requested:
  // HAL_GPIO_WritePin(USER_LED_GPIO_Port, USER_LED_Pin, GPIO_PIN_RESET);
  // This sets Pin 1 Low. If Active Low, this turns it ON.
  // If Active High, this turns it OFF.
  // Since we want to start OFF, and usually LEDs are Active High on these
  // boards unless specified, but we are adding active_low support. If
  // active_low is true, we should start HIGH (OFF). If active_low is false, we
  // should start LOW (OFF).

  HAL_GPIO_WritePin(port_, pin_, active_low_ ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void LED::On() {
  HAL_GPIO_WritePin(port_, pin_, active_low_ ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

void LED::Off() {
  HAL_GPIO_WritePin(port_, pin_, active_low_ ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void LED::Toggle() { HAL_GPIO_TogglePin(port_, pin_); }
