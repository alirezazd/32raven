#include "led.hpp"
#include "board.h"

void LED::Init(const Config &cfg) {
  if (initialized_) {
    ErrorHandler();
  }
  initialized_ = true;

  port_ = cfg.port;
  pin_ = cfg.pin;
  active_low_ = cfg.active_low;

  // Default: LED OFF
  Set(false);
}

void LED::Set(bool on) {
  // Map logical state to electrical level
  const bool drive_high = active_low_ ? !on : on;

  if (drive_high) {
    port_->BSRR = pin_; // set output high
  } else {
    port_->BSRR = uint32_t(pin_) << 16; // set output low
  }
}

bool LED::IsOn() const {
  // Read output latch, not input
  const bool pin_high = (port_->ODR & pin_) != 0;
  return active_low_ ? !pin_high : pin_high;
}

void LED::Toggle() { Set(!IsOn()); }
