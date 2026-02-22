#include "led.hpp"
#include "board.h"
#include "panic.hpp"

void LED::Init(GPIO &gpio, const Config &cfg) {
  if (initialized_) {
    Panic(ErrorCode::kLedReinit);
  }
  initialized_ = true;
  gpio_ = &gpio;
  port_ = cfg.pin.port;
  pin_ = cfg.pin.number;
  active_low_ = cfg.active_low;
  Set(false);
}

void LED::Set(bool on) { gpio_->WritePin(port_, pin_, active_low_ ? !on : on); }

bool LED::IsOn() const { return gpio_->ReadPin(port_, pin_) == !active_low_; }

void LED::Toggle() { Set(!IsOn()); }
