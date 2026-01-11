#include "button.hpp"

void Button::Init(const Config &cfg) {
  if (initialized_) {
    ErrorHandler();
  }
  initialized_ = true;

  port_ = cfg.port;
  pin_ = cfg.pin;
  active_low_ = cfg.active_low;
  debounce_ms_ = cfg.debounce_ms;
  long_press_ms_ = cfg.long_press_ms;

  // Initialize from current pin level
  raw_last_ = ReadRawPressed();
  stable_ = raw_last_;
  pressed_ = stable_;

  raw_last_change_ms_ = 0;
  press_start_ms_ = 0;
  long_fired_ = false;

  ev_press_ = false;
  ev_release_ = false;
  ev_long_ = false;
}

bool Button::ReadRawPressed() const {
  const bool kPinHigh = (port_->IDR & pin_) != 0;
  return active_low_ ? !kPinHigh : kPinHigh;
}

void Button::Poll(uint32_t now_ms) {
  if (!initialized_)
    return;

  const bool kRaw = ReadRawPressed();

  if (kRaw != raw_last_) {
    raw_last_ = kRaw;
    raw_last_change_ms_ = now_ms;
  }

  // Debounce acceptance
  if (kRaw != stable_) {
    if ((now_ms - raw_last_change_ms_) >= debounce_ms_) {
      stable_ = kRaw;

      if (stable_) {
        pressed_ = true;
        press_start_ms_ = now_ms;
        long_fired_ = false;
        ev_press_ = true;
      } else {
        pressed_ = false;
        long_fired_ = false;
        ev_release_ = true;
      }
    }
  }

  // Long press
  if (pressed_ && !long_fired_) {
    if ((now_ms - press_start_ms_) >= long_press_ms_) {
      long_fired_ = true;
      ev_long_ = true;
    }
  }
}

bool Button::ConsumePress() {
  const bool kVal = ev_press_;
  ev_press_ = false;
  return kVal;
}

bool Button::ConsumeRelease() {
  const bool kVal = ev_release_;
  ev_release_ = false;
  return kVal;
}

bool Button::ConsumeLongPress() {
  const bool kVal = ev_long_;
  ev_long_ = false;
  return kVal;
}
