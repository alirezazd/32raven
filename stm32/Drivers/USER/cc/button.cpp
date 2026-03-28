#include "button.hpp"
#include "gpio.hpp"

void Button::Init(GPIO &gpio, const Config &cfg) {
  if (initialized_) {
    Panic(ErrorCode::kButtonReinit);
  }
  initialized_ = true;
  gpio_ = &gpio;

  port_ = cfg.pin.port;
  pin_ = cfg.pin.number;
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
  const bool pin_high = gpio_->ReadPin(port_, pin_);
  return active_low_ ? !pin_high : pin_high;
}

void Button::Poll(uint32_t now_ms) {
  if (!initialized_)
    Panic(ErrorCode::kButtonReinit);

  const bool raw = ReadRawPressed();

  if (raw != raw_last_) {
    raw_last_ = raw;
    raw_last_change_ms_ = now_ms;
  }

  // Debounce acceptance
  if (raw != stable_) {
    if ((now_ms - raw_last_change_ms_) >= debounce_ms_) {
      stable_ = raw;

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
  const bool val = ev_press_;
  ev_press_ = false;
  return val;
}

bool Button::ConsumeRelease() {
  const bool val = ev_release_;
  ev_release_ = false;
  return val;
}

bool Button::ConsumeLongPress() {
  const bool val = ev_long_;
  ev_long_ = false;
  return val;
}
