#include "button.hpp"

extern "C" {
#include "driver/gpio.h"
#include "esp_log.h"
}

static constexpr const char *kTag = "button";

ErrorCode Button::Init(const Config &cfg) {
  if (initialized_)
    return ErrorCode::kOk;

  pin_ = cfg.pin;
  active_low_ = cfg.active_low;
  debounce_ms_ = cfg.debounce_ms;
  long_press_ms_ = cfg.long_press_ms;

  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = (1ULL << pin_);
  io_conf.pull_down_en =
      cfg.pulldown ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en = cfg.pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;

  if (gpio_config(&io_conf) != ESP_OK) {
    return ErrorCode::kButtonGpioConfigFailed;
  }

  // Initialize debouncer from current level
  raw_last_ = ReadRawPressed();
  stable_ = raw_last_;
  pressed_ = stable_;
  raw_last_change_ms_ = 0;
  press_start_ms_ = 0;
  long_fired_ = false;

  ev_press_ = false;
  ev_release_ = false;
  ev_long_ = false;

  ESP_LOGI(
      kTag,
      "init pin=%d active_low=%d pullup=%d pulldown=%d debounce=%u long=%u",
      (int)cfg.pin, (int)cfg.active_low, (int)cfg.pullup, (int)cfg.pulldown,
      (unsigned)cfg.debounce_ms, (unsigned)cfg.long_press_ms);

  initialized_ = true;
  return ErrorCode::kOk;
}

bool Button::ReadRawPressed() const {
  const int kLvl = gpio_get_level(pin_);
  const bool kRawPressed = (kLvl != 0);
  return active_low_ ? !kRawPressed : kRawPressed;
}

void Button::Poll(TimeMs now_ms) {
  if (!initialized_)
    return;

  const bool kRaw = ReadRawPressed();

  if (kRaw != raw_last_) {
    ESP_LOGD(kTag, "raw change -> %d at %u ms", (int)kRaw, (unsigned)now_ms);
    raw_last_ = kRaw;
    raw_last_change_ms_ = now_ms;
  }

  // if raw has been stable long enough, accept it
  if (kRaw != stable_) {
    if ((TimeMs)(now_ms - raw_last_change_ms_) >= debounce_ms_) {
      stable_ = kRaw;
      ESP_LOGI(kTag, "debounced -> %d at %u ms", (int)stable_,
               (unsigned)now_ms);

      // stable edge
      if (stable_) {
        pressed_ = true;
        press_start_ms_ = now_ms;
        long_fired_ = false;
        ev_press_ = true;
      } else {
        pressed_ = false;
        press_start_ms_ = 0;
        long_fired_ = false;
        ev_release_ = true;
      }
    }
  }

  // long press detection: fire once per hold
  if (pressed_ && !long_fired_) {
    if ((TimeMs)(now_ms - press_start_ms_) >= long_press_ms_) {
      long_fired_ = true;
      ev_long_ = true;
      ESP_LOGW(kTag, "LONG press fired at %u ms", (unsigned)now_ms);
    }
  }
}

bool Button::ConsumeLongPress() {
  const bool kV = ev_long_;
  ev_long_ = false;
  return kV;
}

bool Button::ConsumePress() {
  const bool kV = ev_press_;
  ev_press_ = false;
  return kV;
}

bool Button::ConsumeRelease() {
  const bool kV = ev_release_;
  ev_release_ = false;
  return kV;
}
