#include "button.hpp"

#include "panic.hpp"

extern "C" {
#include "driver/gpio.h"
#include "esp_log.h"
}

static constexpr const char *kTag = "button";

void Button::Init(const Config &cfg) {
  pin_ = cfg.input.pin;
  active_low_ = cfg.input.active_low;
  debounce_ms_ = cfg.timing.debounce_ms;
  long_press_ms_ = cfg.timing.long_press_ms;
  long_long_press_ms_ = cfg.timing.long_long_press_ms;

  gpio_config_t io_conf = {};
  io_conf.intr_type = GPIO_INTR_DISABLE;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pin_bit_mask = (1ULL << pin_);
  io_conf.pull_down_en =
      cfg.input.pulldown ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en =
      cfg.input.pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;

  if (gpio_config(&io_conf) != ESP_OK) {
    Panic(ErrorCode::kButtonGpioConfigFailed);
  }

  // Initialize debouncer from current level
  raw_last_ = ReadRawPressed();
  stable_ = raw_last_;
  pressed_ = stable_;
  raw_last_change_ms_ = 0;
  press_start_ms_ = 0;
  long_fired_ = false;
  long_long_fired_ = false;

  ev_press_ = false;
  ev_long_ = false;
  ev_long_long_ = false;

  ESP_LOGI(
      kTag,
      "init pin=%d active_low=%d pullup=%d pulldown=%d debounce=%u long=%u "
      "long_long=%u",
      static_cast<int>(cfg.input.pin), static_cast<int>(cfg.input.active_low),
      static_cast<int>(cfg.input.pullup), static_cast<int>(cfg.input.pulldown),
      static_cast<unsigned>(cfg.timing.debounce_ms),
      static_cast<unsigned>(cfg.timing.long_press_ms),
      static_cast<unsigned>(cfg.timing.long_long_press_ms));
}

bool Button::ReadRawPressed() const {
  const int lvl = gpio_get_level(pin_);
  const bool raw_pressed = (lvl != 0);
  return active_low_ ? !raw_pressed : raw_pressed;
}

void Button::Poll() {
  const TimeMs now_ms = Timebase::GetInstance().NowMs();
  const bool raw = ReadRawPressed();

  if (raw != raw_last_) {
    raw_last_ = raw;
    raw_last_change_ms_ = now_ms;
    ESP_LOGD(kTag, "raw change -> %d at %u ms", (int)raw, (unsigned)now_ms);
  }

  if (raw != stable_) {
    if ((TimeMs)(now_ms - raw_last_change_ms_) >= debounce_ms_) {
      stable_ = raw;
      ESP_LOGI(kTag, "debounced -> %d at %u ms", (int)stable_,
               (unsigned)now_ms);

      if (stable_) {
        pressed_ = true;
        press_start_ms_ = now_ms;
        long_fired_ = false;
        long_long_fired_ = false;
        ev_press_ = true;
      } else {
        pressed_ = false;
        press_start_ms_ = 0;
        long_fired_ = false;
        long_long_fired_ = false;
      }
    }
  }

  if (pressed_ && !long_fired_) {
    if ((TimeMs)(now_ms - press_start_ms_) >= long_press_ms_) {
      long_fired_ = true;
      ev_long_ = true;
      ESP_LOGW(kTag, "LONG press fired at %u ms", (unsigned)now_ms);
    }
  }

  if (pressed_ && long_fired_ && !long_long_fired_) {
    if ((TimeMs)(now_ms - press_start_ms_) >= long_long_press_ms_) {
      long_long_fired_ = true;
      ev_long_long_ = true;
      ESP_LOGW(kTag, "LONG-LONG press fired at %u ms", (unsigned)now_ms);
    }
  }
}

bool Button::ConsumePress() {
  const bool fired = ev_press_;
  ev_press_ = false;
  return fired;
}

bool Button::ConsumeLongPress() {
  const bool fired = ev_long_;
  ev_long_ = false;
  return fired;
}

bool Button::ConsumeLongLongPress() {
  const bool fired = ev_long_long_;
  ev_long_long_ = false;
  return fired;
}

void Button::FlushEvents() {
  ev_press_ = false;
  ev_long_ = false;
  ev_long_long_ = false;
}
