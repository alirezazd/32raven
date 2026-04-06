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

  const TimeMs now_ms = Timebase::GetInstance().NowMs();
  raw_pressed_ = ReadRawPressed();
  debounce_started_ms_ = now_ms;
  phase_ = raw_pressed_ ? Phase::kPressed : Phase::kReleased;
  press_started_ms_ = raw_pressed_ ? now_ms : 0;
  long_fired_ = false;
  long_long_fired_ = false;
  ev_press_ = false;
  ev_long_ = false;
  ev_long_long_ = false;
}

bool Button::ReadRawPressed() const {
  const int lvl = gpio_get_level(pin_);
  const bool raw_pressed = (lvl != 0);
  return active_low_ ? !raw_pressed : raw_pressed;
}

void Button::EnterPressed(TimeMs now_ms) {
  phase_ = Phase::kPressed;
  press_started_ms_ = now_ms;
  long_fired_ = false;
  long_long_fired_ = false;
  ev_press_ = true;
  ESP_LOGI(kTag, "debounced -> 1 at %u ms", (unsigned)now_ms);
}

void Button::EnterReleased(TimeMs now_ms) {
  phase_ = Phase::kReleased;
  press_started_ms_ = 0;
  long_fired_ = false;
  long_long_fired_ = false;
  ESP_LOGI(kTag, "debounced -> 0 at %u ms", (unsigned)now_ms);
}

void Button::UpdateHoldEvents(TimeMs now_ms) {
  if (press_started_ms_ == 0) {
    return;
  }

  const TimeMs held_ms = static_cast<TimeMs>(now_ms - press_started_ms_);
  if (!long_fired_ && held_ms >= long_press_ms_) {
    long_fired_ = true;
    ev_long_ = true;
    ESP_LOGW(kTag, "LONG press fired at %u ms", (unsigned)now_ms);
  }

  if (long_fired_ && !long_long_fired_ && held_ms >= long_long_press_ms_) {
    long_long_fired_ = true;
    ev_long_long_ = true;
    ESP_LOGW(kTag, "LONG-LONG press fired at %u ms", (unsigned)now_ms);
  }
}

void Button::Poll() {
  const TimeMs now_ms = Timebase::GetInstance().NowMs();
  const bool raw_pressed = ReadRawPressed();

  if (raw_pressed != raw_pressed_) {
    raw_pressed_ = raw_pressed;
    debounce_started_ms_ = now_ms;
  }

  switch (phase_) {
    case Phase::kReleased:
      if (raw_pressed_) {
        phase_ = Phase::kDebouncingPress;
      }
      break;

    case Phase::kDebouncingPress:
      if (!raw_pressed_) {
        phase_ = Phase::kReleased;
      } else if (static_cast<TimeMs>(now_ms - debounce_started_ms_) >=
                 debounce_ms_) {
        EnterPressed(now_ms);
      }
      break;

    case Phase::kPressed:
      if (!raw_pressed_) {
        phase_ = Phase::kDebouncingRelease;
      }
      break;

    case Phase::kDebouncingRelease:
      if (raw_pressed_) {
        phase_ = Phase::kPressed;
      } else if (static_cast<TimeMs>(now_ms - debounce_started_ms_) >=
                 debounce_ms_) {
        EnterReleased(now_ms);
      }
      break;
  }

  if (phase_ == Phase::kPressed) {
    UpdateHoldEvents(now_ms);
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
