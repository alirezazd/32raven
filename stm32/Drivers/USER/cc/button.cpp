#include "button.hpp"

void Button::Init(const Config &config) {
  if (initialized_) {
    return;
  }

  pin_ = config.pin;
  active_low_ = config.active_low;
  debounce_ms_ = config.debounce_ms;
  long_press_ms_ = config.long_press_ms;

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin */
  GPIO_InitStruct.Pin = pin_;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;

  if (config.pullup) {
    GPIO_InitStruct.Pull = GPIO_PULLUP;
  } else if (config.pulldown) {
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  } else {
    GPIO_InitStruct.Pull = GPIO_NOPULL;
  }

  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  // Initialize debouncer
  raw_last_ = ReadRawPressed();
  stable_ = raw_last_;
  pressed_ = stable_;
  raw_last_change_ms_ = 0;
  press_start_ms_ = 0;
  long_fired_ = false;

  ev_press_ = false;
  ev_release_ = false;
  ev_long_ = false;

  initialized_ = true;
}

bool Button::ReadRawPressed() const {
  const bool kRaw = (HAL_GPIO_ReadPin(GPIOA, pin_) == GPIO_PIN_SET);
  return active_low_ ? !kRaw : kRaw;
}

void Button::Poll(uint32_t now_ms) {
  if (!initialized_) {
    return;
  }

  const bool kRaw = ReadRawPressed();

  if (kRaw != raw_last_) {
    raw_last_ = kRaw;
    raw_last_change_ms_ = now_ms;
  }

  // if raw has been stable long enough, accept it
  if (kRaw != stable_) {
    if ((now_ms - raw_last_change_ms_) >= debounce_ms_) {
      stable_ = kRaw;

      // stable edge
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

  // long press detection
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

bool Button::ConsumeLongPress() {
  const bool kVal = ev_long_;
  ev_long_ = false;
  return kVal;
}

bool Button::ConsumeRelease() {
  const bool kVal = ev_release_;
  ev_release_ = false;
  return kVal;
}
