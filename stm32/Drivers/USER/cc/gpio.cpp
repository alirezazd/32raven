#include "gpio.hpp"
#include "board.h"

namespace {

inline void EnablePortClock(GPIO_TypeDef *port) {
  if (port == GPIOA)
    __HAL_RCC_GPIOA_CLK_ENABLE();
  else if (port == GPIOB)
    __HAL_RCC_GPIOB_CLK_ENABLE();
  else if (port == GPIOC)
    __HAL_RCC_GPIOC_CLK_ENABLE();
  else if (port == GPIOD)
    __HAL_RCC_GPIOD_CLK_ENABLE();
  else if (port == GPIOE)
    __HAL_RCC_GPIOE_CLK_ENABLE();
  else if (port == GPIOF)
    __HAL_RCC_GPIOF_CLK_ENABLE();
  else if (port == GPIOG)
    __HAL_RCC_GPIOG_CLK_ENABLE();
  else if (port == GPIOH)
    __HAL_RCC_GPIOH_CLK_ENABLE();
}

inline bool IsOutputMode(uint32_t mode) {
  return mode == GPIO_MODE_OUTPUT_PP || mode == GPIO_MODE_OUTPUT_OD;
}

} // namespace

void GPIO::Init(const Config &cfg) {
  if (initialized_) {
    ErrorHandler();
  }
  initialized_ = true;

  // 1) Enable clocks only for ports actually used
  for (size_t i = 0; i < cfg.pin_count; i++) {
    EnablePortClock(cfg.pins[i].port);
  }

  // 2) Set safe default output levels BEFORE configuring outputs
  //    (prevents glitches when switching mode to output)
  for (size_t i = 0; i < cfg.pin_count; i++) {
    const auto &pc = cfg.pins[i];
    if (!IsOutputMode(pc.init.Mode))
      continue;

    // Board rule: USER LED is active-low -> default OFF = pin HIGH.
    // For other outputs you can extend this with per-pin defaults later.
    if (pc.port == USER_LED_GPIO_PORT && pc.init.Pin == USER_LED_Pin) {
      HAL_GPIO_WritePin(pc.port, pc.init.Pin, GPIO_PIN_SET); // OFF
    } else {
      // default low for other outputs unless specified otherwise
      HAL_GPIO_WritePin(pc.port, pc.init.Pin, GPIO_PIN_RESET);
    }
  }

  // 3) Apply pin configurations
  for (size_t i = 0; i < cfg.pin_count; i++) {
    GPIO_InitTypeDef tmp = cfg.pins[i].init;
    HAL_GPIO_Init(cfg.pins[i].port, &tmp);
  }
}
