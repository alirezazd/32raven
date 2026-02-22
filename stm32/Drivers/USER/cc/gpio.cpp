#include "gpio.hpp"
#include "board.h"

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
  else {
    Panic(ErrorCode::kGpioInvalidPort);
  }
}

inline bool IsOutputMode(uint32_t mode) {
  return mode == GPIO_MODE_OUTPUT_PP || mode == GPIO_MODE_OUTPUT_OD;
}

void GPIO::Init(const PinConfig *pins, size_t pin_count) {
  if (initialized_) {
    Panic(ErrorCode::kGpioReinit);
  }
  initialized_ = true;
  for (size_t i = 0; i < pin_count; i++) {
    EnablePortClock(pins[i].port);
  }

  // Set safe default output levels BEFORE configuring outputs
  //    (prevents glitches when switching mode to output)
  for (size_t i = 0; i < pin_count; i++) {
    const auto &pc = pins[i];
    if (!IsOutputMode(pc.init.Mode))
      continue;
    bool initial_level = pc.active_low ? true : false;
    WritePin(pc.port, pc.init.Pin, initial_level);
  }
  for (size_t i = 0; i < pin_count; i++) {
    GPIO_InitTypeDef tmp = pins[i].init;
    // TODO: Get rid of HAL
    HAL_GPIO_Init(pins[i].port, &tmp);
  }
}

void GPIO::WritePin(GPIO_TypeDef *port, uint16_t pin, bool state) {
  if (state) {
    port->BSRR = pin;
  } else {
    port->BSRR = (uint32_t)pin << 16U;
  }
}

bool GPIO::ReadPin(GPIO_TypeDef *port, uint16_t pin) {
  return (port->IDR & pin) != 0;
}
