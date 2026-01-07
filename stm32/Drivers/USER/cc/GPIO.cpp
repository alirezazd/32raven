#include "GPIO.hpp"

#include "board.h"

namespace {
inline void InitPin(GPIO_TypeDef *port, const GPIO_InitTypeDef &cfg) {
  GPIO_InitTypeDef tmp = cfg;
  HAL_GPIO_Init(port, &tmp);
}
} // namespace

void GPIO::Init(const Config &config) {
  if (initialized_) {
    ErrorHandler();
  }
  initialized_ = true;
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(config.led.port, config.led.init.Pin, GPIO_PIN_RESET);

  InitPin(config.button.port, config.button.init);
  InitPin(config.led.port, config.led.init);
  InitPin(config.pb10.port, config.pb10.init);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
