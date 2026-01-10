#include "gpio.hpp"
#include "board.h"
#include "stm32f4xx_hal.h"

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, USER_LED_Pin | SPI1_CS_Pin, GPIO_PIN_SET);

  HAL_GPIO_WritePin(config.led.port, config.led.init.Pin, GPIO_PIN_RESET);

  InitPin(config.button.port, config.button.init);
  InitPin(config.led.port, config.led.init);
  InitPin(config.pb10.port, config.pb10.init);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IMU_INT_Pin */
  GPIO_InitStruct.Pin = IMU_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IMU_INT_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}
