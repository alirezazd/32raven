#ifndef CORE_BOARD_H
#define CORE_BOARD_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Private defines -----------------------------------------------------------*/
#define USER_BTN_Pin GPIO_PIN_0
#define USER_BTN_GPIO_Port GPIOA
#define USER_LED_Pin GPIO_PIN_1
#define USER_LED_GPIO_Port GPIOA

#ifdef __cplusplus
}
#endif

#endif // CORE_BOARD_H
