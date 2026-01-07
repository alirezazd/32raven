#ifndef CORE_BOARD_H
#define CORE_BOARD_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported functions prototypes ---------------------------------------------*/
void ErrorHandler(void);

/* Private defines -----------------------------------------------------------*/
#define USER_BTN_PIN GPIO_PIN_0
#define USER_BTN_GPIO_PORT GPIOA
#define USER_LED_PIN GPIO_PIN_1
#define USER_LED_GPIO_PORT GPIOA

#ifdef __cplusplus
}
#endif

#endif // CORE_BOARD_H
