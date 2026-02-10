#ifndef CORE_BOARD_H
#define CORE_BOARD_H

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Exported functions prototypes ---------------------------------------------*/
void ErrorHandler(void);

/* Private defines -----------------------------------------------------------*/
#define USER_BTN_Pin GPIO_PIN_0
#define USER_BTN_GPIO_PORT GPIOA
#define USER_LED_Pin GPIO_PIN_1
#define USER_LED_GPIO_PORT GPIOA

#define SPI1_CS_Pin GPIO_PIN_4
#define SPI1_CS_GPIO_Port GPIOA
#define IMU_INT_Pin GPIO_PIN_10
#define IMU_INT_GPIO_Port GPIOB
#define IMU_FSYNC_Pin GPIO_PIN_11
#define IMU_FSYNC_GPIO_Port GPIOB
#define IMU_INT_EXTI_IRQn EXTI15_10_IRQn
#define GPS_TIMEPULSE_Pin GPIO_PIN_15
#define GPS_TIMEPULSE_GPIO_Port GPIOA

#ifdef __cplusplus
}
#endif

#endif // CORE_BOARD_H
