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
#define ESC_VBA_PIN GPIO_PIN_0
#define ESC_VBA_GPIO_PORT GPIOC
#define ESC_CUR_PIN GPIO_PIN_1
#define ESC_CUR_GPIO_PORT GPIOC
#define ESC_TLM_PIN GPIO_PIN_11
#define ESC_TLM_GPIO_PORT GPIOB
#define ESC_S1_PIN GPIO_PIN_9
#define ESC_S1_GPIO_PORT GPIOE
#define ESC_S2_PIN GPIO_PIN_11
#define ESC_S2_GPIO_PORT GPIOE
#define ESC_S3_PIN GPIO_PIN_13
#define ESC_S3_GPIO_PORT GPIOE
#define ESC_S4_PIN GPIO_PIN_14
#define ESC_S4_GPIO_PORT GPIOE

#define SPI2_CS_Pin GPIO_PIN_4
#define SPI2_CS_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_15
#define SPI1_CS_GPIO_Port GPIOA
#define IMU_INT_Pin GPIO_PIN_10
#define IMU_INT_GPIO_Port GPIOB
#define IMU_INT_EXTI_IRQn EXTI15_10_IRQn

#ifdef __cplusplus
}
#endif

#endif  // CORE_BOARD_H
