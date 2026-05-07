#pragma once

// 32Raven board pin map.
//
// Single source of truth for this PCB's pin assignments. Hand-curated; mirrors
// the CubeMX .ioc that bootstrapped the project. Drivers consume entries via
// `board::k<Name>` rather than reading globals from a peripheral driver.
//
// Each BoardPin bundles port + pin + (where applicable) alternate function and
// EXTI line, so a typo or mismatched pair becomes a compile error rather than
// a silent runtime brick.

#include "stm32f4xx_hal.h"

namespace board {

struct BoardPin {
  GPIO_TypeDef* port;
  uint16_t pin;
  uint8_t af;            // alternate function (0 = plain GPIO)
  IRQn_Type exti_irqn;   // EXTI IRQ for input pins (NonMaskableInt_IRQn = none)
};

// User I/O.
inline const BoardPin kUserBtn = {GPIOA, GPIO_PIN_0, 0, NonMaskableInt_IRQn};
inline const BoardPin kUserLed = {GPIOA, GPIO_PIN_1, 0, NonMaskableInt_IRQn};

// ESC monitoring.
inline const BoardPin kEscVba = {GPIOC, GPIO_PIN_0,  0, NonMaskableInt_IRQn};
inline const BoardPin kEscCur = {GPIOC, GPIO_PIN_1,  0, NonMaskableInt_IRQn};
inline const BoardPin kEscTlm = {GPIOB, GPIO_PIN_11, 0, NonMaskableInt_IRQn};

// IMU (ICM42688P on SPI2, EXTI on PB10).
// SPI2 bus pins are AF5; CS is plain GPIO toggled by the driver.
inline const BoardPin kImuInt       = {GPIOB, GPIO_PIN_10,               0,             EXTI15_10_IRQn};
inline const BoardPin kSpi2Sck      = {GPIOB, GPIO_PIN_13,               GPIO_AF5_SPI2, NonMaskableInt_IRQn};
inline const BoardPin kSpi2MisoMosi = {GPIOB, GPIO_PIN_14 | GPIO_PIN_15, GPIO_AF5_SPI2, NonMaskableInt_IRQn};
inline const BoardPin kSpi2Cs       = {GPIOA, GPIO_PIN_4,                0,             NonMaskableInt_IRQn};

// EEPROM / external flash on SPI1.
inline const BoardPin kSpi1Sck      = {GPIOB, GPIO_PIN_3,                GPIO_AF5_SPI1, NonMaskableInt_IRQn};
inline const BoardPin kSpi1MisoMosi = {GPIOB, GPIO_PIN_4 | GPIO_PIN_5,   GPIO_AF5_SPI1, NonMaskableInt_IRQn};
inline const BoardPin kSpi1Cs       = {GPIOA, GPIO_PIN_15,               0,             NonMaskableInt_IRQn};

}  // namespace board
