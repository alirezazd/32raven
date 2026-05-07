#pragma once

#include "board.hpp"
#include "stm32f4xx_hal.h"
// Drivers
#include <array>

#include "gpio.hpp"
#include "stm32_config.hpp"
#include "stm32f4xx_hal_gpio.h"

// System Configuration Struct
struct SystemConfig {
  RCC_OscInitTypeDef osc;
  RCC_ClkInitTypeDef clk;
  uint32_t flashLatency;
  uint32_t voltageScaling;
};

// Default Configuration values
// 168 MHz from 8 MHz HSE (PLLM=8, PLLN=336, PLLP=2). APB1=42 MHz, APB2=84 MHz.
constexpr SystemConfig kSystemDefault = {
    .osc =
        {
            .OscillatorType = RCC_OSCILLATORTYPE_HSE,
            .HSEState = RCC_HSE_ON,
            .LSEState = 0,
            .HSIState = 0,
            .HSICalibrationValue = 0,
            .LSIState = 0,
            .PLL =
                {
                    .PLLState = RCC_PLL_ON,
                    .PLLSource = RCC_PLLSOURCE_HSE,
                    .PLLM = 8,
                    .PLLN = 336,
                    .PLLP = RCC_PLLP_DIV2,
                    .PLLQ = 4,
                },
        },
    .clk =
        {
            .ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2,
            .SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK,
            .AHBCLKDivider = RCC_SYSCLK_DIV1,
            .APB1CLKDivider = RCC_HCLK_DIV4,
            .APB2CLKDivider = RCC_HCLK_DIV2,
        },
    .flashLatency = FLASH_LATENCY_5,
    .voltageScaling = PWR_REGULATOR_VOLTAGE_SCALE1,
};

const std::array kGpioDefault = {
    GPIO::PinConfig{board::kUserLed.port,
                    {board::kUserLed.pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,
                     GPIO_SPEED_FREQ_LOW, 0},
                    true},  // LED Active Low
    GPIO::PinConfig{board::kUserBtn.port,
                    {board::kUserBtn.pin, GPIO_MODE_INPUT, GPIO_PULLDOWN,
                     GPIO_SPEED_FREQ_LOW, 0},
                    false},  // Button Active High
    GPIO::PinConfig{board::kImuInt.port,
                    {board::kImuInt.pin, GPIO_MODE_IT_RISING, GPIO_NOPULL,
                     GPIO_SPEED_FREQ_VERY_HIGH, board::kImuInt.af},
                    false},  // IMU INT
    GPIO::PinConfig{board::kSpi2Sck.port,
                    {board::kSpi2Sck.pin, GPIO_MODE_AF_PP, GPIO_NOPULL,
                     GPIO_SPEED_FREQ_VERY_HIGH, board::kSpi2Sck.af},
                    false},  // IMU SPI2 SCK (AF)
    GPIO::PinConfig{board::kSpi2MisoMosi.port,
                    {board::kSpi2MisoMosi.pin, GPIO_MODE_AF_PP, GPIO_NOPULL,
                     GPIO_SPEED_FREQ_VERY_HIGH, board::kSpi2MisoMosi.af},
                    false},  // IMU SPI2 MISO/MOSI (AF)
    GPIO::PinConfig{board::kSpi2Cs.port,
                    {board::kSpi2Cs.pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,
                     GPIO_SPEED_FREQ_VERY_HIGH, board::kSpi2Cs.af},
                    true},  // IMU SPI2 CS Active Low
    GPIO::PinConfig{board::kSpi1Sck.port,
                    {board::kSpi1Sck.pin, GPIO_MODE_AF_PP, GPIO_NOPULL,
                     GPIO_SPEED_FREQ_VERY_HIGH, board::kSpi1Sck.af},
                    false},  // EEPROM SPI1 SCK (AF)
    GPIO::PinConfig{board::kSpi1MisoMosi.port,
                    {board::kSpi1MisoMosi.pin, GPIO_MODE_AF_PP, GPIO_NOPULL,
                     GPIO_SPEED_FREQ_VERY_HIGH, board::kSpi1MisoMosi.af},
                    false},  // EEPROM SPI1 MISO/MOSI (AF)
    GPIO::PinConfig{board::kSpi1Cs.port,
                    {board::kSpi1Cs.pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,
                     GPIO_SPEED_FREQ_VERY_HIGH, 0},
                    true}  // Flash SPI1 CS Active Low
};
