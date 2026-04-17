#pragma once

#include "board.h"
#include "stm32f4xx_hal.h"
// Drivers
#include <array>

#include "battery.hpp"
#include "button.hpp"
#include "dshot_tim1.hpp"
#include "gpio.hpp"
#include "i2c.hpp"
#include "icm42688p.hpp"
#include "led.hpp"
#include "stm32_config.hpp"
#include "stm32f4xx_hal_gpio.h"
#include "time_base.hpp"
#include "uart.hpp"

// System Configuration Struct
struct SystemConfig {
  RCC_OscInitTypeDef osc;
  RCC_ClkInitTypeDef clk;
  uint32_t flashLatency;
  uint32_t voltageScaling;
};

// Default Configuration values
constexpr SystemConfig kSystemDefault = {
    // RCC_OscInitTypeDef
    {RCC_OSCILLATORTYPE_HSE,  // OscillatorType
     RCC_HSE_ON,              // HSEState
     0,                       // LSEState (unused)
     0,                       // HSIState (unused)
     0,                       // HSICalibrationValue (unused)
     0,                       // LSIState (unused)
     {
         // PLL
         RCC_PLL_ON,         // PLLState
         RCC_PLLSOURCE_HSE,  // PLLSource
         8,                  // PLLM
         336,                // PLLN
         RCC_PLLP_DIV2,      // PLLP
         4                   // PLLQ
     }},
    // RCC_ClkInitTypeDef
    {
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
            RCC_CLOCKTYPE_PCLK2,  // ClockType
        RCC_SYSCLKSOURCE_PLLCLK,  // SYSCLKSource
        RCC_SYSCLK_DIV1,          // AHBCLKDivider
        RCC_HCLK_DIV4,            // APB1CLKDivider
        RCC_HCLK_DIV2             // APB2CLKDivider
    },
    FLASH_LATENCY_5,              // flashLatency
    PWR_REGULATOR_VOLTAGE_SCALE1  // voltageScaling
};

constexpr I2CConfig kI2cDefault = {400000,
                                   I2C_DUTYCYCLE_2,
                                   0,
                                   I2C_ADDRESSINGMODE_7BIT,
                                   I2C_DUALADDRESS_DISABLE,
                                   0,
                                   I2C_GENERALCALL_DISABLE,
                                   I2C_NOSTRETCH_DISABLE};

constexpr TimeBaseConfig kTimeBaseDefault = {
    .tim2 =
        {
            // 1MHz tick
            .prescaler = 83,       // Prescaler
            .period = 0xFFFFFFFF,  // Period
        },
    .tim5 = {
        // 1kHz tick
        .prescaler = 85,            // TIM5 Prescaler
        .period = 999,              // TIM5 Period
        .autoreload_preload = true  // TIM5 AutoReloadPreload
    }};

const std::array kGpioDefault = {
    GPIO::PinConfig{USER_LED_GPIO_PORT,
                    {USER_LED_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,
                     GPIO_SPEED_FREQ_LOW, 0},
                    true},  // LED Active Low
    GPIO::PinConfig{
        USER_BTN_GPIO_PORT,
        {USER_BTN_Pin, GPIO_MODE_INPUT, GPIO_PULLDOWN, GPIO_SPEED_FREQ_LOW, 0},
        false},  // Button Active High
    GPIO::PinConfig{GPIOB,
                    {GPIO_PIN_10, GPIO_MODE_IT_RISING, GPIO_NOPULL,
                     GPIO_SPEED_FREQ_VERY_HIGH, 0},
                    false},  // IMU INT
    GPIO::PinConfig{GPIOB,
                    {GPIO_PIN_13, GPIO_MODE_AF_PP, GPIO_NOPULL,
                     GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI2},
                    false},  // IMU SPI2 SCK (AF)
    GPIO::PinConfig{GPIOB,
                    {GPIO_PIN_14 | GPIO_PIN_15, GPIO_MODE_AF_PP, GPIO_NOPULL,
                     GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI2},
                    false},  // IMU SPI2 MISO/MOSI (AF)
    GPIO::PinConfig{SPI2_CS_GPIO_Port,
                    {SPI2_CS_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,
                     GPIO_SPEED_FREQ_VERY_HIGH, 0},
                    true},  // IMU SPI2 CS Active Low
    GPIO::PinConfig{SPI1_CS_GPIO_Port,
                    {SPI1_CS_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,
                     GPIO_SPEED_FREQ_VERY_HIGH, 0},
                    true}  // Flash SPI1 CS Active Low
};

const LED::Config kLedDefault = {
    .pin = {.port = USER_LED_GPIO_PORT, .number = USER_LED_Pin},
    .active_low = true};

constexpr DShotTim1::Config kDshotTim1Default = {
    DShotMode::DSHOT600,  // mode
};

inline constexpr Battery::Config kBatteryConfig = {
    .voltage_v = 16.0f,
    .current_a = 1.2f,
    .mah_drawn = 250.0f,
    .percentage = 75,
};
