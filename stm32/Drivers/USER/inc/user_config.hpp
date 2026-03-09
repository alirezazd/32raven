#pragma once

#include "board.h"
#include "stm32f4xx_hal.h"
// Drivers
#include "button.hpp"
#include "dshot_tim1.hpp"
#include "gpio.hpp"
#include "i2c.hpp"
#include "icm20948.hpp"
#include "icm42688p.hpp"
#include "led.hpp"
#include "m9n.hpp"
#include "spi.hpp"
#include "stm32_config.hpp"
#include "stm32f4xx_hal_gpio.h"
#include "time_base.hpp"
#include "uart.hpp"
#include <array>

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
    {RCC_OSCILLATORTYPE_HSE, // OscillatorType
     RCC_HSE_ON,             // HSEState
     0,                      // LSEState (unused)
     0,                      // HSIState (unused)
     0,                      // HSICalibrationValue (unused)
     0,                      // LSIState (unused)
     {
         // PLL
         RCC_PLL_ON,        // PLLState
         RCC_PLLSOURCE_HSE, // PLLSource
         8,                 // PLLM
         336,               // PLLN
         RCC_PLLP_DIV2,     // PLLP
         4                  // PLLQ
     }},
    // RCC_ClkInitTypeDef
    {
        RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 |
            RCC_CLOCKTYPE_PCLK2, // ClockType
        RCC_SYSCLKSOURCE_PLLCLK, // SYSCLKSource
        RCC_SYSCLK_DIV1,         // AHBCLKDivider
        RCC_HCLK_DIV4,           // APB1CLKDivider
        RCC_HCLK_DIV2            // APB2CLKDivider
    },
    FLASH_LATENCY_5,             // flashLatency
    PWR_REGULATOR_VOLTAGE_SCALE1 // voltageScaling
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
            .prescaler = 83,      // Prescaler
            .period = 0xFFFFFFFF, // Period
            .compensation = 1     // Compensation (us)
        },
    .tim5 = {
        // 1kHz tick
        .prescaler = 85,           // TIM5 Prescaler
        .period = 999,             // TIM5 Period
        .autoreload_preload = true // TIM5 AutoReloadPreload
    }};

const std::array kGpioDefault = {
    GPIO::PinConfig{USER_LED_GPIO_PORT,
                    {USER_LED_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,
                     GPIO_SPEED_FREQ_LOW, 0},
                    true}, // LED Active Low
    GPIO::PinConfig{
        USER_BTN_GPIO_PORT,
        {USER_BTN_Pin, GPIO_MODE_INPUT, GPIO_PULLDOWN, GPIO_SPEED_FREQ_LOW, 0},
        false}, // Button Active High
    GPIO::PinConfig{GPIOB,
                    {GPIO_PIN_10, GPIO_MODE_IT_RISING, GPIO_NOPULL,
                     GPIO_SPEED_FREQ_VERY_HIGH, 0},
                    false}, // IMU INT
    GPIO::PinConfig{GPIOA,
                    {GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_MODE_AF_PP,
                     GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI1},
                    false}, // SPI Pins (AF)
    GPIO::PinConfig{GPIOA,
                    {GPIO_PIN_4, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL,
                     GPIO_SPEED_FREQ_VERY_HIGH, 0},
                    true} // SPI CS Active Low
};

const LED::Config kLedDefault = {
    .pin = {.port = USER_LED_GPIO_PORT, .number = USER_LED_Pin},
    .active_low = true};

const Button::Config kButtonDefault = {
    .pin = {.port = USER_BTN_GPIO_PORT, .number = USER_BTN_Pin},
    .active_low = false, // active_low: button drives HIGH when pressed
    .debounce_ms = 50,
    .long_press_ms = 500};

constexpr DShotTim1::Config kDshotTim1Default = {
    DShotMode::DSHOT600, // mode
};

constexpr UartConfig kUart1Config = {
    .baud_rate = 5000000, // Link to ESP32
    .word_length = UartWordLength::k9Bits,
    .stop_bits = UartStopBits::k1,
    .parity = UartParity::kEven,
    .mode = UartMode::kTxRx,
    .hw_flow_control = UartHwFlowControl::kNone,
    .over_sampling = UartOverSampling::k16,
};

constexpr UartConfig kUart2Config = {
    .baud_rate = 115200, // GPS M9N
    .word_length = UartWordLength::k8Bits,
    .stop_bits = UartStopBits::k1,
    .parity = UartParity::kNone,
    .mode = UartMode::kTxRx,
    .hw_flow_control = UartHwFlowControl::kNone,
    .over_sampling = UartOverSampling::k16,
};

constexpr SpiConfig kSpi1Config = {
    .polarity = SpiPolarity::kHigh,
    .phase = SpiPhase::k2Edge,
    .prescaler = SpiPrescaler::kDiv32,
    .bit_order = SpiBitOrder::kMsbFirst};

// Set to true to re-flash M9N persistent configuration on boot.
// This will force connection at 38400 baud, send golden config to Flash/BBR,
// and reboot/reinit UARTs to 115200.
constexpr M9N::Config kM9nConfig = {
    .flash_config = false,
    .baud_rate = 115200,

    .protocols = {.outprot_ubx = true, .outprot_nmea = false},
    .messages = {.nav_pvt = true,
                 .nav_dop = false,
                 .nav_cov = true,
                 .nav_eoe = true},
    .nav = {.rate_meas_ms = 100, .dyn_model = 7},

    .gnss = {.gps_enable = true,
             .glo_enable = false,
             .gal_enable = true,
             .bds_enable = false,
             .sbas_enable = true,
             .itfm_enable = true},

    .tp1 = {.ena = true,
            .period = 1000000,
            .len = 50000,
            .timegrid = 1,
            .sync_gnss = true,
            .use_locked = true,
            .align_to_tow = true,
            .pol_rising = true,
            .period_lock = 1000000,
            .len_lock = 50000},

    .ack_timeout_us = 100000};

// ICM20948 Configuration
// Default values match previous driver hardcoded settings
constexpr Icm20948::Config kIcm20948Config = {
    .dma_buf_size = 32,
    .fifo_size = 512,
    .accel = {.range = 0x06,       // 16G (Bits 2|1)
              .dlpf_config = 0x19, // 3 << 3 | 1 (Enabled, ~111Hz BW)
              .sample_rate_div = 0},
    .gyro = {.range = 0x06,       // 2000dps (Bits 2|1)
             .dlpf_config = 0x19, // 3 << 3 | 1 (Enabled, ~119Hz BW)
             .sample_rate_div = 0},
    .mag_rate = 0x08, // Bank 3 AK09916 CNTL2: MODE4 (100Hz) = 0x08 (Bit3)
    .spi_prescaler = static_cast<uint8_t>(SpiPrescaler::kDiv32), // ~2.6MHz
    .who_am_i = 0xEA,
};
