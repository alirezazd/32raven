#pragma once

#include "board.h"
#include "stm32f4xx_hal.h"
// Drivers
#include "button.hpp"
#include "dshot_tim1.hpp"
#include "gpio.hpp"
#include "i2c.hpp"
#include "led.hpp"
#include "time_base.hpp"
// Note: uart.hpp and system.hpp are consumers, not providers of config types
// now.

// System Configuration Struct
struct SystemConfig {
  RCC_OscInitTypeDef osc;
  RCC_ClkInitTypeDef clk;
  uint32_t flashLatency;
  uint32_t voltageScaling;
};

// UART Configuration Struct
struct UartConfig {
  uint32_t baudRate;
  uint32_t wordLength;
  uint32_t stopBits;
  uint32_t parity;
  uint32_t mode;
  uint32_t hwFlowCtl;
  uint32_t overSampling;
  size_t tx_buffer_size;
  size_t rx_dma_size;
  size_t rx_ring_size;
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
    83,         // Prescaler
    0xFFFFFFFF, // Period
};

static const GPIO::PinConfig kPins[] = {
    {USER_LED_GPIO_PORT,
     {USER_LED_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0}},
    {USER_BTN_GPIO_PORT,
     {USER_BTN_Pin, GPIO_MODE_INPUT, GPIO_PULLDOWN, GPIO_SPEED_FREQ_LOW, 0}},
    {GPIOB,
     {GPIO_PIN_10, GPIO_MODE_IT_RISING, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0}},
    // SPI1: SCK=PA5, MISO=PA6, MOSI=PA7 (AF5)
    {GPIOA,
     {GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7, GPIO_MODE_AF_PP, GPIO_NOPULL,
      GPIO_SPEED_FREQ_VERY_HIGH, GPIO_AF5_SPI1}},
    // SPI1 CS: PA4
    {GPIOA,
     {GPIO_PIN_4, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_VERY_HIGH,
      0}},
};

const GPIO::Config kGpioDefault = {kPins, sizeof(kPins) / sizeof(kPins[0])};

// LED: Port, Pin, ActiveLow
const LED::Config kLedDefault = {USER_LED_GPIO_PORT, USER_LED_Pin,
                                 true}; // Active Low

// Button: Pin, Debounce, LongPress
const Button::Config kButtonDefault = {
    USER_BTN_GPIO_PORT, USER_BTN_Pin,
    false, // active_low: button drives HIGH when pressed
    50, 500};

constexpr DShotTim1::Config kDshotTim1Default = {
    DShotMode::DSHOT600, // mode
};

// Buffer size constants used by templates
constexpr size_t kUartTxBufSize = 256;
constexpr size_t kUartRxDmaSize = 128;   // Low latency
constexpr size_t kUartRxRingSize = 1024; // Large capacity

constexpr UartConfig kUart1Config = {
    115200,               // baudRate (Console)
    UART_WORDLENGTH_9B,   // wordLength (8 data + 1 parity)
    UART_STOPBITS_1,      // stopBits
    UART_PARITY_EVEN,     // parity
    UART_MODE_TX_RX,      // mode
    UART_HWCONTROL_NONE,  // hwFlowCtl
    UART_OVERSAMPLING_16, // overSampling
    kUartTxBufSize,       // tx_buffer_size
    kUartRxDmaSize,       // rx_dma_size
    kUartRxRingSize       // rx_ring_size
};

constexpr UartConfig kUart2Config = {
    115200,               // baudRate (GPS M9N)
    UART_WORDLENGTH_8B,   // wordLength
    UART_STOPBITS_1,      // stopBits
    UART_PARITY_NONE,     // parity
    UART_MODE_TX_RX,      // mode
    UART_HWCONTROL_NONE,  // hwFlowCtl
    UART_OVERSAMPLING_16, // overSampling
    kUartTxBufSize,       // tx_buffer_size
    kUartRxDmaSize,       // rx_dma_size
    kUartRxRingSize       // rx_ring_size
};

// SPI Configuration Struct
enum class SpiPrescaler : uint8_t {
  kDiv2 = 0,
  kDiv4 = 1,
  kDiv8 = 2,
  kDiv16 = 3,
  kDiv32 = 4,
  kDiv64 = 5,
  kDiv128 = 6,
  kDiv256 = 7,
};

enum class SpiPolarity : uint8_t { kLow = 0, kHigh = 1 };
enum class SpiPhase : uint8_t { k1Edge = 0, k2Edge = 1 };
enum class SpiBitOrder : uint8_t { kMsbFirst = 0, kLsbFirst = 1 };

struct SpiConfig {
  SpiPolarity polarity;
  SpiPhase phase;
  SpiPrescaler prescaler;
  SpiBitOrder bit_order;
};

constexpr SpiConfig kSpi1Config = {SpiPolarity::kHigh, SpiPhase::k2Edge,
                                   SpiPrescaler::kDiv32,
                                   SpiBitOrder::kMsbFirst};

// Set to true to re-flash M9N persistent configuration on boot.
// This will force connection at 38400 baud, send golden config to Flash/BBR,
// and reboot/reinit UARTs to 115200.
constexpr bool kFlashM9nConfig = false;