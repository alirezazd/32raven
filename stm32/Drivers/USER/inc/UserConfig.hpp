#include "DShotTim1.hpp"
#include "GPIO.hpp"
#include "I2C.hpp"
#include "LED.hpp"
#include "TimeBase.hpp"
#include "stm32f4xx_hal.h"
#include "system.hpp" // For C-macros (USER_LED_Pin, etc)
#include "system.hpp" // For System::Config

// Default Configuration values
constexpr System::Config SYSTEM_DEFAULT = {
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

constexpr I2CConfig I2C_DEFAULT = {400000,
                                   I2C_DUTYCYCLE_2,
                                   0,
                                   I2C_ADDRESSINGMODE_7BIT,
                                   I2C_DUALADDRESS_DISABLE,
                                   0,
                                   I2C_GENERALCALL_DISABLE,
                                   I2C_NOSTRETCH_DISABLE};

constexpr TimeBaseConfig TimeBase_DEFAULT = {
    83,         // Prescaler
    0xFFFFFFFF, // Period
};

constexpr GPIO::Config GPIO_DEFAULT = {
    // LED (Output, No Pull, Low Speed)
    {USER_LED_GPIO_Port,
     {USER_LED_Pin, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0}},

    // Button (Input, Pull Down)
    {USER_BTN_GPIO_Port,
     {USER_BTN_Pin, GPIO_MODE_INPUT, GPIO_PULLDOWN, GPIO_SPEED_FREQ_LOW, 0}},

    // PB10 (EXTI Rising, No Pull)
    {GPIOB,
     {GPIO_PIN_10, GPIO_MODE_IT_RISING, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW, 0}},
};

constexpr LED::Config LED_DEFAULT = {USER_LED_GPIO_Port, USER_LED_Pin};

constexpr DShotTim1::Config DSHOT_TIM1_DEFAULT = {
    DShotMode::DSHOT600, // mode
};