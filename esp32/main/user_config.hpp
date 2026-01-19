#pragma once

#include "uart.hpp"

// STM32 UART Link Configuration
constexpr Uart::Config kStm32UartConfig = {
    .uart_num = 1,
    .tx_gpio = GPIO_NUM_4,
    .rx_gpio = GPIO_NUM_5,
    .baud_rate = 115200, // Console
    .parity = Uart::Config::Parity::kEven,
    .rx_buf = 2048,
    .tx_buf = 2048,
};

// EP2 Receiver UART Configuration
constexpr Uart::Config kEp2UartConfig = {
    .uart_num = 0,
    .tx_gpio = GPIO_NUM_20, // Verify with schematic
    .rx_gpio = GPIO_NUM_21, // Verify with schematic
    .baud_rate = 115200,    // Configurable?
    .parity = Uart::Config::Parity::kNone,
    .rx_buf = 2048,
    .tx_buf = 2048,
};
