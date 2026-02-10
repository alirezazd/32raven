#pragma once

#include "button.hpp"

#include "uart.hpp"

#include "mavlink.hpp"

// STM32 UART Link Configuration
constexpr Uart::Config kStm32UartConfig = {
    .uart_num = 0,
    .tx_gpio = GPIO_NUM_4,
    .rx_gpio = GPIO_NUM_5,
    .baud_rate = 5000000, // Link to STM32
    .parity = Uart::Config::Parity::kEven,
    .rx_buf = 2048,
    .tx_buf = 2048,
};

// EP2 Receiver UART Configuration
constexpr Uart::Config kEp2UartConfig = {
    .uart_num = 1,
    .tx_gpio = GPIO_NUM_20,
    .rx_gpio = GPIO_NUM_21,
    .baud_rate = 460800, // Fixed ELRS MAVLink rate
    .parity = Uart::Config::Parity::kNone,
    .rx_buf = 2048,
    .tx_buf = 256,
};

// Button Configuration
constexpr Button::Config kButtonConfig = {
    .pin = GPIO_NUM_9, // Boot button on C3/S3 often 9, checking defaults
    .active_low = true,
    .pullup = true,
    .pulldown = false,
    .debounce_ms = 30,
    .long_press_ms = 500,
};

constexpr Mavlink::Config kMavlinkConfig = {
    .sysid = 1,
    .compid = MAV_COMP_ID_AUTOPILOT1,
    .hb_period_ms = 1000,
    .gps_period_ms = 2000,
    .att_period_ms = 2000,
    .gpos_period_ms = 2000,
    .batt_period_ms = 2000,
};

constexpr uint8_t kTelemetryRateHz = 25;
