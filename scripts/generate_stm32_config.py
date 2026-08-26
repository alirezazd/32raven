#!/usr/bin/env python3

# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi

# /// script
# dependencies = [
#     "jinja2",
#     "kconfiglib",
# ]
# ///

from __future__ import annotations

import math
import pathlib
import sys
import tomllib
from dataclasses import dataclass

import kconfiglib

# Sibling module — pin_constraints.py lives next to this script.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from kconfig_gen import (
    autogen_warning,
    choice_value,
    cpp_string_literal,
    sym_bool,
    sym_hex_literal,
    sym_int,
    sym_str,
)
from kconfig_gen import run as run_generator
from pin_constraints import PinConstraints

# Twin of Icm42688p::kNominalOdrReferenceHz. A property of the part, not of the
# board: the datasheet ODRs are quoted against the part's own 32 kHz oscillator,
# so driving CLKIN instead scales every one of them by CLKIN/32000.
IMU_NOMINAL_ODR_REFERENCE_HZ = 32000

# The serial-number string is the 96-bit device UID rendered as hex, so its
# length is a property of the silicon rather than of any configured string.
USB_CDC_SERIAL_CHARS = 24
# A string descriptor is bLength + bDescriptorType + two bytes per character,
# and bLength is one byte, so no descriptor can exceed 255.
USB_STRING_DESCRIPTOR_HEADER_BYTES = 2
USB_STRING_DESCRIPTOR_MAX_BYTES = 255

# Joined here rather than in the firmware so the buffer below is sized from
# the longest final string: BuildStringDescriptor truncates silently.
USB_PRODUCT_SUFFIXES = {
    "product": " Virtual COM Port",
    "product_msc": " SD Card",
}

# Every serial link on this board runs 8N1, so a byte costs ten bit times.
UART_BITS_PER_BYTE_8N1 = 10
# Twin of the UART_STOP_BITS_1 the ESP32 driver opens the FcLink port with.
# Not a knob: that side takes no configuration for it.
FCLINK_UART_STOP_BITS = 1
# UBX framing is sync(2) + class(1) + id(1) + length(2) + checksum(2). Payload
# sizes come from the M10 interface description; the driver only writes msgout
# keys, so nothing else in the tree knows what the link carries.
UBX_FRAMING_BYTES = 8
UBX_NAV_PAYLOAD_BYTES = {
    "STM32_GPS_M10_MSG_NAV_PVT": 92,
    "STM32_GPS_M10_MSG_NAV_DOP": 18,
    "STM32_GPS_M10_MSG_NAV_COV": 64,
    "STM32_GPS_M10_MSG_NAV_EOE": 4,
}

# SPI_CR1.BR selects these dividers of the peripheral's own APB clock. A knob
# naming the divider rather than the rate means the same setting is a different
# SCK on SPI1 (APB2) and SPI2 (APB1), and moves silently when the tree does.
SPI_PRESCALER_DIVISORS = (2, 4, 8, 16, 32, 64, 128, 256)
# Twin of Icm42688p::kMaxSckHz; also the ceiling on STM32_IMU_SPI_MAX_SCK_HZ.
ICM42688P_MAX_SCK_HZ = 24_000_000

# ICM-42688P anti-alias filter, datasheet section 5.3. The three register
# fields are a tabulated triple, not a formula -- DELT_SQR is 170 where DELT is
# 13 and 440 where DELT is 21, neither of which is DELT squared -- so the row
# is written whole rather than letting the three be configured apart. The
# 585 Hz row is cited from PX4's InvenSense_ICM42688P_registers.hpp, the rest
# from Betaflight's aafLUT42688.
AAF_TRIPLES = {
    "258": (6, 36, 10),
    "536": (12, 144, 8),
    "585": (13, 170, 8),
    "997": (21, 440, 6),
    "1962": (37, 1376, 4),
}
GYRO_AAF_CHOICES = {f"STM32_IMU_GYRO_AAF_{hz}HZ": hz for hz in AAF_TRIPLES}
ACCEL_AAF_CHOICES = {f"STM32_IMU_ACCEL_AAF_{hz}HZ": hz for hz in AAF_TRIPLES}

# ADC_CCR.ADCPRE selects these dividers of PCLK2, and RM0090 Table 67 caps
# ADCCLK at 36 MHz. The field value is the index of its divider here, so an
# APB2 above 8 * 36 MHz has no legal setting rather than a slower one.
ADC_PRESCALER_DIVISORS = (2, 4, 6, 8)
ADC_MAX_CLOCK_HZ = 36_000_000

M10_BAUD_RATE_CHOICES = {
    "STM32_GPS_M10_BAUD_9600": "M10::BaudRate::k9600",
    "STM32_GPS_M10_BAUD_19200": "M10::BaudRate::k19200",
    "STM32_GPS_M10_BAUD_38400": "M10::BaudRate::k38400",
    "STM32_GPS_M10_BAUD_57600": "M10::BaudRate::k57600",
    "STM32_GPS_M10_BAUD_115200": "M10::BaudRate::k115200",
    "STM32_GPS_M10_BAUD_230400": "M10::BaudRate::k230400",
    "STM32_GPS_M10_BAUD_460800": "M10::BaudRate::k460800",
    "STM32_GPS_M10_BAUD_921600": "M10::BaudRate::k921600",
}

M10_UART_BAUD_RATE_CHOICES = {
    "STM32_GPS_M10_BAUD_9600": "9600",
    "STM32_GPS_M10_BAUD_19200": "19200",
    "STM32_GPS_M10_BAUD_38400": "38400",
    "STM32_GPS_M10_BAUD_57600": "57600",
    "STM32_GPS_M10_BAUD_115200": "115200",
    "STM32_GPS_M10_BAUD_230400": "230400",
    "STM32_GPS_M10_BAUD_460800": "460800",
    "STM32_GPS_M10_BAUD_921600": "921600",
}

# ---- Shared UART config value tables -------------------------------------
# The framing/flow values are identical across every UART (FcLink, M10, RC
# receiver). Only the Kconfig prefix and the baud-rate set vary. Define the
# value mappings once and stamp them out per-UART via `_prefixed`.

_UART_WORD_LENGTH_VALUES = {
    "8BITS": "UartWordLength::k8Bits",
    "9BITS": "UartWordLength::k9Bits",
}
_UART_STOP_BITS_VALUES = {
    "1": "UartStopBits::k1",
    "2": "UartStopBits::k2",
}
_UART_PARITY_VALUES = {
    "NONE": "UartParity::kNone",
    "EVEN": "UartParity::kEven",
    "ODD": "UartParity::kOdd",
}
_UART_MODE_VALUES = {
    "RX": "UartMode::kRx",
    "TX": "UartMode::kTx",
    "TX_RX": "UartMode::kTxRx",
}
_UART_HW_FLOW_VALUES = {
    "NONE": "UartHwFlowControl::kNone",
    "RTS": "UartHwFlowControl::kRts",
    "CTS": "UartHwFlowControl::kCts",
    "RTS_CTS": "UartHwFlowControl::kRtsCts",
}
_UART_OVERSAMPLING_VALUES = {
    "16": "UartOverSampling::k16",
    "8": "UartOverSampling::k8",
}


def _prefixed(prefix: str, suffix_map: dict[str, str]) -> dict[str, str]:
    """Stamp out a Kconfig-prefix mapping from a shared value table."""
    return {f"{prefix}_{k}": v for k, v in suffix_map.items()}


RC_RECEIVER_UART_BAUD_RATE_CHOICES = {
    f"STM32_RC_RECEIVER_UART_BAUD_{rate}": str(rate)
    for rate in (115200, 400000, 416666, 420000, 460800, 921600)
}
RC_RECEIVER_UART_WORD_LENGTH_CHOICES = _prefixed(
    "STM32_RC_RECEIVER_UART_WORD_LENGTH", _UART_WORD_LENGTH_VALUES
)
RC_RECEIVER_UART_STOP_BITS_CHOICES = _prefixed(
    "STM32_RC_RECEIVER_UART_STOP_BITS", _UART_STOP_BITS_VALUES
)
RC_RECEIVER_UART_PARITY_CHOICES = _prefixed(
    "STM32_RC_RECEIVER_UART_PARITY", _UART_PARITY_VALUES
)
RC_RECEIVER_UART_MODE_CHOICES = _prefixed(
    "STM32_RC_RECEIVER_UART_MODE", _UART_MODE_VALUES
)
RC_RECEIVER_UART_HW_FLOW_CONTROL_CHOICES = _prefixed(
    "STM32_RC_RECEIVER_UART_HW_FLOW_CONTROL", _UART_HW_FLOW_VALUES
)
RC_RECEIVER_UART_OVERSAMPLING_CHOICES = _prefixed(
    "STM32_RC_RECEIVER_UART_OVERSAMPLING", _UART_OVERSAMPLING_VALUES
)

FCLINK_UART_OVERSAMPLING_CHOICES = _prefixed(
    "STM32_FCLINK_UART_OVERSAMPLING", _UART_OVERSAMPLING_VALUES
)

M10_DYNAMIC_MODEL_CHOICES = {
    "STM32_GPS_M10_DYN_MODEL_PORTABLE": "M10::DynamicModel::kPortable",
    "STM32_GPS_M10_DYN_MODEL_STATIONARY": "M10::DynamicModel::kStationary",
    "STM32_GPS_M10_DYN_MODEL_PEDESTRIAN": "M10::DynamicModel::kPedestrian",
    "STM32_GPS_M10_DYN_MODEL_AUTOMOTIVE": "M10::DynamicModel::kAutomotive",
    "STM32_GPS_M10_DYN_MODEL_SEA": "M10::DynamicModel::kSea",
    "STM32_GPS_M10_DYN_MODEL_AIRBORNE_1G": "M10::DynamicModel::kAirborne1g",
    "STM32_GPS_M10_DYN_MODEL_AIRBORNE_2G": "M10::DynamicModel::kAirborne2g",
    "STM32_GPS_M10_DYN_MODEL_AIRBORNE_4G": "M10::DynamicModel::kAirborne4g",
    "STM32_GPS_M10_DYN_MODEL_WRIST": "M10::DynamicModel::kWrist",
    "STM32_GPS_M10_DYN_MODEL_BIKE": "M10::DynamicModel::kBike",
}

M10_TIMEGRID_CHOICES = {
    "STM32_GPS_M10_TP1_TIMEGRID_UTC": "M10::TimeGrid::kUtc",
    "STM32_GPS_M10_TP1_TIMEGRID_GPS": "M10::TimeGrid::kGps",
    "STM32_GPS_M10_TP1_TIMEGRID_GLONASS": "M10::TimeGrid::kGlonass",
    "STM32_GPS_M10_TP1_TIMEGRID_BEIDOU": "M10::TimeGrid::kBeiDou",
    "STM32_GPS_M10_TP1_TIMEGRID_GALILEO": "M10::TimeGrid::kGalileo",
}

M10_UART_WORD_LENGTH_CHOICES = _prefixed(
    "STM32_GPS_M10_UART_WORD_LENGTH", _UART_WORD_LENGTH_VALUES
)
M10_UART_STOP_BITS_CHOICES = _prefixed(
    "STM32_GPS_M10_UART_STOP_BITS", _UART_STOP_BITS_VALUES
)
M10_UART_PARITY_CHOICES = _prefixed(
    "STM32_GPS_M10_UART_PARITY", _UART_PARITY_VALUES
)
M10_UART_MODE_CHOICES = _prefixed("STM32_GPS_M10_UART_MODE", _UART_MODE_VALUES)
M10_UART_HW_FLOW_CONTROL_CHOICES = _prefixed(
    "STM32_GPS_M10_UART_HW_FLOW_CONTROL", _UART_HW_FLOW_VALUES
)
M10_UART_OVERSAMPLING_CHOICES = _prefixed(
    "STM32_GPS_M10_UART_OVERSAMPLING", _UART_OVERSAMPLING_VALUES
)

# The M10 GPS protocol layer's stop-bits / parity choices reuse the same
# Kconfig symbols as the M10 UART driver but render to M10-namespaced enum
# values (the protocol module has its own enum classes).
M10_CFG_UART_STOP_BITS_CHOICES = _prefixed(
    "STM32_GPS_M10_UART_STOP_BITS",
    {"1": "M10::UartStopBits::k1", "2": "M10::UartStopBits::k2"},
)
M10_CFG_UART_PARITY_CHOICES = _prefixed(
    "STM32_GPS_M10_UART_PARITY",
    {
        "NONE": "M10::UartParity::kNone",
        "EVEN": "M10::UartParity::kEven",
        "ODD": "M10::UartParity::kOdd",
    },
)

ODR_CHOICES = {
    "STM32_IMU_GYRO_ODR_32KHZ": "Icm42688pReg::Odr::k32kHz",
    "STM32_IMU_GYRO_ODR_16KHZ": "Icm42688pReg::Odr::k16kHz",
    "STM32_IMU_GYRO_ODR_8KHZ": "Icm42688pReg::Odr::k8kHz",
    "STM32_IMU_GYRO_ODR_4KHZ": "Icm42688pReg::Odr::k4kHz",
    "STM32_IMU_GYRO_ODR_2KHZ": "Icm42688pReg::Odr::k2kHz",
    "STM32_IMU_GYRO_ODR_1KHZ": "Icm42688pReg::Odr::k1kHz",
    "STM32_IMU_GYRO_ODR_500HZ": "Icm42688pReg::Odr::k500Hz",
    "STM32_IMU_GYRO_ODR_200HZ": "Icm42688pReg::Odr::k200Hz",
    "STM32_IMU_GYRO_ODR_100HZ": "Icm42688pReg::Odr::k100Hz",
    "STM32_IMU_GYRO_ODR_50HZ": "Icm42688pReg::Odr::k50Hz",
    "STM32_IMU_GYRO_ODR_25HZ": "Icm42688pReg::Odr::k25Hz",
    "STM32_IMU_GYRO_ODR_12_5HZ": "Icm42688pReg::Odr::k12_5Hz",
}

# The same choice in Hz, so the FIFO watermark can be derived from it.
GYRO_ODR_HZ = {
    "STM32_IMU_GYRO_ODR_32KHZ": "32000",
    "STM32_IMU_GYRO_ODR_16KHZ": "16000",
    "STM32_IMU_GYRO_ODR_8KHZ": "8000",
    "STM32_IMU_GYRO_ODR_4KHZ": "4000",
    "STM32_IMU_GYRO_ODR_2KHZ": "2000",
    "STM32_IMU_GYRO_ODR_1KHZ": "1000",
    "STM32_IMU_GYRO_ODR_500HZ": "500",
    "STM32_IMU_GYRO_ODR_200HZ": "200",
    "STM32_IMU_GYRO_ODR_100HZ": "100",
    "STM32_IMU_GYRO_ODR_50HZ": "50",
    "STM32_IMU_GYRO_ODR_25HZ": "25",
    "STM32_IMU_GYRO_ODR_12_5HZ": "12.5",
}
GYRO_FS_CHOICES = {
    "STM32_IMU_GYRO_FS_2000DPS": "Icm42688pReg::GyroFs::k2000dps",
    "STM32_IMU_GYRO_FS_1000DPS": "Icm42688pReg::GyroFs::k1000dps",
    "STM32_IMU_GYRO_FS_500DPS": "Icm42688pReg::GyroFs::k500dps",
    "STM32_IMU_GYRO_FS_250DPS": "Icm42688pReg::GyroFs::k250dps",
    "STM32_IMU_GYRO_FS_125DPS": "Icm42688pReg::GyroFs::k125dps",
    "STM32_IMU_GYRO_FS_62_5DPS": "Icm42688pReg::GyroFs::k62_5dps",
    "STM32_IMU_GYRO_FS_31_25DPS": "Icm42688pReg::GyroFs::k31_25dps",
    "STM32_IMU_GYRO_FS_15_625DPS": "Icm42688pReg::GyroFs::k15_625dps",
}

ACCEL_FS_CHOICES = {
    "STM32_IMU_ACCEL_FS_16G": "Icm42688pReg::AccelFs::k16g",
    "STM32_IMU_ACCEL_FS_8G": "Icm42688pReg::AccelFs::k8g",
    "STM32_IMU_ACCEL_FS_4G": "Icm42688pReg::AccelFs::k4g",
    "STM32_IMU_ACCEL_FS_2G": "Icm42688pReg::AccelFs::k2g",
}

DSHOT_TIM1_MODE_CHOICES = {
    "STM32_DSHOT_TIM1_MODE_150": "DShotMode::kDshot150",
    "STM32_DSHOT_TIM1_MODE_300": "DShotMode::kDshot300",
    "STM32_DSHOT_TIM1_MODE_600": "DShotMode::kDshot600",
}

# TIM1 has four capture/compare channels; DShot drives one motor per
# channel from a single burst-DMA transfer.
DSHOT_CHANNEL_COUNT = 4

# ---- System clock choice maps --------------------------------------------
# The Rcc enum classes are hand-written in stm32/Drivers/rcc.hpp
# (matching the SPI / UART pattern: enum values are HAL register bits, so the
# C++ side can `static_cast` straight into the HAL init structs). These dicts
# only carry the Kconfig-symbol -> C++-enum-value mapping the generator needs
# to render `kSystemDefault` in stm32_config.hpp.


def _rcc_choices(
    enum_cpp_name: str, kconfig_to_value: dict[str, str]
) -> dict[str, str]:
    """Build a Kconfig sym -> `Rcc::<Enum>::<value>` dict."""
    return {
        sym: f"Rcc::{enum_cpp_name}::{cpp_name}"
        for sym, cpp_name in kconfig_to_value.items()
    }


RCC_OSC_CHOICES = _rcc_choices(
    "Oscillator",
    {"STM32_RCC_OSC_HSI": "kHsi", "STM32_RCC_OSC_HSE": "kHse"},
)
RCC_PLL_P_CHOICES = _rcc_choices(
    "PllP",
    {f"STM32_RCC_PLL_P_DIV{n}": f"kDiv{n}" for n in (2, 4, 6, 8)},
)
RCC_AHB_DIV_CHOICES = _rcc_choices(
    "AhbDiv",
    {
        f"STM32_RCC_AHB_DIV{n}": f"kDiv{n}"
        for n in (1, 2, 4, 8, 16, 64, 128, 256, 512)
    },
)
RCC_APB_DIV_CHOICES = _rcc_choices(
    "ApbDiv",
    {f"STM32_RCC_APB1_DIV{n}": f"kDiv{n}" for n in (1, 2, 4, 8, 16)},
)
RCC_APB2_DIV_CHOICES = _rcc_choices(
    "ApbDiv",
    {f"STM32_RCC_APB2_DIV{n}": f"kDiv{n}" for n in (1, 2, 4, 8, 16)},
)
RCC_VOLTAGE_SCALE_CHOICES = _rcc_choices(
    "VoltageScale",
    {f"STM32_RCC_VOLTAGE_SCALE{n}": f"kScale{n}" for n in (1, 2)},
)


@dataclass(frozen=True)
class _SignalPin:
    """Peripheral pin constrained by the silicon AF table.

    GPIO programming for AF pins is fixed by peripheral function:
      mode = AF_PP, speed = VERY_HIGH.
    `pull` defaults to NOPULL but can be overridden for protocol-specific
    idle states (e.g. ESC telemetry RX is a single-wire idle-high line).
    """

    board_const: str
    signal: str
    choice_options: dict[str, str]
    pull: str = "nopull"


@dataclass(frozen=True)
class _GpioPin:
    """Plain GPIO pin (no alternate function).

    Direction-specific defaults:
      output → mode = OUTPUT_PP, pull = NOPULL, speed configurable
      input  → mode = INPUT, pull configurable (or derived from active_low),
               speed = LOW
    For outputs, `active_low` controls the initial pin level driven by
    GPIO::Init() and may either be a constant or pinned to a Kconfig bool
    via `active_low_sym` (kept in sync with the consuming driver's config).
    For inputs, `active_low_sym` (when set) auto-selects PULLUP / PULLDOWN
    to match the switch convention.
    """

    board_const: str
    port_options: dict[str, str]
    pin_int_symbol: str
    direction: str = "output"  # "output" | "input"
    speed: str = "low"  # output speed; ignored for inputs
    pull: str | None = None  # input pull; None → derive from active_low
    active_low: bool = False  # constant (used when active_low_sym is None)
    active_low_sym: str | None = None  # Kconfig bool symbol pinning active_low


@dataclass(frozen=True)
class _ExtiPin:
    """GPIO input wired to an EXTI line.

    GPIO programming is fixed: mode = IT_<edge>, pull = NOPULL,
    speed = VERY_HIGH. The EXTI IRQn is silicon-derived from the pin
    number (EXTI0..4 individual; 5-9 share EXTI9_5; 10-15 share
    EXTI15_10).
    """

    board_const: str
    port_options: dict[str, str]
    pin_int_symbol: str
    edge: str = "rising"  # "rising" | "falling"


@dataclass(frozen=True)
class _AnalogPin:
    """ADC1 analog input.

    GPIO programming is fixed: mode = ANALOG, pull = NOPULL, speed = LOW.
    The (port, pin) → ADC channel mapping is silicon-fixed on the F407V
    package — only PA0-PA7, PB0-PB1, PC0-PC5 are valid. The generator
    validates the pick and emits a matching `kFooAdcChannel` constant.
    """

    board_const: str
    adc_const: str
    port_options: dict[str, str]
    pin_int_symbol: str
    # A bool symbol that has to be on for the pin to be claimed at all.
    enable_symbol: str | None = None


# F407V ADC1 input-channel mapping. ADC2 shares the same pinout; ADC3 has a
# different (smaller) set we don't use.
_ADC1_CHANNEL_MAP: dict[str, int] = {
    **{f"PA{n}": n for n in range(8)},  # IN0..IN7
    "PB0": 8,
    "PB1": 9,  # IN8..IN9
    **{f"PC{n}": 10 + n for n in range(6)},  # IN10..IN15
}

_GPIO_PULL_MAP = {
    "nopull": "GPIO_NOPULL",
    "pullup": "GPIO_PULLUP",
    "pulldown": "GPIO_PULLDOWN",
}

_GPIO_SPEED_MAP = {
    "low": "GPIO_SPEED_FREQ_LOW",
    "medium": "GPIO_SPEED_FREQ_MEDIUM",
    "high": "GPIO_SPEED_FREQ_HIGH",
    "very_high": "GPIO_SPEED_FREQ_VERY_HIGH",
}


def _exti_irqn(pin_num: int) -> str:
    """Map an EXTI line to its NVIC IRQn on STM32F4."""
    if pin_num <= 4:
        return f"EXTI{pin_num}_IRQn"
    if pin_num <= 9:
        return "EXTI9_5_IRQn"
    return "EXTI15_10_IRQn"


# Registry of pin-map peripherals. Adding a new entry here + Kconfig choice/
# int symbol(s) + (optionally) a kGpioDefault wiring is the recipe for a new
# tunable pin.
PINMAP_ENTRIES: tuple = (
    # ---- User I/O ------------------------------------------------------
    # Plain GPIOs (no alternate function). Any pin on a package-available
    # port is valid.
    _GpioPin(
        board_const="kUserLed",
        port_options={
            "STM32_USER_LED_PORT_A": "A",
            "STM32_USER_LED_PORT_B": "B",
            "STM32_USER_LED_PORT_C": "C",
            "STM32_USER_LED_PORT_D": "D",
            "STM32_USER_LED_PORT_E": "E",
        },
        pin_int_symbol="STM32_USER_LED_PIN",
        direction="output",
        speed="low",
        active_low_sym="STM32_LED_ACTIVE_LOW",
    ),
    _GpioPin(
        board_const="kUserBtn",
        port_options={
            "STM32_USER_BTN_PORT_A": "A",
            "STM32_USER_BTN_PORT_B": "B",
            "STM32_USER_BTN_PORT_C": "C",
            "STM32_USER_BTN_PORT_D": "D",
            "STM32_USER_BTN_PORT_E": "E",
        },
        pin_int_symbol="STM32_USER_BTN_PIN",
        direction="input",
        active_low_sym="STM32_BUTTON_ACTIVE_LOW",
    ),
    # ---- UART ----------------------------------------------------------
    # USART1 (FcLink to the ESP32 bridge): full-duplex. PA9/PA10 is also the
    # pair the ROM bootloader listens on, so moving this off the default keeps
    # the link but costs over-the-air flashing.
    _SignalPin(
        board_const="kUart1Tx",
        signal="USART1_TX",
        choice_options={
            "STM32_UART1_TX_PIN_PA9": "PA9",
            "STM32_UART1_TX_PIN_PB6": "PB6",
        },
    ),
    _SignalPin(
        board_const="kUart1Rx",
        signal="USART1_RX",
        choice_options={
            "STM32_UART1_RX_PIN_PA10": "PA10",
            "STM32_UART1_RX_PIN_PB7": "PB7",
        },
    ),
    # USART2 (M10 GPS): full-duplex.
    _SignalPin(
        board_const="kUart2Tx",
        signal="USART2_TX",
        choice_options={
            "STM32_UART2_TX_PIN_PA2": "PA2",
            "STM32_UART2_TX_PIN_PD5": "PD5",
        },
    ),
    _SignalPin(
        board_const="kUart2Rx",
        signal="USART2_RX",
        choice_options={
            "STM32_UART2_RX_PIN_PA3": "PA3",
            "STM32_UART2_RX_PIN_PD6": "PD6",
        },
    ),
    # USART3 (ESC telemetry): RX-only one-way line from the ESC. PCB routes
    # to PB11; PD9 is the other silicon-valid choice (PC11 is spent on
    # SDIO_D3). Idles high — the pull-up holds it when the ESC isn't driving.
    _SignalPin(
        board_const="kEscTlmRx",
        signal="USART3_RX",
        choice_options={
            "STM32_ESC_TLM_RX_PIN_PB11": "PB11",
            "STM32_ESC_TLM_RX_PIN_PD9": "PD9",
        },
        pull="pullup",
    ),
    # USART6 (CRSF RC receiver): full-duplex, not RX-only — the link carries
    # telemetry back to the transmitter, so TX matters as much as RX. PC6/PC7
    # is the only USART6 pair bonded out on the F407V package.
    _SignalPin(
        board_const="kUart6Tx",
        signal="USART6_TX",
        choice_options={
            "STM32_UART6_TX_PIN_PC6": "PC6",
        },
    ),
    _SignalPin(
        board_const="kUart6Rx",
        signal="USART6_RX",
        choice_options={
            "STM32_UART6_RX_PIN_PC7": "PC7",
        },
    ),
    # ---- USB -----------------------------------------------------------
    # PA11/PA12 is the only pair on this package; these entries exist so the
    # pins are visible to the collision checker, not to offer a choice.
    _SignalPin(
        board_const="kUsbDm",
        signal="USB_OTG_FS_DM",
        choice_options={
            "STM32_USB_DM_PIN_PA11": "PA11",
        },
    ),
    _SignalPin(
        board_const="kUsbDp",
        signal="USB_OTG_FS_DP",
        choice_options={
            "STM32_USB_DP_PIN_PA12": "PA12",
        },
    ),
    # ---- SD card -------------------------------------------------------
    # SDIO 4-bit group, bonded out exactly once on this package. CMD and
    # D0-D3 idle high per the SD spec, backed by the internal pull-ups.
    _SignalPin(
        board_const="kSdioD0",
        signal="SDIO_D0",
        choice_options={
            "STM32_SDIO_D0_PIN_PC8": "PC8",
        },
        pull="pullup",
    ),
    _SignalPin(
        board_const="kSdioD1",
        signal="SDIO_D1",
        choice_options={
            "STM32_SDIO_D1_PIN_PC9": "PC9",
        },
        pull="pullup",
    ),
    _SignalPin(
        board_const="kSdioD2",
        signal="SDIO_D2",
        choice_options={
            "STM32_SDIO_D2_PIN_PC10": "PC10",
        },
        pull="pullup",
    ),
    _SignalPin(
        board_const="kSdioD3",
        signal="SDIO_D3",
        choice_options={
            "STM32_SDIO_D3_PIN_PC11": "PC11",
        },
        pull="pullup",
    ),
    _SignalPin(
        board_const="kSdioCk",
        signal="SDIO_CK",
        choice_options={
            "STM32_SDIO_CK_PIN_PC12": "PC12",
        },
    ),
    _SignalPin(
        board_const="kSdioCmd",
        signal="SDIO_CMD",
        choice_options={
            "STM32_SDIO_CMD_PIN_PD2": "PD2",
        },
        pull="pullup",
    ),
    # ---- Motors --------------------------------------------------------
    # TIM1 DShot motor outputs. Each channel only routes to PORTA or PORTE
    # on the F407V package; pin_constraints validates the (pin, signal)
    # combo at generation time.
    _SignalPin(
        board_const="kDshotMotor1",
        signal="TIM1_CH1",
        choice_options={
            "STM32_DSHOT_MOTOR1_PA8": "PA8",
            "STM32_DSHOT_MOTOR1_PE9": "PE9",
        },
    ),
    _SignalPin(
        board_const="kDshotMotor2",
        signal="TIM1_CH2",
        choice_options={
            "STM32_DSHOT_MOTOR2_PA9": "PA9",
            "STM32_DSHOT_MOTOR2_PE11": "PE11",
        },
    ),
    _SignalPin(
        board_const="kDshotMotor3",
        signal="TIM1_CH3",
        choice_options={
            "STM32_DSHOT_MOTOR3_PA10": "PA10",
            "STM32_DSHOT_MOTOR3_PE13": "PE13",
        },
    ),
    _SignalPin(
        board_const="kDshotMotor4",
        signal="TIM1_CH4",
        choice_options={
            "STM32_DSHOT_MOTOR4_PE14": "PE14",
        },
    ),
    # ---- SPI -----------------------------------------------------------
    # SPI1 (EEPROM / external flash on this board). All bus pairs are AF5;
    # CS is plain GPIO toggled by the driver.
    _SignalPin(
        board_const="kSpi1Sck",
        signal="SPI1_SCK",
        choice_options={
            "STM32_SPI1_SCK_PIN_PA5": "PA5",
            "STM32_SPI1_SCK_PIN_PB3": "PB3",
        },
    ),
    _SignalPin(
        board_const="kSpi1Miso",
        signal="SPI1_MISO",
        choice_options={
            "STM32_SPI1_MISO_PIN_PA6": "PA6",
            "STM32_SPI1_MISO_PIN_PB4": "PB4",
        },
    ),
    _SignalPin(
        board_const="kSpi1Mosi",
        signal="SPI1_MOSI",
        choice_options={
            "STM32_SPI1_MOSI_PIN_PA7": "PA7",
            "STM32_SPI1_MOSI_PIN_PB5": "PB5",
        },
    ),
    # SPI1 chip-select. PCB currently routes to PA15. CS is universally
    # active-low (every SPI peripheral expects this) — not exposed as a
    # Kconfig knob.
    _GpioPin(
        board_const="kSpi1Cs",
        port_options={
            "STM32_SPI1_CS_PORT_A": "A",
            "STM32_SPI1_CS_PORT_B": "B",
            "STM32_SPI1_CS_PORT_C": "C",
            "STM32_SPI1_CS_PORT_D": "D",
            "STM32_SPI1_CS_PORT_E": "E",
        },
        pin_int_symbol="STM32_SPI1_CS_PIN",
        direction="output",
        speed="very_high",
        active_low=True,
    ),
    # SPI2 (IMU on this board). All bus pairs are AF5; CS is plain GPIO.
    _SignalPin(
        board_const="kSpi2Sck",
        signal="SPI2_SCK",
        choice_options={
            "STM32_SPI2_SCK_PIN_PB10": "PB10",
            "STM32_SPI2_SCK_PIN_PB13": "PB13",
        },
    ),
    _SignalPin(
        board_const="kSpi2Miso",
        signal="SPI2_MISO",
        choice_options={
            "STM32_SPI2_MISO_PIN_PB14": "PB14",
            "STM32_SPI2_MISO_PIN_PC2": "PC2",
        },
    ),
    _SignalPin(
        board_const="kSpi2Mosi",
        signal="SPI2_MOSI",
        choice_options={
            "STM32_SPI2_MOSI_PIN_PB15": "PB15",
            "STM32_SPI2_MOSI_PIN_PC3": "PC3",
        },
    ),
    # SPI2 chip-select. PCB currently routes to PA4. CS is universally
    # active-low.
    _GpioPin(
        board_const="kSpi2Cs",
        port_options={
            "STM32_SPI2_CS_PORT_A": "A",
            "STM32_SPI2_CS_PORT_B": "B",
            "STM32_SPI2_CS_PORT_C": "C",
            "STM32_SPI2_CS_PORT_D": "D",
            "STM32_SPI2_CS_PORT_E": "E",
        },
        pin_int_symbol="STM32_SPI2_CS_PIN",
        direction="output",
        speed="very_high",
        active_low=True,
    ),
    # IMU data-ready interrupt (ICM42688P INT pin → STM32 EXTI input).
    # PCB routes to PB10 → EXTI15_10_IRQn.
    _ExtiPin(
        board_const="kImuInt",
        port_options={
            "STM32_IMU_INT_PORT_A": "A",
            "STM32_IMU_INT_PORT_B": "B",
            "STM32_IMU_INT_PORT_C": "C",
            "STM32_IMU_INT_PORT_D": "D",
            "STM32_IMU_INT_PORT_E": "E",
        },
        pin_int_symbol="STM32_IMU_INT_PIN",
    ),
    # ---- Battery sense (ADC1) ------------------------------------------
    # Voltage and current sense are wired to ADC1 inputs. The pin -> ADC
    # channel mapping is silicon-fixed; the generator emits a matching
    # `kEsc{Vba,Cur}AdcChannel` constant so battery.cpp picks up the right
    # channel automatically.
    _AnalogPin(
        board_const="kEscVba",
        adc_const="kEscVbaAdcChannel",
        port_options={
            "STM32_BATTERY_VOLTAGE_PORT_A": "A",
            "STM32_BATTERY_VOLTAGE_PORT_B": "B",
            "STM32_BATTERY_VOLTAGE_PORT_C": "C",
        },
        pin_int_symbol="STM32_BATTERY_VOLTAGE_PIN",
    ),
    _AnalogPin(
        board_const="kEscCur",
        adc_const="kEscCurAdcChannel",
        port_options={
            "STM32_BATTERY_CURRENT_PORT_A": "A",
            "STM32_BATTERY_CURRENT_PORT_B": "B",
            "STM32_BATTERY_CURRENT_PORT_C": "C",
        },
        pin_int_symbol="STM32_BATTERY_CURRENT_PIN",
        enable_symbol="STM32_BATTERY_CURRENT_MONITORING",
    ),
)


def _m10_uart_data_bits_value(kconf: kconfiglib.Kconfig) -> str:
    word_length_9 = sym_bool(kconf, "STM32_GPS_M10_UART_WORD_LENGTH_9BITS")
    parity_none = sym_bool(kconf, "STM32_GPS_M10_UART_PARITY_NONE")

    if word_length_9 and parity_none:
        raise ValueError(
            "CONFIG_STM32_GPS_M10_UART_WORD_LENGTH_9BITS requires parity "
            "on the "
            "M10 link; choose 8 bits or enable parity"
        )

    if word_length_9:
        return "M10::UartDataBits::k8"
    if parity_none:
        return "M10::UartDataBits::k8"
    return "M10::UartDataBits::k7"


def _aaf_fields(cutoff_hz: str) -> dict[str, int]:
    """The register row for an AAF corner, as three named fields."""
    delt, delt_sqr, bitshift = AAF_TRIPLES[cutoff_hz]
    return {"delt": delt, "delt_sqr": delt_sqr, "bitshift": bitshift}


def _whole_hz(value: str, who: str) -> int:
    """An ODR the arithmetic downstream can actually carry.

    The watermark record count and the control loop rate are both whole Hz, so a
    fractional ODR has no integer that is not a lie about it. Parsing to int
    through float truncated the part's 12.5 Hz setting to 12 and derived every
    rate from there, 4% adrift with nothing to say so.
    """
    hz = float(value)
    if not hz.is_integer():
        raise ValueError(
            f"the {who} ODR of {value} Hz is not a whole number of Hz; the "
            "FIFO watermark and the control loop rate are both derived from it "
            "as integers, so pick 25 Hz or faster"
        )
    return int(hz)


def _imu_record_rate_hz(kconf: kconfiglib.Kconfig) -> int:
    """Records the FIFO actually fills per second.

    Mirrors Icm42688p::EffectiveOdrHz. The nominal ODRs assume the part's own
    32 kHz oscillator; driven from CLKIN they scale by CLKIN/32000, so a 32768
    Hz crystal makes a nominal 8 kHz part run at 8192 Hz. The interrupt that
    drives the control loop follows the real rate, not the label, so anything
    deriving a loop period from the nominal value is off by that ratio.
    """
    gyro_odr_hz = _whole_hz(choice_value(kconf, GYRO_ODR_HZ), "gyro")

    if not sym_bool(kconf, "STM32_IMU_EXTERNAL_CLOCK_ENABLED"):
        return gyro_odr_hz

    clkin_hz = sym_int(kconf, "STM32_IMU_EXTERNAL_CLOCK_FREQ_HZ")
    scaled = (gyro_odr_hz * clkin_hz + IMU_NOMINAL_ODR_REFERENCE_HZ // 2) // (
        IMU_NOMINAL_ODR_REFERENCE_HZ
    )
    if scaled * IMU_NOMINAL_ODR_REFERENCE_HZ != gyro_odr_hz * clkin_hz:
        raise ValueError(
            f"a {clkin_hz} Hz CLKIN scales the {gyro_odr_hz} Hz ODR to a "
            "fractional record rate, so no whole watermark gives a fixed loop "
            f"period; ODR x CLKIN ({gyro_odr_hz * clkin_hz}) has to be a whole "
            f"multiple of {IMU_NOMINAL_ODR_REFERENCE_HZ}"
        )
    return scaled


def _imu_watermark_records(kconf: kconfiglib.Kconfig) -> int:
    """Records per FIFO interrupt.

    The watermark interrupt is what runs the attitude and rate controllers
    (icm42688p.cpp pends PendSV, which calls ExpressMain), so the loop rate is
    the record rate divided by this. Deriving it the other way round keeps the
    loop rate fixed when the gyro ODR changes.
    """
    record_rate_hz = _imu_record_rate_hz(kconf)
    loop_hz = sym_int(kconf, "STM32_CONTROL_LOOP_HZ")

    if loop_hz <= 0:
        raise ValueError("CONFIG_STM32_CONTROL_LOOP_HZ must be > 0")
    if record_rate_hz < loop_hz:
        raise ValueError(
            f"a {record_rate_hz} Hz record rate cannot drive a {loop_hz} Hz "
            "control loop"
        )

    records, remainder = divmod(record_rate_hz, loop_hz)
    if remainder:
        achievable = sorted(
            {
                record_rate_hz // n
                for n in range(1, record_rate_hz + 1)
                if record_rate_hz % n == 0
                and record_rate_hz // n <= record_rate_hz
            },
            reverse=True,
        )
        near = [hz for hz in achievable if loop_hz // 2 <= hz <= loop_hz * 2]
        raise ValueError(
            f"CONFIG_STM32_CONTROL_LOOP_HZ ({loop_hz} Hz) does not divide the "
            f"{record_rate_hz} Hz record rate, so every tick would land a "
            "fraction of a record early or late and the controller timestep "
            "would never match the interval it integrates over. "
            f"Nearby exact rates: {', '.join(str(hz) for hz in near) or 'none'}"
        )
    return records


def _resolve_pin(
    kconf: kconfiglib.Kconfig, entry: object
) -> tuple[str, str, int]:
    """Pick the pin for a PINMAP_ENTRIES item — no silicon validation.

    Returns (pin_name, port_letter, pin_num). Used by both `_validate_pinmap`
    (which then runs the silicon checks) and `_pinmap_context` (which trusts
    that validation already passed).
    """
    if isinstance(entry, _SignalPin):
        pin_name = choice_value(kconf, entry.choice_options)
        return pin_name, pin_name[1], int(pin_name[2:])
    if isinstance(entry, (_GpioPin, _ExtiPin, _AnalogPin)):
        port_letter = choice_value(kconf, entry.port_options)
        pin_num = sym_int(kconf, entry.pin_int_symbol)
        return f"P{port_letter}{pin_num}", port_letter, pin_num
    raise TypeError(f"unknown PINMAP_ENTRIES entry type: {type(entry)}")


def _validate_pinmap_entry(
    db: PinConstraints, kconf: kconfiglib.Kconfig, entry: object
) -> None:
    """Run every silicon / Kconfig sanity check the entry needs.

    Mirrors the lookups `_pinmap_context` performs: any combination that
    would force `_pinmap_context` to fail is rejected here first, so a
    successful `_validate_pinmap` guarantees `_pinmap_context` will not
    raise.
    """
    pin_name, _, _ = _resolve_pin(kconf, entry)

    if isinstance(entry, _SignalPin):
        if db.af_for(pin_name, entry.signal) is None:
            valid = [p.pin for p in db.pins_for_signal(entry.signal)]
            raise ValueError(
                f"pinmap {entry.board_const}: ({pin_name}, {entry.signal}) "
                f"is not a valid AF combo per ST data. Valid pins for "
                f"{entry.signal}: {valid}"
            )
        return

    if not db.is_valid_pin(pin_name):
        raise ValueError(
            f"pinmap {entry.board_const}: pin {pin_name} does not exist on "
            f"the STM32F407V package"
        )

    if isinstance(entry, _GpioPin):
        if entry.direction not in ("output", "input"):
            raise ValueError(
                f"pinmap {entry.board_const}: invalid _GpioPin direction "
                f"'{entry.direction}'"
            )
    elif isinstance(entry, _ExtiPin):
        if entry.edge not in ("rising", "falling"):
            raise ValueError(
                f"pinmap {entry.board_const}: invalid _ExtiPin edge "
                f"'{entry.edge}'"
            )
    elif isinstance(entry, _AnalogPin):
        if pin_name not in _ADC1_CHANNEL_MAP:
            valid = sorted(_ADC1_CHANNEL_MAP.keys())
            raise ValueError(
                f"pinmap {entry.board_const}: pin {pin_name} is not an "
                f"ADC1 input on the F407V package. Valid pins: {valid}"
            )


BOARD_TOML = (
    pathlib.Path(__file__).resolve().parent.parent / "config" / "board.toml"
)


def _board_reserved_pins() -> dict[str, str]:
    """Pins spent by something outside the pin map, mapped to the reason."""
    with BOARD_TOML.open("rb") as fp:
        return tomllib.load(fp).get("reserved", {})


def _legal_signal_pins(
    db: PinConstraints, reserved: set[str]
) -> dict[str, set[str]]:
    """Pins each signal may still be offered: ST's list minus what is spent.

    A signal ST gives no alternative holds its pin unconditionally, so that pin
    leaves every other signal's list -- which can strand a second signal on one
    option in turn, so this settles rather than running a single pass.
    """
    available = {
        entry.signal: {p.pin for p in db.pins_for_signal(entry.signal)}
        - reserved
        for entry in PINMAP_ENTRIES
        if isinstance(entry, _SignalPin)
    }

    spent: dict[str, str] = {}
    while True:
        newly = {
            next(iter(pins)): signal
            for signal, pins in available.items()
            if len(pins) == 1 and next(iter(pins)) not in spent
        }
        if not newly:
            return available
        spent.update(newly)
        for signal in available:
            available[signal] -= {
                pin for pin, owner in spent.items() if owner != signal
            }
            if not available[signal]:
                raise ValueError(
                    f"no pin left for {signal}: every pin ST allows on this "
                    "package is either held by a peripheral with no "
                    "alternative "
                    "or reserved in config/board.toml"
                )


def _entry_enabled(kconf: kconfiglib.Kconfig, entry: object) -> bool:
    """Whether an optional pin is claimed on this build.

    A gated-off entry leaves every table: nothing validates it, nothing
    programs its GPIO, and its pin is free for something else to take.
    """
    symbol = getattr(entry, "enable_symbol", None)
    return symbol is None or sym_bool(kconf, symbol)


def _validate_pinmap(kconf: kconfiglib.Kconfig) -> None:
    db = PinConstraints.load_default()
    reserved = _board_reserved_pins()

    # A reservation for a pin that does not exist protects nothing while
    # reading as though it does.
    for pin_name in sorted(reserved):
        if not db.is_valid_pin(pin_name):
            raise ValueError(
                f"config/board.toml reserves {pin_name}, which is not a pin on "
                "the STM32F407V package"
            )

    legal = _legal_signal_pins(db, set(reserved))

    for entry in PINMAP_ENTRIES:
        if not _entry_enabled(kconf, entry):
            continue
        _validate_pinmap_entry(db, kconf, entry)

        pin_name, _, _ = _resolve_pin(kconf, entry)
        if pin_name in reserved:
            raise ValueError(
                f"pinmap {entry.board_const}: {pin_name} is reserved in "
                f"config/board.toml -- {reserved[pin_name]}"
            )

        # The offered list is written twice, in the Kconfig and in the entry
        # above. Neither states why a pin is absent, so an option ST allows can
        # go missing without anything noticing.
        if isinstance(entry, _SignalPin):
            offered = set(entry.choice_options.values())
            expected = legal[entry.signal]
            if offered != expected:
                detail = []
                if expected - offered:
                    detail.append(
                        f"never offered: {sorted(expected - offered)}"
                    )
                if offered - expected:
                    detail.append(
                        f"offered but spent: {sorted(offered - expected)}"
                    )
                raise ValueError(
                    f"pinmap {entry.board_const}: {entry.signal} offers "
                    f"{sorted(offered)} but the legal set is "
                    f"{sorted(expected)} "
                    f"({'; '.join(detail)})"
                )


def _pinmap_context(kconf: kconfiglib.Kconfig) -> list[dict[str, object]]:
    """Resolve every PINMAP_ENTRIES item to its template payload.

    Computes both the BoardPin geometry (port/pin/AF/IRQn, optional ADC
    channel) and the GPIO programming the entry's role requires
    (mode/pull/speed/active_low). The template emits both — the BoardPin
    into `namespace board`, the GPIO programming as an entry in the
    `kGpioDefault` array consumed by `GPIO::Init()`.

    Pure rendering: assumes `_validate_pinmap` already ran. If a silicon
    lookup unexpectedly fails here it indicates `_validate` was bypassed
    or the pin tables drifted, which is treated as an internal error.
    """
    db = PinConstraints.load_default()
    rendered: list[dict[str, object]] = []
    for entry in PINMAP_ENTRIES:
        if not _entry_enabled(kconf, entry):
            continue
        pin_name, port_letter, pin_num = _resolve_pin(kconf, entry)

        af = "0"
        irqn = "NonMaskableInt_IRQn"
        adc_const: str | None = None
        adc_channel: int | None = None
        gpio_mode = "GPIO_MODE_INPUT"
        gpio_pull = "GPIO_NOPULL"
        gpio_speed = "GPIO_SPEED_FREQ_LOW"
        gpio_active_low = "false"

        if isinstance(entry, _SignalPin):
            af = db.af_for(pin_name, entry.signal)
            assert af is not None, "validate-bypass: missing AF for signal pin"
            gpio_mode = "GPIO_MODE_AF_PP"
            gpio_pull = _GPIO_PULL_MAP[entry.pull]
            gpio_speed = "GPIO_SPEED_FREQ_VERY_HIGH"
        elif isinstance(entry, _GpioPin):
            if entry.direction == "output":
                gpio_mode = "GPIO_MODE_OUTPUT_PP"
                gpio_speed = _GPIO_SPEED_MAP[entry.speed]
                active_low_val = (
                    sym_bool(kconf, entry.active_low_sym)
                    if entry.active_low_sym
                    else entry.active_low
                )
                gpio_active_low = "true" if active_low_val else "false"
            else:  # input — validation enforces direction in {"output","input"}
                gpio_mode = "GPIO_MODE_INPUT"
                if entry.pull is not None:
                    gpio_pull = _GPIO_PULL_MAP[entry.pull]
                elif entry.active_low_sym is not None:
                    # Active-low input → switch shorts to GND when pressed
                    # → idle line needs PULLUP. Inverted for active-high.
                    gpio_pull = (
                        "GPIO_PULLUP"
                        if sym_bool(kconf, entry.active_low_sym)
                        else "GPIO_PULLDOWN"
                    )
        elif isinstance(entry, _ExtiPin):
            irqn = _exti_irqn(pin_num)
            gpio_mode = (
                "GPIO_MODE_IT_RISING"
                if entry.edge == "rising"
                else "GPIO_MODE_IT_FALLING"
            )
            gpio_speed = "GPIO_SPEED_FREQ_VERY_HIGH"
        elif isinstance(entry, _AnalogPin):
            adc_channel = _ADC1_CHANNEL_MAP[pin_name]
            adc_const = entry.adc_const
            gpio_mode = "GPIO_MODE_ANALOG"

        rendered.append(
            {
                "name": entry.board_const,
                "port": f"GPIO{port_letter}",
                "pin": f"GPIO_PIN_{pin_num}",
                "af": af,
                "irqn": irqn,
                "adc_const": adc_const,
                "adc_channel": adc_channel,
                "gpio_mode": gpio_mode,
                "gpio_pull": gpio_pull,
                "gpio_speed": gpio_speed,
                "gpio_active_low": gpio_active_low,
            }
        )
    return rendered


def _validate(kconf: kconfiglib.Kconfig) -> None:
    _validate_pinmap(kconf)

    rc_map = _rc_map(kconf)
    if sorted(rc_map.values()) != [1, 2, 3, 4]:
        raise ValueError(
            "CONFIG_STM32_RC_MAP_ROLL/PITCH/YAW/THROTTLE must be a unique "
            "mapping of channels 1..4"
        )

    # Both AHRS gates ramp from full weight down to zero across the span
    # between their two thresholds. An inverted pair reads to the runtime as no
    # gate at all -- weight pinned at 1.0 -- so the accel stays fully trusted
    # through exactly the sustained maneuver its band exists to reject.
    for gate, full_sym, zero_sym in (
        (
            "accel trust",
            "STM32_AHRS_ACCEL_TRUST_FULL_DEV_MILLI",
            "STM32_AHRS_ACCEL_TRUST_ZERO_DEV_MILLI",
        ),
        (
            "gyro quiescence",
            "STM32_AHRS_GYRO_QUIESCENT_FULL_MILLI",
            "STM32_AHRS_GYRO_QUIESCENT_ZERO_MILLI",
        ),
    ):
        if sym_int(kconf, zero_sym) < sym_int(kconf, full_sym):
            raise ValueError(
                f"CONFIG_{zero_sym} must be at least CONFIG_{full_sym}; the "
                f"{gate} gate treats an inverted pair as no gate at all"
            )

    cell_empty_mv = sym_int(kconf, "STM32_BATTERY_CELL_EMPTY_MV")
    cell_full_mv = sym_int(kconf, "STM32_BATTERY_CELL_FULL_MV")
    if cell_empty_mv >= cell_full_mv:
        raise ValueError(
            "CONFIG_STM32_BATTERY_CELL_EMPTY_MV must be lower than "
            "CONFIG_STM32_BATTERY_CELL_FULL_MV"
        )

    # Checked even when TP1 is disabled: the pair is written either way, and
    # the M10 clamps rather than refusing, so a bad pair is silent.
    tp1_period_us = sym_int(kconf, "STM32_GPS_M10_TP1_PERIOD")
    tp1_len_us = sym_int(kconf, "STM32_GPS_M10_TP1_LEN")
    if tp1_len_us >= tp1_period_us:
        raise ValueError(
            f"CONFIG_STM32_GPS_M10_TP1_LEN ({tp1_len_us} us) must be shorter "
            f"than CONFIG_STM32_GPS_M10_TP1_PERIOD ({tp1_period_us} us); a "
            "pulse cannot outlast the period that repeats it"
        )

    _validate_gps_link_budget(kconf)
    _validate_battery_sample_budget(kconf)


def _validate_gps_link_budget(kconf: kconfiglib.Kconfig) -> None:
    """The enabled UBX set at the configured rate against UART2's baud.

    An epoch that takes longer to send than the interval before the next one
    starts leaves the link permanently behind, so the fix that arrives is
    older than the one being measured.
    """
    enabled_bytes = sum(
        payload + UBX_FRAMING_BYTES
        for sym, payload in UBX_NAV_PAYLOAD_BYTES.items()
        if sym_bool(kconf, sym)
    )
    if enabled_bytes == 0:
        return

    rate_meas_ms = sym_int(kconf, "STM32_GPS_M10_RATE_MEAS_MS")
    baud = int(choice_value(kconf, M10_UART_BAUD_RATE_CHOICES))
    epoch_ms = (
        enabled_bytes * UART_BITS_PER_BYTE_8N1 * MILLIS_PER_SECOND
    ) / baud
    if epoch_ms > rate_meas_ms:
        raise ValueError(
            f"the enabled UBX messages are {enabled_bytes} bytes, which take "
            f"{epoch_ms:.1f} ms to send at {baud} baud -- longer than the "
            f"{rate_meas_ms} ms CONFIG_STM32_GPS_M10_RATE_MEAS_MS leaves "
            "before the next epoch starts. Raise CONFIG_STM32_GPS_M10_BAUD, "
            "slow the measurement rate, or turn off a message"
        )


def _validate_battery_sample_budget(kconf: kconfiglib.Kconfig) -> None:
    """The span of one sample against the period it has to fit in.

    Battery::Poll starts one conversion per main tick and collects it on
    the next, so a sample spans one pass per channel per oversample.
    Overrunning the period would stretch the publish interval that
    filter_alpha was solved against, which the smoothing then no longer
    matches.
    """
    oversample = sym_int(kconf, "STM32_BATTERY_ADC_OVERSAMPLE_COUNT")
    tick_hz = sym_int(kconf, "STM32_TIMEBASE_TIM5_TICK_HZ")
    period_us = (
        sym_int(kconf, "STM32_BATTERY_SAMPLE_PERIOD_MS") * MICROS_PER_MILLI
    )
    pass_us = MICROS_PER_SECOND // tick_hz
    channels = (
        2 if sym_bool(kconf, "STM32_BATTERY_CURRENT_MONITORING") else 1
    )
    span_us = oversample * channels * pass_us
    if span_us >= period_us:
        raise ValueError(
            f"one battery sample spans {span_us} us "
            f"(CONFIG_STM32_BATTERY_ADC_OVERSAMPLE_COUNT {oversample} x "
            f"{channels} conversions, one per {pass_us} us "
            f"CONFIG_STM32_TIMEBASE_TIM5_TICK_HZ pass), which does not fit "
            f"the {period_us} us "
            "CONFIG_STM32_BATTERY_SAMPLE_PERIOD_MS gives it"
        )


def _flight_mode_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    tilt_rad = math.radians(
        sym_int(kconf, "STM32_PILOT_STABILIZE_MAX_TILT_DEG")
    )
    return {
        "acro_max_rate_roll_pitch_milli": sym_int(
            kconf, "STM32_PILOT_ACRO_MAX_RATE_ROLL_PITCH_MILLI"
        ),
        "acro_max_rate_yaw_milli": sym_int(
            kconf, "STM32_PILOT_ACRO_MAX_RATE_YAW_MILLI"
        ),
        # Degrees are what the knob states and radians are what the cascade
        # wants, so the conversion happens once here rather than at the use
        # site.
        "stabilize_max_tilt_rad": f"{tilt_rad:.6f}f",
        # The knob is numbered the way a transmitter numbers channels; the
        # array is not.
        "slot": sym_int(kconf, "STM32_FLIGHT_MODE_RC_CHANNEL") - 1,
        "threshold_us": sym_int(kconf, "STM32_FLIGHT_MODE_THRESHOLD_US"),
    }


def _rc_map(kconf: kconfiglib.Kconfig) -> dict[str, int]:
    return {
        "roll": sym_int(kconf, "STM32_RC_MAP_ROLL"),
        "pitch": sym_int(kconf, "STM32_RC_MAP_PITCH"),
        "yaw": sym_int(kconf, "STM32_RC_MAP_YAW"),
        "throttle": sym_int(kconf, "STM32_RC_MAP_THROTTLE"),
    }


def _usb_string_descriptor_bytes(kconf: kconfiglib.Kconfig) -> int:
    """Size the string-descriptor build buffer to the longest string it encodes.

    Sized rather than clamped: BuildStringDescriptor truncates silently, so a
    fixed buffer would cut a long product name in the host's port picker with
    nothing to say it had happened.
    """
    base = sym_str(kconf, "STM32_USB_PRODUCT")
    candidates = [
        sym_str(kconf, "STM32_USB_MANUFACTURER"),
        *(base + suffix for suffix in USB_PRODUCT_SUFFIXES.values()),
    ]
    longest = max(USB_CDC_SERIAL_CHARS, *(len(v) for v in candidates))

    total = USB_STRING_DESCRIPTOR_HEADER_BYTES + (longest * 2)
    if total > USB_STRING_DESCRIPTOR_MAX_BYTES:
        raise SystemExit(
            f"USB string of {longest} characters needs a {total}-byte "
            "descriptor, "
            f"over the {USB_STRING_DESCRIPTOR_MAX_BYTES}-byte limit "
            "imposed by a "
            f"single-byte bLength. Shorten STM32_USB_MANUFACTURER or "
            f"STM32_USB_PRODUCT."
        )
    return total


def _limits_context(
    source: pathlib.Path, kconf: kconfiglib.Kconfig
) -> dict[str, object]:
    return {
        "autogen_warning": autogen_warning(source),
        "max_watermark_records": _imu_watermark_records(kconf),
        "log_raw_imu": sym_bool(kconf, "STM32_LOG_TOPIC_RAW_IMU"),
        "dshot_min_period_ticks": DSHOT_MIN_PERIOD_TICKS,
        "dshot_max_period_ticks": DSHOT_MAX_PERIOD_TICKS,
        "usb_cdc_string_descriptor_bytes": _usb_string_descriptor_bytes(kconf),
        "dshot_channel_count": DSHOT_CHANNEL_COUNT,
    }


# ---- Per-peripheral runtime contexts -------------------------------------
# Each helper resolves Kconfig + chosen-value maps for one peripheral and
# returns the sub-dict used by the matching block in stm32_config.hpp.j2.
# `_runtime_context` just stitches them together.

# RM0090 §6.3.2 PLL limits, plus the F407's SYSCLK ceiling.
PLL_VCO_IN_HZ = (1_000_000, 2_000_000)
PLL_VCO_OUT_HZ = (100_000_000, 432_000_000)
USB_OTG_FS_HZ = 48_000_000
SYSCLK_MAX_HZ = 168_000_000
HSI_HZ = 16_000_000
APB1_MAX_HZ = 42_000_000
MICROS_PER_SECOND = 1_000_000
MICROS_PER_MILLI = 1_000
MILLIS_PER_SECOND = 1_000
APB2_MAX_HZ = 84_000_000
# RM0090 Table 10. One wait state per 30 MHz of HCLK holds for VDD 2.7-3.6 V;
# the step tightens to 24, 22 and 20 MHz on the lower supply bands. VDD is
# modelled nowhere in Kconfig, so this constant is where the board's 3.3 V rail
# is written down -- a shift to a lower rail has to change it.
FLASH_WAIT_STATE_STEP_HZ = 30_000_000
# The regulator scale caps SYSCLK: the F407 needs scale 1 above this.
VOLTAGE_SCALE2_MAX_HZ = 144_000_000


# RM0090 6.2: a timer on an APB bus runs at PCLK when that bus divides by one,
# and at twice PCLK otherwise. TIMPRE is never written, so the rule is
# unconditional. This is why APB2 /1 and /2 both leave TIM1 at 168 MHz and only
# the coarser dividers move it.
def _timer_clock_hz(pclk_hz: int, apb_divider: int) -> int:
    return pclk_hz if apb_divider == 1 else pclk_hz * 2


DSHOT_BIT_RATE_HZ: dict[str, int] = {
    "DShotMode::kDshot150": 150_000,
    "DShotMode::kDshot300": 300_000,
    "DShotMode::kDshot600": 600_000,
}
# The symbols sit at 37.5% and 75% of the bit period and each rounds to a whole
# tick, so the worst case is half a tick, or 50/N percent of the period. Twenty
# holds that to 2.5%. It is a budget, not a cliff: multiples of eight land
# exactly, and the two symbols stay far apart well below this.
DSHOT_MIN_PERIOD_TICKS = 20
# ARR is 16 bits and holds ticks - 1, so this many ticks is the most it can
# express.
DSHOT_MAX_PERIOD_TICKS = 0x10000
# An ESC locks to the frame, not to a nominal rate, but a bit period that does
# not divide evenly leaves every edge off by the rounding error.
DSHOT_MAX_BIT_RATE_ERROR = 0.005


def _dshot_tim1_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    """TIM1's kernel clock, plus a check that the chosen rate lands on it.

    The bit period is the timer clock divided by the DShot rate, so it moves
    with the APB2 divider while the mode says nothing about it. Emitting the
    clock and deriving the period in the driver keeps the knob on the intent;
    checking it here is what stops a coarser APB2 from quietly halving the wire
    rate while the build stays green and the ESCs stay silent.
    """
    mode = choice_value(kconf, DSHOT_TIM1_MODE_CHOICES)
    hclk = _solve_rcc_clock(kconf)
    apb2_div = _apb_divider(kconf, 2)
    timer_hz = _timer_clock_hz(hclk // apb2_div, apb2_div)

    bit_rate = DSHOT_BIT_RATE_HZ[mode]
    # Round half up in integers. round() is half-to-even, so a period landing
    # exactly on .5 would go to the nearer even tick count rather than the
    # longer one, and float division can only add error to an exact ratio.
    ticks = (timer_hz + bit_rate // 2) // bit_rate
    if ticks < DSHOT_MIN_PERIOD_TICKS or ticks > DSHOT_MAX_PERIOD_TICKS:
        raise SystemExit(
            f"{mode} needs a {ticks}-tick bit period at TIM1's "
            f"{timer_hz / 1e6:g} MHz, outside the usable "
            f"{DSHOT_MIN_PERIOD_TICKS}..{DSHOT_MAX_PERIOD_TICKS} range. "
            "Choose a slower DShot rate or a finer APB2 divider."
        )
    error = abs(timer_hz / ticks - bit_rate) / bit_rate
    if error > DSHOT_MAX_BIT_RATE_ERROR:
        raise SystemExit(
            f"{mode} would run at {timer_hz / ticks / 1e3:.1f} kbit/s from "
            f"TIM1's {timer_hz / 1e6:g} MHz ({error * 100:.2f}% off); the "
            f"limit is {DSHOT_MAX_BIT_RATE_ERROR * 100:g}%. Pick an APB2 "
            "divider that divides evenly into the DShot rate."
        )
    return {"mode": mode, "timer_clock_hz": timer_hz}


def _flash_wait_states(hclk_hz: int) -> int:
    """Wait states HCLK needs, per RM0090 Table 10.

    Not a tunable: there is one correct minimum for a given HCLK and supply,
    and exceeding it only slows the core. Deriving it is what keeps it attached
    to the clock -- a hand-set value stays behind when a PLL divider moves, and
    too few wait states faults on the first fetch after the switch, which
    presents as a dead board rather than as a misconfiguration.
    """
    return -(-hclk_hz // FLASH_WAIT_STATE_STEP_HZ) - 1


def _apb_divider(kconf: kconfiglib.Kconfig, bus: int) -> int:
    choices = RCC_APB_DIV_CHOICES if bus == 1 else RCC_APB2_DIV_CHOICES
    return int(choice_value(kconf, choices).rsplit("kDiv", 1)[1])


def _solve_rcc_clock(kconf: kconfiglib.Kconfig) -> int:
    """Resolve HCLK from the clock tree, checking it against RM0090.

    Kconfig ranges police each field alone; nothing checks them against
    each other or against the crystal they divide.
    """
    # choice_value returns the qualified enumerator (Rcc::PllP::kDiv2).
    hse = sym_int(kconf, "STM32_RCC_HSE_HZ")
    use_hse = choice_value(kconf, RCC_OSC_CHOICES).endswith("kHse")
    src_hz = hse if use_hse else HSI_HZ
    src_name = (
        f"HSE {hse / 1e6:g} MHz" if use_hse else f"HSI {HSI_HZ / 1e6:g} MHz"
    )

    pllm = sym_int(kconf, "STM32_RCC_PLL_M")
    plln = sym_int(kconf, "STM32_RCC_PLL_N")
    pllp = int(choice_value(kconf, RCC_PLL_P_CHOICES).rsplit("kDiv", 1)[1])
    pllq = sym_int(kconf, "STM32_RCC_PLL_Q")
    ahb_div = int(choice_value(kconf, RCC_AHB_DIV_CHOICES).rsplit("kDiv", 1)[1])

    vco_in = src_hz / pllm
    vco_out = vco_in * plln
    sysclk = vco_out / pllp

    errors: list[str] = []
    if not PLL_VCO_IN_HZ[0] <= vco_in <= PLL_VCO_IN_HZ[1]:
        errors.append(
            f"PLL input is {vco_in / 1e6:.3f} MHz ({src_name} / PLLM {pllm}); "
            f"RM0090 requires 1..2 MHz"
        )
    if not PLL_VCO_OUT_HZ[0] <= vco_out <= PLL_VCO_OUT_HZ[1]:
        errors.append(
            f"VCO output is {vco_out / 1e6:.1f} MHz (PLL input x PLLN {plln}); "
            f"RM0090 requires 100..432 MHz"
        )
    if sysclk > SYSCLK_MAX_HZ:
        errors.append(
            f"SYSCLK is {sysclk / 1e6:.1f} MHz (VCO / PLLP {pllp}); "
            f"the STM32F407 maximum is 168 MHz"
        )
    # UsbCdc::CoreInit panics from inside System::Init when the core will not
    # reset, so an off-spec divider stops the board booting, not just USB.
    usb_clk = vco_out / pllq
    if usb_clk != USB_OTG_FS_HZ:
        errors.append(
            f"USB clock is {usb_clk / 1e6:.3f} MHz (VCO / PLLQ {pllq}); "
            f"OTG FS requires exactly 48 MHz"
        )

    # The bus ceilings are the one part of the tree the menu can violate on its
    # own: every APB divider is offered at every HCLK, and /1 at 168 MHz is
    # double what either bus is rated for.
    apb1_div = _apb_divider(kconf, 1)
    apb2_div = _apb_divider(kconf, 2)
    for bus, divider, maximum in (
        (1, apb1_div, APB1_MAX_HZ),
        (2, apb2_div, APB2_MAX_HZ),
    ):
        pclk = (sysclk / ahb_div) / divider
        if pclk > maximum:
            errors.append(
                f"PCLK{bus} is {pclk / 1e6:.1f} MHz (HCLK / APB{bus} divider "
                f"{divider}); the STM32F407 maximum is {maximum / 1e6:g} MHz"
            )

    # The scale's ceiling lives in the menu label and nowhere else, so scale 2
    # at 168 MHz is accepted today and browns out the core logic.
    scale = choice_value(kconf, RCC_VOLTAGE_SCALE_CHOICES)
    if scale.endswith("kScale2") and sysclk > VOLTAGE_SCALE2_MAX_HZ:
        errors.append(
            f"SYSCLK is {sysclk / 1e6:.1f} MHz on regulator scale 2, which the "
            f"STM32F407 caps at {VOLTAGE_SCALE2_MAX_HZ / 1e6:g} MHz; "
            "select scale 1 or lower the PLL"
        )

    if errors:
        raise SystemExit(
            "system clock configuration is invalid:\n  "
            + "\n  ".join(errors)
            + "\n\nCheck CONFIG_STM32_RCC_HSE_HZ matches the crystal actually "
            "fitted to the board, then the PLL dividers around it."
        )

    hclk = sysclk / ahb_div
    if hclk != int(hclk):
        raise SystemExit(
            f"HCLK is {hclk} Hz, not a whole number of hertz — "
            f"the PLL dividers do not divide evenly"
        )
    return int(hclk)


def _control_loop_hz(kconf: kconfiglib.Kconfig) -> int:
    """The rate the control cascade actually ticks at.

    Equal to CONFIG_STM32_CONTROL_LOOP_HZ in every configuration that builds,
    because _imu_watermark_records rejects a knob that does not divide the
    record rate evenly. Taking it from the record rate rather than reading the
    knob keeps the header carrying the rate the FIFO interrupt produces, which
    is what kControlLoopDtSec has to integrate over.
    """
    return _imu_record_rate_hz(kconf) // _imu_watermark_records(kconf)


def _rcc_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    hclk_hz = _solve_rcc_clock(kconf)
    return {
        "hclk_hz": hclk_hz,
        "flash_latency": _flash_wait_states(hclk_hz),
        "oscillator": choice_value(kconf, RCC_OSC_CHOICES),
        "pllm": sym_int(kconf, "STM32_RCC_PLL_M"),
        "plln": sym_int(kconf, "STM32_RCC_PLL_N"),
        "pllp": choice_value(kconf, RCC_PLL_P_CHOICES),
        "pllq": sym_int(kconf, "STM32_RCC_PLL_Q"),
        "ahb_divider": choice_value(kconf, RCC_AHB_DIV_CHOICES),
        "apb1_divider": choice_value(kconf, RCC_APB_DIV_CHOICES),
        "apb2_divider": choice_value(kconf, RCC_APB2_DIV_CHOICES),
        "voltage_scale": choice_value(kconf, RCC_VOLTAGE_SCALE_CHOICES),
    }


def _spi_prescaler(pclk_hz: int, max_sck_hz: int, who: str) -> str:
    """Fastest SPI_CR1.BR divider with SCK at or under max_sck_hz."""
    for divisor in SPI_PRESCALER_DIVISORS:
        if pclk_hz // divisor <= max_sck_hz:
            return f"SpiPrescaler::kDiv{divisor}"
    raise ValueError(
        f"no SPI prescaler brings {pclk_hz} Hz down to the {max_sck_hz} Hz "
        f"{who} allows; the slowest available is /{SPI_PRESCALER_DIVISORS[-1]}"
    )


def _adc_prescaler(pclk2_hz: int) -> int:
    """Fastest ADC_CCR.ADCPRE setting with ADCCLK at or under the cap."""
    for bits, divisor in enumerate(ADC_PRESCALER_DIVISORS):
        if pclk2_hz // divisor <= ADC_MAX_CLOCK_HZ:
            return bits
    raise ValueError(
        f"no ADC prescaler brings PCLK2 ({pclk2_hz} Hz) down to the "
        f"{ADC_MAX_CLOCK_HZ} Hz ADCCLK ceiling; the slowest available is "
        f"/{ADC_PRESCALER_DIVISORS[-1]}"
    )


def _timebase_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    """TIM2/TIM5 dividers, derived rather than configured.

    Both timers count microseconds, so one prescaler serves both. TIM2 free-runs
    as the 32-bit microsecond clock; TIM5's period is how many microseconds it
    counts before releasing a main-tick pass. A prescaler hand-set against an
    assumed bus clock is how TIM5 came to tick at 976.74 Hz while every comment
    called it 1 kHz.
    """
    hclk_hz = _solve_rcc_clock(kconf)
    apb1_div = _apb_divider(kconf, 1)
    timer_hz = _timer_clock_hz(hclk_hz // apb1_div, apb1_div)
    prescaler, remainder = divmod(timer_hz, MICROS_PER_SECOND)
    if remainder:
        raise ValueError(
            f"the APB1 timer clock ({timer_hz} Hz) is not a whole "
            "number of MHz, so no prescaler gives the microsecond tick "
            "TIM2 and TIM5 both count"
        )

    tick_hz = sym_int(kconf, "STM32_TIMEBASE_TIM5_TICK_HZ")
    tim5_counts, remainder = divmod(MICROS_PER_SECOND, tick_hz)
    if remainder:
        raise ValueError(
            f"CONFIG_STM32_TIMEBASE_TIM5_TICK_HZ ({tick_hz} Hz) does "
            "not divide "
            f"{MICROS_PER_SECOND}, so the tick would land a fraction of a "
            "microsecond early or late on every period"
        )

    return {
        "tim2": {
            "prescaler": prescaler - 1,
            "period": "0xFFFFFFFF",
        },
        "tim5": {
            "prescaler": prescaler - 1,
            "period": tim5_counts - 1,
            "autoreload_preload": sym_bool(
                kconf, "STM32_TIMEBASE_TIM5_AUTORELOAD_PRELOAD"
            ),
        },
    }


def _esc_telemetry_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "has_current": sym_bool(kconf, "STM32_ESC_TELEMETRY_HAS_CURRENT"),
        "response_timeout_us": sym_int(
            kconf, "STM32_ESC_TELEMETRY_RESPONSE_TIMEOUT_US"
        ),
    }


# Scheduled log topics, in MsgId order. The estimator stream is absent
# because it rides the control tick rather than the scheduler.
LOG_TOPICS = (
    "rc_input",
    "battery",
    "esc_telemetry",
    "gps",
    "imu_health",
    "crsf_link",
    "system_health",
    "logger_status",
)


def _log_service_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    """Preallocation size plus one cadence per scheduled topic.

    `max_silence` mirrors `period` and `priority` is uniform: this scheduler
    emits every due topic instead of choosing one, and never suppresses an
    unchanged payload, so neither field is read. They are emitted so the
    TopicConfig this renders is the same shape TelemetryPublisher's is.
    """
    topics = {
        name: {
            "period_ms": sym_int(
                kconf, f"STM32_LOG_TOPIC_{name.upper()}_PERIOD_MS"
            ),
        }
        for name in LOG_TOPICS
    }
    return {
        "prealloc_mb": sym_int(kconf, "STM32_LOG_PREALLOC_MB"),
        "topics": topics,
    }


def _sentinel_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    """Sentinel's thresholds, with the loss share resolved to a sample count.

    The knob is a share of the record rate so it keeps its meaning when the
    rate changes, and the rate is the CLKIN-scaled one rather than the nominal
    ODR label -- `missed_samples` counts real records. The firmware wants the
    count, and resolving it here keeps the multiply out of a comparison that
    runs every window. A share that rounds
    to zero would be met by an empty window and raise on every pass, so it
    fails the build rather than reaching the board.
    """
    rate_hz = _imu_record_rate_hz(kconf)
    permille = sym_int(kconf, "STM32_SENTINEL_IMU_LOSS_PERMILLE")
    window_ms = sym_int(kconf, "STM32_SENTINEL_IMU_LOSS_WINDOW_MS")

    samples_per_window = (rate_hz * window_ms) // 1000
    threshold = (samples_per_window * permille) // 1000
    if threshold == 0:
        raise SystemExit(
            f"STM32_SENTINEL_IMU_LOSS_PERMILLE={permille} over "
            f"{window_ms} ms at {rate_hz} Hz rounds to a zero-sample "
            f"threshold, which every window meets. Raise the share or "
            f"lengthen the window."
        )

    return {
        "loss_threshold_samples": threshold,
        "loss_window_ms": window_ms,
        "loss_permille": permille,
        "loss_consecutive": sym_int(
            kconf, "STM32_SENTINEL_IMU_LOSS_CONSECUTIVE"
        ),
        "fault_threshold": sym_int(kconf, "STM32_SENTINEL_IMU_FAULT_THRESHOLD"),
        "fault_window_ms": sym_int(
            kconf, "STM32_SENTINEL_IMU_FAULT_WINDOW_MS"
        ),
        "stall_timeout_ms": sym_int(
            kconf, "STM32_SENTINEL_IMU_STALL_TIMEOUT_MS"
        ),
        "test_throttle_silence_ms": sym_int(
            kconf, "STM32_SENTINEL_TEST_THROTTLE_SILENCE_MS"
        ),
    }


def _usb_cdc_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "vendor_id": sym_hex_literal(kconf, "STM32_USB_VENDOR_ID"),
        "product_id": sym_hex_literal(kconf, "STM32_USB_PRODUCT_ID"),
        "manufacturer": cpp_string_literal(
            sym_str(kconf, "STM32_USB_MANUFACTURER")
        ),
        **{
            key: cpp_string_literal(
                sym_str(kconf, "STM32_USB_PRODUCT") + suffix
            )
            for key, suffix in USB_PRODUCT_SUFFIXES.items()
        },
    }


def _msp_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "board_identifier": cpp_string_literal(
            sym_str(kconf, "STM32_MSP_BOARD_IDENTIFIER")
        ),
        "board_name": cpp_string_literal(
            sym_str(kconf, "STM32_MSP_BOARD_NAME")
        ),
        "manufacturer_id": cpp_string_literal(
            sym_str(kconf, "STM32_MSP_MANUFACTURER_ID")
        ),
        "craft_name": cpp_string_literal(
            sym_str(kconf, "STM32_MSP_CRAFT_NAME")
        ),
    }


def _esc_service_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "dshot_gap_bits": sym_int(kconf, "STM32_ESC_SERVICE_DSHOT_GAP_BITS"),
        "idle_period_us": sym_int(kconf, "STM32_ESC_SERVICE_IDLE_PERIOD_US"),
        "command_period_us": sym_int(
            kconf, "STM32_ESC_SERVICE_COMMAND_PERIOD_US"
        ),
        "telemetry_request_period_us": sym_int(
            kconf, "STM32_ESC_SERVICE_TELEMETRY_REQUEST_PERIOD_US"
        ),
        "command_repeat_count": sym_int(
            kconf, "STM32_ESC_SERVICE_COMMAND_REPEAT_COUNT"
        ),
        "firmware_checks": sym_bool(kconf, "STM32_ESC_SERVICE_FIRMWARE_CHECKS"),
    }


def _multirotor_mixer_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "idle_milli": sym_int(kconf, "STM32_MULTIROTOR_MIXER_IDLE_MILLI"),
        "pilot_throttle_min_milli": sym_int(
            kconf, "STM32_PILOT_THROTTLE_MIN_MILLI"
        ),
    }


def _iir_alpha_from_tau(time_constant: float, sample_period: float) -> float:
    """First-order IIR coefficient; both arguments in the same time unit.

    The exact pole mapping. T/(T+tau) looks equivalent and is not: it
    undershoots by more the closer the filter runs to the sample rate, which
    is where a knob stops meaning what it says. Zero disables.
    """
    if time_constant <= 0.0:
        return 1.0
    return 1.0 - math.exp(-sample_period / time_constant)


def _iir_alpha(cutoff_hz: int, sample_hz: int) -> float:
    """First-order IIR coefficient for a corner frequency, at the sample rate.

    alpha is a function of the sample period, so storing alpha instead of the
    corner pins the coefficient and lets the corner move: the same 0.1 that
    filters at 18 Hz on a 1024 Hz loop filters at 36 Hz on a 2048 Hz one, with
    nothing in the configuration to say the filter changed. 0 means no filter.
    """
    if cutoff_hz == 0:
        return 1.0
    return _iir_alpha_from_tau(
        1.0 / (2.0 * math.pi * cutoff_hz), 1.0 / sample_hz
    )


def _rate_controller_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    loop_hz = _control_loop_hz(kconf)

    def axis_gains(axis: str) -> dict[str, object]:
        return {
            "kp_milli": sym_int(kconf, f"STM32_RATE_CTRL_{axis}_KP_MILLI"),
            "ki_milli": sym_int(kconf, f"STM32_RATE_CTRL_{axis}_KI_MILLI"),
            "kd_micro": sym_int(kconf, f"STM32_RATE_CTRL_{axis}_KD_MICRO"),
            "sp_rate_limit": sym_int(
                kconf, f"STM32_RATE_CTRL_{axis}_SP_RATE_LIMIT"
            ),
            "sp_lpf_alpha": _iir_alpha(
                sym_int(kconf, f"STM32_RATE_CTRL_{axis}_SP_LPF_CUTOFF_HZ"),
                loop_hz,
            ),
        }

    return {
        "smoothing_enabled": sym_bool(
            kconf, "STM32_RATE_CTRL_SMOOTHING_ENABLED"
        ),
        "integrator_clamp_milli": sym_int(
            kconf, "STM32_RATE_CTRL_INTEGRATOR_CLAMP_MILLI"
        ),
        "output_clamp_milli": sym_int(
            kconf, "STM32_RATE_CTRL_OUTPUT_CLAMP_MILLI"
        ),
        "d_term_lpf_alpha": _iir_alpha(
            sym_int(kconf, "STM32_RATE_CTRL_DLPF_CUTOFF_HZ"), loop_hz
        ),
        "iterm_freeze_below_throttle_milli": sym_int(
            kconf, "STM32_RATE_CTRL_ITERM_FREEZE_BELOW_THROTTLE_MILLI"
        ),
        "i_factor_error_thresh_milli": sym_int(
            kconf, "STM32_RATE_CTRL_I_FACTOR_ERROR_THRESH_MILLI"
        ),
        "yaw_output_lpf_alpha": _iir_alpha(
            sym_int(kconf, "STM32_RATE_CTRL_YAW_OUTPUT_LPF_CUTOFF_HZ"), loop_hz
        ),
        "roll": axis_gains("ROLL"),
        "pitch": axis_gains("PITCH"),
        "yaw": axis_gains("YAW"),
    }


def _ahrs_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "kp_accel_milli": sym_int(kconf, "STM32_AHRS_KP_ACCEL_MILLI"),
        "ki_bias_milli": sym_int(kconf, "STM32_AHRS_KI_BIAS_MILLI"),
        "accel_trust_full_dev_milli": sym_int(
            kconf, "STM32_AHRS_ACCEL_TRUST_FULL_DEV_MILLI"
        ),
        "accel_trust_zero_dev_milli": sym_int(
            kconf, "STM32_AHRS_ACCEL_TRUST_ZERO_DEV_MILLI"
        ),
        "gyro_quiescent_full_milli": sym_int(
            kconf, "STM32_AHRS_GYRO_QUIESCENT_FULL_MILLI"
        ),
        "gyro_quiescent_zero_milli": sym_int(
            kconf, "STM32_AHRS_GYRO_QUIESCENT_ZERO_MILLI"
        ),
    }


def _attitude_controller_context(
    kconf: kconfiglib.Kconfig,
) -> dict[str, object]:
    def axis_gains(axis: str) -> dict[str, int]:
        return {
            "kp_milli": sym_int(kconf, f"STM32_ATT_CTRL_{axis}_KP_MILLI"),
            "rate_clamp_milli": sym_int(
                kconf, f"STM32_ATT_CTRL_{axis}_RATE_CLAMP_MILLI"
            ),
        }

    return {
        "roll": axis_gains("ROLL"),
        "pitch": axis_gains("PITCH"),
        "yaw": axis_gains("YAW"),
    }


def _fclink_topic_context(
    kconf: kconfiglib.Kconfig, key: str, *, max_silence: bool = False
) -> dict[str, object]:
    """One TelemetryPublisher topic; key picks the prefix.

    Only topics whose publisher suppresses unchanged payloads carry a
    max-silence knob; the rest emit zero, which means "never times out".
    """
    prefix = f"STM32_FCLINK_TOPIC_{key.upper()}"
    return {
        "period_ms": sym_int(kconf, f"{prefix}_PERIOD_MS"),
        "priority": sym_int(kconf, f"{prefix}_PRIORITY"),
        "max_silence_ms": (
            sym_int(kconf, f"{prefix}_MAX_SILENCE_MS") if max_silence else 0
        ),
    }


def _fclink_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "topics": {
            "max_frames_per_poll": sym_int(
                kconf, "STM32_FCLINK_TOPIC_MAX_FRAMES_PER_POLL"
            ),
            "system_status": _fclink_topic_context(kconf, "system_status"),
            "vehicle_status": _fclink_topic_context(kconf, "vehicle_status"),
            "esc_telemetry": _fclink_topic_context(kconf, "esc_telemetry"),
            "gps": _fclink_topic_context(kconf, "gps", max_silence=True),
            "attitude": _fclink_topic_context(
                kconf, "attitude", max_silence=True
            ),
            "rc_channels": _fclink_topic_context(
                kconf, "rc_channels", max_silence=True
            ),
            "usb_status": _fclink_topic_context(
                kconf, "usb_status", max_silence=True
            ),
        },
        "uart": {
            "stop_bits": _UART_STOP_BITS_VALUES[str(FCLINK_UART_STOP_BITS)],
            "over_sampling": choice_value(
                kconf, FCLINK_UART_OVERSAMPLING_CHOICES
            ),
        },
    }


def _m10_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "uart": {
            "baud_rate": choice_value(kconf, M10_UART_BAUD_RATE_CHOICES),
            "word_length": choice_value(kconf, M10_UART_WORD_LENGTH_CHOICES),
            "stop_bits": choice_value(kconf, M10_UART_STOP_BITS_CHOICES),
            "parity": choice_value(kconf, M10_UART_PARITY_CHOICES),
            "mode": choice_value(kconf, M10_UART_MODE_CHOICES),
            "hw_flow_control": choice_value(
                kconf, M10_UART_HW_FLOW_CONTROL_CHOICES
            ),
            "over_sampling": choice_value(kconf, M10_UART_OVERSAMPLING_CHOICES),
        },
        "config": {
            "baud_rate": choice_value(kconf, M10_BAUD_RATE_CHOICES),
            "uart1": {
                "enabled": sym_bool(kconf, "STM32_GPS_M10_UART_ENABLED"),
                "stop_bits": choice_value(
                    kconf, M10_CFG_UART_STOP_BITS_CHOICES
                ),
                "data_bits": _m10_uart_data_bits_value(kconf),
                "parity": choice_value(kconf, M10_CFG_UART_PARITY_CHOICES),
            },
            "protocols": {
                "outprot_ubx": sym_bool(kconf, "STM32_GPS_M10_PROTOCOL_UBX"),
                "outprot_nmea": sym_bool(kconf, "STM32_GPS_M10_PROTOCOL_NMEA"),
            },
            "messages": {
                "nav_pvt": sym_bool(kconf, "STM32_GPS_M10_MSG_NAV_PVT"),
                "nav_dop": sym_bool(kconf, "STM32_GPS_M10_MSG_NAV_DOP"),
                "nav_cov": sym_bool(kconf, "STM32_GPS_M10_MSG_NAV_COV"),
                "nav_eoe": sym_bool(kconf, "STM32_GPS_M10_MSG_NAV_EOE"),
            },
            "nav": {
                "rate_meas_ms": sym_int(kconf, "STM32_GPS_M10_RATE_MEAS_MS"),
                "dyn_model": choice_value(kconf, M10_DYNAMIC_MODEL_CHOICES),
            },
            "gnss": {
                "gps_enable": sym_bool(kconf, "STM32_GPS_M10_GNSS_GPS"),
                "glo_enable": sym_bool(kconf, "STM32_GPS_M10_GNSS_GLO"),
                "gal_enable": sym_bool(kconf, "STM32_GPS_M10_GNSS_GAL"),
                "bds_enable": sym_bool(kconf, "STM32_GPS_M10_GNSS_BDS"),
                "sbas_enable": sym_bool(kconf, "STM32_GPS_M10_GNSS_SBAS"),
                "itfm_enable": sym_bool(kconf, "STM32_GPS_M10_GNSS_ITFM"),
            },
            "tp1": {
                "ena": sym_bool(kconf, "STM32_GPS_M10_TP1_ENA"),
                "period": sym_int(kconf, "STM32_GPS_M10_TP1_PERIOD"),
                "len": sym_int(kconf, "STM32_GPS_M10_TP1_LEN"),
                "timegrid": choice_value(kconf, M10_TIMEGRID_CHOICES),
                "sync_gnss": sym_bool(kconf, "STM32_GPS_M10_TP1_SYNC_GNSS"),
                "align_to_tow": sym_bool(
                    kconf, "STM32_GPS_M10_TP1_ALIGN_TO_TOW"
                ),
                "pol_rising": sym_bool(kconf, "STM32_GPS_M10_TP1_POL_RISING"),
            },
            "ack_timeout_us": sym_int(kconf, "STM32_GPS_M10_ACK_TIMEOUT_US"),
        },
    }


def _ee_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    # SPI1 hangs off APB2, so the same divider is a different SCK here than on
    # the IMU's SPI2.
    apb2_div = _apb_divider(kconf, 2)
    prescaler = _spi_prescaler(
        _solve_rcc_clock(kconf) // apb2_div,
        sym_int(kconf, "STM32_EE_SPI1_MAX_SCK_HZ"),
        "the EEPROM flash",
    )
    return {"spi1_prescaler": prescaler}


def _sensor_cal_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    # Knobs of the procedure, not of the part: the symbols keep their IMU names,
    # but SensorCalService consumes them and nothing in the driver reads them.
    return {
        "duration_s": sym_int(kconf, "STM32_IMU_GYRO_CAL_DURATION_S"),
        "timeout_s": sym_int(kconf, "STM32_IMU_GYRO_CAL_TIMEOUT_S"),
        "still_threshold_raw": sym_int(
            kconf, "STM32_IMU_GYRO_CAL_STILL_THRESHOLD_RAW"
        ),
    }


def _icm42688p_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    # SPI2 hangs off APB1. The part is rated to 24 MHz and the flight loop wants
    # every bit of that, so take the fastest divider the ceiling allows.
    max_sck_hz = sym_int(kconf, "STM32_IMU_SPI_MAX_SCK_HZ")
    if max_sck_hz > ICM42688P_MAX_SCK_HZ:
        raise ValueError(
            f"CONFIG_STM32_IMU_SPI_MAX_SCK_HZ ({max_sck_hz} Hz) is above the "
            f"ICM42688P's {ICM42688P_MAX_SCK_HZ} Hz rating"
        )
    apb1_div = _apb_divider(kconf, 1)
    return {
        "spi_prescaler": _spi_prescaler(
            _solve_rcc_clock(kconf) // apb1_div, max_sck_hz, "the ICM42688P"
        ),
        "external_clock": {
            "enabled": sym_bool(kconf, "STM32_IMU_EXTERNAL_CLOCK_ENABLED"),
            "frequency_hz": sym_int(kconf, "STM32_IMU_EXTERNAL_CLOCK_FREQ_HZ"),
        },
        # One FIFO record carries both sensors, so a split rate would leave the
        # record rate -- and with it the loop rate -- undefined. Both fields
        # take the same Odr enumerator.
        "rates": {
            "gyro": choice_value(kconf, ODR_CHOICES),
            "accel": choice_value(kconf, ODR_CHOICES),
        },
        "fs": {
            "gyro": choice_value(kconf, GYRO_FS_CHOICES),
            "accel": choice_value(kconf, ACCEL_FS_CHOICES),
        },
        "ui_filter": {
            "gyro_bw": sym_int(kconf, "STM32_IMU_UI_FILTER_GYRO_BW"),
            "accel_bw": sym_int(kconf, "STM32_IMU_UI_FILTER_ACCEL_BW"),
            "gyro_cfg1": sym_int(kconf, "STM32_IMU_UI_FILTER_GYRO_CFG1"),
            "accel_cfg1": sym_int(kconf, "STM32_IMU_UI_FILTER_ACCEL_CFG1"),
        },
        "notch": {
            "freq_hz": (
                f"{float(sym_int(kconf, 'STM32_IMU_NOTCH_FREQ_HZ')):.6f}f"
            ),
            "bw_idx": sym_int(kconf, "STM32_IMU_NOTCH_BW_IDX"),
            "enabled": sym_bool(kconf, "STM32_IMU_NOTCH_ENABLED"),
        },
        "gyro_aaf": {
            "dis": sym_bool(kconf, "STM32_IMU_GYRO_AAF_DISABLE"),
            **_aaf_fields(choice_value(kconf, GYRO_AAF_CHOICES)),
        },
        "accel_aaf": {
            "dis": sym_bool(kconf, "STM32_IMU_ACCEL_AAF_DISABLE"),
            **_aaf_fields(choice_value(kconf, ACCEL_AAF_CHOICES)),
        },
        "fifo": {
            "watermark_records": _imu_watermark_records(kconf),
            "hold_last": sym_bool(kconf, "STM32_IMU_FIFO_HOLD_LAST"),
            "hires": sym_bool(kconf, "STM32_IMU_FIFO_HIRES_EN"),
        },
    }


def _button_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "active_low": sym_bool(kconf, "STM32_BUTTON_ACTIVE_LOW"),
        "debounce_ms": sym_int(kconf, "STM32_BUTTON_DEBOUNCE_MS"),
        "long_press_ms": sym_int(kconf, "STM32_BUTTON_LONG_PRESS_MS"),
    }


def _battery_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    # ADC1 hangs off APB2. Deriving the divider here rather than in the driver
    # is what turns an APB2 no ADCPRE can bring into range into a build error
    # instead of a board that panics on its first boot.
    current_monitoring = sym_bool(kconf, "STM32_BATTERY_CURRENT_MONITORING")
    context: dict[str, object] = {
        "current_monitoring": current_monitoring,
        "sample_period_ms": sym_int(kconf, "STM32_BATTERY_SAMPLE_PERIOD_MS"),
        "adc_reference_mv": sym_int(kconf, "STM32_BATTERY_ADC_REFERENCE_MV"),
        "adc_prescaler_bits": _adc_prescaler(
            _solve_rcc_clock(kconf) // _apb_divider(kconf, 2)
        ),
        "oversample_count": sym_int(
            kconf, "STM32_BATTERY_ADC_OVERSAMPLE_COUNT"
        ),
        "filter_alpha": _iir_alpha_from_tau(
            sym_int(kconf, "STM32_BATTERY_FILTER_TIME_CONSTANT_MS"),
            sym_int(kconf, "STM32_BATTERY_SAMPLE_PERIOD_MS"),
        ),
        "adc_timeout_us": sym_int(kconf, "STM32_BATTERY_ADC_TIMEOUT_US"),
        "voltage_multiplier_milli": sym_int(
            kconf, "STM32_BATTERY_VOLTAGE_MULTIPLIER_MILLI"
        ),
        "voltage_offset_mv": sym_int(kconf, "STM32_BATTERY_VOLTAGE_OFFSET_MV"),
        "cell_count": sym_int(kconf, "STM32_BATTERY_CELL_COUNT"),
        "cell_empty_mv": sym_int(kconf, "STM32_BATTERY_CELL_EMPTY_MV"),
        "cell_full_mv": sym_int(kconf, "STM32_BATTERY_CELL_FULL_MV"),
        # Charge already drawn from the pack on the bench is runtime state, not
        # a property of the build: honouring a non-zero value would mean a
        # regenerate, rebuild and reflash between packs. The field stays for
        # callers that set Config directly; the firmware always boots at zero.
        "initial_mah_drawn": 0,
    }

    # Absent rather than zero when the sense path is off, so the template has
    # nothing to render and the driver nothing to read.
    if current_monitoring:
        context.update(
            {
                "current_scale_ma_per_v": sym_int(
                    kconf, "STM32_BATTERY_CURRENT_SCALE_MA_PER_V"
                ),
                "current_offset_mv": sym_int(
                    kconf, "STM32_BATTERY_CURRENT_OFFSET_MV"
                ),
                "current_deadband_ma": sym_int(
                    kconf, "STM32_BATTERY_CURRENT_DEADBAND_MA"
                ),
            }
        )
    return context


def _crsf_periodic_msg_context(
    kconf: kconfiglib.Kconfig, key: str
) -> dict[str, object]:
    """CRSF GPS/battery messages share one shape; key picks the prefix."""
    prefix = f"STM32_RC_RECEIVER_CRSF_{key.upper()}"
    return {
        "period_ms": sym_int(kconf, f"{prefix}_PERIOD_MS"),
        "priority": sym_int(kconf, f"{prefix}_PRIORITY"),
        "send_on_change": sym_bool(kconf, f"{prefix}_SEND_ON_CHANGE"),
        "max_silence_ms": sym_int(kconf, f"{prefix}_MAX_SILENCE_MS"),
    }


def _rc_receiver_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "rc_map": _rc_map(kconf),
        "uart": {
            "baud_rate": choice_value(
                kconf, RC_RECEIVER_UART_BAUD_RATE_CHOICES
            ),
            "word_length": choice_value(
                kconf, RC_RECEIVER_UART_WORD_LENGTH_CHOICES
            ),
            "stop_bits": choice_value(
                kconf, RC_RECEIVER_UART_STOP_BITS_CHOICES
            ),
            "parity": choice_value(kconf, RC_RECEIVER_UART_PARITY_CHOICES),
            "mode": choice_value(kconf, RC_RECEIVER_UART_MODE_CHOICES),
            "hw_flow_control": choice_value(
                kconf, RC_RECEIVER_UART_HW_FLOW_CONTROL_CHOICES
            ),
            "over_sampling": choice_value(
                kconf, RC_RECEIVER_UART_OVERSAMPLING_CHOICES
            ),
        },
        "crsf": {
            "max_frames_per_poll": sym_int(
                kconf, "STM32_RC_RECEIVER_CRSF_MAX_FRAMES_PER_POLL"
            ),
            "gps_fresh_timeout_ms": sym_int(
                kconf, "STM32_RC_RECEIVER_CRSF_GPS_FRESH_TIMEOUT_MS"
            ),
            "heartbeat": {
                "period_ms": sym_int(
                    kconf, "STM32_RC_RECEIVER_CRSF_HEARTBEAT_PERIOD_MS"
                ),
                "priority": sym_int(
                    kconf, "STM32_RC_RECEIVER_CRSF_HEARTBEAT_PRIORITY"
                ),
                # Constant payload, so it is never suppressed and never has a
                # silence to bound. Present so the topics share one shape.
                "max_silence_ms": 0,
            },
            "gps": _crsf_periodic_msg_context(kconf, "gps"),
            "battery": _crsf_periodic_msg_context(kconf, "battery"),
            "flight_mode": _crsf_periodic_msg_context(
                kconf, "flight_mode"
            ),
            "attitude": _crsf_periodic_msg_context(kconf, "attitude"),
            "rpm": _crsf_periodic_msg_context(kconf, "rpm"),
            "temperature": _crsf_periodic_msg_context(
                kconf, "temperature"
            ),
        },
    }


def _runtime_context(
    source: pathlib.Path, kconf: kconfiglib.Kconfig
) -> dict[str, object]:
    return {
        "autogen_warning": autogen_warning(source),
        "pinmap": _pinmap_context(kconf),
        "led": {"active_low": sym_bool(kconf, "STM32_LED_ACTIVE_LOW")},
        "dshot_tim1": _dshot_tim1_context(kconf),
        "ee": _ee_context(kconf),
        "rcc": _rcc_context(kconf),
        "flight_mode": _flight_mode_context(kconf),
        "timebase": _timebase_context(kconf),
        "esc_telemetry": _esc_telemetry_context(kconf),
        "esc_service": _esc_service_context(kconf),
        "log_service": _log_service_context(kconf),
        "sentinel": _sentinel_context(kconf),
        "usb_cdc": _usb_cdc_context(kconf),
        "msp": _msp_context(kconf),
        "fclink": _fclink_context(kconf),
        "m10": _m10_context(kconf),
        "icm42688p": _icm42688p_context(kconf),
        "sensor_cal": _sensor_cal_context(kconf),
        "control_loop_hz": _control_loop_hz(kconf),
        "enable_profiling": sym_bool(kconf, "STM32_PROFILING"),
        "button": _button_context(kconf),
        "battery": _battery_context(kconf),
        "rc_receiver": _rc_receiver_context(kconf),
        "multirotor_mixer": _multirotor_mixer_context(kconf),
        "rate_controller": _rate_controller_context(kconf),
        "ahrs": _ahrs_context(kconf),
        "attitude_controller": _attitude_controller_context(kconf),
    }


def main() -> int:
    return run_generator(
        validate_fn=_validate,
        runtime_context_fn=_runtime_context,
        limits_context_fn=_limits_context,
        runtime_template_name="stm32_config.hpp.j2",
        limits_template_name="stm32_limits.hpp.j2",
    )


if __name__ == "__main__":
    raise SystemExit(main())
