#!/usr/bin/env python3

# /// script
# dependencies = [
#     "jinja2",
#     "kconfiglib",
# ]
# ///

from __future__ import annotations

import pathlib
import sys
from dataclasses import dataclass

import kconfiglib

# Sibling module — pin_constraints.py lives next to this script.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from pin_constraints import PinConstraints  # noqa: E402
from kconfig_gen import (  # noqa: E402
    autogen_warning,
    choice_value,
    sym_bool,
    sym_hex_literal,
    sym_int,
)
from kconfig_gen import run as run_generator  # noqa: E402

ICM42688P_FIFO_BYTES = 2048
ICM42688P_PACKET3_BYTES = 16
FCLINK_TELEMETRY_RATE_MIN = 1
FCLINK_TELEMETRY_RATE_MAX = 255
BATTERY_ADC_RESOLUTION_BITS = 12
BATTERY_ADC_MAX_RAW = (1 << BATTERY_ADC_RESOLUTION_BITS) - 1


SPI_PRESCALER_CHOICES = {
    f"STM32_IMU_SPI_PRESCALER_DIV{n}": f"SpiPrescaler::kDiv{n}"
    for n in (2, 4, 8, 16, 32, 64, 128, 256)
}

EE_SPI1_PRESCALER_CHOICES = {
    f"STM32_EE_SPI1_PRESCALER_DIV{n}": f"SpiPrescaler::kDiv{n}"
    for n in (2, 4, 8, 16, 32, 64, 128, 256)
}

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

# FcLink baud rates are owned by the ESP32 side (the link-shared symbol lives
# under ESP32_FCLINK_UART_BAUD_*); the rest of the framing config lives
# under STM32_FCLINK_UART_*.
FCLINK_UART_BAUD_RATE_CHOICES = {
    f"ESP32_FCLINK_UART_BAUD_{rate}": str(rate)
    for rate in (
        9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600,
        1000000, 2000000, 5000000,
    )
}
FCLINK_UART_WORD_LENGTH_CHOICES = _prefixed(
    "STM32_FCLINK_UART_WORD_LENGTH", _UART_WORD_LENGTH_VALUES
)
FCLINK_UART_STOP_BITS_CHOICES = _prefixed(
    "STM32_FCLINK_UART_STOP_BITS", _UART_STOP_BITS_VALUES
)
FCLINK_UART_PARITY_CHOICES = _prefixed(
    "STM32_FCLINK_UART_PARITY", _UART_PARITY_VALUES
)
FCLINK_UART_MODE_CHOICES = _prefixed(
    "STM32_FCLINK_UART_MODE", _UART_MODE_VALUES
)
FCLINK_UART_HW_FLOW_CONTROL_CHOICES = _prefixed(
    "STM32_FCLINK_UART_HW_FLOW_CONTROL", _UART_HW_FLOW_VALUES
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
M10_UART_MODE_CHOICES = _prefixed(
    "STM32_GPS_M10_UART_MODE", _UART_MODE_VALUES
)
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

ACCEL_ODR_CHOICES = {
    key.replace("GYRO", "ACCEL"): value for key, value in ODR_CHOICES.items()
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

# ---- System clock choice maps --------------------------------------------
# The system_clock enum classes are hand-written in stm32/Core/Inc/system.hpp
# (matching the SPI / UART pattern: enum values are HAL register bits, so the
# C++ side can `static_cast` straight into the HAL init structs). These dicts
# only carry the Kconfig-symbol -> C++-enum-value mapping the generator needs
# to render `kSystemDefault` in stm32_config.hpp.

def _system_clock_choices(
    enum_cpp_name: str, kconfig_to_value: dict[str, str]
) -> dict[str, str]:
    """Build a Kconfig sym -> `System::<Enum>::<value>` dict."""
    return {
        sym: f"System::{enum_cpp_name}::{cpp_name}"
        for sym, cpp_name in kconfig_to_value.items()
    }


SYSTEM_OSC_CHOICES = _system_clock_choices(
    "Oscillator",
    {"STM32_SYSTEM_OSC_HSI": "kHsi", "STM32_SYSTEM_OSC_HSE": "kHse"},
)
SYSTEM_PLL_P_CHOICES = _system_clock_choices(
    "PllP",
    {f"STM32_SYSTEM_PLL_P_DIV{n}": f"kDiv{n}" for n in (2, 4, 6, 8)},
)
SYSTEM_AHB_DIV_CHOICES = _system_clock_choices(
    "AhbDiv",
    {
        f"STM32_SYSTEM_AHB_DIV{n}": f"kDiv{n}"
        for n in (1, 2, 4, 8, 16, 64, 128, 256, 512)
    },
)
SYSTEM_APB_DIV_CHOICES = _system_clock_choices(
    "ApbDiv",
    {f"STM32_SYSTEM_APB1_DIV{n}": f"kDiv{n}" for n in (1, 2, 4, 8, 16)},
)
SYSTEM_APB2_DIV_CHOICES = _system_clock_choices(
    "ApbDiv",
    {f"STM32_SYSTEM_APB2_DIV{n}": f"kDiv{n}" for n in (1, 2, 4, 8, 16)},
)
SYSTEM_VOLTAGE_SCALE_CHOICES = _system_clock_choices(
    "VoltageScale",
    {f"STM32_SYSTEM_VOLTAGE_SCALE{n}": f"kScale{n}" for n in (1, 2)},
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
    direction: str = "output"          # "output" | "input"
    speed: str = "low"                 # output speed; ignored for inputs
    pull: str | None = None            # input pull; None → derive from active_low
    active_low: bool = False           # constant (used when active_low_sym is None)
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


# F407V ADC1 input-channel mapping. ADC2 shares the same pinout; ADC3 has a
# different (smaller) set we don't use.
_ADC1_CHANNEL_MAP: dict[str, int] = {
    **{f"PA{n}": n for n in range(8)},      # IN0..IN7
    "PB0": 8, "PB1": 9,                     # IN8..IN9
    **{f"PC{n}": 10 + n for n in range(6)}, # IN10..IN15
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
    # to PB11; PC11 / PD9 are the other silicon-valid choices. The line
    # idles high — pull-up holds it there when the ESC isn't driving.
    _SignalPin(
        board_const="kEscTlmRx",
        signal="USART3_RX",
        choice_options={
            "STM32_ESC_TLM_RX_PIN_PB11": "PB11",
            "STM32_ESC_TLM_RX_PIN_PC11": "PC11",
            "STM32_ESC_TLM_RX_PIN_PD9": "PD9",
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
            "STM32_DSHOT_MOTOR4_PA11": "PA11",
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
    ),
)


def _m10_uart_data_bits_value(kconf: kconfiglib.Kconfig) -> str:
    word_length_9 = sym_bool(kconf, "STM32_GPS_M10_UART_WORD_LENGTH_9BITS")
    parity_none = sym_bool(kconf, "STM32_GPS_M10_UART_PARITY_NONE")

    if word_length_9 and parity_none:
        raise ValueError(
            "CONFIG_STM32_GPS_M10_UART_WORD_LENGTH_9BITS requires parity on the "
            "M10 link; choose 8 bits or enable parity"
        )

    if word_length_9:
        return "M10::UartDataBits::k8"
    if parity_none:
        return "M10::UartDataBits::k8"
    return "M10::UartDataBits::k7"


def _imu_fifo_capacity_records() -> int:
    return ICM42688P_FIFO_BYTES // ICM42688P_PACKET3_BYTES


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


def _validate_pinmap(kconf: kconfiglib.Kconfig) -> None:
    db = PinConstraints.load_default()
    for entry in PINMAP_ENTRIES:
        _validate_pinmap_entry(db, kconf, entry)


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

        rendered.append({
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
        })
    return rendered


def _validate(kconf: kconfiglib.Kconfig) -> None:
    _validate_pinmap(kconf)

    telemetry_rate_hz = sym_int(kconf, "STM32_FCLINK_TELEMETRY_RATE_HZ")
    if not FCLINK_TELEMETRY_RATE_MIN <= telemetry_rate_hz <= FCLINK_TELEMETRY_RATE_MAX:
        raise ValueError(
            "CONFIG_STM32_FCLINK_TELEMETRY_RATE_HZ must be in the range "
            f"{FCLINK_TELEMETRY_RATE_MIN}..{FCLINK_TELEMETRY_RATE_MAX}"
        )

    watermark_records = sym_int(kconf, "STM32_IMU_FIFO_WATERMARK_RECORDS")
    hardware_max_records = _imu_fifo_capacity_records()
    if watermark_records <= 0:
        raise ValueError("CONFIG_STM32_IMU_FIFO_WATERMARK_RECORDS must be > 0")
    if watermark_records > hardware_max_records:
        raise ValueError(
            "CONFIG_STM32_IMU_FIFO_WATERMARK_RECORDS exceeds ICM42688P FIFO capacity "
            f"({watermark_records} > {hardware_max_records})"
        )

    rc_map = _rc_map(kconf)
    if sorted(rc_map.values()) != [1, 2, 3, 4]:
        raise ValueError(
            "CONFIG_STM32_RC_MAP_ROLL/PITCH/YAW/THROTTLE must be a unique "
            "mapping of channels 1..4"
        )

    enabled_channels = _enabled_rc_channels(kconf)
    for axis, channel in rc_map.items():
        if not enabled_channels[channel - 1]:
            raise ValueError(
                f"CONFIG_STM32_RC_MAP_{axis.upper()} requires channel {channel} "
                "to be enabled"
            )

    cell_empty_mv = sym_int(kconf, "STM32_BATTERY_CELL_EMPTY_MV")
    cell_full_mv = sym_int(kconf, "STM32_BATTERY_CELL_FULL_MV")
    if cell_empty_mv >= cell_full_mv:
        raise ValueError(
            "CONFIG_STM32_BATTERY_CELL_EMPTY_MV must be lower than "
            "CONFIG_STM32_BATTERY_CELL_FULL_MV"
        )


def _enabled_rc_channels(kconf: kconfiglib.Kconfig) -> list[bool]:
    return [
        sym_bool(kconf, f"STM32_RC_CHANNEL_{channel_index + 1}_ENABLED")
        for channel_index in range(16)
    ]


def _rc_map(kconf: kconfiglib.Kconfig) -> dict[str, int]:
    return {
        "roll": sym_int(kconf, "STM32_RC_MAP_ROLL"),
        "pitch": sym_int(kconf, "STM32_RC_MAP_PITCH"),
        "yaw": sym_int(kconf, "STM32_RC_MAP_YAW"),
        "throttle": sym_int(kconf, "STM32_RC_MAP_THROTTLE"),
    }


def _limits_context(source: pathlib.Path, kconf: kconfiglib.Kconfig) -> dict[str, object]:
    enabled_rc_channels = _enabled_rc_channels(kconf)
    enabled_rc_indices = [
        channel_index
        for channel_index, enabled in enumerate(enabled_rc_channels)
        if enabled
    ]
    return {
        "autogen_warning": autogen_warning(source),
        "max_watermark_records": sym_int(kconf, "STM32_IMU_FIFO_WATERMARK_RECORDS"),
        "imu_hires_en": sym_bool(kconf, "STM32_IMU_FIFO_HIRES_EN"),
        "rc_enabled_indices": enabled_rc_indices,
        "battery_adc_resolution_bits": BATTERY_ADC_RESOLUTION_BITS,
        "battery_adc_max_raw": BATTERY_ADC_MAX_RAW,
    }


# ---- Per-peripheral runtime contexts -------------------------------------
# Each helper resolves Kconfig + chosen-value maps for one peripheral and
# returns the sub-dict used by the matching block in stm32_config.hpp.j2.
# `_runtime_context` just stitches them together.

def _system_clock_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "oscillator": choice_value(kconf, SYSTEM_OSC_CHOICES),
        "pllm": sym_int(kconf, "STM32_SYSTEM_PLL_M"),
        "plln": sym_int(kconf, "STM32_SYSTEM_PLL_N"),
        "pllp": choice_value(kconf, SYSTEM_PLL_P_CHOICES),
        "pllq": sym_int(kconf, "STM32_SYSTEM_PLL_Q"),
        "ahb_divider": choice_value(kconf, SYSTEM_AHB_DIV_CHOICES),
        "apb1_divider": choice_value(kconf, SYSTEM_APB_DIV_CHOICES),
        "apb2_divider": choice_value(kconf, SYSTEM_APB2_DIV_CHOICES),
        "flash_latency": sym_int(kconf, "STM32_SYSTEM_FLASH_LATENCY"),
        "voltage_scale": choice_value(kconf, SYSTEM_VOLTAGE_SCALE_CHOICES),
    }


def _timebase_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "tim2": {
            "prescaler": sym_int(kconf, "STM32_TIMEBASE_TIM2_PRESCALER"),
            "period": sym_hex_literal(kconf, "STM32_TIMEBASE_TIM2_PERIOD"),
        },
        "tim5": {
            "prescaler": sym_int(kconf, "STM32_TIMEBASE_TIM5_PRESCALER"),
            "period": sym_int(kconf, "STM32_TIMEBASE_TIM5_PERIOD"),
            "autoreload_preload": sym_bool(
                kconf, "STM32_TIMEBASE_TIM5_AUTORELOAD_PRELOAD"
            ),
        },
    }


def _esc_telemetry_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "baud_rate": sym_int(kconf, "STM32_ESC_TELEMETRY_BAUD_RATE"),
        "motor_pole_count": sym_int(kconf, "STM32_ESC_TELEMETRY_MOTOR_POLE_COUNT"),
        "response_timeout_us": sym_int(
            kconf, "STM32_ESC_TELEMETRY_RESPONSE_TIMEOUT_US"
        ),
    }


def _esc_service_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "dshot_gap_bits": sym_int(kconf, "STM32_ESC_SERVICE_DSHOT_GAP_BITS"),
        "idle_period_us": sym_int(kconf, "STM32_ESC_SERVICE_IDLE_PERIOD_US"),
        "command_period_us": sym_int(kconf, "STM32_ESC_SERVICE_COMMAND_PERIOD_US"),
        "telemetry_request_period_us": sym_int(
            kconf, "STM32_ESC_SERVICE_TELEMETRY_REQUEST_PERIOD_US"
        ),
        "command_repeat_count": sym_int(
            kconf, "STM32_ESC_SERVICE_COMMAND_REPEAT_COUNT"
        ),
    }


def _multirotor_mixer_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "idle_milli": sym_int(kconf, "STM32_MULTIROTOR_MIXER_IDLE_MILLI"),
        "pilot_throttle_min_milli": sym_int(
            kconf, "STM32_PILOT_THROTTLE_MIN_MILLI"
        ),
    }


def _rate_controller_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    def axis_gains(axis: str) -> dict[str, int]:
        return {
            "kp_milli": sym_int(kconf, f"STM32_RATE_CTRL_{axis}_KP_MILLI"),
            "ki_milli": sym_int(kconf, f"STM32_RATE_CTRL_{axis}_KI_MILLI"),
            "kd_micro": sym_int(kconf, f"STM32_RATE_CTRL_{axis}_KD_MICRO"),
            "sp_rate_limit": sym_int(
                kconf, f"STM32_RATE_CTRL_{axis}_SP_RATE_LIMIT"
            ),
            "sp_lpf_alpha_milli": sym_int(
                kconf, f"STM32_RATE_CTRL_{axis}_SP_LPF_ALPHA_MILLI"
            ),
        }

    return {
        "smoothing_enabled": sym_bool(kconf, "STM32_RATE_CTRL_SMOOTHING_ENABLED"),
        "integrator_clamp_milli": sym_int(
            kconf, "STM32_RATE_CTRL_INTEGRATOR_CLAMP_MILLI"
        ),
        "output_clamp_milli": sym_int(kconf, "STM32_RATE_CTRL_OUTPUT_CLAMP_MILLI"),
        "d_term_lpf_alpha_milli": sym_int(kconf, "STM32_RATE_CTRL_DLPF_ALPHA_MILLI"),
        "iterm_freeze_below_throttle_milli": sym_int(
            kconf, "STM32_RATE_CTRL_ITERM_FREEZE_BELOW_THROTTLE_MILLI"
        ),
        "i_factor_error_thresh_milli": sym_int(
            kconf, "STM32_RATE_CTRL_I_FACTOR_ERROR_THRESH_MILLI"
        ),
        "yaw_output_lpf_alpha_milli": sym_int(
            kconf, "STM32_RATE_CTRL_YAW_OUTPUT_LPF_ALPHA_MILLI"
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


def _attitude_controller_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
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


def _fclink_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "telemetry_rate_hz": sym_int(kconf, "STM32_FCLINK_TELEMETRY_RATE_HZ"),
        "uart": {
            "baud_rate": choice_value(kconf, FCLINK_UART_BAUD_RATE_CHOICES),
            "word_length": choice_value(kconf, FCLINK_UART_WORD_LENGTH_CHOICES),
            "stop_bits": choice_value(kconf, FCLINK_UART_STOP_BITS_CHOICES),
            "parity": choice_value(kconf, FCLINK_UART_PARITY_CHOICES),
            "mode": choice_value(kconf, FCLINK_UART_MODE_CHOICES),
            "hw_flow_control": choice_value(
                kconf, FCLINK_UART_HW_FLOW_CONTROL_CHOICES
            ),
            "over_sampling": choice_value(kconf, FCLINK_UART_OVERSAMPLING_CHOICES),
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
            "hw_flow_control": choice_value(kconf, M10_UART_HW_FLOW_CONTROL_CHOICES),
            "over_sampling": choice_value(kconf, M10_UART_OVERSAMPLING_CHOICES),
        },
        "config": {
            "baud_rate": choice_value(kconf, M10_BAUD_RATE_CHOICES),
            "uart1": {
                "enabled": sym_bool(kconf, "STM32_GPS_M10_UART_ENABLED"),
                "stop_bits": choice_value(kconf, M10_CFG_UART_STOP_BITS_CHOICES),
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
                "align_to_tow": sym_bool(kconf, "STM32_GPS_M10_TP1_ALIGN_TO_TOW"),
                "pol_rising": sym_bool(kconf, "STM32_GPS_M10_TP1_POL_RISING"),
            },
            "ack_timeout_us": sym_int(kconf, "STM32_GPS_M10_ACK_TIMEOUT_US"),
        },
    }


def _icm42688p_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "spi_prescaler": choice_value(kconf, SPI_PRESCALER_CHOICES),
        "external_clock": {
            "enabled": sym_bool(kconf, "STM32_IMU_EXTERNAL_CLOCK_ENABLED"),
            "frequency_hz": sym_int(kconf, "STM32_IMU_EXTERNAL_CLOCK_FREQ_HZ"),
        },
        "rates": {
            "gyro": choice_value(kconf, ODR_CHOICES),
            "accel": choice_value(kconf, ACCEL_ODR_CHOICES),
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
            "freq_hz": f'{float(sym_int(kconf, "STM32_IMU_NOTCH_FREQ_HZ")):.6f}f',
            "bw_idx": sym_int(kconf, "STM32_IMU_NOTCH_BW_IDX"),
            "enabled": sym_bool(kconf, "STM32_IMU_NOTCH_ENABLED"),
        },
        "gyro_aaf": {
            "dis": sym_bool(kconf, "STM32_IMU_GYRO_AAF_DISABLE"),
            "delt": sym_int(kconf, "STM32_IMU_GYRO_AAF_DELT"),
            "delt_sqr": sym_int(kconf, "STM32_IMU_GYRO_AAF_DELT_SQR"),
            "bitshift": sym_int(kconf, "STM32_IMU_GYRO_AAF_BITSHIFT"),
        },
        "accel_aaf": {
            "dis": sym_bool(kconf, "STM32_IMU_ACCEL_AAF_DISABLE"),
            "delt": sym_int(kconf, "STM32_IMU_ACCEL_AAF_DELT"),
            "delt_sqr": sym_int(kconf, "STM32_IMU_ACCEL_AAF_DELT_SQR"),
            "bitshift": sym_int(kconf, "STM32_IMU_ACCEL_AAF_BITSHIFT"),
        },
        "fifo": {
            "watermark_records": sym_int(kconf, "STM32_IMU_FIFO_WATERMARK_RECORDS"),
            "hold_last": sym_bool(kconf, "STM32_IMU_FIFO_HOLD_LAST"),
        },
        "calibration": {
            "gyro_duration_s": sym_int(kconf, "STM32_IMU_GYRO_CAL_DURATION_S"),
            "gyro_timeout_s": sym_int(kconf, "STM32_IMU_GYRO_CAL_TIMEOUT_S"),
            "gyro_still_threshold_raw": sym_int(
                kconf, "STM32_IMU_GYRO_CAL_STILL_THRESHOLD_RAW"
            ),
        },
        "recovery": {
            "overrun_threshold": sym_int(
                kconf, "STM32_IMU_RECOVERY_OVERRUN_THRESHOLD"
            ),
            "overrun_window_s": sym_int(
                kconf, "STM32_IMU_RECOVERY_OVERRUN_WINDOW_S"
            ),
            "fault_led_period_ms": sym_int(
                kconf, "STM32_IMU_RECOVERY_FAULT_LED_PERIOD_MS"
            ),
        },
    }


def _button_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "active_low": sym_bool(kconf, "STM32_BUTTON_ACTIVE_LOW"),
        "debounce_ms": sym_int(kconf, "STM32_BUTTON_DEBOUNCE_MS"),
        "long_press_ms": sym_int(kconf, "STM32_BUTTON_LONG_PRESS_MS"),
    }


def _battery_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "sample_period_ms": sym_int(kconf, "STM32_BATTERY_SAMPLE_PERIOD_MS"),
        "adc_reference_mv": sym_int(kconf, "STM32_BATTERY_ADC_REFERENCE_MV"),
        "oversample_count": sym_int(kconf, "STM32_BATTERY_ADC_OVERSAMPLE_COUNT"),
        "filter_alpha_permille": sym_int(
            kconf, "STM32_BATTERY_FILTER_ALPHA_PERMILLE"
        ),
        "adc_timeout_us": sym_int(kconf, "STM32_BATTERY_ADC_TIMEOUT_US"),
        "voltage_multiplier_milli": sym_int(
            kconf, "STM32_BATTERY_VOLTAGE_MULTIPLIER_MILLI"
        ),
        "voltage_offset_mv": sym_int(kconf, "STM32_BATTERY_VOLTAGE_OFFSET_MV"),
        "cell_count": sym_int(kconf, "STM32_BATTERY_CELL_COUNT"),
        "cell_empty_mv": sym_int(kconf, "STM32_BATTERY_CELL_EMPTY_MV"),
        "cell_full_mv": sym_int(kconf, "STM32_BATTERY_CELL_FULL_MV"),
        "current_scale_ma_per_v": sym_int(
            kconf, "STM32_BATTERY_CURRENT_SCALE_MA_PER_V"
        ),
        "current_offset_mv": sym_int(kconf, "STM32_BATTERY_CURRENT_OFFSET_MV"),
        "current_deadband_ma": sym_int(kconf, "STM32_BATTERY_CURRENT_DEADBAND_MA"),
        "initial_mah_drawn": sym_int(kconf, "STM32_BATTERY_INITIAL_MAH_DRAWN"),
    }


def _crsf_periodic_msg_context(
    kconf: kconfiglib.Kconfig, key: str
) -> dict[str, object]:
    """CRSF GPS / battery messages share the same shape; key picks the prefix."""
    prefix = f"STM32_RC_RECEIVER_CRSF_{key.upper()}"
    return {
        "period_ms": sym_int(kconf, f"{prefix}_PERIOD_MS"),
        "start_delay_ms": sym_int(kconf, f"{prefix}_START_DELAY_MS"),
        "priority": sym_int(kconf, f"{prefix}_PRIORITY"),
        "send_on_change": sym_bool(kconf, f"{prefix}_SEND_ON_CHANGE"),
        "max_silence_ms": sym_int(kconf, f"{prefix}_MAX_SILENCE_MS"),
    }


def _rc_receiver_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "enabled_channels": _enabled_rc_channels(kconf),
        "rc_map": _rc_map(kconf),
        "uart": {
            "baud_rate": choice_value(kconf, RC_RECEIVER_UART_BAUD_RATE_CHOICES),
            "word_length": choice_value(
                kconf, RC_RECEIVER_UART_WORD_LENGTH_CHOICES
            ),
            "stop_bits": choice_value(kconf, RC_RECEIVER_UART_STOP_BITS_CHOICES),
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
                "start_delay_ms": sym_int(
                    kconf, "STM32_RC_RECEIVER_CRSF_HEARTBEAT_START_DELAY_MS"
                ),
                "priority": sym_int(
                    kconf, "STM32_RC_RECEIVER_CRSF_HEARTBEAT_PRIORITY"
                ),
            },
            "gps": _crsf_periodic_msg_context(kconf, "gps"),
            "battery": _crsf_periodic_msg_context(kconf, "battery"),
        },
    }


def _runtime_context(
    source: pathlib.Path, kconf: kconfiglib.Kconfig
) -> dict[str, object]:
    return {
        "autogen_warning": autogen_warning(source),
        "pinmap": _pinmap_context(kconf),
        "led": {"active_low": sym_bool(kconf, "STM32_LED_ACTIVE_LOW")},
        "dshot_tim1": {"mode": choice_value(kconf, DSHOT_TIM1_MODE_CHOICES)},
        "ee": {"spi1_prescaler": choice_value(kconf, EE_SPI1_PRESCALER_CHOICES)},
        "system_clock": _system_clock_context(kconf),
        "timebase": _timebase_context(kconf),
        "esc_telemetry": _esc_telemetry_context(kconf),
        "esc_service": _esc_service_context(kconf),
        "fclink": _fclink_context(kconf),
        "m10": _m10_context(kconf),
        "icm42688p": _icm42688p_context(kconf),
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
