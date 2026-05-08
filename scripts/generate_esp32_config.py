#!/usr/bin/env python3

# /// script
# dependencies = [
#     "jinja2",
#     "kconfiglib",
# ]
# ///

from __future__ import annotations

import pathlib
import subprocess

import kconfiglib
from generate_firmware_ver import resolve_firmware_version
from kconfig_gen import (
    autogen_warning,
    choice_value,
    cpp_string_literal,
    sym_bool,
    sym_int,
    sym_str,
)
from kconfig_gen import run as run_generator


REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent

ESP32C3_GPIO_MIN = 0
ESP32C3_GPIO_MAX = 21
FCLINK_TELEMETRY_RATE_MIN = 1
FCLINK_TELEMETRY_RATE_MAX = 255
FCLINK_INVALID_PACKET_THRESHOLD_MIN = 1
FCLINK_INVALID_PACKET_THRESHOLD_MAX = 255
FCLINK_HANDSHAKE_ATTEMPTS_MIN = 1
FCLINK_HANDSHAKE_ATTEMPTS_MAX = 1000
FCLINK_HANDSHAKE_RETRY_PERIOD_MIN_MS = 1
FCLINK_HANDSHAKE_RETRY_PERIOD_MAX_MS = 1000
FCLINK_RX_QUEUE_DEPTH_MIN = 1
FCLINK_RX_QUEUE_DEPTH_MAX = 32
UART_BUFFER_SIZE_MIN = 256
UART_BUFFER_SIZE_MAX = 16384
MAVLINK_SYSID_MIN = 1
MAVLINK_SYSID_MAX = 255
MAVLINK_COMPID_MIN = 0
MAVLINK_COMPID_MAX = 255
MAVLINK_HEARTBEAT_PERIOD_MIN_MS = 1
MAVLINK_HEARTBEAT_PERIOD_MAX_MS = 60000
MAVLINK_TX_PERIOD_MIN_MS = 0
MAVLINK_TX_PERIOD_MAX_MS = 60000
MAVLINK_HEARTBEAT_DEADLINE_MIN_MS = 1
MAVLINK_HEARTBEAT_DEADLINE_MAX_MS = 60000
MAVLINK_TX_SCHEDULE_START_DELAY_MIN_MS = 0
MAVLINK_TX_SCHEDULE_START_DELAY_MAX_MS = 10000
MAVLINK_FIRMWARE_VERSION_TYPE_OFFICIAL = 0xFF

# MAVLink runtime metadata (the parts that aren't Kconfig-tunable today).
# `MAVLINK_SYSTEM_STATUS_FRESH_MS` bounds how long a SYS_STATUS sample
# stays current before downstream fields drop out — protocol timing, not
# expected to vary per board.
MAVLINK_SYSTEM_STATUS_FRESH_MS = 3000

# PX4 SYS_AUTOSTART airframe IDs surfaced as a Kconfig choice. The
# canonical full list is huge; we curate the common multirotor / plane /
# VTOL / rover entries and let the user supply a raw integer for anything
# else via ESP32_MAVLINK_SYS_AUTOSTART_CUSTOM.
MAVLINK_SYS_AUTOSTART_CHOICES: dict[str, int] = {
    "ESP32_MAVLINK_SYS_AUTOSTART_QUAD_X": 4001,
    "ESP32_MAVLINK_SYS_AUTOSTART_QUAD_PLUS": 4002,
    "ESP32_MAVLINK_SYS_AUTOSTART_HEXA_X": 4011,
    "ESP32_MAVLINK_SYS_AUTOSTART_HEXA_PLUS": 4012,
    "ESP32_MAVLINK_SYS_AUTOSTART_OCTO_X": 4013,
    "ESP32_MAVLINK_SYS_AUTOSTART_PLANE": 100,
    "ESP32_MAVLINK_SYS_AUTOSTART_VTOL_STANDARD": 13000,
    "ESP32_MAVLINK_SYS_AUTOSTART_VTOL_TAILSITTER": 13003,
    "ESP32_MAVLINK_SYS_AUTOSTART_ROVER": 50000,
}

# Panic background task (FreeRTOS).
PANIC_TASK_STACK_DEPTH_WORDS = 4096
PANIC_TASK_PRIORITY = 24

# SSD1306 panel hardware geometry. Fixed by the chosen 72x40 display
# module + its SSD1306 controller's column offset.
DISPLAY_PANEL_WIDTH_PX = 72
DISPLAY_PANEL_HEIGHT_PX = 40
DISPLAY_PANEL_CONTROLLER_WIDTH_PX = 128
DISPLAY_PANEL_COLUMN_OFFSET_PX = 28

# Compile-time buffer / queue sizes used as template parameters.
TCP_SERVER_MAX_LINE_BYTES = 160
TONE_PLAYER_PENDING_REQUEST_QUEUE_DEPTH = 5
WIFI_AP_SSID_MIN_LEN = 1
WIFI_AP_SSID_MAX_LEN = 32
WIFI_AP_PASSWORD_MAX_LEN = 63
WIFI_AP_PASSWORD_MIN_LEN = 8
WIFI_AP_CHANNEL_MIN = 1
WIFI_AP_CHANNEL_MAX = 13
WIFI_AP_MAX_CONNECTIONS_MIN = 1
WIFI_AP_MAX_CONNECTIONS_MAX = 10
WIFI_AP_BEACON_INTERVAL_MIN_TU = 100
WIFI_AP_BEACON_INTERVAL_MAX_TU = 60000
PROGRAMMER_RESET_PULSE_MIN_MS = 1
PROGRAMMER_RESET_PULSE_MAX_MS = 1000
PROGRAMMER_BOOT_SETTLE_MIN_MS = 1
PROGRAMMER_BOOT_SETTLE_MAX_MS = 1000
PROGRAMMER_SYNC_TIMEOUT_MIN_MS = 1
PROGRAMMER_SYNC_TIMEOUT_MAX_MS = 5000
PROGRAMMER_SYNC_RETRIES_MIN = 1
PROGRAMMER_SYNC_RETRIES_MAX = 100
PROGRAMMER_STM32_BLOCK_BYTES = 256
PROGRAMMER_ESP32_VERIFY_CHUNK_MIN_BYTES = 1
PROGRAMMER_ESP32_VERIFY_CHUNK_MAX_BYTES = 32768  # matches the Kconfig staging-buffer max
DISPLAY_PANEL_I2C_CLOCK_MIN_HZ = 10000
DISPLAY_PANEL_I2C_CLOCK_MAX_HZ = 1000000
DISPLAY_PANEL_I2C_TIMEOUT_MIN_MS = 1
DISPLAY_PANEL_I2C_TIMEOUT_MAX_MS = 1000
DISPLAY_PANEL_I2C_SCL_WAIT_MIN_US = 0
DISPLAY_PANEL_I2C_SCL_WAIT_MAX_US = 200000
DISPLAY_PANEL_I2C_GLITCH_IGNORE_CNT_MIN = 0
DISPLAY_PANEL_I2C_GLITCH_IGNORE_CNT_MAX = 15
DISPLAY_PANEL_SETTLE_TIME_MIN_MS = 0
DISPLAY_PANEL_SETTLE_TIME_MAX_MS = 1000
DISPLAY_MANAGER_FPS_CAP_MIN = 1
DISPLAY_MANAGER_FPS_CAP_MAX = 60
TONE_PLAYER_VOLUME_MIN = 0
TONE_PLAYER_VOLUME_MAX = 10
TCP_SERVER_PORT_MIN = 1
TCP_SERVER_PORT_MAX = 65535
TCP_SERVER_BACKLOG_MIN = 1
TCP_SERVER_BACKLOG_MAX = 8
TCP_SERVER_KEEPALIVE_IDLE_MIN_S = 0
TCP_SERVER_KEEPALIVE_IDLE_MAX_S = 3600
TCP_SERVER_KEEPALIVE_INTERVAL_MIN_S = 1
TCP_SERVER_KEEPALIVE_INTERVAL_MAX_S = 3600
TCP_SERVER_KEEPALIVE_COUNT_MIN = 1
TCP_SERVER_KEEPALIVE_COUNT_MAX = 20
UDP_SERVER_UPLOAD_CAP_MIN_KBITS = 0
UDP_SERVER_UPLOAD_CAP_MAX_KBITS = 100000
UDP_SERVER_DOWNLOAD_CAP_MIN_KBITS = 0
UDP_SERVER_DOWNLOAD_CAP_MAX_KBITS = 100000
UDP_SERVER_OVERFLOW_THRESHOLD_MIN_PACKETS = 1
UDP_SERVER_OVERFLOW_THRESHOLD_MAX_PACKETS = 1000
UDP_SERVER_SHAPER_BUFFER_WINDOW_MS = 300
UDP_SERVER_SHAPER_BUFFER_MAX_BYTES = 32768
WIDGET_BOOT_TIMEOUT_MIN_S = 0
WIDGET_BOOT_TIMEOUT_MAX_S = 15
WIDGET_UI_TIMEOUT_MIN_S = 0
WIDGET_UI_TIMEOUT_MAX_S = 255

FCLINK_UART_BAUD_RATE_CHOICES = {
    "ESP32_FCLINK_UART_BAUD_9600": "9600",
    "ESP32_FCLINK_UART_BAUD_19200": "19200",
    "ESP32_FCLINK_UART_BAUD_38400": "38400",
    "ESP32_FCLINK_UART_BAUD_57600": "57600",
    "ESP32_FCLINK_UART_BAUD_115200": "115200",
    "ESP32_FCLINK_UART_BAUD_230400": "230400",
    "ESP32_FCLINK_UART_BAUD_460800": "460800",
    "ESP32_FCLINK_UART_BAUD_921600": "921600",
    "ESP32_FCLINK_UART_BAUD_1000000": "1000000",
    "ESP32_FCLINK_UART_BAUD_2000000": "2000000",
    "ESP32_FCLINK_UART_BAUD_5000000": "5000000",
}

FCLINK_UART_PARITY_CHOICES = {
    "ESP32_FCLINK_UART_PARITY_NONE": "UartParity::kNone",
    "ESP32_FCLINK_UART_PARITY_EVEN": "UartParity::kEven",
    "ESP32_FCLINK_UART_PARITY_ODD": "UartParity::kOdd",
}

TELEM_UART_BAUD_RATE_CHOICES = {
    "ESP32_TELEM_UART_BAUD_9600": "9600",
    "ESP32_TELEM_UART_BAUD_19200": "19200",
    "ESP32_TELEM_UART_BAUD_38400": "38400",
    "ESP32_TELEM_UART_BAUD_57600": "57600",
    "ESP32_TELEM_UART_BAUD_115200": "115200",
    "ESP32_TELEM_UART_BAUD_230400": "230400",
    "ESP32_TELEM_UART_BAUD_460800": "460800",
}

WIFI_POWER_SAVE_CHOICES = {
    "ESP32_WIFI_POWER_SAVE_NONE": "WIFI_PS_NONE",
    "ESP32_WIFI_POWER_SAVE_MIN_MODEM": "WIFI_PS_MIN_MODEM",
    "ESP32_WIFI_POWER_SAVE_MAX_MODEM": "WIFI_PS_MAX_MODEM",
}

UI_TRANSITION_SPEED_CHOICES = {
    "ESP32_WIDGET_UI_TRANSITION_SPEED_1X": "1",
    "ESP32_WIDGET_UI_TRANSITION_SPEED_2X": "2",
    "ESP32_WIDGET_UI_TRANSITION_SPEED_3X": "3",
}

def _git_head_short_hash() -> str:
    try:
        result = subprocess.run(
            ["git", "rev-parse", "--short=8", "HEAD"],
            cwd=REPO_ROOT,
            check=True,
            capture_output=True,
            text=True,
        )
        value = result.stdout.strip().lower()
    except (OSError, subprocess.CalledProcessError):
        return "00000000"

    if len(value) < 8 or any(ch not in "0123456789abcdef" for ch in value[:8]):
        return "00000000"
    return value[:8]


def _mavlink_flight_sw_version_from_version_string(version_string: str) -> int:
    parts = version_string.strip().split(".")
    if len(parts) != 3:
        raise SystemExit(
            f"unexpected firmware version format '{version_string}'; expected X.Y.Z"
        )

    try:
        major = int(parts[0], 10)
        minor = int(parts[1], 10)
        patch = int(parts[2], 10)
    except ValueError as exc:
        raise SystemExit(
            f"unexpected firmware version format '{version_string}'; expected X.Y.Z"
        ) from exc

    if not 0 <= major <= 0xFF:
        raise SystemExit("firmware major version must fit in one byte")
    if not 0 <= minor <= 0xFF:
        raise SystemExit("firmware minor version must fit in one byte")
    if not 0 <= patch <= 0xFF:
        raise SystemExit("firmware patch version must fit in one byte")

    return (
        (major << 24)
        | (minor << 16)
        | (patch << 8)
        | MAVLINK_FIRMWARE_VERSION_TYPE_OFFICIAL
    )


def _firmware_version_string() -> str:
    version_string = resolve_firmware_version().strip()
    if not version_string:
        raise SystemExit("firmware version tool returned an empty version string")
    return version_string


def _validate_gpio_num(kconf: kconfiglib.Kconfig, name: str) -> None:
    gpio_num = sym_int(kconf, name)
    if not ESP32C3_GPIO_MIN <= gpio_num <= ESP32C3_GPIO_MAX:
        raise ValueError(
            f"{name} must be an ESP32-C3 GPIO in the range "
            f"{ESP32C3_GPIO_MIN}..{ESP32C3_GPIO_MAX}"
        )


def _validate_int_range(
    kconf: kconfiglib.Kconfig, name: str, minimum: int, maximum: int
) -> None:
    value = sym_int(kconf, name)
    if not minimum <= value <= maximum:
        raise ValueError(f"{name} must be in the range {minimum}..{maximum}")


def _udp_shaper_buffer_bytes(cap_kbits: int) -> int:
    if cap_kbits <= 0:
        return 0
    # window_ms at cap_kbits resolves to (cap_kbits * window_ms) bits.
    window_bytes = (cap_kbits * UDP_SERVER_SHAPER_BUFFER_WINDOW_MS + 7) // 8
    return min(max(window_bytes, 1), UDP_SERVER_SHAPER_BUFFER_MAX_BYTES)


def _validate_unique_gpio_assignments(
    kconf: kconfiglib.Kconfig, names: tuple[str, ...]
) -> None:
    assigned: dict[int, str] = {}
    duplicates: list[str] = []

    for name in names:
        gpio_num = sym_int(kconf, name)
        previous = assigned.get(gpio_num)
        if previous is None:
            assigned[gpio_num] = name
            continue
        duplicates.append(f"GPIO{gpio_num}: {previous}, {name}")

    if duplicates:
        raise ValueError(
            "duplicate ESP32 pin assignments are not allowed: " + "; ".join(duplicates)
        )


# ---- Validation tables ---------------------------------------------------
# Each tuple is (Kconfig symbol, min, max). The shared range constants live
# at the top of this file. Adding a new int symbol = one new line.

PROGRAMMER_STAGING_BUFFER_MIN_BYTES = 1024
PROGRAMMER_STAGING_BUFFER_MAX_BYTES = 32768
TCP_SERVER_EVENT_QUEUE_DEPTH_MIN = 2
TCP_SERVER_EVENT_QUEUE_DEPTH_MAX = 32
TCP_SERVER_DOWNLOAD_BUFFER_MIN_BYTES = 1024
TCP_SERVER_DOWNLOAD_BUFFER_MAX_BYTES = 32768
MAVLINK_SYS_AUTOSTART_CUSTOM_MIN = 1
MAVLINK_SYS_AUTOSTART_CUSTOM_MAX = 1000000


_INT_RANGES: tuple[tuple[str, int, int], ...] = (
    ("STM32_FCLINK_TELEMETRY_RATE_HZ", FCLINK_TELEMETRY_RATE_MIN, FCLINK_TELEMETRY_RATE_MAX),
    ("ESP32_FCLINK_INVALID_PACKET_THRESHOLD", FCLINK_INVALID_PACKET_THRESHOLD_MIN, FCLINK_INVALID_PACKET_THRESHOLD_MAX),
    ("ESP32_FCLINK_HANDSHAKE_ATTEMPTS", FCLINK_HANDSHAKE_ATTEMPTS_MIN, FCLINK_HANDSHAKE_ATTEMPTS_MAX),
    ("ESP32_FCLINK_HANDSHAKE_RETRY_PERIOD_MS", FCLINK_HANDSHAKE_RETRY_PERIOD_MIN_MS, FCLINK_HANDSHAKE_RETRY_PERIOD_MAX_MS),
    ("ESP32_FCLINK_RX_QUEUE_DEPTH", FCLINK_RX_QUEUE_DEPTH_MIN, FCLINK_RX_QUEUE_DEPTH_MAX),
    ("ESP32_FCLINK_UART_RX_BUFFER_SIZE", UART_BUFFER_SIZE_MIN, UART_BUFFER_SIZE_MAX),
    ("ESP32_FCLINK_UART_TX_BUFFER_SIZE", UART_BUFFER_SIZE_MIN, UART_BUFFER_SIZE_MAX),
    ("ESP32_MAVLINK_IDENTITY_SYSID", MAVLINK_SYSID_MIN, MAVLINK_SYSID_MAX),
    ("ESP32_MAVLINK_IDENTITY_COMPID", MAVLINK_COMPID_MIN, MAVLINK_COMPID_MAX),
    ("ESP32_MAVLINK_TX_PERIODS_HB_MS", MAVLINK_HEARTBEAT_PERIOD_MIN_MS, MAVLINK_HEARTBEAT_PERIOD_MAX_MS),
    ("ESP32_MAVLINK_TX_PERIODS_GPS_MS", MAVLINK_TX_PERIOD_MIN_MS, MAVLINK_TX_PERIOD_MAX_MS),
    ("ESP32_MAVLINK_TX_PERIODS_ATT_MS", MAVLINK_TX_PERIOD_MIN_MS, MAVLINK_TX_PERIOD_MAX_MS),
    ("ESP32_MAVLINK_TX_PERIODS_GPOS_MS", MAVLINK_TX_PERIOD_MIN_MS, MAVLINK_TX_PERIOD_MAX_MS),
    ("ESP32_MAVLINK_TX_PERIODS_BATT_MS", MAVLINK_TX_PERIOD_MIN_MS, MAVLINK_TX_PERIOD_MAX_MS),
    ("ESP32_MAVLINK_TX_SCHEDULE_HB_DEADLINE_MS", MAVLINK_HEARTBEAT_DEADLINE_MIN_MS, MAVLINK_HEARTBEAT_DEADLINE_MAX_MS),
    ("ESP32_MAVLINK_TX_SCHEDULE_GPS_START_DELAY_MS", MAVLINK_TX_SCHEDULE_START_DELAY_MIN_MS, MAVLINK_TX_SCHEDULE_START_DELAY_MAX_MS),
    ("ESP32_MAVLINK_TX_SCHEDULE_ATT_START_DELAY_MS", MAVLINK_TX_SCHEDULE_START_DELAY_MIN_MS, MAVLINK_TX_SCHEDULE_START_DELAY_MAX_MS),
    ("ESP32_MAVLINK_TX_SCHEDULE_GPOS_START_DELAY_MS", MAVLINK_TX_SCHEDULE_START_DELAY_MIN_MS, MAVLINK_TX_SCHEDULE_START_DELAY_MAX_MS),
    ("ESP32_MAVLINK_TX_SCHEDULE_BATT_START_DELAY_MS", MAVLINK_TX_SCHEDULE_START_DELAY_MIN_MS, MAVLINK_TX_SCHEDULE_START_DELAY_MAX_MS),
    ("ESP32_WIFI_AP_CHANNEL", WIFI_AP_CHANNEL_MIN, WIFI_AP_CHANNEL_MAX),
    ("ESP32_WIFI_AP_MAX_CONNECTIONS", WIFI_AP_MAX_CONNECTIONS_MIN, WIFI_AP_MAX_CONNECTIONS_MAX),
    ("ESP32_WIFI_AP_BEACON_INTERVAL_TU", WIFI_AP_BEACON_INTERVAL_MIN_TU, WIFI_AP_BEACON_INTERVAL_MAX_TU),
    ("ESP32_PROGRAMMER_RESET_PULSE_MS", PROGRAMMER_RESET_PULSE_MIN_MS, PROGRAMMER_RESET_PULSE_MAX_MS),
    ("ESP32_PROGRAMMER_BOOT_SETTLE_MS", PROGRAMMER_BOOT_SETTLE_MIN_MS, PROGRAMMER_BOOT_SETTLE_MAX_MS),
    ("ESP32_PROGRAMMER_SYNC_TIMEOUT_MS", PROGRAMMER_SYNC_TIMEOUT_MIN_MS, PROGRAMMER_SYNC_TIMEOUT_MAX_MS),
    ("ESP32_PROGRAMMER_SYNC_RETRIES", PROGRAMMER_SYNC_RETRIES_MIN, PROGRAMMER_SYNC_RETRIES_MAX),
    ("ESP32_PROGRAMMER_VERIFY_ESP32_CHUNK_BYTES", PROGRAMMER_ESP32_VERIFY_CHUNK_MIN_BYTES, PROGRAMMER_ESP32_VERIFY_CHUNK_MAX_BYTES),
    ("ESP32_PROGRAMMER_STAGING_BUFFER_BYTES", PROGRAMMER_STAGING_BUFFER_MIN_BYTES, PROGRAMMER_STAGING_BUFFER_MAX_BYTES),
    ("ESP32_TCP_SERVER_EVENT_QUEUE_DEPTH", TCP_SERVER_EVENT_QUEUE_DEPTH_MIN, TCP_SERVER_EVENT_QUEUE_DEPTH_MAX),
    ("ESP32_TCP_SERVER_DOWNLOAD_BUFFER_BYTES", TCP_SERVER_DOWNLOAD_BUFFER_MIN_BYTES, TCP_SERVER_DOWNLOAD_BUFFER_MAX_BYTES),
    ("ESP32_DISPLAY_PANEL_I2C_CLOCK_HZ", DISPLAY_PANEL_I2C_CLOCK_MIN_HZ, DISPLAY_PANEL_I2C_CLOCK_MAX_HZ),
    ("ESP32_DISPLAY_PANEL_I2C_TIMEOUT_MS", DISPLAY_PANEL_I2C_TIMEOUT_MIN_MS, DISPLAY_PANEL_I2C_TIMEOUT_MAX_MS),
    ("ESP32_DISPLAY_PANEL_I2C_SCL_WAIT_US", DISPLAY_PANEL_I2C_SCL_WAIT_MIN_US, DISPLAY_PANEL_I2C_SCL_WAIT_MAX_US),
    ("ESP32_DISPLAY_PANEL_I2C_GLITCH_IGNORE_CNT", DISPLAY_PANEL_I2C_GLITCH_IGNORE_CNT_MIN, DISPLAY_PANEL_I2C_GLITCH_IGNORE_CNT_MAX),
    ("ESP32_DISPLAY_PANEL_SETTLE_TIME_MS", DISPLAY_PANEL_SETTLE_TIME_MIN_MS, DISPLAY_PANEL_SETTLE_TIME_MAX_MS),
    ("ESP32_DISPLAY_MANAGER_FPS_CAP", DISPLAY_MANAGER_FPS_CAP_MIN, DISPLAY_MANAGER_FPS_CAP_MAX),
    ("ESP32_TONE_PLAYER_VOLUME", TONE_PLAYER_VOLUME_MIN, TONE_PLAYER_VOLUME_MAX),
    ("ESP32_TCP_SERVER_CTRL_PORT", TCP_SERVER_PORT_MIN, TCP_SERVER_PORT_MAX),
    ("ESP32_TCP_SERVER_DATA_PORT", TCP_SERVER_PORT_MIN, TCP_SERVER_PORT_MAX),
    ("ESP32_TCP_SERVER_BACKLOG", TCP_SERVER_BACKLOG_MIN, TCP_SERVER_BACKLOG_MAX),
    ("ESP32_TCP_SERVER_KEEPALIVE_IDLE_S", TCP_SERVER_KEEPALIVE_IDLE_MIN_S, TCP_SERVER_KEEPALIVE_IDLE_MAX_S),
    ("ESP32_TCP_SERVER_KEEPALIVE_INTERVAL_S", TCP_SERVER_KEEPALIVE_INTERVAL_MIN_S, TCP_SERVER_KEEPALIVE_INTERVAL_MAX_S),
    ("ESP32_TCP_SERVER_KEEPALIVE_COUNT", TCP_SERVER_KEEPALIVE_COUNT_MIN, TCP_SERVER_KEEPALIVE_COUNT_MAX),
    ("ESP32_UDP_SERVER_PORT", TCP_SERVER_PORT_MIN, TCP_SERVER_PORT_MAX),
    ("ESP32_UDP_SERVER_UPLOAD_CAP_KBITS", UDP_SERVER_UPLOAD_CAP_MIN_KBITS, UDP_SERVER_UPLOAD_CAP_MAX_KBITS),
    ("ESP32_UDP_SERVER_OVERFLOW_THRESHOLD", UDP_SERVER_OVERFLOW_THRESHOLD_MIN_PACKETS, UDP_SERVER_OVERFLOW_THRESHOLD_MAX_PACKETS),
    ("ESP32_UDP_SERVER_DOWNLOAD_CAP_KBITS", UDP_SERVER_DOWNLOAD_CAP_MIN_KBITS, UDP_SERVER_DOWNLOAD_CAP_MAX_KBITS),
    ("ESP32_WIDGET_BOOT_TIMEOUT_S", WIDGET_BOOT_TIMEOUT_MIN_S, WIDGET_BOOT_TIMEOUT_MAX_S),
    ("ESP32_WIDGET_UI_TIMEOUT_S", WIDGET_UI_TIMEOUT_MIN_S, WIDGET_UI_TIMEOUT_MAX_S),
)


def _validate_pin_assignments(kconf: kconfiglib.Kconfig) -> None:
    gpio_symbols = (
        "ESP32_PINMAP_LED_GPIO_NUM",
        "ESP32_PINMAP_BUTTON_GPIO_NUM",
        "ESP32_PINMAP_BUZZER_GPIO_NUM",
        "ESP32_PINMAP_FCLINK_UART_TX_GPIO_NUM",
        "ESP32_PINMAP_FCLINK_UART_RX_GPIO_NUM",
        "ESP32_PINMAP_PROGRAMMER_BOOT0_GPIO_NUM",
        "ESP32_PINMAP_PROGRAMMER_NRST_GPIO_NUM",
    )
    if sym_bool(kconf, "ESP32_DISPLAY_PANEL_ENABLE"):
        gpio_symbols = gpio_symbols + (
            "ESP32_PINMAP_SSD1306_PANEL_I2C_SDA_GPIO_NUM",
            "ESP32_PINMAP_SSD1306_PANEL_I2C_SCL_GPIO_NUM",
        )
    for symbol in gpio_symbols:
        _validate_gpio_num(kconf, symbol)
    _validate_unique_gpio_assignments(kconf, gpio_symbols)


def _validate_wifi(kconf: kconfiglib.Kconfig) -> None:
    wifi_ssid = sym_str(kconf, "ESP32_WIFI_AP_SSID")
    wifi_password = sym_str(kconf, "ESP32_WIFI_AP_PASSWORD")
    if not WIFI_AP_SSID_MIN_LEN <= len(wifi_ssid) <= WIFI_AP_SSID_MAX_LEN:
        raise ValueError(
            "ESP32_WIFI_AP_SSID must be between "
            f"{WIFI_AP_SSID_MIN_LEN} and {WIFI_AP_SSID_MAX_LEN} characters"
        )
    if len(wifi_password) > WIFI_AP_PASSWORD_MAX_LEN:
        raise ValueError(
            f"ESP32_WIFI_AP_PASSWORD must be at most {WIFI_AP_PASSWORD_MAX_LEN} characters"
        )
    if 0 < len(wifi_password) < WIFI_AP_PASSWORD_MIN_LEN:
        raise ValueError(
            "ESP32_WIFI_AP_PASSWORD must be empty or at least "
            f"{WIFI_AP_PASSWORD_MIN_LEN} characters"
        )
    if sym_int(kconf, "ESP32_WIFI_AP_BEACON_INTERVAL_TU") % 100 != 0:
        raise ValueError("ESP32_WIFI_AP_BEACON_INTERVAL_TU must be a multiple of 100")
    if sym_bool(kconf, "ESP32_WIFI_PMF_REQUIRED") and not sym_bool(
        kconf, "ESP32_WIFI_PMF_CAPABLE"
    ):
        raise ValueError("ESP32_WIFI_PMF_REQUIRED requires ESP32_WIFI_PMF_CAPABLE")


def _validate_cross_field(kconf: kconfiglib.Kconfig) -> None:
    if sym_int(kconf, "ESP32_TCP_SERVER_CTRL_PORT") == sym_int(
        kconf, "ESP32_TCP_SERVER_DATA_PORT"
    ):
        raise ValueError(
            "ESP32_TCP_SERVER_CTRL_PORT and ESP32_TCP_SERVER_DATA_PORT must differ"
        )
    if sym_bool(kconf, "ESP32_BUTTON_PULLUP") and sym_bool(
        kconf, "ESP32_BUTTON_PULLDOWN"
    ):
        raise ValueError("ESP32 button pull-up and pull-down cannot both be enabled")
    if sym_bool(kconf, "ESP32_PROGRAMMER_VERIFY_ESP32"):
        chunk = sym_int(kconf, "ESP32_PROGRAMMER_VERIFY_ESP32_CHUNK_BYTES")
        staging = sym_int(kconf, "ESP32_PROGRAMMER_STAGING_BUFFER_BYTES")
        if chunk > staging:
            raise ValueError(
                "ESP32_PROGRAMMER_VERIFY_ESP32_CHUNK_BYTES "
                f"({chunk}) must be <= ESP32_PROGRAMMER_STAGING_BUFFER_BYTES "
                f"({staging})"
            )


def _validate(kconf: kconfiglib.Kconfig) -> None:
    _validate_pin_assignments(kconf)
    for name, lo, hi in _INT_RANGES:
        _validate_int_range(kconf, name, lo, hi)
    _validate_wifi(kconf)
    _validate_cross_field(kconf)


# ---- Per-peripheral runtime contexts -------------------------------------
# Each helper resolves Kconfig + chosen-value maps for one peripheral and
# returns the sub-dict used by the matching block in esp32_config.hpp.j2.

def _pin_map_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "led": sym_int(kconf, "ESP32_PINMAP_LED_GPIO_NUM"),
        "button": sym_int(kconf, "ESP32_PINMAP_BUTTON_GPIO_NUM"),
        "buzzer": sym_int(kconf, "ESP32_PINMAP_BUZZER_GPIO_NUM"),
        "ssd1306_panel_i2c_sda": sym_int(
            kconf, "ESP32_PINMAP_SSD1306_PANEL_I2C_SDA_GPIO_NUM"
        ),
        "ssd1306_panel_i2c_scl": sym_int(
            kconf, "ESP32_PINMAP_SSD1306_PANEL_I2C_SCL_GPIO_NUM"
        ),
        "fclink_uart_tx": sym_int(kconf, "ESP32_PINMAP_FCLINK_UART_TX_GPIO_NUM"),
        "fclink_uart_rx": sym_int(kconf, "ESP32_PINMAP_FCLINK_UART_RX_GPIO_NUM"),
        "telem_uart_tx": sym_int(kconf, "ESP32_PINMAP_TELEM_UART_TX_GPIO_NUM"),
        "telem_uart_rx": sym_int(kconf, "ESP32_PINMAP_TELEM_UART_RX_GPIO_NUM"),
        "programmer_boot0": sym_int(kconf, "ESP32_PINMAP_PROGRAMMER_BOOT0_GPIO_NUM"),
        "programmer_nrst": sym_int(kconf, "ESP32_PINMAP_PROGRAMMER_NRST_GPIO_NUM"),
    }


def _button_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "active_low": sym_bool(kconf, "ESP32_BUTTON_ACTIVE_LOW"),
        "pullup": sym_bool(kconf, "ESP32_BUTTON_PULLUP"),
        "pulldown": sym_bool(kconf, "ESP32_BUTTON_PULLDOWN"),
        "debounce_ms": sym_int(kconf, "ESP32_BUTTON_DEBOUNCE_MS"),
        "long_press_ms": sym_int(kconf, "ESP32_BUTTON_LONG_PRESS_MS"),
        "long_long_press_ms": sym_int(kconf, "ESP32_BUTTON_LONG_LONG_PRESS_MS"),
    }


def _display_i2c_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "glitch_ignore_cnt": sym_int(
            kconf, "ESP32_DISPLAY_PANEL_I2C_GLITCH_IGNORE_CNT"
        ),
        "xfer_timeout_ms": sym_int(kconf, "ESP32_DISPLAY_PANEL_I2C_TIMEOUT_MS"),
        "enable_internal_pullup": sym_bool(
            kconf, "ESP32_DISPLAY_PANEL_I2C_ENABLE_INTERNAL_PULLUP"
        ),
    }


def _display_panel_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "enabled": sym_bool(kconf, "ESP32_DISPLAY_PANEL_ENABLE"),
        "settle_time_ms": sym_int(kconf, "ESP32_DISPLAY_PANEL_SETTLE_TIME_MS"),
        "i2c_address": sym_int(kconf, "ESP32_DISPLAY_PANEL_I2C_ADDRESS"),
        "i2c_scl_speed_hz": sym_int(kconf, "ESP32_DISPLAY_PANEL_I2C_CLOCK_HZ"),
        "i2c_scl_wait_us": sym_int(kconf, "ESP32_DISPLAY_PANEL_I2C_SCL_WAIT_US"),
        "invert": sym_bool(kconf, "ESP32_DISPLAY_PANEL_INVERT"),
        "rotate_180": sym_bool(kconf, "ESP32_DISPLAY_PANEL_ROTATE_180"),
    }


def _ui_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "fps_cap": sym_int(kconf, "ESP32_DISPLAY_MANAGER_FPS_CAP"),
        "boot_logo_timeout_s": sym_int(kconf, "ESP32_WIDGET_BOOT_TIMEOUT_S"),
        "ui_timeout_s": sym_int(kconf, "ESP32_WIDGET_UI_TIMEOUT_S"),
        "transition_speed_x": int(choice_value(kconf, UI_TRANSITION_SPEED_CHOICES)),
    }


def _tcp_server_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "ctrl_port": sym_int(kconf, "ESP32_TCP_SERVER_CTRL_PORT"),
        "data_port": sym_int(kconf, "ESP32_TCP_SERVER_DATA_PORT"),
        "backlog": sym_int(kconf, "ESP32_TCP_SERVER_BACKLOG"),
        "nonblocking": sym_bool(kconf, "ESP32_TCP_SERVER_NONBLOCKING"),
        "keepalive_idle_s": sym_int(kconf, "ESP32_TCP_SERVER_KEEPALIVE_IDLE_S"),
        "keepalive_intvl_s": sym_int(kconf, "ESP32_TCP_SERVER_KEEPALIVE_INTERVAL_S"),
        "keepalive_cnt": sym_int(kconf, "ESP32_TCP_SERVER_KEEPALIVE_COUNT"),
    }


def _udp_server_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "port": sym_int(kconf, "ESP32_UDP_SERVER_PORT"),
        "upload_cap_kbits": sym_int(kconf, "ESP32_UDP_SERVER_UPLOAD_CAP_KBITS"),
        "download_cap_kbits": sym_int(kconf, "ESP32_UDP_SERVER_DOWNLOAD_CAP_KBITS"),
        "overflow_threshold": sym_int(kconf, "ESP32_UDP_SERVER_OVERFLOW_THRESHOLD"),
    }


def _usb_cdc_server_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "rx_buffer_bytes": sym_int(
            kconf, "ESP32_USB_CDC_SERVER_RX_BUFFER_BYTES"
        ),
        "tx_buffer_bytes": sym_int(
            kconf, "ESP32_USB_CDC_SERVER_TX_BUFFER_BYTES"
        ),
    }


def _fclink_uart_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "baud_rate": choice_value(kconf, FCLINK_UART_BAUD_RATE_CHOICES),
        "parity": choice_value(kconf, FCLINK_UART_PARITY_CHOICES),
        "rx_buf": sym_int(kconf, "ESP32_FCLINK_UART_RX_BUFFER_SIZE"),
        "tx_buf": sym_int(kconf, "ESP32_FCLINK_UART_TX_BUFFER_SIZE"),
    }


def _telem_uart_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "baud_rate": choice_value(kconf, TELEM_UART_BAUD_RATE_CHOICES),
        "rx_buf": sym_int(kconf, "ESP32_TELEM_UART_RX_BUFFER_SIZE"),
        "tx_buf": sym_int(kconf, "ESP32_TELEM_UART_TX_BUFFER_SIZE"),
    }


def _fclink_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "rx_queue_depth": sym_int(kconf, "ESP32_FCLINK_RX_QUEUE_DEPTH"),
        "handshake_attempts": sym_int(kconf, "ESP32_FCLINK_HANDSHAKE_ATTEMPTS"),
        "handshake_retry_period_ms": sym_int(
            kconf, "ESP32_FCLINK_HANDSHAKE_RETRY_PERIOD_MS"
        ),
        "invalid_packet_threshold": sym_int(
            kconf, "ESP32_FCLINK_INVALID_PACKET_THRESHOLD"
        ),
    }


def _programmer_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "reset_pulse_ms": sym_int(kconf, "ESP32_PROGRAMMER_RESET_PULSE_MS"),
        "boot_settle_ms": sym_int(kconf, "ESP32_PROGRAMMER_BOOT_SETTLE_MS"),
        "sync_timeout_ms": sym_int(kconf, "ESP32_PROGRAMMER_SYNC_TIMEOUT_MS"),
        "sync_retries": sym_int(kconf, "ESP32_PROGRAMMER_SYNC_RETRIES"),
        "verify": {
            "esp32": sym_bool(kconf, "ESP32_PROGRAMMER_VERIFY_ESP32"),
            "esp32_chunk_bytes": sym_int(
                kconf, "ESP32_PROGRAMMER_VERIFY_ESP32_CHUNK_BYTES"
            ),
            "stm32": sym_bool(kconf, "ESP32_PROGRAMMER_VERIFY_STM32"),
        },
    }


def _wifi_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "ap": {
            "ssid": cpp_string_literal(sym_str(kconf, "ESP32_WIFI_AP_SSID")),
            "password": cpp_string_literal(sym_str(kconf, "ESP32_WIFI_AP_PASSWORD")),
            "channel": sym_int(kconf, "ESP32_WIFI_AP_CHANNEL"),
            "max_connections": sym_int(kconf, "ESP32_WIFI_AP_MAX_CONNECTIONS"),
            "beacon_interval_tu": sym_int(kconf, "ESP32_WIFI_AP_BEACON_INTERVAL_TU"),
            "hidden": sym_bool(kconf, "ESP32_WIFI_AP_HIDDEN"),
        },
        "pmf": {
            "capable": sym_bool(kconf, "ESP32_WIFI_PMF_CAPABLE"),
            "required": sym_bool(kconf, "ESP32_WIFI_PMF_REQUIRED"),
        },
        "power_save": choice_value(kconf, WIFI_POWER_SAVE_CHOICES),
    }


def _resolve_mavlink_sys_autostart(kconf: kconfiglib.Kconfig) -> int:
    if sym_bool(kconf, "ESP32_MAVLINK_SYS_AUTOSTART_CUSTOM"):
        return sym_int(kconf, "ESP32_MAVLINK_SYS_AUTOSTART_CUSTOM_VALUE")
    for sym, value in MAVLINK_SYS_AUTOSTART_CHOICES.items():
        if sym_bool(kconf, sym):
            return value
    raise ValueError("no ESP32_MAVLINK_SYS_AUTOSTART_* choice selected")


def _mavlink_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    git_hash = _git_head_short_hash()
    firmware_version_string = _firmware_version_string()
    return {
        "identity": {
            "sysid": sym_int(kconf, "ESP32_MAVLINK_IDENTITY_SYSID"),
            "compid": sym_int(kconf, "ESP32_MAVLINK_IDENTITY_COMPID"),
        },
        "sys_autostart": _resolve_mavlink_sys_autostart(kconf),
        "system_status_fresh_ms": MAVLINK_SYSTEM_STATUS_FRESH_MS,
        "git_hash": git_hash,
        "version_string": firmware_version_string,
        "flight_sw_version_hex": (
            f"0x{_mavlink_flight_sw_version_from_version_string(firmware_version_string):08X}u"
        ),
        "tx": {
            "periods": {
                "hb_ms": sym_int(kconf, "ESP32_MAVLINK_TX_PERIODS_HB_MS"),
                "gps_ms": sym_int(kconf, "ESP32_MAVLINK_TX_PERIODS_GPS_MS"),
                "att_ms": sym_int(kconf, "ESP32_MAVLINK_TX_PERIODS_ATT_MS"),
                "gpos_ms": sym_int(kconf, "ESP32_MAVLINK_TX_PERIODS_GPOS_MS"),
                "batt_ms": sym_int(kconf, "ESP32_MAVLINK_TX_PERIODS_BATT_MS"),
                "rc_ms": sym_int(kconf, "ESP32_MAVLINK_TX_PERIODS_RC_MS"),
            },
            "schedule": {
                "hb_deadline_ms": sym_int(
                    kconf, "ESP32_MAVLINK_TX_SCHEDULE_HB_DEADLINE_MS"
                ),
                "gps_start_delay_ms": sym_int(
                    kconf, "ESP32_MAVLINK_TX_SCHEDULE_GPS_START_DELAY_MS"
                ),
                "att_start_delay_ms": sym_int(
                    kconf, "ESP32_MAVLINK_TX_SCHEDULE_ATT_START_DELAY_MS"
                ),
                "gpos_start_delay_ms": sym_int(
                    kconf, "ESP32_MAVLINK_TX_SCHEDULE_GPOS_START_DELAY_MS"
                ),
                "batt_start_delay_ms": sym_int(
                    kconf, "ESP32_MAVLINK_TX_SCHEDULE_BATT_START_DELAY_MS"
                ),
                "rc_start_delay_ms": sym_int(
                    kconf, "ESP32_MAVLINK_TX_SCHEDULE_RC_START_DELAY_MS"
                ),
            },
        },
    }


def _runtime_context(
    source: pathlib.Path, kconf: kconfiglib.Kconfig
) -> dict[str, object]:
    return {
        "autogen_warning": autogen_warning(source),
        "pin_map": _pin_map_context(kconf),
        "button": _button_context(kconf),
        "led": {"active_low": sym_bool(kconf, "ESP32_LED_ACTIVE_LOW")},
        "buzzer": {"active_low": sym_bool(kconf, "ESP32_BUZZER_ACTIVE_LOW")},
        "display_i2c": _display_i2c_context(kconf),
        "display_panel": _display_panel_context(kconf),
        "ui": _ui_context(kconf),
        "tone_player": {"volume": sym_int(kconf, "ESP32_TONE_PLAYER_VOLUME")},
        "tcp_server": _tcp_server_context(kconf),
        "udp_server": _udp_server_context(kconf),
        "usb_cdc_server": _usb_cdc_server_context(kconf),
        "fclink_uart": _fclink_uart_context(kconf),
        "telem_uart": _telem_uart_context(kconf),
        "fclink": _fclink_context(kconf),
        "programmer": _programmer_context(kconf),
        "wifi": _wifi_context(kconf),
        "mavlink": _mavlink_context(kconf),
        "panic": {
            "task_stack_depth_words": PANIC_TASK_STACK_DEPTH_WORDS,
            "task_priority": PANIC_TASK_PRIORITY,
        },
    }


def _limits_context(
    source: pathlib.Path, kconf: kconfiglib.Kconfig
) -> dict[str, object]:
    return {
        "autogen_warning": autogen_warning(source),
        "display_panel": {
            "width": DISPLAY_PANEL_WIDTH_PX,
            "height": DISPLAY_PANEL_HEIGHT_PX,
            "controller_width": DISPLAY_PANEL_CONTROLLER_WIDTH_PX,
            "column_offset": DISPLAY_PANEL_COLUMN_OFFSET_PX,
        },
        "fclink": {
            "rx_queue_depth": sym_int(kconf, "ESP32_FCLINK_RX_QUEUE_DEPTH"),
        },
        "programmer": {
            "staging_buffer_bytes": sym_int(
                kconf, "ESP32_PROGRAMMER_STAGING_BUFFER_BYTES"
            ),
            "stm32_block_bytes": PROGRAMMER_STM32_BLOCK_BYTES,
        },
        "tcp_server": {
            "event_queue_depth": sym_int(
                kconf, "ESP32_TCP_SERVER_EVENT_QUEUE_DEPTH"
            ),
            "download_buffer_bytes": sym_int(
                kconf, "ESP32_TCP_SERVER_DOWNLOAD_BUFFER_BYTES"
            ),
            "max_line_bytes": TCP_SERVER_MAX_LINE_BYTES,
        },
        "udp_server": {
            "upload_buffer_bytes": _udp_shaper_buffer_bytes(
                sym_int(kconf, "ESP32_UDP_SERVER_UPLOAD_CAP_KBITS")
            ),
            "download_buffer_bytes": _udp_shaper_buffer_bytes(
                sym_int(kconf, "ESP32_UDP_SERVER_DOWNLOAD_CAP_KBITS")
            ),
        },
        "tone_player": {
            "pending_request_queue_depth": TONE_PLAYER_PENDING_REQUEST_QUEUE_DEPTH,
        },
    }


def main() -> int:
    return run_generator(
        validate_fn=_validate,
        runtime_context_fn=_runtime_context,
        limits_context_fn=_limits_context,
        runtime_template_name="esp32_config.hpp.j2",
        limits_template_name="esp32_limits.hpp.j2",
    )


if __name__ == "__main__":
    raise SystemExit(main())
