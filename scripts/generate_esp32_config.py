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

import pathlib
import subprocess

import kconfiglib
from generate_firmware_ver import resolve_firmware_version
from kconfig_gen import (
    autogen_warning,
    choice_value,
    cpp_string_literal,
    sym,
    sym_bool,
    sym_int,
    sym_str,
)
from kconfig_gen import run as run_generator


REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent

ESP32C3_GPIO_MIN = 0
ESP32C3_GPIO_MAX = 21
MAVLINK_FIRMWARE_VERSION_TYPE_OFFICIAL = 0xFF

# MAVLink runtime metadata (the parts that aren't Kconfig-tunable today).
# `MAVLINK_SYSTEM_STATUS_FRESH_MS` bounds how long a SYS_STATUS sample
# stays current before downstream fields drop out — protocol timing, not
# expected to vary per board.
MAVLINK_SYSTEM_STATUS_FRESH_MS = 3000

# PX4 airframe ID reported as the SYS_AUTOSTART parameter. The heartbeat
# hardcodes MAV_TYPE_QUADROTOR, so any other value here would describe an
# airframe the same link contradicts a second later.

# Panic background task (FreeRTOS).

# SSD1306 panel hardware geometry. Fixed by the chosen 72x40 display module
# and its controller. Everything that follows from these three -- page count,
# framebuffer size, column offset -- is derived in ssd1306_panel.hpp.

# Compile-time buffer / queue sizes used as template parameters. These size
# member arrays inside driver headers, so they have to be constants the
# compiler sees rather than fields of a runtime Config. None of them is a
# tuning choice with a second right answer -- each is the smallest size that
# absorbs the burst it exists for -- so they are fixed here rather than
# exposed as Kconfig knobs nobody could set better.
# Async network events queued while the loop services the previous one.
# Inbound staging for the DFU download; drained a chunk per app tick.
# Parsed FcLink packets held between UART parsing and the app loop.
WIFI_AP_SSID_MIN_LEN = 1
WIFI_AP_SSID_MAX_LEN = 32
WIFI_AP_PASSWORD_MAX_LEN = 63
WIFI_AP_PASSWORD_MIN_LEN = 8
# AN3155 WRITE_MEMORY carries a count byte holding len-1, so 256 is the most a
# single block can express, not a chosen buffer size.
# In-flight bytes received over the wire before they reach the target. 32 STM32
# blocks: enough headroom for a TCP burst, and it must stay above the ESP32
# write chunk, which programmer.hpp static_asserts.
PROGRAMMER_STAGING_BUFFER_BYTES = 8192

# ESP32-C3 LEDC, from soc_caps.h. The peripheral is shared, so the claims below
# are checked against each other the way the pin map is: two consumers on one
# timer means the second to initialise silently retunes the first, and that is
# a configuration error rather than something the drivers can detect.
LEDC_TIMER_COUNT = 4
LEDC_CHANNEL_COUNT = 6
LEDC_MAX_DUTY_RES_BITS = 14
# APB is the fastest source LEDC_AUTO_CLK can pick on this part, and
# ledc_timer_config() fails when freq * 2^resolution exceeds it.
LEDC_MAX_SOURCE_HZ = 80_000_000
# The C3 has no high-speed LEDC bank.
LEDC_SPEED_MODE = "LEDC_LOW_SPEED_MODE"

# Every LEDC claim the firmware makes: (label, timer sym, channel sym,
# duty-resolution sym, carrier frequency in Hz or None when the consumer sets
# its own note by note).
LEDC_CLAIMS = (
    ("LED", "ESP32_LED_LEDC_TIMER", "ESP32_LED_LEDC_CHANNEL",
     "ESP32_LED_LEDC_DUTY_RES_BITS", "ESP32_LED_LEDC_FREQ_HZ"),
    ("buzzer", "ESP32_BUZZER_LEDC_TIMER", "ESP32_BUZZER_LEDC_CHANNEL",
     "ESP32_BUZZER_LEDC_DUTY_RES_BITS", "ESP32_BUZZER_MAX_NOTE_HZ"),
)
UDP_SERVER_SHAPER_BUFFER_WINDOW_MS = 300
UDP_SERVER_SHAPER_BUFFER_MAX_BYTES = 32768

FCLINK_UART_PARITY_CHOICES = {
    "COMMON_FCLINK_UART_PARITY_NONE": "UartParity::kNone",
    "COMMON_FCLINK_UART_PARITY_EVEN": "UartParity::kEven",
    "COMMON_FCLINK_UART_PARITY_ODD": "UartParity::kOdd",
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

def _validate_pin_assignments(kconf: kconfiglib.Kconfig) -> None:
    gpio_symbols = (
        "ESP32_PINMAP_LED_GPIO_NUM",
        "ESP32_PINMAP_BUTTON_GPIO_NUM",
        "ESP32_PINMAP_BUZZER_GPIO_NUM",
        "ESP32_PINMAP_FCLINK_UART_TX_GPIO_NUM",
        "ESP32_PINMAP_FCLINK_UART_RX_GPIO_NUM",
        "ESP32_PINMAP_TELEM_UART_TX_GPIO_NUM",
        "ESP32_PINMAP_TELEM_UART_RX_GPIO_NUM",
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
    # A long press is only recognised once debounce has already elapsed, so a
    # debounce at or above it makes DFU entry unreachable.
    debounce_ms = sym_int(kconf, "ESP32_BUTTON_DEBOUNCE_MS")
    long_press_ms = sym_int(kconf, "ESP32_BUTTON_LONG_PRESS_MS")
    if debounce_ms >= long_press_ms:
        raise ValueError(
            f"ESP32_BUTTON_DEBOUNCE_MS ({debounce_ms}) must be below "
            f"ESP32_BUTTON_LONG_PRESS_MS ({long_press_ms})"
        )
    if sym_bool(kconf, "ESP32_PROGRAMMER_VERIFY_ESP32"):
        chunk = sym_int(kconf, "ESP32_PROGRAMMER_VERIFY_ESP32_CHUNK_BYTES")
        if chunk > PROGRAMMER_STAGING_BUFFER_BYTES:
            raise ValueError(
                "ESP32_PROGRAMMER_VERIFY_ESP32_CHUNK_BYTES "
                f"({chunk}) must be <= the staging buffer "
                f"({PROGRAMMER_STAGING_BUFFER_BYTES})"
            )


def _ledc_claim(kconf: kconfiglib.Kconfig, claim: tuple) -> dict[str, object]:
    label, timer_sym, channel_sym, res_sym, freq_sym = claim
    resolution = sym_int(kconf, res_sym)
    entry = {
        "label": label,
        "timer": sym_int(kconf, timer_sym),
        "channel": sym_int(kconf, channel_sym),
        "duty_resolution": resolution,
        "freq_hz": sym_int(kconf, freq_sym) if freq_sym else None,
    }
    if entry["timer"] >= LEDC_TIMER_COUNT:
        raise ValueError(
            f"{label} claims LEDC timer {entry['timer']}; this SoC has "
            f"{LEDC_TIMER_COUNT}"
        )
    if entry["channel"] >= LEDC_CHANNEL_COUNT:
        raise ValueError(
            f"{label} claims LEDC channel {entry['channel']}; this SoC has "
            f"{LEDC_CHANNEL_COUNT}"
        )
    if resolution > LEDC_MAX_DUTY_RES_BITS:
        raise ValueError(
            f"{label} asks for {resolution} duty bits; this SoC tops out at "
            f"{LEDC_MAX_DUTY_RES_BITS}"
        )
    if entry["freq_hz"] is not None:
        carrier = entry["freq_hz"] << resolution
        if carrier > LEDC_MAX_SOURCE_HZ:
            raise ValueError(
                f"{label} wants {entry['freq_hz']} Hz at {resolution} duty "
                f"bits, which needs a {carrier} Hz LEDC source; the fastest "
                f"this SoC offers is {LEDC_MAX_SOURCE_HZ} Hz"
            )
    return entry


def _ledc_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    claims = [_ledc_claim(kconf, c) for c in LEDC_CLAIMS]
    for field in ("timer", "channel"):
        seen: dict[int, str] = {}
        for claim in claims:
            owner = seen.get(claim[field])
            if owner is not None:
                raise ValueError(
                    f"{claim['label']} and {owner} both claim LEDC {field} "
                    f"{claim[field]}; whichever initialises second retunes the "
                    "first"
                )
            seen[claim[field]] = claim["label"]
    # The label names the claim rather than describing the peripheral, so it
    # stays behind with the checks that quote it.
    return {
        "speed_mode": LEDC_SPEED_MODE,
        **{
            str(claim["label"]).lower(): {
                key: value for key, value in claim.items() if key != "label"
            }
            for claim in claims
        },
    }


def _validate(kconf: kconfiglib.Kconfig) -> None:
    _validate_pin_assignments(kconf)
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
    }


def _display_i2c_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    return {
        "xfer_timeout_ms": sym_int(kconf, "ESP32_DISPLAY_PANEL_I2C_TIMEOUT_MS"),
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
        "handshake_attempts": sym_int(kconf, "ESP32_FCLINK_HANDSHAKE_ATTEMPTS"),
        "handshake_retry_period_ms": sym_int(
            kconf, "ESP32_FCLINK_HANDSHAKE_RETRY_PERIOD_MS"
        ),
        "invalid_packet_threshold": sym_int(
            kconf, "ESP32_FCLINK_INVALID_PACKET_THRESHOLD"
        ),
    }


def _programmer_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    verify_esp32 = sym_bool(kconf, "ESP32_PROGRAMMER_VERIFY_ESP32")
    return {
        "reset_pulse_ms": sym_int(kconf, "ESP32_PROGRAMMER_RESET_PULSE_MS"),
        "boot_settle_ms": sym_int(kconf, "ESP32_PROGRAMMER_BOOT_SETTLE_MS"),
        "sync_timeout_ms": sym_int(kconf, "ESP32_PROGRAMMER_SYNC_TIMEOUT_MS"),
        "sync_retries": sym_int(kconf, "ESP32_PROGRAMMER_SYNC_RETRIES"),
        "verify": {
            "esp32": verify_esp32,
            # The chunk size depends on verification in Kconfig, so it has no
            # value at all when that is off. Nothing reads the field then
            # either: TargetVerifyChunkSize is only reached from the verifying
            # state, which the same flag guards.
            "esp32_chunk_bytes": (
                sym_int(kconf, "ESP32_PROGRAMMER_VERIFY_ESP32_CHUNK_BYTES")
                if verify_esp32
                else 0
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
    }


def _mavlink_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    git_hash = _git_head_short_hash()
    firmware_version_string = _firmware_version_string()
    return {
        "identity": {
            "sysid": sym_int(kconf, "ESP32_MAVLINK_IDENTITY_SYSID"),
            "compid": sym_int(kconf, "ESP32_MAVLINK_IDENTITY_COMPID"),
        },
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
                "esc_ms": sym_int(kconf, "ESP32_MAVLINK_TX_PERIODS_ESC_MS"),
            },
            "schedule": {
                "hb_deadline_ms": sym_int(
                    kconf, "ESP32_MAVLINK_TX_SCHEDULE_HB_DEADLINE_MS"
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
        "ledc": _ledc_context(kconf),
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
    }


def _limits_context(
    source: pathlib.Path, kconf: kconfiglib.Kconfig
) -> dict[str, object]:
    return {
        "autogen_warning": autogen_warning(source),
        "programmer": {
            "staging_buffer_bytes": PROGRAMMER_STAGING_BUFFER_BYTES,
        },
        "udp_server": {
            "upload_buffer_bytes": _udp_shaper_buffer_bytes(
                sym_int(kconf, "ESP32_UDP_SERVER_UPLOAD_CAP_KBITS")
            ),
            "download_buffer_bytes": _udp_shaper_buffer_bytes(
                sym_int(kconf, "ESP32_UDP_SERVER_DOWNLOAD_CAP_KBITS")
            ),
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
