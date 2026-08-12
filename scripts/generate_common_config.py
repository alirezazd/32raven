#!/usr/bin/env python3

# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi

# /// script
# dependencies = [
#     "jinja2",
#     "kconfiglib",
# ]
# ///

"""Generate libs/inc/common_config.hpp, included by both firmwares.

Run (normally invoked by the top-level CMake, which owns this output):
  uv run --quiet --script scripts/generate_common_config.py \
      --kconfig config/Kconfig --config config/32raven.config \
      --out libs/inc/common_config.hpp
"""

from __future__ import annotations

import argparse
import pathlib
import sys

import kconfiglib

# Sibling module.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from kconfig_gen import (  # noqa: E402
    autogen_warning,
    choice_value,
    render_template,
    sym_int,
    template_env,
)

TEMPLATE_NAME = "common_config.hpp.j2"

FCLINK_BAUD_CHOICES = {
    f"COMMON_FCLINK_BAUD_{rate}": str(rate)
    for rate in (
        9600,
        19200,
        38400,
        57600,
        115200,
        230400,
        460800,
        921600,
        1000000,
        2000000,
        5000000,
    )
}


def _validate(kconf: kconfiglib.Kconfig) -> None:
    """Reject a link framing the STM32's ROM bootloader will not answer.

    The programmer reaches that bootloader over the FcLink UART, switching the
    baud and carrying the parity through, and AN3155 specifies it as 8E1. So
    the runtime parity is what a DFU session speaks -- and the STM32 is flashed
    over this path, making a wrong setting the one you would have to flash to
    undo. Checked here because the symbol is shared by both boards.
    """
    if not kconf.syms["COMMON_FCLINK_UART_PARITY_EVEN"].tri_value:
        raise ValueError(
            "CONFIG_COMMON_FCLINK_UART_PARITY has to be Even: the programmer "
            "reuses the FcLink UART to reach the STM32's ROM bootloader, which "
            "AN3155 specifies as 8E1, and Uart::SetBaudRate carries the parity "
            "into the DFU session unchanged. None or Odd leaves the bootloader "
            "silent and takes over-the-air flashing with it. Give the "
            "programmer its own framing first if you want to move this."
        )


def fclink_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    exchange_interval_ms = sym_int(kconf, "COMMON_FCLINK_EXCHANGE_INTERVAL_MS")
    return {
        "baud": choice_value(kconf, FCLINK_BAUD_CHOICES),
        "exchange_interval_ms": exchange_interval_ms,
        # Derived, so the window can never be shorter than the renewal cadence.
        "peer_timeout_ms": exchange_interval_ms
        * sym_int(kconf, "COMMON_FCLINK_PEER_TIMEOUT_EXCHANGES"),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--kconfig", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    kconfig_path = pathlib.Path(args.kconfig).resolve()
    config_path = pathlib.Path(args.config).resolve()
    out_path = pathlib.Path(args.out).resolve()

    kconf = kconfiglib.Kconfig(str(kconfig_path))
    if config_path.exists():
        kconf.load_config(str(config_path))

    _validate(kconf)

    text = render_template(
        template_env(),
        TEMPLATE_NAME,
        {
            "autogen_warning": autogen_warning(config_path),
            "fclink": fclink_context(kconf),
        },
    )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(text, encoding="ascii")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
