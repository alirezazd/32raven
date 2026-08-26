#!/usr/bin/env python3

# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi

# /// script
# dependencies = [
#     "jinja2",
#     "kconfiglib",
# ]
# ///

"""Generate common_config.hpp, included by both firmwares.

The top-level CMake owns this output and writes it under the binary dir, not
the source tree. Naming a path inside libs/ puts a stale copy ahead of the
generated one on the include path, since libs/ is searched first.

Run:
  uv run --quiet --script scripts/generate_common_config.py \
      --kconfig config/Kconfig --config config/32raven.config \
      --out build/Ninja/generated/common_config.hpp
"""

from __future__ import annotations

import argparse
import pathlib
import sys

import kconfiglib

# Sibling module.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from kconfig_gen import (
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


# Keyed on the frame, so a new one cannot arrive with half its facts. The
# enumerator name is the geometry the mixer selects; sys_autostart is the PX4
# airframe the ground station is told, and the two must describe one aircraft.
AIRFRAMES = {
    "COMMON_AIRFRAME_QUAD_X": {
        "name": "kQuadX",
        "motor_count": 4,
        "sys_autostart": 4001,
    },
    "COMMON_AIRFRAME_QUAD_PLUS": {
        "name": "kQuadPlus",
        "motor_count": 4,
        "sys_autostart": 5001,
    },
}


def airframe_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    selected = choice_value(
        kconf, {sym: sym for sym in AIRFRAMES}
    )
    frame = AIRFRAMES[selected]
    return {
        "names": [f["name"] for f in AIRFRAMES.values()],
        "name": frame["name"],
        "motor_count": frame["motor_count"],
        "sys_autostart": frame["sys_autostart"],
    }


def fclink_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    exchange_interval_ms = sym_int(kconf, "COMMON_FCLINK_EXCHANGE_INTERVAL_MS")
    return {
        "baud": choice_value(kconf, FCLINK_BAUD_CHOICES),
        "exchange_interval_ms": exchange_interval_ms,
        # Derived, so the window can never be shorter than the renewal cadence.
        "peer_timeout_ms": exchange_interval_ms
        * sym_int(kconf, "COMMON_FCLINK_PEER_TIMEOUT_EXCHANGES"),
    }


def template_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    """Everything the template reads apart from the autogen banner.

    Separate from main() because check_config_sweep renders this header too,
    and a second hand-built copy of this dict would go stale the next time a
    key is added here.
    """
    return {
        "airframe": airframe_context(kconf),
        "fclink": fclink_context(kconf),
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

    text = render_template(
        template_env(),
        TEMPLATE_NAME,
        {
            "autogen_warning": autogen_warning(config_path),
            **template_context(kconf),
        },
    )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(text, encoding="ascii")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
