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
    return {"fclink": fclink_context(kconf)}


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
