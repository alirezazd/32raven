#!/usr/bin/env python3

# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi

# /// script
# dependencies = [
#     "jinja2",
#     "kconfiglib",
# ]
# ///

"""Generate the config header both firmwares include.

The FC link only works if the two MCUs agree on its exchange cadence and the
timeout derived from it. Emitting those from the per-target generators would
give each side its own constant, in its own units, under its own name -- which
is what this replaces. One header, included by both.

Deliberately not built on kconfig_gen.run(): that runner is fixed to the
runtime/limits output pair, a shape only the per-target generators have.
generate_ee_schema.py sets the precedent for a single-output generator.

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
    render_template,
    sym_int,
    template_env,
)

TEMPLATE_NAME = "common_config.hpp.j2"


def fclink_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    exchange_interval_ms = sym_int(kconf, "COMMON_FCLINK_EXCHANGE_INTERVAL_MS")
    return {
        "exchange_interval_ms": exchange_interval_ms,
        # Expressed as a count of missed exchanges rather than authored directly,
        # so the window can never end up shorter than the cadence the far side
        # renews at -- which would revoke a lease its holder still believed in,
        # mid ESC flash.
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
