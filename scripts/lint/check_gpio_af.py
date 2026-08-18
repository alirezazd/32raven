#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi
# /// script
# dependencies = []
# ///

"""Check that each GPIO_AF<n>_<PERIPHERAL> constant equals <n>.

The generator copies the AF *macro name* out of ST's pin data, so a wrong name
is a compile error. The value is the unguarded half: nothing connects
`GPIO_AF12_SDIO` to the number 12, and a wrong index still builds -- it muxes
the pin to whatever peripheral does own that AF, which on PC8-PC12 is a
working-looking board whose SD card never answers.

A lint rather than a static_assert per constant, because the failure is a
*new* constant arriving unchecked -- the missing assert would be the bug.

Run:
  uv run --quiet --script scripts/lint/check_gpio_af.py
"""

from __future__ import annotations

import pathlib
import re
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent.parent
HEADER = REPO / "stm32/Drivers/gpio.hpp"

# inline constexpr uint32_t GPIO_AF12_SDIO = 12u;
DECL = re.compile(
    r"^\s*inline\s+constexpr\s+uint32_t\s+"
    r"GPIO_AF(?P<index>\d+)_(?P<peripheral>\w+)\s*=\s*"
    r"(?P<value>0[xX][0-9a-fA-F]+|\d+)u?\s*;"
)


def main() -> int:
    if not HEADER.exists():
        print(f"{HEADER} not found", file=sys.stderr)
        return 1

    problems: list[str] = []
    seen = 0

    for number, line in enumerate(HEADER.read_text().splitlines(), start=1):
        match = DECL.match(line)
        if not match:
            continue
        seen += 1
        index = int(match["index"])
        value = int(match["value"], 0)
        if value != index:
            rel = HEADER.relative_to(REPO).as_posix()
            problems.append(
                f"{rel}:{number}: GPIO_AF{index}_{match['peripheral']} = "
                f"{match['value']} ({value}), but the name says AF{index}"
            )

    if seen == 0:
        print(
            f"No GPIO_AF constants matched in {HEADER.name}. The declarations "
            "were reformatted or moved, and this check is now silently "
            "passing over nothing -- fix DECL in this script.",
            file=sys.stderr,
        )
        return 1

    if problems:
        print(
            "GPIO alternate-function index does not match its name:",
            file=sys.stderr,
        )
        for problem in problems:
            print(f"  {problem}", file=sys.stderr)
        print(
            "\nThe digits in GPIO_AF<n>_<PERIPHERAL> are ST's AF index "
            "for that\n"
            "signal (RM0090 'Alternate function mapping'). Correct the value,\n"
            "or the name if the constant is for a different AF.",
            file=sys.stderr,
        )
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
