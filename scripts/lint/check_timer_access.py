#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi

"""Forbid reading the timebase timer directly instead of through TimeBase.

TIM2 is the 1 MHz free-running counter TimeBase owns. Reading it through
`TIM2->CNT` works, which is exactly the problem: hand-rolled `Micros()` helpers
and busy-wait loops accumulate, each one a copy of `TimeBase::Micros` or
`TimeBase::DelayMicros` that nothing keeps in step with the driver.

Use `System::GetInstance().Time()` and call `Micros()` or `DelayMicros()`.

The allowlist below is deliberately short. Adding to it means arguing that the
driver cannot be reached from that context, not that it is inconvenient.
"""
from __future__ import annotations

import pathlib
import re
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent.parent

# Each entry needs a reason, because "it was easier" is not one.
ALLOWED = {
    # Owns the timer.
    "stm32/Drivers/Src/time_base.cpp",
    "stm32/Drivers/Inc/time_base.hpp",
    # Runs after a fault, when System may be the thing that faulted.
    "stm32/Core/Src/panic.cpp",
    # Bit-banged UART: several reads per bit with interrupts masked.
    "stm32/Drivers/Src/uart_soft.cpp",
}

PATTERN = re.compile(r"\bTIM2\s*->\s*CNT\b")


def main() -> int:
    paths = [pathlib.Path(p) for p in sys.argv[1:]]
    if not paths:
        paths = sorted((REPO / "stm32").rglob("*.[ch]pp"))

    findings: list[str] = []
    for path in paths:
        try:
            rel = path.resolve().relative_to(REPO).as_posix()
        except ValueError:
            continue
        if rel in ALLOWED or not rel.startswith("stm32/"):
            continue
        try:
            text = path.read_text(encoding="utf-8")
        except (OSError, UnicodeDecodeError):
            continue
        for number, line in enumerate(text.splitlines(), start=1):
            if PATTERN.search(line):
                findings.append(f"{rel}:{number}: {line.strip()}")

    if findings:
        print("Direct TIM2 access outside TimeBase:", file=sys.stderr)
        for finding in findings:
            print(f"  {finding}", file=sys.stderr)
        print(
            "\nUse System::GetInstance().Time().Micros() / .DelayMicros().\n"
            "If the driver genuinely cannot be reached there, add the file to\n"
            "ALLOWED in scripts/lint/check_timer_access.py with the reason.",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
