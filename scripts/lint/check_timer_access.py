#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi

"""Two ways to get the timebase wrong: reach past it, or widen what it returns.

TIM2 is the 1 MHz free-running counter TimeBase owns. Reading it through
`TIM2->CNT` works, which is exactly the problem: hand-rolled `Micros()` helpers
and busy-wait loops accumulate, each one a copy of `TimeBase::Micros` or
`TimeBase::DelayMicros` that nothing keeps in step with the driver.

Use `System::GetInstance().Time()` and call `Micros()` or `DelayMicros()`.

The allowlist below is deliberately short. Adding to it means arguing that the
driver cannot be reached from that context, not that it is inconvenient.

The second rule catches `Micros()` being stored in a 64-bit variable. It returns
`TIM2->CNT`, a 32-bit counter that wraps every 71.6 minutes, and every elapsed
comparison in this firmware relies on that wrap cancelling in unsigned 32-bit
arithmetic. Widening zero-extends instead, so the wrap becomes a 4295-second
discontinuity in a type that promises it cannot happen -- and the compiler has
no reason to object. Accumulate short deltas into a 64-bit total if a long
horizon is genuinely needed; do not widen a single read.
"""

from __future__ import annotations

import pathlib
import re
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent.parent

# Each entry needs a reason, because "it was easier" is not one.
ALLOWED = {
    # Owns the timer.
    "stm32/Drivers/time_base.cpp",
    "stm32/Drivers/time_base.hpp",
    # Runs after a fault, when System may be the thing that faulted.
    "stm32/Core/panic.cpp",
    # Bit-banged UART: several reads per bit with interrupts masked.
    "stm32/Drivers/uart_soft.cpp",
}

PATTERN = re.compile(r"\bTIM2\s*->\s*CNT\b")

# `uint64_t x = ... Micros()`, type and assignment on one line. The word
# boundary keeps SecondsToMicros() out, and a genuine 64-bit source would not
# be named Micros().
WIDENING = re.compile(r"\buint64_t\b[^;=]*=[^;]*\bMicros\s*\(\s*\)")

# The declaration half of the split case: `volatile uint64_t last_idle_time_`.
DECL_64 = re.compile(
    r"\buint64_t\b\s*[*&]?\s*(?P<name>[A-Za-z_]\w*)\s*(?:[=;,)[]|$)"
)


# `name = ... Micros()`. Assignment only -- `==` and `>=` widen nothing.
def _assignment_to(name: str) -> re.Pattern[str]:
    return re.compile(
        rf"(?<![=!<>])\b{re.escape(name)}\s*=(?!=)[^;]*\bMicros\s*\(\s*\)"
    )


def _declared_64bit_names() -> set[str]:
    """Every identifier declared uint64_t anywhere under stm32/.

    Tree-wide because the declaring header and the assigning .cpp are
    different files. A name reused at another type can false-positive; that is
    the trade for catching the split case at all.
    """
    names: set[str] = set()
    for path in sorted((REPO / "stm32").rglob("*.[ch]pp")):
        try:
            text = path.read_text(encoding="utf-8")
        except (OSError, UnicodeDecodeError):
            continue
        for match in DECL_64.finditer(text):
            names.add(match["name"])
    return names


def main() -> int:
    paths = [pathlib.Path(p) for p in sys.argv[1:]]
    if not paths:
        paths = sorted((REPO / "stm32").rglob("*.[ch]pp"))

    wide_names = [
        _assignment_to(name) for name in sorted(_declared_64bit_names())
    ]

    findings: list[str] = []
    widened: list[str] = []
    for path in paths:
        try:
            rel = path.resolve().relative_to(REPO).as_posix()
        except ValueError:
            continue
        if not rel.startswith("stm32/"):
            continue
        try:
            text = path.read_text(encoding="utf-8")
        except (OSError, UnicodeDecodeError):
            continue
        for number, line in enumerate(text.splitlines(), start=1):
            if rel not in ALLOWED and PATTERN.search(line):
                findings.append(f"{rel}:{number}: {line.strip()}")
            if WIDENING.search(line) or any(p.search(line) for p in wide_names):
                widened.append(f"{rel}:{number}: {line.strip()}")

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

    if widened:
        print("Micros() widened to 64 bits:", file=sys.stderr)
        for finding in widened:
            print(f"  {finding}", file=sys.stderr)
        print(
            "\nMicros() is a 32-bit counter that wraps every 71.6 minutes.\n"
            "Hold it in a uint32_t and let the wrap cancel, or accumulate\n"
            "short deltas into a 64-bit total -- do not widen a single read.",
            file=sys.stderr,
        )

    return 1 if (findings or widened) else 0


if __name__ == "__main__":
    raise SystemExit(main())
