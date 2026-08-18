#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi

"""A singleton's instance lives where its GetInstance body is compiled.

An inline body in a header emits the `static X instance;` initialization
decision as a COMDAT in every including TU, and any TU that can see a
constexpr-eligible constructor may constant-initialize it: one non-zero member
drags the whole object, zero-filled buffers included, out of .bss into .data,
and flash stores the zeros. That cost 17 KB.

So both firmwares declare `static X &GetInstance();` in the header and define
it in the .cpp. Class templates are exempt: their body has nowhere else to go.
"""

from __future__ import annotations

import pathlib
import re
import subprocess
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent.parent

# Each entry needs a reason.
ALLOWED = {
    # Kicked from the panic loop when nothing else -- System included -- is
    # trusted, and Kick() must stay a single inlined register write. Trivial
    # class, no buffers, so the .data trap cannot bite it.
    "stm32/Drivers/watchdog.hpp",
}

DEFINITION = re.compile(r"static\s+[\w:]+\s*&\s*GetInstance\(\)\s*\{")
CLASS_LINE = re.compile(r"^\s*(?:class|struct)\s+\w+")
TEMPLATE_LINE = re.compile(r"^\s*template\s*<")


def enclosing_class_is_template(lines: list[str], hit: int) -> bool:
    depth = 0
    for i in range(hit, -1, -1):
        line = lines[i]
        depth += line.count("}") - line.count("{")
        if depth <= 0 and CLASS_LINE.match(line):
            # The template head may span lines; stop at the previous
            # statement boundary.
            for j in range(i - 1, -1, -1):
                stripped = lines[j].strip()
                if not stripped or stripped.startswith("//"):
                    continue
                if TEMPLATE_LINE.match(lines[j]):
                    return True
                if stripped.endswith((";", "{", "}")):
                    return False
            return False
    return False


def main(argv: list[str]) -> int:
    names = argv[1:]
    if not names:
        # CI runs with no arguments and expects a whole-tree walk.
        out = subprocess.run(
            ["git", "ls-files", "*.h", "*.hpp"],
            cwd=REPO,
            capture_output=True,
            text=True,
            check=True,
        )
        names = [str(REPO / line) for line in out.stdout.splitlines()]

    findings: list[str] = []
    for name in names:
        path = pathlib.Path(name)
        if path.suffix not in (".h", ".hpp") or "third_party" in path.parts:
            continue
        rel = path.resolve().relative_to(REPO).as_posix()
        if rel in ALLOWED:
            continue
        lines = path.read_text(encoding="utf-8", errors="replace").splitlines()
        for idx, line in enumerate(lines):
            if not DEFINITION.search(line):
                continue
            if enclosing_class_is_template(lines, idx):
                continue
            findings.append(f"{name}:{idx + 1}: inline GetInstance body")

    if findings:
        for finding in findings:
            print(f"  {finding}", file=sys.stderr)
        print(
            "\nDeclare `static X &GetInstance();` in the header and define it\n"
            "in the .cpp. An inline body lets any including TU constant-\n"
            "initialize the instance; one non-zero member then moves the\n"
            "whole object into .data and flash stores its zero buffers.\n"
            "Class templates are exempt: their body has nowhere else to go.",
            file=sys.stderr,
        )

    return 1 if findings else 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
