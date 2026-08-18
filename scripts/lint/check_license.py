#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi

"""Verify every first-party source file carries the project's licence header.

The header is two lines, in the file's own comment syntax:

    SPDX-License-Identifier: GPL-3.0-only
    Copyright (C) <year> Alireza Azadi

Files are discovered with `git ls-files`, so untracked scratch files and
anything in .gitignore are out of scope by construction, and the walk never
descends into build directories.

Third-party and machine-generated files are exempt via EXEMPT_PATTERNS below.
Those are not ours to licence: vendored shims keep their upstream terms, and a
generated file would just get its header rewritten by the next generator run.

`--fix` inserts a missing header (after any shebang and, for Python, before the
PEP 723 metadata block). It never edits a header that is already present, so a
deliberate year or holder is safe from it.

Run:
  uv run --quiet --script scripts/lint/check_license.py
  uv run --quiet --script scripts/lint/check_license.py --fix
"""

from __future__ import annotations

import argparse
import datetime
import fnmatch
import pathlib
import re
import subprocess
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent.parent

SPDX_ID = "GPL-3.0-only"
COPYRIGHT_HOLDER = "Alireza Azadi"

# Comment syntax per extension, as (prefix, suffix). An extension absent here is
# simply not checked, which is what keeps binary assets, Markdown and JSON out
# of scope. Jinja templates need their own delimiters: a `#` line is not a
# comment to Jinja and would be copied verbatim into the C++ it generates.
COMMENT_SYNTAX = {
    ".c": ("//", ""),
    ".h": ("//", ""),
    ".cpp": ("//", ""),
    ".hpp": ("//", ""),
    ".py": ("#", ""),
    ".sh": ("#", ""),
    ".j2": ("{#", " #}"),
}

# Not ours to licence. Each entry needs a reason — an unexplained exemption is
# indistinguishable from a file someone forgot to add a header to.
EXEMPT_PATTERNS = (
    # Vendored upstream sources, under their own licences.
    "third_party/*",
    "stm32/lib/*",
    "esp32/ui/adafruit_gfx_compat/*",
    # ST/CubeMX-generated startup and system files, kept close to upstream so
    # they can be regenerated and diffed against a fresh CubeMX export.
    "stm32/Core/stm32f4xx_it.c",
    "stm32/Core/stm32f4xx_it.h",
    "stm32/Core/syscalls.c",
    "stm32/Core/sysmem.c",
    "stm32/Core/system_stm32f4xx.c",
    # Machine-generated: the generator owns the contents, header included.
    "esp32/ui/assets/bitmap/*",
    "stm32/Drivers/stm32_config.hpp",
    "stm32/Drivers/stm32_limits.hpp",
    "stm32/Drivers/ee_schema.hpp",
    "esp32/main/esp32_config.hpp",
    "esp32/main/esp32_limits.hpp",
    "libs/common_config.hpp",
)

# Header lines are matched, not compared, so reformatting or a different year
# does not read as a missing licence.
SPDX_RE = re.compile(r"SPDX-License-Identifier:\s*(?P<id>\S+)")
COPYRIGHT_RE = re.compile(
    r"Copyright\s+\(C\)\s+(?P<year>\d{4})\s+(?P<holder>.+?)\s*(?:#\}|\*/)?\s*$"
)

# How far into the file to look. Generous enough for a shebang plus a PEP 723
# block, tight enough that a licence buried mid-file still counts as missing.
SEARCH_LINES = 20


def tracked_files() -> list[pathlib.Path]:
    out = subprocess.run(
        ["git", "ls-files", "-z"],
        cwd=REPO,
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    return [REPO / p for p in out.split("\0") if p]


def is_exempt(rel: str) -> bool:
    return any(fnmatch.fnmatch(rel, pat) for pat in EXEMPT_PATTERNS)


def check(path: pathlib.Path) -> str | None:
    """Return a diagnostic, or None when the header is present and correct."""
    try:
        head = path.read_text(encoding="utf-8", errors="replace").splitlines()[
            :SEARCH_LINES
        ]
    except OSError as exc:
        return f"unreadable: {exc}"

    spdx = next((m for m in (SPDX_RE.search(ln) for ln in head) if m), None)
    copyright_ = next(
        (m for m in (COPYRIGHT_RE.search(ln) for ln in head) if m), None
    )

    if spdx is None and copyright_ is None:
        return "no licence header"
    if spdx is None:
        return "missing SPDX-License-Identifier line"
    if copyright_ is None:
        return "missing Copyright line"
    if spdx.group("id") != SPDX_ID:
        return f"SPDX id is {spdx.group('id')!r}, expected {SPDX_ID!r}"
    if copyright_.group("holder") != COPYRIGHT_HOLDER:
        return (
            f"copyright holder is {copyright_.group('holder')!r}, "
            f"expected {COPYRIGHT_HOLDER!r}"
        )
    return None


def insert_header(path: pathlib.Path, syntax: tuple[str, str]) -> None:
    text = path.read_text(encoding="utf-8")
    # Preserve the file's existing line ending; rewriting CRLF to LF would bury
    # a two-line change in a whole-file diff.
    newline = "\r\n" if "\r\n" in text.split("\n", 1)[0] + "\n" else "\n"
    lines = text.splitlines()

    prefix, suffix = syntax
    year = datetime.date.today().year
    header = [
        f"{prefix} SPDX-License-Identifier: {SPDX_ID}{suffix}",
        f"{prefix} Copyright (C) {year} {COPYRIGHT_HOLDER}{suffix}",
        "",
    ]

    # A shebang must stay on line 1 to keep working.
    at = 1 if lines and lines[0].startswith("#!") else 0
    if at == 1 and len(lines) > 1 and lines[1].strip() == "":
        at = 2

    path.write_text(
        newline.join(lines[:at] + header + lines[at:]) + newline,
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--fix",
        action="store_true",
        help="insert headers that are entirely missing",
    )
    args = parser.parse_args()

    checked = 0
    failures: list[tuple[str, str]] = []
    fixed: list[str] = []

    for path in tracked_files():
        rel = path.relative_to(REPO).as_posix()
        syntax = COMMENT_SYNTAX.get(path.suffix)
        if syntax is None or is_exempt(rel) or not path.is_file():
            continue

        checked += 1
        problem = check(path)
        if problem is None:
            continue

        # Only a wholly absent header is safe to synthesise. A malformed or
        # outdated one may carry a deliberate year or a second holder, so it is
        # reported for a human instead.
        if args.fix and problem == "no licence header":
            insert_header(path, syntax)
            fixed.append(rel)
            continue
        failures.append((rel, problem))

    for rel in fixed:
        print(f"added header: {rel}")
    for rel, problem in failures:
        print(f"{rel}: {problem}", file=sys.stderr)

    if failures:
        print(
            f"\nLicense lint: {len(failures)} of {checked} file(s) failed.\n"
            f"Add the two-line header, or exempt the file in EXEMPT_PATTERNS "
            f"(with a reason) if it is vendored or generated.",
            file=sys.stderr,
        )
        return 1

    suffix = f", {len(fixed)} fixed" if fixed else ""
    print(
        f"License lint: {checked} files checked, all headers present{suffix}."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
