#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi

"""Check that numbered TODO markers point at a real roadmap item.

An inline marker is a pointer, never a plan: the detail belongs in
docs/roadmap.md, where it can be prioritised and deleted when delivered. The
tracked form names the item it points at, and the number is what makes the
link checkable:

    // TODO(#15): detect the RC-loss condition here

Catches:
  - Dangling references. A `TODO(#N)` whose number has no `### #N —` entry in
    docs/roadmap.md. That is the same failure `check_docs.py` rejects for a
    `TBD(#N)` in the handbook, and it happens the same way: an item is
    delivered and deleted (numbers are never reused) while a marker naming it
    stays behind, so the code cites a plan that no longer exists.

Untracked markers -- `TODO(crsf)`, `TODO(ui)` -- are legal and counted rather
than rejected, matching how check_docs.py treats a bare `TBD`. The form is
useful for a note too small to earn a roadmap item; counting it is what keeps
a growing pile of them visible.

Marker *shape* is not checked here. `check_comments.py` already rejects a
marker with no parenthesised tag at all, for C++ sources.

Exit codes:
  0  no errors
  1  at least one dangling reference

Run:
  uv run --quiet --script scripts/lint/check_todos.py
"""

from __future__ import annotations

import pathlib
import re
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
ROADMAP = ROOT / "docs" / "roadmap.md"

SCAN_SUFFIXES = {".c", ".h", ".cpp", ".hpp", ".py", ".md", ".cmake", ".sh"}

# These define the marker patterns themselves, so their own text is not a
# marker. Same carve-out check_error_codes.py makes for the enum's two files.
EXCLUDE = {
    "scripts/lint/check_todos.py",
    "scripts/lint/check_comments.py",
    "scripts/lint/check_docs.py",
}

KEYWORDS = r"(?:TODO|FIXME|XXX|HACK)"
# A tracked marker names a roadmap item; the tag is the part in parentheses.
TRACKED = re.compile(rf"\b({KEYWORDS})\(#(\d+)\)")
# Any parenthesised tag, so the untracked ones can be counted.
TAGGED = re.compile(rf"\b{KEYWORDS}\(([^)]*)\)")
# Reused verbatim from check_docs.py -- one roadmap grammar, not two.
ROADMAP_ITEM = re.compile(r"^###\s+#(\d+)\s+—", re.MULTILINE)


def _tracked_items() -> set[int]:
    text = ROADMAP.read_text(encoding="utf-8")
    return {int(n) for n in ROADMAP_ITEM.findall(text)}


def _files() -> list[pathlib.Path]:
    out = subprocess.run(
        ["git", "ls-files"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=True,
    ).stdout.split("\n")
    return [
        ROOT / rel
        for rel in out
        if rel
        and rel not in EXCLUDE
        and not rel.startswith("third_party/")
        and pathlib.Path(rel).suffix in SCAN_SUFFIXES
    ]


def main() -> int:
    items = _tracked_items()
    errors: list[str] = []
    untracked: list[str] = []

    files = _files()
    for path in files:
        try:
            text = path.read_text(encoding="utf-8")
        except (OSError, UnicodeDecodeError):
            continue
        rel = path.relative_to(ROOT)
        for lineno, line in enumerate(text.splitlines(), start=1):
            for keyword, number in TRACKED.findall(line):
                if int(number) not in items:
                    errors.append(
                        f"{rel}:{lineno}: {keyword}(#{number}) has no matching "
                        f"'### #{number} —' in docs/roadmap.md"
                    )
            for tag in TAGGED.findall(line):
                if not tag.startswith("#"):
                    untracked.append(f"{rel}:{lineno}: {tag}")

    if errors:
        for err in errors:
            print(f"error: {err}", file=sys.stderr)
        print(
            f"\ncheck_todos: {len(errors)} dangling reference(s)",
            file=sys.stderr,
        )
        return 1

    print(
        f"check_todos: {len(files)} file(s) OK "
        f"({len(items)} roadmap item(s))"
        + (f"; {len(untracked)} untracked marker(s)" if untracked else "")
    )
    for entry in untracked:
        print(f"  {entry}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
