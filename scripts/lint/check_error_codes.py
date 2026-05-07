#!/usr/bin/env python3
"""Detect (and optionally remove) unused ErrorCode enumerators.

Default mode (used by CMake at build time): scan `libs/inc/error_code.hpp` for
every value in each nested enum class, count callsites in `stm32/`, `esp32/`,
and `libs/` (excluding `error_code.hpp` itself and `error_code.cpp`), and exit
non-zero with a diagnostic if any enumerator has zero references.

`--fix` mode: do the same scan, then remove dead enumerators from
`error_code.hpp` and the matching `case ...: return "...";` blocks from
`error_code.cpp`, print a list of what was pruned, and exit 0.

Auto-pruning is not the build-time default on purpose: a build that silently
mutates source files breaks reproducibility and would erase a freshly-added
enumerator before its consumer was wired up. Run `--fix` manually when you
want the cleanup applied.
"""
from __future__ import annotations

import argparse
import pathlib
import re
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent.parent
HEADER = REPO / "libs/inc/error_code.hpp"
SOURCE = REPO / "libs/src/error_code.cpp"

SEARCH_ROOTS = [REPO / "stm32", REPO / "esp32", REPO / "libs"]
EXCLUDE_PATHS = {HEADER, SOURCE}


def parse_enum_values(text: str) -> dict[str, list[str]]:
    """Return {domain: [enumerator, ...]} parsed from error_code.hpp."""
    out: dict[str, list[str]] = {}
    for m in re.finditer(
        r"enum class (\w+)\s*:\s*uint32_t\s*\{([^}]*)\}", text, re.DOTALL
    ):
        domain = m.group(1)
        body = m.group(2)
        body = re.sub(r"//[^\n]*", "", body)
        body = re.sub(r"/\*.*?\*/", "", body, flags=re.DOTALL)
        names: list[str] = []
        for entry in body.split(","):
            entry = entry.strip()
            if not entry:
                continue
            ident = entry.split("=", 1)[0].strip()
            if ident.startswith("k"):
                names.append(ident)
        out[domain] = names
    return out


def grep_usage(domain: str, name: str) -> int:
    needle = f"ErrorCode::{domain}::{name}"
    pat = re.compile(re.escape(needle) + r"\b")
    hits = 0
    for root in SEARCH_ROOTS:
        for path in root.rglob("*"):
            if path in EXCLUDE_PATHS:
                continue
            if path.suffix not in (".cpp", ".hpp", ".c", ".h", ".inl"):
                continue
            if any(part == "build" for part in path.parts):
                continue
            try:
                text = path.read_text(encoding="utf-8")
            except (UnicodeDecodeError, OSError):
                continue
            hits += len(pat.findall(text))
    return hits


def find_dead(enums: dict[str, list[str]]) -> list[tuple[str, str]]:
    return [
        (domain, name)
        for domain, names in enums.items()
        for name in names
        if grep_usage(domain, name) == 0
    ]


def prune_header(dead_by_domain: dict[str, list[str]]) -> int:
    """Strip dead enumerator lines from error_code.hpp. Returns count pruned."""
    lines = HEADER.read_text(encoding="utf-8").splitlines()
    out: list[str] = []
    domain: str | None = None
    removed = 0
    for line in lines:
        m = re.match(r"\s*enum class (\w+)\s*:\s*uint32_t", line)
        if m:
            domain = m.group(1)
            out.append(line)
            continue
        if "};" in line and domain:
            domain = None
            out.append(line)
            continue
        if domain and domain in dead_by_domain:
            ms = re.match(r"\s*(\w+)\s*(?:=\s*[^,]+)?,?\s*$", line)
            if ms and ms.group(1) in dead_by_domain[domain]:
                removed += 1
                continue
        out.append(line)
    HEADER.write_text("\n".join(out) + "\n", encoding="utf-8")
    return removed


def prune_source(dead: list[tuple[str, str]]) -> int:
    """Strip matching `case ...: return "...";` pairs from error_code.cpp."""
    if not SOURCE.exists():
        return 0
    lines = SOURCE.read_text(encoding="utf-8").splitlines()
    dead_tokens = {f"ErrorCode::{d}::{n}" for d, n in dead}
    out: list[str] = []
    i = 0
    removed = 0
    while i < len(lines):
        line = lines[i]
        if any(tok + ":" in line for tok in dead_tokens):
            if i + 1 < len(lines) and "return " in lines[i + 1]:
                i += 2
                removed += 1
                continue
        out.append(line)
        i += 1
    SOURCE.write_text("\n".join(out) + "\n", encoding="utf-8")
    return removed


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n", 1)[0])
    ap.add_argument(
        "--fix",
        action="store_true",
        help="remove dead enumerators in-place from header + source, then exit 0",
    )
    args = ap.parse_args()

    if not HEADER.exists():
        print(f"error: {HEADER} not found", file=sys.stderr)
        return 2

    enums = parse_enum_values(HEADER.read_text(encoding="utf-8"))
    if not enums:
        print(f"error: no enum class blocks parsed from {HEADER}", file=sys.stderr)
        return 2

    dead = find_dead(enums)
    total = sum(len(v) for v in enums.values())

    if not dead:
        print(f"ErrorCode lint: {total} enumerators, all referenced.")
        return 0

    print(
        f"check_error_codes: {len(dead)} unused enumerator(s) out of {total}:",
        file=sys.stderr,
    )
    for domain, name in dead:
        print(f"  ErrorCode::{domain}::{name}", file=sys.stderr)

    if args.fix:
        dead_by_domain: dict[str, list[str]] = {}
        for domain, name in dead:
            dead_by_domain.setdefault(domain, []).append(name)
        h = prune_header(dead_by_domain)
        c = prune_source(dead)
        print(
            f"check_error_codes --fix: pruned {h} enumerator line(s) from "
            f"{HEADER.name}, {c} case block(s) from {SOURCE.name}.",
            file=sys.stderr,
        )
        return 0

    print(
        "Run with --fix to remove them automatically, or wire each one up at "
        "a callsite.",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
