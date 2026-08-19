#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi

"""Constructs this firmware may not contain, and where they hide.

Most rules are ones a build gate has to enforce because the compiler will not:
no heap, no double, no container that allocates. The build fails on them rather
than a hygiene hook flagging them, because each one changes what the artifact
does -- a `double` on this FPU is software-emulated, a `malloc` is a failure
mode with no policy behind it.

A rule may also carry a `scope`, which makes it a statement about layering
rather than about the artifact: the construct is fine in general and wrong in
these files. That is not an exception. An exception excuses a violation after
the fact; a scope says the rule never reached there, and it is declared in this
table where a reviewer reads it rather than in a suppression file.

Matching runs over source with comments and string literals blanked out first.
The shell version this replaced dropped any line containing `//`, which meant a
trailing comment hid every violation on that line -- a linter that a comment
defeats reports success it has not earned. Blanking preserves line and column
numbers, so what is reported still points at the real position.
"""

from __future__ import annotations

import argparse
import os
import pathlib
import re
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent.parent

SOURCE_SUFFIXES = (".cpp", ".hpp")
# .docker and .venv are gitignored local caches holding a whole C++ toolchain
# and its standard library, so a bare run finds <iostream> in someone else's
# headers and fails for anyone who has built through Docker. CI never has them.
SKIP_DIRS = frozenset(
    {".git", "build", "cmake", "third_party", ".docker", ".venv"}
)

FORBIDDEN_HEADERS = (
    "iostream",
    "sstream",
    "locale",
    "regex",
    "vector",
    "list",
    "map",
    "set",
    "unordered_map",
    "unordered_set",
)
FREERTOS_DYNAMIC_APIS = (
    "xTaskCreate",
    "xTaskCreatePinnedToCore",
    "xQueueCreate",
    "xSemaphoreCreateBinary",
    "xSemaphoreCreateCounting",
    "xSemaphoreCreateMutex",
    "xSemaphoreCreateRecursiveMutex",
    "xTimerCreate",
    "xEventGroupCreate",
    "xStreamBufferCreate",
    "xMessageBufferCreate",
    "pvPortMalloc",
)
LIBC_ALLOC_APIS = (
    "malloc",
    "calloc",
    "realloc",
    "free",
    "aligned_alloc",
    "strdup",
    "strndup",
)


class Rule:
    """One forbidden construct, optionally narrowed to the files it governs.

    `scope` holds repo-relative paths; empty means the whole tree.
    """

    def __init__(
        self,
        name: str,
        pattern: str,
        reason: str,
        scope: tuple[str, ...] = (),
    ) -> None:
        self.name = name
        self.regex = re.compile(pattern)
        self.reason = reason
        self.scope = scope

    def covers(self, rel: str) -> bool:
        return not self.scope or rel in self.scope


# The two state machines sequence; they do not decide what is fatal. A halt
# belongs to whatever owns the condition -- the driver that detected it, or
# Sentinel, the only thing positioned to weigh it against the armed state.
# Panicking from the sequencer also reaches for whatever error code is nearest
# instead of one that names a domain, which is how a board ends up halting on
# a code that says nothing about what broke.
STATE_MACHINES = (
    "stm32/Core/states.cpp",
    "esp32/main/states.cpp",
)


RULES = (
    Rule(
        "header",
        r"#include\s*<(?:" + "|".join(FORBIDDEN_HEADERS) + r")>",
        "allocating or locale-aware standard header",
    ),
    # Placement new takes its address in parentheses and allocates nothing, so
    # it is the one form that stays.
    Rule("new", r"\bnew\b(?!\s*\()", "no heap -- allocate statically"),
    # A word-boundary match covers the cast forms too: static_cast<double> and
    # (double) are both `double` at a boundary, so they need no rule of their
    # own. The FPU is single-precision (-mfpu=fpv4-sp-d16), which makes every
    # one of them a software-emulated conversion.
    Rule("double", r"\bdouble\b", "single-precision FPU -- use float"),
    Rule(
        "freertos-dynamic",
        r"\b(?:" + "|".join(FREERTOS_DYNAMIC_APIS) + r")\s*\(",
        "dynamic FreeRTOS object -- use the static Create variant",
    ),
    Rule(
        "libc-alloc",
        r"\b(?:" + "|".join(LIBC_ALLOC_APIS) + r")\s*\(",
        "no heap -- allocate statically",
    ),
    Rule(
        "state-panic",
        r"\bPanic\s*\(",
        "state machines sequence -- halt where the condition is owned",
        scope=STATE_MACHINES,
    ),
)


def check_scopes() -> list[str]:
    """A scope naming a path that is gone is a rule that quietly stopped.

    Renaming a scoped file would otherwise disable its rule with nothing said,
    which is the failure mode that makes suppression files rot.
    """
    return [
        f"[{rule.name}] scope names a path that does not exist: {rel}"
        for rule in RULES
        for rel in rule.scope
        if not (REPO / rel).is_file()
    ]


def blank_comments_and_strings(src: str) -> str:
    """Replace comment and literal bodies with spaces, keeping every offset.

    Newlines inside block comments and raw strings are kept so line numbers do
    not shift; everything else becomes a space so a match cannot start inside
    what was quoted out.
    """
    out: list[str] = []
    i, n = 0, len(src)
    while i < n:
        ch = src[i]
        nxt = src[i + 1] if i + 1 < n else ""

        if ch == "/" and nxt == "/":
            while i < n and src[i] != "\n":
                out.append(" ")
                i += 1
            continue

        if ch == "/" and nxt == "*":
            out.append("  ")
            i += 2
            while i < n and not (
                src[i] == "*" and i + 1 < n and src[i + 1] == "/"
            ):
                out.append("\n" if src[i] == "\n" else " ")
                i += 1
            out.append("  ")
            i = min(i + 2, n)
            continue

        # Raw string: R"delim( ... )delim", where backslashes are literal and
        # the only terminator is the matching delimiter.
        raw = re.match(r'(?:u8|u|U|L)?R"([^()\\ ]{0,16})\(', src[i:])
        if raw:
            close = ")" + raw.group(1) + '"'
            end = src.find(close, i + raw.end())
            end = n if end == -1 else end + len(close)
            for j in range(i, end):
                out.append("\n" if src[j] == "\n" else " ")
            i = end
            continue

        if ch in "\"'":
            quote = ch
            out.append(" ")
            i += 1
            while i < n and src[i] != quote:
                if src[i] == "\\" and i + 1 < n:
                    out.append("  ")
                    i += 2
                    continue
                out.append("\n" if src[i] == "\n" else " ")
                i += 1
            if i < n:
                out.append(" ")
                i += 1
            continue

        out.append(ch)
        i += 1

    return "".join(out)


def load_exceptions(path: pathlib.Path | None) -> list[str]:
    """Substrings that excuse a match, one per line.

    A blank line is dropped rather than kept: as a substring it matches every
    path, which in the shell version silently disabled every rule.
    """
    if path is None or not path.is_file():
        return []
    return [
        line.strip()
        for line in path.read_text(
            encoding="utf-8", errors="ignore"
        ).splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]


def iter_sources(targets: list[pathlib.Path]):
    """Walk with the skip list applied to directories, not to their contents.

    Pruning at the directory matters more than it looks: third_party carries a
    22 MB vendored Eigen, so filtering paths after the walk costs a stat on
    every header in it.
    """
    for target in targets:
        if target.is_file():
            if target.suffix in SOURCE_SUFFIXES:
                yield target
            continue
        for root, dirs, files in os.walk(target):
            dirs[:] = sorted(d for d in dirs if d not in SKIP_DIRS)
            for name in sorted(files):
                if name.endswith(SOURCE_SUFFIXES):
                    yield pathlib.Path(root) / name


def scan(path: pathlib.Path, exceptions: list[str]) -> list[str]:
    try:
        shown = path.relative_to(REPO)
    except ValueError:
        shown = path

    rules = [rule for rule in RULES if rule.covers(shown.as_posix())]
    if not rules:
        return []

    try:
        src = path.read_text(encoding="utf-8", errors="ignore")
    except OSError as exc:
        return [f"{shown}: unreadable ({exc})"]

    findings: list[str] = []
    lines = blank_comments_and_strings(src).splitlines()
    raw_lines = src.splitlines()

    for number, line in enumerate(lines, start=1):
        for rule in rules:
            for match in rule.regex.finditer(line):
                text = (
                    raw_lines[number - 1].strip()
                    if number <= len(raw_lines)
                    else ""
                )
                location = f"{shown}:{number}:{match.start() + 1}"
                if any(exc in location or exc in text for exc in exceptions):
                    continue
                findings.append(
                    f"{location}: [{rule.name}] {rule.reason}\n      {text}"
                )
    return findings


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--exceptions",
        type=pathlib.Path,
        default=REPO / "scripts/lint/forbidden_exceptions.txt",
        help="file of substrings that excuse a match (missing file means none)",
    )
    parser.add_argument(
        "paths",
        nargs="*",
        type=pathlib.Path,
        help="files or directories to scan (default: the whole repo)",
    )
    args = parser.parse_args()

    stale = check_scopes()
    if stale:
        print("Forbidden rule table is stale:", file=sys.stderr)
        for entry in stale:
            print(f"  {entry}", file=sys.stderr)
        return 1

    targets = args.paths or [REPO]
    exceptions = load_exceptions(args.exceptions)

    findings: list[str] = []
    for source in iter_sources(targets):
        findings.extend(scan(source, exceptions))

    if not findings:
        return 0

    print("Forbidden constructs:", file=sys.stderr)
    for finding in findings:
        print(f"  {finding}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
