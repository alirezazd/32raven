#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi

"""Run the clang-tidy checks that find defects rather than opinions.

clang-tidy's default set is mostly style, and on this codebase style checks
alone produce ~19000 findings that contradict deliberate choices -- literal
suffix case, identifier length, magic numbers. None of that says a program is
wrong. The set below is the subset that does: analyzer paths, and the bugprone
and performance checks whose findings are defects.

Every exclusion is a domain judgement, recorded with its reason. Bare-metal
firmware breaks several of clang-tidy's assumptions outright -- a peripheral
register really is a dereference of a fixed address, and a register-mirror enum
really does need the width the hardware field has.

Shares its clang-tidy plumbing with check_unused_includes.py; see
scripts/clang_tidy_env.py for why the compile flags need filtering.

Run:
  uv run --quiet --script scripts/lint/check_tidy.py [files...]
"""
from __future__ import annotations

import concurrent.futures
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile

# Sibling module — clang_tidy_env.py lives one directory up, next to the
# generators that share it.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))
from clang_tidy_env import (  # noqa: E402
    FIRST_PARTY,
    REPO,
    SKIP,
    databases,
    filtered_database,
    first_party_targets,
    toolchain_args,
)

# Checks whose findings would be true of correct firmware, with the reason each
# does not apply. Dropping one of these means accepting the noise it brings.
EXCLUDED = {
    "performance-enum-size": (
        "register-mirror enums take the width of the hardware field they are "
        "written to; narrowing them would need a cast at every register write"
    ),
    "clang-analyzer-core.FixedAddressDereference": (
        "every peripheral access on bare metal is a dereference of a fixed "
        "address -- that is what a memory-mapped register is"
    ),
    "bugprone-dynamic-static-initializers": (
        "the function-local `static Foo instance` singleton is this codebase's "
        "deliberate pattern for driver instances"
    ),
    "bugprone-easily-swappable-parameters": (
        "would require a distinct wrapper type per argument across every driver"
    ),
    "bugprone-narrowing-conversions": (
        "int to float in sensor scaling, where the integer range is bounded by "
        "the ADC or register width"
    ),
    "clang-analyzer-optin.performance.Padding": (
        "layout advice for types that exist once"
    ),
    "clang-analyzer-optin.core.EnumCastOutOfRange": (
        "ESP-IDF's flag enums enumerate single bits, so any combination is a "
        "value no enumerator names -- ESP_NETIF_INHERENT_DEFAULT_WIFI_AP builds "
        "DHCP_SERVER | FLAG_AUTOUP and casts it back itself"
    ),
    "bugprone-reserved-identifier": (
        "ST's header guards and the linker's __wrap_ convention are fixed by "
        "things outside this repo"
    ),
    "performance-no-int-to-ptr": (
        "peripheral base addresses arrive as integers from CMSIS"
    ),
    "bugprone-invalid-enum-default-initialization": (
        "every finding is an ESP-IDF struct value-initialised before its fields "
        "are filled in, which is how that API is meant to be used; the enum "
        "holds 0 only between the brace and the assignment below it"
    ),
}

# Checks named one at a time because the module they live in is mostly style.
# Each of these finds a defect: a cycle the stack bound cannot measure through,
# a header definition that breaks the one-definition rule, an assignment
# operator that cannot chain, a const that binds to the wrong side of a
# pointer, a call that is not safe to make from more than one task, and an
# arithmetic precedence a reader has to hold in their head.
NAMED = (
    "misc-no-recursion",
    "misc-definitions-in-headers",
    "misc-unconventional-assign-operator",
    "misc-misplaced-const",
    "concurrency-mt-unsafe",
    "readability-math-missing-parentheses",
)

CHECKS = ",".join(
    ["-*", "bugprone-*", "clang-analyzer-*", "performance-*", *NAMED]
    + [f"-{name}" for name in sorted(EXCLUDED)]
)

DIAG_RE = re.compile(
    r"^(?P<file>[^:]+):(?P<line>\d+):(?P<col>\d+): warning: "
    r"(?P<msg>.*?) \[(?P<check>[\w\-,.]+)\]$"
)


def run(
    path: pathlib.Path, build_dir: pathlib.Path, compiler: str
) -> tuple[list[str], str | None]:
    extra = [f"--extra-arg={a}" for a in toolchain_args(compiler)]
    proc = subprocess.run(
        [
            "clang-tidy",
            f"--checks={CHECKS}",
            "--quiet",
            *extra,
            "-p",
            str(build_dir),
            str(path),
        ],
        capture_output=True,
        text=True,
        cwd=REPO,
    )
    blob = proc.stderr + proc.stdout
    if "clang-diagnostic-error" in blob:
        first = next(
            line.strip() for line in blob.split("\n") if "clang-diagnostic-error" in line
        )
        return [], first

    findings = []
    for line in proc.stdout.split("\n"):
        match = DIAG_RE.match(line.strip())
        if not match:
            continue
        try:
            rel = pathlib.Path(match["file"]).resolve().relative_to(REPO).as_posix()
        except ValueError:
            continue
        # A diagnostic can land in a generated or vendored header the TU pulled
        # in; only our own files are ours to fix.
        if not FIRST_PARTY.match(rel) or SKIP.match(rel):
            continue
        findings.append(
            f"{rel}:{match['line']}: [{match['check']}] {match['msg']}"
        )
    return findings, None


def main() -> int:
    if shutil.which("clang-tidy") is None:
        print("check_tidy: clang-tidy not installed, skipping.")
        return 0

    if not databases():
        print("check_tidy: no compilation database, skipping. Build first.")
        return 0

    wanted = (
        [pathlib.Path(a).resolve() for a in sys.argv[1:]] if len(sys.argv) > 1 else None
    )
    targets = first_party_targets(wanted)
    if not targets:
        return 0

    findings: list[str] = []
    unparsed: list[tuple[str, str]] = []
    with tempfile.TemporaryDirectory() as tmp:
        cache: dict[pathlib.Path, pathlib.Path] = {}
        prepared = []
        for path, build_dir, compiler in targets:
            if build_dir not in cache:
                cache[build_dir] = filtered_database(build_dir, pathlib.Path(tmp))
            prepared.append((path, cache[build_dir], compiler))

        with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
            for (path, _, _), (found, error) in zip(
                prepared, pool.map(lambda t: run(*t), prepared)
            ):
                if error is not None:
                    unparsed.append((path.relative_to(REPO).as_posix(), error))
                findings += found

    if unparsed:
        print(
            f"{len(unparsed)}/{len(targets)} files did not parse, so they were "
            "not checked:",
            file=sys.stderr,
        )
        for rel, error in sorted(unparsed):
            print(f"  {rel}\n    {error}", file=sys.stderr)
        return 1

    if findings:
        print("clang-tidy:", file=sys.stderr)
        for finding in sorted(set(findings)):
            print(f"  {finding}", file=sys.stderr)
        print(
            "\nFix them, or add `// NOLINT(check-name)` with a comment saying why\n"
            "the check does not apply here. If a whole check is wrong for this\n"
            "codebase, retire it in EXCLUDED with its reason instead.",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
