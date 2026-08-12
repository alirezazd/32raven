#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi

"""Report `#include`s the including file does not use.

Unused includes are invisible to the compiler and to `--gc-sections`: they cost
nothing at runtime, so nothing ever complains, and they accumulate until the
dependency graph in the headers says something quite different from the one the
code actually has.

Runs clang-tidy's misc-include-cleaner against the real compile flags, so the
answer accounts for macros, transitively-required types and the target's own
headers -- a text search cannot do that. Only the "not used directly" half of
the check is reported; its suggestions about includes to ADD are ignored,
because this codebase deliberately leans on a few umbrella headers.

`// IWYU pragma: keep` on the include line suppresses a finding, which is how
headers included for a side effect (linker sections, ISR registration) stay put.

Needs a compilation database, i.e. a build must have happened. Without one, or
without clang-tidy, it skips rather than fails -- the lint CI job installs no
compiler, so a hard failure there would only ever be about the environment. A
file that has a compile command but does not parse is a different matter and
fails the check: an unchecked file is an unknown, and an unknown reported as a
pass is worse than no check at all.

Run:
  uv run --quiet --script scripts/lint/check_unused_includes.py [files...]
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
    REPO,
    SKIP,
    databases,
    filtered_database,
    first_party_targets,
    toolchain_args,
)

FINDING_RE = re.compile(
    r"^(?P<file>[^:]+):(?P<line>\d+):\d+: warning: included header "
    r"(?P<header>\S+) is not used directly"
)


def run(
    path: pathlib.Path, build_dir: pathlib.Path, compiler: str
) -> tuple[list[str], str | None]:
    extra = [f"--extra-arg={a}" for a in toolchain_args(compiler)]
    proc = subprocess.run(
        [
            "clang-tidy",
            "--checks=-*,misc-include-cleaner",
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
    # A TU that did not parse reports every include as unused. Refuse to guess.
    blob = proc.stderr + proc.stdout
    if "error:" in blob:
        first = next(line.strip() for line in blob.split("\n") if "error:" in line)
        return [], first
    findings = []
    for line in proc.stdout.split("\n"):
        match = FINDING_RE.match(line.strip())
        if not match:
            continue
        try:
            rel = pathlib.Path(match["file"]).resolve().relative_to(REPO).as_posix()
        except ValueError:
            continue
        if SKIP.match(rel):
            continue
        findings.append(f"{rel}:{match['line']}: unused include <{match['header']}>")
    return findings, None


def main() -> int:
    if shutil.which("clang-tidy") is None:
        print("check_unused_includes: clang-tidy not installed, skipping.")
        return 0

    if not databases():
        print("check_unused_includes: no compilation database, skipping. Build first.")
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
        print()
        sys.stdout.flush()
        print(
            f"{len(unparsed)}/{len(targets)} files did not parse, so they were "
            "not checked:",
            file=sys.stderr,
        )
        for rel, error in sorted(unparsed):
            print(f"  {rel}\n    {error}", file=sys.stderr)
        print(
            "\nA compiler flag clang's driver rejects outright is the usual cause;\n"
            "add it to GCC_ONLY_FLAGS in scripts/clang_tidy_env.py. Coverage is\n"
            "part of the result, so this fails rather than passing quietly.",
            file=sys.stderr,
        )
        return 1

    if findings:
        print("Unused includes:", file=sys.stderr)
        for finding in sorted(set(findings)):
            print(f"  {finding}", file=sys.stderr)
        print(
            "\nRemove them, or add `// IWYU pragma: keep` to the include line when\n"
            "the header is there for a side effect rather than a name.",
            file=sys.stderr,
        )
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
