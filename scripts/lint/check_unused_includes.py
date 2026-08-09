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
compiler, so a hard failure there would only ever be about the environment.

Run:
  uv run --quiet --script scripts/lint/check_unused_includes.py [files...]
"""
from __future__ import annotations

import concurrent.futures
import json
import pathlib
import re
import shutil
import subprocess
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent.parent
BUILD_DIRS = [REPO / "build/Ninja/stm32", REPO / "build/Ninja/esp32"]

# Generated or vendored: not ours to tidy. syscalls.c and sysmem.c are ST's libc
# stubs, where the includes state the newlib/picolibc contract the file
# implements rather than naming types it happens to spell.
SKIP = re.compile(
    r"^(third_party/|stm32/lib/|esp32/ui/assets/bitmap/"
    r"|stm32/Core/(Src|Inc)/(system_)?stm32f4xx"
    r"|stm32/Core/Src/(syscalls|sysmem)\.c$"
    r"|.*_(config|limits|schema)\.hpp$)"
)

# ESP-IDF headers do not parse under clang today, so those TUs yield no
# answer at all. A sentinel keeps that separate from "no unused includes".
SKIPPED = "\x00skipped"

# Only first-party code. The esp32 compilation database also carries every
# ESP-IDF component and generated build artifact.
FIRST_PARTY = re.compile(r"^(stm32|esp32|libs)/")

FINDING_RE = re.compile(
    r"^(?P<file>[^:]+):(?P<line>\d+):\d+: warning: included header "
    r"(?P<header>\S+) is not used directly"
)


def databases() -> dict[str, tuple[pathlib.Path, dict]]:
    """Maps an absolute source path to the build dir that knows how to build it."""
    out: dict[str, tuple[pathlib.Path, dict]] = {}
    for d in BUILD_DIRS:
        cdb = d / "compile_commands.json"
        if not cdb.exists():
            continue
        for entry in json.loads(cdb.read_text()):
            out.setdefault(str(pathlib.Path(entry["file"]).resolve()), (d, entry))
    return out


_TOOLCHAIN: dict[str, list[str]] = {}


def toolchain_args(compiler: str) -> list[str]:
    """The cross toolchain's own target and system headers.

    clang-tidy has no equivalent of clangd's --query-driver, so without this it
    cannot find newlib or libstdc++ and the translation unit fails to parse. A
    file that does not parse reports EVERY include as unused, which looks like a
    result rather than a failure -- so this is load-bearing, not a nicety.
    """
    if compiler in _TOOLCHAIN:
        return _TOOLCHAIN[compiler]

    args: list[str] = []
    name = pathlib.Path(compiler).name
    if "-" in name:  # e.g. arm-none-eabi-g++, riscv32-esp-elf-g++
        args.append("--target=" + name.rsplit("-", 1)[0])

    probe = subprocess.run(
        [compiler, "-E", "-x", "c++", "-", "-v"],
        input="",
        capture_output=True,
        text=True,
    )
    collecting = False
    for line in probe.stderr.split("\n"):
        if line.startswith("#include <...>"):
            collecting = True
            continue
        if line.startswith("End of search list"):
            break
        if collecting and line.startswith(" "):
            args += ["-isystem", line.strip()]

    _TOOLCHAIN[compiler] = args
    return args


def run(path: pathlib.Path, build_dir: pathlib.Path, compiler: str) -> list[str]:
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
    if "error:" in proc.stderr or "error:" in proc.stdout:
        return [SKIPPED]
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
    return findings


def main() -> int:
    if shutil.which("clang-tidy") is None:
        print("check_unused_includes: clang-tidy not installed, skipping.")
        return 0

    db = databases()
    if not db:
        print("check_unused_includes: no compilation database, skipping. Build first.")
        return 0

    if len(sys.argv) > 1:
        wanted = [pathlib.Path(a).resolve() for a in sys.argv[1:]]
    else:
        wanted = [pathlib.Path(p) for p in db]

    # Headers carry no compile command of their own; misc-include-cleaner reaches
    # them through the translation units that include them.
    targets = []
    for p in wanted:
        if str(p) not in db:
            continue
        try:
            rel = p.relative_to(REPO).as_posix()
        except ValueError:
            continue
        if not FIRST_PARTY.match(rel) or SKIP.match(rel):
            continue
        build_dir, entry = db[str(p)]
        targets.append((p, build_dir, entry["command"].split()[0]))
    if not targets:
        return 0

    findings: list[str] = []
    skipped = 0
    with concurrent.futures.ThreadPoolExecutor(max_workers=8) as pool:
        for result in pool.map(lambda t: run(*t), targets):
            for item in result:
                if item is SKIPPED or item == SKIPPED:
                    skipped += 1
                else:
                    findings.append(item)

    if skipped:
        print(
            f"check_unused_includes: {skipped}/{len(targets)} files did not parse "
            "under clang and were not checked."
        )

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
