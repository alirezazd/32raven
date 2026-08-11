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
import json
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile

REPO = pathlib.Path(__file__).resolve().parent.parent.parent
BUILD_DIRS = [REPO / "build/Ninja/stm32", REPO / "build/Ninja/esp32"]

# Generated or vendored: not ours to tidy. syscalls.c and sysmem.c are ST's libc
# stubs, where the includes state the newlib/picolibc contract the file
# implements rather than naming types it happens to spell.
SKIP = re.compile(
    r"^(third_party/|stm32/lib/|esp32/ui/assets/bitmap/"
    r"|stm32/Core/(Src|Inc)/(system_)?stm32f4xx"
    r"|stm32/Core/(syscalls|sysmem)\.c$"
    r"|.*_(config|limits|schema)\.hpp$)"
)

# GCC accepts these, and ESP-IDF puts all three on every ESP32 translation
# unit. clang's driver rejects an unknown argument outright rather than warning
# past it, so a command carrying one never reaches the first #include. Stripping
# them from the compilation database is what puts the ESP32 under this check.
GCC_ONLY_FLAGS = frozenset(
    {
        "-fno-shrink-wrap",
        "-fno-tree-switch-conversion",
        "-fstrict-volatile-bitfields",
    }
)

# misc-include-cleaner is a C/C++ check, and the database carries the assembly
# startup file like any other entry.
SOURCE_SUFFIXES = frozenset({".c", ".cc", ".cpp", ".cxx"})

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


def parseable(arg: str) -> bool:
    """Keep only what clang needs to parse the file the way GCC compiled it.

    A -Werror promotion turns every difference between clang's warning set and
    GCC's into a parse failure -- clang's -Wall catches things in the ESP-IDF
    headers that GCC's does not. Warnings are the real build's job; this
    borrows the compile command to parse, nothing more.
    """
    return arg not in GCC_ONLY_FLAGS and not arg.startswith("-Werror")


def filtered_database(build_dir: pathlib.Path, into: pathlib.Path) -> pathlib.Path:
    """A copy of build_dir's compilation database clang's driver will accept."""
    out = into / build_dir.name
    out.mkdir(parents=True, exist_ok=True)
    entries = json.loads((build_dir / "compile_commands.json").read_text())
    for entry in entries:
        entry["command"] = " ".join(
            a for a in entry["command"].split() if parseable(a)
        )
    (out / "compile_commands.json").write_text(json.dumps(entries))
    return out


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
        if str(p) not in db or p.suffix not in SOURCE_SUFFIXES:
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
        print(
            "\nA compiler flag clang's driver rejects outright is the usual cause;\n"
            "add it to GCC_ONLY_FLAGS. Coverage is part of the result, so this\n"
            "fails rather than passing quietly.",
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
