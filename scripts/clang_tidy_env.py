#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi

"""Shared plumbing for running clang-tidy against the cross-compiled tree.

clang-tidy has to be told three things GCC's compilation database does not
supply on its own: where the cross toolchain keeps its system headers, that a
handful of GCC-only flags must be dropped, and that -Werror promotions turn
every clang-vs-GCC warning difference into a parse failure. Getting any of them
wrong does not produce an error -- it produces a translation unit that fails to
parse, which reads as "no findings".

Lives at scripts/ root rather than scripts/lint/ because more than one lint
imports it, matching pin_constraints.py.
"""
from __future__ import annotations

import json
import pathlib
import re
import subprocess

REPO = pathlib.Path(__file__).resolve().parent.parent
BUILD_DIRS = [REPO / "build/Ninja/stm32", REPO / "build/Ninja/esp32"]

# GCC accepts these, and ESP-IDF puts all three on every ESP32 translation
# unit. clang's driver rejects an unknown argument outright rather than warning
# past it, so a command carrying one never reaches the first #include.
GCC_ONLY_FLAGS = frozenset(
    {
        "-fno-shrink-wrap",
        "-fno-tree-switch-conversion",
        "-fstrict-volatile-bitfields",
    }
)

# Only first-party code. The esp32 compilation database also carries every
# ESP-IDF component and generated build artifact.
FIRST_PARTY = re.compile(r"^(stm32|esp32|libs)/")

# Generated or vendored: not ours to tidy. syscalls.c and sysmem.c are ST's libc
# stubs, where the includes state the newlib/picolibc contract the file
# implements rather than naming types it happens to spell.
SKIP = re.compile(
    r"^(third_party/|stm32/lib/|esp32/ui/assets/bitmap/"
    r"|stm32/Core/(Src|Inc)/(system_)?stm32f4xx"
    r"|stm32/Core/(syscalls|sysmem)\.c$"
    r"|.*_(config|limits|schema)\.hpp$)"
)

# clang-tidy is a C/C++ tool; the database carries the assembly startup file
# like any other entry.
SOURCE_SUFFIXES = frozenset({".c", ".cc", ".cpp", ".cxx"})

_TOOLCHAIN: dict[str, list[str]] = {}


def parseable(arg: str) -> bool:
    """Keep only what clang needs to parse the file the way GCC compiled it.

    A -Werror promotion turns every difference between clang's warning set and
    GCC's into a parse failure -- clang's -Wall catches things in the ESP-IDF
    headers that GCC's does not. Warnings are the real build's job.
    """
    return arg not in GCC_ONLY_FLAGS and not arg.startswith("-Werror")


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


def filtered_database(build_dir: pathlib.Path, into: pathlib.Path) -> pathlib.Path:
    """A copy of build_dir's compilation database clang's driver will accept."""
    out = into / build_dir.name
    out.mkdir(parents=True, exist_ok=True)
    entries = json.loads((build_dir / "compile_commands.json").read_text())
    for entry in entries:
        entry["command"] = " ".join(a for a in entry["command"].split() if parseable(a))
    (out / "compile_commands.json").write_text(json.dumps(entries))
    return out


def toolchain_args(compiler: str) -> list[str]:
    """The cross toolchain's own target and system headers.

    clang-tidy has no equivalent of clangd's --query-driver, so without this it
    cannot find newlib or libstdc++ and the translation unit fails to parse. A
    file that does not parse reports every include as unused, which looks like
    a result rather than a failure -- so this is load-bearing, not a nicety.
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


def first_party_targets(
    wanted: list[pathlib.Path] | None = None,
) -> list[tuple[pathlib.Path, pathlib.Path, str]]:
    """(source, build dir, compiler) for every first-party TU we can compile."""
    db = databases()
    paths = wanted if wanted is not None else [pathlib.Path(p) for p in db]

    targets = []
    for p in paths:
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
    return targets
