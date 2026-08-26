#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi
"""Reproduce the linker's memory-region table from an ELF already on disk.

`-Wl,--print-memory-usage` writes that table during the link, so a build with
nothing to do prints nothing. This recomputes it, which is what makes the
figure available on every `make stm32` rather than only after a relink.

Usage is measured as `max(end) - origin` per region, not as the sum of the
parts: the linker counts the extent it consumed, and alignment padding between
sections sits inside that extent while belonging to no section. Each LOAD
segment is placed twice -- once at its physical address for the image it
occupies in flash, once at its virtual address for the bytes it needs at
runtime -- so an initialised-data segment lands in both regions and a .bss one
in neither's image.
"""

from __future__ import annotations

import argparse
import pathlib
import re
import subprocess
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
DEFAULT_ELF = REPO / "build/Ninja/stm32/32Raven_stm32.elf"
DEFAULT_LD = REPO / "stm32/STM32F407XX_FLASH.ld"

_MEMORY_BLOCK = re.compile(r"^MEMORY\s*\{(.*?)^\}", re.S | re.M)
_REGION = re.compile(
    r"^\s*(?P<name>\w+)\s*\([^)]*\)\s*:"
    r"\s*ORIGIN\s*=\s*(?P<origin>0[xX][0-9a-fA-F]+|\d+)\s*,"
    r"\s*LENGTH\s*=\s*(?P<length>\d+)(?P<unit>[KMG]?)",
    re.M,
)
_SUFFIX = {"": 1, "K": 1024, "M": 1024**2, "G": 1024**3}


class ReadelfError(Exception):
    """The ELF could not be read.

    Reporting a size is not worth failing a build whose compile and link both
    succeeded, so callers warn on this rather than exiting non-zero.
    """


class Region:
    def __init__(self, name: str, origin: int, length: int) -> None:
        self.name = name
        self.origin = origin
        self.length = length
        self.end = origin  # highest address consumed so far

    def place(self, addr: int, size: int) -> None:
        if size == 0 or not (self.origin <= addr < self.origin + self.length):
            return
        self.end = max(self.end, addr + size)

    @property
    def used(self) -> int:
        return self.end - self.origin


def parse_regions(ld_path: pathlib.Path) -> list[Region]:
    block = _MEMORY_BLOCK.search(ld_path.read_text(encoding="utf-8"))
    if not block:
        raise SystemExit(f"no MEMORY block in {ld_path}")
    regions = [
        Region(
            m.group("name"),
            int(m.group("origin"), 0),
            int(m.group("length")) * _SUFFIX[m.group("unit").upper()],
        )
        for m in _REGION.finditer(block.group(1))
    ]
    if not regions:
        raise SystemExit(f"no regions parsed from {ld_path}")
    return regions


def place_segments(
    elf: pathlib.Path, regions: list[Region], readelf: str
) -> None:
    try:
        out = subprocess.run(
            [readelf, "-lW", str(elf)],
            capture_output=True,
            text=True,
            check=True,
        ).stdout
    except FileNotFoundError:
        raise ReadelfError(f"{readelf} not found") from None
    except subprocess.CalledProcessError as exc:
        raise ReadelfError(f"{readelf}: {exc.stderr.strip()}") from None

    for line in out.splitlines():
        parts = line.split()
        if len(parts) < 6 or parts[0] != "LOAD":
            continue
        _, _, virt, phys, filesz, memsz = parts[:6]
        for region in regions:
            # The image it occupies, then the bytes it needs at runtime.
            region.place(int(phys, 0), int(filesz, 0))
            region.place(int(virt, 0), int(memsz, 0))


def humanize(size: int) -> str:
    """Match the linker: exact multiples collapse, anything else stays bytes."""
    if size >= 1024**2 and size % 1024**2 == 0:
        return f"{size // 1024**2} MB"
    if size >= 1024 and size % 1024 == 0:
        return f"{size // 1024} KB"
    return f"{size} B"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--elf", type=pathlib.Path, default=DEFAULT_ELF)
    parser.add_argument("--ld", type=pathlib.Path, default=DEFAULT_LD)
    parser.add_argument("--readelf", default="arm-none-eabi-readelf")
    args = parser.parse_args()

    if not args.elf.is_file():
        print(f"elf not found: {args.elf}", file=sys.stderr)
        return 1
    if not args.ld.is_file():
        print(f"linker script not found: {args.ld}", file=sys.stderr)
        return 1

    regions = parse_regions(args.ld)
    try:
        place_segments(args.elf, regions, args.readelf)
    except ReadelfError as exc:
        print(f"size report skipped: {exc}", file=sys.stderr)
        return 0

    print("Memory region         Used Size  Region Size  %age Used")
    for region in regions:
        pct = (region.used / region.length * 100.0) if region.length else 0.0
        print(
            f"{region.name + ':':>17}"
            f"{humanize(region.used):>14}"
            f"{humanize(region.length):>13}"
            f"{pct:>10.2f}%"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
