#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi
"""Bound every FreeRTOS task's stack from the ELF; fail on a thin margin.

Frame sizes come from each function's prologue and edges from its call
instructions, so the bound tracks the build rather than a comment that was true
once. Indirect calls are the whole difficulty: the graph cannot follow one, and
a chain that stops at an indirect site reports a small number rather than an
unknown -- which reads as a pass. Every indirect site that lands on a worst-case
chain must therefore appear in RESOLVED or ACKNOWLEDGED below, or this check
fails and says which one is missing.
"""

from __future__ import annotations

import argparse
import pathlib
import re
import subprocess
import sys
from collections import defaultdict

# Interrupt entry pushes an RvExcFrame on the interruptee's stack before it
# switches to xIsrStack (riscv/vectors.S), so every task pays it on top of its
# deepest call chain. sizeof(RvExcFrame) rounded up to 16.
EXC_FRAME_BYTES = 144

# Below this, a chain that the graph slightly under-measures stops being a
# theoretical concern. Overflow is caught by the hardware stack guard, so the
# failure is a reboot rather than corruption -- still fatal mid-DFU.
MIN_MARGIN = 0.20

# The task the handoff leads to, so its own chain keeps the recovery loop.
PANIC_ENTRY = "_ZN12_GLOBAL__N_19PanicTaskEPv"

# label, task entry symbol, pattern matching its static stack array
TASKS = (
    ("panic", PANIC_ENTRY, r"s_panic_task_stackE$"),
    ("display", "_ZN2Ui4TaskEv", r"^_ZZN2Ui4Init.*task_stack$"),
    (
        "tone_player",
        "_ZN10TonePlayer4TaskEv",
        r"^_ZZN10TonePlayer4Init.*task_stack$",
    ),
    ("led_task", "_ZN3LED4TaskEv", r"^_ZZN3LED4Init.*task_stack$"),
)

# Indirect sites whose targets are known, so the walk can carry on through them.
RESOLVED = {
    # the vprintf_like_t hook, whatever esp_log_set_vprintf last installed
    "esp_log": ("vprintf",),
    # Widget::OnStep, one entry per widget that can be current
    "_ZN2Ui4StepEmR13WidgetContext": (
        "_ZN10BootWidget6OnStepER13WidgetContextm",
        "_ZN11ErrorWidget6OnStepER13WidgetContextm",
        "_ZN12MainUiWidget6OnStepER13WidgetContextm",
    ),
}

# Indirect sites the walk stops at, each with the reason that is safe.
ACKNOWLEDGED = {
    "_printf_float": (
        "newlib's own output hooks; the depth behind them is __cvt "
        "and _dtoa_r, "
        "which the chain already counts"
    ),
    "gdma_acquire_group_handle": (
        "mbedtls's hardware-SHA DMA tail behind sha256_update on the recovery "
        "flash chain; the pointer selects the AHB HAL group init, register "
        "writes with shallow frames"
    ),
}

# PanicImpl notifies the panic task and suspends the caller, reaching
# RunPanicLoop itself only if task creation failed -- which static buffers make
# impossible. So the recovery loop is charged to the panic task, and every other
# task's chain stops at the handoff.
HANDOFF = "_ZN12_GLOBAL__N_112RunPanicLoopEm"

FUNC_RE = re.compile(r"^([0-9a-f]+) <(.+)>:$")
INSN_RE = re.compile(
    r"^\s*[0-9a-f]+:\s+[0-9a-f]+(?: [0-9a-f]+)*\s+(\S+)\s*(.*)$"
)
SYM_RE = re.compile(r"<([^+>]+?)(?:\+0x[0-9a-f]+)?>")


class Graph:
    def __init__(self, disassembly: str) -> None:
        self.frames: dict[str, int] = {}
        self.calls: dict[str, set[str]] = defaultdict(set)
        self.indirect: dict[str, int] = defaultdict(int)
        cur: str | None = None
        for line in disassembly.splitlines():
            header = FUNC_RE.match(line)
            if header:
                name = header.group(2)
                cur = name
                self.frames.setdefault(name, 0)
                continue
            if cur is None:
                continue
            insn = INSN_RE.match(line)
            if not insn:
                continue
            op, args = insn.groups()
            body, _, comment = args.partition("#")
            body = body.strip()

            if op in ("addi", "c.addi", "c.addi16sp") and body.startswith(
                "sp,sp,"
            ):
                try:
                    delta = int(body.rsplit(",", 1)[1])
                except ValueError:
                    delta = 0
                if delta < 0:
                    self.frames[cur] = max(self.frames[cur], -delta)

            if op in ("jal", "c.jal", "j", "c.j", "tail"):
                target = SYM_RE.search(body) or SYM_RE.search(comment)
                if target and target.group(1) != cur:
                    self.calls[cur].add(target.group(1))
            elif op in ("jalr", "c.jalr"):
                # A far call is auipc ra + jalr, which objdump annotates with
                # its target; without an annotation it is a real indirect call.
                target = SYM_RE.search(comment)
                if target:
                    if target.group(1) != cur:
                        self.calls[cur].add(target.group(1))
                else:
                    self.indirect[cur] += 1

        for caller, callees in RESOLVED.items():
            if caller in self.frames:
                self.calls[caller].update(callees)

    def deepest(self, entry: str, cut: str | None) -> tuple[int, list[str]]:
        memo: dict[str, tuple[int, list[str]]] = {}
        on_path: set[str] = set()

        def walk(fn: str) -> tuple[int, list[str]]:
            if fn in memo:
                return memo[fn]
            if fn in on_path:
                return 0, []
            on_path.add(fn)
            best: tuple[int, list[str]] = (0, [])
            for callee in sorted(self.calls.get(fn, ())):
                if callee == cut or callee not in self.frames:
                    continue
                found = walk(callee)
                if found[0] > best[0]:
                    best = found
            on_path.discard(fn)
            result = (self.frames.get(fn, 0) + best[0], [fn] + best[1])
            memo[fn] = result
            return result

        return walk(entry)


def tool_from_cache(build: pathlib.Path, var: str) -> str:
    cache = (build / "CMakeCache.txt").read_text(
        encoding="utf-8", errors="replace"
    )
    found = re.search(rf"^{var}:FILEPATH=(.+)$", cache, re.MULTILINE)
    if not found:
        raise SystemExit(
            f"{var} missing from {build}/CMakeCache.txt; build first"
        )
    return found.group(1)


def stack_sizes(nm: str, elf: pathlib.Path) -> list[tuple[str, int]]:
    out = subprocess.run(
        [nm, "--print-size", "--radix=d", str(elf)],
        capture_output=True,
        text=True,
    )
    sizes = []
    for line in out.stdout.splitlines():
        parts = line.split()
        if len(parts) == 4 and parts[2] in ("b", "B", "d", "D"):
            sizes.append((parts[3], int(parts[1])))
    return sizes


def declared_task_count(root: pathlib.Path) -> int:
    total = 0
    for path in (root / "esp32").rglob("*.cpp"):
        total += len(
            re.findall(r"\bxTaskCreateStatic\w*\s*\(", path.read_text())
        )
    return total


def main() -> int:
    root = pathlib.Path(__file__).resolve().parent.parent
    ap = argparse.ArgumentParser(
        description="Check FreeRTOS task stack margins."
    )
    ap.add_argument("--build-dir", default=str(root / "build/Ninja/esp32"))
    build = pathlib.Path(ap.parse_args().build_dir).resolve()

    elfs = [e for e in build.glob("*.elf") if e.is_file()]
    if len(elfs) != 1:
        raise SystemExit(f"expected exactly one .elf in {build}")
    elf = elfs[0]

    objdump = tool_from_cache(build, "CMAKE_OBJDUMP")
    nm = tool_from_cache(build, "CMAKE_NM")

    created = declared_task_count(root)
    if created != len(TASKS):
        raise SystemExit(
            f"esp32/ creates {created} static tasks but TASKS covers "
            f"{len(TASKS)}.\n"
            "Add the new task's entry symbol and stack array to TASKS, or this "
            "check silently skips it."
        )

    graph = Graph(
        subprocess.run(
            [objdump, "-d", str(elf)], capture_output=True, text=True
        ).stdout
    )
    sizes = stack_sizes(nm, elf)

    print(f"{'task':<13} {'stack':>7} {'worst':>7} {'margin':>8}")
    failures: list[str] = []
    for label, entry, stack_pattern in TASKS:
        if entry not in graph.frames:
            failures.append(f"{label}: entry symbol {entry} not in {elf.name}")
            continue
        matches = [
            size for name, size in sizes if re.search(stack_pattern, name)
        ]
        if len(matches) != 1:
            failures.append(
                f"{label}: {len(matches)} symbols match "
                f"{stack_pattern!r}, expected 1"
            )
            continue
        stack = matches[0]

        cut = None if entry == PANIC_ENTRY else HANDOFF
        depth, chain = graph.deepest(entry, cut)
        worst = depth + EXC_FRAME_BYTES
        margin = 1.0 - (worst / stack)
        print(f"{label:<13} {stack:>7} {worst:>7} {margin:>7.0%}")

        unnamed = [
            fn
            for fn in chain
            if graph.indirect.get(fn)
            and fn not in RESOLVED
            and fn not in ACKNOWLEDGED
        ]
        for fn in unnamed:
            failures.append(
                f"{label}: unfollowed indirect call in {fn}; the bound stops "
                "there and is not a bound. Resolve it in RESOLVED or "
                "record why "
                "it is safe in ACKNOWLEDGED."
            )
        if margin < MIN_MARGIN:
            failures.append(
                f"{label}: {worst} B worst case in a {stack} B stack leaves "
                f"{margin:.0%}, under the {MIN_MARGIN:.0%} minimum.\n"
                "  deepest chain: " + " -> ".join(chain[:8])
            )

    if failures:
        print()
        sys.stdout.flush()
        for failure in failures:
            print(f"error: {failure}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
