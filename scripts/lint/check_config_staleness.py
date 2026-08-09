#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi
# /// script
# dependencies = [
#     "jinja2",
#     "kconfiglib",
# ]
# ///

"""Detect Kconfig symbols nothing consumes, and .config lines that do not stick.

The build already catches drift in one direction: `sym_int`/`sym_bool` raise on
a symbol a generator asks for but Kconfig does not define, and Jinja's
StrictUndefined raises on a template variable no context supplies. Both are
hard errors at generation time.

The other direction is silent. A symbol declared in Kconfig that no generator
reads produces a menuconfig entry that changes nothing, and a `.config` value
outside its declared `range` is quietly replaced by the default rather than
rejected. Either one leaves a knob that looks live and is not.

Consumption is measured by running the generators, not by scanning them.
Symbol names are routinely built at runtime -- `f"STM32_ATT_CTRL_{axis}_KP_MILLI"`,
`key.replace("GYRO", "ACCEL")` -- so a text search reports most of the tree as
dead. Every read funnels through `kconfig_gen.sym`, so recording calls there
observes what is actually consumed.

Two things that observation alone would miss:

  choice tables   `choice_value` returns at the first symbol that is set, so the
                  rest of the mapping is never probed. Module-level tables keyed
                  by symbol name are therefore counted whole.
  Kconfig-side use A symbol named only in a `depends on`, `select` or
                  `default ... if ...` expression is read by kconfiglib rather
                  than by us. Those come from `Symbol.referenced`.
"""
from __future__ import annotations

import importlib
import inspect
import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent.parent
KCONFIG = REPO / "config/Kconfig"
DOT_CONFIG = REPO / "config/32raven.config"

sys.path.insert(0, str(REPO / "scripts"))

import kconfiglib  # noqa: E402
import kconfig_gen  # noqa: E402

# Generator module -> the entry points that read Kconfig. Anything a generator
# consumes is reached through one of these; `main()` only adds argparse and
# file writing on top.
GENERATORS: dict[str, tuple[str, ...]] = {
    "generate_stm32_config": ("_validate", "_runtime_context", "_limits_context"),
    "generate_esp32_config": ("_validate", "_runtime_context", "_limits_context"),
    "generate_common_config": ("fclink_context",),
}

read: set[str] = set()


def _record_sym(kconf: kconfiglib.Kconfig, name: str) -> kconfiglib.Symbol:
    read.add(name)
    return _ORIGINAL_SYM(kconf, name)


_ORIGINAL_SYM = kconfig_gen.sym
kconfig_gen.sym = _record_sym


def _call(fn, source: pathlib.Path, kconf: kconfiglib.Kconfig) -> None:
    """Invoke a generator entry point, which takes kconf with or without a source."""
    params = inspect.signature(fn).parameters
    fn(source, kconf) if len(params) == 2 else fn(kconf)


def _harvest_tables(module, defined: set[str]) -> set[str]:
    """Symbol names reachable from a module-level table.

    A generator that maps symbols to rendered values consumes the whole table
    even though it stops reading at the one that is set. The tables nest --
    PINMAP_ENTRIES is a tuple of dataclasses whose fields are the candidate
    dicts -- so this walks containers and plain objects alike.
    """
    found: set[str] = set()
    seen: set[int] = set()

    def walk(obj: object, depth: int) -> None:
        if depth > 8 or id(obj) in seen:
            return
        seen.add(id(obj))
        if isinstance(obj, str):
            if obj in defined:
                found.add(obj)
        elif isinstance(obj, dict):
            for key, value in obj.items():
                walk(key, depth + 1)
                walk(value, depth + 1)
        elif isinstance(obj, (list, tuple, set, frozenset)):
            for item in obj:
                walk(item, depth + 1)
        elif not (inspect.ismodule(obj) or inspect.isclass(obj) or callable(obj)):
            fields = getattr(obj, "__dict__", None)
            if isinstance(fields, dict):
                for value in fields.values():
                    walk(value, depth + 1)

    for value in vars(module).values():
        if inspect.ismodule(value) or inspect.isclass(value) or callable(value):
            continue
        walk(value, 0)
    return found


def _expression_symbols(kconf: kconfiglib.Kconfig) -> set[str]:
    """Symbols kconfiglib evaluates itself, via depends on / select / default-if."""
    found: set[str] = set()
    for item in (*kconf.unique_defined_syms, *kconf.unique_choices):
        for ref in item.referenced:
            if isinstance(ref, kconfiglib.Symbol) and ref.name:
                found.add(ref.name)
    return found


def _unstuck_values(kconf: kconfiglib.Kconfig) -> list[str]:
    """`.config` assignments the loaded configuration did not keep.

    An int outside its `range` falls back to the default, and a symbol that no
    longer exists is dropped. Both leave the file saying one thing and the build
    doing another.
    """
    problems: list[str] = []
    for line in DOT_CONFIG.read_text().splitlines():
        line = line.strip()
        if not line.startswith("CONFIG_") or "=" not in line:
            continue
        name, _, wanted = line.partition("=")
        name = name.removeprefix("CONFIG_")
        symbol = kconf.syms.get(name)
        if symbol is None or not symbol.nodes:
            problems.append(f"{name} is set in .config but Kconfig does not define it")
            continue
        if symbol.type in (kconfiglib.INT, kconfiglib.HEX) and symbol.str_value != wanted:
            problems.append(
                f"{name}={wanted} did not stick (in use: {symbol.str_value}); "
                "check its range"
            )
    return problems


def main() -> int:
    kconf = kconfiglib.Kconfig(str(KCONFIG), warn=False)
    kconf.load_config(str(DOT_CONFIG))
    defined = {s.name for s in kconf.unique_defined_syms if s.name}

    consumed: set[str] = set()
    for module_name, entry_points in GENERATORS.items():
        module = importlib.import_module(module_name)
        for entry_point in entry_points:
            _call(getattr(module, entry_point), DOT_CONFIG, kconf)
        consumed |= _harvest_tables(module, defined)
    consumed |= read
    consumed |= _expression_symbols(kconf)

    # A symbol whose dependencies are unmet cannot be set in this
    # configuration, so no run can observe it being read -- reporting it would
    # only say that the branch guarding it is not the one selected.
    unreachable = {
        s.name
        for s in kconf.unique_defined_syms
        if s.name
        and s.visibility == 0
        and any(node.prompt for node in s.nodes)
    }

    orphans = sorted(defined - consumed - unreachable)
    stuck = _unstuck_values(kconf)
    if not orphans and not stuck:
        return 0

    print("Kconfig staleness:", file=sys.stderr)
    for name in orphans:
        print(f"  [orphan] {name} is declared but no generator reads it", file=sys.stderr)
    for problem in stuck:
        print(f"  [value]  {problem}", file=sys.stderr)
    print(
        "\nAn orphan is either a knob that needs wiring up in a generator, or "
        "one whose consumer is gone and should be deleted from config/Kconfig.",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
