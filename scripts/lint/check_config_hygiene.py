#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi
# /// script
# dependencies = [
#     "jinja2",
#     "kconfiglib",
# ]
# ///

"""Kconfig well-formedness checks that neither the build nor the generators make.

check_config_staleness.py answers "is this symbol read by anything". These are
the other questions: is the declaration itself well formed, does the saved
configuration still say what it looks like it says, and does a generator run
depend on anything but its inputs.

Each check exists because the failure it catches is silent. A stale `default`
is only discovered by whoever first builds without config/32raven.config; a
choice with one option renders a menu entry that cannot change anything; an int
with no `range` accepts a value the driver will not; a hand-edited .config
drifts from the form menuconfig writes; and a generator that reads the clock or
iterates a set produces a header that differs run to run for no stated reason.
"""
from __future__ import annotations

import pathlib
import sys
import tempfile

REPO = pathlib.Path(__file__).resolve().parent.parent.parent
KCONFIG = REPO / "config/Kconfig"
DOT_CONFIG = REPO / "config/32raven.config"

sys.path.insert(0, str(REPO / "scripts"))

import kconfiglib  # noqa: E402

# Generator module -> (validate fn, context fns). The context functions are the
# ones whose output lands in a header, so they are what determinism applies to.
GENERATORS: dict[str, tuple[str, tuple[str, ...]]] = {
    "generate_stm32_config": ("_validate", ("_runtime_context", "_limits_context")),
    "generate_esp32_config": ("_validate", ("_runtime_context", "_limits_context")),
    "generate_common_config": ("", ("fclink_context",)),
}


def _load(with_saved_config: bool) -> kconfiglib.Kconfig:
    kconf = kconfiglib.Kconfig(str(KCONFIG), warn=False)
    if with_saved_config:
        kconf.load_config(str(DOT_CONFIG))
    return kconf


def _call(fn, kconf: kconfiglib.Kconfig):
    """Invoke a generator entry point, which takes kconf with or without a source."""
    import inspect

    params = inspect.signature(fn).parameters
    return fn(DOT_CONFIG, kconf) if len(params) == 2 else fn(kconf)


def check_defaults_build_a_legal_board(problems: list[str]) -> None:
    """Every generator must accept the Kconfig defaults with no .config at all.

    The saved configuration masks its own defaults, so a `default` line can name
    a pin the board reassigned years ago and nothing says so until someone
    builds a fresh checkout or resets the menu.
    """
    import importlib

    for module_name, (validate_name, _) in GENERATORS.items():
        if not validate_name:
            continue
        module = importlib.import_module(module_name)
        try:
            _call(getattr(module, validate_name), _load(with_saved_config=False))
        except Exception as exc:  # noqa: BLE001 - the generator's own error is the message
            problems.append(
                f"[defaults] {module_name} rejects the Kconfig defaults: {exc}"
            )


def _pin_map_symbols() -> set[str]:
    """Symbol names the STM32 pin-map tables name.

    A pin bonded out on exactly one pad still has to reach the generated pin
    map, because that is what the collision and alternate-function checks read.
    Its choice therefore has one arm on purpose, and is not bloat.
    """
    import importlib

    found: set[str] = set()
    seen: set[int] = set()

    def walk(obj: object, depth: int) -> None:
        if depth > 8 or id(obj) in seen:
            return
        seen.add(id(obj))
        if isinstance(obj, str):
            found.add(obj)
        elif isinstance(obj, dict):
            for key, value in obj.items():
                walk(key, depth + 1)
                walk(value, depth + 1)
        elif isinstance(obj, (list, tuple, set, frozenset)):
            for item in obj:
                walk(item, depth + 1)
        else:
            fields = getattr(obj, "__dict__", None)
            if isinstance(fields, dict):
                for value in fields.values():
                    walk(value, depth + 1)

    walk(getattr(importlib.import_module("generate_stm32_config"), "PINMAP_ENTRIES", ()), 0)
    return found


def check_pin_defaults_match_the_board(problems: list[str]) -> None:
    """A pin's `default` has to describe the board that exists.

    Pure-defaults validation cannot catch this on its own: a pin behind a
    feature that defaults off is never even reached, so a default naming a pad
    the board reassigned survives until someone enables that feature on a fresh
    tree and collides with whatever moved in. Comparing each pin's default
    against the saved configuration finds it without guessing which feature
    combinations are legal. Non-pin knobs are excluded -- deviating from a
    default is the entire point of a tunable.
    """
    saved = _load(with_saved_config=True)
    defaults = _load(with_saved_config=False)
    pin_symbols = _pin_map_symbols()

    for sym in saved.unique_defined_syms:
        name = sym.name
        if not name:
            continue
        if not (name.startswith("ESP32_PINMAP_") or name in pin_symbols):
            continue
        default_sym = defaults.syms.get(name)
        if default_sym is None or default_sym.str_value == sym.str_value:
            continue
        problems.append(
            f"[pin]      {name} defaults to {default_sym.str_value or '<unset>'} "
            f"but the board uses {sym.str_value or '<unset>'}; a stale pin "
            "default builds a map that cannot exist"
        )


def check_single_arm_choices(kconf: kconfiglib.Kconfig, problems: list[str]) -> None:
    pin_symbols = _pin_map_symbols()
    for choice in kconf.unique_choices:
        options = [s for s in choice.syms if s.name]
        if len(options) != 1 or options[0].name in pin_symbols:
            continue
        prompt = choice.nodes[0].prompt[0] if choice.nodes and choice.nodes[0].prompt else "?"
        problems.append(
            f"[choice]   {options[0].name} is the only option under \"{prompt}\"; "
            "a one-armed choice is a menu entry that cannot change anything"
        )


def check_int_symbols_have_ranges(
    kconf: kconfiglib.Kconfig, problems: list[str]
) -> None:
    for sym in kconf.unique_defined_syms:
        if not sym.name or sym.type not in (kconfiglib.INT, kconfiglib.HEX):
            continue
        if any(node.ranges for node in sym.nodes):
            continue
        problems.append(
            f"[range]    {sym.name} is {kconfiglib.TYPE_TO_STR[sym.type]} with no "
            "range, so menuconfig accepts values the firmware will not"
        )


def check_saved_config_is_canonical(problems: list[str]) -> None:
    """The saved config must match what kconfiglib would write.

    Editing it by hand is normal; leaving it in a form menuconfig will rewrite
    is what turns the next unrelated menu visit into a hundred-line diff.
    """
    kconf = _load(with_saved_config=True)
    with tempfile.TemporaryDirectory() as tmp:
        written = pathlib.Path(tmp) / "canonical.config"
        kconf.write_config(str(written))
        canonical_text = written.read_text()

    def settings(text: str) -> list[str]:
        return [l for l in text.splitlines() if l.startswith(("CONFIG_", "# CONFIG_"))]

    saved = settings(DOT_CONFIG.read_text())
    canonical = settings(canonical_text)
    if saved == canonical:
        return
    extra = [l for l in saved if l not in set(canonical)]
    missing = [l for l in canonical if l not in set(saved)]
    detail = ""
    if extra:
        detail += f" first unexpected line: {extra[0]}"
    elif missing:
        detail += f" first missing line: {missing[0]}"
    problems.append(
        f"[saved]    config/32raven.config is not in canonical form "
        f"({len(saved)} lines saved vs {len(canonical)} written);{detail}. "
        "Run menuconfig and save, or regenerate it"
    )


def check_generators_are_deterministic(problems: list[str]) -> None:
    """Two runs over one configuration must produce the same context.

    Ordering that comes from a set, or a value read from the clock, makes the
    generated header change without any input changing -- which shows up as a
    diff nobody can explain rather than as a failure.
    """
    import importlib

    for module_name, (_, context_names) in GENERATORS.items():
        module = importlib.import_module(module_name)
        for context_name in context_names:
            fn = getattr(module, context_name, None)
            if fn is None:
                continue
            first = _call(fn, _load(with_saved_config=True))
            second = _call(fn, _load(with_saved_config=True))
            if first != second:
                problems.append(
                    f"[nondet]   {module_name}.{context_name} returned different "
                    "output for the same configuration"
                )


def main() -> int:
    problems: list[str] = []
    kconf = _load(with_saved_config=True)

    check_defaults_build_a_legal_board(problems)
    check_pin_defaults_match_the_board(problems)
    check_single_arm_choices(kconf, problems)
    check_int_symbols_have_ranges(kconf, problems)
    check_saved_config_is_canonical(problems)
    check_generators_are_deterministic(problems)

    if not problems:
        return 0

    print("Kconfig hygiene:", file=sys.stderr)
    for problem in problems:
        print(f"  {problem}", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
