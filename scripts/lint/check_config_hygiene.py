#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi
# /// script
# dependencies = [
#     "jinja2",
#     "kconfiglib",
# ]
# ///

"""Kconfig well-formedness checks the build and generators do not make.

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
    "generate_stm32_config": (
        "_validate",
        ("_runtime_context", "_limits_context"),
    ),
    "generate_esp32_config": (
        "_validate",
        ("_runtime_context", "_limits_context"),
    ),
    "generate_common_config": ("", ("fclink_context",)),
}


def _load(with_saved_config: bool) -> kconfiglib.Kconfig:
    kconf = kconfiglib.Kconfig(str(KCONFIG), warn=False)
    if with_saved_config:
        kconf.load_config(str(DOT_CONFIG))
    return kconf


def _call(fn, kconf: kconfiglib.Kconfig):
    """Invoke an entry point; takes kconf with or without a source."""
    import inspect

    params = inspect.signature(fn).parameters
    return fn(DOT_CONFIG, kconf) if len(params) == 2 else fn(kconf)


def check_defaults_build_a_legal_board(problems: list[str]) -> None:
    """Every generator must run to completion on the Kconfig defaults alone.

    The saved configuration masks its own defaults, so a `default` line can name
    a pin the board reassigned years ago and nothing says so until someone
    builds a fresh checkout or resets the menu.

    The context functions are exercised alongside the validator because passing
    validation is not the same as producing a header: a symbol behind a feature
    that defaults off carries no value, validation skips it as unreachable, and
    the context function that reads it anyway is where the run actually dies.
    """
    import importlib

    for module_name, (validate_name, context_names) in GENERATORS.items():
        module = importlib.import_module(module_name)
        for entry_name in (validate_name, *context_names):
            fn = getattr(module, entry_name, None) if entry_name else None
            if fn is None:
                continue
            try:
                _call(fn, _load(with_saved_config=False))
            # SystemExit derives from BaseException, and the generators raise it
            # as readily as ValueError. Catching Exception alone would let a
            # generator that rejects its own defaults kill this check instead of
            # be reported by it -- the one outcome the check exists to prevent.
            except (Exception, SystemExit) as exc:  # noqa: BLE001
                problems.append(
                    f"[defaults] {module_name}.{entry_name} fails on "
                    "the Kconfig "
                    f"defaults: {exc}"
                )


# PINMAP_ENTRIES fields that name where a pin sits: the port or signal choice
# arms, and the pin-number int. active_low_sym rides in the same table but sets
# polarity, not position, so it belongs with the ordinary tunables.
_PLACEMENT_FIELDS = frozenset(
    {"port_options", "choice_options", "pin_int_symbol"}
)
_TUNABLE_FIELDS = frozenset({"active_low_sym"})


def _pin_entries() -> tuple:
    import importlib

    return getattr(
        importlib.import_module("generate_stm32_config"), "PINMAP_ENTRIES", ()
    )


def _symbol_strings(value: object) -> set[str]:
    """Symbol names a field holds; for a dict (symbol -> pin), its keys."""
    if isinstance(value, dict):
        return {k for k in value if isinstance(k, str)}
    return {value} if isinstance(value, str) else set()


def _pin_map_symbols() -> set[str]:
    """Kconfig symbols that place a pin on the STM32 board map.

    A pin bonded out on exactly one pad still has to reach the generated pin
    map, because that is what the collision and alternate-function checks read.
    Its choice therefore has one arm on purpose, and is not bloat.
    """
    return {
        name
        for entry in _pin_entries()
        for field in _PLACEMENT_FIELDS
        for name in _symbol_strings(getattr(entry, field, None))
    }


def check_pin_map_fields_are_classified(
    kconf: kconfiglib.Kconfig, problems: list[str]
) -> None:
    """Every symbol-bearing field of a pin entry has to be placement or tunable.

    The two checks reading _pin_map_symbols fail in opposite directions: a
    placement symbol left out lets a stale pin default through, and a tunable
    swept in reports a board that merely set it as one that cannot exist. A new
    field holding symbols must therefore not default quietly into either.
    """
    import dataclasses

    known = _PLACEMENT_FIELDS | _TUNABLE_FIELDS
    for entry in _pin_entries():
        if not dataclasses.is_dataclass(entry):
            continue
        for field in dataclasses.fields(entry):
            if field.name in known:
                continue
            value = getattr(entry, field.name, None)
            held = (
                {s for s in (*value, *value.values()) if isinstance(s, str)}
                if isinstance(value, dict)
                else {value}
                if isinstance(value, str)
                else set()
            )
            stray = sorted(n for n in held if n in kconf.syms)
            if stray:
                problems.append(
                    f"[pinmap]   {type(entry).__name__}.{field.name} "
                    "holds Kconfig "
                    f"symbol(s) {', '.join(stray)} but is classified "
                    "as neither "
                    "placement nor tunable, so _pin_map_symbols ignores it"
                )


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
            f"[pin]      {name} defaults to "
            f"{default_sym.str_value or '<unset>'} "
            f"but the board uses {sym.str_value or '<unset>'}; a stale pin "
            "default builds a map that cannot exist"
        )


def check_single_arm_choices(
    kconf: kconfiglib.Kconfig, problems: list[str]
) -> None:
    pin_symbols = _pin_map_symbols()
    for choice in kconf.unique_choices:
        options = [s for s in choice.syms if s.name]
        if len(options) != 1 or options[0].name in pin_symbols:
            continue
        prompt = (
            choice.nodes[0].prompt[0]
            if choice.nodes and choice.nodes[0].prompt
            else "?"
        )
        problems.append(
            f"[choice]   {options[0].name} is the only option under "
            f'"{prompt}"; '
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
            f"[range]    {sym.name} is "
            f"{kconfiglib.TYPE_TO_STR[sym.type]} with no "
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
        return [
            ln
            for ln in text.splitlines()
            if ln.startswith(("CONFIG_", "# CONFIG_"))
        ]

    saved = settings(DOT_CONFIG.read_text())
    canonical = settings(canonical_text)
    if saved == canonical:
        return
    extra = [ln for ln in saved if ln not in set(canonical)]
    missing = [ln for ln in canonical if ln not in set(saved)]
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
                    f"[nondet]   {module_name}.{context_name} returned "
                    "different "
                    "output for the same configuration"
                )


def main() -> int:
    problems: list[str] = []
    kconf = _load(with_saved_config=True)

    check_defaults_build_a_legal_board(problems)
    check_pin_map_fields_are_classified(kconf, problems)
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
