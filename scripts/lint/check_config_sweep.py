#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi
# /// script
# dependencies = [
#     "jinja2",
#     "kconfiglib",
# ]
# ///

"""Find knobs that cannot change anything, and knobs with only one legal value.

check_config_staleness proves a symbol is read. Being read is not the same as
mattering: a symbol can be read into a value that renders identical bytes
whatever it is set to, and a symbol can be read into a validation that rejects
every setting but the one already in `.config`. Either way the menu offers a
choice the firmware does not have.

Every settable symbol is swept by assigning it each value it can take and
regenerating all five headers in memory:

  inert   No value changes a single byte of any header. Either the knob is
          decoration, or the only thing it feeds is a check -- the sweep cannot
          tell those apart, so read the symbol before deleting it.
  unread  Some value leaves another symbol with no value at all, because a
          generator reads unconditionally what its Kconfig made conditional.
          The menu offers a setting the build cannot produce.
  crash   Some value reaches a traceback. A refused configuration is supposed
          to arrive as an explained error, so this is a validation that is
          missing rather than one that is failing.

`--pinned` adds a fourth: symbols where every value but one is rejected. It is
off by default because the sweep moves one symbol at a time and holds the rest
at their configured values, which cannot distinguish a constant from a knob
boxed in by a sibling. STM32_RC_MAP_ROLL is only 1 because pitch, throttle and
yaw hold 2, 3 and 4; calling that a constant would be wrong.

Int and hex ranges narrower than EXHAUSTIVE_RANGE_SPAN are swept whole, since
sampling cannot tell a knob with one legal value from a knob whose other legal
values went untried. Wider ones are sampled at both ends, the midpoint and the
configured value, and are never reported as pinned. String symbols are skipped,
having no enumerable domain.

Run:
  uv run --quiet --script scripts/lint/check_config_sweep.py [--pinned]
"""
from __future__ import annotations

import hashlib
import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent.parent
KCONFIG = REPO / "config/Kconfig"
DOT_CONFIG = REPO / "config/32raven.config"

sys.path.insert(0, str(REPO / "scripts"))

import kconfiglib  # noqa: E402
import kconfig_gen  # noqa: E402
import generate_common_config  # noqa: E402
import generate_esp32_config  # noqa: E402
import generate_stm32_config  # noqa: E402

# Neither is a Kconfig input, and both cost a subprocess on every render. Held
# fixed so the fingerprint reflects the sweep and not the working tree.
generate_esp32_config._git_head_short_hash = lambda: "0" * 8
generate_esp32_config._firmware_version_string = lambda: "0.0.0"

TARGETS = (
    (generate_stm32_config, "stm32_config.hpp.j2", "stm32_limits.hpp.j2"),
    (generate_esp32_config, "esp32_config.hpp.j2", "esp32_limits.hpp.j2"),
)

SKIPPED_TYPES = (kconfiglib.STRING, kconfiglib.UNKNOWN)

# An int range no wider than this is tried value by value rather than sampled.
EXHAUSTIVE_RANGE_SPAN = 64


def _fingerprint(kconf: kconfiglib.Kconfig) -> str:
    """Every header this configuration would produce, hashed into one value."""
    env = kconfig_gen.template_env()
    parts = [
        kconfig_gen.render_template(
            env,
            "common_config.hpp.j2",
            {
                "autogen_warning": "",
                "fclink": generate_common_config.fclink_context(kconf),
            },
        )
    ]
    for module, runtime_template, limits_template in TARGETS:
        module._validate(kconf)
        parts.append(
            kconfig_gen.render_template(
                env, runtime_template, module._runtime_context(DOT_CONFIG, kconf)
            )
        )
        parts.append(
            kconfig_gen.render_template(
                env, limits_template, module._limits_context(DOT_CONFIG, kconf)
            )
        )
    return hashlib.sha256("".join(parts).encode()).hexdigest()


def _active_range(symbol: kconfiglib.Symbol) -> tuple[int, int] | None:
    for low, high, condition in symbol.ranges:
        if kconfiglib.expr_value(condition):
            return int(low.str_value, 0), int(high.str_value, 0)
    return None


def _candidates(symbol: kconfiglib.Symbol) -> tuple[list[str], bool]:
    """The values to try, and whether that covers the symbol's whole domain.

    A narrow range is swept entry by entry, because sampling one cannot tell a
    knob with a single legal value from a knob whose other legal values simply
    were not among the samples. Wide ranges are sampled and say so.
    """
    if symbol.type == kconfiglib.BOOL:
        return ["n", "y"], True
    bounds = _active_range(symbol)
    if bounds is None:
        return [], False
    low, high = bounds
    fmt = hex if symbol.type == kconfiglib.HEX else str
    if high - low <= EXHAUSTIVE_RANGE_SPAN:
        return [fmt(value) for value in range(low, high + 1)], True
    sampled = {low, high, (low + high) // 2, int(symbol.str_value, 0)}
    return [fmt(value) for value in sorted(sampled)], False


def _outcome(kconf: kconfiglib.Kconfig, setter) -> tuple[str, str]:
    """Apply one candidate to a freshly reloaded config and generate.

    The reload is the isolation: a candidate that flips a menu's visibility
    changes what every symbol under it resolves to, and those have to be back
    the way `.config` left them before the next one is tried.
    """
    kconf.load_config(str(DOT_CONFIG))
    if not setter():
        return "unset", ""
    try:
        return "ok", _fingerprint(kconf)
    except kconfig_gen.UnsetSymbolError as exc:
        return "unread", str(exc)
    # SystemExit counts as a rejection: the clock-tree checks raise it to print
    # their explanation without a traceback behind it.
    except (ValueError, SystemExit) as exc:
        return "rejected", str(exc)
    except Exception as exc:  # noqa: BLE001 - see the `crash` class above
        return "crash", f"{type(exc).__name__}: {exc}"


def _sweep(
    kconf: kconfiglib.Kconfig,
    name: str,
    setters: dict[str, object],
    exhaustive: bool,
):
    results = {}
    for label, setter in setters.items():
        status, detail = _outcome(kconf, setter)
        if status != "unset":
            results[label] = (status, detail)
    if len(results) < 2:
        return None

    crashes = {v: d for v, (s, d) in results.items() if s == "crash"}
    if crashes:
        value, detail = next(iter(crashes.items()))
        return ("crash", f"{name}={value} raised {detail}")

    # Reported ahead of everything else: a value the generator cannot read is
    # a configuration the menu offers and the build cannot produce, whatever
    # the rest of the sweep concludes about the knob.
    unread = {v: d for v, (s, d) in results.items() if s == "unread"}
    if unread:
        value, detail = next(iter(unread.items()))
        return ("unread", f"{name}={value} left a symbol unreadable: {detail}")

    accepted = {v: d for v, (s, d) in results.items() if s == "ok"}
    if len(accepted) == 1 and exhaustive:
        survivor = next(iter(accepted))
        return ("pinned", f"{name} only generates at {survivor}")
    # Inert holds under sampling: a value that renders the same bytes as the
    # ones around it cannot be the one that proves the knob does something,
    # while a lone survivor among samples may just mean the rest went untried.
    if len(accepted) == len(results) and len(set(accepted.values())) == 1:
        return ("inert", f"{name} renders the same headers at every value")
    return None


def _setters(kconf: kconfiglib.Kconfig, symbol_name: str, text: str):
    """Assign, and report whether the assignment survived the dependency graph."""

    def apply() -> bool:
        symbol = kconf.syms[symbol_name]
        return symbol.set_value(text) and symbol.str_value == text

    return apply


def main() -> int:
    # Pinned is off by default. Sweeping one symbol at a time holds every other
    # one at its configured value, so a knob constrained by a sibling reads the
    # same as a knob with a single legal value -- and this tree is full of
    # deliberate cross-symbol checks. The verdict needs a human, not a gate.
    report_pinned = "--pinned" in sys.argv[1:]

    kconf = kconfiglib.Kconfig(str(KCONFIG), warn=False)
    kconf.load_config(str(DOT_CONFIG))

    in_choice = {
        symbol.name
        for choice in kconf.unique_choices
        for symbol in choice.syms
        if symbol.name
    }
    findings: list[tuple[str, str]] = []

    for choice in kconf.unique_choices:
        members = [s.name for s in choice.syms if s.name]
        if len(members) < 2 or choice.visibility == 0:
            continue
        label = choice.name or f"choice({members[0]}...)"
        finding = _sweep(
            kconf,
            label,
            {name: _setters(kconf, name, "y") for name in members},
            exhaustive=True,
        )
        if finding:
            findings.append(finding)

    for symbol in kconf.unique_defined_syms:
        if (
            not symbol.name
            or symbol.name in in_choice
            or symbol.type in SKIPPED_TYPES
            or symbol.visibility == 0
        ):
            continue
        values, exhaustive = _candidates(symbol)
        finding = _sweep(
            kconf,
            symbol.name,
            {text: _setters(kconf, symbol.name, text) for text in values},
            exhaustive=exhaustive,
        )
        if finding:
            findings.append(finding)

    if not report_pinned:
        findings = [f for f in findings if f[0] != "pinned"]
    if not findings:
        return 0

    print("Kconfig knobs that do not choose anything:", file=sys.stderr)
    for kind, message in sorted(findings):
        print(f"  [{kind}] {message}", file=sys.stderr)
    print(
        "\nAn inert knob either wants deleting, or wants the value it guards "
        "wired through to a header. Unread means the menu offers a setting\n"
        "the generator cannot then read, and crash means a value got past "
        "validation into a traceback. A pinned knob is only pinned while its\n"
        "siblings hold still -- read the rejection before calling it a "
        "constant.",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
