#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi
# /// script
# dependencies = [
#     "jinja2",
#     "kconfiglib",
# ]
# ///

"""Detect generator context keys no template reads.

StrictUndefined already fails a render that asks for a key the context does not
supply. The reverse renders clean: a context can carry a key no template
mentions, and nothing says so.

That silence costs more than a stray dict entry. Building the key reads its
Kconfig symbols through `kconfig_gen.sym`, which is what check_config_staleness
counts as consumption, and `choice_value` marks a whole table read at once. A
dead context key therefore keeps its symbols looking live while nothing they
feed ever reaches a header.

Measured by rendering, not by scanning. Keys are reached through loop variables
and runtime subscripts -- `rate_controller[axis_name]` -- that no text search
resolves. Each generator runs against the real Kconfig and `.config` with its
output redirected to a temporary directory, and every context dict below the
root is replaced by a proxy that records the lookups. Root names come from the
parsed template instead, since Jinja resolves those against its own context
before any of ours.

Blind spot: a key read only inside a branch this `.config` does not take counts
as unread. There is one such branch -- `p.adc_channel` under
`{% if p.adc_const %}` in stm32_config.hpp.j2 -- and it stays covered for as
long as one pin is an ADC pin.

Run:
  uv run --quiet --script scripts/lint/check_context_keys.py
"""
from __future__ import annotations

import importlib
import pathlib
import sys
import tempfile
from typing import Callable

REPO = pathlib.Path(__file__).resolve().parent.parent.parent
KCONFIG = REPO / "config/Kconfig"
DOT_CONFIG = REPO / "config/32raven.config"
EE_CONFIG = REPO / "config/ee.toml"

sys.path.insert(0, str(REPO / "scripts"))

import jinja2  # noqa: E402
import jinja2.meta  # noqa: E402

# Every generator, invoked the way its own CMake rule invokes it. Calling main()
# rather than the context functions keeps validation and argument handling in
# the picture, so a context this check renders is the context the build renders.
GENERATORS: dict[str, Callable[[pathlib.Path], list[str]]] = {
    "generate_common_config": lambda out: [
        "--kconfig", str(KCONFIG),
        "--config", str(DOT_CONFIG),
        "--out", str(out / "common_config.hpp"),
    ],
    "generate_stm32_config": lambda out: [
        "--kconfig", str(KCONFIG),
        "--config", str(DOT_CONFIG),
        "--runtime-out", str(out / "stm32_config.hpp"),
        "--limits-out", str(out / "stm32_limits.hpp"),
    ],
    "generate_esp32_config": lambda out: [
        "--kconfig", str(KCONFIG),
        "--config", str(DOT_CONFIG),
        "--runtime-out", str(out / "esp32_config.hpp"),
        "--limits-out", str(out / "esp32_limits.hpp"),
    ],
    "generate_ee_schema": lambda out: [
        "--config", str(EE_CONFIG),
        "--out", str(out / "ee_schema.hpp"),
    ],
}


class _Recorder:
    """The paths one template's context offers, and the ones it was asked for."""

    def __init__(self) -> None:
        self.produced: set[str] = set()
        self.read: set[str] = set()

    def wrap(self, value: object, path: str) -> object:
        if isinstance(value, dict):
            tracked = {}
            for key, item in value.items():
                child = f"{path}.{key}" if path else str(key)
                self.produced.add(child)
                tracked[key] = self.wrap(item, child)
            return _TrackedDict(tracked, path, self)
        if isinstance(value, (list, tuple)):
            # One path for the whole sequence: the entries of a rendered table
            # carry the same keys, so a key read on any entry is a key in use.
            return [self.wrap(item, f"{path}[]") for item in value]
        return value


class _TrackedDict(dict):
    """A context dict that records which keys a template looks up.

    Jinja resolves `foo.bar` with getattr first and falls back to `foo["bar"]`,
    so overriding __getitem__ catches both spellings.
    """

    def __init__(self, data: dict, path: str, recorder: _Recorder) -> None:
        super().__init__(data)
        self._path = path
        self._recorder = recorder

    def __getitem__(self, key: object) -> object:
        self._recorder.read.add(f"{self._path}.{key}" if self._path else str(key))
        return super().__getitem__(key)


_RECORDERS: dict[str, _Recorder] = {}
_ORIGINAL_RENDER = jinja2.Template.render


def _root_names(template: jinja2.Template) -> set[str]:
    """The root context names a template mentions.

    `render(**context)` unpacks into Jinja's own context, so a root lookup never
    reaches the dict we were handed. The parsed template answers instead: every
    name it uses without declaring has to come from the context.
    """
    env = template.environment
    source = env.loader.get_source(env, template.name)[0]
    return jinja2.meta.find_undeclared_variables(env.parse(source))


def _recording_render(self: jinja2.Template, *args: object, **kwargs: object) -> str:
    context = dict(*args, **kwargs)
    recorder = _RECORDERS.setdefault(self.name, _Recorder())
    recorder.produced.update(context)
    recorder.read |= _root_names(self)
    wrapped = {key: recorder.wrap(value, key) for key, value in context.items()}
    return _ORIGINAL_RENDER(self, **wrapped)


def main() -> int:
    jinja2.Template.render = _recording_render
    saved_argv = sys.argv
    try:
        with tempfile.TemporaryDirectory() as tmp:
            out = pathlib.Path(tmp)
            for module_name, argv in GENERATORS.items():
                sys.argv = [module_name, *argv(out)]
                importlib.import_module(module_name).main()
    finally:
        sys.argv = saved_argv
        jinja2.Template.render = _ORIGINAL_RENDER

    unread = [
        (template, path)
        for template, recorder in sorted(_RECORDERS.items())
        for path in sorted(recorder.produced - recorder.read)
    ]
    if not unread:
        return 0

    print("Unread generator context keys:", file=sys.stderr)
    for template, path in unread:
        print(f"  {template}: {path}", file=sys.stderr)
    print(
        "\nEach one is a value a generator builds and no template renders. Drop "
        "it from the context, and the Kconfig symbols it read stop counting as\n"
        "consumed -- check_config_staleness will then say whether they are dead "
        "too. If the key is meant to be rendered, the template is missing it.",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
