#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi
# /// script
# dependencies = [
#     "libclang",
# ]
# ///

"""Find generated config headers that reach into the firmware.

A generated header is data: it instantiates the drivers' Config structs and
names their enumerators, and that is the whole of what it needs from the
source tree. When it also calls a firmware function or reads a firmware
constant, the dependency runs backwards -- config depending on behaviour --
and whatever check it carries belongs with the code that owns the behaviour,
where the compiler, clang-tidy and an editor can see it. The link-budget
checks lived in the templates that way until they moved beside their ladders;
this keeps them from drifting back.

The line is naming against computing. A generated header may name what the
firmware defines -- its types, enumerators, fields, and constants such as a
pin or an alternate function, which is the only way a pin map can be said at
all. It may not compute with the firmware, and it may not check itself against
the firmware. So, for every expression in a generated header that resolves to
a declaration in this tree -- outside third_party, the vendored CMSIS and the
generated directories:

  [reach] a call to a firmware function, anywhere in the header
  [reach] a firmware constant read inside a static_assert -- a check that
          belongs with the code that owns the constant. A static_assert that
          compares the header's own values against each other is fine.

An exception has to be mandatory and make sense: a name the config could not
express any other way. Those go in ALLOWED by qualified name, with the reason
beside them. Nothing qualifies today.

Needs a compilation database. Without one it skips, like the other
compilation-database checks, which is why build.yml runs it after each build.
"""

from __future__ import annotations

import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent.parent

sys.path.insert(0, str(REPO / "scripts/lint"))

import check_config_coverage as coverage  # noqa: E402

# Qualified name -> why the config may name it.
ALLOWED: dict[str, str] = {}

# Every generated header, each parsed with the flags of the build that
# includes it. The shared and limits headers carry only numbers today, and
# are here so that stays true.
_ESP32_DB = coverage.TARGETS[0][1]
_STM32_DB = coverage.TARGETS[1][1]
TARGETS = (
    *coverage.TARGETS,
    ("build/Ninja/generated/common_config.hpp", _STM32_DB),
    ("build/Ninja/stm32/generated/stm32_limits.hpp", _STM32_DB),
    ("build/Ninja/stm32/generated/ee_schema.hpp", _STM32_DB),
    ("build/Ninja/esp32/esp-idf/main/generated/esp32_limits.hpp", _ESP32_DB),
)

# Trees that are not the firmware: vendored, third-party, or generated.
NOT_FIRMWARE = (("third_party",), ("build",), ("stm32", "lib"))


def _is_firmware(path: str) -> bool:
    if not path:
        return False
    try:
        parts = pathlib.Path(path).resolve().relative_to(REPO).parts
    except ValueError:
        return False
    return not any(parts[: len(p)] == p for p in NOT_FIRMWARE)


def _qualified(ci, cursor) -> str:
    parts = []
    while cursor is not None and cursor.kind != ci.CursorKind.TRANSLATION_UNIT:
        if cursor.spelling:
            parts.append(cursor.spelling)
        cursor = cursor.semantic_parent
    return "::".join(reversed(parts))


def _findings(ci, tu, header: pathlib.Path) -> list[str]:
    reference_kinds = (
        ci.CursorKind.CALL_EXPR,
        ci.CursorKind.DECL_REF_EXPR,
        ci.CursorKind.MEMBER_REF_EXPR,
    )
    function_kinds = (
        ci.CursorKind.FUNCTION_DECL,
        ci.CursorKind.CXX_METHOD,
        ci.CursorKind.FUNCTION_TEMPLATE,
        ci.CursorKind.CONVERSION_FUNCTION,
    )
    resolved = header.resolve()
    seen: set[tuple[int, str]] = set()
    findings: list[str] = []

    def in_header(cursor) -> bool:
        f = cursor.location.file
        return f is not None and pathlib.Path(f.name).resolve() == resolved

    def check(cursor, checking: bool) -> None:
        for node in cursor.walk_preorder():
            # The Python bindings lag libclang: a cursor kind they do not
            # know raises here, and none of those kinds is a reference.
            try:
                kind = node.kind
            except ValueError:
                continue
            if kind not in reference_kinds or not in_header(node):
                continue
            ref = node.referenced
            if ref is None:
                continue
            try:
                ref_kind = ref.kind
            except ValueError:
                continue
            is_call = ref_kind in function_kinds
            is_constant = ref_kind == ci.CursorKind.VAR_DECL
            if not is_call and not (is_constant and checking):
                continue
            defined_in = ref.location.file
            if defined_in is None or not _is_firmware(defined_in.name):
                continue
            name = _qualified(ci, ref)
            line = node.location.line
            if name in ALLOWED or (line, name) in seen:
                continue
            seen.add((line, name))
            defined = pathlib.Path(defined_in.name).resolve().relative_to(REPO)
            verb = "calls" if is_call else "checks against"
            findings.append(
                f"[reach]    {header.relative_to(REPO)}:{line}: {verb} "
                f"{name} ({defined}:{ref.location.line})"
            )

    def descend(cursor) -> None:
        for child in cursor.get_children():
            if not in_header(child):
                continue
            try:
                kind = child.kind
            except ValueError:
                continue
            if kind in (ci.CursorKind.NAMESPACE, ci.CursorKind.LINKAGE_SPEC):
                descend(child)
            elif kind == ci.CursorKind.STATIC_ASSERT:
                check(child, checking=True)
            else:
                check(child, checking=False)

    descend(tu.cursor)
    return findings


def main() -> int:
    try:
        import clang.cindex as ci
    except ImportError:
        print("check-config-reach: clang bindings unavailable, skipping")
        return 0

    problems: list[str] = []
    checked: list[str] = []
    for header_rel, db_rel in TARGETS:
        header = REPO / header_rel
        db = REPO / db_rel
        if not header.exists() or not db.exists():
            print(
                f"check-config-reach: skipped {header_rel} (no header or "
                "compilation database)",
                file=sys.stderr,
            )
            continue
        flags = coverage._flags_for(db, header)
        if flags is None:
            print(
                f"check-config-reach: skipped {header_rel} (no usable compile "
                "flags)",
                file=sys.stderr,
            )
            continue
        tu = ci.Index.create().parse(
            str(header), args=flags, options=ci.TranslationUnit.PARSE_INCOMPLETE
        )
        problems += _findings(ci, tu, header)
        checked.append(header.name)

    if not problems:
        print(
            "check-config-reach: "
            + (", ".join(checked) if checked else "nothing")
            + " only name the firmware, and check nothing against it"
        )
        return 0

    print("Generated config reaching into the firmware:", file=sys.stderr)
    for problem in problems:
        print(f"  {problem}", file=sys.stderr)
    print(
        "\nA check that needs the firmware's model, or its constants, belongs "
        "in the .cpp that owns them, asserting on the generated values; the "
        "generated header names things and checks itself against itself.",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
