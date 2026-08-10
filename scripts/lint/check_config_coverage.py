#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi
# /// script
# dependencies = [
#     "libclang",
# ]
# ///

"""Find Config fields the generator never assigns.

A generated config initialises a driver's Config with designated initialisers.
Leave one out and the field silently takes the default member initialiser in
the driver header -- a value that looks configurable, is a live input, and
appears in no menu. `esc_ms` reached the MAVLink scheduler that way.

No compiler diagnostic covers this. -Wmissing-field-initializers is silent the
moment the struct has default member initialisers, which every Config here
does, so the omission is invisible to the build:

    struct P { int a = 0; int esc = 2000; };
    inline constexpr P k = { .a = 1 };      // esc silently 2000, no warning

Field names come from clang's AST, so nested and inherited members resolve
properly. Designators come from the initialiser's own tokens, because the
Python bindings do not expose designated-initialiser nodes. A config this
cannot read -- initialised positionally, or an array of configs -- fails as
[unchecked] rather than passing, because a check that stays quiet about what
it skipped reads exactly like a check that found nothing wrong.

Needs a compilation database. Without one it skips, like check_unused_includes.
"""
from __future__ import annotations

import json
import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent.parent

# Generated headers and the build whose flags parse them.
TARGETS = (
    ("esp32/main/esp32_config.hpp", "build/Ninja/esp32/compile_commands.json"),
    ("stm32/Drivers/stm32_config.hpp", "build/Ninja/stm32/compile_commands.json"),
)

MAX_DEPTH = 6


def _flags_for(db_path: pathlib.Path, header: pathlib.Path) -> list[str] | None:
    """Compile flags from a TU that can actually resolve this header.

    A header has no entry of its own, so it borrows a translation unit's flags.
    It has to borrow from one of ours: the ESP-IDF components dominate the
    database and are built with an include path that does not reach the driver
    headers, so flags taken from the first entry resolve nothing.
    """
    try:
        entries = json.loads(db_path.read_text())
    except (OSError, ValueError):
        return None
    firmware_dir = str(REPO / header.parts[len(REPO.parts)])
    ours = [
        e
        for e in entries
        if e.get("file", "").startswith(firmware_dir)
        and e.get("file", "").endswith((".cpp", ".cc", ".cxx"))
    ]
    for entry in ours or entries:
        source = entry.get("file", "")
        if not source.endswith((".cpp", ".cc", ".cxx")):
            continue
        args = entry.get("arguments")
        if args is None:
            args = entry.get("command", "").split()
        keep: list[str] = []
        skip_next = False
        for arg in args[1:]:
            if skip_next:
                skip_next = False
                continue
            if arg in ("-c", "-o", "-MF", "-MT", "-MD", "-MMD"):
                skip_next = arg in ("-o", "-MF", "-MT")
                continue
            if arg == source or arg.endswith(".obj") or arg.endswith(".o"):
                continue
            keep.append(arg)
        # The cross toolchain's own target and headers. Without them the unit
        # does not parse, and a header that does not parse yields no variables
        # at all -- which reads as "everything is assigned" rather than as a
        # failure. check_unused_includes learned this the same way.
        compiler = args[0] if args else ""
        keep = [a for a in keep if not a.startswith(("-m", "-f"))]
        keep += _toolchain_args(compiler)
        keep += ["-Wno-everything", f"-I{entry.get('directory', '.')}"]
        return keep
    return None


def _toolchain_args(compiler: str) -> list[str]:
    import subprocess

    args: list[str] = []
    name = pathlib.Path(compiler).name
    if "-" in name:  # arm-none-eabi-g++, riscv32-esp-elf-g++
        args.append("--target=" + name.rsplit("-", 1)[0])
    try:
        probe = subprocess.run(
            [compiler, "-E", "-x", "c++", "-", "-v"],
            input="",
            capture_output=True,
            text=True,
        )
    except OSError:
        return args
    collecting = False
    for line in probe.stderr.split("\n"):
        if line.startswith("#include <...>"):
            collecting = True
            continue
        if line.startswith("End of search list"):
            break
        if collecting and line.startswith(" "):
            args += ["-isystem", line.strip()]
    return args


def _field_paths(cursor, clang, depth: int = 0) -> list[str]:
    """Every leaf field path of a record type, as dotted names."""
    import clang.cindex as ci

    if depth > MAX_DEPTH:
        return []
    paths: list[str] = []
    for child in cursor.get_children():
        if child.kind != ci.CursorKind.FIELD_DECL:
            continue
        name = child.spelling
        decl = child.type.get_declaration()
        nested = (
            decl
            if decl is not None
            and decl.kind
            in (ci.CursorKind.STRUCT_DECL, ci.CursorKind.CLASS_DECL, ci.CursorKind.UNION_DECL)
            and decl.is_definition()
            else None
        )
        sub = _field_paths(nested, clang, depth + 1) if nested is not None else []
        if sub:
            paths.extend(f"{name}.{s}" for s in sub)
        else:
            paths.append(name)
    return paths


def _designator_paths(tokens: list[str]) -> tuple[set[str], set[str]]:
    """Dotted designator paths in an aggregate initialiser's token stream.

    Tracks brace depth so `.tx = { .periods = { .hb_ms = ...` yields
    `tx.periods.hb_ms` rather than three unrelated names.

    Returns (braced, whole), split by what the designator's value was, because
    the two cover different things. `.tx = {...}` opens a sub-aggregate whose
    own leaves can still be left out, so it covers only itself. `.tx = kShared`
    copies the sub-object entire, so it covers every leaf beneath it too.
    """
    braced: set[str] = set()
    whole: set[str] = set()
    stack: list[str] = []
    i = 0
    while i < len(tokens):
        tok = tokens[i]
        if tok == "." and i + 2 < len(tokens) and tokens[i + 2] == "=":
            name = tokens[i + 1]
            path = ".".join([s for s in stack if s] + [name])
            i += 3
            if i < len(tokens) and tokens[i] == "{":
                # A braced value need not carry designators of its own -- an
                # array of positional values is still an assignment -- so the
                # field is recorded before descending into it.
                braced.add(path)
                stack.append(name)
                i += 1
            else:
                whole.add(path)
            continue
        if tok == "{":
            # Braces that carry no designator of their own still nest, so they
            # sit on the stack as blanks and drop out of the path.
            stack.append("")
        elif tok == "}":
            if stack:
                stack.pop()
        i += 1
    return (
        {p for p in braced if not p.startswith(".")},
        {p for p in whole if not p.startswith(".")},
    )


def _is_covered(field: str, braced: set[str], whole: set[str]) -> bool:
    """Whether a declared leaf was assigned by any designator in the tokens.

    The prefix walk applies to `whole` only. Extending it to `braced` would
    make every sub-aggregate vouch for leaves it never named, which is the one
    thing this check exists to catch.
    """
    if field in braced or field in whole:
        return True
    parts = field.split(".")
    return any(".".join(parts[:i]) in whole for i in range(1, len(parts)))


def main() -> int:
    try:
        import clang.cindex as ci
    except ImportError:
        print("check-config-coverage: clang bindings unavailable, skipping")
        return 0

    problems: list[str] = []
    skipped: list[str] = []
    per_target: dict[str, int] = {}

    for header_rel, db_rel in TARGETS:
        per_target.setdefault(header_rel, 0)
        header = REPO / header_rel
        db = REPO / db_rel
        if not header.exists() or not db.exists():
            skipped.append(f"{header_rel} (no header or compilation database)")
            continue
        flags = _flags_for(db, header)
        if flags is None:
            skipped.append(f"{header_rel} (no usable compile flags)")
            continue

        index = ci.Index.create()
        try:
            tu = index.parse(
                str(header), args=flags, options=ci.TranslationUnit.PARSE_INCOMPLETE
            )
        except ci.TranslationUnitLoadError as exc:
            skipped.append(f"{header_rel} (parse failed: {exc})")
            continue

        for cursor in tu.cursor.get_children():
            if cursor.kind != ci.CursorKind.VAR_DECL:
                continue
            if not str(cursor.location.file or "").endswith(header.name):
                continue
            if cursor.type.kind == ci.TypeKind.CONSTANTARRAY:
                element = cursor.type.element_type.get_declaration()
                if element is not None and _field_paths(element, ci):
                    problems.append(
                        f"[unchecked] {header_rel}: {cursor.spelling} is an array "
                        f"of {element.spelling}; this check reads one initialiser "
                        "per variable and does not walk array elements"
                    )
                continue
            decl = cursor.type.get_declaration()
            if decl is None or not decl.is_definition():
                continue
            declared = _field_paths(decl, ci)
            if not declared:
                continue
            tokens = [t.spelling for t in cursor.get_tokens()]
            braced, whole = _designator_paths(tokens)
            if not braced and not whole:
                # No designators at all. Copy-initialised from another constant
                # assigns every field, and that constant is checked in its own
                # right; a brace without designators is exactly the silent
                # default this check exists to catch, so it cannot be waved past.
                if "{" in tokens:
                    problems.append(
                        f"[unchecked] {header_rel}: {cursor.spelling} is "
                        f"initialised without designators, so none of "
                        f"{decl.spelling}'s fields were checked"
                    )
                else:
                    per_target[header_rel] += 1
                continue
            per_target[header_rel] += 1
            missing = [f for f in declared if not _is_covered(f, braced, whole)]
            for field in missing:
                problems.append(
                    f"[unset]    {header_rel}: {cursor.spelling}.{field} is never "
                    f"assigned, so it silently takes {decl.spelling}'s default"
                )

    for note in skipped:
        print(f"check-config-coverage: skipped {note}", file=sys.stderr)

    # A header that yields nothing looks identical to a header with nothing
    # wrong. Say which one it was, because silent zero coverage is how this
    # check would rot into an unconditional pass.
    barren = [h for h, n in per_target.items() if n == 0 and not any(h in s for s in skipped)]
    for header_rel in barren:
        problems.append(
            f"[blind]    {header_rel} parsed but yielded no aggregate configs, "
            "so nothing in it was checked"
        )

    if not problems:
        print(
            "check-config-coverage: "
            + ", ".join(f"{h.rsplit('/', 1)[-1]}={n}" for h, n in per_target.items())
            + " configs fully assigned"
        )
        return 0

    print("Generated config coverage:", file=sys.stderr)
    for problem in problems:
        print(f"  {problem}", file=sys.stderr)
    print(
        "\nEither give the field a generator source, or drop it from the struct. "
        "A Config field with no config is a knob that does not exist.",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
