# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi

"""Validate the handbook against the drift `mkdocs build --strict` cannot see.

`--strict` already rejects broken internal links, orphan pages, dangling heading
anchors, and missing snippet *files*. It is blind to three failure modes that
matter for a build guide someone follows with a soldering iron in hand.

Catches:
  - Orphaned transclusion anchors. `--8<-- "path:name"` with a live PATH but a
    deleted `[start:name]` marker transcludes *empty* — the page builds clean
    and silently ships a missing block. snippets' check_paths only proves the
    file exists, never the section inside it.
  - Raw GitHub admonitions (`> [!NOTE]`). These render correctly on github.com
    and as a plain blockquote containing a literal "[!NOTE]" on the site, so
    they survive review by anyone reading the diff instead of the build.
  - Stale Kconfig citations. A `CONFIG_STM32_*` / `CONFIG_ESP32_*` symbol named
    in the docs that no longer exists in config/Kconfig — a renamed tunable
    that sends a reader hunting through menuconfig for something that is gone.
    Scoped to the two 32Raven namespaces so ESP-IDF's own CONFIG_* symbols
    (which live in the submodule, not our tree) don't false-positive.
  - Deselected Kconfig citations. Most pin symbols are Kconfig *choices*, so
    every alternative exists in Kconfig and an existence check proves only that
    a symbol is spellable. A page citing CONFIG_STM32_SPI1_SCK_PIN_PA5 when the
    reference build selects ..._PB3 documents a pin the aircraft does not use,
    and passes an existence check. Citations are therefore checked against
    config/32raven.config for `# CONFIG_X is not set`, which is what the wiring
    page already promises the reader ("pins come from config/32raven.config").
  - Untracked placeholders. A `TBD(#N)` marker whose number has no `### #N —`
    entry in docs/roadmap.md. Placeholders are how an unfinished page admits
    what it is missing; one that points nowhere is just a dead end, and it is
    the failure mode a reader hits hardest because it looks deliberate.

Exit codes:
  0  no errors
  1  at least one error

Run:
  python3 scripts/lint/check_docs.py
"""

from __future__ import annotations

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
DOCS = ROOT / "docs"
KCONFIG = ROOT / "config" / "Kconfig"
DEFCONFIG = ROOT / "config" / "32raven.config"
ROADMAP = DOCS / "roadmap.md"
BOM = DOCS / "build" / "bom.md"

# pymdownx.snippets section syntax: --8<-- "relative/path.hpp:anchor_name"
SNIPPET_REF = re.compile(r'--8<--\s+"([^"]+)"')
# GitHub's blockquote admonitions; Material uses `!!! note` instead.
GH_ADMONITION = re.compile(
    r"^\s*>\s*\[!(?:NOTE|TIP|IMPORTANT|WARNING|CAUTION)\]"
)
# Only our own namespaces — see the docstring.
CONFIG_REF = re.compile(r"\bCONFIG_((?:STM32|ESP32)_[A-Z0-9_]+)\b")
KCONFIG_DECL = re.compile(
    r"^\s*(?:menuconfig|config)\s+([A-Za-z0-9_]+)\s*$", re.MULTILINE
)
# Placeholder marker in a page, and the roadmap entry that must back it.
TBD_MARKER = re.compile(r"\bTBD\(#(\d+)\)")
ROADMAP_ITEM = re.compile(r"^###\s+#(\d+)\s+—", re.MULTILINE)
# Untracked placeholder — legal, but counted so its growth stays visible.
BARE_TBD = re.compile(r"\bTBD\b(?!\(#\d+\))")
# kconfiglib writes unselected symbols as a comment rather than omitting them.
CONFIG_UNSET = re.compile(
    r"^#\s*CONFIG_([A-Za-z0-9_]+) is not set\s*$", re.MULTILINE
)
# Only symbols whose NAME encodes the value they select are checked for being
# selected — a deselected ..._PIN_PB13 means the page documents the wrong pin.
# Plain feature booleans (..._ACTIVE_LOW) are legitimately named while unset, as
# when a page explains that an option exists; those are existence-checked only.
VALUE_BEARING = re.compile(
    r"(?:_PIN_P[A-Z]\d+|_PORT_[A-Z]|_P[A-Z]\d{1,2}|_BAUD_\d+"
    r"|_PARITY_[A-Z]+|_\d+BITS)$"
)
# A priced BOM row — four cells, the last holding `~$N`. Rows still carrying a
# TBD price simply do not match, so the total covers what is actually known.
BOM_ROW = re.compile(
    r"^\|([^|]*)\|[^|]*\|[^|]*\|\s*~\$([\d.]+)\s*\|\s*$", re.MULTILINE
)
# The stated totals: everything, then everything non-optional.
BOM_TOTAL = re.compile(r"\*\*Running subtotal: ~\$([\d.]+)\*\*")
BOM_REQUIRED = re.compile(r"~\$([\d.]+) without the optional")


def _markdown_files() -> list[pathlib.Path]:
    return sorted(DOCS.rglob("*.md"))


def _line_of(text: str, pos: int) -> int:
    return text[:pos].count("\n") + 1


def _split_ref(ref: str) -> tuple[str, str | None]:
    """Split `path[:anchor]`; a bare path that exists on disk wins."""
    if (ROOT / ref).is_file():
        return ref, None
    path, _, name = ref.rpartition(":")
    return (path, name) if path else (ref, None)


def check_anchors(md: pathlib.Path, text: str) -> list[str]:
    errors: list[str] = []
    for match in SNIPPET_REF.finditer(text):
        line = text[: match.start()].count("\n") + 1
        path, name = _split_ref(match.group(1))
        target = ROOT / path
        if not target.is_file():
            errors.append(
                f"{md.relative_to(ROOT)}:{line}: snippet file not found: {path}"
            )
            continue
        if name is None:
            continue
        if f"--8<-- [start:{name}]" not in target.read_text(encoding="utf-8"):
            errors.append(
                f"{md.relative_to(ROOT)}:{line}: orphan anchor '{name}' — "
                f"{path} has no `--8<-- [start:{name}]` marker"
            )
    return errors


def check_admonitions(md: pathlib.Path, text: str) -> list[str]:
    return [
        f"{md.relative_to(ROOT)}:{n}: GitHub admonition renders as a plain "
        f"blockquote in Material — use `!!! note` / `!!! warning`"
        for n, line in enumerate(text.splitlines(), start=1)
        if GH_ADMONITION.match(line)
    ]


def check_config_refs(
    md: pathlib.Path, text: str, known: set[str], unset: set[str]
) -> list[str]:
    errors: list[str] = []
    for match in CONFIG_REF.finditer(text):
        symbol = match.group(1)
        if symbol not in known:
            errors.append(
                f"{md.relative_to(ROOT)}:{_line_of(text, match.start())}: "
                f"CONFIG_{symbol} is not declared in config/Kconfig — "
                "renamed or "
                f"removed tunable"
            )
        elif symbol in unset and VALUE_BEARING.search(symbol):
            errors.append(
                f"{md.relative_to(ROOT)}:{_line_of(text, match.start())}: "
                f"CONFIG_{symbol} exists but is NOT SELECTED in "
                "config/32raven.config "
                "— the docs describe a pin or option the reference "
                "build does not use"
            )
    return errors


def check_tbd_markers(
    md: pathlib.Path, text: str, tracked: set[str]
) -> list[str]:
    errors: list[str] = []
    for match in TBD_MARKER.finditer(text):
        item = match.group(1)
        if item in tracked:
            continue
        errors.append(
            f"{md.relative_to(ROOT)}:{_line_of(text, match.start())}: "
            f"TBD(#{item}) has no `### #{item} — ` entry in docs/roadmap.md — "
            f"untracked placeholder"
        )
    return errors


def check_bom_total(md: pathlib.Path, text: str) -> list[str]:
    """The BOM's stated subtotals must equal the priced rows above them.

    Prices stay hand-written so the page remains plain, editable markdown —
    this only checks the arithmetic, so a row revised months from now cannot
    leave a stale total sitting under it.
    """
    rows = [(part, float(usd)) for part, usd in BOM_ROW.findall(text)]
    if not rows:
        return [
            f"{md.relative_to(ROOT)}: no priced rows matched — "
            "the table format changed"
        ]

    expected = {
        "Running subtotal": (BOM_TOTAL, round(sum(usd for _, usd in rows), 2)),
        "without the optional": (
            BOM_REQUIRED,
            round(
                sum(usd for part, usd in rows if "*optional*" not in part), 2
            ),
        ),
    }

    errors: list[str] = []
    for label, (pattern, total) in expected.items():
        match = pattern.search(text)
        if match is None:
            errors.append(
                f"{md.relative_to(ROOT)}: no `{label}` figure found — "
                f"expected ~${total:.2f} from {len(rows)} priced row(s)"
            )
        elif round(float(match.group(1)), 2) != total:
            errors.append(
                f"{md.relative_to(ROOT)}:{_line_of(text, match.start())}: "
                f"`{label}` says ~${float(match.group(1)):.2f} "
                "but the rows sum to "
                f"~${total:.2f} — a price changed and the total did not"
            )
    return errors


def main() -> int:
    if not DOCS.is_dir():
        print(f"check_docs: no docs/ directory at {DOCS}", file=sys.stderr)
        return 1

    known = set(KCONFIG_DECL.findall(KCONFIG.read_text(encoding="utf-8")))
    unset = (
        set(CONFIG_UNSET.findall(DEFCONFIG.read_text(encoding="utf-8")))
        if DEFCONFIG.is_file()
        else set()
    )
    tracked = (
        set(ROADMAP_ITEM.findall(ROADMAP.read_text(encoding="utf-8")))
        if ROADMAP.is_file()
        else set()
    )

    errors: list[str] = []
    bare_tbd = 0
    files = _markdown_files()
    for md in files:
        text = md.read_text(encoding="utf-8")
        errors += check_anchors(md, text)
        errors += check_admonitions(md, text)
        errors += check_config_refs(md, text, known, unset)
        errors += check_tbd_markers(md, text, tracked)
        if md == BOM:
            errors += check_bom_total(md, text)
        bare_tbd += len(BARE_TBD.findall(text))

    if errors:
        for err in errors:
            print(f"error: {err}", file=sys.stderr)
        print(
            f"\ncheck_docs: {len(errors)} error(s) across {len(files)} page(s)",
            file=sys.stderr,
        )
        return 1

    # Bare TBD is a deliberate, legal form for short-lived placeholders, so it
    # is counted rather than rejected — an unchecked marker that nobody ever
    # sees is how a placeholder quietly becomes permanent.
    print(
        f"check_docs: {len(files)} page(s) OK "
        f"(anchors, admonitions, Kconfig refs, {len(tracked)} roadmap item(s))"
        + (f"; {bare_tbd} untracked TBD placeholder(s)" if bare_tbd else "")
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
