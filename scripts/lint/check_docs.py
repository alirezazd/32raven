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
ROADMAP = DOCS / "roadmap.md"

# pymdownx.snippets section syntax: --8<-- "relative/path.hpp:anchor_name"
SNIPPET_REF = re.compile(r'--8<--\s+"([^"]+)"')
# GitHub's blockquote admonitions; Material uses `!!! note` instead.
GH_ADMONITION = re.compile(r"^\s*>\s*\[!(?:NOTE|TIP|IMPORTANT|WARNING|CAUTION)\]")
# Only our own namespaces — see the docstring.
CONFIG_REF = re.compile(r"\bCONFIG_((?:STM32|ESP32)_[A-Z0-9_]+)\b")
KCONFIG_DECL = re.compile(r"^\s*(?:menuconfig|config)\s+([A-Za-z0-9_]+)\s*$", re.M)
# Placeholder marker in a page, and the roadmap entry that must back it.
TBD_MARKER = re.compile(r"\bTBD\(#(\d+)\)")
ROADMAP_ITEM = re.compile(r"^###\s+#(\d+)\s+—", re.M)


def _markdown_files() -> list[pathlib.Path]:
    return sorted(DOCS.rglob("*.md"))


def _split_ref(ref: str) -> tuple[str, str | None]:
    """Split `path[:anchor]`. A bare path that exists on disk wins over a split."""
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
            errors.append(f"{md.relative_to(ROOT)}:{line}: snippet file not found: {path}")
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


def check_config_refs(md: pathlib.Path, text: str, known: set[str]) -> list[str]:
    errors: list[str] = []
    for match in CONFIG_REF.finditer(text):
        symbol = match.group(1)
        if symbol in known:
            continue
        line = text[: match.start()].count("\n") + 1
        errors.append(
            f"{md.relative_to(ROOT)}:{line}: CONFIG_{symbol} is not declared in "
            f"config/Kconfig — renamed or removed tunable"
        )
    return errors


def check_tbd_markers(md: pathlib.Path, text: str, tracked: set[str]) -> list[str]:
    errors: list[str] = []
    for match in TBD_MARKER.finditer(text):
        item = match.group(1)
        if item in tracked:
            continue
        line = text[: match.start()].count("\n") + 1
        errors.append(
            f"{md.relative_to(ROOT)}:{line}: TBD(#{item}) has no `### #{item} — ` "
            f"entry in docs/roadmap.md — untracked placeholder"
        )
    return errors


def main() -> int:
    if not DOCS.is_dir():
        print(f"check_docs: no docs/ directory at {DOCS}", file=sys.stderr)
        return 1

    known = set(KCONFIG_DECL.findall(KCONFIG.read_text(encoding="utf-8")))
    tracked = (
        set(ROADMAP_ITEM.findall(ROADMAP.read_text(encoding="utf-8")))
        if ROADMAP.is_file()
        else set()
    )

    errors: list[str] = []
    files = _markdown_files()
    for md in files:
        text = md.read_text(encoding="utf-8")
        errors += check_anchors(md, text)
        errors += check_admonitions(md, text)
        errors += check_config_refs(md, text, known)
        errors += check_tbd_markers(md, text, tracked)

    if errors:
        for err in errors:
            print(f"error: {err}", file=sys.stderr)
        print(f"\ncheck_docs: {len(errors)} error(s) across {len(files)} page(s)", file=sys.stderr)
        return 1

    print(
        f"check_docs: {len(files)} page(s) OK "
        f"(anchors, admonitions, Kconfig refs, {len(tracked)} roadmap item(s))"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
