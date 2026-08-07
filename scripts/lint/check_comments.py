#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi

"""Check comments for the shapes that are objectively wrong, not merely bad.

Whether a comment earns its place is a review question -- no linter can tell
that a line eliminates confusion rather than adding to it. What a linter can
tell is that a comment is empty, is decoration, is code someone stopped
compiling, or is a marker nobody can act on. Those are the rules here.

Rules, all whole-line or line-local:

  banner        A rule of dashes, equals signs or box-drawing characters,
                either bare or wrapped around a label. It never moves when the
                code under it does, so it ends up separating things it no
                longer describes -- and the labelled form is the worse of the
                two, since the label is the part that goes stale.
  todo-owner    TODO/FIXME/XXX/HACK with no owner or issue in parentheses. An
                unattributed marker is a wish, not a task.
  commented-code
                A statement or #include commented out rather than deleted. Git
                remembers it; the file should not. Indented lines are exempt,
                since a usage example in a doc block is deliberately code.
  attribution   Tool or assistant attribution. Authorship belongs in git, and
                this project does not carry generated-by markers.
  format-guard  clang-format off with no matching on, which silently disables
                formatting for the rest of the file.
  nolint-reason NOLINT with neither a check name nor a reason. A suppression
                nobody can evaluate outlives whatever justified it.
  box-drawing   Box-drawing characters used as decoration. Units and symbols
                (us, deg, ohm, squared) are untouched -- those carry meaning.

Off by default under --strict, because these are preferences rather than
defects and reasonable code breaks them on purpose:

  restates      A one-line comment above a declaration that says only what the
                declaration already says. Often wanted anyway -- an editor shows
                it on hover, where the signature is not in view.
  empty         A comment with no content. Usually a paragraph break inside a
                block, which is legitimate formatting.
  narration     Wording that describes the code's past rather than its present.
                The word list is a guess, and prose about protocol history is a
                fair use of every word on it.

Deliberately absent: "comment narrates history". It is a real problem with no
rule that separates it from a legitimate short clarifier.

Files come from `git ls-files`, so build output and untracked scratch are out of
scope by construction. Exemptions live in comment_exceptions.txt, one
`path:rule` per line with a reason, for vendored or generated sources.

Run:
  uv run --quiet --script scripts/lint/check_comments.py
  uv run --quiet --script scripts/lint/check_comments.py --fix
  uv run --quiet --script scripts/lint/check_comments.py --strict
"""

from __future__ import annotations

import argparse
import pathlib
import re
import subprocess
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parents[2]
EXCEPTIONS_FILE = pathlib.Path(__file__).resolve().parent / "comment_exceptions.txt"

SOURCE_SUFFIXES = {".c", ".h", ".cpp", ".hpp", ".cc", ".cxx"}

# Third-party or generated sources. Not ours to restyle: a vendored file keeps
# upstream's conventions, and a generated one is rewritten by its generator.
EXEMPT_PREFIXES = (
    "third_party/",
    "stm32/lib/CMSIS_F4xx/",
    "esp32/ui/assets/bitmap/",
)

# Two shapes. Bare: a rule of six or more and nothing else -- six sits above
# any plausible run in prose ("----" as an em-dash stand-in). Labelled: a run
# of three or more on both sides of a label, which is the form banners actually
# take. The pair of anchors is what makes the shorter run safe, since prose
# that uses -- mid-sentence never also opens and closes with it.
FILL = r"[-=_*#~─━│┃—]"
BANNER_RE = re.compile(
    rf"^\s*//\s*(?:{FILL}{{6,}}|{FILL}{{3,}}\s*\S.*?\s*{FILL}{{3,}})\s*$"
)
EMPTY_RE = re.compile(r"^\s*//+\s*$")
TODO_RE = re.compile(r"//\s*(TODO|FIXME|XXX|HACK)(?!\s*\()", re.IGNORECASE)
# Assistant and tool attribution. Authorship belongs in git history, not in the
# source, and a generated-by marker is stale the moment a human edits the line.
# The vendor list is the assistants in common use, not a complete one -- add to
# it rather than loosening it, so the rule stays free of false positives.
ATTRIBUTION_RE = re.compile(
    r"(co-authored-by"
    r"|generated (with|by) (claude|chatgpt|gpt|copilot|gemini|cursor|codex|ai)"
    r"|written by (an? )?ai"
    r"|ai[- ]generated"
    r"|\b(claude code|github copilot|copilot chat|chatgpt|openai codex"
    r"|amazon q|codewhisperer|windsurf|codeium|tabnine|sourcegraph cody"
    r"|aider|devin|cline|roo code|continue\.dev)\b"
    r"|🤖)",
    re.IGNORECASE,
)

# Conservative on purpose: a commented line that ends in a semicolon and also
# carries a call or an assignment. Prose rarely does both, and the rule is a
# ratchet from zero rather than a cleanup, so a miss costs nothing.
# A call or an assignment where the identifier is the first thing on the line.
# Prose that happens to end in ");" -- a sentence about a register, say -- puts a
# space after its first word, so it never matches.
COMMENTED_CODE_RE = re.compile(
    r"^\s*//[ ]?(#include\s*[<\"]"
    r"|[A-Za-z_][\w:.<>]*\s*\(.*\)\s*;\s*$"
    r"|[A-Za-z_][\w:.\[\]]*\s*=[^=].*;\s*$)"
)

# Words that carry no information about what a declaration does, so a comment
# made only of these plus the identifier's own words is saying nothing new.
FILLER_WORDS = {
    "a", "an", "the", "for", "of", "to", "is", "this", "it", "and", "or",
    "returns", "return", "sets", "set", "gets", "get", "getter", "setter",
    "in", "on", "at", "by", "with", "from", "value", "function", "method",
}
DECLARATION_RE = re.compile(r"\b([A-Za-z_][A-Za-z0-9_]*)\s*[({;=]")
WORD_RE = re.compile(r"[A-Z]?[a-z]+|[A-Z]+(?![a-z])")


def restates_declaration(comment: str, following: str) -> bool:
    words = {w.lower() for w in WORD_RE.findall(comment)} - FILLER_WORDS
    if not words:
        return False
    match = DECLARATION_RE.search(following)
    if match is None:
        return False
    identifier = {w.lower() for w in WORD_RE.findall(match.group(1))}
    return words <= identifier

# The optional suffix has to be consumed before the lookahead, or NOLINTBEGIN
# matches as a bare NOLINT followed by a "B".
NOLINT_RE = re.compile(r"//\s*NOLINT(?:NEXTLINE|BEGIN|END)?(?![A-Z(:])")
BOX_DRAWING_RE = re.compile(r"^\s*(//|\*).*[\u2500-\u257F]")
NARRATION_RE = re.compile(
    r"//.*\b(previously|used to|no longer|formerly|originally|we changed|"
    r"instead of before|was renamed|old code|before this)\b",
    re.IGNORECASE,
)

FORMAT_OFF = "clang-format off"
FORMAT_ON = "clang-format on"


def selected_sources(explicit: list[str]) -> list[pathlib.Path]:
    """The files to check: the ones named, or every tracked source.

    A hook passes the staged paths, so a commit costs one pass over what it
    touched rather than over the tree. Called with nothing -- CI, or by hand --
    it walks `git ls-files`, which is what makes it a repo-wide gate.
    """
    if explicit:
        return [
            pathlib.Path(name)
            for name in explicit
            if name.endswith(tuple(SOURCE_SUFFIXES))
            and not name.startswith(EXEMPT_PREFIXES)
        ]
    return tracked_sources()


def tracked_sources() -> list[pathlib.Path]:
    out = subprocess.run(
        ["git", "ls-files", "-z"],
        cwd=REPO_ROOT,
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    paths = []
    for name in out.split("\0"):
        if not name or not name.endswith(tuple(SOURCE_SUFFIXES)):
            continue
        if name.startswith(EXEMPT_PREFIXES):
            continue
        paths.append(pathlib.Path(name))
    return paths


def load_exceptions() -> set[str]:
    if not EXCEPTIONS_FILE.is_file():
        return set()
    entries = set()
    for line in EXCEPTIONS_FILE.read_text(encoding="utf-8").splitlines():
        line = line.split("#", 1)[0].strip()
        if line:
            entries.add(line)
    return entries


def is_comment(line: str) -> bool:
    return line.lstrip().startswith("//")


def check_file(lines: list[str], strict: bool) -> list[tuple[int, str, str]]:
    """Returns (line_number, rule, text) for each violation."""
    findings: list[tuple[int, str, str]] = []
    format_off_line = 0

    for number, line in enumerate(lines, start=1):
        stripped = line.rstrip("\n")
        if FORMAT_OFF in stripped:
            format_off_line = number
        elif FORMAT_ON in stripped:
            format_off_line = 0

        if BANNER_RE.match(stripped):
            findings.append((number, "banner", stripped.strip()))
        elif EMPTY_RE.match(stripped):
            if strict:
                findings.append((number, "empty", stripped.strip()))
        elif COMMENTED_CODE_RE.match(stripped):
            findings.append((number, "commented-code", stripped.strip()))

        if (
            strict
            and is_comment(stripped)
            and number < len(lines)
            and not is_comment(lines[number])
            and not (number > 1 and is_comment(lines[number - 2]))
            and restates_declaration(stripped.split("//", 1)[1], lines[number])
        ):
            findings.append((number, "restates", stripped.strip()))

        if NOLINT_RE.search(stripped):
            findings.append((number, "nolint-reason", stripped.strip()))
        if BOX_DRAWING_RE.match(stripped):
            findings.append((number, "box-drawing", stripped.strip()))
        if strict and NARRATION_RE.search(stripped):
            findings.append((number, "narration", stripped.strip()))

        if TODO_RE.search(stripped):
            findings.append((number, "todo-owner", stripped.strip()))
        if ATTRIBUTION_RE.search(stripped):
            findings.append((number, "attribution", stripped.strip()))

    if format_off_line:
        findings.append((format_off_line, "format-guard", FORMAT_OFF))

    return findings


# Only the rules with one correct fix. Everything else needs a human to say
# what the comment should have been.
FIXABLE = {"banner", "empty"}

# A labelled banner has two defects and only one of them is certain: the rules
# are always wrong, the label may be the sole thing naming a block. Stripping
# to the label is the edit that cannot lose information; whether the label then
# earns its line is a review question.
LABELLED_BANNER_RE = re.compile(rf"^(\s*//\s*){FILL}{{3,}}\s*(\S.*?)\s*{FILL}{{3,}}\s*$")


def fix_file(path: pathlib.Path, lines: list[str], findings) -> int:
    edited = list(lines)
    drop: set[int] = set()
    changed = 0

    for number, rule, _ in findings:
        if rule not in FIXABLE:
            continue
        labelled = LABELLED_BANNER_RE.match(edited[number - 1].rstrip("\n"))
        if rule == "banner" and labelled is not None:
            edited[number - 1] = f"{labelled.group(1)}{labelled.group(2)}\n"
        else:
            drop.add(number)
        changed += 1

    if changed == 0:
        return 0

    path.write_text(
        "".join(
            line for number, line in enumerate(edited, start=1) if number not in drop
        ),
        encoding="utf-8",
    )
    return changed


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--fix",
        action="store_true",
        help="strip banner rules and delete empty comment lines "
        "(other rules are reported only)",
    )
    parser.add_argument(
        "--strict",
        action="store_true",
        help="also flag empty comments and ones that only repeat the declaration",
    )
    parser.add_argument(
        "paths", nargs="*", help="files to check; defaults to every tracked source"
    )
    args = parser.parse_args()

    exceptions = load_exceptions()
    checked = 0
    fixed = 0
    failures: list[str] = []

    for relative in selected_sources(args.paths):
        path = REPO_ROOT / relative
        try:
            lines = path.read_text(encoding="utf-8").splitlines(keepends=True)
        except (OSError, UnicodeDecodeError):
            continue

        checked += 1
        findings = [
            (number, rule, text)
            for number, rule, text in check_file(lines, args.strict)
            if f"{relative}:{rule}" not in exceptions
        ]
        if not findings:
            continue

        if args.fix:
            removed = fix_file(path, lines, findings)
            fixed += removed
            findings = [(n, r, t) for n, r, t in findings if r not in FIXABLE]
            if not findings:
                continue

        for number, rule, text in findings:
            failures.append(f"{relative}:{number}: [{rule}] {text}")

    if failures:
        print("Comment lint failed:", file=sys.stderr)
        for failure in failures:
            print(f"  {failure}", file=sys.stderr)
        print(
            f"\n{len(failures)} violation(s). Fix them, or add '<path>:<rule>' to "
            f"{EXCEPTIONS_FILE.relative_to(REPO_ROOT)} with a reason if the file "
            f"is vendored or generated.",
            file=sys.stderr,
        )
        return 1

    suffix = f", {fixed} line(s) removed" if fixed else ""
    print(f"Comment lint: {checked} files checked, clean{suffix}.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
