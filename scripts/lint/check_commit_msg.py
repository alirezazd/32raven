#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi

"""Check a commit message against the shape this history already uses.

The rules are read off `git log`, not imported from a style guide: every type
and scope below is one the tree already commits under. What the check adds is
a closed set, because the drift is always the same -- `tools:` and `lint:` and
`config:` appeared as types when they are places, and a type that exists once
is a type nobody can grep for.

Rules:

  subject-shape   `type(scope): description`. Scope is optional; when present
                  it names where the change lives, not what it does.
  type            One of TYPES. Closed on purpose.
  scope           One of SCOPES, which are the top-level directories plus the
                  build. Closed for the same reason.
  subject-case    Description starts lower-case and does not end in a period.
                  It is a fragment completing "this commit will ...".
  subject-length  <= 72 characters, so `git log --oneline` and a GitHub
                  listing show it whole.
  body-blank      A blank line between subject and body, or `git log --format`
                  and every other tool treats the whole thing as the subject.
  body-width      Body lines <= 80 characters.
  attribution     No tool or assistant trailers. Authorship belongs in the
                  author field, and this project does not carry them.

Deliberately absent: imperative mood. "add" and "adds" are both common here
and no rule separates them from a noun phrase without guessing.

Run:
  uv run --quiet --script scripts/lint/check_commit_msg.py .git/COMMIT_EDITMSG
  uv run --quiet --script scripts/lint/check_commit_msg.py --range HEAD~20..HEAD
"""

from __future__ import annotations

import argparse
import pathlib
import re
import subprocess
import sys

# Every type the tree commits under, collapsed to a closed set. `lint`,
# `tools` and `config` were types in older commits and are scopes here.
TYPES = (
    "feat",
    "fix",
    "refactor",
    "perf",
    "docs",
    "test",
    "build",
    "chore",
    "revert",
)

# The top-level directories plus `build` for the toolchain itself. A scope
# answers "where", so anything that is not a place does not belong.
SCOPES = (
    "stm32",
    "esp32",
    "libs",
    "tools",
    "scripts",
    "config",
    "docs",
    "site",
    "build",
)

SUBJECT_MAX = 72
BODY_MAX = 80

SUBJECT_RE = re.compile(r"^(?P<type>[a-z]+)(?:\((?P<scope>[^)]*)\))?: (?P<desc>.+)$")

# Rebase helpers and reverts git writes itself. Rejecting these fails a
# `git rebase -i` mid-flight, which is never what the rule is for.
PASSTHROUGH_RE = re.compile(r"^(fixup|squash|amend)!|^Revert \"|^Merge ")

ATTRIBUTION_RE = re.compile(
    r"(co-authored-by"
    r"|generated (with|by) (claude|chatgpt|gpt|copilot|gemini|cursor|codex|ai)"
    r"|\b(claude code|github copilot|chatgpt|openai codex)\b"
    r"|🤖)",
    re.IGNORECASE,
)


def check(message: str) -> list[str]:
    """Returns one string per broken rule, empty when the message is fine."""
    # git strips comments and any scissors section before the hook sees the
    # final message, but COMMIT_EDITMSG on disk still carries them.
    lines = [
        line.rstrip("\n")
        for line in message.splitlines()
        if not line.startswith("#")
    ]
    while lines and not lines[-1].strip():
        lines.pop()
    if not lines:
        return ["empty commit message"]

    problems: list[str] = []
    subject = lines[0]

    for number, line in enumerate(lines, start=1):
        if ATTRIBUTION_RE.search(line):
            problems.append(f"[attribution] line {number}: {line.strip()}")

    if PASSTHROUGH_RE.match(subject):
        return problems

    match = SUBJECT_RE.match(subject)
    if match is None:
        problems.append(
            f"[subject-shape] expected 'type(scope): description', got: {subject}"
        )
    else:
        kind = match.group("type")
        scope = match.group("scope")
        desc = match.group("desc")

        if kind not in TYPES:
            problems.append(
                f"[type] '{kind}' is not one of: {', '.join(TYPES)}"
            )
        if scope is not None and scope not in SCOPES:
            problems.append(
                f"[scope] '{scope}' is not one of: {', '.join(SCOPES)}"
            )
        if desc[:1].isupper():
            problems.append(f"[subject-case] description is capitalised: {desc}")
        if desc.endswith("."):
            problems.append(f"[subject-case] description ends with a period: {desc}")

    if len(subject) > SUBJECT_MAX:
        problems.append(
            f"[subject-length] {len(subject)} > {SUBJECT_MAX} characters"
        )

    if len(lines) > 1 and lines[1].strip():
        problems.append("[body-blank] no blank line between subject and body")

    for number, line in enumerate(lines[1:], start=2):
        if len(line) > BODY_MAX:
            problems.append(
                f"[body-width] line {number}: {len(line)} > {BODY_MAX} characters"
            )

    return problems


def messages_in_range(rev_range: str) -> list[tuple[str, str]]:
    out = subprocess.run(
        ["git", "log", "--format=%H%x00%B%x01", rev_range],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    entries = []
    for record in out.split("\x01"):
        record = record.strip("\n")
        if not record:
            continue
        sha, _, body = record.partition("\x00")
        entries.append((sha[:9], body))
    return entries


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", nargs="?", help="file holding the commit message")
    parser.add_argument("--range", help="check every commit in a revision range")
    args = parser.parse_args()

    if args.range:
        failed = 0
        for sha, body in messages_in_range(args.range):
            problems = check(body)
            if problems:
                failed += 1
                print(f"{sha}  {body.splitlines()[0]}", file=sys.stderr)
                for problem in problems:
                    print(f"    {problem}", file=sys.stderr)
        if failed:
            print(f"\n{failed} commit(s) need rewording.", file=sys.stderr)
            return 1
        print("Commit messages: clean.")
        return 0

    if not args.path:
        parser.error("a message file is required unless --range is given")

    problems = check(pathlib.Path(args.path).read_text(encoding="utf-8"))
    if not problems:
        return 0

    print("Commit message rejected:", file=sys.stderr)
    for problem in problems:
        print(f"  {problem}", file=sys.stderr)
    print(
        f"\nTypes:  {', '.join(TYPES)}"
        f"\nScopes: {', '.join(SCOPES)} (optional)"
        f"\nShape:  type(scope): lower-case description, <= {SUBJECT_MAX} chars",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
