#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi

"""Check a commit message against the template in .gitmessage.

A message states what changed and why. It is not a record of how the change
was arrived at, what was tried first, or what the author thought along the
way -- `git log` is read by someone deciding whether a commit is the one they
are looking for, and prose buries that.

Template:

    type(scope): what changed

    Why, in at most BODY_MAX_LINES lines. Facts only.

Rules:

  subject-shape   `type(scope): description`. The scope names where the change
                  lives, not what it does.
  type            One of TYPES. Closed on purpose.
  scope           One of SCOPES, which are the top-level directories plus the
                  build. Closed for the same reason.
  subject-scope   Required, so a subject says where before it says what.
                  Two exemptions, both narrow: commits at or before
                  SCOPE_REQUIRED_AFTER predate the rule, and a type that is
                  itself a place already answers "where" -- `docs:` and
                  `build:` stand alone.
  subject-repeat  The scope may not repeat the type. `docs(docs)` names one
                  thing twice and leaves the subject saying less than
                  `docs:` does.
  subject-case    Description starts lower-case and does not end in a period.
                  It is a fragment completing "this commit will ...".
  subject-length  <= 72 characters, so `git log --oneline` and a GitHub
                  listing show it whole.
  body-blank      A blank line between subject and body, or `git log --format`
                  and every other tool treats the whole thing as the subject.
  body-width      Body lines <= 80 characters.
  body-length     <= BODY_MAX_LINES non-blank lines. A subject that needs more
                  than that to justify it is usually more than one commit.
  body-person     No first person. The commit is the subject of the sentence,
                  not its author.
  body-narrative  No storytelling markers. These only ever introduce the
                  process, and the process is not the change.
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
# answers "where", so anything that is not a place does not belong. `site` was
# here and is gone: mkdocs writes it and it is gitignored, so it named nothing
# a commit could touch.
SCOPES = (
    "stm32",
    "esp32",
    "libs",
    "tools",
    "scripts",
    "config",
    "docs",
    "build",
)

# A type that is also a place answers "where" by itself, so it may stand
# without a scope -- and repeating it as one would say the same thing twice.
# Derived rather than listed: the property is the overlap, and spelling it out
# would let the two sets drift apart from the rule that reads them.
SELF_SCOPED_TYPES = frozenset(TYPES) & frozenset(SCOPES)

# The last commit written before the scope became mandatory. Everything at or
# before it is checked under the older rules, because CI walks the whole
# history on purpose and rewriting 144 subjects to satisfy a rule they predate
# would be a worse trade than carrying one constant.
SCOPE_REQUIRED_AFTER = "79d15c1c21abb964b6927e6ffe8ce5365e224246"

SUBJECT_MAX = 72
BODY_MAX = 80
BODY_MAX_LINES = 5

SUBJECT_RE = re.compile(r"^(?P<type>[a-z]+)(?:\((?P<scope>[^)]*)\))?: (?P<desc>.+)$")

# Git trailers: `Key: value` at the foot of a message. Exempt from the prose
# rules and from the line budget, which is there to bound explanation.
TRAILER_RE = re.compile(r"^[A-Z][A-Za-z-]*(-[A-Za-z]+)*: \S")

# The author is never the subject of a commit message. Bare `I` is excluded
# from the pattern: it collides with identifiers and Roman numerals, and a
# message written in the first person always trips one of the others too.
# `us` needs the lookbehinds -- unqualified it matches microseconds, which
# outnumber the pronoun by a wide margin in firmware.
FIRST_PERSON_RE = re.compile(
    r"\b(we|our|ours|(?<!\d)(?<!\d )us|my|mine|i'(m|ve|d|ll))\b", re.I
)

# Phrases that exist to narrate. Each one introduces the route to the change
# rather than the change, which is the failure this check is here to stop.
NARRATIVE_RE = re.compile(
    r"\b("
    r"turn(s|ed) out"
    r"|as it happen(s|ed)"
    r"|it seems"
    r"|worth (noting|reading|a|the|it)"
    r"|note that"
    r"|found in this order"
    r"|interestingly|unfortunately|sadly|happily|admittedly|frankly|honestly"
    r"|of course|obviously|naturally"
    r"|earlier (note|estimate|assumption|claim)"
    r"|older note"
    r"|this session"
    r")\b",
    re.I,
)

# Rebase helpers and reverts git writes itself. Rejecting these fails a
# `git rebase -i` mid-flight, which is never what the rule is for.
PASSTHROUGH_RE = re.compile(r"^(fixup|squash|amend)!|^Revert \"|^Merge ")

# Assistant attribution, in the four shapes it arrives in. Names are listed
# only where they cannot mean anything else here: `cursor`, `continue`, `amp`,
# `goose` and `sweep` are words this tree uses in earnest, and `codex` names a
# directory in it, so those are caught by their agent spelling or their
# trailer, never bare.
ATTRIBUTION_RE = re.compile(
    # Trailer keys that credit a collaborator. Anchored to the line, so prose
    # using the same words does not trip them.
    r"(^(co-authored-by|co-developed-by|assisted-by|ai-assisted-by"
    r"|generated-by|on-behalf-of)\s*:"
    # The footer a tool appends to a message it wrote. Kept to a name list
    # rather than a bare `generated with`, which this repo says about headers.
    r"|generated (with|by) (claude|chatgpt|gpt|copilot|gemini|cursor|codex"
    r"|devin|aider|windsurf|codeium|tabnine|amp|opencode|an? ai|ai)"
    # Product names distinctive enough to mean only the tool.
    r"|\b(claude|copilot|chatgpt|openai codex|codex cli|gemini|devin|aider"
    r"|windsurf|codeium|tabnine|openhands|sourcegraph cody|amazon q"
    r"|cursor agent|continue\.dev|jetbrains junie|replit agent)\b"
    # Bot identities, and the address Claude Code signs with.
    r"|\[bot\]|noreply@anthropic\.com"
    # The robot every generated footer leads with.
    r"|🤖)",
    re.IGNORECASE,
)


def check(message: str, *, require_scope: bool = True) -> list[str]:
    """Returns one string per broken rule, empty when the message is fine.

    `require_scope` is False only for commits that predate the rule; a message
    being written now has no history to be exempted by.
    """
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
        if scope is not None and scope == kind:
            problems.append(
                f"[subject-repeat] scope '{scope}' repeats the type; "
                f"write '{kind}: {desc}'"
            )
        if scope is None and require_scope and kind not in SELF_SCOPED_TYPES:
            problems.append(
                f"[subject-scope] '{kind}' needs a scope, one of: "
                f"{', '.join(SCOPES)}"
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

    body = lines[1:]
    for number, line in enumerate(body, start=2):
        if len(line) > BODY_MAX:
            problems.append(
                f"[body-width] line {number}: {len(line)} > {BODY_MAX} characters"
            )
        # Trailers carry addresses and issue links, which are not prose and
        # are not the author writing about themselves.
        if TRAILER_RE.match(line):
            continue
        if FIRST_PERSON_RE.search(line):
            problems.append(f"[body-person] line {number}: {line.strip()}")
        if NARRATIVE_RE.search(line):
            problems.append(f"[body-narrative] line {number}: {line.strip()}")

    filled = [line for line in body if line.strip() and not TRAILER_RE.match(line)]
    if len(filled) > BODY_MAX_LINES:
        problems.append(
            f"[body-length] {len(filled)} > {BODY_MAX_LINES} non-blank body lines"
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
        entries.append((sha, body))
    return entries


def commits_predating_scope_rule() -> frozenset[str]:
    """Every commit reachable from SCOPE_REQUIRED_AFTER, itself included.

    Unreachable means a graft, a shallow clone, or a rewritten boundary. None
    of those can tell old commits from new ones, so the check reports that it
    cannot and exempts nothing -- a rule that silently stops applying is worse
    than one that fails loudly.
    """
    result = subprocess.run(
        ["git", "rev-list", SCOPE_REQUIRED_AFTER],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        print(
            f"warning: {SCOPE_REQUIRED_AFTER[:9]} is unreachable, so no commit "
            f"is exempt from [subject-scope]",
            file=sys.stderr,
        )
        return frozenset()
    return frozenset(result.stdout.split())


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("path", nargs="?", help="file holding the commit message")
    parser.add_argument("--range", help="check every commit in a revision range")
    args = parser.parse_args()

    if args.range:
        failed = 0
        legacy = commits_predating_scope_rule()
        for sha, body in messages_in_range(args.range):
            problems = check(body, require_scope=sha not in legacy)
            if problems:
                failed += 1
                print(f"{sha[:9]}  {body.splitlines()[0]}", file=sys.stderr)
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
        f"\n    type(scope): what changed"
        f"\n"
        f"\n    Why, in at most {BODY_MAX_LINES} lines. Facts only."
        f"\n"
        f"\nTypes:  {', '.join(TYPES)}"
        f"\nScopes: {', '.join(SCOPES)}"
        f"\nA scope is required, and may not repeat the type."
        f"\n{' and '.join(sorted(SELF_SCOPED_TYPES))} name a place already, so"
        f" they stand alone."
        f"\nSubject <= {SUBJECT_MAX} chars, lower-case, no trailing period."
        f"\nBody states what was wrong and why this fixes it -- not how it was"
        f"\nfound, what was tried first, or what the author thought about it.",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
