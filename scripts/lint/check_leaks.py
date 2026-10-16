#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi

"""Keep text that belongs on this machine out of everything pushed to GitHub.

The rules are the secret, so the tree never holds them: one Python regex per
line in `.git/info/leak-patterns`, beside git's own `info/exclude`, where no
commit can pick them up. Blank lines and lines starting with `#` are skipped,
and a rule that should ignore case says so with `(?i)`. A clone without the
file -- CI, a fresh checkout -- has nothing to look for and passes.

Every rule is matched line by line against each surface a push carries: file
contents, whether code, comment or prose; file paths; commit messages. Author
and committer identities are not text a commit adds, so they are left alone.

Modes, one per hook stage:

  <path>...        pre-commit: the staged contents of these paths, and the
                   paths themselves.
  --message FILE   commit-msg: the message as git will store it.
  --push           pre-push: every commit the push adds to the remote --
                   messages, added lines and paths. A commit that skipped its
                   hooks, or was built in another clone, is caught here before
                   it leaves.
  (none)           every tracked file in the working tree: the sweep to run
                   after adding a rule.

Run:
  uv run --quiet --script scripts/lint/check_leaks.py
"""

from __future__ import annotations

import argparse
import os
import pathlib
import re
import subprocess
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent.parent
RULES_FILE = "info/leak-patterns"

# `git commit -v` appends the staged diff below this line. git drops it from
# the stored message, and its removed lines would otherwise read as leaks.
SCISSORS = "# ------------------------ >8 ------------------------"

HUNK_RE = re.compile(r"^@@ -\S+ \+(\d+)")


def git(*args: str) -> str:
    return subprocess.run(
        ["git", *args], cwd=REPO, check=True, capture_output=True, text=True
    ).stdout


def rules_path() -> pathlib.Path:
    # The common dir, so a linked worktree reads its main clone's rules.
    common = pathlib.Path(git("rev-parse", "--git-common-dir").strip())
    return REPO / common / RULES_FILE


def load_rules(path: pathlib.Path) -> list[re.Pattern[str]]:
    if not path.is_file():
        return []
    rules = []
    lines = path.read_text(encoding="utf-8").splitlines()
    for number, line in enumerate(lines, 1):
        pattern = line.strip()
        if not pattern or pattern.startswith("#"):
            continue
        try:
            rules.append(re.compile(pattern))
        except re.error as err:
            # Skipping it would pass everything that rule exists to stop.
            raise SystemExit(f"{path}:{number}: {err}") from err
    return rules


def scan(where: str, text: str, rules: list[re.Pattern[str]]) -> list[str]:
    """`where:line` for every line a rule matches."""
    return [
        f"{where}:{number}: /{rule.pattern}/"
        for number, line in enumerate(text.splitlines(), 1)
        for rule in rules
        if rule.search(line)
    ]


def as_text(blob: bytes) -> str | None:
    if b"\0" in blob:
        return None
    return blob.decode("utf-8", errors="replace")


def staged_hits(paths: list[str], rules: list[re.Pattern[str]]) -> list[str]:
    hits = []
    for path in paths:
        hits += scan(f"path {path}", path, rules)
        # The index's copy is what the commit holds, whatever the working tree
        # has since. Nothing staged at that path, nothing to commit.
        shown = subprocess.run(
            ["git", "show", f":{path}"], cwd=REPO, capture_output=True
        )
        text = as_text(shown.stdout) if shown.returncode == 0 else None
        if text is not None:
            hits += scan(path, text, rules)
    return hits


def message_hits(raw: str, rules: list[re.Pattern[str]]) -> list[str]:
    kept = []
    for line in raw.splitlines():
        if line.startswith(SCISSORS):
            break
        if not line.startswith("#"):
            kept.append(line)
    return scan("commit message", "\n".join(kept), rules)


def pushed_commits() -> list[str]:
    """The commits the push adds: reachable from its tip, on no remote ref."""
    # pre-commit names the tip and the remote at pre-push. By hand, HEAD
    # against every remote.
    tip = os.environ.get("PRE_COMMIT_TO_REF", "HEAD")
    remote = os.environ.get("PRE_COMMIT_REMOTE_NAME")
    known = f"--remotes={remote}" if remote else "--remotes"
    return git("rev-list", "--reverse", tip, "--not", known).split()


def commit_hits(sha: str, rules: list[re.Pattern[str]]) -> list[str]:
    short = sha[:9]
    hits = scan(f"{short} message", git("log", "-1", "--format=%B", sha), rules)

    # No rename detection: a moved file is then a whole file added, so its
    # new path and every line under it are read.
    shown = ["show", "--format=", "--no-renames", "--diff-merges=first-parent"]
    for path in git(*shown, "--name-only", "--diff-filter=d", sha).split("\n"):
        if path:
            hits += scan(f"{short} path {path}", path, rules)

    diff = git(*shown, "--unified=0", "--no-color", "--no-ext-diff", sha)
    path = None
    line_no = 0
    for line in diff.splitlines():
        if line.startswith("+++ "):
            path = line[6:] if line.startswith("+++ b/") else None
        elif line.startswith("@@"):
            match = HUNK_RE.match(line)
            line_no = int(match.group(1)) if match else 0
        elif line.startswith("+") and path is not None:
            hits += [
                f"{short} {path}:{line_no}: /{rule.pattern}/"
                for rule in rules
                if rule.search(line[1:])
            ]
            line_no += 1
    return hits


def tree_hits(rules: list[re.Pattern[str]]) -> list[str]:
    hits = []
    for path in git("ls-files", "-z").split("\0"):
        if not path:
            continue
        hits += scan(f"path {path}", path, rules)
        # Submodules list as directories; their text is upstream's.
        file = REPO / path
        if file.is_symlink() or not file.is_file():
            continue
        text = as_text(file.read_bytes())
        if text is not None:
            hits += scan(path, text, rules)
    return hits


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="*", help="staged paths to check")
    parser.add_argument("--message", help="file holding the commit message")
    parser.add_argument(
        "--push", action="store_true", help="check the commits being pushed"
    )
    args = parser.parse_args()

    path = rules_path()
    rules = load_rules(path)
    sweep = not (args.paths or args.message or args.push)
    if not rules:
        if sweep:
            print(f"No rules in {path}; nothing checked.", file=sys.stderr)
        return 0

    if args.message:
        raw = pathlib.Path(args.message).read_text(encoding="utf-8")
        hits = message_hits(raw, rules)
        fix = "Reword the message."
    elif args.push:
        hits = []
        for sha in pushed_commits():
            hits += commit_hits(sha, rules)
        fix = "These commits are not on the remote yet; rewrite them first."
    elif args.paths:
        hits = staged_hits(args.paths, rules)
        fix = "Remove the text before committing."
    else:
        hits = tree_hits(rules)
        fix = "Remove the text from the tree."

    if not hits:
        if sweep:
            print(f"Leak check: clean against {len(rules)} rule(s).")
        return 0

    print("Text that must not leave this machine:", file=sys.stderr)
    for hit in hits:
        print(f"  {hit}", file=sys.stderr)
    print(f"\n{fix} A wrong rule is fixed in {path}.", file=sys.stderr)
    return 1


if __name__ == "__main__":
    sys.exit(main())
