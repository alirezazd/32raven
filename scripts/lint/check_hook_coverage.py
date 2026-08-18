#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi
# /// script
# dependencies = [
#     "pyyaml",
# ]
# ///

"""Check every linter runs somewhere, and commit hooks have a backstop.

Two ways a check stops checking without anyone noticing, both of which have
happened here:

  uncovered  A hook in `.pre-commit-config.yaml` that no workflow repeats.
             Commit hooks are skippable with `--no-verify` and absent from a
             fresh clone until someone installs them, so a hook CI does not
             repeat is advisory. `lint.yml` says as much in its first line.
  unrun      A script under `scripts/lint/` that nothing invokes at all --
             not a hook, not a workflow, not the build. It passes review as
             working code and never runs again.

Checks that need a compilation database are the ones this catches most easily:
they return 0 and print "skipping" when the tree has not been built, so a hook
that only ever runs pre-commit reports success forever. Those belong in
`build.yml`, after a build, and this is what says so.

Coverage is per script, not per job. The compilation-database checks run once
per firmware because each build job sees only its own build directory, and one
of those two steps going missing still leaves the script named in a workflow.

Run:
  uv run --quiet --script scripts/lint/check_hook_coverage.py
"""

from __future__ import annotations

import pathlib
import sys

import yaml

REPO = pathlib.Path(__file__).resolve().parent.parent.parent
PRE_COMMIT = REPO / ".pre-commit-config.yaml"
WORKFLOWS = REPO / ".github/workflows"
LINT_DIR = REPO / "scripts/lint"

# Where an invocation can legitimately live besides a workflow. The build files
# run two checks as compile dependencies, which is stricter than any hook.
BUILD_FILES = (
    REPO / "CMakeLists.txt",
    REPO / "stm32/CMakeLists.txt",
    REPO / "esp32/main/CMakeLists.txt",
    REPO / "Makefile",
)

SCRIPT_SUFFIXES = (".py", ".sh")


def _script_in(command: str) -> str | None:
    """The `scripts/lint/...` path a command runs, if it runs one."""
    for token in command.split():
        if token.startswith("scripts/lint/") and token.endswith(
            SCRIPT_SUFFIXES
        ):
            return token
    return None


def _local_hooks() -> dict[str, str]:
    """Hook id -> the lint script it runs, for this repo's own hooks."""
    config = yaml.safe_load(PRE_COMMIT.read_text())
    hooks = {}
    for repo in config.get("repos", []):
        if repo.get("repo") != "local":
            continue
        for hook in repo.get("hooks", []):
            script = _script_in(hook.get("entry", ""))
            if script:
                hooks[hook["id"]] = script
    return hooks


def _workflow_scripts() -> dict[str, set[str]]:
    """Lint script -> the workflow files that run it."""
    found: dict[str, set[str]] = {}
    for path in sorted(WORKFLOWS.glob("*.yml")):
        workflow = yaml.safe_load(path.read_text())
        for job in (workflow.get("jobs") or {}).values():
            for step in job.get("steps") or []:
                script = _script_in(step.get("run", "") or "")
                if script:
                    found.setdefault(script, set()).add(path.name)
    return found


def main() -> int:
    hooks = _local_hooks()
    in_workflow = _workflow_scripts()
    build_text = "\n".join(
        path.read_text() for path in BUILD_FILES if path.exists()
    )

    problems: list[str] = []

    for hook_id, script in sorted(hooks.items()):
        if script not in in_workflow:
            problems.append(
                f"[uncovered] {hook_id} runs {script} at commit time and no "
                "workflow repeats it"
            )

    hooked = set(hooks.values())
    for path in sorted(LINT_DIR.iterdir()):
        if path.suffix not in SCRIPT_SUFFIXES:
            continue
        rel = path.relative_to(REPO).as_posix()
        if rel in hooked or rel in in_workflow or rel in build_text:
            continue
        problems.append(
            f"[unrun] {rel} is not run by a hook, a workflow or the build"
        )

    if not problems:
        return 0

    print("Checks that do not check anything:", file=sys.stderr)
    for problem in problems:
        print(f"  {problem}", file=sys.stderr)
    print(
        "\nAdd the step to a workflow -- lint.yml when the script needs only "
        "uv, build.yml when it reads a compilation database and would\n"
        "otherwise skip. A script nothing runs is either owed a home or owed "
        "deleting.",
        file=sys.stderr,
    )
    return 1


if __name__ == "__main__":
    sys.exit(main())
