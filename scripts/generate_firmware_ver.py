#!/usr/bin/env python3

from __future__ import annotations

import argparse
import pathlib
import re
import subprocess


REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent

DEFAULT_VERSION_REF = "origin/master"
VERSION_TAG_PREFIX = "fw-v"
INITIAL_BASE_VERSION = 0


def _git(*args: str) -> str:
    try:
        result = subprocess.run(
            ["git", *args],
            cwd=REPO_ROOT,
            check=True,
            capture_output=True,
            text=True,
        )
    except OSError as exc:
        raise SystemExit("git is required to resolve the firmware version") from exc
    except subprocess.CalledProcessError as exc:
        stderr = exc.stderr.strip()
        message = stderr or f"git {' '.join(args)} failed"
        raise SystemExit(message) from exc

    return result.stdout.strip()


def _resolve_commit(ref: str) -> str:
    return _git("rev-parse", "--verify", f"{ref}^{{commit}}")


def _first_commit(ref: str) -> str:
    commits = _git("rev-list", "--max-parents=0", "--reverse", ref).splitlines()
    if not commits:
        raise SystemExit(f"could not determine first commit for {ref}")
    return commits[0]


def _ensure_ancestor(base_commit: str, ref_commit: str) -> None:
    try:
        subprocess.run(
            ["git", "merge-base", "--is-ancestor", base_commit, ref_commit],
            cwd=REPO_ROOT,
            check=True,
            capture_output=True,
            text=True,
        )
    except OSError as exc:
        raise SystemExit("git is required to resolve the firmware version") from exc
    except subprocess.CalledProcessError as exc:
        raise SystemExit(
            f"base commit {base_commit} is not an ancestor of {ref_commit}"
        ) from exc


def _commit_count_since(base_commit: str, ref_commit: str) -> int:
    return int(_git("rev-list", "--count", f"{base_commit}..{ref_commit}"))


def _nearest_version_tag(ref: str) -> str | None:
    try:
        return _git(
            "describe",
            "--tags",
            "--abbrev=0",
            "--match",
            f"{VERSION_TAG_PREFIX}[0-9]*",
            ref,
        )
    except SystemExit:
        return None


def _base_version_from_tag(tag: str) -> int:
    match = re.fullmatch(rf"{re.escape(VERSION_TAG_PREFIX)}(\d+)", tag)
    if match is None:
        raise SystemExit(
            f"version tag '{tag}' must match {VERSION_TAG_PREFIX}<integer>"
        )
    return int(match.group(1))


def _resolve_version_base(ref: str) -> tuple[int, str]:
    tag = _nearest_version_tag(ref)
    if tag is None:
        return INITIAL_BASE_VERSION, _first_commit(ref)

    return _base_version_from_tag(tag), _resolve_commit(tag)


def resolve_firmware_version(version_ref: str = DEFAULT_VERSION_REF) -> str:
    ref_commit = _resolve_commit(version_ref)
    base_version, base_commit = _resolve_version_base(version_ref)
    _ensure_ancestor(base_commit, ref_commit)

    commit_count = _commit_count_since(base_commit, ref_commit)
    if not 0 <= base_version <= 0xFF:
        raise SystemExit("firmware base version must fit in one byte")
    if not 0 <= commit_count <= 0xFF:
        raise SystemExit(
            "firmware minor version exceeds 255 commits; tag a new base version"
        )

    return f"{base_version}.{commit_count}.0"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ref", default=DEFAULT_VERSION_REF)
    args = parser.parse_args()

    print(resolve_firmware_version(args.ref))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
