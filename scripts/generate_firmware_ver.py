#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
import pathlib
import re
import subprocess
from typing import Any


REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent

DEFAULT_VERSION_REF = "origin/master"
VERSION_TAG_PREFIX = "fw-v"
INITIAL_BASE_VERSION = 0

# MAVLink exposes flight_sw_version as packed major.minor.patch.type bytes.
# We keep the user-managed base version in the major byte and spread the
# commit count across the minor/patch bytes for up to 65535 commits.
MAVLINK_VERSION_TYPE_DEV = 0
MAVLINK_COMMIT_COUNT_MAX = 0xFFFF


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


def _resolve_version_base(ref: str) -> dict[str, Any]:
    tag = _nearest_version_tag(ref)
    if tag is None:
        return {
            "base_version": INITIAL_BASE_VERSION,
            "base_commit": _first_commit(ref),
            "base_tag": None,
        }

    return {
        "base_version": _base_version_from_tag(tag),
        "base_commit": _resolve_commit(tag),
        "base_tag": tag,
    }


def _pack_mavlink_flight_sw_version(base_version: int, commit_count: int) -> int:
    if not 0 <= base_version <= 0xFF:
        raise ValueError("BASE_VERSION must fit in the MAVLink major version byte")
    if not 0 <= commit_count <= MAVLINK_COMMIT_COUNT_MAX:
        raise ValueError(
            "commit count exceeds MAVLink minor/patch capacity; "
            "bump BASE_VERSION and BASE_VERSION_COMMIT"
        )

    minor = (commit_count >> 8) & 0xFF
    patch = commit_count & 0xFF
    return (
        (base_version << 24)
        | (minor << 16)
        | (patch << 8)
        | MAVLINK_VERSION_TYPE_DEV
    )


def resolve_firmware_version(version_ref: str = DEFAULT_VERSION_REF) -> dict[str, Any]:
    ref_commit = _resolve_commit(version_ref)
    version_base = _resolve_version_base(version_ref)
    base_version = int(version_base["base_version"])
    base_commit = str(version_base["base_commit"])
    _ensure_ancestor(base_commit, ref_commit)

    commit_count = _commit_count_since(base_commit, ref_commit)
    packed_version = _pack_mavlink_flight_sw_version(base_version, commit_count)

    return {
        "base_version": base_version,
        "base_commit": base_commit,
        "base_tag": version_base["base_tag"],
        "version_ref": version_ref,
        "ref_commit": ref_commit,
        "commit_count": commit_count,
        "version_string": f"{base_version}.{commit_count}",
        "mavlink_flight_sw_version": packed_version,
        "mavlink_flight_sw_version_hex": f"0x{packed_version:08X}u",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ref", default=DEFAULT_VERSION_REF)
    parser.add_argument("--format", choices=("json", "text"), default="json")
    args = parser.parse_args()

    try:
        version = resolve_firmware_version(args.ref)
    except ValueError as exc:
        raise SystemExit(str(exc)) from exc

    if args.format == "text":
        print(version["version_string"])
        return 0

    print(json.dumps(version, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
