#!/usr/bin/env python3

from __future__ import annotations

import argparse
import pathlib
import re
import subprocess
from dataclasses import dataclass


REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
README_PATH = REPO_ROOT / "README.md"
VERSION_PATH = REPO_ROOT / "VERSION"

DEFAULT_VERSION_REF = "HEAD"
VERSION_RE = re.compile(r"\bv?(\d+)\.(\d+)(?:\.(?:x|\d+))?(?:\s+Stable)?\b")


@dataclass(frozen=True)
class VersionSource:
    path: str
    content: str


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


def _read_file_at_ref(path: str, ref: str, ref_commit: str) -> str | None:
    head_commit = _resolve_commit("HEAD")
    if ref == DEFAULT_VERSION_REF and ref_commit == head_commit:
        local_path = REPO_ROOT / path
        try:
            return local_path.read_text(encoding="utf-8")
        except OSError:
            return None

    try:
        return _git("show", f"{ref_commit}:{path}")
    except SystemExit:
        return None


def _read_version_source(ref: str, ref_commit: str) -> VersionSource:
    for path in (VERSION_PATH.name, README_PATH.name):
        content = _read_file_at_ref(path, ref, ref_commit)
        if content is not None:
            return VersionSource(path=path, content=content)

    raise SystemExit("VERSION or README.md is required to resolve the firmware version")


def _parse_firmware_version_base(version_source: str) -> tuple[int, int]:
    match = VERSION_RE.search(version_source)
    if not match:
        raise SystemExit(
            "unable to find firmware version; expected MAJOR.MINOR in VERSION "
            "or legacy 'vMAJOR.MINOR.x Stable' in README.md"
        )

    parts = tuple(int(match.group(index), 10) for index in range(1, 3))
    if any(part > 0xFF for part in parts):
        raise SystemExit("firmware version components must fit in one byte")

    return parts


def _base_start_commit(
    ref_commit: str, version_base: tuple[int, int], source_path: str
) -> str | None:
    commits = _git("log", "--format=%H", "--reverse", ref_commit, "--", source_path)
    for commit in commits.splitlines():
        try:
            source = _git("show", f"{commit}:{source_path}")
        except SystemExit:
            continue
        try:
            if _parse_firmware_version_base(source) == version_base:
                return commit
        except SystemExit:
            continue
    return None


def _auto_patch_version(
    ref_commit: str, version_base: tuple[int, int], source_path: str
) -> int:
    base_commit = _base_start_commit(ref_commit, version_base, source_path)
    if base_commit is None:
        patch = 1
    else:
        patch = int(_git("rev-list", "--count", f"{base_commit}..{ref_commit}")) + 1

    if patch > 0xFF:
        raise SystemExit(
            "auto firmware patch version exceeds one byte; bump the firmware "
            f"major/minor version in {source_path}"
        )
    return patch


def resolve_firmware_version(version_ref: str = DEFAULT_VERSION_REF) -> str:
    ref_commit = _resolve_commit(version_ref)
    source = _read_version_source(version_ref, ref_commit)
    version_base = _parse_firmware_version_base(source.content)
    major, minor = version_base
    patch = _auto_patch_version(ref_commit, version_base, source.path)
    return f"{major}.{minor}.{patch}"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ref", default=DEFAULT_VERSION_REF)
    args = parser.parse_args()

    print(resolve_firmware_version(args.ref))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
