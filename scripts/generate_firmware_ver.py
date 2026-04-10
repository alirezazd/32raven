#!/usr/bin/env python3

from __future__ import annotations

import argparse
import datetime
import pathlib
import subprocess


REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent

DEFAULT_VERSION_REF = "HEAD"


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


def _commit_unix_timestamp(ref: str) -> int:
    return int(_git("show", "-s", "--format=%ct", ref))


def resolve_firmware_version(version_ref: str = DEFAULT_VERSION_REF) -> str:
    ref_commit = _resolve_commit(version_ref)
    commit_time = datetime.datetime.fromtimestamp(
        _commit_unix_timestamp(ref_commit), tz=datetime.timezone.utc
    )
    return commit_time.strftime("%y.%m.%d")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ref", default=DEFAULT_VERSION_REF)
    args = parser.parse_args()

    print(resolve_firmware_version(args.ref))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
