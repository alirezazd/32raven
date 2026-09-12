#!/usr/bin/env python3

# /// script
# requires-python = ">=3.11"
# ///

# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi

"""Hold the parameter dictionary to the parameters the bridge actually serves.

The dictionary is loaded by the ground station, not by the firmware, so
nothing at runtime notices when the two drift. A parameter added to
kParamTable and not described here reaches the user as a bare name with no
units and no bounds; one described but no longer served is text about
something that does not exist. Both are caught here instead.

Run:
  uv run --quiet --script scripts/lint/check_param_metadata.py
"""

from __future__ import annotations

import pathlib
import sys

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(REPO_ROOT / "scripts"))

from generate_param_metadata import (  # noqa: E402
    METADATA_SOURCE,
    build_metadata,
    served_parameters,
)

# Without these a parameter is worse off than the PX4 entry it replaced: the
# name is all the user gets, and at least PX4's guess had prose attached.
REQUIRED_KEYS = ("shortDesc", "longDesc", "group", "category")


def main() -> int:
    # Raises when a served parameter has no description, which is half the
    # check; the rest is everything the generator cannot notice.
    metadata = build_metadata()

    errors: list[str] = []
    described = {entry["name"] for entry in metadata["parameters"]}
    served = {name for name, _ in served_parameters()}

    for name in sorted(described - served):
        errors.append(f"{name} is described but the bridge does not serve it")

    for entry in metadata["parameters"]:
        missing = [key for key in REQUIRED_KEYS if not entry.get(key)]
        if missing:
            errors.append(f"{entry['name']} is missing {', '.join(missing)}")

        low, high = entry.get("min"), entry.get("max")
        if low is not None and high is not None and low > high:
            errors.append(f"{entry['name']} has min {low} above max {high}")

        for value in entry.get("values", []):
            if "value" not in value or not value.get("description"):
                errors.append(f"{entry['name']} has an unlabelled value")

    if errors:
        for error in errors:
            print(f"check_param_metadata: {error}", file=sys.stderr)
        return 1

    print(
        f"check_param_metadata: {len(metadata['parameters'])} parameter(s) "
        f"described, matching {METADATA_SOURCE.name}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
