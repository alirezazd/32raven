#!/usr/bin/env python3

# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi

"""Emit the parameter dictionary the ground station describes 32Raven with.

The names the bridge serves are PX4's, so a ground station that has PX4's
dictionary compiled in will describe our parameters with PX4's text: wrong
where a name collides and absent where it does not. This writes the
replacement -- the same schema, our parameters, our words.

Two inputs, and the split is deliberate. What the vehicle serves comes from
`kParamTable` and the RC calibration encoder, because those are the truth and
restating them here would only let the two drift. What each parameter means
comes from `config/mavlink_params.toml`, because no table in the firmware
holds it and none should: none of this text needs to reach flash.

Run:
  uv run --quiet --script scripts/generate_param_metadata.py --output out.json
"""

from __future__ import annotations

import argparse
import json
import pathlib
import re
import sys
import tomllib

REPO_ROOT = pathlib.Path(__file__).resolve().parent.parent
PARAM_SOURCE = REPO_ROOT / "esp32" / "services" / "mavlink_param.cpp"
METADATA_SOURCE = REPO_ROOT / "config" / "mavlink_params.toml"

sys.path.insert(0, str(REPO_ROOT / "scripts"))

from generate_firmware_ver import resolve_firmware_version  # noqa: E402

# The schema's spelling of each MAVLink type. Only the ones kParamTable uses:
# a type appearing there without a row here is a new type nobody has decided
# how to describe, which should stop the build rather than be guessed.
MAV_TYPE_TO_SCHEMA = {
    "MAV_PARAM_TYPE_UINT8": "UINT8",
    "MAV_PARAM_TYPE_INT8": "INT8",
    "MAV_PARAM_TYPE_UINT16": "UINT16",
    "MAV_PARAM_TYPE_INT16": "INT16",
    "MAV_PARAM_TYPE_UINT32": "UINT32",
    "MAV_PARAM_TYPE_INT32": "INT32",
    "MAV_PARAM_TYPE_REAL32": "FLOAT",
}

# Optional per-parameter keys, in the order the schema lists them so a diff
# between two generated files reads as a change of content and not of order.
OPTIONAL_KEYS = (
    "shortDesc",
    "longDesc",
    "units",
    "min",
    "max",
    "default",
    "increment",
    "decimalPlaces",
    "group",
    "category",
    "rebootRequired",
    "readOnly",
    "volatile",
    "values",
    "bitmask",
)

_TABLE_RE = re.compile(
    r"inline constexpr ParamDef kParamTable\[\] = \{(.*?)\n\};", re.DOTALL
)
_ROW_RE = re.compile(r'\{"([A-Z0-9_]+)",\s*(MAV_PARAM_TYPE_\w+)')
_RC_BLOCK_RE = re.compile(
    r"Mavlink::TryEncodeRcCalibrationParam\b(.*?)\n\}", re.DOTALL
)
_RC_FIELD_RE = re.compile(
    r'"RC%u_(\w+)"[^;]*;\s*encoded\.type = (MAV_PARAM_TYPE_\w+)'
)
_CHANNEL_COUNT_RE = re.compile(
    r"kRcCalibrationChannelCount\s*=\s*(\d+)u?", re.MULTILINE
)


class GenerateError(SystemExit):
    def __init__(self, message: str) -> None:
        super().__init__(f"generate_param_metadata: {message}")


def _read(path: pathlib.Path) -> str:
    try:
        return path.read_text(encoding="utf-8")
    except OSError as exc:
        raise GenerateError(f"cannot read {path}") from exc


def _schema_type(mav_type: str, name: str) -> str:
    try:
        return MAV_TYPE_TO_SCHEMA[mav_type]
    except KeyError as exc:
        raise GenerateError(
            f"{name} uses {mav_type}, which has no schema spelling"
        ) from exc


def served_parameters() -> list[tuple[str, str]]:
    """Every parameter the bridge answers for, as (name, schema type)."""
    source = _read(PARAM_SOURCE)

    table = _TABLE_RE.search(source)
    if not table:
        raise GenerateError("kParamTable not found in the bridge source")
    served = [
        (name, _schema_type(mav_type, name))
        for name, mav_type in _ROW_RE.findall(table.group(1))
    ]
    if not served:
        raise GenerateError("kParamTable parsed as empty")

    rc_block = _RC_BLOCK_RE.search(source)
    if not rc_block:
        raise GenerateError("the RC calibration encoder was not found")
    rc_fields = _RC_FIELD_RE.findall(rc_block.group(1))
    if not rc_fields:
        raise GenerateError("the RC calibration encoder named no fields")

    message_source = _read(REPO_ROOT / "libs" / "message.hpp")
    channels = _CHANNEL_COUNT_RE.search(message_source)
    if not channels:
        raise GenerateError("kRcCalibrationChannelCount not found")

    for channel in range(1, int(channels.group(1)) + 1):
        for suffix, mav_type in rc_fields:
            name = f"RC{channel}_{suffix}"
            served.append((name, _schema_type(mav_type, name)))

    return served


def _entry(name: str, schema_type: str, described: dict) -> dict:
    entry: dict = {"name": name, "type": schema_type}
    for key in OPTIONAL_KEYS:
        if key in described:
            entry[key] = described[key]
    return entry


def build_metadata() -> dict:
    described = tomllib.loads(_read(METADATA_SOURCE))
    fixed = described.get("params", {})
    rc_fields = described.get("rc_calibration", {})

    parameters = []
    for name, schema_type in served_parameters():
        rc = re.fullmatch(r"RC(\d+)_(\w+)", name)
        if rc and rc.group(2) in rc_fields:
            channel = rc.group(1)
            entry = {
                key: (
                    value.replace("{n}", channel)
                    if isinstance(value, str)
                    else value
                )
                for key, value in rc_fields[rc.group(2)].items()
            }
        elif name in fixed:
            entry = fixed[name]
        else:
            raise GenerateError(
                f"{name} is served by the bridge but not described in "
                f"{METADATA_SOURCE.name}"
            )
        parameters.append(_entry(name, schema_type, entry))

    return {
        "version": int(described.get("schema_version", 1)),
        # Not the schema's `version`, and deliberately under a key the schema
        # does not define: a ground station that cached this file keys on the
        # schema version, and stamping the firmware there would make every
        # build look like a new format.
        "firmwareVersion": resolve_firmware_version(),
        "parameters": parameters,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=pathlib.Path)
    args = parser.parse_args()

    rendered = json.dumps(build_metadata(), indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(rendered, encoding="utf-8")
    else:
        sys.stdout.write(rendered)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
