#!/usr/bin/env python3

# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi

# /// script
# dependencies = [
#     "jinja2",
#     "kconfiglib",
# ]
# ///

"""Generate common_config.hpp, included by both firmwares.

The top-level CMake owns this output and writes it under the binary dir, not
the source tree. Naming a path inside libs/ puts a stale copy ahead of the
generated one on the include path, since libs/ is searched first.

Run:
  uv run --quiet --script scripts/generate_common_config.py \
      --kconfig config/Kconfig --config config/32raven.config \
      --out build/Ninja/generated/common_config.hpp
"""

from __future__ import annotations

import argparse
import pathlib
import sys
from typing import Any

import kconfiglib

# Sibling module.
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
from kconfig_gen import (
    autogen_warning,
    choice_value,
    render_template,
    sym_int,
    template_env,
)

TEMPLATE_NAME = "common_config.hpp.j2"

FCLINK_BAUD_CHOICES = {
    f"COMMON_FCLINK_BAUD_{rate}": str(rate)
    for rate in (
        9600,
        19200,
        38400,
        57600,
        115200,
        230400,
        460800,
        921600,
        1000000,
        2000000,
        5000000,
    )
}


# Keyed on the frame, so a new one cannot arrive with half its facts. The
# enumerator name is the geometry the mixer selects; sys_autostart is the PX4
# airframe the ground station is told, and the two must describe one aircraft.
# `motors` places each motor in DShot channel order (M1..) on the body's unit
# square -- NED axes, x forward and y right -- with the sense its prop turns
# seen from above. The motor count is the length of that list and the mixer's
# factors are derived from it (mix_factors), so the frame, the allocator and
# anything reading the header see one record and nobody types a matrix.
AIRFRAMES: dict[str, dict[str, Any]] = {
    "COMMON_AIRFRAME_QUAD_X": {
        "name": "kQuadX",
        "sys_autostart": 4001,
        # Betaflight mixerQuadX numbering.
        "motors": [
            {"name": "front-right", "x": 1, "y": 1, "ccw": True},
            {"name": "back-right", "x": -1, "y": 1, "ccw": False},
            {"name": "back-left", "x": -1, "y": -1, "ccw": True},
            {"name": "front-left", "x": 1, "y": -1, "ccw": False},
        ],
    },
    "COMMON_AIRFRAME_QUAD_PLUS": {
        "name": "kQuadPlus",
        "sys_autostart": 5001,
        "motors": [
            {"name": "front", "x": 1, "y": 0, "ccw": True},
            {"name": "right", "x": 0, "y": 1, "ccw": False},
            {"name": "back", "x": -1, "y": 0, "ccw": True},
            {"name": "left", "x": 0, "y": -1, "ccw": False},
        ],
    },
}


Motor = dict[str, Any]


def mix_factors(motor: Motor) -> dict[str, float]:
    """The allocator's roll/pitch/yaw factors from a motor's placement.

    Aerospace right-hand rule about NED body axes, as multirotor_mixer.hpp
    documents its inputs: +roll is right wing down, so the motors on the
    right (y > 0) slow and those on the left speed up (roll = -y); +pitch is
    nose up, so the front motors (x > 0) speed up (pitch = +x); +yaw is nose
    right, produced by speeding up the counter-clockwise props, whose drag
    reaction turns the body clockwise (yaw = +1 for a CCW prop).
    """
    return {
        "roll": float(-int(motor["y"])),
        "pitch": float(int(motor["x"])),
        "yaw": 1.0 if motor["ccw"] else -1.0,
    }


AXES = ("roll", "pitch", "yaw")


def axis_norm_sq(motors: list[Motor], axis: str) -> float:
    """Squared norm of one factor column.

    Mix() projects the motor vector onto each column to recover the torque it
    actually delivered, and an orthogonal projection divides by this. It is 4
    on every column of a quad-X, where all four motors contribute to all three
    axes, but only 2 for roll and pitch on a quad-+, where the two motors on
    the perpendicular arm have no moment -- so the divisor is per frame and
    per axis rather than the motor count.
    """
    return sum(mix_factors(m)[axis] ** 2 for m in motors)


def validate_airframe(name: str, frame: dict[str, Any]) -> None:
    """A placement the allocator's projection arithmetic can rely on.

    Every arm on the unit square and nowhere near the centre; each axis
    balanced (the factor columns sum to zero) so a pure thrust command yields
    no torque; each axis actually driven, since a column of zeros is an axis
    the aircraft cannot command and a projection that would divide by it.
    """
    motors: list[Motor] = frame["motors"]
    if not motors:
        raise SystemExit(f"{name}: an airframe needs at least one motor")
    for m in motors:
        if m["x"] not in (-1, 0, 1) or m["y"] not in (-1, 0, 1):
            raise SystemExit(
                f"{name}: motor {m['name']} is off the unit square"
            )
        if m["x"] == 0 and m["y"] == 0:
            raise SystemExit(f"{name}: motor {m['name']} sits at the centre")
    for axis in AXES:
        if sum(mix_factors(m)[axis] for m in motors) != 0.0:
            raise SystemExit(f"{name}: the {axis} factors do not balance")
        if axis_norm_sq(motors, axis) == 0.0:
            raise SystemExit(f"{name}: no motor produces {axis} torque")

    # Mutually orthogonal columns, and each orthogonal to thrust (+1 per
    # motor, which the balance check above already gives). Without that,
    # projecting onto one axis picks up torque delivered about another and
    # the anti-windup back-calculation drains the wrong integrator.
    for i, first in enumerate(AXES):
        for second in AXES[i + 1 :]:
            dot = sum(
                mix_factors(m)[first] * mix_factors(m)[second] for m in motors
            )
            if dot != 0.0:
                raise SystemExit(
                    f"{name}: the {first} and {second} factors are not "
                    f"orthogonal, so Mix() cannot separate the two torques"
                )


def airframe_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    selected = choice_value(
        kconf, {sym: sym for sym in AIRFRAMES}
    )
    # Every frame, not just the selected one: a malformed entry that ships
    # unselected is one nobody finds until the day it is picked.
    for name, frame in AIRFRAMES.items():
        validate_airframe(name, frame)

    frame = AIRFRAMES[selected]
    motors = frame["motors"]
    return {
        "names": [f["name"] for f in AIRFRAMES.values()],
        "name": frame["name"],
        "motor_count": len(motors),
        "sys_autostart": frame["sys_autostart"],
        # Placement and spin stay here, where the factors are derived from
        # them; the header carries only what Mix() reads.
        "motors": [{"name": m["name"], **mix_factors(m)} for m in motors],
        "axis_inv_norm_sq": [
            1.0 / axis_norm_sq(motors, axis) for axis in AXES
        ],
    }


def fclink_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    exchange_interval_ms = sym_int(kconf, "COMMON_FCLINK_EXCHANGE_INTERVAL_MS")
    return {
        "baud": choice_value(kconf, FCLINK_BAUD_CHOICES),
        "exchange_interval_ms": exchange_interval_ms,
        # Derived, so the window can never be shorter than the renewal cadence.
        "peer_timeout_ms": exchange_interval_ms
        * sym_int(kconf, "COMMON_FCLINK_PEER_TIMEOUT_EXCHANGES"),
    }


def template_context(kconf: kconfiglib.Kconfig) -> dict[str, object]:
    """Everything the template reads apart from the autogen banner.

    Separate from main() because check_config_sweep renders this header too,
    and a second hand-built copy of this dict would go stale the next time a
    key is added here.
    """
    return {
        "airframe": airframe_context(kconf),
        "fclink": fclink_context(kconf),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--kconfig", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()

    kconfig_path = pathlib.Path(args.kconfig).resolve()
    config_path = pathlib.Path(args.config).resolve()
    out_path = pathlib.Path(args.out).resolve()

    kconf = kconfiglib.Kconfig(str(kconfig_path))
    if config_path.exists():
        kconf.load_config(str(config_path))

    text = render_template(
        template_env(),
        TEMPLATE_NAME,
        {
            "autogen_warning": autogen_warning(config_path),
            **template_context(kconf),
        },
    )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(text, encoding="ascii")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
