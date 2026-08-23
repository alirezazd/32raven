#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi

"""Preflight checks for a fresh 32Raven checkout (`make doctor`).

Reports per build target rather than one pass/fail, because the two firmwares
have disjoint prerequisites: `make stm32` needs the ARM toolchain and three
submodules, `make esp32` needs ESP-IDF and its installed tools. A host set up
for one is legitimately not set up for the other.

To install what is missing:  make install-deps
To skip the host toolchain entirely:  make enable-docker

The presence probes are imported by install_deps.py, so "missing" means the
same thing in both. Stdlib-only -- this runs before uv exists.
"""

from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

_SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = _SCRIPT_DIR.parent

# The floor is what CI proves, not what the language nominally needs: the build
# job installs Ubuntu's gcc-arm-none-eabi (13.2) and stm32/CMakeLists.txt asks
# for C++23 with CMAKE_CXX_STANDARD_REQUIRED. Older majors are simply untested.
ARM_GCC_MIN_MAJOR = 13

# cmake_minimum_required in the root CMakeLists.
CMAKE_MIN = (3, 16)

# Which submodule each firmware actually needs, mirroring the two jobs in
# .github/workflows/build.yml -- the stm32 job deliberately skips the recursive
# clone and names these three.
STM32_SUBMODULES = {
    "third_party/eigen": "Eigen/Dense",
    "third_party/nanoprintf": "nanoprintf.h",
    "third_party/stm32_open_pin_data": "mcu/STM32F407V(E-G)Tx.xml",
}
ESP32_SUBMODULES = {
    "third_party/mavlink": "common/mavlink.h",
    "third_party/Adafruit-GFX-Library": "Adafruit_GFX.h",
    "third_party/esp-idf": "export.sh",
}


# -- presence probes (shared with install_deps.py) ---------------------------


def have(binary: str) -> bool:
    return shutil.which(binary) is not None


def _version_tuple(cmd: list[str]) -> tuple[int, ...] | None:
    try:
        proc = subprocess.run(
            cmd, capture_output=True, text=True, errors="replace", check=False
        )
    except OSError:
        return None
    m = re.search(
        r"(\d+)\.(\d+)(?:\.(\d+))?", (proc.stdout or "") + (proc.stderr or "")
    )
    if not m:
        return None
    return tuple(int(g) for g in m.groups() if g is not None)


def cmake_status() -> tuple[bool, str]:
    if not have("cmake"):
        return False, "not found"
    ver = _version_tuple(["cmake", "--version"])
    if ver is None:
        return True, "version unreadable"
    text = ".".join(str(n) for n in ver)
    if ver[:2] < CMAKE_MIN:
        return False, f"{text} -- need >= {CMAKE_MIN[0]}.{CMAKE_MIN[1]}"
    return True, text


def arm_gcc_status() -> tuple[bool, str]:
    """(ok, detail) for the ARM compiler pair at a usable major.

    Both the C and C++ drivers are named in stm32/cmake/gcc-arm-none-eabi.cmake,
    and Debian splits them across packages -- so gcc present without g++ is a
    real state that fails at the first .cpp rather than at configure.
    """
    missing = [
        t for t in ("arm-none-eabi-gcc", "arm-none-eabi-g++") if not have(t)
    ]
    if missing:
        return False, f"{', '.join(missing)} not found"
    ver = _version_tuple(["arm-none-eabi-g++", "-dumpversion"])
    if ver is None:
        return True, "version unreadable"
    text = ".".join(str(n) for n in ver)
    if ver[0] < ARM_GCC_MIN_MAJOR:
        return False, f"{text} -- need >= {ARM_GCC_MIN_MAJOR} (C++23)"
    return True, text


def arm_binutils_status() -> tuple[bool, str]:
    """(ok, detail) for the ARM binary utilities the toolchain file names.

    objcopy makes the .bin the OTA flasher sends and size prints the build
    report. Both come from binutils-arm-none-eabi, packaged apart from gcc.
    """
    missing = [
        t
        for t in ("arm-none-eabi-objcopy", "arm-none-eabi-size")
        if not have(t)
    ]
    if missing:
        return False, f"{', '.join(missing)} not found"
    return True, "objcopy, size"


_ARM_PROBE = (
    "#include <atomic>\n"
    "#include <complex>\n"
    "#include <sys/stat.h>\n"
    "int main() { return 0; }\n"
)
_ARM_PROBE_FLAGS = [
    "-mcpu=cortex-m4",
    "-mfpu=fpv4-sp-d16",
    "-mfloat-abi=hard",
    "-std=c++23",
    "-fno-exceptions",
    "-fno-rtti",
    "-fsyntax-only",
]


def arm_headers_status() -> tuple[bool, str]:
    """(ok, detail): can the ARM compiler parse this firmware's TU shape?

    A `which` check is not enough. Debian's gcc-arm-none-eabi ships neither the
    C++ headers nor newlib, so <atomic>, <complex> and <sys/stat.h> are all
    absent until libnewlib-arm-none-eabi and libstdc++-arm-none-eabi-newlib are
    installed alongside it -- and the failure lands deep in the build instead of
    at configure. .github/workflows/build.yml installs all four for this reason.
    """
    if not have("arm-none-eabi-g++"):
        return False, "not probed -- no compiler (see above)"
    with tempfile.TemporaryDirectory() as td:
        src = Path(td) / "probe.cpp"
        src.write_text(_ARM_PROBE, encoding="utf-8")
        try:
            proc = subprocess.run(
                ["arm-none-eabi-g++", *_ARM_PROBE_FLAGS, str(src)],
                capture_output=True,
                text=True,
                errors="replace",
                check=False,
            )
        except OSError as exc:
            return False, str(exc)
    if proc.returncode == 0:
        return True, "newlib + libstdc++ headers OK"
    first = next(
        (ln for ln in (proc.stderr or "").splitlines() if ln.strip()),
        "compile probe failed",
    )
    return False, (
        f"{first.strip()} "
        "(libnewlib-arm-none-eabi + libstdc++-arm-none-eabi-newlib)"
    )


def idf_path() -> Path:
    """Resolved ESP-IDF location, honouring the Makefile's own overrides.

    The environment first, then user_config.cmake, then the submodule.
    """
    env = os.environ.get("IDF_PATH", "").strip()
    if env:
        return Path(env)
    user_config = REPO_ROOT / "user_config.cmake"
    if user_config.is_file():
        for line in user_config.read_text(
            encoding="utf-8", errors="replace"
        ).splitlines():
            stripped = line.strip()
            if stripped.startswith("#") or "IDF_PATH" not in stripped:
                continue
            parts = stripped.split('"')
            if len(parts) >= 2 and parts[1]:
                return Path(parts[1])
    return REPO_ROOT / "third_party/esp-idf"


def idf_tools_status() -> tuple[bool, str]:
    """(ok, detail) for the toolchain `make idf-install` unpacks.

    The ESP32-C3 is RISC-V, so riscv32-esp-elf is the one that has to be there;
    the xtensa toolchains install beside it and prove nothing about this build.
    """
    tools_root = Path(
        os.environ.get("IDF_TOOLS_PATH") or Path.home() / ".espressif"
    )
    riscv = tools_root / "tools" / "riscv32-esp-elf"
    if riscv.is_dir():
        return True, str(tools_root)
    return False, f"riscv32-esp-elf missing under {tools_root}"


def missing_submodules(group: dict[str, str]) -> list[str]:
    return [
        path
        for path, sentinel in group.items()
        if not (REPO_ROOT / path / sentinel).exists()
    ]


# -- reporting ---------------------------------------------------------------


class Section:
    """A group of checks, gating one build target."""

    def __init__(self, title: str, blocks: str = "") -> None:
        self.title = title
        self.blocks = blocks
        self.failed = False


def _paint(text: str, code: str, stream) -> str:
    if os.environ.get("NO_COLOR") or not stream.isatty():
        return text
    return f"\033[{code}m{text}\033[0m"


def ok(msg: str) -> None:
    print(f"{_paint('ok  ', '1;32', sys.stdout)} {msg}")


def fail(msg: str) -> None:
    # Failures go to stderr so `make doctor 2>/dev/null` reduces to what passed,
    # but the two streams buffer independently off a TTY -- flush across the
    # handover or the report reads out of order when piped.
    sys.stdout.flush()
    print(f"{_paint('fail', '1;31', sys.stderr)} {msg}", file=sys.stderr)
    sys.stderr.flush()


def warn(msg: str) -> None:
    print(f"{_paint('warn', '1;33', sys.stdout)} {msg}")


def check(section: Section, label: str, present: bool, detail: str) -> None:
    if present:
        ok(f"{label}: {detail}")
    else:
        fail(f"{label}: {detail}")
        section.failed = True


def check_optional(label: str, present: bool, detail: str) -> None:
    """Report without gating -- for tools no build target needs."""
    (ok if present else warn)(f"{label}: {detail}")


def _heading(title: str) -> None:
    print(f"\n{_paint(title, '1', sys.stdout)}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Check the host toolchain for a 32Raven build."
    )
    # CI needs this: each build job provisions one firmware's prerequisites, so
    # a whole-repo verdict would have the stm32 job fail on absent IDF tools it
    # was never going to use.
    parser.add_argument(
        "--target",
        choices=("all", "stm32", "esp32"),
        default="all",
        help="which firmware's prerequisites decide the exit code",
    )
    args = parser.parse_args()
    want_stm32 = args.target in ("all", "stm32")
    want_esp32 = args.target in ("all", "esp32")

    print(f"32raven setup check\nrepo: {REPO_ROOT}")

    common = Section("Host tools", blocks="host tools -- both firmwares")
    _heading("Host tools")
    for tool in ("git", "make", "ninja", "uv", "python3"):
        check(common, tool, have(tool), shutil.which(tool) or "not found")
    check(common, "cmake", *cmake_status())

    stm32 = Section("STM32", blocks="make stm32")
    if want_stm32:
        _heading("STM32 -- ARM bare-metal toolchain")
        check(stm32, "arm-none-eabi-gcc/g++", *arm_gcc_status())
        check(stm32, "arm-none-eabi binutils", *arm_binutils_status())
        check(stm32, "ARM C++23 headers", *arm_headers_status())
        stm32_missing = missing_submodules(STM32_SUBMODULES)
        check(
            stm32,
            "submodules",
            not stm32_missing,
            "eigen, nanoprintf, stm32_open_pin_data"
            if not stm32_missing
            else f"missing {', '.join(stm32_missing)} -- "
            f"git submodule update --init {' '.join(stm32_missing)}",
        )

    esp32 = Section("ESP32", blocks="make esp32")
    if want_esp32:
        _heading("ESP32 -- ESP-IDF")
        esp32_missing = missing_submodules(ESP32_SUBMODULES)
        check(
            esp32,
            "submodules",
            not esp32_missing,
            "mavlink, Adafruit-GFX, esp-idf"
            if not esp32_missing
            else f"missing {', '.join(esp32_missing)} -- "
            "git submodule update --init --recursive "
            f"{' '.join(esp32_missing)}",
        )
        idf = idf_path()
        check(
            esp32,
            "IDF_PATH",
            (idf / "export.sh").is_file(),
            str(idf)
            if (idf / "export.sh").is_file()
            else f"{idf} has no export.sh",
        )
        check(esp32, "IDF tools", *idf_tools_status())

    _heading("Optional")
    # clang-tidy gates check_tidy.py and check_unused_includes.py, which skip
    # themselves when it is absent -- so a host without it commits clean and
    # fails in CI, where the build jobs install it.
    check_optional(
        "clang-tidy (lint)",
        have("clang-tidy"),
        shutil.which("clang-tidy")
        or "not found -- check-tidy and check-unused-includes "
        "self-skip locally, then fail in CI",
    )
    fmt = have("clang-format") and have("rg")
    check_optional(
        "clang-format + rg (make format-cpp)",
        fmt,
        "present" if fmt else "not found",
    )
    runtime = next((r for r in ("docker", "podman") if have(r)), None)
    check_optional(
        "container runtime (make enable-docker)",
        runtime is not None,
        runtime or "not found",
    )
    # tools/esp32_client.py joins the aircraft's AP with it before an OTA flash.
    check_optional(
        "nmcli (WiFi flashing)",
        have("nmcli"),
        shutil.which("nmcli") or "not found",
    )

    print()
    selected = ([stm32] if want_stm32 else []) + ([esp32] if want_esp32 else [])
    ready = [s for s in selected if not s.failed and not common.failed]
    blocked = [s for s in (common, *selected) if s.failed]

    for section in ready:
        ok(f"ready: {section.blocks}")
    for section in blocked:
        fail(f"blocked: {section.blocks}")

    if blocked:
        print(
            _paint(
                "\nInstall what is missing:  make install-deps"
                "\nOr skip the host toolchain:  make enable-docker",
                "1;31",
                sys.stderr,
            ),
            file=sys.stderr,
        )
        return 1
    nxt = "make all" if args.target == "all" else f"make {args.target}"
    print(_paint(f"\nSetup looks ready. Next: {nxt}", "1;32", sys.stdout))
    return 0


if __name__ == "__main__":
    sys.exit(main())
