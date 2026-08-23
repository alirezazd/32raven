#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi

"""Install the 32Raven host dependencies this machine is missing.

Detects the package manager, runs the SAME presence probes `make doctor` uses
(imported from check_setup.py), and installs only the packages behind the
checks that fail. Re-running after a partial install converges.

  scripts/install_deps.py             install what is missing (asks first)
  scripts/install_deps.py --dry-run   print the commands, install nothing
  scripts/install_deps.py --list      print the missing packages, one per line
  scripts/install_deps.py --yes       no confirmation prompt
  scripts/install_deps.py --no-uv     skip the uv bootstrap

Covers apt (Debian/Ubuntu), dnf (Fedora/RHEL), pacman (Arch) and brew (macOS).
Submodules and ESP-IDF tools are not package-manager work, so they are reported
as the git / make commands that fix them rather than installed here.

Stdlib-only: this bootstraps the environment before uv and any venv exist.
"""

from __future__ import annotations

import argparse
import os
import platform
import shutil
import subprocess
import sys
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path

_SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(_SCRIPT_DIR))

from check_setup import (  # noqa: E402 -- shared probes = shared "missing"
    ESP32_SUBMODULES,
    STM32_SUBMODULES,
    arm_binutils_status,
    arm_gcc_status,
    arm_headers_status,
    cmake_status,
    have,
    idf_tools_status,
    missing_submodules,
)


@dataclass
class Profile:
    """How a package manager installs -- not what (that is DEPS)."""

    name: str
    sudo: bool
    install: list[str]
    refresh: list[str] | None = None
    uv_pkg: str | None = None  # manager's own uv package, else pipx
    note: str = ""


PROFILES = {
    "apt": Profile(
        "apt (Debian/Ubuntu)",
        True,
        install=["apt-get", "install", "-y", "--no-install-recommends"],
        refresh=["apt-get", "update"],
        note="if uv is still missing afterwards: pipx ensurepath, then restart "
        "the shell",
    ),
    "dnf": Profile(
        "dnf (Fedora/RHEL)", True, install=["dnf", "install", "-y"], uv_pkg="uv"
    ),
    "pacman": Profile(
        "pacman (Arch)",
        True,
        install=["pacman", "-S", "--needed", "--noconfirm"],
        refresh=["pacman", "-Sy"],
        uv_pkg="uv",
    ),
    "brew": Profile(
        "Homebrew (macOS)",
        False,
        install=["brew", "install"],
        uv_pkg="uv",
        note="the ARM toolchain is a cask: "
        "brew install --cask gcc-arm-embedded",
    ),
}


@dataclass
class Dep:
    label: str
    probe: Callable[[], bool]
    packages: dict[str, list[str]] = field(default_factory=dict)
    # Which firmware needs it; "host" is everything's floor. Mirrors the
    # doctor's --target so a CI job installs only its own prerequisites.
    target: str = "host"


def _same(package: str) -> dict[str, list[str]]:
    """Package map for the common case: one name across all four managers."""
    return {mgr: [package] for mgr in PROFILES}


# Debian splits the ARM toolchain four ways and the compiler alone cannot build
# a C++ TU -- .github/workflows/build.yml installs all four for that reason.
# Fedora's equivalents carry the CodeSourcery `-cs` suffix and fold newlib into
# one noarch package; Arch ships the compiler and newlib as two.
_ARM_PACKAGES = {
    "apt": ["gcc-arm-none-eabi", "binutils-arm-none-eabi",
            "libnewlib-arm-none-eabi", "libstdc++-arm-none-eabi-newlib"],
    "dnf": ["arm-none-eabi-gcc-cs", "arm-none-eabi-gcc-cs-c++",
            "arm-none-eabi-binutils-cs", "arm-none-eabi-newlib"],
    "pacman": ["arm-none-eabi-gcc", "arm-none-eabi-binutils",
               "arm-none-eabi-newlib"],
    "brew": [],  # cask, see the profile note
}  # fmt: skip

# ESP-IDF's own host prerequisites. `make idf-install` downloads the toolchain
# but builds its Python env against these, so a missing one surfaces there
# rather than here -- which is why they install unconditionally with the rest.
_IDF_PREREQS = {
    "apt": ["flex", "bison", "gperf", "ccache", "libffi-dev", "libssl-dev",
            "dfu-util", "libusb-1.0-0"],
    "dnf": ["flex", "bison", "gperf", "ccache", "libffi-devel",
            "openssl-devel", "dfu-util", "libusbx"],
    "pacman": ["flex", "bison", "gperf", "ccache", "libffi", "openssl",
               "dfu-util", "libusb"],
    "brew": ["flex", "bison", "gperf", "ccache", "dfu-util", "libusb"],
}  # fmt: skip

_NINJA = {
    "apt": ["ninja-build"], "dnf": ["ninja-build"],
    "pacman": ["ninja"], "brew": ["ninja"],
}  # fmt: skip
_PYTHON = {
    "apt": ["python3", "python3-venv"], "dnf": ["python3"],
    "pacman": ["python"], "brew": ["python"],
}  # fmt: skip
# Fedora and Arch each ship clang-tidy and clang-format from one package.
_CLANG_TIDY = {
    "apt": ["clang-tidy"], "dnf": ["clang-tools-extra"],
    "pacman": ["clang"], "brew": ["llvm"],
}  # fmt: skip
_CLANG_FORMAT = {
    "apt": ["clang-format"], "dnf": ["clang-tools-extra"],
    "pacman": ["clang"], "brew": ["clang-format"],
}  # fmt: skip

DEPS: list[Dep] = [
    Dep("git", lambda: have("git"), _same("git")),
    Dep("make", lambda: have("make"), _same("make")),
    Dep("cmake", lambda: cmake_status()[0], _same("cmake")),
    Dep("ninja", lambda: have("ninja"), _NINJA),
    Dep("python3", lambda: have("python3"), _PYTHON),
    Dep("clang-tidy", lambda: have("clang-tidy"), _CLANG_TIDY),
    Dep("clang-format", lambda: have("clang-format"), _CLANG_FORMAT),
    Dep("ripgrep", lambda: have("rg"), _same("ripgrep")),
    Dep("ARM toolchain", lambda: arm_gcc_status()[0], _ARM_PACKAGES, "stm32"),
    Dep(
        "ARM binutils", lambda: arm_binutils_status()[0], _ARM_PACKAGES, "stm32"
    ),
    Dep(
        "ARM C++ headers",
        lambda: arm_headers_status()[0],
        _ARM_PACKAGES,
        "stm32",
    ),
    Dep(
        "ESP-IDF prerequisites",
        lambda: have("dfu-util") and have("ccache"),
        _IDF_PREREQS,
        "esp32",
    ),
]


def detect() -> str:
    system = platform.system()
    if system == "Darwin":
        return "brew"
    if system == "Linux":
        tokens: set[str] = set()
        osr = Path("/etc/os-release")
        if osr.is_file():
            for line in osr.read_text(
                encoding="utf-8", errors="replace"
            ).splitlines():
                key, _, val = line.partition("=")
                if key in ("ID", "ID_LIKE"):
                    tokens.update(val.strip().strip("\"'").lower().split())
        for mgr, keys in (
            ("apt", {"debian", "ubuntu"}),
            ("dnf", {"fedora", "rhel", "centos"}),
            ("pacman", {"arch"}),
        ):
            if tokens & keys:
                return mgr
        for mgr, binary in (
            ("apt", "apt-get"),
            ("dnf", "dnf"),
            ("pacman", "pacman"),
        ):
            if shutil.which(binary):
                return mgr
    raise SystemExit(
        f"Unsupported OS/distro: {system}.\n"
        "Install the deps by hand (see the Dockerfile for the apt set), or "
        "run the whole toolchain in a container with `make enable-docker`."
    )


def _dedupe(seq: list[str]) -> list[str]:
    seen: set[str] = set()
    out: list[str] = []
    for item in seq:
        if item not in seen:
            seen.add(item)
            out.append(item)
    return out


def missing_packages(mgr: str, target: str) -> list[str]:
    wanted = {"host", target} if target != "all" else {"host", "stm32", "esp32"}
    pkgs: list[str] = []
    for dep in DEPS:
        if dep.target in wanted and not dep.probe():
            pkgs += dep.packages.get(mgr, [])
    return _dedupe(pkgs)


def _sudo(cmd: list[str], profile: Profile) -> list[str]:
    is_root = hasattr(os, "geteuid") and os.geteuid() == 0
    if profile.sudo and not is_root and shutil.which("sudo"):
        return ["sudo", *cmd]
    return cmd


def build_commands(
    profile: Profile, pkgs: list[str], want_uv: bool
) -> list[list[str]]:
    uv_step: list[str] | None = None
    if want_uv and not have("uv"):
        if profile.uv_pkg:
            pkgs = pkgs + [profile.uv_pkg]
        else:
            pkgs = pkgs + ["pipx"]  # apt has no uv package
            uv_step = ["pipx", "install", "uv"]
    cmds: list[list[str]] = []
    if not pkgs and not uv_step:
        return cmds
    if profile.refresh:
        cmds.append(_sudo(profile.refresh, profile))
    if pkgs:
        cmds.append(_sudo([*profile.install, *pkgs], profile))
    if uv_step:
        cmds.append(uv_step)
    return cmds


def followups(target: str) -> list[str]:
    """Steps no package manager can do: the submodules and the IDF toolchain."""
    steps: list[str] = []
    if target in ("all", "stm32"):
        stm32 = missing_submodules(STM32_SUBMODULES)
        if stm32:
            steps.append(f"git submodule update --init {' '.join(stm32)}")
    if target in ("all", "esp32"):
        esp32 = missing_submodules(ESP32_SUBMODULES)
        if esp32:
            steps.append(
                f"git submodule update --init --recursive {' '.join(esp32)}"
            )
        if not idf_tools_status()[0]:
            steps.append("make idf-install")
    return steps


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Install the 32Raven host deps this machine is missing."
    )
    parser.add_argument(
        "--dry-run", action="store_true", help="print commands only"
    )
    parser.add_argument(
        "--list",
        dest="list_only",
        action="store_true",
        help="print missing packages only",
    )
    parser.add_argument(
        "--yes", action="store_true", help="no confirmation prompt"
    )
    parser.add_argument(
        "--no-uv", action="store_true", help="skip the uv bootstrap"
    )
    parser.add_argument(
        "--target",
        choices=("all", "stm32", "esp32"),
        default="all",
        help="install one firmware's prerequisites instead of both",
    )
    args = parser.parse_args()

    mgr = detect()
    profile = PROFILES[mgr]
    pkgs = missing_packages(mgr, args.target)

    if args.list_only:
        print("\n".join(pkgs))
        return 0

    cmds = build_commands(profile, pkgs, want_uv=not args.no_uv)
    extra = followups(args.target)

    if not cmds:
        print(f"{profile.name}: all packages present.")
    else:
        print(f"{profile.name} -- installing missing packages:")
        for cmd in cmds:
            print(f"  $ {' '.join(cmd)}")
        if profile.note:
            print(f"  note: {profile.note}")

    if extra:
        print("\nThen, to finish the checkout:")
        for step in extra:
            print(f"  $ {step}")

    if not cmds:
        return 0
    if args.dry_run:
        return 0
    if not args.yes:
        try:
            reply = input("\nProceed? [y/N] ").strip().lower()
        except EOFError:
            reply = ""
        if reply not in ("y", "yes"):
            print("Aborted.")
            return 1

    # One sudo prompt up front, cached for the steps that follow.
    if any(cmd and cmd[0] == "sudo" for cmd in cmds):
        if subprocess.run(["sudo", "-v"], check=False).returncode != 0:
            print("sudo authentication failed.", file=sys.stderr)
            return 1

    for cmd in cmds:
        if subprocess.run(cmd, check=False).returncode != 0:
            print(f"failed: {' '.join(cmd)}", file=sys.stderr)
            return 1

    print("\nDone. Next: make doctor")
    return 0


if __name__ == "__main__":
    sys.exit(main())
