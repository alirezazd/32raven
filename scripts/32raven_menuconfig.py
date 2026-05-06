#!/usr/bin/env python3

from __future__ import annotations

import argparse
import os
import pathlib

import kconfiglib
import menuconfig


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--kconfig", required=True)
    parser.add_argument("--config", required=True)
    args = parser.parse_args()

    kconfig_path = pathlib.Path(args.kconfig).resolve()
    config_path = pathlib.Path(args.config).resolve()

    os.environ["KCONFIG_CONFIG"] = str(config_path)

    kconf = kconfiglib.Kconfig(str(kconfig_path))
    if config_path.exists():
        kconf.load_config(str(config_path))

    menuconfig.menuconfig(kconf)
    kconf.write_config(str(config_path))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
