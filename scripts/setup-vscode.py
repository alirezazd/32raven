#!/usr/bin/env python3

# SPDX-License-Identifier: GPL-3.0-only
# Copyright (C) 2026 Alireza Azadi

"""Configure VSCode settings for 32Raven development.

Run:
  uv run --quiet --script scripts/setup-vscode.py

Merge-aware: deep-merges the managed keys below into any existing
.vscode/settings.json, so personal customizations (font size, themes, extra
excludes added by hand) survive a re-run.

Managed keys:
  - git submodule scan disabled (Source Control perf with ESP-IDF)
  - file watcher / search exclusions for vendored code and build dirs
  - clangd.arguments: --query-driver for arm-none-eabi + riscv32-esp-elf GCC,
    indexing, tidy
  - editor.formatOnSave + per-language clangd formatter for c/cpp/h/hpp
"""

from __future__ import annotations

import json
import pathlib
import sys

SETTINGS_PATH = pathlib.Path(".vscode/settings.json")

MANAGED: dict[str, object] = {
    "git.detectSubmodules": False,
    "git.scanRepositories": [],
    "files.watcherExclude": {
        "**/third_party/esp-idf/**": True,
        "**/third_party/mavlink/**": True,
        "**/third_party/Adafruit-GFX-Library/**": True,
        "**/build/**": True,
        "**/.docker/**": True,
    },
    "search.exclude": {
        "**/third_party/esp-idf": True,
        "**/build": True,
        "**/.docker": True,
    },
    "clangd.arguments": [
        "--query-driver=/usr/bin/arm-none-eabi-*,/usr/local/bin/arm-none-eabi-*,"
        "/opt/**/arm-none-eabi-*,**/bin/arm-none-eabi-*,**/bin/riscv32-esp-elf-*",
        "--background-index",
        "--clang-tidy",
        "--header-insertion=never",
    ],
    "editor.formatOnSave": True,
    "[c]": {"editor.defaultFormatter": "llvm-vs-code-extensions.vscode-clangd"},
    "[cpp]": {
        "editor.defaultFormatter": "llvm-vs-code-extensions.vscode-clangd"
    },
    "[h]": {"editor.defaultFormatter": "llvm-vs-code-extensions.vscode-clangd"},
    "[hpp]": {
        "editor.defaultFormatter": "llvm-vs-code-extensions.vscode-clangd"
    },
}


def deep_merge(base: dict, override: dict) -> dict:
    """Recursively merge `override` into `base`.

    Dicts merge key-wise, so a user's extra entries under e.g.
    files.watcherExclude survive. Scalars and lists from `override` replace
    what was there: for arrays we manage wholesale, like clangd.arguments,
    replacement is the right call.
    """
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            deep_merge(base[key], value)
        else:
            base[key] = value
    return base


def load_settings() -> dict:
    if not SETTINGS_PATH.exists():
        return {}

    text = SETTINGS_PATH.read_text().strip()
    if not text:
        return {}

    try:
        settings = json.loads(text)
    except json.JSONDecodeError as exc:
        # Don't clobber an unreadable file silently. JSONC (with comments)
        # lands here too — strip comments by hand or back the file up.
        sys.exit(
            f"ERROR: {SETTINGS_PATH} exists but is not valid JSON ({exc}).\n"
            f"  Fix or remove the file, then re-run `make setup-vscode`."
        )

    if not isinstance(settings, dict):
        sys.exit(f"ERROR: {SETTINGS_PATH} top-level is not a JSON object.")

    return settings


def main() -> int:
    SETTINGS_PATH.parent.mkdir(parents=True, exist_ok=True)

    settings = load_settings()
    before = json.dumps(settings, sort_keys=True)

    deep_merge(settings, MANAGED)

    after = json.dumps(settings, sort_keys=True)
    SETTINGS_PATH.write_text(json.dumps(settings, indent=2) + "\n")

    if before == after:
        print(f"✓ {SETTINGS_PATH} already up to date")
    else:
        print(
            f"✓ {SETTINGS_PATH} merged "
            "(managed keys updated; user keys preserved)"
        )

    print("  Reload VSCode: Cmd/Ctrl+Shift+P → 'Developer: Reload Window'")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
