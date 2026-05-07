#!/bin/bash
# Configure VSCode settings for 32Raven development.
#
# Merge-aware: deep-merges the managed keys below into any existing
# .vscode/settings.json, so personal customizations (font size, themes,
# extra excludes you added by hand) are preserved across re-runs.
#
# Managed keys:
#   - git submodule scan disabled (Source Control perf with ESP-IDF)
#   - file watcher / search exclusions for vendored code and build dirs
#   - clangd.arguments: --query-driver for arm-none-eabi GCC, indexing, tidy
#   - editor.formatOnSave + per-language clangd formatter for c/cpp/h/hpp
#
# Run again any time the managed list changes; your edits stay.

set -euo pipefail

mkdir -p .vscode

python3 - <<'PY'
import json
import pathlib
import sys

MANAGED = {
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
        "--query-driver=/usr/bin/arm-none-eabi-*,/usr/local/bin/arm-none-eabi-*,/opt/**/arm-none-eabi-*,**/bin/arm-none-eabi-*",
        "--background-index",
        "--clang-tidy",
        "--header-insertion=never",
    ],
    "editor.formatOnSave": True,
    "[c]":   {"editor.defaultFormatter": "llvm-vs-code-extensions.vscode-clangd"},
    "[cpp]": {"editor.defaultFormatter": "llvm-vs-code-extensions.vscode-clangd"},
    "[h]":   {"editor.defaultFormatter": "llvm-vs-code-extensions.vscode-clangd"},
    "[hpp]": {"editor.defaultFormatter": "llvm-vs-code-extensions.vscode-clangd"},
}


def deep_merge(base: dict, override: dict) -> dict:
    """Recursively merge `override` into `base`.

    Dicts merge key-wise (so a user's extra entries under e.g.
    files.watcherExclude survive). Scalars and lists from `override` replace
    whatever was in `base` — for arrays we manage like clangd.arguments,
    full replacement is the right call.
    """
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            deep_merge(base[key], value)
        else:
            base[key] = value
    return base


path = pathlib.Path(".vscode/settings.json")
existing: dict = {}

if path.exists():
    text = path.read_text().strip()
    if text:
        try:
            existing = json.loads(text)
        except json.JSONDecodeError as exc:
            # Don't clobber an unreadable file silently. JSONC (with comments)
            # would land here too — strip comments by hand or back the file
            # up and re-run.
            print(
                f"ERROR: {path} exists but is not valid JSON ({exc}).\n"
                f"  Fix or remove the file, then re-run `make setup-vscode`.",
                file=sys.stderr,
            )
            sys.exit(1)
        if not isinstance(existing, dict):
            print(
                f"ERROR: {path} top-level is not a JSON object.",
                file=sys.stderr,
            )
            sys.exit(1)

before = json.dumps(existing, sort_keys=True)
deep_merge(existing, MANAGED)
after = json.dumps(existing, sort_keys=True)

path.write_text(json.dumps(existing, indent=2) + "\n")

if before == after:
    print(f"✓ {path} already up to date")
else:
    print(f"✓ {path} merged (managed keys updated; user keys preserved)")
PY

echo "  Reload VSCode: Cmd/Ctrl+Shift+P → 'Developer: Reload Window'"
