#!/bin/bash
# Configure VSCode settings for 32Raven development.
# Disables submodule detection (fixing perf issues with ESP-IDF's nested submodules),
# sets up file watchers and search exclusions for vendored code, and points clangd
# at the arm-none-eabi GCC driver so it can resolve the C++ stdlib (cstdint, etc.)
# under the bare-metal cross-compiler.

set -euo pipefail

mkdir -p .vscode

cat > .vscode/settings.json <<'EOF'
{
  "git.detectSubmodules": false,
  "git.scanRepositories": [],
  "files.watcherExclude": {
    "**/third_party/esp-idf/**": true,
    "**/third_party/mavlink/**": true,
    "**/third_party/Adafruit-GFX-Library/**": true,
    "**/build/**": true,
    "**/.docker/**": true
  },
  "search.exclude": {
    "**/third_party/esp-idf": true,
    "**/build": true,
    "**/.docker": true
  },
  "clangd.arguments": [
    "--query-driver=/usr/bin/arm-none-eabi-*,/usr/local/bin/arm-none-eabi-*,/opt/**/arm-none-eabi-*,**/bin/arm-none-eabi-*",
    "--background-index",
    "--clang-tidy",
    "--header-insertion=never"
  ]
}
EOF

echo "✓ VSCode settings configured in .vscode/settings.json"
echo "  Reload VSCode: Cmd/Ctrl+Shift+P → 'Developer: Reload Window'"
