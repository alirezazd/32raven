#!/bin/bash
# Configure VSCode settings for 32Raven development.
# Disables submodule detection (fixing perf issues with ESP-IDF's nested submodules),
# sets up file watchers and search exclusions for vendored code.

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
  }
}
EOF

echo "✓ VSCode settings configured in .vscode/settings.json"
echo "  Reload VSCode: Cmd/Ctrl+Shift+P → 'Developer: Reload Window'"
