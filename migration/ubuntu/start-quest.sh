#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")" && pwd -P)"
# Compatibility alias: all input sources now share the ordinary launcher.
exec bash "$SCRIPT_DIR/start-track.sh" "$@"
