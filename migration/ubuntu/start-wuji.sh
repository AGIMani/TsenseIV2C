#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")" && pwd -P)"
export WUJI_TELEMETRY_DISABLE=1
unset WUJI_BRIDGE_URL
args=(--wuji-source sdk)
if [[ -n "${WUJI_DEVICE_ADDRESS:-}" ]]; then args+=(--wuji-address "$WUJI_DEVICE_ADDRESS"); fi
if [[ -n "${WUJI_DEVICE_SERIAL:-}" ]]; then args+=(--wuji-serial "$WUJI_DEVICE_SERIAL"); fi
exec bash "$SCRIPT_DIR/start-track.sh" "${args[@]}" "$@"
