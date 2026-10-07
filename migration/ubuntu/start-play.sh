#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")" && pwd -P)"
source "$SCRIPT_DIR/common.sh"
hand_require_venv play
exec "$HAND_PYTHON" "$HAND_PROJECT_ROOT/tools/restart_local_service.py" play \
  "$HAND_PYTHON" online_control/online_motion_play/server.py \
  --config "$SCRIPT_DIR/play.json" --no-browser "$@"
