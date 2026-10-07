#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")" && pwd -P)"
source "$SCRIPT_DIR/common.sh"
hand_require_venv track
export MUJOCO_GL=disable
if [[ -z "${LEAPSDK_INSTALL_LOCATION:-}" ]]; then
  LEAPSDK_INSTALL_LOCATION="$("$HAND_PYTHON" -c 'import sysconfig; print(sysconfig.get_path("purelib"))')"
  export LEAPSDK_INSTALL_LOCATION
fi
network_args=(--host 127.0.0.1)
auto_https=1
# Keep diagnostics offline and preserve explicitly configured deployments.
for arg in "$@"; do
  case "$arg" in
    --check|--help|-h|--public|--host|--host=*|--tls-cert|--tls-cert=*|--tls-key|--tls-key=*) auto_https=0 ;;
  esac
done
if [[ "$auto_https" == 1 ]]; then
  QUEST_ADDRESS="$("$HAND_PYTHON" "$HAND_PROJECT_ROOT/tools/prepare_quest_https.py")"
  QUEST_TLS_DIR="$HAND_PROJECT_ROOT/runtime/ubuntu/quest-tls"
  if [[ -n "$QUEST_ADDRESS" ]]; then
    network_args=(--host 0.0.0.0 --allowed-host "$QUEST_ADDRESS")
  fi
  network_args+=(--tls-cert "$QUEST_TLS_DIR/server.crt" --tls-key "$QUEST_TLS_DIR/server.key")
fi
wuji_args=()
if [[ -n "${WUJI_DEVICE_ADDRESS:-}" ]]; then wuji_args+=(--wuji-address "$WUJI_DEVICE_ADDRESS"); fi
if [[ -n "${WUJI_DEVICE_SERIAL:-}" ]]; then wuji_args+=(--wuji-serial "$WUJI_DEVICE_SERIAL"); fi
exec "$HAND_PYTHON" "$HAND_PROJECT_ROOT/tools/restart_local_service.py" track \
  "$HAND_PYTHON" online_control/online_motion_track/server.py \
  "${network_args[@]}" --port 8766 --log-dir "$HAND_PROJECT_ROOT/logs/control_sessions" --no-open \
  "${wuji_args[@]}" "$@"
