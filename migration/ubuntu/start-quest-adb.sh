#!/usr/bin/env bash
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")" && pwd -P)"
source "$SCRIPT_DIR/common.sh"
hand_require_venv track
quest_adb_port="${HAND_QUEST_ADB_PORT:-8766}"
if [[ ! "$quest_adb_port" =~ ^[0-9]{1,5}$ ]] || (( 10#$quest_adb_port < 1 || 10#$quest_adb_port > 65535 )); then
  echo 'HAND_QUEST_ADB_PORT 必须为 1..65535' >&2
  exit 2
fi
quest_adb_port=$((10#$quest_adb_port))
prepare_adb=1
for arg in "$@"; do
  case "$arg" in
    --check|--help|-h) prepare_adb=0 ;;
    --host|--host=*|--port|--port=*|--public|--tls*|--allowed-host*)
      echo 'USB ADB 启动器固定使用回环 HTTP；端口请用 HAND_QUEST_ADB_PORT 设置。' >&2
      exit 2 ;;
  esac
done
if [[ "$prepare_adb" == 1 ]]; then
  "$HAND_PYTHON" "$HAND_PROJECT_ROOT/tools/prepare_quest_adb.py" --port "$quest_adb_port"
fi
exec bash "$SCRIPT_DIR/start-track.sh" --host 127.0.0.1 --port "$quest_adb_port" "$@"
