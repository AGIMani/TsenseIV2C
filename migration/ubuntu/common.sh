#!/usr/bin/env bash
# Shared paths and environment for the native Ubuntu launchers.
HAND_SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
HAND_PROJECT_ROOT="$(cd -- "$HAND_SCRIPT_DIR/../.." && pwd -P)"
HAND_VENV_ROOT="${HAND_VENV_ROOT:-$HAND_PROJECT_ROOT/.venvs}"
case "$HAND_VENV_ROOT" in
  /*) ;;
  *) HAND_VENV_ROOT="$HAND_PROJECT_ROOT/$HAND_VENV_ROOT" ;;
esac
export HAND_PROJECT_ROOT HAND_VENV_ROOT
unset PYTHONPATH PYTHONHOME PYTHONUSERBASE CONDA_PREFIX CONDA_DEFAULT_ENV VIRTUAL_ENV
export PYTHONNOUSERSITE=1 PYTHONDONTWRITEBYTECODE=1 PYTHONUNBUFFERED=1
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 WUJI_TELEMETRY_DISABLE=1
# Old bridge addresses should never select a Windows transport by accident.
unset WUJI_BRIDGE_URL
hand_require_venv() {
  local name="$1"
  HAND_PYTHON="$HAND_VENV_ROOT/$name/bin/python"
  if [[ ! -x "$HAND_PYTHON" ]]; then
    printf 'Missing environment: %s\nRun: bash "%s/install.sh" %s\n' \
      "$HAND_VENV_ROOT/$name" "$HAND_SCRIPT_DIR" "$name" >&2
    exit 1
  fi
}
cd -- "$HAND_PROJECT_ROOT"
