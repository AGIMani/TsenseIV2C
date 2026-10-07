#!/usr/bin/env bash
# Recreate Linux Python environments; never installs system packages or starts hardware.
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "$(readlink -f -- "${BASH_SOURCE[0]}")")" && pwd -P)"
usage() {
  cat <<'HELP'
Usage: bash migration/ubuntu/install.sh [track|play|all]
Default: all. Run as your regular Ubuntu user (not sudo).
Environments: PROJECT/.venvs/{track,play}; override with HAND_VENV_ROOT.
Internet is required to restore pinned Python packages and web assets.

Install system prerequisites yourself if missing (the script will not use sudo):
  sudo apt update
  sudo apt install python3.12 python3.12-venv python3.12-dev build-essential \
    cmake ninja-build pkg-config ca-certificates nodejs npm \
    libffi-dev libgl1 libegl1 libopengl0 libglib2.0-0t64 \
    libusb-1.0-0 libudev-dev usbutils
HELP
}
mode="${1:-all}"
if [[ "$mode" == --help || "$mode" == -h ]]; then usage; exit 0; fi
if [[ $# -gt 1 || ! "$mode" =~ ^(track|play|all)$ ]]; then usage >&2; exit 2; fi
if [[ $EUID == 0 ]]; then echo 'Run as your regular Ubuntu user, without sudo.' >&2; exit 1; fi
source "$SCRIPT_DIR/common.sh"
source /etc/os-release
if [[ "$ID" != ubuntu || "$VERSION_ID" != 24.04 || "$(uname -m)" != x86_64 ]]; then
  echo 'These frozen requirements target Ubuntu 24.04, CPython 3.12, x86_64.' >&2
  exit 1
fi
for command_name in python3.12 cmake ninja c++; do
  command -v "$command_name" >/dev/null || { usage >&2; echo "Missing command: $command_name" >&2; exit 1; }
done
python3.12 -c 'import sys, venv; assert sys.version_info[:2] == (3, 12)'
if [[ "$mode" == track || "$mode" == all ]]; then
  for command_name in node npm; do
    command -v "$command_name" >/dev/null || { usage >&2; echo "Missing command: $command_name" >&2; exit 1; }
  done
  node -e 'if (Number(process.versions.node.split(".")[0]) < 18) { console.error("Node.js >=18 required"); process.exit(1); }'
  npm_major="$(npm --version)"; npm_major="${npm_major%%.*}"
  if [[ ! "$npm_major" =~ ^[0-9]+$ || "$npm_major" -lt 9 ]]; then echo 'npm >=9 required for the version-3 lockfile.' >&2; exit 1; fi
fi
# Never allow inherited pip options to write outside the selected virtualenv.
unset PIP_TARGET PIP_PREFIX PIP_USER
export PIP_REQUIRE_VIRTUALENV=true PIP_CONFIG_FILE=/dev/null
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-2}"
export CMAKE_ARGS='-DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_CLOUD_CLIENT=OFF'
environments=("$mode")
if [[ "$mode" == all ]]; then environments=(track play); fi
mkdir -p -- "$HAND_VENV_ROOT"
for environment in "${environments[@]}"; do
  venv_path="$HAND_VENV_ROOT/$environment"
  if [[ ! -f "$venv_path/pyvenv.cfg" ]]; then
    if [[ -e "$venv_path" ]]; then echo "Refusing to overwrite an unrecognized directory: $venv_path" >&2; exit 1; fi
    python3.12 -m venv "$venv_path"
  fi
  python_path="$venv_path/bin/python"
  "$python_path" -c 'import sys; assert sys.prefix != sys.base_prefix; assert sys.version_info[:2] == (3, 12)'
  "$python_path" -m pip install --upgrade 'pip>=25.3'
  if [[ "$environment" == track ]]; then
    "$python_path" -m pip install --only-binary=:all: -r "$SCRIPT_DIR/requirements-track.txt"
  else
    # Ruckig 0.17.3 is built from source with its compatible build backend.
    # Binary-only scientific packages avoid incompatible local ABI builds.
    "$python_path" -m pip install --only-binary=numpy,scipy,PyYAML \
      --build-constraint "$SCRIPT_DIR/build-constraints.txt" \
      -r "$SCRIPT_DIR/requirements-$environment.txt"
  fi
  "$python_path" -m pip check
  printf 'Ready: %s\n' "$venv_path"
done
if [[ "$mode" == track || "$mode" == all ]]; then
  # Runtime dependencies have no Node engine restriction. Node 18/npm 9 from
  # Ubuntu 24.04 are sufficient; Playwright devDependencies require Node >=20.
  npm ci --omit=dev --ignore-scripts --prefix "$HAND_PROJECT_ROOT/online_control/online_motion_track"
fi
mkdir -p -- "$HAND_PROJECT_ROOT/logs/control_sessions" "$HAND_PROJECT_ROOT/runtime/ubuntu/play"
printf '\nInstallation finished. No service or hardware was started.\n'
printf 'See %s/README-install.txt for launch commands and offline checks.\n' "$SCRIPT_DIR"
