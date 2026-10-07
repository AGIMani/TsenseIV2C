#!/usr/bin/env bash
# Ubuntu 22.04/24.04, Debian 12/13. Run: sudo bash deploy.sh
set -Eeuo pipefail
umask 027
SOURCE_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
INSTALL_ROOT=/opt/online-motion-play
CONFIG_DIR=/etc/online-motion-play
DATA_DIR=/var/lib/online-motion-play
SERVICE=online-motion-play
PORT=""
EXTRA_HOSTS=()
ALLOW_CIDR=""

usage() {
  cat <<'HELP'
Usage: sudo bash deploy.sh [--port 8765] [--allowed-host hand.example.lan]
                          [--allow-cidr 192.168.1.0/24]

Installs isolated dependencies, creates a service account with password-free web access,
checks the application, and enables/restarts a systemd service. Re-running
upgrades code while preserving configuration and saved motions and removing old web passwords.
--allow-cidr adds a scoped UFW rule only if UFW is already active.
HELP
}
while (($#)); do
  case "$1" in
    --port) PORT="${2:?missing port}"; shift 2 ;;
    --allowed-host) EXTRA_HOSTS+=(--allowed-host "${2:?missing hostname}"); shift 2 ;;
    --allow-cidr) ALLOW_CIDR="${2:?missing CIDR}"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 2 ;;
  esac
done
if [[ $EUID -ne 0 ]]; then
  echo "Run this script with sudo: sudo bash $SOURCE_DIR/deploy.sh" >&2; exit 1
fi
if ! command -v apt-get >/dev/null || ! command -v systemctl >/dev/null || [[ ! -d /run/systemd/system ]]; then
  echo "This installer requires Ubuntu/Debian with systemd (not a bare container)." >&2; exit 1
fi
if [[ -n "$PORT" && (! "$PORT" =~ ^[0-9]+$ || ${#PORT} -gt 5) ]]; then
  echo "Invalid port" >&2; exit 2
fi
if [[ -n "$PORT" ]] && ((10#$PORT < 1024 || 10#$PORT > 65535)); then
  echo "Port must be 1024-65535 for the unprivileged service" >&2; exit 2
fi

echo '[1/7] Installing system dependencies'
apt-get update
GLIB_PACKAGE=libglib2.0-0
if apt-cache show libglib2.0-0t64 >/dev/null 2>&1; then GLIB_PACKAGE=libglib2.0-0t64; fi
DEBIAN_FRONTEND=noninteractive apt-get install -y python3 python3-venv python3-dev \
  build-essential cmake ninja-build ca-certificates iproute2 \
  "$GLIB_PACKAGE"
python3 -c 'import sys; assert (3,10) <= sys.version_info[:2] < (3,14), "Python 3.10-3.13 required"'
if [[ -n "$ALLOW_CIDR" ]]; then
  python3 -c 'import ipaddress,sys; ipaddress.IPv4Network(sys.argv[1], strict=False)' "$ALLOW_CIDR"
fi

echo '[2/7] Preparing service account and persistent data'
if ! id "$SERVICE" >/dev/null 2>&1; then
  useradd --system --user-group --home-dir "$DATA_DIR" --shell /usr/sbin/nologin "$SERVICE"
fi
install -d -m 755 "$INSTALL_ROOT" "$INSTALL_ROOT/releases"
install -d -m 750 -o root -g "$SERVICE" "$CONFIG_DIR"
install -d -m 750 -o "$SERVICE" -g "$SERVICE" "$DATA_DIR"
CONFIG_BACKUP=""
if [[ -f "$CONFIG_DIR/config.json" ]]; then
  CONFIG_BACKUP="$(mktemp "$CONFIG_DIR/config.previous.XXXXXX")"
  cp -- "$CONFIG_DIR/config.json" "$CONFIG_BACKUP"
fi
CONFIG_ARGS=(--config "$CONFIG_DIR/config.json" --data-dir "$DATA_DIR")
if [[ -n "$PORT" ]]; then CONFIG_ARGS+=(--port "$PORT"); fi
python3 "$SOURCE_DIR/deploy_config.py" "${CONFIG_ARGS[@]}" "${EXTRA_HOSTS[@]}"
chown root:"$SERVICE" "$CONFIG_DIR/config.json"
chmod 640 "$CONFIG_DIR/config.json"
PORT="$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1]))["port"])' "$CONFIG_DIR/config.json")"
if ((PORT < 1024 || PORT > 65535)); then
  echo "Configured port must be 1024-65535; rerun with --port 8765" >&2; exit 2
fi

echo '[3/7] Staging a new release'
RELEASE="$INSTALL_ROOT/releases/$(date +%Y%m%d-%H%M%S)-$$"
install -d -m 755 "$RELEASE"
python3 "$SOURCE_DIR/package.py" --stage "$RELEASE"
# Root-owned code is readable but not writable by the service account.
chmod -R a+rX "$RELEASE"

echo '[4/7] Creating a Python environment (Ruckig may compile on first install)'
python3 -m venv "$RELEASE/.venv"
"$RELEASE/.venv/bin/python" -m pip install --upgrade pip
export CMAKE_BUILD_PARALLEL_LEVEL=2
export CMAKE_ARGS="-DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF -DBUILD_CLOUD_CLIENT=OFF"
"$RELEASE/.venv/bin/python" -m pip install --only-binary=numpy,scipy,PyYAML \
  --build-constraint "$RELEASE/build-constraints.txt" -r "$RELEASE/requirements-server.txt"
"$RELEASE/.venv/bin/python" -m pip check
chmod -R a+rX "$RELEASE/.venv"

echo '[5/7] Checking dependencies and model resources as the service user'
runuser -u "$SERVICE" -- "$RELEASE/.venv/bin/python" "$RELEASE/server.py" \
  --config "$CONFIG_DIR/config.json" --check --no-browser

echo '[6/7] Installing the systemd service'
PREVIOUS="$(readlink -f "$INSTALL_ROOT/current" 2>/dev/null || true)"
cat > "/etc/systemd/system/$SERVICE.service" <<UNIT
[Unit]
Description=Form2 Online Motion Play
After=network.target
StartLimitIntervalSec=60
StartLimitBurst=5

[Service]
Type=simple
User=$SERVICE
Group=$SERVICE
WorkingDirectory=$DATA_DIR
Environment=PYTHONUNBUFFERED=1
Environment=OPENBLAS_NUM_THREADS=1
Environment=XDG_RUNTIME_DIR=/run/$SERVICE
RuntimeDirectory=$SERVICE
RuntimeDirectoryMode=0700
ExecStart=$INSTALL_ROOT/current/.venv/bin/python $INSTALL_ROOT/current/server.py --config $CONFIG_DIR/config.json --no-browser
Restart=on-failure
RestartSec=3
TimeoutStopSec=15
NoNewPrivileges=true
PrivateTmp=true
PrivateDevices=true
ProtectSystem=strict
ProtectHome=true
ReadWritePaths=$DATA_DIR
UMask=0077

[Install]
WantedBy=multi-user.target
UNIT
ln -sfn "$RELEASE" "$INSTALL_ROOT/current"
systemctl daemon-reload
systemctl enable "$SERVICE.service"
systemctl reset-failed "$SERVICE.service" || true
echo '[7/7] Checking readiness'
if ! systemctl restart "$SERVICE.service" || ! python3 "$RELEASE/deploy_config.py" --config "$CONFIG_DIR/config.json" --probe; then
  journalctl -u "$SERVICE" -n 50 --no-pager >&2
  if [[ -n "$PREVIOUS" && -d "$PREVIOUS" && "$PREVIOUS" != "$RELEASE" ]]; then
    if [[ -n "$CONFIG_BACKUP" ]]; then
      cp -- "$CONFIG_BACKUP" "$CONFIG_DIR/config.json"
      chown root:"$SERVICE" "$CONFIG_DIR/config.json"
      chmod 640 "$CONFIG_DIR/config.json"
    fi
    ln -sfn "$PREVIOUS" "$INSTALL_ROOT/current"
    systemctl restart "$SERVICE.service"
    echo "New release failed; restored $PREVIOUS" >&2
  else
    systemctl stop "$SERVICE.service"
  fi
  exit 1
fi
if command -v ufw >/dev/null && LC_ALL=C ufw status | grep -q '^Status: active'; then
  if [[ -n "$ALLOW_CIDR" ]]; then
    ufw allow from "$ALLOW_CIDR" to any port "$PORT" proto tcp comment 'Form2 intranet access'
  else
    echo "UFW is active. Allow your LAN subnet to TCP $PORT if access is blocked."
  fi
fi
echo
echo 'Deployment complete. Open a reachable LAN address:'
python3 "$RELEASE/deploy_config.py" --config "$CONFIG_DIR/config.json" --show-urls
echo 'Web access is password-free.'
echo "Status: sudo systemctl status $SERVICE"
echo "Logs:   sudo journalctl -u $SERVICE -f"
echo "Data:   $DATA_DIR/motions.json"
echo 'Connect the hand USB/serial device to this server to control real hardware.'
