#!/usr/bin/env bash
# Native systemd deployment. Run from the source checkout or extracted bundle.
set -Eeuo pipefail
DOMAIN=""; PORT=8766; MAX_SESSIONS=4; CADDY=1
while (($#)); do
  case "$1" in
    --domain) DOMAIN="${2:?Missing domain}"; shift 2 ;;
    --port) PORT="${2:?Missing port}"; shift 2 ;;
    --max-sessions) MAX_SESSIONS="${2:?Missing session count}"; shift 2 ;;
    --no-caddy) CADDY=0; shift ;;
    -h|--help) echo 'sudo bash deploy.sh --domain hand.example.com [--port 8766] [--max-sessions 4] [--no-caddy]'; exit 0 ;;
    *) echo "Unknown argument: $1" >&2; exit 2 ;;
  esac
done
[[ $EUID == 0 ]] || { echo 'Run with sudo.' >&2; exit 2; }
[[ $DOMAIN =~ ^([a-zA-Z0-9]([a-zA-Z0-9-]{0,61}[a-zA-Z0-9])?\.)+[a-zA-Z]{2,63}$ && ${#DOMAIN} -le 253 ]] || { echo 'Supply a DNS domain with --domain (no scheme or path).' >&2; exit 2; }
DOMAIN="${DOMAIN,,}"
[[ $PORT =~ ^[1-9][0-9]{3,4}$ ]] && ((PORT >= 1024 && PORT <= 65535)) || { echo 'Invalid port.' >&2; exit 2; }
[[ $MAX_SESSIONS =~ ^[1-9][0-9]?$ ]] && ((MAX_SESSIONS <= 32)) || { echo 'max-sessions must be 1..32.' >&2; exit 2; }
command -v apt-get >/dev/null && command -v systemctl >/dev/null || { echo 'Requires Debian/Ubuntu with systemd.' >&2; exit 2; }
SOURCE=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
APP=/opt/tsensei-hand-web
SERVICE=tsensei-hand-web
DATA=/var/lib/tsensei-hand-web
UNIT=/etc/systemd/system/tsensei-hand-web.service
export DEBIAN_FRONTEND=noninteractive
apt-get update
apt-get install -y python3 python3-venv ca-certificates curl iproute2
python3 -c 'import sys; assert (3,11) <= sys.version_info[:2] < (3,14), "Use Python 3.11-3.13 (Ubuntu 24.04 / Debian 12,13)"'
if ((CADDY)); then
  if ss -H -ltnp '( sport = :80 or sport = :443 )' | grep -v '"caddy"' | grep -q .; then
    echo '80/443 are occupied by another service. Use --no-caddy and configure your existing HTTPS proxy (DEPLOY.md).' >&2
    exit 2
  fi
fi
if ss -H -ltn "sport = :$PORT" | grep -q . && ! systemctl is-active --quiet "$SERVICE"; then
  echo "Port $PORT is occupied; choose --port." >&2; exit 2
fi
install -d -m 755 "$APP/releases" "$APP/backups"
if [[ -e $APP/current && ! -L $APP/current ]]; then echo "$APP/current must be a symlink, refusing to overwrite." >&2; exit 2; fi
exec 9>"$APP/deploy.lock"
flock -n 9 || { echo 'Another deployment is running.' >&2; exit 2; }
id "$SERVICE" >/dev/null 2>&1 || useradd --system --home-dir "$DATA" --shell /usr/sbin/nologin "$SERVICE"
install -d -o "$SERVICE" -g "$SERVICE" -m 750 "$DATA" "$DATA/logs"
STAMP=$(date -u +%Y%m%dT%H%M%S)-$$
RELEASE="$APP/releases/$STAMP"
BACKUP="$APP/backups/$STAMP"
mkdir "$BACKUP"
PREVIOUS=$(readlink -f "$APP/current" 2>/dev/null || true)
HAD_UNIT=0; WAS_ACTIVE=0; SWITCHED=0; CADDY_CHANGED=0; HAD_CADDY=0; HAD_SITE=0
[[ ! -f $UNIT ]] || { cp -a "$UNIT" "$BACKUP/service"; HAD_UNIT=1; }
systemctl is-active --quiet "$SERVICE" && WAS_ACTIVE=1
rollback() {
  code=$?
  trap - ERR
  set +e
  if ((SWITCHED)); then
    if [[ -n $PREVIOUS && -d $PREVIOUS ]]; then
      ln -sfn "$PREVIOUS" "$APP/current"
    else
      systemctl disable --now "$SERVICE"
      [[ ! -L $APP/current ]] || unlink "$APP/current"
    fi
    if ((HAD_UNIT)); then cp -a "$BACKUP/service" "$UNIT"; else rm -f -- "$UNIT"; fi
    systemctl daemon-reload
    if ((WAS_ACTIVE)); then systemctl restart "$SERVICE"; else systemctl stop "$SERVICE"; fi
  fi
  if ((CADDY_CHANGED)); then
    if ((HAD_CADDY)); then cp -a "$BACKUP/Caddyfile" /etc/caddy/Caddyfile; fi
    if ((HAD_SITE)); then cp -a "$BACKUP/site.caddy" /etc/caddy/tsensei.d/hand.caddy; else rm -f -- /etc/caddy/tsensei.d/hand.caddy; fi
    systemctl reload caddy
  fi
  echo "Deployment failed (status $code). Previous application/config restored where present. Diagnostics: $BACKUP and journalctl -u $SERVICE" >&2
  exit "$code"
}
trap rollback ERR
python3 "$SOURCE/package.py" --stage "$RELEASE"
python3 -m venv "$RELEASE/.venv"
"$RELEASE/.venv/bin/python" -m pip install --only-binary=:all: -r "$RELEASE/requirements-server.txt"
"$RELEASE/.venv/bin/python" -m pip check
OPENBLAS_NUM_THREADS=1 "$RELEASE/.venv/bin/python" "$RELEASE/server.py" --check
if ((CADDY)) && ! command -v caddy >/dev/null; then
  apt-get install -y debian-keyring debian-archive-keyring apt-transport-https gnupg
  curl -fsSL https://dl.cloudsmith.io/public/caddy/stable/gpg.key | gpg --dearmor --yes -o /usr/share/keyrings/caddy-stable-archive-keyring.gpg
  curl -fsSL https://dl.cloudsmith.io/public/caddy/stable/debian.deb.txt -o /etc/apt/sources.list.d/caddy-stable.list
  chmod o+r /usr/share/keyrings/caddy-stable-archive-keyring.gpg /etc/apt/sources.list.d/caddy-stable.list
  apt-get update
  apt-get install -y caddy
fi
cat > "$BACKUP/new.service" <<EOF
[Unit]
Description=TsenseI V3.2 hand retarget web service
After=network.target

[Service]
Type=simple
User=$SERVICE
Group=$SERVICE
WorkingDirectory=$DATA
Environment=PYTHONUNBUFFERED=1
Environment=PYTHONDONTWRITEBYTECODE=1
Environment=OPENBLAS_NUM_THREADS=1
Environment=OMP_NUM_THREADS=1
ExecStart=$APP/current/.venv/bin/python $APP/current/server.py --public --allowed-host $DOMAIN --port $PORT --max-sessions $MAX_SESSIONS --no-open --log-dir $DATA/logs
Restart=on-failure
RestartSec=3
TimeoutStopSec=20
NoNewPrivileges=true
PrivateTmp=true
ProtectSystem=strict
ProtectHome=true
ReadWritePaths=$DATA
UMask=0077

[Install]
WantedBy=multi-user.target
EOF
SWITCHED=1
ln -sfn "$RELEASE" "$APP/current"
install -m 644 "$BACKUP/new.service" "$UNIT"
systemctl daemon-reload
systemctl enable "$SERVICE"
systemctl restart "$SERVICE"
READY=0
for ((i=0; i<30; i++)); do
  if systemctl is-active --quiet "$SERVICE" && curl -fsS --max-time 2 "http://127.0.0.1:$PORT/api/health" > "$BACKUP/health.json" &&
    python3 -c 'import json,sys; d=json.load(open(sys.argv[1])); assert d.get("ok") and d.get("public") and d.get("recording_allowed") is False' "$BACKUP/health.json"; then READY=1; break; fi
  sleep 1
done
if ((!READY)); then echo 'Application readiness check failed.' >&2; false; fi
if ((CADDY)); then
  install -d -m 755 /etc/caddy/tsensei.d
  if [[ -f /etc/caddy/Caddyfile ]]; then cp -a /etc/caddy/Caddyfile "$BACKUP/Caddyfile"; HAD_CADDY=1; fi
  if [[ -f /etc/caddy/tsensei.d/hand.caddy ]]; then cp -a /etc/caddy/tsensei.d/hand.caddy "$BACKUP/site.caddy"; HAD_SITE=1; fi
  CADDY_CHANGED=1
  cat > /etc/caddy/tsensei.d/hand.caddy <<EOF
$DOMAIN {
    encode zstd gzip
    request_body {
        max_size 128KB
    }
    reverse_proxy 127.0.0.1:$PORT {
        header_up X-Real-IP {remote_host}
    }
}
EOF
  touch /etc/caddy/Caddyfile
  if ! grep -Fqx 'import /etc/caddy/tsensei.d/*.caddy' /etc/caddy/Caddyfile; then
    printf '\nimport /etc/caddy/tsensei.d/*.caddy\n' >> /etc/caddy/Caddyfile
  fi
  chmod 644 /etc/caddy/Caddyfile /etc/caddy/tsensei.d/hand.caddy
  caddy validate --config /etc/caddy/Caddyfile --adapter caddyfile
  systemctl enable --now caddy
  systemctl reload caddy
fi
trap - ERR
echo "Application installed: $RELEASE"
if ((CADDY)); then
  echo "Open https://$DOMAIN after DNS A/AAAA records resolve here and TCP 80/443 are reachable. Caddy obtains/renews TLS automatically."
  echo 'If TLS is still pending: journalctl -u caddy -n 80 --no-pager'
else
  echo "Configure your HTTPS proxy for $DOMAIN -> 127.0.0.1:$PORT (DEPLOY.md)."
fi
echo "Status/logs: systemctl status $SERVICE; journalctl -u $SERVICE -f"
