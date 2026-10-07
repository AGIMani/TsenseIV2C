"""Deployment configuration and readiness probe (stdlib only)."""
import argparse
import json
import os
from pathlib import Path
import socket
import ssl
import subprocess
import time
import urllib.request



def detected_hosts():
    names = {"localhost", "127.0.0.1", socket.gethostname(), socket.getfqdn()}
    try:
        # Include every IPv4 interface, not just the default route.
        interfaces = json.loads(subprocess.check_output(["ip", "-j", "-4", "addr"], text=True))
        names.update(info["local"] for interface in interfaces for info in interface.get("addr_info", []))
    except (OSError, ValueError, subprocess.SubprocessError):
        names.update(info[4][0] for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET))
    return names


def configure(path, data_dir, port=None, extra_hosts=(), reset_password=False):
    path = Path(path)
    config = json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}
    config.setdefault("host", "0.0.0.0")
    config.setdefault("port", 8765)
    if port is not None:
        if not 1 <= port <= 65535:
            raise ValueError("port must be between 1 and 65535")
        config["port"] = port
    config["data_dir"] = str(Path(data_dir).resolve())
    names = set(config.get("allowed_hosts", [])) | detected_hosts() | set(extra_hosts)
    if any(not host or any(ch in host for ch in "/:@* \t\n\r") for host in names):
        raise ValueError("allowed_hosts must contain IPv4 addresses or DNS names without port/scheme")
    config["allowed_hosts"] = sorted(names)
    # Migrate existing deployments to password-free access.
    config.pop("password_hash", None)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(".tmp")
    # Keep deployment configuration private while writing it.
    fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(fd, "w", encoding="utf-8") as output:
        json.dump(config, output, indent=2)
        output.write("\n")
    temporary.replace(path)
    (path.parent / "access-password.txt").unlink(missing_ok=True)
    return config


def probe(config_path, timeout=30):
    config = json.loads(Path(config_path).read_text(encoding="utf-8"))
    secure = bool(config.get("tls_cert"))
    context = None
    if secure:
        # Pin the configured certificate for a loopback-only readiness check;
        # the browser still performs normal hostname/certificate validation.
        context = ssl.create_default_context(cafile=config["tls_cert"])
        context.check_hostname = False
    url = f"{'https' if secure else 'http'}://127.0.0.1:{config['port']}/api/health"
    deadline = time.monotonic() + timeout
    error = None
    while time.monotonic() < deadline:
        try:
            with urllib.request.urlopen(url, timeout=3, context=context) as response:
                payload = json.load(response)
            if payload.get("status") == "ok" and payload.get("service") == "online-motion-play":
                return
        except (OSError, ValueError) as exc:
            error = exc
        time.sleep(.5)
    raise RuntimeError(f"Service did not become ready: {error}")


def show_urls(config):
    if config.get("public_origin"):
        print("  " + config["public_origin"])
    for host in config["allowed_hosts"]:
        if host not in ("localhost", "127.0.0.1"):
            print(f"  {'https' if config.get('tls_cert') else 'http'}://{host}:{config['port']}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--data-dir", default="/var/lib/online-motion-play")
    parser.add_argument("--port", type=int)
    parser.add_argument("--allowed-host", action="append", default=[])
    parser.add_argument("--reset-password", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--probe", action="store_true")
    parser.add_argument("--show-urls", action="store_true")
    args = parser.parse_args()
    if args.probe:
        probe(args.config)
        print("Health check passed")
    elif args.show_urls:
        show_urls(json.loads(args.config.read_text(encoding="utf-8")))
    else:
        config = configure(args.config, args.data_dir, args.port, args.allowed_host, args.reset_password)
        print(f"Configuration ready: {args.config}")
        show_urls(config)


if __name__ == "__main__":
    main()
