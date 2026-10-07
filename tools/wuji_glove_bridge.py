"""Read-only Wuji skeleton HTTP bridge. Run on Linux with wuji-sdk installed.

No robot commands are exposed. /latest?side=left|right returns only actual SDK
observations. The HTTP server stays responsive during discovery/reconnection.
"""
from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import signal
import threading
from urllib.parse import parse_qs, urlsplit

# Keep the standalone HTTP adapter thin; acquisition is identical to local SDK mode.
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from online_control.online_motion_track.wuji_sdk_source import SCHEMA, SkeletonStore, acquire


def make_server(store, host, port):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"
        disable_nagle_algorithm = True
        def do_GET(self):
            parsed = urlsplit(self.path)
            if parsed.path not in ("/latest", "/health"):
                self.send_error(404)
                return
            side = parse_qs(parsed.query).get("side", ["right"])[0]
            if side not in ("left", "right"):
                self.send_error(400)
                return
            payload = json.dumps(store.latest(side), allow_nan=False).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(payload)))
            self.end_headers()
            try:
                self.wfile.write(payload)
            except (BrokenPipeError, ConnectionResetError):
                self.close_connection = True

        def setup(self):
            super().setup()
            self.connection.settimeout(2)

        def log_message(self, *args):
            pass
    return ThreadingHTTPServer((host, port), Handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8787)
    parser.add_argument("--address", help="Explicit glove IP:port; otherwise discover gloves")
    parser.add_argument("--serial", help="Select one discovered glove by serial number")
    args = parser.parse_args()
    store, stopped = SkeletonStore(), threading.Event()
    server = make_server(store, args.host, args.port)
    worker = threading.Thread(target=acquire, args=(store, stopped, args.address, args.serial), daemon=True)
    worker.start()
    def stop(*_):
        stopped.set()
        threading.Thread(target=server.shutdown, daemon=True).start()
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    print(f"Wuji skeleton bridge http://{args.host}:{server.server_port}", flush=True)
    try:
        server.serve_forever()
    finally:
        stopped.set()
        worker.join(timeout=5)
        server.server_close()


if __name__ == "__main__":
    main()
