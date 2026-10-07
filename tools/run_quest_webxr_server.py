#!/usr/bin/env python3
"""Run the Quest WebXR25 HTTPS/WSS probe without starting the Qt Form."""

from __future__ import annotations

import argparse
from pathlib import Path
import signal
import sys
import threading
import time


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from tracking.webxr import (  # noqa: E402
    DEFAULT_WEBXR_CA_CERT,
    DEFAULT_WEBXR_CERT,
    DEFAULT_WEBXR_HOST,
    DEFAULT_WEBXR_KEY,
    DEFAULT_WEBXR_PORT,
    QuestWebXRServer,
    WebXRHandFrame,
    WebXRServerStatus,
)


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Serve the Quest capture page and receive native WebXR 25-joint hands "
            "over same-origin TLS/WSS. No image or UDP transport is used."
        )
    )
    parser.add_argument("--host", default=DEFAULT_WEBXR_HOST, help="bind address")
    parser.add_argument("--port", type=int, default=DEFAULT_WEBXR_PORT)
    parser.add_argument("--cert", type=Path, default=DEFAULT_WEBXR_CERT)
    parser.add_argument("--key", type=Path, default=DEFAULT_WEBXR_KEY)
    parser.add_argument(
        "--advertise-host",
        action="append",
        default=[],
        help="Windows LAN IP/DNS to include in the certificate and Quest URL; repeatable",
    )
    parser.add_argument("--jsonl", type=Path, default=None, help="optional raw message log")
    parser.add_argument("--append-jsonl", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    stopped = threading.Event()
    latest: dict[str, WebXRHandFrame] = {}
    frame_count = 0
    count_lock = threading.Lock()

    def on_frame(frame: WebXRHandFrame) -> None:
        nonlocal frame_count
        with count_lock:
            latest[frame.hand_side] = frame
            frame_count += 1

    def on_status(status: WebXRServerStatus) -> None:
        print(
            f"[{status.state}] {status.message} "
            f"connections={status.connections} frames={status.frames_received} "
            f"invalid={status.invalid_messages}",
            flush=True,
        )

    server = QuestWebXRServer(on_frame=on_frame, on_status=on_status)
    try:
        server.start(
            args.host,
            args.port,
            cert_path=args.cert,
            key_path=args.key,
            advertise_hosts=tuple(args.advertise_host),
            jsonl_path=args.jsonl,
            append_jsonl=args.append_jsonl,
        )
    except Exception as exc:
        print(f"WebXR server failed: {exc}", file=sys.stderr, flush=True)
        return 2

    health = server.health_snapshot()
    print("\nQuest 3 打开以下地址（选择与 Quest 同一局域网的 Windows IP）：", flush=True)
    for url in server.quest_urls:
        print(f"  {url}", flush=True)
    print(f"WSS 路由: /hand-skeleton", flush=True)
    print(f"健康检查: https://127.0.0.1:{server.endpoint[1]}/healthz", flush=True)
    print(f"TLS 证书: {Path(args.cert).expanduser().resolve()}", flush=True)
    generated_ca = Path(args.cert).expanduser().resolve().with_name(DEFAULT_WEBXR_CA_CERT.name)
    if generated_ca.is_file():
        print(f"Quest 要信任的本地 CA: {generated_ca}", flush=True)
    print(f"SHA-256: {health['certificate_sha256']}", flush=True)
    print("按 Ctrl+C 停止。\n", flush=True)

    def request_stop(_signum=None, _frame=None) -> None:
        stopped.set()

    signal.signal(signal.SIGINT, request_stop)
    if hasattr(signal, "SIGTERM"):
        signal.signal(signal.SIGTERM, request_stop)

    last_printed_count = -1
    try:
        while not stopped.wait(1.0):
            with count_lock:
                count = frame_count
                snapshot = dict(latest)
            if count == last_printed_count:
                continue
            last_printed_count = count
            hands = ", ".join(
                f"{side}={'tracked' if frame.tracked else frame.event}"
                f"/{frame.valid_joint_count}"
                for side, frame in sorted(snapshot.items())
            ) or "waiting"
            print(f"frames={count} latest: {hands}", flush=True)
    finally:
        server.stop()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
