#!/usr/bin/env python3
"""Publish Quest 3 hand joints from AGIMani/Teleop to the Form application.

Run this process in the same Linux environment and shell as CloudXR and
IsaacTeleop.  The Qt Form application may run on the same host or on another
machine reachable through LAN/VPN UDP.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import socket
import sys
import time
from typing import Any, Callable, Mapping


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = PROJECT_ROOT / "src"
if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from quest3_teleop_bridge import (  # noqa: E402
    TeleopHandPacketEncoder,
    encode_udp_packet,
)


DEFAULT_TELEOP_ROOT = PROJECT_ROOT / "external" / "Teleop"
DEFAULT_FORM_HOST = os.environ.get("CLOUDXR_LANDMARK_HOST", "127.0.0.1")
DEFAULT_FORM_PORT = int(os.environ.get("CLOUDXR_LANDMARK_PORT", "28770"))


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Read Quest 3 optical hand tracking through AGIMani/Teleop + "
            "IsaacTeleop and publish OpenXR-26 JSON/UDP frames to the Form app."
        )
    )
    parser.add_argument(
        "--teleop-root",
        default=os.environ.get("TELEOP_ROOT", str(DEFAULT_TELEOP_ROOT)),
        help="AGIMani/Teleop checkout (the directory containing src/teleop_stack).",
    )
    parser.add_argument(
        "--isaac-teleop-root",
        default=os.environ.get("ISAAC_TELEOP_ROOT"),
        help="Optional NVIDIA IsaacTeleop checkout; Teleop also discovers its normal defaults.",
    )
    parser.add_argument(
        "--host",
        default=DEFAULT_FORM_HOST,
        help="IPv4 address of the Form receiver (default: %(default)s).",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=DEFAULT_FORM_PORT,
        help="Form UDP receiver port (default: %(default)s).",
    )
    parser.add_argument(
        "--source-host",
        default=os.environ.get("QUEST3_BRIDGE_SOURCE_HOST"),
        help=(
            "Optional local IPv4 address for outgoing Form UDP packets. "
            "Useful on multi-address CloudXR hosts when the Form firewall "
            "allows only one server address."
        ),
    )
    parser.add_argument("--hand", choices=("left", "right"), default="right")
    parser.add_argument(
        "--loop-hz",
        type=float,
        default=60.0,
        help="OpenXR poll and UDP publication rate.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Run time in seconds; <=0 runs until Ctrl+C.",
    )
    parser.add_argument(
        "--loss-timeout",
        type=float,
        default=0.35,
        help="Grace period before publishing tracked=false.",
    )
    parser.add_argument(
        "--startup-timeout",
        type=float,
        default=30.0,
        help="Wait this long for an active Quest OpenXR session.",
    )
    parser.add_argument(
        "--startup-retry-interval",
        type=float,
        default=1.0,
        help="Delay between OpenXR startup attempts.",
    )
    parser.add_argument(
        "--print-every",
        type=int,
        default=60,
        help="Print statistics every N transmitted tracked frames.",
    )
    parser.add_argument(
        "--disable-synthetic-hands-plugin",
        action="store_true",
        help="Disable Teleop's controller-to-synthetic-hands plugin.",
    )
    parser.add_argument(
        "--skip-runtime-check",
        action="store_true",
        help="Skip the NV_CXR_RUNTIME_DIR/ipc_cloudxr preflight check.",
    )
    parser.add_argument(
        "--check-only",
        action="store_true",
        help="Validate paths/imports and exit without opening OpenXR or UDP.",
    )
    return parser


def _resolve_teleop_root(value: str | os.PathLike[str]) -> Path:
    root = Path(value).expanduser().resolve()
    package_dir = root / "src" / "teleop_stack"
    if not package_dir.is_dir():
        raise FileNotFoundError(
            f"Teleop package not found at {package_dir}; pass --teleop-root correctly."
        )
    source_dir = root / "src"
    if str(source_dir) not in sys.path:
        sys.path.insert(0, str(source_dir))
    return root


def _load_teleop_api() -> dict[str, Any]:
    try:
        from isaacteleop.retargeting_engine.deviceio_source_nodes import HandsSource
        from isaacteleop.retargeting_engine.interface import OutputCombiner
        from isaacteleop.teleop_session_manager import TeleopSession, TeleopSessionConfig
        from teleop_stack.devices import QuestInputConfig, build_quest_input_bundle
        from teleop_stack.retargeting.converters import optional_hand_debug_snapshot
    except ModuleNotFoundError as exc:
        raise RuntimeError(
            "Teleop/IsaacTeleop import failed. Activate Teleop's isaac_teleop "
            "environment and make sure IsaacTeleop is installed/discoverable. "
            f"Missing module: {exc.name}"
        ) from exc

    return {
        "HandsSource": HandsSource,
        "OutputCombiner": OutputCombiner,
        "QuestInputConfig": QuestInputConfig,
        "TeleopSession": TeleopSession,
        "TeleopSessionConfig": TeleopSessionConfig,
        "build_quest_input_bundle": build_quest_input_bundle,
        "optional_hand_debug_snapshot": optional_hand_debug_snapshot,
    }


def _validate_cloudxr_runtime() -> Path:
    runtime_dir_raw = os.environ.get("NV_CXR_RUNTIME_DIR")
    if not runtime_dir_raw:
        raise RuntimeError(
            "NV_CXR_RUNTIME_DIR is not set. Start CloudXR with Teleop's "
            "./scripts/start_cloudxr.sh, then source ~/.cloudxr/run/cloudxr.env "
            "in this terminal."
        )
    socket_path = Path(runtime_dir_raw).expanduser() / "ipc_cloudxr"
    if not socket_path.exists():
        raise RuntimeError(
            f"CloudXR OpenXR IPC socket does not exist: {socket_path}. "
            "Start or restart CloudXR first."
        )

    if hasattr(socket, "AF_UNIX"):
        probe = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        probe.settimeout(0.5)
        try:
            probe.connect(str(socket_path))
        except OSError as exc:
            raise RuntimeError(
                f"CloudXR OpenXR IPC socket is not accepting connections: {socket_path}"
            ) from exc
        finally:
            probe.close()
    return socket_path


def _build_raw_hand_session(
    api: Mapping[str, Any],
    *,
    hand_side: str,
    isaac_teleop_root: str | None,
    enable_synthetic_hands_plugin: bool,
) -> tuple[Any, Callable[[dict[str, Any]], dict[str, object] | None]]:
    input_bundle = api["build_quest_input_bundle"](
        api["QuestInputConfig"](
            enable_hands=True,
            enable_controllers=False,
            enable_head=False,
            enable_synthetic_hands_plugin=enable_synthetic_hands_plugin,
            isaac_teleop_root=isaac_teleop_root,
        )
    )
    hands = input_bundle.sources.get("hands")
    if hands is None:
        raise RuntimeError("IsaacTeleop did not create a hand tracking source.")

    hand_device = (
        api["HandsSource"].LEFT
        if hand_side == "left"
        else api["HandsSource"].RIGHT
    )
    pipeline = api["OutputCombiner"]({"raw_hand": hands.output(hand_device)})
    config = api["TeleopSessionConfig"](
        app_name="Quest3FormLandmarkBridge",
        trackers=[],
        pipeline=pipeline,
        plugins=input_bundle.plugins,
    )
    session = api["TeleopSession"](config)
    return session, api["optional_hand_debug_snapshot"]


class UdpHandPublisher:
    def __init__(
        self,
        *,
        host: str,
        port: int,
        source_host: str | None,
        encoder: TeleopHandPacketEncoder,
        print_every: int,
    ) -> None:
        if not (1 <= int(port) <= 65_535):
            raise ValueError("UDP port must be between 1 and 65535")
        self.host = str(host).strip()
        self.port = int(port)
        self.source_host = str(source_host).strip() if source_host else ""
        self.encoder = encoder
        self.print_every = max(1, int(print_every))
        self._socket: socket.socket | None = None
        self.frames_seen = 0
        self.packets_sent = 0
        self.tracked_packets_sent = 0
        self._last_reported_error = ""

    def connect(self) -> None:
        addresses = socket.getaddrinfo(
            self.host,
            self.port,
            family=socket.AF_INET,
            type=socket.SOCK_DGRAM,
        )
        if not addresses:
            raise OSError(f"Could not resolve Form destination {self.host}:{self.port}")
        destination = addresses[0][4]
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        if self.source_host:
            sock.bind((self.source_host, 0))
        sock.connect(destination)
        self._socket = sock
        print(f"[quest3-bridge] udp_destination={destination[0]}:{destination[1]}")
        source = sock.getsockname()
        print(f"[quest3-bridge] udp_source={source[0]}:{source[1]}")

    def publish(self, snapshot: Mapping[str, Any] | None) -> None:
        self.frames_seen += 1
        packet = self.encoder.packet_for_snapshot(snapshot)
        if packet is None:
            self._report_encoder_error_if_changed()
            return
        self._send(packet)
        if packet["tracked"]:
            self.tracked_packets_sent += 1
            if (
                self.tracked_packets_sent == 1
                or self.tracked_packets_sent % self.print_every == 0
            ):
                print(
                    "[quest3-bridge] "
                    f"tracking=active sent={self.tracked_packets_sent} "
                    f"source_frames={self.frames_seen} sequence={packet['sequence']}"
                )
        else:
            detail = self.encoder.last_error or "hand_not_tracked"
            print(f"[quest3-bridge] tracking=lost detail={detail}")

    def close(self) -> None:
        if self._socket is None:
            return
        loss_packet = self.encoder.force_tracking_lost()
        if loss_packet is not None:
            try:
                self._send(loss_packet)
            except OSError:
                pass
        self._socket.close()
        self._socket = None

    def _send(self, packet: Mapping[str, Any]) -> None:
        if self._socket is None:
            raise RuntimeError("UDP publisher is not connected")
        self._socket.send(encode_udp_packet(packet))
        self.packets_sent += 1

    def _report_encoder_error_if_changed(self) -> None:
        error = self.encoder.last_error
        if error and error != self._last_reported_error:
            self._last_reported_error = error
            print(f"[quest3-bridge] frame_dropped={error}", file=sys.stderr)


def _run_session(
    session: Any,
    snapshot_converter: Callable[[dict[str, Any]], dict[str, object] | None],
    publisher: UdpHandPublisher,
    *,
    loop_hz: float,
    duration_s: float,
) -> None:
    period_s = 0.0 if loop_hz <= 0.0 else 1.0 / loop_hz
    deadline = None if duration_s <= 0.0 else time.monotonic() + duration_s
    entered = False
    exc_info: tuple[Any, Any, Any] = (None, None, None)
    try:
        session.__enter__()
        entered = True
        while deadline is None or time.monotonic() < deadline:
            started_at = time.monotonic()
            result = session.step()
            publisher.publish(snapshot_converter(result))
            remaining = period_s - (time.monotonic() - started_at)
            if remaining > 0.0:
                time.sleep(remaining)
    except BaseException:
        exc_info = sys.exc_info()
        raise
    finally:
        # Teleop's own runner also calls __exit__ after a failed __enter__ so
        # partially created OpenXR resources are released before a retry.
        if entered or exc_info[0] is not None:
            try:
                session.__exit__(*exc_info)
            except Exception:
                if entered and exc_info[0] is None:
                    raise


def _is_retryable_openxr_startup_error(exc: Exception) -> bool:
    message = str(exc)
    return any(
        marker in message
        for marker in (
            "Failed to get OpenXR system: -35",
            "Failed to get OpenXR system: -33",
        )
    )


def _run_with_startup_retry(
    session_factory: Callable[[], tuple[Any, Callable[[dict[str, Any]], dict[str, object] | None]]],
    publisher: UdpHandPublisher,
    *,
    loop_hz: float,
    duration_s: float,
    startup_timeout_s: float,
    retry_interval_s: float,
) -> None:
    deadline = time.monotonic() + max(0.0, startup_timeout_s)
    attempt = 0
    while True:
        attempt += 1
        session, snapshot_converter = session_factory()
        try:
            _run_session(
                session,
                snapshot_converter,
                publisher,
                loop_hz=loop_hz,
                duration_s=duration_s,
            )
            return
        except Exception as exc:
            if not _is_retryable_openxr_startup_error(exc) or time.monotonic() >= deadline:
                raise
            remaining = max(0.0, deadline - time.monotonic())
            print(
                "[quest3-bridge] waiting_for_quest_session "
                f"attempt={attempt} remaining_s={remaining:.1f}"
            )
            time.sleep(max(0.0, retry_interval_s))


def main(argv: list[str] | None = None) -> int:
    args = _build_parser().parse_args(argv)
    if args.loop_hz < 0.0:
        raise ValueError("--loop-hz must be non-negative")
    if args.loss_timeout < 0.0:
        raise ValueError("--loss-timeout must be non-negative")

    teleop_root = _resolve_teleop_root(args.teleop_root)
    api = _load_teleop_api()
    print(f"[quest3-bridge] teleop_root={teleop_root}")
    if args.check_only:
        print("[quest3-bridge] check=ok Teleop/IsaacTeleop imports are available")
        return 0

    if not args.skip_runtime_check:
        socket_path = _validate_cloudxr_runtime()
        print(f"[quest3-bridge] cloudxr_ipc={socket_path}")

    publisher = UdpHandPublisher(
        host=args.host,
        port=args.port,
        source_host=args.source_host,
        encoder=TeleopHandPacketEncoder(
            hand_side=args.hand,
            loss_timeout_s=args.loss_timeout,
        ),
        print_every=args.print_every,
    )
    publisher.connect()
    print(
        "[quest3-bridge] starting "
        f"hand={args.hand} loop_hz={args.loop_hz:g} "
        f"synthetic_hands={not args.disable_synthetic_hands_plugin}"
    )
    try:
        _run_with_startup_retry(
            lambda: _build_raw_hand_session(
                api,
                hand_side=args.hand,
                isaac_teleop_root=args.isaac_teleop_root,
                enable_synthetic_hands_plugin=not args.disable_synthetic_hands_plugin,
            ),
            publisher,
            loop_hz=float(args.loop_hz),
            duration_s=float(args.duration),
            startup_timeout_s=float(args.startup_timeout),
            retry_interval_s=float(args.startup_retry_interval),
        )
    except KeyboardInterrupt:
        print("\n[quest3-bridge] interrupted")
    finally:
        publisher.close()
    print(
        "[quest3-bridge] stopped "
        f"source_frames={publisher.frames_seen} packets_sent={publisher.packets_sent}"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"[quest3-bridge] fatal={type(exc).__name__}: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc
