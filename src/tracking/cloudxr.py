"""CloudXR Quest 3 hand-landmark input.

The CloudXR/OpenXR process stays outside this application and publishes one
JSON datagram per hand-tracking frame.  This module owns the small UDP bridge,
validates packets, and converts OpenXR's 26-joint order to the MediaPipe 21
joint order used by the existing retargeting pipeline.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import socket
import threading
import time
from typing import Any, Callable, Mapping, Sequence


DEFAULT_CLOUDXR_HOST = "127.0.0.1"
DEFAULT_CLOUDXR_PORT = 28770
MAX_DATAGRAM_BYTES = 64 * 1024
RECEIVER_STALE_STATUS_S = 0.5

# XR_EXT_hand_tracking order -> MediaPipe Hands order.  OpenXR has an extra
# palm joint and one metacarpal joint for every non-thumb finger.
OPENXR_26_TO_MEDIAPIPE_21: tuple[int, ...] = (
    1,                          # wrist
    2, 3, 4, 5,                # thumb: metacarpal, proximal, distal, tip
    7, 8, 9, 10,               # index: proximal, intermediate, distal, tip
    12, 13, 14, 15,            # middle
    17, 18, 19, 20,            # ring
    22, 23, 24, 25,            # little
)


class CloudXRPacketError(ValueError):
    """Raised when a CloudXR landmark datagram does not match the protocol."""


@dataclass(frozen=True)
class CloudXRHandFrame:
    """One validated hand frame in MediaPipe-21 order and metres."""

    landmarks: tuple[tuple[float, float, float], ...]
    hand_side: str
    tracked: bool
    timestamp_ms: int
    sequence: int | None
    coordinate_space: str
    source_joint_count: int
    received_monotonic: float


@dataclass(frozen=True)
class CloudXRReceiverStatus:
    state: str                 # stopped | listening | receiving | warning | error
    message: str
    host: str
    port: int
    peer: str = ""
    packets_received: int = 0
    invalid_packets: int = 0


def _first(mapping: Mapping[str, Any], *names: str, default: Any = None) -> Any:
    for name in names:
        if name in mapping:
            return mapping[name]
    return default


def _as_bool(value: Any, *, default: bool) -> bool:
    if value is None:
        return default
    if isinstance(value, str):
        text = value.strip().lower()
        if text in {"0", "false", "no", "off", "lost", "inactive"}:
            return False
        if text in {"1", "true", "yes", "on", "tracked", "active"}:
            return True
    return bool(value)


def _normalize_hand_side(value: Any) -> str:
    text = str(value or "left").strip().lower()
    if text in {"r", "right", "right_hand", "hand_right"} or text.startswith("right"):
        return "right"
    return "left"


def _timestamp_ms(record: Mapping[str, Any], envelope: Mapping[str, Any]) -> int:
    try:
        value = _first(record, "timestamp_ms", "time_ms")
        if value is None:
            value = _first(envelope, "timestamp_ms", "time_ms")
        if value is not None:
            return int(float(value))

        value = _first(record, "timestamp_us", "time_us")
        if value is None:
            value = _first(envelope, "timestamp_us", "time_us")
        if value is not None:
            return int(float(value) / 1_000.0)

        value = _first(record, "timestamp_ns", "time_ns")
        if value is None:
            value = _first(envelope, "timestamp_ns", "time_ns")
        if value is not None:
            return int(float(value) / 1_000_000.0)
    except (TypeError, ValueError, OverflowError) as exc:
        raise CloudXRPacketError("timestamp must be a finite number") from exc

    return int(time.monotonic() * 1000.0)


def _point_xyz(point: Any) -> tuple[float, float, float]:
    if isinstance(point, Mapping):
        nested = _first(point, "position", "pos", "translation")
        if isinstance(nested, Mapping) or (
            isinstance(nested, Sequence) and not isinstance(nested, (str, bytes, bytearray))
        ):
            point = nested

    if isinstance(point, Mapping):
        try:
            xyz = (float(point["x"]), float(point["y"]), float(point["z"]))
        except (KeyError, TypeError, ValueError) as exc:
            raise CloudXRPacketError("joint object must contain numeric x/y/z") from exc
    elif isinstance(point, Sequence) and not isinstance(point, (str, bytes, bytearray)):
        if len(point) < 3:
            raise CloudXRPacketError("joint array must contain at least three values")
        try:
            xyz = (float(point[0]), float(point[1]), float(point[2]))
        except (TypeError, ValueError) as exc:
            raise CloudXRPacketError("joint coordinates must be numeric") from exc
    else:
        raise CloudXRPacketError("each joint must be an [x,y,z] array or position object")

    if not all(math.isfinite(value) for value in xyz):
        raise CloudXRPacketError("joint coordinates must be finite")
    return xyz


def _unit_scale(value: Any) -> float:
    unit = str(value or "m").strip().lower()
    scales = {
        "m": 1.0,
        "meter": 1.0,
        "meters": 1.0,
        "metre": 1.0,
        "metres": 1.0,
        "cm": 0.01,
        "centimeter": 0.01,
        "centimeters": 0.01,
        "mm": 0.001,
        "millimeter": 0.001,
        "millimeters": 0.001,
    }
    try:
        return scales[unit]
    except KeyError as exc:
        raise CloudXRPacketError(f"unsupported landmark unit: {unit!r}") from exc


def _select_hand(envelope: Mapping[str, Any]) -> Mapping[str, Any]:
    hands = envelope.get("hands")
    if hands is None:
        hand = envelope.get("hand_data")
        return hand if isinstance(hand, Mapping) else envelope
    if not isinstance(hands, Sequence) or isinstance(hands, (str, bytes, bytearray)):
        raise CloudXRPacketError("hands must be an array")
    if not hands:
        return {"tracked": False}

    candidates = [hand for hand in hands if isinstance(hand, Mapping)]
    if not candidates:
        raise CloudXRPacketError("hands array contains no hand objects")

    tracked = [
        hand
        for hand in candidates
        if _as_bool(_first(hand, "tracked", "is_tracked", "active"), default=True)
    ]
    pool = tracked or candidates

    def confidence(hand: Mapping[str, Any]) -> float:
        try:
            return float(_first(hand, "confidence", "score", default=0.0) or 0.0)
        except (TypeError, ValueError):
            return 0.0

    return max(pool, key=confidence)


def _decode_envelope(payload: bytes | bytearray | str | Mapping[str, Any]) -> Mapping[str, Any]:
    if isinstance(payload, Mapping):
        return payload
    try:
        if isinstance(payload, (bytes, bytearray)):
            payload = bytes(payload).decode("utf-8")
        decoded = json.loads(payload)
    except (UnicodeDecodeError, json.JSONDecodeError, TypeError) as exc:
        raise CloudXRPacketError("datagram must be a UTF-8 JSON object") from exc
    if not isinstance(decoded, Mapping):
        raise CloudXRPacketError("JSON root must be an object")
    return decoded


def parse_cloudxr_packet(
    payload: bytes | bytearray | str | Mapping[str, Any],
) -> CloudXRHandFrame:
    """Parse one CloudXR JSON packet.

    Accepted point arrays contain either 21 joints in MediaPipe order or 26
    joints in ``XR_EXT_hand_tracking`` order.  Coordinates are normalized to
    metres; global orientation is intentionally left intact for the tracker's
    wrist-frame conversion.
    """

    envelope = _decode_envelope(payload)
    record = _select_hand(envelope)
    tracked = _as_bool(
        _first(record, "tracked", "is_tracked", "active", "present"),
        default=True,
    )
    side = _normalize_hand_side(
        _first(
            record,
            "hand_side",
            "handedness",
            "side",
            "hand",
            default=_first(envelope, "hand_side", "handedness", "side", "hand"),
        )
    )
    sequence_value = _first(
        record,
        "sequence",
        "seq",
        "frame_id",
        default=_first(envelope, "sequence", "seq", "frame_id"),
    )
    try:
        sequence = int(sequence_value) if sequence_value is not None else None
    except (TypeError, ValueError) as exc:
        raise CloudXRPacketError("sequence/frame_id must be an integer") from exc

    coordinate_space = str(
        _first(
            record,
            "coordinate_space",
            "space",
            default=_first(envelope, "coordinate_space", "space", default="openxr"),
        )
        or "openxr"
    ).strip().lower()
    timestamp_ms = _timestamp_ms(record, envelope)

    raw_points = _first(record, "landmarks", "keypoints", "points", "joints")
    if raw_points is None:
        if not tracked:
            return CloudXRHandFrame(
                landmarks=(),
                hand_side=side,
                tracked=False,
                timestamp_ms=timestamp_ms,
                sequence=sequence,
                coordinate_space=coordinate_space,
                source_joint_count=0,
                received_monotonic=time.monotonic(),
            )
        raise CloudXRPacketError("tracked hand packet has no landmarks/joints array")
    if not isinstance(raw_points, Sequence) or isinstance(
        raw_points, (str, bytes, bytearray)
    ):
        raise CloudXRPacketError("landmarks/joints must be an array")

    if not tracked and len(raw_points) == 0:
        return CloudXRHandFrame(
            landmarks=(),
            hand_side=side,
            tracked=False,
            timestamp_ms=timestamp_ms,
            sequence=sequence,
            coordinate_space=coordinate_space,
            source_joint_count=0,
            received_monotonic=time.monotonic(),
        )

    source_count = len(raw_points)
    if source_count not in (21, 26):
        raise CloudXRPacketError(
            f"expected 21 MediaPipe or 26 OpenXR joints, got {source_count}"
        )
    points = tuple(_point_xyz(point) for point in raw_points)
    if source_count == 26:
        points = tuple(points[index] for index in OPENXR_26_TO_MEDIAPIPE_21)

    scale = _unit_scale(
        _first(record, "unit", "units", default=_first(envelope, "unit", "units", default="m"))
    )
    if scale != 1.0:
        points = tuple(
            (point[0] * scale, point[1] * scale, point[2] * scale)
            for point in points
        )

    return CloudXRHandFrame(
        landmarks=points,
        hand_side=side,
        tracked=tracked,
        timestamp_ms=timestamp_ms,
        sequence=sequence,
        coordinate_space=coordinate_space,
        source_joint_count=source_count,
        received_monotonic=time.monotonic(),
    )


class CloudXRLandmarkReceiver:
    """Small stoppable UDP receiver used by the main window.

    Callbacks run on the receiver thread.  GUI owners must forward them through
    a Qt signal before touching widgets.
    """

    def __init__(
        self,
        *,
        on_frame: Callable[[CloudXRHandFrame], None],
        on_status: Callable[[CloudXRReceiverStatus], None] | None = None,
    ) -> None:
        self._on_frame = on_frame
        self._on_status = on_status
        self._lock = threading.RLock()
        self._socket: socket.socket | None = None
        self._thread: threading.Thread | None = None
        self._stop_event = threading.Event()
        self._host = DEFAULT_CLOUDXR_HOST
        self._port = DEFAULT_CLOUDXR_PORT
        self._packets_received = 0
        self._invalid_packets = 0

    @property
    def running(self) -> bool:
        with self._lock:
            return self._thread is not None and self._thread.is_alive()

    @property
    def endpoint(self) -> tuple[str, int]:
        with self._lock:
            return self._host, self._port

    def start(self, host: str = DEFAULT_CLOUDXR_HOST, port: int = DEFAULT_CLOUDXR_PORT) -> tuple[str, int]:
        self.stop(notify=False)
        host = str(host or DEFAULT_CLOUDXR_HOST).strip()
        port = int(port)
        if not (0 <= port <= 65535):
            raise ValueError("CloudXR UDP port must be between 0 and 65535")

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(0.2)
        try:
            sock.bind((host, port))
        except Exception:
            sock.close()
            raise
        bound_host, bound_port = sock.getsockname()[:2]

        with self._lock:
            self._host = str(bound_host)
            self._port = int(bound_port)
            self._socket = sock
            self._packets_received = 0
            self._invalid_packets = 0
            self._stop_event = threading.Event()
            stop_event = self._stop_event
            self._thread = threading.Thread(
                target=self._run,
                args=(sock, stop_event),
                name="CloudXRLandmarks",
                daemon=True,
            )
            thread = self._thread
        thread.start()
        self._emit_status("listening", f"监听 {bound_host}:{bound_port}")
        return str(bound_host), int(bound_port)

    def stop(self, *, notify: bool = True) -> None:
        with self._lock:
            thread = self._thread
            sock = self._socket
            stop_event = self._stop_event
            was_running = thread is not None or sock is not None
            self._thread = None
            self._socket = None
            stop_event.set()
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        if notify and was_running:
            self._emit_status("stopped", "未监听")

    def _emit_status(self, state: str, message: str, *, peer: str = "") -> None:
        callback = self._on_status
        if callback is None:
            return
        with self._lock:
            status = CloudXRReceiverStatus(
                state=state,
                message=message,
                host=self._host,
                port=self._port,
                peer=peer,
                packets_received=self._packets_received,
                invalid_packets=self._invalid_packets,
            )
        try:
            callback(status)
        except Exception:
            pass

    def _run(self, sock: socket.socket, stop_event: threading.Event) -> None:
        last_status_at = 0.0
        last_packet_at = 0.0
        stale_notified = False
        while not stop_event.is_set():
            try:
                payload, address = sock.recvfrom(MAX_DATAGRAM_BYTES)
            except socket.timeout:
                if (
                    last_packet_at > 0.0
                    and not stale_notified
                    and time.monotonic() - last_packet_at >= RECEIVER_STALE_STATUS_S
                ):
                    stale_notified = True
                    self._emit_status("listening", "等待 CloudXR 数据")
                continue
            except OSError as exc:
                if stop_event.is_set():
                    break
                with self._lock:
                    if self._socket is sock:
                        self._socket = None
                    if self._thread is threading.current_thread():
                        self._thread = None
                try:
                    sock.close()
                except OSError:
                    pass
                self._emit_status("error", f"UDP 接收失败: {exc}")
                return

            peer = f"{address[0]}:{address[1]}"
            last_packet_at = time.monotonic()
            stale_notified = False
            try:
                frame = parse_cloudxr_packet(payload)
            except Exception as exc:
                with self._lock:
                    self._invalid_packets += 1
                now = time.monotonic()
                if now - last_status_at >= 1.0:
                    last_status_at = now
                    self._emit_status("warning", f"报文格式错误: {exc}", peer=peer)
                continue

            with self._lock:
                self._packets_received += 1
            try:
                self._on_frame(frame)
            except Exception as exc:
                self._emit_status("error", f"分发 CloudXR 数据失败: {exc}", peer=peer)
                continue

            now = time.monotonic()
            if self._packets_received == 1 or now - last_status_at >= 1.0:
                last_status_at = now
                self._emit_status(
                    "receiving",
                    f"接收中 · {self._packets_received} 帧",
                    peer=peer,
                )

        with self._lock:
            if self._socket is sock:
                self._socket = None
            if self._thread is threading.current_thread():
                self._thread = None
