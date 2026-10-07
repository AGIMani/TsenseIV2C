"""Non-blocking MediaPipe -> retarget -> H7 session recorder.

The real-time threads only build a small event dictionary and enqueue it.  JSON
serialization and disk I/O are owned by a daemon writer thread, so enabling the
recorder does not put file writes on the tracker or 500 Hz downlink path.
"""

from __future__ import annotations

import json
import math
import queue
import threading
import time
import struct
from contextlib import ExitStack
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence

from utils.control_log_catalog import control_log_parent


_QUEUE_CAPACITY = 32768
_STOP = object()
LOG_PART_BYTES = 512 * 1024 * 1024


def control_log_directory(metadata: Mapping[str, Any] | None = None) -> Path:
    model = str((metadata or {}).get("model_id", "TsenseI_v2")).lower()
    folder = "A30" if "v3_0" in model or "a30" in model else "A32" if "v3_2" in model or "a32" in model else "V2"
    root = control_log_parent()
    return root / folder


def _finite_float(value: Any) -> float | None:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    if number != number or number in (float("inf"), float("-inf")):
        return None
    return round(number, 9)


def _joint_dict(values: Mapping[str, Any] | None) -> dict[str, float]:
    if not values:
        return {}
    result: dict[str, float] = {}
    for name, value in values.items():
        number = _finite_float(value)
        if number is not None:
            result[str(name)] = number
    return result


def _landmark_rows(values: Sequence[Sequence[Any]] | None) -> list[list[float | None]]:
    if values is None:
        return []
    rows: list[list[float | None]] = []
    for point in values:
        rows.append([_finite_float(value) for value in point[:3]])
    return rows


class ControlSessionRecorder:
    """Thread-safe JSONL recorder with a bounded, non-blocking producer queue."""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._active = False
        self._queue: queue.Queue[Any] | None = None
        self._thread: threading.Thread | None = None
        self._path: Path | None = None
        self._t0 = 0.0
        self._seq = 0
        self._dropped = 0
        self._last_commands: dict[str, dict[str, Any]] = {}
        self._error: str | None = None

    @property
    def error(self) -> str | None:
        if not self._active and self._thread is not None and self._thread.is_alive():
            return self._error or "Recording is still draining to disk"
        return self._error

    @property
    def active(self) -> bool:
        with self._lock:
            return self._active

    @property
    def path(self) -> Path | None:
        with self._lock:
            return self._path

    def start(
        self,
        *,
        output_dir: str | Path | None = None,
        metadata: Mapping[str, Any] | None = None,
    ) -> Path:
        with self._lock:
            if self._active and self._path is not None:
                return self._path
            directory = Path(output_dir) if output_dir else control_log_directory(metadata)
            directory.mkdir(parents=True, exist_ok=True)
            if self._thread is not None and self._thread.is_alive():
                raise RuntimeError("Previous recording is still being saved")
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            path = directory / f"hand_control_{stamp}.jsonl"
            event_queue: queue.Queue[Any] = queue.Queue(maxsize=_QUEUE_CAPACITY)
            self._queue = event_queue
            self._path = path
            self._t0 = time.perf_counter()
            self._seq = 0
            self._dropped = 0
            self._last_commands = {}
            self._error = None
            # Fail synchronously on an unwritable destination, before advertising
            # an active recording to the UI.
            stream = path.open("x", encoding="utf-8", newline="\n", buffering=1024 * 1024)
            self._active = True
            self._thread = threading.Thread(
                target=self._writer_main,
                args=(stream, event_queue),
                name="ControlSessionWriter",
                daemon=True,
            )
            self._thread.start()
            self._enqueue_locked(
                {
                    "type": "session",
                    "schema": "hand-control-session/v2",
                    "session_id": path.stem,
                    "recording_part": {"index": 1, "count": None, "session_id": path.stem},
                    "started_at": datetime.now(timezone.utc).astimezone().isoformat(),
                    "joint_unit": "rad",
                    "landmark_unit": "m",
                    "units": {"linear": "mm", "rotary": "deg", "temperature": "degC", "current": "firmware_native"},
                    "feedback_correlation": "latest successful host write before receive; not a firmware acknowledgement",
                    "metadata": {"model_id": "TsenseI_v2", **dict(metadata or {}),
                                 "time_fields": {"t": "monotonic seconds since session start",
                                                 "wall_time_unix_ms": "host wall clock at event capture (Unix ms)",
                                                 "wall_time": "ISO 8601 local time with UTC offset",
                                                 "tick_ms": "device clock from telemetry, not host time"}},
                }
            )
            return path

    def stop(self, *, reason: str = "sync_stopped") -> Path | None:
        with self._lock:
            if not self._active:
                return self._path
            path = self._path
            event_queue = self._queue
            writer = self._thread
            self._enqueue_locked(
                {
                    "type": "session_end",
                    "reason": str(reason),
                    "dropped_events": self._dropped,
                    "ended_at": datetime.now(timezone.utc).astimezone().isoformat(),
                },
                force=True,
            )
            self._active = False
            if event_queue is not None:
                try:
                    event_queue.put(_STOP, timeout=1.0)
                except queue.Full:
                    # The writer drains continuously; retry once after yielding.
                    time.sleep(0.01)
                    event_queue.put(_STOP, timeout=1.0)
            self._queue = None
        if writer is not None:
            writer.join(timeout=10.0)
        return path

    def record_mediapipe(
        self,
        landmarks: Sequence[Sequence[Any]],
        *,
        frame_id: int,
        hand_side: str,
        depth_source: str,
        timestamp_ms: int | None = None,
    ) -> None:
        self._record(
            "mediapipe",
            frame_id=int(frame_id),
            hand_side=str(hand_side),
            depth_source=str(depth_source),
            timestamp_ms=int(timestamp_ms) if timestamp_ms is not None else None,
            landmarks=_landmark_rows(landmarks),
        )

    def record_retarget(
        self,
        joints: Mapping[str, Any],
        *,
        frame_id: int,
        hand_side: str,
        solve_ms: float | None = None,
        landmark_repairs: Sequence[Mapping[str, Any]] | None = None,
    ) -> None:
        self._record(
            "retarget",
            frame_id=int(frame_id),
            hand_side=str(hand_side),
            solve_ms=_finite_float(solve_ms),
            joints=_joint_dict(joints),
            landmark_repairs=[dict(item) for item in (landmark_repairs or ())],
        )

    def record_h7(
        self,
        *,
        joints_ui: Mapping[str, Any],
        joints_hw: Mapping[str, Any],
        frame_id: int,
        hand_side: str,
        sent: bool,
        usb_connected: bool,
        ruckig_faults: Any = None,
        guard_limited: bool | None = None,
        joints_before_guard: Mapping[str, Any] | None = None,
        joints_ruckig_ui: Mapping[str, Any] | None = None,
        control_dt_s: float | None = None,
    ) -> None:
        self._record(
            "h7",
            control_dt_s=_finite_float(control_dt_s),
            frame_id=int(frame_id),
            hand_side=str(hand_side),
            sent=bool(sent),
            usb_connected=bool(usb_connected),
            guard_limited=guard_limited,
            ruckig_faults=list(ruckig_faults or ()),
            joints_ui=_joint_dict(joints_ui),
            joints_hw=_joint_dict(joints_hw),
            joints_before_guard=_joint_dict(joints_before_guard),
            joints_ruckig_ui=_joint_dict(joints_ruckig_ui),
        )

    def record_stage(self, stage: str, joints: Mapping[str, Any], *,
                     frame_id: int, hand_side: str, **details: Any) -> None:
        if not self.active:
            return
        self._record(stage, joints=_joint_dict(joints), frame_id=frame_id,
                     hand_side=hand_side, **details)

    def record_mediapipe_raw(self, result: Any, *, frame_id: int,
                             timestamp_ms: int, infer_ms: float) -> None:
        if not self.active:
            return
        def points(hands):
            return [_landmark_rows([(p.x, p.y, p.z) for p in hand]) for hand in (hands or ())]
        self._record("mediapipe_raw", frame_id=frame_id, timestamp_ms=timestamp_ms,
                     infer_ms=_finite_float(infer_ms),
                     normalized_landmarks=points(result.hand_landmarks),
                     world_landmarks=points(result.hand_world_landmarks),
                     handedness=[[{"label": c.category_name, "score": _finite_float(c.score)}
                                  for c in cats] for cats in (result.handedness or ())])

    def record_conversion(self, joints: Mapping[str, Any], linear: Sequence[float],
                          rotary: Sequence[float], *, frame_id: int | None,
                          hand_side: str, source: str) -> int | None:
        if not self.active:
            return None
        return self._record("conversion", joints_hw=_joint_dict(joints),
                            linear_mm=list(linear), rotary_deg=list(rotary),
                            frame_id=frame_id, hand_side=hand_side, source=source)

    def record_downlink(self, frame: bytes, *, sent: bool, hand_side: str,
                        source: str, frame_id: int | None = None,
                        command_id: int | None = None, **details: Any) -> None:
        if not self.active:
            return
        # Decode the actual float32 wire values, including protocol sign changes.
        values = struct.unpack_from("<B14f7f", frame, 6)
        with self._lock:
            seq = self._record("downlink", wire_seq=values[0],
                               linear_mm=list(values[1:15]), rotary_deg=list(values[15:22]),
                               wire_hex=frame.hex(), sent=bool(sent), hand_side=hand_side,
                               source=source, frame_id=frame_id, command_id=command_id, **details)
            if sent and seq is not None:
                self._last_commands[hand_side] = {"downlink_seq": seq,
                    "command_id": command_id, "wire_seq": values[0], "frame_id": frame_id}

    def record_usb_tx(self, frame: bytes, *, sent: bool) -> None:
        if self.active:
            self._record("usb_tx", wire_hex=frame.hex(), sent=bool(sent))

    def record_telemetry_payload(self, payload: bytes, *, accepted: bool) -> None:
        # Called for EVERY CRC-valid telemetry packet, before the UI's latest-only
        # cache. Parsing and dataclass conversion run on the disk writer thread.
        with self._lock:
            self._record("telemetry", _payload=bytes(payload), accepted=bool(accepted),
                         latest_host_commands={k: dict(v) for k, v in self._last_commands.items()})

    @staticmethod
    def _decode_telemetry(item: dict[str, Any]) -> None:
        from usb.telemetry import (TELEM_FRAME_SIZE, parse_hand_telem_v2,
                                   parse_hand_telem_legacy, mcu_temp_plausible,
                                   slot_is_active_linear, slot_is_active_rotary)
        payload = item.pop("_payload")
        item["payload_hex"] = payload.hex()
        frame = (parse_hand_telem_v2(payload) if len(payload) == TELEM_FRAME_SIZE
                 else parse_hand_telem_legacy(payload))
        if frame is None:
            item["parse_error"] = "unsupported telemetry payload"
            return
        side = "right" if frame.get("hand_type", 0) == 1 else "left"
        item.update(hand_side=side, telemetry_seq=frame["seq"], tick_ms=frame["tick_ms"],
                    correlation="latest_host_write",
                    latest_host_command=item.pop("latest_host_commands", {}).get(side))
        item["linear"] = []
        for index, slot in enumerate(frame["linear"]):
            # Parsed slot fields are scalars; avoid dataclasses.asdict's recursive
            # deepcopy on every motor of every high-frequency telemetry frame.
            row = vars(slot).copy()
            row["active"] = slot_is_active_linear(slot, index)
            row["mcu_temp_c"] = (slot.mcu_temp_c if frame.get("fmt") == "v2"
                                  and mcu_temp_plausible(slot.mcu_temp_centi) else None)
            item["linear"].append(row)
        item["rotary"] = [{**vars(slot), "active": slot_is_active_rotary(slot)}
                          for slot in frame["rotary"]]
        item["board"] = {k: v for k, v in frame.items()
                         if k not in ("linear", "rotary", "payload", "_parsed")}

    def _record(self, event_type: str, **payload: Any) -> int | None:
        with self._lock:
            if not self._active:
                return
            payload["type"] = event_type
            return self._enqueue_locked(payload)

    def _enqueue_locked(self, payload: dict[str, Any], *, force: bool = False) -> int | None:
        event_queue = self._queue
        if event_queue is None:
            return
        self._seq += 1
        payload["seq"] = self._seq
        payload["t"] = round(max(0.0, time.perf_counter() - self._t0), 9)
        # Sample at the producer, not later when the disk writer drains a queue.
        # Durations continue to use monotonic t if the OS wall clock changes.
        payload["wall_time_unix_ms"] = time.time_ns() / 1_000_000
        try:
            if force:
                event_queue.put(payload, timeout=1.0)
            else:
                event_queue.put_nowait(payload)
            return self._seq
        except queue.Full:
            self._dropped += 1

    def _writer_main(self, stream: Any, event_queue: queue.Queue[Any]) -> None:
        def json_safe(value):
            if isinstance(value, float):
                return value if math.isfinite(value) else None
            if isinstance(value, dict):
                return {k: json_safe(v) for k, v in value.items()}
            if isinstance(value, (list, tuple)):
                return [json_safe(v) for v in value]
            return value
        with ExitStack() as streams:
            streams.enter_context(stream)
            byte_count, part_index, header = 0, 1, None
            pending = 0
            last_flush = time.monotonic()
            while True:
                try:
                    item = event_queue.get(timeout=0.5)
                except queue.Empty:
                    if pending and self._error is None:
                        try:
                            stream.flush()
                        except OSError as exc:
                            self._error = str(exc)
                        pending = 0
                    continue
                if item is _STOP:
                    break
                if self._error is not None:
                    continue  # Drain after disk failure so stop never deadlocks.
                try:
                    if "wall_time_unix_ms" in item:
                        item["wall_time"] = datetime.fromtimestamp(
                            item["wall_time_unix_ms"] / 1000., timezone.utc).astimezone().isoformat(timespec="milliseconds")
                    if "_payload" in item:
                        try:
                            self._decode_telemetry(item)
                        except Exception as exc:
                            item.pop("_payload", None)
                            item["parse_error"] = str(exc)
                    line = json.dumps(json_safe(item), ensure_ascii=False,
                                      allow_nan=False, separators=(",", ":")) + "\n"
                    size = len(line.encode("utf-8"))
                    if item["type"] == "session":
                        header = json_safe(item)
                    if size + (len(json.dumps(header, ensure_ascii=False).encode("utf-8")) if header else 0) + 256 > LOG_PART_BYTES:
                        raise ValueError("Single recording event exceeds the log part size limit")
                    if byte_count and byte_count + size > LOG_PART_BYTES:
                        part_index += 1
                        part_path = self._path.with_name(f"{self._path.stem}_part{part_index:03d}.jsonl")
                        next_stream = streams.enter_context(part_path.open("x", encoding="utf-8", newline="\n", buffering=1024 * 1024))
                        stream.close()
                        stream = next_stream
                        part_header = {**header, "recording_part": {"index": part_index, "count": None, "session_id": header["session_id"]}}
                        header_line = json.dumps(part_header, ensure_ascii=False, separators=(",", ":")) + "\n"
                        stream.write(header_line)
                        byte_count = len(header_line.encode("utf-8"))
                    stream.write(line)
                    byte_count += size
                except (OSError, TypeError, ValueError) as exc:
                    self._error = str(exc)
                    continue
                pending += 1
                now = time.monotonic()
                if pending >= 256 or now - last_flush >= 0.5:
                    try:
                        stream.flush()
                    except OSError as exc:
                        self._error = str(exc)
                    pending = 0
                    last_flush = now
            try:
                stream.flush()
            except OSError as exc:
                self._error = str(exc)


RECORDER = ControlSessionRecorder()
