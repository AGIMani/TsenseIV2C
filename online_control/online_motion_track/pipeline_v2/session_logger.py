"""Per-run JSONL diagnostics for the light MediaPipe/retarget pipeline."""

from __future__ import annotations

import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping

import numpy as np


RECORDING_OFF = "off"
RECORDING_MEDIAPIPE = "mediapipe"
RECORDING_FULL = "full"
RECORDING_MODES = frozenset(
    {RECORDING_OFF, RECORDING_MEDIAPIPE, RECORDING_FULL}
)

# This is the smallest lossless replay boundary for MediaPipe -> TopoRetarget.
# Raw fields are kept for depth/smoothing diagnosis; ``wuji_points_21_m`` lets
# a future run reproduce the downstream chain without needing a webcam or the
# exact MediaPipe binary/model version.
MEDIAPIPE_REPLAY_FIELDS = frozenset(
    {
        "frame_id",
        "hand_side",
        "camera_size",
        "detected",
        "held_last_valid",
        "used_fallback_hand",
        "detected_label",
        "expected_label",
        "hand_selection_match",
        "downstream_blocked_reason",
        "raw_normalized_points_21",
        "smoothed_normalized_points_21",
        "raw_world_points_21_m",
        "smoothed_world_points_21_m",
        "depth_source",
        "wuji_points_21_m",
        "topo_input_points_21_m",
        "retarget_quality_ok",
        "tracking_transition",
    }
)


def normalise_recording_mode(value: str) -> str:
    mode = str(value).strip().lower()
    if mode not in RECORDING_MODES:
        raise ValueError(
            f"recording mode must be one of {sorted(RECORDING_MODES)}, got {value!r}"
        )
    return mode


def select_frame_fields(
    frame_record: Mapping[str, Any],
    mode: str,
) -> dict[str, Any]:
    """Return the stable per-frame schema for the selected recording level."""
    selected_mode = normalise_recording_mode(mode)
    if selected_mode == RECORDING_OFF:
        return {}
    if selected_mode == RECORDING_FULL:
        return dict(frame_record)
    return {
        key: value
        for key, value in frame_record.items()
        if key in MEDIAPIPE_REPLAY_FIELDS
    }


def _json_safe(value: Any) -> Any:
    """Convert numpy/scalar values and replace non-finite floats with null."""
    if isinstance(value, np.ndarray):
        return _json_safe(value.tolist())
    if isinstance(value, np.generic):
        return _json_safe(value.item())
    if isinstance(value, Mapping):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, Path):
        return str(value)
    return value


def _utc_iso_now() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec="milliseconds")


class SessionJsonlLogger:
    """Write one self-contained JSON object per camera frame.

    The file is intentionally plain JSONL so a broken/aborted run still leaves
    all complete lines readable.  A flush every few frames balances durability
    and camera-thread overhead.
    """

    def __init__(
        self,
        log_dir: str | Path,
        *,
        session_metadata: Mapping[str, Any],
        recording_mode: str = RECORDING_FULL,
        flush_every: int = 10,
    ) -> None:
        self.recording_mode = normalise_recording_mode(recording_mode)
        if self.recording_mode == RECORDING_OFF:
            raise ValueError("off recording mode must not create a log file")
        directory = Path(log_dir)
        directory.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        self.path = directory / f"session_{stamp}.jsonl"
        self._stream = self.path.open("x", encoding="utf-8", newline="\n")
        self._flush_every = max(1, int(flush_every))
        self._pending = 0
        self._closed = False
        self._started_perf = time.perf_counter()
        self._write(
            {
                "type": "session_start",
                "schema_version": 2,
                "timestamp": _utc_iso_now(),
                "recording_mode": self.recording_mode,
                "replay_input_field": "wuji_points_21_m",
                "replay_entrypoint": "MediaPipe/Wuji -> TopoRetarget",
                **dict(session_metadata),
            },
            flush=True,
        )

    def _write(self, record: Mapping[str, Any], *, flush: bool = False) -> None:
        if self._closed:
            return
        payload = _json_safe(dict(record))
        self._stream.write(
            json.dumps(payload, ensure_ascii=False, separators=(",", ":")) + "\n"
        )
        self._pending += 1
        if flush or self._pending >= self._flush_every:
            self._stream.flush()
            self._pending = 0

    def record_frame(self, frame_record: Mapping[str, Any]) -> None:
        selected = select_frame_fields(frame_record, self.recording_mode)
        self._write(
            {
                "type": "frame",
                "timestamp": _utc_iso_now(),
                "session_elapsed_s": round(
                    time.perf_counter() - self._started_perf, 6
                ),
                **selected,
            }
        )

    def close(self, *, reason: str = "stopped", error: str | None = None) -> None:
        if self._closed:
            return
        self._write(
            {
                "type": "session_end",
                "timestamp": _utc_iso_now(),
                "session_elapsed_s": round(
                    time.perf_counter() - self._started_perf, 6
                ),
                "reason": str(reason),
                "error": error,
            },
            flush=True,
        )
        self._closed = True
        self._stream.close()

    def __enter__(self) -> "SessionJsonlLogger":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close(
            reason="error" if exc is not None else "stopped",
            error=None if exc is None else f"{type(exc).__name__}: {exc}",
        )


__all__ = [
    "MEDIAPIPE_REPLAY_FIELDS",
    "RECORDING_FULL",
    "RECORDING_MEDIAPIPE",
    "RECORDING_MODES",
    "RECORDING_OFF",
    "SessionJsonlLogger",
    "normalise_recording_mode",
    "select_frame_fields",
]
