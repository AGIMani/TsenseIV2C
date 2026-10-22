"""Frame-aligned JSONL skeleton data and CSV angle comparisons."""
from __future__ import annotations

import csv
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import time
from uuid import uuid4

import numpy as np

from .skeleton_angles import degrees
from .pipeline import observation_depth_source

DEFAULT_LOG_DIR = Path(__file__).resolve().parent / "logs"


def json_safe(value):
    if isinstance(value, Path):
        return str(value)
    if isinstance(value, np.ndarray):
        return json_safe(value.tolist())
    if isinstance(value, np.generic):
        return json_safe(value.item())
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


class SessionRecorder:
    def __init__(self, directory, pipeline, *, camera_source, smoothing, input_source="mediapipe"):
        directory = Path(directory).expanduser().resolve()
        directory.mkdir(parents=True, exist_ok=True)
        stem = f"session_{datetime.now():%Y%m%d_%H%M%S_%f}_{uuid4().hex[:6]}"
        self.path = directory / (stem + ".jsonl")
        self.csv_path = directory / (stem + ".angles.csv")
        self._json = self._csv_stream = None
        self._closed = False
        self.frame_count = 0
        self.started = time.perf_counter()
        session = pipeline.session
        self.hand_side = session.hand_side
        self.input_source = input_source
        self.model_id = session.profile.model_id
        self.joints = list(session.limits)
        self.columns = [(group, unit, key) for group, keys in (
            ("retarget", self.joints), ("direct", self.joints), ("mink", self.joints))
            for unit in ("rad", "deg") for key in keys]
        try:
            self._json = self.path.open("x", encoding="utf-8", newline="\n")
            self._csv_stream = self.csv_path.open("x", encoding="utf-8-sig", newline="")
            self._csv = csv.writer(self._csv_stream)
            self._csv.writerow(["frame_id", "capture_timestamp_unix_ns", "hand_side", "detected_label",
                                "depth_source", "tracking", "retarget_success", "held_last_valid",
                                "used_fallback_hand", "message"] +
                               [f"{group}.{key}.{unit}" for group, unit, key in self.columns])
            calibration_path = session.profile.path("keypoints", session.hand_side)
            calibration = calibration_path.read_bytes()
            urdf_path = session.profile.path("urdf", session.hand_side)
            self._write({
                "type": "session_start", "schema_version": 1,
                "schema_name": f"{session.profile.model_id}_angle_comparison",
                "model_id": session.profile.model_id, "hand_side": session.hand_side,
                "camera_source": camera_source, "input_source": input_source, "smoothing_enabled": smoothing,
                "collision_mode": pipeline.collision_mode,
                "retarget_config": asdict(session.retargeter.config),
                "urdf_path": str(urdf_path), "urdf_sha256": hashlib.sha256(urdf_path.read_bytes()).hexdigest(),
                "calibration_path": str(calibration_path),
                "calibration_sha256": hashlib.sha256(calibration).hexdigest(),
                "calibration_yaml": calibration.decode("utf-8"),
                "joint_names": self.joints, "joint_stages": ["retarget", "direct", "ruckig", "mink", "sdk"],
                "actuation": pipeline.stroke_mapper.description(),
                "retarget_angles": "URDF radians/degrees, after joint limits, BEFORE collision filter",
                "direct_angles": "Actual direct-mode output, absent when direct mode is disabled",
                "mink_angles": "Actual Mink projection output, absent when Mink is disabled",
                "coordinate_units": {"world_points": "meter", "topo_input_points": "meter",
                                     "normalized_points": "MediaPipe image coordinates only; null for WebXR"},
            })
            self._csv_stream.flush()
        except Exception:
            for stream in (self._json, self._csv_stream):
                if stream is not None:
                    stream.close()
            raise

    def _write(self, record):
        record = {"timestamp": datetime.now(timezone.utc).isoformat(timespec="milliseconds"), **record}
        self._json.write(json.dumps(json_safe(record), ensure_ascii=False, allow_nan=False) + "\n")
        self._json.flush()

    def record_frame(self, frame_id, observation, result, *, capture_timestamp_ns, camera_size,
                     input_source=None, source_metadata=None, raw_webxr=None, mediapipe_filter_enabled=None):
        def attr(name):
            return None if observation is None else getattr(observation, name, None)
        world = attr("world_points")
        source = world if world is not None else attr("points")
        retarget = result.retarget_pose
        depth = observation_depth_source(observation)
        input_source = input_source or attr("source") or self.input_source
        stages = {"retarget": retarget, "direct": result.direct_pose, "mink": result.mink_pose}
        groups = {f"{stage}_{unit}": values if unit == "rad" else degrees(values)
                  for stage, values in stages.items() for unit in ("rad", "deg")}
        elapsed = time.perf_counter() - self.started
        record = {
            "type": "frame", "frame_id": frame_id, "hand_side": self.hand_side,
            "input_source": input_source,
            "mediapipe_filter_enabled": (attr("mediapipe_filter_enabled")
                                         if mediapipe_filter_enabled is None else mediapipe_filter_enabled),
            "source_metadata": source_metadata if source_metadata is not None else attr("source_metadata"),
            "raw_webxr": raw_webxr if raw_webxr is not None else attr("raw_webxr"),
            "raw_wuji": (source_metadata or attr("source_metadata") or {}).get("raw_wuji"),
            "raw_leapmotion": (source_metadata or attr("source_metadata") or {}).get("raw_leapmotion"),
            "capture_timestamp_unix_ns": capture_timestamp_ns,
            "session_elapsed_s": elapsed,
            "camera_size": camera_size, "detected": observation is not None,
            "detected_label": attr("detected_label"), "expected_label": attr("expected_label"),
            "held_last_valid": bool(attr("held_last_valid")),
            "used_fallback_hand": bool(attr("used_fallback_hand")),
            "tracking": result.tracking, "retarget_success": retarget is not None,
            "message": result.message, "depth_source": depth,
            "raw_normalized_points_21": attr("raw_normalized_points") if input_source in ("mediapipe", "realsense_rgbd") else None,
            "smoothed_normalized_points_21": attr("normalized_points") if input_source in ("mediapipe", "realsense_rgbd") else None,
            "raw_world_points_21_m": attr("raw_world_points"),
            "smoothed_world_points_21_m": world,
            "source_points_21_m": source, "topo_input_points_21_m": result.points,
            "solve_ms": result.solve_ms, "contacts": result.contacts,
            "collision_diagnostics": getattr(result, "collision_diagnostics", {}),
            "actuation": getattr(result, "actuation", {}),
        }
        self._write(record)
        for stage, values in stages.items():
            if values:
                self._write({"type": "joint_angles", "stage": stage, "joints": values,
                             "frame_id": frame_id, "model_id": self.model_id, "hand_side": self.hand_side,
                             "t": elapsed, "capture_timestamp_unix_ns": capture_timestamp_ns})
        self._csv.writerow([frame_id, capture_timestamp_ns, self.hand_side, record["detected_label"],
                            depth, result.tracking, retarget is not None, record["held_last_valid"],
                            record["used_fallback_hand"], result.message] +
                           [(groups[f"{group}_{unit}"] or {}).get(key) for group, unit, key in self.columns])
        self._csv_stream.flush()
        self.frame_count += 1

    def close(self, *, reason="stopped", error=None):
        if self._closed:
            return
        self._closed = True
        try:
            self._write({"type": "session_end", "frame_count": self.frame_count,
                         "reason": reason, "error": error})
        finally:
            self._json.close()
            self._csv_stream.close()
