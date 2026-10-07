"""Wuji MediaPipe Tasks 21-point recognition and pre-retarget preprocessing.

The output boundary intentionally matches ``wuji-retargeting``'s camera input
devices: the returned ``(21, 3)`` array is what those devices pass directly to
``Retargeter.retarget``.  With a webcam it uses MediaPipe Tasks world landmarks,
matching the main project's no-depth-camera path.  The old normalized-Z
heuristic remains only as a fallback when a supplied result has no world
landmarks.  Wrist centering, global hand-scale normalization, per-segment length
correction, optional last-valid-frame holding, and configurable One Euro
temporal smoothing remain unchanged.  It does not include the wrist/MANO
transform or any retarget optimizer/filter.

Typical use::

    tracker = WujiMediaPipe21(hand_side="right")
    points = tracker.process_bgr(frame_bgr)
    if points is not None:
        # points.shape == (21, 3), ready for Wuji Retargeter.retarget(points)
        ...
    tracker.close()
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping, Optional

import numpy as np


WUJI_REFERENCE_SEGMENT_LENGTHS: dict[str, tuple[float, float, float]] = {
    "thumb": (0.0505, 0.0318, 0.0302),
    "index": (0.0418, 0.0243, 0.0223),
    "middle": (0.0489, 0.0289, 0.0227),
    "ring": (0.0422, 0.0274, 0.0227),
    "pinky": (0.0343, 0.0195, 0.0201),
}

WUJI_FINGER_INDICES: dict[str, tuple[int, int, int, int]] = {
    "thumb": (1, 2, 3, 4),
    "index": (5, 6, 7, 8),
    "middle": (9, 10, 11, 12),
    "ring": (13, 14, 15, 16),
    "pinky": (17, 18, 19, 20),
}


@dataclass(frozen=True)
class WujiMediaPipeConfig:
    """Parameters used by the light Wuji MediaPipe input."""

    z_scale: float = 2.5
    correct_segments: bool = True
    reference_wrist_to_mid_mcp: float = 0.09
    max_num_hands: int = 1
    # MediaPipe's handedness classifier assumes a horizontally mirrored
    # selfie image.  OpenCV VideoCapture supplies an unmirrored frame, so the
    # label must be swapped unless the caller explicitly mirrors its input.
    input_is_mirrored: bool = False
    # Optional empirical override for pipelines whose Tasks/model/camera
    # combination has been calibrated directly.  None preserves the official
    # mirror-derived behaviour above; False uses the raw Left/Right label.
    swap_handedness: Optional[bool] = None
    min_detection_confidence: float = 0.5
    min_tracking_confidence: float = 0.5
    hold_last_valid: bool = True
    smoothing_enabled: bool = True
    smoothing_min_cutoff: float = 1.5
    smoothing_beta: float = 0.25
    smoothing_derivative_cutoff: float = 1.0

    @classmethod
    def from_mapping(cls, config: Optional[Mapping[str, Any]]) -> "WujiMediaPipeConfig":
        """Build from Wuji YAML's ``video_input`` mapping."""
        cfg = dict(config or {})
        return cls(
            z_scale=float(cfg.get("z_scale", 2.5)),
            correct_segments=bool(cfg.get("correct_segments", True)),
            reference_wrist_to_mid_mcp=float(
                cfg.get("reference_wrist_to_mid_mcp", 0.09)
            ),
            max_num_hands=int(cfg.get("max_num_hands", 1)),
            input_is_mirrored=bool(cfg.get("input_is_mirrored", False)),
            swap_handedness=(
                None
                if cfg.get("swap_handedness") is None
                else bool(cfg.get("swap_handedness"))
            ),
            min_detection_confidence=float(
                cfg.get("min_detection_confidence", 0.5)
            ),
            min_tracking_confidence=float(
                cfg.get("min_tracking_confidence", 0.5)
            ),
            hold_last_valid=bool(cfg.get("hold_last_valid", True)),
            smoothing_enabled=bool(cfg.get("smoothing_enabled", True)),
            smoothing_min_cutoff=float(cfg.get("smoothing_min_cutoff", 1.5)),
            smoothing_beta=float(cfg.get("smoothing_beta", 0.25)),
            smoothing_derivative_cutoff=float(
                cfg.get("smoothing_derivative_cutoff", 1.0)
            ),
        )


@dataclass(frozen=True)
class WujiMediaPipeFrame:
    """One detector result at the exact pre-retarget boundary."""

    points: np.ndarray
    raw_normalized_points: np.ndarray
    normalized_points: np.ndarray
    detected_label: Optional[str]
    expected_label: str
    raw_world_points: Optional[np.ndarray] = None
    world_points: Optional[np.ndarray] = None
    depth_source: str = "normalized_z_fallback"
    used_fallback_hand: bool = False
    held_last_valid: bool = False

    def copy(self, *, held_last_valid: Optional[bool] = None) -> "WujiMediaPipeFrame":
        return WujiMediaPipeFrame(
            points=self.points.copy(),
            raw_normalized_points=self.raw_normalized_points.copy(),
            normalized_points=self.normalized_points.copy(),
            detected_label=self.detected_label,
            expected_label=self.expected_label,
            raw_world_points=(
                None if self.raw_world_points is None else self.raw_world_points.copy()
            ),
            world_points=None if self.world_points is None else self.world_points.copy(),
            depth_source=self.depth_source,
            used_fallback_hand=self.used_fallback_hand,
            held_last_valid=(
                self.held_last_valid
                if held_last_valid is None
                else bool(held_last_valid)
            ),
        )


def _validate_image_size(width: int, height: int) -> tuple[int, int]:
    width = int(width)
    height = int(height)
    if width <= 0 or height <= 0:
        raise ValueError(f"image dimensions must be positive, got {width}x{height}")
    return width, height


def normalized_to_wuji_pixel_points(
    normalized_points: np.ndarray,
    width: int,
    height: int,
    *,
    z_scale: float = 2.5,
) -> np.ndarray:
    """Apply Wuji's normalized landmark -> anisotropic pixel/Z scaling."""
    width, height = _validate_image_size(width, height)
    points = np.asarray(normalized_points, dtype=np.float32)
    if points.shape != (21, 3):
        raise ValueError(f"expected normalized points shape (21, 3), got {points.shape}")
    points = points.copy()
    points[:, 0] *= width
    points[:, 1] *= height
    points[:, 2] *= width * float(z_scale)
    return points


def correct_wuji_segment_lengths(
    wrist_relative_points: np.ndarray,
    segment_lengths: Mapping[
        str, tuple[float, float, float]
    ] = WUJI_REFERENCE_SEGMENT_LENGTHS,
) -> np.ndarray:
    """Replace each finger segment length while preserving detected directions."""
    points = np.asarray(wrist_relative_points, dtype=np.float32)
    if points.shape != (21, 3):
        raise ValueError(f"expected points shape (21, 3), got {points.shape}")
    corrected = points.copy()

    for finger, indices in WUJI_FINGER_INDICES.items():
        lengths = segment_lengths[finger]
        mcp_i, pip_i, dip_i, tip_i = indices
        base = corrected[mcp_i].copy()

        seg1 = points[pip_i] - points[mcp_i]
        seg1_len = np.linalg.norm(seg1)
        if seg1_len > 1e-6:
            corrected[pip_i] = base + (seg1 / seg1_len) * lengths[0]

        seg2 = points[dip_i] - points[pip_i]
        seg2_len = np.linalg.norm(seg2)
        if seg2_len > 1e-6:
            corrected[dip_i] = corrected[pip_i] + (seg2 / seg2_len) * lengths[1]

        seg3 = points[tip_i] - points[dip_i]
        seg3_len = np.linalg.norm(seg3)
        if seg3_len > 1e-6:
            corrected[tip_i] = corrected[dip_i] + (seg3 / seg3_len) * lengths[2]

    return corrected


def preprocess_wuji_pixel_points(
    pixel_points: np.ndarray,
    *,
    reference_wrist_to_mid_mcp: float = 0.09,
    correct_segments: bool = True,
) -> np.ndarray:
    """Apply Wuji wrist centering, global scale, and segment correction."""
    points = np.asarray(pixel_points, dtype=np.float32)
    if points.shape != (21, 3):
        raise ValueError(f"expected points shape (21, 3), got {points.shape}")

    points = points - points[0:1, :]
    wrist_to_mid_mcp = np.linalg.norm(points[9])
    # Preserve Wuji's degenerate behavior: return wrist-relative pixel values.
    if wrist_to_mid_mcp < 1e-6:
        return points

    points = points * (float(reference_wrist_to_mid_mcp) / wrist_to_mid_mcp)
    if correct_segments:
        points = correct_wuji_segment_lengths(points)
    return points


def preprocess_wuji_normalized_points(
    normalized_points: np.ndarray,
    width: int,
    height: int,
    *,
    config: Optional[WujiMediaPipeConfig] = None,
) -> np.ndarray:
    """Apply the unchanged legacy normalized-Z geometry fallback."""
    cfg = config or WujiMediaPipeConfig()
    pixel_points = normalized_to_wuji_pixel_points(
        normalized_points,
        width,
        height,
        z_scale=cfg.z_scale,
    )
    return preprocess_wuji_pixel_points(
        pixel_points,
        reference_wrist_to_mid_mcp=cfg.reference_wrist_to_mid_mcp,
        correct_segments=cfg.correct_segments,
    )


def preprocess_wuji_world_points(
    world_points: np.ndarray,
    *,
    config: Optional[WujiMediaPipeConfig] = None,
) -> np.ndarray:
    """Apply the existing Wuji scale/bone stages to metric Tasks landmarks."""
    cfg = config or WujiMediaPipeConfig()
    return preprocess_wuji_pixel_points(
        world_points,
        reference_wrist_to_mid_mcp=cfg.reference_wrist_to_mid_mcp,
        correct_segments=cfg.correct_segments,
    )


def _classification_label(handedness: Any) -> Optional[str]:
    """Read a label from either a Tasks category list or an injected result."""
    try:
        categories = getattr(handedness, "classification", handedness)
        if categories:
            category = categories[0]
            label = getattr(category, "category_name", None)
            if not label:
                label = getattr(category, "label", None)
            if label:
                return str(label)
    except Exception:
        pass
    return None


def _landmark_array(hand_landmarks: Any) -> np.ndarray:
    try:
        landmarks = getattr(hand_landmarks, "landmark", hand_landmarks)
        points = np.asarray(
            [[lm.x, lm.y, lm.z] for lm in landmarks],
            dtype=np.float32,
        )
    except Exception as exc:
        raise ValueError(f"invalid MediaPipe hand landmarks: {exc}") from exc
    if points.shape != (21, 3):
        raise ValueError(f"expected 21 MediaPipe landmarks, got {points.shape}")
    return points


def _result_collections(results: Any) -> tuple[Any, Any, Any]:
    """Return normalized landmarks, handedness, and world landmarks."""
    tasks_landmarks = getattr(results, "hand_landmarks", None)
    if tasks_landmarks is not None:
        return (
            tasks_landmarks,
            getattr(results, "handedness", None),
            getattr(results, "hand_world_landmarks", None),
        )
    # Keep process_results useful for simple injected detectors/tests.  The
    # production detector is always the Tasks HandLandmarker above.
    return (
        getattr(results, "multi_hand_landmarks", None),
        getattr(results, "multi_handedness", None),
        getattr(results, "multi_hand_world_landmarks", None),
    )


class _OneEuroLandmarkFilter:
    """Adaptive low-pass filter for one set of 21 MediaPipe landmarks."""

    def __init__(
        self,
        *,
        min_cutoff: float,
        beta: float,
        derivative_cutoff: float,
    ) -> None:
        self.min_cutoff = max(float(min_cutoff), 1e-4)
        self.beta = max(float(beta), 0.0)
        self.derivative_cutoff = max(float(derivative_cutoff), 1e-4)
        self.reset()

    @staticmethod
    def _alpha(cutoff: np.ndarray | float, dt: float) -> np.ndarray:
        cutoff_arr = np.asarray(cutoff, dtype=np.float32)
        tau = 1.0 / (2.0 * math.pi * cutoff_arr)
        return 1.0 / (1.0 + tau / dt)

    def reset(self) -> None:
        self._last_time: Optional[float] = None
        self._last_raw: Optional[np.ndarray] = None
        self._last_filtered: Optional[np.ndarray] = None
        self._last_derivative: Optional[np.ndarray] = None

    def filter(self, points: np.ndarray, timestamp_s: float) -> np.ndarray:
        value = np.asarray(points, dtype=np.float32)
        if value.shape != (21, 3):
            raise ValueError(f"expected points shape (21, 3), got {value.shape}")
        if self._last_filtered is None or self._last_raw is None or self._last_time is None:
            self._last_time = float(timestamp_s)
            self._last_raw = value.copy()
            self._last_filtered = value.copy()
            self._last_derivative = np.zeros_like(value)
            return value.copy()

        dt = min(max(float(timestamp_s) - self._last_time, 1.0 / 240.0), 0.2)
        derivative = (value - self._last_raw) / dt
        derivative_alpha = self._alpha(self.derivative_cutoff, dt)
        previous_derivative = (
            self._last_derivative
            if self._last_derivative is not None
            else np.zeros_like(value)
        )
        derivative_hat = (
            derivative_alpha * derivative
            + (1.0 - derivative_alpha) * previous_derivative
        )
        cutoff = self.min_cutoff + self.beta * np.abs(derivative_hat)
        value_alpha = self._alpha(cutoff, dt)
        filtered = (
            value_alpha * value
            + (1.0 - value_alpha) * self._last_filtered
        ).astype(np.float32, copy=False)

        self._last_time = float(timestamp_s)
        self._last_raw = value.copy()
        self._last_filtered = filtered.copy()
        self._last_derivative = derivative_hat.copy()
        return filtered.copy()


class WujiMediaPipe21:
    """Stateful BGR-frame detector that emits Wuji-ready 21-point positions."""

    def __init__(
        self,
        hand_side: str = "right",
        video_config: Optional[Mapping[str, Any]] = None,
        *,
        config: Optional[WujiMediaPipeConfig] = None,
        detector: Any = None,
        model_path: str | Path | None = None,
    ) -> None:
        side = str(hand_side).strip().lower()
        if side not in {"left", "right"}:
            raise ValueError(f"hand_side must be 'left' or 'right', got {hand_side!r}")
        if config is not None and video_config is not None:
            raise ValueError("pass either config or video_config, not both")

        self.hand_side = side
        self.config = config or WujiMediaPipeConfig.from_mapping(video_config)
        physical_label = "Right" if side == "right" else "Left"
        should_swap = self.config.swap_handedness
        if should_swap is None:
            should_swap = not self.config.input_is_mirrored
        self.swap_handedness = bool(should_swap)
        if self.swap_handedness:
            self.expected_mp_label = "Left" if physical_label == "Right" else "Right"
        else:
            self.expected_mp_label = physical_label
        self._lock = threading.Lock()
        self._last_valid: Optional[WujiMediaPipeFrame] = None
        self._last_selection_signature: Optional[tuple[Optional[str], bool]] = None
        self._closed = False
        self._owns_detector = detector is None
        self._mp = None
        self._last_timestamp_ms = -1

        if detector is None:
            try:
                import mediapipe as mp
            except ImportError as exc:
                raise RuntimeError(
                    "WujiMediaPipe21 requires mediapipe; install the project requirements"
                ) from exc
            resolved_model = (
                Path(model_path).expanduser().resolve()
                if model_path is not None
                else Path(__file__).resolve().parents[2] / "hand_landmarker.task"
            )
            if not resolved_model.is_file():
                raise FileNotFoundError(
                    f"MediaPipe HandLandmarker model not found: {resolved_model}"
                )
            base_options = mp.tasks.BaseOptions(
                model_asset_path=str(resolved_model),
                delegate=mp.tasks.BaseOptions.Delegate.CPU,
            )
            options = mp.tasks.vision.HandLandmarkerOptions(
                base_options=base_options,
                running_mode=mp.tasks.vision.RunningMode.VIDEO,
                num_hands=self.config.max_num_hands,
                min_hand_detection_confidence=self.config.min_detection_confidence,
                min_hand_presence_confidence=self.config.min_detection_confidence,
                min_tracking_confidence=self.config.min_tracking_confidence,
            )
            detector = mp.tasks.vision.HandLandmarker.create_from_options(options)
            self._mp = mp
        self._detector = detector
        self._uses_tasks_api = callable(getattr(detector, "detect_for_video", None))
        self._landmark_filter = _OneEuroLandmarkFilter(
            min_cutoff=self.config.smoothing_min_cutoff,
            beta=self.config.smoothing_beta,
            derivative_cutoff=self.config.smoothing_derivative_cutoff,
        )
        self._world_landmark_filter = _OneEuroLandmarkFilter(
            min_cutoff=self.config.smoothing_min_cutoff,
            beta=self.config.smoothing_beta,
            derivative_cutoff=self.config.smoothing_derivative_cutoff,
        )

    def process_bgr(self, frame_bgr: np.ndarray) -> Optional[np.ndarray]:
        """Return ``(21,3)`` points ready for Wuji ``Retargeter.retarget``."""
        result = self.process_bgr_detailed(frame_bgr)
        return None if result is None else result.points.copy()

    def process_bgr_detailed(
        self,
        frame_bgr: np.ndarray,
    ) -> Optional[WujiMediaPipeFrame]:
        """Detect one BGR frame and retain provenance metadata for diagnostics."""
        frame = np.asarray(frame_bgr)
        if frame.ndim != 3 or frame.shape[2] != 3:
            raise ValueError(f"expected BGR frame shape (H,W,3), got {frame.shape}")
        if frame.size == 0:
            raise ValueError("BGR frame is empty")
        try:
            import cv2
        except ImportError as exc:
            raise RuntimeError("WujiMediaPipe21 requires opencv-python") from exc

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        rgb = np.ascontiguousarray(rgb)
        with self._lock:
            if self._closed:
                raise RuntimeError("WujiMediaPipe21 is closed")
            if self._uses_tasks_api:
                if self._mp is None:
                    try:
                        import mediapipe as mp
                    except ImportError as exc:
                        raise RuntimeError(
                            "MediaPipe Tasks detector requires mediapipe"
                        ) from exc
                    self._mp = mp
                timestamp_ms = int(time.perf_counter() * 1000.0)
                self._last_timestamp_ms = max(
                    timestamp_ms,
                    self._last_timestamp_ms + 1,
                )
                mp_image = self._mp.Image(
                    image_format=self._mp.ImageFormat.SRGB,
                    data=rgb,
                )
                results = self._detector.detect_for_video(
                    mp_image,
                    self._last_timestamp_ms,
                )
            else:
                # Lightweight injected detectors used by unit tests may expose
                # a process(ndarray) method; production never constructs this path.
                results = self._detector.process(rgb)
            return self._process_results_locked(results, frame.shape[1], frame.shape[0])

    def process_results(
        self,
        results: Any,
        width: int,
        height: int,
    ) -> Optional[WujiMediaPipeFrame]:
        """Process an existing HandLandmarker-compatible result."""
        with self._lock:
            if self._closed:
                raise RuntimeError("WujiMediaPipe21 is closed")
            return self._process_results_locked(results, width, height)

    def _process_results_locked(
        self,
        results: Any,
        width: int,
        height: int,
    ) -> Optional[WujiMediaPipeFrame]:
        width, height = _validate_image_size(width, height)
        hand_landmarks, handedness, world_landmarks = _result_collections(results)

        selected = None
        selected_index: Optional[int] = None
        selected_label: Optional[str] = None
        used_fallback = False
        if hand_landmarks and handedness:
            for index, (landmarks, hand_class) in enumerate(
                zip(hand_landmarks, handedness)
            ):
                label = _classification_label(hand_class)
                if label == self.expected_mp_label:
                    selected = landmarks
                    selected_index = index
                    selected_label = label
                    break
            if selected is None and hand_landmarks:
                # Exact Wuji fallback: use the first detection even if hand side differs.
                selected = hand_landmarks[0]
                selected_index = 0
                selected_label = _classification_label(handedness[0]) if handedness else None
                used_fallback = True
        elif hand_landmarks:
            selected = hand_landmarks[0]
            selected_index = 0
            used_fallback = True

        if selected is None:
            if self.config.hold_last_valid and self._last_valid is not None:
                return self._last_valid.copy(held_last_valid=True)
            return None

        raw_normalized = _landmark_array(selected)
        normalized = raw_normalized.copy()
        raw_world: Optional[np.ndarray] = None
        world: Optional[np.ndarray] = None
        if (
            selected_index is not None
            and world_landmarks
            and selected_index < len(world_landmarks)
        ):
            try:
                candidate = _landmark_array(world_landmarks[selected_index])
                if np.all(np.isfinite(candidate)):
                    raw_world = candidate
                    world = candidate.copy()
            except ValueError:
                # A malformed/missing world result falls back to the unchanged
                # normalized-z path instead of discarding an otherwise valid hand.
                pass

        selection_signature = (selected_label, used_fallback)
        if (
            self.config.max_num_hands > 1
            and self._last_selection_signature is not None
            and selection_signature != self._last_selection_signature
        ):
            # Never blend coordinates from two hands or from fallback/expected selection.
            self._landmark_filter.reset()
            self._world_landmark_filter.reset()
        self._last_selection_signature = selection_signature
        timestamp_s = time.perf_counter()
        if self.config.smoothing_enabled:
            normalized = self._landmark_filter.filter(normalized, timestamp_s)
            if world is not None:
                world = self._world_landmark_filter.filter(world, timestamp_s)

        if world is not None:
            points = preprocess_wuji_world_points(world, config=self.config)
            depth_source = "mediapipe_world"
        else:
            points = preprocess_wuji_normalized_points(
                normalized,
                width,
                height,
                config=self.config,
            )
            depth_source = "normalized_z_fallback"
        frame = WujiMediaPipeFrame(
            points=points,
            raw_normalized_points=raw_normalized,
            normalized_points=normalized,
            detected_label=selected_label,
            expected_label=self.expected_mp_label,
            raw_world_points=raw_world,
            world_points=world,
            depth_source=depth_source,
            used_fallback_hand=used_fallback,
            held_last_valid=False,
        )
        self._last_valid = frame.copy()
        return frame.copy()

    @property
    def last_points(self) -> Optional[np.ndarray]:
        with self._lock:
            return None if self._last_valid is None else self._last_valid.points.copy()

    def reset(self) -> None:
        """Clear Wuji's last-valid-frame hold state."""
        with self._lock:
            self._last_valid = None
            self._last_selection_signature = None
            self._landmark_filter.reset()
            self._world_landmark_filter.reset()
            self._last_timestamp_ms = -1

    def close(self) -> None:
        with self._lock:
            if self._closed:
                return
            self._closed = True
            detector = self._detector
            self._detector = None
        if self._owns_detector and detector is not None:
            close = getattr(detector, "close", None)
            if callable(close):
                close()

    def __enter__(self) -> "WujiMediaPipe21":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()


__all__ = [
    "WUJI_FINGER_INDICES",
    "WUJI_REFERENCE_SEGMENT_LENGTHS",
    "WujiMediaPipe21",
    "WujiMediaPipeConfig",
    "WujiMediaPipeFrame",
    "correct_wuji_segment_lengths",
    "normalized_to_wuji_pixel_points",
    "preprocess_wuji_normalized_points",
    "preprocess_wuji_pixel_points",
    "preprocess_wuji_world_points",
]
