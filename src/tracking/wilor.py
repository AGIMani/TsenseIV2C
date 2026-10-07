"""WiLoR hand 3D pose backend (replaces MediaPipe as default tracker).

Uses vendored ``external/wilor_mini`` (fallback: pip ``wilor_mini``).
Outputs MediaPipe-compatible 21 landmarks for skeleton tracker / topo retarget.

MANO / WiLoR 21 joints follow the same MediaPipe Hand order:
  0 wrist; 1-4 thumb; 5-8 index; 9-12 middle; 13-16 ring; 17-20 pinky.
"""

from __future__ import annotations

import sys
import threading
import traceback
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np

from utils.log_control import emit as log_emit
from utils.tracker_config import WILOR_DEVICE, WILOR_DTYPE, WILOR_PREFER_LEFT

# Prefer project-vendored WiLoR over site-packages.
_EXTERNAL = Path(__file__).resolve().parents[2] / "external"
if _EXTERNAL.is_dir() and str(_EXTERNAL) not in sys.path:
    sys.path.insert(0, str(_EXTERNAL))


def _log(msg: str, tag: str = "WiLoR"):
    log_emit(msg, tag)


class _Lm:
    """MediaPipe Landmark–compatible container."""

    __slots__ = ("x", "y", "z")

    def __init__(self, x: float, y: float, z: float):
        self.x, self.y, self.z = float(x), float(y), float(z)


class WiLoRBackend:
    """Lazy-init WiLoR-mini pipeline singleton."""

    def __init__(self) -> None:
        self._pipe = None
        self._initialized = False
        self._init_error = ""
        self._lock = threading.Lock()
        # Stick to last chosen chirality to avoid L/R flip jitter.
        self._last_is_right: Optional[int] = None
        self._hold_norm: Optional[List[_Lm]] = None
        self._hold_world: Optional[List[_Lm]] = None
        self._hold_misses = 0
        self._hold_max_misses = 6  # reuse last good pose for a few blank YOLO frames

    @property
    def is_available(self) -> bool:
        return self._initialized and self._pipe is not None

    @property
    def init_error(self) -> str:
        return self._init_error

    def init(self, device: str = "", dtype: str = "") -> bool:
        with self._lock:
            if self._initialized and self._pipe is not None:
                return True
            self._initialized = False
            self._pipe = None
            self._init_error = ""
            try:
                import torch

                # PyTorch>=2.6 默认 weights_only=True，WiLoR/YOLO ckpt 需 False 才能反序列化。
                if not getattr(torch.load, "_wilor_weights_only_patched", False):
                    _orig_torch_load = torch.load

                    def _torch_load_compat(*args, **kwargs):
                        kwargs.setdefault("weights_only", False)
                        return _orig_torch_load(*args, **kwargs)

                    _torch_load_compat._wilor_weights_only_patched = True  # type: ignore[attr-defined]
                    torch.load = _torch_load_compat  # type: ignore[assignment]

                from wilor_mini.pipelines.wilor_hand_pose3d_estimation_pipeline import (
                    WiLorHandPose3dEstimationPipeline,
                )

                device_s = (device or WILOR_DEVICE or "").strip().lower()
                if not device_s:
                    device_s = "cuda" if torch.cuda.is_available() else "cpu"
                if device_s.startswith("cuda") and not torch.cuda.is_available():
                    device_s = "cpu"
                torch_device = torch.device(device_s)

                dtype_s = (dtype or WILOR_DTYPE or "float16").strip().lower()
                torch_dtype = torch.float16 if dtype_s in ("float16", "fp16", "half") else torch.float32
                if torch_device.type == "cpu" and torch_dtype == torch.float16:
                    # CPU often lacks fp16 kernels for this stack
                    torch_dtype = torch.float32

                _log(f"加载 WiLoR-mini pipeline device={torch_device} dtype={torch_dtype}")
                self._pipe = WiLorHandPose3dEstimationPipeline(
                    device=torch_device,
                    dtype=torch_dtype,
                    verbose=False,
                )
                self._initialized = True
                _log("WiLoR 初始化成功")
                return True
            except Exception as exc:  # noqa: BLE001
                self._init_error = f"{exc}\n{traceback.format_exc()}"
                _log(f"WiLoR 初始化失败: {exc}")
                return False

    def reset_tracking(self) -> None:
        self._last_is_right = None
        self._hold_norm = None
        self._hold_world = None
        self._hold_misses = 0

    def _bbox_area(self, out: dict) -> float:
        box = out.get("hand_bbox")
        if not box or len(box) < 4:
            return 0.0
        return max(0.0, float(box[2] - box[0])) * max(0.0, float(box[3] - box[1]))

    def _pick_hand(self, outputs: list, prefer_left: bool) -> Optional[dict]:
        """Prefer sticky chirality, then preferred side, then largest bbox."""
        candidates = [o for o in outputs if "wilor_preds" in o]
        if not candidates:
            return None

        def side_score(out: dict) -> tuple:
            is_right = int(round(float(out.get("is_right", 1))))
            sticky = 2 if (self._last_is_right is not None and is_right == self._last_is_right) else 0
            prefer = 1 if ((prefer_left and is_right == 0) or ((not prefer_left) and is_right == 1)) else 0
            return (sticky, prefer, self._bbox_area(out))

        return max(candidates, key=side_score)

    def infer(
        self,
        bgr: np.ndarray,
        *,
        prefer_left: Optional[bool] = None,
    ) -> Tuple[Optional[List[_Lm]], Optional[List[_Lm]], int]:
        """Run detection+reconstruction.

        Returns (norm_landmarks, world_landmarks, hand_count).
        norm: image-normalized [0,1]; world: wrist-relative meters (pred_keypoints_3d).
        """
        if not self.is_available and not self.init():
            return None, None, 0
        assert self._pipe is not None

        if bgr is None or bgr.size == 0:
            return None, None, 0
        h, w = bgr.shape[:2]
        if h < 2 or w < 2:
            return None, None, 0

        # Ultralytics YOLO + WiLoR demo use BGR frames from cv2
        try:
            # Slightly lower conf reduces intermittent empty detections (main jitter source).
            outputs = self._pipe.predict(bgr, hand_conf=0.25, verbose=False)
        except Exception as exc:  # noqa: BLE001
            _log(f"infer 失败: {exc}")
            return self._held_or_none(0)

        if not outputs:
            return self._held_or_none(0)

        prefer_left = WILOR_PREFER_LEFT if prefer_left is None else bool(prefer_left)
        chosen = self._pick_hand(outputs, prefer_left)
        if chosen is None or "wilor_preds" not in chosen:
            return self._held_or_none(len(outputs))

        preds = chosen["wilor_preds"]
        k2d = np.asarray(preds["pred_keypoints_2d"], dtype=np.float64)
        k3d = np.asarray(preds["pred_keypoints_3d"], dtype=np.float64)
        if k2d.ndim == 3:
            k2d = k2d[0]
        if k3d.ndim == 3:
            k3d = k3d[0]
        if k2d.shape[0] < 21 or k3d.shape[0] < 21:
            _log(f"关键点数不足: 2d={k2d.shape} 3d={k3d.shape}")
            return self._held_or_none(len(outputs))

        # Left-hand flip already applied inside WiLoR pipeline for is_right==0
        norm: List[_Lm] = []
        world: List[_Lm] = []
        for i in range(21):
            x_px, y_px = float(k2d[i, 0]), float(k2d[i, 1])
            norm.append(_Lm(x_px / max(w, 1), y_px / max(h, 1), 0.0))
            world.append(_Lm(float(k3d[i, 0]), float(k3d[i, 1]), float(k3d[i, 2])))

        # Wrist-relative world (retarget / topo expect wrist origin)
        wx, wy, wz = world[0].x, world[0].y, world[0].z
        world_arr = np.array([[p.x - wx, p.y - wy, p.z - wz] for p in world], dtype=np.float64)
        # WiLoR MANO coords → MediaPipe wrist frame (required by topo retarget)
        from utils.mediapipe_frame import apply_mediapipe_transformations

        hand_side = "left" if WILOR_PREFER_LEFT else "right"
        world_arr = apply_mediapipe_transformations(world_arr, hand_type=hand_side)
        world = [_Lm(float(x), float(y), float(z)) for x, y, z in world_arr]

        self._last_is_right = int(round(float(chosen.get("is_right", 1))))
        self._hold_norm = norm
        self._hold_world = world
        self._hold_misses = 0
        return norm, world, len(outputs)

    def _held_or_none(
        self, hand_count: int
    ) -> Tuple[Optional[List[_Lm]], Optional[List[_Lm]], int]:
        if self._hold_norm is not None and self._hold_world is not None:
            self._hold_misses += 1
            if self._hold_misses <= self._hold_max_misses:
                return self._hold_norm, self._hold_world, max(1, hand_count)
        self._hold_norm = None
        self._hold_world = None
        self._hold_misses = 0
        return None, None, hand_count


_backend: Optional[WiLoRBackend] = None


def get_wilor_backend() -> WiLoRBackend:
    global _backend
    if _backend is None:
        _backend = WiLoRBackend()
    return _backend
