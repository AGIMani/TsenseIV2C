"""Geometry boundary helpers shared by the light viewer and its tests."""

from __future__ import annotations

import math
from typing import Any, Mapping

import numpy as np

from utils.mediapipe_frame import apply_mediapipe_transformations


THUMB_ROTA_SOLVER_UPPER_RAD = 1.57


def prepare_topo_points(points: np.ndarray, hand_side: str) -> np.ndarray:
    """Match the main application's MediaPipe → Topo wrist-frame boundary."""
    arr = np.asarray(points, dtype=np.float64)
    if arr.shape != (21, 3):
        raise ValueError(f"expected points shape (21, 3), got {arr.shape}")
    side = str(hand_side).strip().lower()
    if side not in {"left", "right"}:
        raise ValueError(f"hand_side must be left/right, got {hand_side!r}")
    transformed = apply_mediapipe_transformations(arr, hand_type=side)
    if transformed.shape != (21, 3) or np.any(~np.isfinite(transformed)):
        raise ValueError("MediaPipe wrist-frame transform produced invalid points")
    return np.asarray(transformed, dtype=np.float64)


def landmarks_ok_for_retarget(points: np.ndarray) -> bool:
    """Reject collapsed/outlier 21-point frames before they seed TopoRetarget.

    These are the same geometric gates used by the main SkeletonTracker.  A
    single bad first frame otherwise becomes the solver warm-start and can
    hold an anatomically wrong local minimum for many subsequent frames.
    """
    try:
        pts = np.asarray(points, dtype=np.float64)
        if pts.shape != (21, 3) or not np.isfinite(pts).all():
            return False
        palm_width = float(np.linalg.norm(pts[5] - pts[17]))
        if palm_width < 0.025 or palm_width > 0.28:
            return False
        tip_lengths = [
            float(np.linalg.norm(pts[index] - pts[0]))
            for index in (4, 8, 12, 16, 20)
        ]
        if max(tip_lengths) < 0.045:
            return False
        if float(np.linalg.norm(pts[8] - pts[20])) < 0.02:
            return False
        return True
    except Exception:
        return False


def _palm_plane_signed_distance(points: np.ndarray, point_index: int) -> float:
    palm = np.asarray(points, dtype=np.float64)[[0, 5, 9, 13, 17]]
    center = np.mean(palm, axis=0)
    _, _, vh = np.linalg.svd(palm - center, full_matrices=False)
    normal = vh[-1]
    return float(np.dot(np.asarray(points[point_index], dtype=float) - center, normal))


def thumb_diagnostics(
    wuji_points: np.ndarray | None,
    topo_points: np.ndarray | None,
    joint_values: Mapping[str, float] | None,
) -> dict[str, Any]:
    """Build compact thumb-root diagnostics for each JSONL frame."""
    values = dict(joint_values or {})
    thumb_name = next(
        (name for name in values if str(name).endswith("thumb_rota_joint")),
        None,
    )
    thumb_rota = None if thumb_name is None else float(values[thumb_name])
    diagnostics: dict[str, Any] = {
        "thumb_rota_joint": thumb_name,
        "thumb_rota_rad": thumb_rota,
        "thumb_rota_deg": (
            None if thumb_rota is None else math.degrees(thumb_rota)
        ),
        "thumb_rota_at_solver_upper_limit": bool(
            thumb_rota is not None
            and thumb_rota >= THUMB_ROTA_SOLVER_UPPER_RAD - 1.0e-3
        ),
        "solver_upper_limit_rad": THUMB_ROTA_SOLVER_UPPER_RAD,
    }

    if wuji_points is not None:
        wuji = np.asarray(wuji_points, dtype=np.float64)
        if wuji.shape == (21, 3):
            diagnostics.update(
                {
                    "wuji_wrist_xyz_m": wuji[0].tolist(),
                    "wuji_cmc_xyz_m": wuji[1].tolist(),
                    "wuji_thumb_mcp_xyz_m": wuji[2].tolist(),
                    "wuji_wrist_to_cmc_m": float(np.linalg.norm(wuji[1] - wuji[0])),
                    "wuji_cmc_to_mcp_m": float(np.linalg.norm(wuji[2] - wuji[1])),
                    "wuji_cmc_palm_plane_signed_distance_m": (
                        _palm_plane_signed_distance(wuji, 1)
                    ),
                }
            )
    if topo_points is not None:
        topo = np.asarray(topo_points, dtype=np.float64)
        if topo.shape == (21, 3):
            diagnostics.update(
                {
                    "topo_cmc_xyz_m": topo[1].tolist(),
                    "topo_wrist_to_cmc_vector_m": (topo[1] - topo[0]).tolist(),
                    "topo_cmc_to_mcp_vector_m": (topo[2] - topo[1]).tolist(),
                }
            )
    return diagnostics


__all__ = [
    "THUMB_ROTA_SOLVER_UPPER_RAD",
    "landmarks_ok_for_retarget",
    "prepare_topo_points",
    "thumb_diagnostics",
]
