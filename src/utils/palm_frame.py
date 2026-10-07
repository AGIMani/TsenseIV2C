"""Palm-local frame from 21 MediaPipe-compatible landmarks."""

from __future__ import annotations

import numpy as np


def normalize(v: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(v)
    return v / norm if norm > 1e-10 else np.zeros_like(v)


def _pt(landmarks_3d, idx: int) -> np.ndarray:
    p = landmarks_3d[idx]
    return np.array(p[:3], dtype=np.float64)


def compute_palm_frame(landmarks_3d):
    """Build palm-local frame aligned with URDF base_link convention.

    origin = wrist (0)
    Z = wrist -> middle MCP
    X = palm normal (outward)
    Y = X x Z
    """
    origin = _pt(landmarks_3d, 0)
    v_mid = _pt(landmarks_3d, 9) - origin
    v_idx = _pt(landmarks_3d, 5) - origin
    v_pky = _pt(landmarks_3d, 17) - origin

    normal = np.cross(v_idx, v_pky)
    n = np.linalg.norm(normal)
    if n > 1e-8:
        normal /= n
    else:
        normal = np.array([0.0, 0.0, 1.0], dtype=np.float64)

    ax_z = normalize(v_mid)
    ax_x = normal
    ax_y = normalize(np.cross(ax_x, ax_z))
    return origin, ax_x, ax_y, ax_z
