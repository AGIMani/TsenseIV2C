"""MediaPipe wrist-frame conversion (MANO / WiLoR → topo retarget input)."""

from __future__ import annotations

import numpy as np

OPERATOR2MANO_RIGHT = np.array(
    [
        [0, 0, -1],
        [-1, 0, 0],
        [0, 1, 0],
    ],
    dtype=np.float64,
)

OPERATOR2MANO_LEFT = np.array(
    [
        [0, 0, -1],
        [1, 0, 0],
        [0, -1, 0],
    ],
    dtype=np.float64,
)


def estimate_frame_from_hand_points(keypoint_3d_array: np.ndarray) -> np.ndarray:
    """Compute wrist orientation frame from 21 MediaPipe keypoints."""
    assert keypoint_3d_array.shape == (21, 3)
    points = keypoint_3d_array[[0, 5, 9], :]

    x_vector = points[0] - points[2]
    points = points - np.mean(points, axis=0, keepdims=True)
    _, _, v = np.linalg.svd(points)
    normal = v[2, :]

    x = x_vector - np.sum(x_vector * normal) * normal
    x = x / np.linalg.norm(x)
    z = np.cross(x, normal)

    if np.sum(z * (points[1] - points[2])) < 0:
        normal *= -1
        z *= -1
    return np.stack([x, normal, z], axis=1)


def apply_mediapipe_transformations(
    keypoint_3d_array: np.ndarray,
    hand_type: str = "left",
) -> np.ndarray:
    """Transform wrist-relative landmarks into MediaPipe/MANO wrist frame."""
    hand_type = hand_type.lower()
    arr = np.asarray(keypoint_3d_array, dtype=np.float64)
    arr = arr - arr[0:1, :]
    wrist_rot = estimate_frame_from_hand_points(arr)
    operator2mano = OPERATOR2MANO_RIGHT if hand_type == "right" else OPERATOR2MANO_LEFT
    return arr @ wrist_rot @ operator2mano
