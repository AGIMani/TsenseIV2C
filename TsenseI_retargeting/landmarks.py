from __future__ import annotations

from typing import Any

import numpy as np

NUM_LANDMARKS = 21
SOURCE_SKELETON = "mediapipe_hands_21"

WRIST_ID = 0
INDEX_MCP_ID = 5
MIDDLE_MCP_ID = 9
PINKY_MCP_ID = 17

TIP_IDS: tuple[int, ...] = (4, 8, 12, 16, 20)
PINCH_PAIRS: tuple[tuple[int, int], ...] = ((4, 8), (4, 12), (4, 16), (4, 20))

LANDMARK_NAMES: tuple[str, ...] = (
    "Wrist",
    "ThumbCMC",
    "ThumbMCP",
    "ThumbIP",
    "ThumbTip",
    "IndexMCP",
    "IndexPIP",
    "IndexDIP",
    "IndexTip",
    "MiddleMCP",
    "MiddlePIP",
    "MiddleDIP",
    "MiddleTip",
    "RingMCP",
    "RingPIP",
    "RingDIP",
    "RingTip",
    "PinkyMCP",
    "PinkyPIP",
    "PinkyDIP",
    "PinkyTip",
)

FINGER_CHAINS: dict[str, tuple[int, ...]] = {
    "thumb": (1, 2, 3, 4),
    "index": (5, 6, 7, 8),
    "middle": (9, 10, 11, 12),
    "ring": (13, 14, 15, 16),
    "little": (17, 18, 19, 20),
}

ROOTED_FINGER_CHAINS: dict[str, tuple[int, ...]] = {
    finger: (WRIST_ID, *chain) for finger, chain in FINGER_CHAINS.items()
}

SKELETON_EDGES: tuple[tuple[int, int], ...] = tuple(
    (chain[idx], chain[idx + 1])
    for chain in FINGER_CHAINS.values()
    for idx in range(len(chain) - 1)
) + tuple((WRIST_ID, chain[0]) for chain in FINGER_CHAINS.values())

BONE_DIRECTION_EDGES: tuple[tuple[int, int], ...] = tuple(
    dict.fromkeys(
        (chain[idx], chain[idx + 1])
        for chain in ROOTED_FINGER_CHAINS.values()
        for idx in range(len(chain) - 1)
    )
)


def points21(points: np.ndarray) -> np.ndarray:
    arr = np.asarray(points, dtype=float)
    if arr.shape != (NUM_LANDMARKS, 3):
        raise ValueError(f"expected MediaPipe hand landmarks shape ({NUM_LANDMARKS}, 3), got {arr.shape}")
    if not np.isfinite(arr).all():
        raise ValueError("MediaPipe hand landmarks contain NaN or Inf")
    return arr


def palm_width_m(points: np.ndarray) -> float:
    arr = points21(points)
    width = float(np.linalg.norm(arr[INDEX_MCP_ID] - arr[PINKY_MCP_ID]))
    if width <= 1.0e-9:
        raise ValueError("palm width is non-positive")
    return width


def points_from_record(data: dict[str, Any]) -> np.ndarray:
    for key in ("mediapipe_landmarks_xyz", "keypoints", "points", "landmarks"):
        if key in data:
            return points21(np.asarray(data[key], dtype=float))
    raise ValueError("record must contain mediapipe_landmarks_xyz, keypoints, points, or landmarks")
