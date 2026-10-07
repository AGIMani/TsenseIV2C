"""Landmark temporal smoother (formerly MediaPipeSmoother)."""

from __future__ import annotations

import numpy as np


class LandmarkSmoother:
    """Exponential multi-frame smoother for (21, 3) landmark arrays."""

    def __init__(self, buffer_size: int = 3):
        self.buffer_size = int(buffer_size)
        # Keep legacy attribute name used by skeleton tracker reset helpers.
        self.mediapipe_pose_buffer: list[np.ndarray] | None = None

    def reset(self) -> None:
        self.mediapipe_pose_buffer = None

    def smooth(self, pose: np.ndarray) -> np.ndarray:
        pose = np.asarray(pose, dtype=np.float64)
        if self.mediapipe_pose_buffer is None:
            self.mediapipe_pose_buffer = [pose.copy() for _ in range(self.buffer_size)]
            return pose

        self.mediapipe_pose_buffer.append(pose.copy())
        if len(self.mediapipe_pose_buffer) > self.buffer_size:
            self.mediapipe_pose_buffer.pop(0)

        n = len(self.mediapipe_pose_buffer)
        if n == 1:
            weights = np.array([1.0])
        else:
            weights = np.exp(np.linspace(-2.0, 0.0, n))
            weights = weights / np.sum(weights)

        smoothed = np.zeros_like(pose)
        for frame, weight in zip(self.mediapipe_pose_buffer, weights):
            smoothed += weight * frame
        return smoothed


# Backward-compatible alias
MediaPipeSmoother = LandmarkSmoother
