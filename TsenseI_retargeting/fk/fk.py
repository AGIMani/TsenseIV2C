from __future__ import annotations

from pathlib import Path

import numpy as np

from .types import RobotKeypointSet
from .urdf import UrdfModel


class RobotKeypointFK:
    def __init__(self, urdf_path: str | Path, keypoint_yaml: str | Path, base_link: str | None = None):
        self.model = UrdfModel(urdf_path)
        if base_link is not None:
            self.model.base_link = base_link
        self.keypoints = RobotKeypointSet.load(keypoint_yaml)

    def compute(self, q: dict[str, float] | np.ndarray | None = None) -> np.ndarray:
        q_map = self._q_to_map(q)
        link_poses = self.model.forward_kinematics(q_map)
        points = []
        for kp in sorted(self.keypoints.keypoints, key=lambda item: item.id):
            if kp.link not in link_poses:
                raise KeyError(f"link {kp.link!r} has no FK pose")
            local = np.array([kp.local_xyz[0], kp.local_xyz[1], kp.local_xyz[2], 1.0], dtype=float)
            world = link_poses[kp.link] @ local
            points.append(world[:3])
        return np.stack(points, axis=0)

    def compute_named(self, q: dict[str, float] | np.ndarray | None = None) -> dict[str, np.ndarray]:
        points = self.compute(q)
        return {kp.source_name: points[kp.id] for kp in self.keypoints.keypoints}

    def to_wrist_frame(self, points_21: np.ndarray) -> np.ndarray:
        wrist = points_21[0]
        return points_21 - wrist.reshape(1, 3)

    def _q_to_map(self, q: dict[str, float] | np.ndarray | None) -> dict[str, float]:
        if q is None:
            return {}
        if isinstance(q, dict):
            return {str(k): float(v) for k, v in q.items()}
        arr = np.asarray(q, dtype=float).reshape(-1)
        names = self.model.joint_names()
        if arr.shape[0] != len(names):
            raise ValueError(f"q has {arr.shape[0]} values but model has {len(names)} movable joints")
        return dict(zip(names, arr.tolist(), strict=True))
