"""Explicit model selection for calibration, simulation and future hardware adapters.

No process-wide mutation of V2 constants and no import of USB/camera modules.
Each session owns its solver and collision state; switching creates a new session.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from pathlib import Path
from typing import Protocol, Mapping
import math
import xml.etree.ElementTree as ET

import yaml

ROOT = Path(__file__).resolve().parents[1]
A30_MODEL_IDS = ("TsenseI_v3_0", "TsenseI_v3_0_withid")
A32_MODEL_IDS = ("TsenseI_v3_2", "TsenseI_v3_2_withid")
PROFILE_PATHS = {
    "TsenseI_v3_0_withid": ROOT / "assets/hands/TsenseI_hand_v3_0_withid/hand_profile.yaml",
    "TsenseI_v3_0": ROOT / "assets/hands/TsenseI_hand_v3_0/hand_profile.yaml",
    "TsenseI_v2": ROOT / "assets/hands/TsenseI_hand/hand_profile.yaml",
    "TsenseI_v3_2": ROOT / "assets/hands/TsenseI_hand_v3_2/hand_profile.yaml",
    "TsenseI_v3_2_withid": ROOT / "assets/hands/TsenseI_hand_v3_2_withid/hand_profile.yaml",
}


class HardwareAdapter(Protocol):
    """Implement version-specific joint-to-motor conversion outside the session."""
    def send_joint_targets(self, model_id: str, hand_side: str,
                           joint_values: Mapping[str, float]) -> None: ...


@dataclass(frozen=True)
class HandModelProfile:
    model_id: str
    display_name: str
    root: Path
    sides: dict
    fixed_joints: dict
    hardware_enabled: bool
    retarget_options: dict = field(default_factory=dict)
    mirror_axis: str = "y"
    right_hand_provisional: bool = False

    def path(self, key: str, hand_side: str = "left") -> Path:
        if hand_side not in self.sides:
            raise ValueError(f"Unsupported hand side: {hand_side}")
        return (self.root / self.sides[hand_side][key]).resolve()

    def joint_limits(self, hand_side: str = "left") -> dict[str, tuple[float, float]]:
        root = ET.parse(self.path("urdf", hand_side)).getroot()
        return {j.get("name"): (float(j.find("limit").get("lower")),
                               float(j.find("limit").get("upper")))
                for j in root.findall("joint") if j.get("type") == "revolute" and j.find("mimic") is None}

    def init_pose(self, hand_side: str = "left") -> dict[str, float]:
        if self.model_id in A32_MODEL_IDS:
            from a32_kinematics.joint_coordinates import initial_pose
            return initial_pose(self.joint_limits(hand_side))
        return {name: min(hi, max(lo, 0.0)) for name, (lo, hi) in self.joint_limits(hand_side).items()}

    def calibration_status(self, hand_side: str = "left") -> dict:
        data = yaml.safe_load(self.path("keypoints", hand_side).read_text(encoding="utf-8"))
        rows = data.get("keypoints", [])
        reviewed = sum(row.get("annotation_status") == "manually_adjusted" for row in rows)
        return {"point_count": len(rows), "reviewed_count": reviewed,
                "pending": len(rows) != 21 or reviewed != 21,
                "quality_status": data.get("quality_status", "Draft")}

    def create_retargeter(self, hand_side: str = "left"):
        from TsenseI_retargeting import TopoRetargetConfig, TopoRetargeter
        # V2 right assets share calibrated left solver. V3.2 has a true right
        # URDF and mirrored local points and can solve either side directly.
        solver_side = self.sides[hand_side].get("solver_side", hand_side)
        names = tuple(n for n in self.joint_limits(solver_side) if n not in self.fixed_joints)
        # Asset selection belongs to the profile's sides, never solver tuning.
        reserved = {"robot_root", "urdf_path", "keypoints_path", "joint_names", "initial_joint_values"}
        if reserved.intersection(self.retarget_options):
            raise ValueError("retarget options must not override model assets")
        return TopoRetargeter(TopoRetargetConfig(
            robot_root=self.root, urdf_path=self.path("urdf", solver_side),
            keypoints_path=self.path("keypoints", solver_side), joint_names=names,
            initial_joint_values={name: self.init_pose(solver_side)[name] for name in names}
                if self.model_id in A32_MODEL_IDS else None,
            **self.retarget_options,
        ))

    def create_collision_filter(self, hand_side: str = "left"):
        from physics.mujoco_filter import MuJoCoPhysicsFilter
        collision_side = self.sides[hand_side].get("collision_side", hand_side)
        filt = MuJoCoPhysicsFilter(self.path("mjcf", collision_side), warmup_steps=0,
                                  urdf_joint_names=list(self.joint_limits(collision_side)))
        filt.sync_pose_from_dict(self.init_pose(collision_side))
        return filt


def load_hand_profile(model_id: str = "TsenseI_v2") -> HandModelProfile:
    path = PROFILE_PATHS[model_id]
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if data["model_id"] != model_id or data["schema_version"] != 1:
        raise ValueError(f"Invalid hand profile: {path}")
    return HandModelProfile(model_id, data["display_name"], path.parent,
                            data["sides"], data.get("fixed_joints", {}),
                            bool(data.get("hardware", {}).get("enabled", False)),
                            dict(data.get("retarget", {})),
                            data.get("mirror_axis", "y"),
                            bool(data.get("right_hand_provisional", False)))


class HandModelSession:
    """Per-model simulation entry point. Input/output joint angles are URDF radians.

    Retarget input is 21 world points in the chosen hand's robot wrist frame,
    not image pixel coordinates. Camera/Quest coordinate conversion is upstream.
    """
    def __init__(self, model_id: str = "TsenseI_v3_2", hand_side: str = "left"):
        self.profile = load_hand_profile(model_id)
        self.hand_side = hand_side
        self.limits = self.profile.joint_limits(hand_side)
        self.pose = self.profile.init_pose(hand_side)
        self.retargeter = None
        self.collision = None
        self.hardware_adapter: HardwareAdapter | None = None

    def set_pose(self, values: Mapping[str, float], *, collision: bool = False) -> dict[str, float]:
        unknown = set(values) - set(self.limits)
        if unknown:
            raise ValueError(f"Joints do not belong to {self.profile.model_id}: {sorted(unknown)}")
        if any(not math.isfinite(float(v)) for v in values.values()):
            raise ValueError("Joint targets must be finite")
        pose = dict(self.pose)
        for name, value in values.items():
            lo, hi = self.limits[name]
            pose[name] = min(hi, max(lo, float(value)))
        if collision:
            filt = self.ensure_collision()
            pose = filt.step_dict(pose)
        self.pose = pose
        return dict(pose)

    def ensure_collision(self):
        if self.collision is None:
            self.collision = self.profile.create_collision_filter(self.hand_side)
        return self.collision

    def retarget(self, points_21, *, collision: bool = False) -> dict[str, float]:
        import numpy as np
        if self.retargeter is None:
            self.retargeter = self.profile.create_retargeter(self.hand_side)
        points = np.array(points_21, dtype=float, copy=True)
        shared_left = self.profile.sides[self.hand_side].get("solver_side") == "left" and self.hand_side == "right"
        if shared_left:
            points[:, 1] *= -1
        result = self.retargeter.retarget(points, include_artifacts=False)
        if not result.success:
            raise RuntimeError(f"Retarget failed: {result.diagnostics}")
        values = {**result.joint_values, **self.profile.fixed_joints}
        if shared_left:
            values = {k.replace("left_hand_", "right_hand_", 1): v for k, v in values.items()}
        return self.set_pose(values, collision=collision)

    def send_to_hardware(self) -> None:
        if not self.profile.hardware_enabled or self.hardware_adapter is None:
            raise RuntimeError(f"{self.profile.model_id}: hardware adapter is not configured")
        self.hardware_adapter.send_joint_targets(self.profile.model_id, self.hand_side, dict(self.pose))
