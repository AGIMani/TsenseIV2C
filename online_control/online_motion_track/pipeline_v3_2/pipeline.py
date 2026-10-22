"""V3.2-specific observation gate and retargeting, independent of camera/Qt/USB."""
from __future__ import annotations

from dataclasses import dataclass, field, replace
import time

import numpy as np

from hand_profiles import A32_MODEL_IDS, HandModelSession
from a32_kinematics import A32StrokeMapper
from utils.mediapipe_frame import apply_mediapipe_transformations
from online_control.online_motion_track.command_compensation import compensate_command_pose


MODEL_ID = "TsenseI_v3_2"
COLLISION_MODES = ("monitor", "off", "filter", "mink", "stop")
EDGES = tuple((a, b) for start in (1, 5, 9, 13, 17)
              for a, b in zip((0, start, start + 1, start + 2),
                              (start, start + 1, start + 2, start + 3)))


def observation_depth_source(observation) -> str:
    """Describe the actual metric source, including held/wrong-hand records."""
    if observation is None:
        return "none"
    if getattr(observation, "source", "mediapipe") == "webxr":
        return "webxr_metric"
    if getattr(observation, "source", "mediapipe") == "wuji_glove":
        return "wuji_glove_metric"
    if getattr(observation, "source", "mediapipe") == "leapmotion":
        return "leapmotion_metric"
    if getattr(observation, "source", "mediapipe") == "realsense_rgbd":
        return "realsense_rgbd_fused"
    return "mediapipe_world" if observation.world_points is not None else "normalized_z_fallback"


@dataclass
class PoseUpdate:
    pose: dict[str, float]
    tracking: bool
    message: str
    points: np.ndarray | None = None
    depth_source: str = "none"
    solve_ms: float = 0.0
    contacts: int = 0
    capsules: list = field(default_factory=list)
    retarget_pose: dict[str, float] | None = None  # Before optional collision filtering.
    collision_diagnostics: dict = field(default_factory=dict)
    actuation: dict = field(default_factory=dict)
    mink_pose: dict[str, float] | None = None
    direct_pose: dict[str, float] | None = None


def prepare_points(points, hand_side: str) -> np.ndarray:
    """Convert finite nondegenerate 3-D observations; preserve hand chirality.

    MANO wrist alignment is a proper rotation. V3.2 has separate left/right
    solvers, so the shared-left V2 pipeline's extra reflection is NOT applied.
    TopoRetarget handles palm-width normalization itself.
    """
    arr = np.asarray(points, dtype=np.float64)
    if arr.shape != (21, 3) or not np.isfinite(arr).all():
        raise ValueError("需要有限的 (21, 3) 三维关键点")
    if hand_side not in ("left", "right"):
        raise ValueError(f"Invalid hand side: {hand_side}")
    lengths = np.array([np.linalg.norm(arr[b] - arr[a]) for a, b in EDGES])
    across, forward = arr[5] - arr[17], arr[9] - arr[0]
    denom = np.linalg.norm(across) * np.linalg.norm(forward)
    # Reject collapsed bones/palm, but allow closed fingers and touching tips.
    if lengths.min() < 1e-6 or denom < 1e-10:
        raise ValueError("关键点骨段或掌宽退化")
    if np.linalg.norm(np.cross(across, forward)) / denom < 0.05:
        raise ValueError("掌部关键点接近共线")
    return apply_mediapipe_transformations(arr, hand_type=hand_side)


class MediaPipeHandPipeline:
    def __init__(self, hand_side="left", collision_mode="monitor", *, expected_label=None, retarget_options=None,
                 model_id=MODEL_ID, stroke_mapper=None, collision_guard=None):
        if collision_mode not in COLLISION_MODES:
            raise ValueError(f"Invalid collision mode: {collision_mode}")
        if model_id not in A32_MODEL_IDS and stroke_mapper is None:
            raise ValueError("A separate actuation adapter is required for this model")
        self.session = HandModelSession(model_id, hand_side)
        self.stroke_mapper = stroke_mapper if stroke_mapper is not None else A32StrokeMapper(hand_side, model_id=model_id)
        if retarget_options:
            self.session.profile = replace(self.session.profile, retarget_options={
                **self.session.profile.retarget_options, **retarget_options})
        self.session.retargeter = self.session.profile.create_retargeter(hand_side)
        self.expected_label = expected_label or hand_side.title()
        self.collision_mode = collision_mode
        self.collision_bypass = False
        self.contacts = 0
        self.capsules = []
        self.mink_guard = collision_guard
        if collision_guard is not None:
            pass
        elif collision_mode == "stop":
            from online_control.online_motion_track.pipeline_v3_0.collision import StopCollisionGuard
            self.mink_guard = StopCollisionGuard(self.session.profile, hand_side)
        elif collision_mode == "mink":
            from online_control.online_motion_track.mink_guard import MinkCollisionGuard
            self.mink_guard = MinkCollisionGuard(self.session.profile, hand_side)
        elif collision_mode != "off":
            self.session.ensure_collision()
            self._update_contacts()

    def _update_contacts(self):
        filt = self.session.ensure_collision()
        filt.sync_pose_from_dict(self.session.pose)
        self.contacts = int(filt.data.ncon)
        self.capsules = filt.get_capsule_render_data()

    def _hold(self, message, *, retarget_pose=None, points=None, depth_source="none"):
        if self.collision_bypass:
            return PoseUpdate(dict(self.session.pose), False, message,
                              retarget_pose=retarget_pose, points=points, depth_source=depth_source,
                              actuation=self.stroke_mapper.preview(self.session.pose))
        return PoseUpdate(dict(self.session.pose), False, message,
                          contacts=self.contacts, capsules=self.capsules,
                          retarget_pose=retarget_pose, points=points, depth_source=depth_source,
                          collision_diagnostics=dict(self.mink_guard.diagnostics) if self.mink_guard else
                          ({"mode": "monitor", "contact_count": self.contacts} if self.collision_mode == "monitor" else {}),
                          actuation=self.stroke_mapper.preview(self.session.pose))

    def process(self, observation) -> PoseUpdate:
        if observation is None or observation.held_last_valid:
            return self._hold("未检测到所选手，保持上一姿态")
        if (observation.used_fallback_hand
                or observation.detected_label != self.expected_label
                or observation.expected_label != self.expected_label):
            return self._hold(f"当前检测 {observation.detected_label or '未知'}，"
                              f"等待 {self.expected_label}；保持上一姿态")
        # world_points are the tracker's filtered metric landmarks. Avoid Wuji
        # fixed bone-length correction: this solver uses calibrated V3.2 points.
        world = observation.world_points
        source = world if world is not None else observation.points
        depth = observation_depth_source(observation)
        try:
            points = prepare_points(source, self.session.hand_side)
        except (ValueError, TypeError) as exc:
            return self._hold(f"关键点无效：{exc}；保持上一姿态")
        previous = dict(self.session.pose)
        mode = "off" if self.collision_bypass else self.collision_mode
        guard = None if self.collision_bypass else self.mink_guard
        if self.collision_bypass:
            self.contacts, self.capsules = 0, []
        retarget_pose = None
        start = time.perf_counter()
        try:
            retarget_pose = self.session.retarget(points, collision=False)
            target = compensate_command_pose(retarget_pose, self.session.limits)
            target = guard.project(target) if guard else target
            pose = self.session.set_pose(target, collision=mode == "filter")
            # Numerical contact integration may slightly exceed a URDF limit.
            # Validate/re-clamp its output before handing it to the renderer.
            pose = self.session.set_pose(pose)
            if guard:
                solver = self.session.retargeter
                solver.previous_q = np.array([pose[n] for n in solver.joint_names])
                solver.previous_init_q = solver.previous_q.copy()
            elif mode != "off":
                self._update_contacts()
        except (ValueError, RuntimeError, FloatingPointError) as exc:
            self.session.pose = previous
            self.session.retargeter.reset()
            if guard is None and mode != "off":
                self._update_contacts()
            return self._hold(f"求解失败：{exc}；保持上一姿态",
                              retarget_pose=retarget_pose, points=points, depth_source=depth)
        message = "正在跟随 · 三维 world landmarks"
        if depth == "webxr_metric":
            message = "正在跟随 · Quest 3 WebXR 三维关节"
        elif depth == "wuji_glove_metric":
            message = "正在跟随 · Wuji 手套 21 点骨架"
        elif depth == "leapmotion_metric":
            message = "正在跟随 · Leap Motion 2 三维 21 点骨架"
        elif depth == "realsense_rgbd_fused":
            message = "正在跟随 · D455f 深度约束 21 点骨架"
        elif depth != "mediapipe_world":
            message = "正在跟随 · 归一化 Z 回退（深度精度较低）"
        return PoseUpdate(pose, True, message, points, depth,
                          (time.perf_counter() - start) * 1000,
                          self.contacts, self.capsules, retarget_pose,
                          dict(guard.diagnostics) if guard else
                          ({"mode": "monitor", "contact_count": self.contacts,
                            "contacts": self.session.ensure_collision().contact_summary()}
                           if mode == "monitor" else {}),
                          self.stroke_mapper.preview(pose),
                          mink_pose=dict(target) if mode == "mink" and guard is not None else None,
                          direct_pose=dict(pose) if self.collision_bypass else None)
