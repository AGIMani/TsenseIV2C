import json
from pathlib import Path
import sys

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
LIGHT_TEST = ROOT / "example" / "light_test"
PROJECT_SRC = ROOT / "src"
for path in (LIGHT_TEST, PROJECT_SRC):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from collision_guard import SelfCollisionGuard  # noqa: E402
from moveit_style_planner import JointSpaceRRTConnectPlanner  # noqa: E402
from hand_gate import (  # noqa: E402
    expected_label_for_selected_hand,
    hand_gate_block_reason,
    hand_selection_matches,
)
from pipeline_geometry import landmarks_ok_for_retarget, prepare_topo_points  # noqa: E402
from replay import (  # noqa: E402
    build_verified_h7_frame,
    load_recording,
    replay_recording,
)
from session_logger import (  # noqa: E402
    RECORDING_MEDIAPIPE,
    SessionJsonlLogger,
)
from utils.mediapipe_frame import apply_mediapipe_transformations  # noqa: E402
from tracking_transition import TrackingTransition  # noqa: E402
from utils.ruckig_interpolator import PhaseSynchronizedRuckigInterpolator  # noqa: E402


def _hand_points() -> np.ndarray:
    points = np.zeros((21, 3), dtype=np.float64)
    points[0] = [0.12, -0.08, 0.04]
    points[5] = [0.08, 0.01, 0.02]
    points[9] = [0.13, 0.035, 0.015]
    points[13] = [0.17, 0.02, 0.01]
    points[17] = [0.20, -0.005, 0.02]
    for index in range(1, 21):
        if not np.any(points[index]):
            points[index] = points[0] + [0.002 * index, 0.006 * index, -0.001 * index]
    return points


def test_topo_boundary_matches_main_pipeline_wrist_transform_for_both_hands():
    points = _hand_points()
    for side in ("left", "right"):
        actual = prepare_topo_points(points, side)
        expected = apply_mediapipe_transformations(points, hand_type=side)
        np.testing.assert_allclose(actual, expected, atol=1.0e-12)
        np.testing.assert_allclose(actual[0], 0.0, atol=1.0e-12)
    assert not np.allclose(
        prepare_topo_points(points, "left"),
        prepare_topo_points(points, "right"),
    )


def test_handedness_gate_requires_explicit_non_fallback_match():
    assert expected_label_for_selected_hand("left") == "Left"
    assert expected_label_for_selected_hand("right") == "Right"
    assert hand_selection_matches("Left", "left")
    assert not hand_selection_matches("Right", "Left")
    assert not hand_selection_matches("Left", "Left", used_fallback_hand=True)
    assert not hand_selection_matches(None, "Left")
    assert hand_gate_block_reason("right", "Right", "Left") is not None


class _FakeCollisionFilter:
    """One joint enters a collision after q=0.6 rad."""

    def __init__(self) -> None:
        self.current = {"joint": 0.0}

    def sync_pose_from_dict(self, pose):
        self.current = dict(pose)

    def contact_summary(self):
        penetration_mm = max(0.0, (float(self.current["joint"]) - 0.6) * 10.0)
        if penetration_mm <= 0.0:
            return []
        return [
            {
                "g1": "finger_a",
                "g2": "finger_b",
                "dist_mm": -penetration_mm,
                "pen_mm": penetration_mm,
                "force_n": 0.0,
            }
        ]

    def reset(self):
        self.current = {"joint": 0.0}


def test_collision_guard_allows_grazing_but_projects_deep_penetration():
    guard = SelfCollisionGuard(
        allowed_penetration_mm=0.2,
        path_step_rad=0.05,
        bisection_iterations=14,
        physics_filter=_FakeCollisionFilter(),
    )

    grazing, grazing_diag = guard.project({"joint": 0.61})
    assert grazing["joint"] == 0.61
    assert not grazing_diag["limited"]
    assert grazing_diag["output_max_penetration_mm"] == 0.1

    projected, diagnostics = guard.project({"joint": 1.0})
    assert diagnostics["limited"]
    assert diagnostics["safe_fraction"] < 1.0
    assert projected["joint"] <= 0.62001
    assert diagnostics["output_max_penetration_mm"] <= 0.20001
    assert diagnostics["target_max_penetration_mm"] == 4.0


class _AvoidableCollisionFilter:
    """Flexion collides unless the index lateral joint moves out first."""

    def __init__(self) -> None:
        self.current = {}

    def sync_pose_from_dict(self, pose):
        self.current = dict(pose)

    def contact_summary(self):
        flexion = float(self.current.get("left_hand_index_MCP_joint", 0.0))
        lateral = float(self.current.get("left_hand_index_swing_joint", 0.0))
        if flexion <= 0.6 or lateral >= 0.10:
            return []
        return [
            {
                "g1": "left_hand_index_tip_geom",
                "g2": "left_hand_mid_tip_geom",
                "dist_mm": -1.0,
                "pen_mm": 1.0,
                "force_n": 0.0,
            }
        ]

    def reset(self):
        self.current = {}


class _DistanceBarrierFilter:
    """Analytic signed-distance model for the CBF/QP avoidance path."""

    def __init__(self) -> None:
        self.current = {}

    def sync_pose_from_dict(self, pose):
        self.current = dict(pose)

    def _distance(self):
        flexion = float(self.current.get("left_hand_index_MCP_joint", 0.0))
        lateral = float(self.current.get("left_hand_index_swing_joint", 0.0))
        return 1.0 + 12.0 * lateral - 8.0 * flexion

    def contact_summary(self):
        distance = self._distance()
        if distance >= 0.0:
            return []
        return [
            {
                "g1": "left_hand_index_tip_geom",
                "g2": "left_hand_mid_tip_geom",
                "dist_mm": distance,
                "pen_mm": -distance,
                "force_n": 0.0,
            }
        ]

    def collision_pair_distances(self, *, max_distance_mm=25.0):
        distance = self._distance()
        if distance >= max_distance_mm:
            return []
        return [
            {
                "geom1": 1,
                "geom2": 2,
                "g1": "left_hand_index_tip_geom",
                "g2": "left_hand_mid_tip_geom",
                "distance_mm": distance,
                "dist_mm": distance,
                "pen_mm": max(0.0, -distance),
            }
        ]

    def collision_pair_distance_mm(
        self, geom1, geom2, *, max_distance_mm=25.0
    ):
        return min(float(max_distance_mm), self._distance())

    def reset(self):
        self.current = {}


class _BlockedTransitionFilter:
    """Both endpoints are safe, but their straight joint path is not."""

    def __init__(self) -> None:
        self.current = {}

    def sync_pose_from_dict(self, pose):
        self.current = dict(pose)

    def contact_summary(self):
        flexion = float(self.current.get("left_hand_index_MCP_joint", 0.0))
        lateral = float(self.current.get("left_hand_index_swing_joint", 0.0))
        if 0.35 < flexion < 0.65 and lateral < 0.08:
            return [
                {
                    "g1": "left_hand_index_tip_geom",
                    "g2": "left_hand_mid_tip_geom",
                    "dist_mm": -1.0,
                    "pen_mm": 1.0,
                    "force_n": 0.0,
                }
            ]
        return []

    def reset(self):
        self.current = {}


def test_collision_avoid_mode_uses_lateral_detour_instead_of_stopping_hand():
    target = {
        "left_hand_index_swing_joint": 0.0,
        "left_hand_index_MCP_joint": 1.0,
    }
    stop_guard = SelfCollisionGuard(
        strategy="stop",
        allowed_penetration_mm=0.2,
        path_step_rad=0.04,
        physics_filter=_AvoidableCollisionFilter(),
    )
    stopped, stop_diag = stop_guard.project(target)
    assert stop_diag["resolution"] == "stop"
    assert stopped["left_hand_index_MCP_joint"] < 0.61

    avoid_guard = SelfCollisionGuard(
        strategy="avoid",
        allowed_penetration_mm=0.2,
        path_step_rad=0.04,
        physics_filter=_AvoidableCollisionFilter(),
    )
    current = {
        "left_hand_index_swing_joint": 0.0,
        "left_hand_index_MCP_joint": 0.0,
    }
    history = []
    for _ in range(60):
        current, avoid_diag = avoid_guard.project(
            target,
            current_values=current,
        )
        history.append((dict(current), dict(avoid_diag)))

    # Phase 1 changes only the lateral joint, so independently timed Ruckig
    # profiles cannot flex into the neighbouring finger before clearance.
    first, first_diag = history[0]
    assert first_diag["avoided"]
    assert first_diag["resolution"] == "avoid"
    assert first_diag["avoidance_phase"] == "sidestep"
    assert first["left_hand_index_MCP_joint"] == 0.0
    assert 0.0 < first["left_hand_index_swing_joint"] <= 0.03501

    assert any(
        row[1]["avoidance_phase"] in {"advance", "hold"}
        and row[0]["left_hand_index_MCP_joint"] > 0.0
        for row in history
    )
    assert current["left_hand_index_MCP_joint"] == 1.0
    assert current["left_hand_index_swing_joint"] >= 0.10
    assert avoid_diag["output_max_penetration_mm"] <= 0.2


def test_collision_guard_never_uses_fixed_pinky_rotation_for_avoidance():
    guard = SelfCollisionGuard(
        strategy="avoid",
        physics_filter=_AvoidableCollisionFilter(),
    )
    output, _ = guard.project(
        {
            "left_hand_index_swing_joint": 0.0,
            "left_hand_index_MCP_joint": 0.0,
            "left_hand_pinky_rota_joint": 1.2,
        }
    )
    assert output["left_hand_pinky_rota_joint"] == 0.0


def test_moveit_style_plan_routes_around_a_blocked_safe_transition():
    guard = SelfCollisionGuard(
        strategy="avoid",
        path_step_rad=0.025,
        physics_filter=_BlockedTransitionFilter(),
    )
    target = {
        "left_hand_index_swing_joint": 0.0,
        "left_hand_index_MCP_joint": 1.0,
    }
    current = {name: 0.0 for name in target}
    history = []
    for _ in range(100):
        current, diagnostics = guard.project(target, current_values=current)
        history.append((dict(current), dict(diagnostics)))
        assert diagnostics["output_max_penetration_mm"] <= 0.20001

    first, first_diag = history[0]
    assert first_diag["avoidance_backend"] == "moveit_style_hybrid"
    assert first_diag["planning_result"] == "structured"
    assert first_diag["avoidance_phase"] == "sidestep"
    assert first["left_hand_index_MCP_joint"] == 0.0
    assert first["left_hand_index_swing_joint"] > 0.0
    assert current == target


@pytest.mark.parametrize("mode", ["avoid", "stop"])
def test_loss_recovery_plans_from_stopped_pose_and_checks_every_ruckig_sample(mode):
    flex = "left_hand_index_MCP_joint"
    lateral = "left_hand_index_swing_joint"
    fixed = "left_hand_pinky_rota_joint"
    current = {flex: 0.1, lateral: 0.0, fixed: 0.0}
    interp = PhaseSynchronizedRuckigInterpolator(joint_names=list(current))
    interp.seed_ramp(current, current)
    tracking = TrackingTransition(interp)
    planner = SelfCollisionGuard(strategy=mode, path_step_rad=0.025, physics_filter=_BlockedTransitionFilter())
    sample_guard = SelfCollisionGuard(strategy="stop", path_step_rad=0.01, physics_filter=_BlockedTransitionFilter())
    sample_guard.rebase(current)
    now = 0.0
    for _ in range(3):
        tracking.observe(True, now=now, current=current)
        now += 0.03
    interp.push({flex: 0.3, lateral: 0.0, fixed: 0.0})
    history = []

    def sample():
        nonlocal now, current
        tracking.tick(now=now, current=current)
        pose, diag = sample_guard.project(interp.sample())
        assert diag["output_max_penetration_mm"] <= 0.200001
        assert all(abs(pose[name] - current[name]) <= velocity * 0.002 + 1e-6
                   for name, velocity in zip(interp._joint_names, interp._input.max_velocity))
        assert pose[fixed] == 0.0
        assert not interp.last_faults
        if diag["limited"]:
            interp.sync_current_pose(pose)
        current = pose
        history.append(dict(current))
        now += 0.002

    for tick in range(225):
        if tick % 16 == 0:
            tracking.observe(True, now=now, current=current)
        sample()
    lost_pose = dict(current)
    tracking.observe(False, now=now, current=current)
    for _ in range(650):
        sample()
    assert tracking.phase == "holding"
    stopped = dict(current)
    assert stopped[flex] >= lost_pose[flex]
    assert stopped[flex] < 0.35
    assert history[-1] == history[-50]

    # The new endpoints are both safe, but straight interpolation intersects
    # the obstacle. Pretend an old route existed and prove it is discarded.
    planner._global_waypoints = [{flex: 0.9, lateral: 0.0, fixed: 0.0}]
    for _ in range(3):
        decision = tracking.observe(True, now=now, current=current)
        now += 0.03
    assert decision["reset_planner"]
    planner.rebase(current)
    assert planner._global_waypoints == []
    assert planner._previous_safe == stopped
    target = {flex: 1.0, lateral: 0.0, fixed: 0.0}
    reentry_index = len(history)
    saw_detour = False
    for tick in range(6500):
        if tick % 16 == 0:
            decision = tracking.observe(True, now=now, current=current)
            assert decision["allow_retarget"]
            safe, diag = planner.project(target, current_values=current)
            saw_detour = saw_detour or diag.get("avoided", False)
            interp.push(safe)
        sample()
    assert max(abs(history[reentry_index][name] - stopped[name]) for name in stopped) < 1e-5
    if mode == "avoid":
        assert saw_detour
        assert current[flex] > 0.99
        assert current[lateral] < 0.01
        assert max(row[lateral] for row in history) >= 0.08
    else:
        assert not saw_detour
        assert current[flex] <= 0.350001


def test_rebase_rejects_an_unsafe_reacquisition_start():
    guard = SelfCollisionGuard(physics_filter=_BlockedTransitionFilter())
    with pytest.raises(RuntimeError, match="reacquisition pose"):
        guard.rebase({"left_hand_index_MCP_joint": 0.5, "left_hand_index_swing_joint": 0.0})


def test_rrt_connect_finds_and_shortcuts_a_collision_free_joint_path():
    planner = JointSpaceRRTConnectPlanner(
        max_iterations=220,
        extension_step_rad=0.16,
        max_edge_checks=900,
        seed=17,
    )

    def state_is_valid(pose):
        return not (
            -0.30 < float(pose["joint_a"]) < 0.30
            and abs(float(pose["joint_b"])) < 0.22
        )

    def edge_is_valid(first, second):
        maximum = max(
            abs(float(second[name]) - float(first[name])) for name in first
        )
        steps = max(1, int(np.ceil(maximum / 0.025)))
        for step in range(1, steps + 1):
            alpha = step / steps
            pose = {
                name: float(first[name])
                + alpha * (float(second[name]) - float(first[name]))
                for name in first
            }
            if not state_is_valid(pose):
                return False
        return True

    plan = planner.plan(
        {"joint_a": -1.0, "joint_b": 0.0},
        {"joint_a": 1.0, "joint_b": 0.0},
        bounds={"joint_a": (-1.2, 1.2), "joint_b": (-1.0, 1.0)},
        state_is_valid=state_is_valid,
        edge_is_valid=edge_is_valid,
        exploration_joints=("joint_b",),
    )

    assert plan is not None
    assert plan.method == "rrt_connect"
    assert len(plan.waypoints) >= 3
    assert all(
        edge_is_valid(plan.waypoints[index], plan.waypoints[index + 1])
        for index in range(len(plan.waypoints) - 1)
    )


def test_distance_barrier_drives_a_lateral_joint_before_interpenetration():
    guard = SelfCollisionGuard(
        strategy="avoid",
        allowed_penetration_mm=0.2,
        safety_clearance_mm=0.25,
        activation_distance_mm=1.25,
        physics_filter=_DistanceBarrierFilter(),
    )
    target = {
        "left_hand_index_swing_joint": 0.0,
        "left_hand_index_MCP_joint": 1.0,
    }
    current = {name: 0.0 for name in target}
    saw_barrier = False
    for _ in range(100):
        current, diagnostics = guard.project(target, current_values=current)
        saw_barrier = saw_barrier or diagnostics.get("cbf_constraints", 0) > 0
        assert diagnostics["output_max_penetration_mm"] <= 0.20001

    assert saw_barrier
    assert diagnostics["avoidance_backend"] == "mujoco_cbf_qp"
    assert current["left_hand_index_MCP_joint"] > 0.8
    assert current["left_hand_index_swing_joint"] > 0.3


def test_retarget_quality_gate_rejects_collapsed_points():
    assert landmarks_ok_for_retarget(_hand_points())
    assert not landmarks_ok_for_retarget(np.zeros((21, 3), dtype=np.float64))


def test_session_logger_writes_recoverable_jsonl_with_numpy_values(tmp_path: Path):
    logger = SessionJsonlLogger(
        tmp_path / "logs",
        session_metadata={"hand_side": "right"},
        flush_every=1,
    )
    log_path = logger.path
    logger.record_frame(
        {
            "frame_id": 7,
            "wuji_points_21_m": np.zeros((21, 3), dtype=np.float32),
            "joint_angles_rad": {"right_hand_thumb_rota_joint": np.float32(0.4)},
        }
    )
    logger.close(reason="test")

    rows = [json.loads(line) for line in log_path.read_text(encoding="utf-8").splitlines()]
    assert [row["type"] for row in rows] == ["session_start", "frame", "session_end"]
    assert len(rows[1]["wuji_points_21_m"]) == 21
    assert rows[1]["joint_angles_rad"]["right_hand_thumb_rota_joint"] == np.float32(0.4)
    assert rows[-1]["reason"] == "test"


def test_mediapipe_recording_mode_keeps_replay_input_but_omits_outputs(
    tmp_path: Path,
):
    logger = SessionJsonlLogger(
        tmp_path / "logs",
        session_metadata={"hand_side": "right"},
        recording_mode=RECORDING_MEDIAPIPE,
        flush_every=1,
    )
    log_path = logger.path
    logger.record_frame(
        {
            "frame_id": 1,
            "hand_side": "right",
            "wuji_points_21_m": np.ones((21, 3)),
            "retarget_joint_angles_rad": {"should_not": 1.0},
            "linear_strokes_mm": [1.0] * 14,
        }
    )
    logger.close(reason="test")

    rows = [json.loads(line) for line in log_path.read_text(encoding="utf-8").splitlines()]
    assert rows[0]["schema_version"] == 2
    assert rows[0]["recording_mode"] == RECORDING_MEDIAPIPE
    assert len(rows[1]["wuji_points_21_m"]) == 21
    assert "retarget_joint_angles_rad" not in rows[1]
    assert "linear_strokes_mm" not in rows[1]

    metadata, replay_frames = load_recording(log_path)
    assert metadata["replay_input_field"] == "wuji_points_21_m"
    assert replay_frames[0]["frame_id"] == 1


def test_offline_replay_builds_crc_verified_h7_frame():
    frame = build_verified_h7_frame(37, [0.0] * 14, [0.0] * 7)
    assert len(frame) == 96
    assert frame[6] == 37


def test_offline_replay_blocks_mismatching_hand_before_retarget_and_h7(
    tmp_path: Path,
):
    logger = SessionJsonlLogger(
        tmp_path / "logs",
        session_metadata={
            "hand_side": "right",
            "requested_camera_fps": 30,
            "self_collision_mode": "off",
        },
        recording_mode=RECORDING_MEDIAPIPE,
        flush_every=1,
    )
    log_path = logger.path
    logger.record_frame(
        {
            "frame_id": 1,
            "hand_side": "right",
            "detected": True,
            "held_last_valid": False,
            "used_fallback_hand": False,
            "detected_label": "Left",
            # Deliberately stale/incorrect recorded policy: replay must use
            # the current direct-label mapping and still block this frame.
            "expected_label": "Left",
            "wuji_points_21_m": _hand_points(),
        }
    )
    logger.close(reason="test")

    summary = replay_recording(log_path, collision_mode="off")
    assert summary["hand_mismatch_blocked_frames"] == 1
    assert summary["retarget_frames"] == 0
    assert summary["ruckig_ticks"] == 0
    assert summary["h7_frames_verified"] == 0
    assert not summary["ok"]


def test_offline_replay_held_frames_brake_then_reacquire_without_a_pose_jump(tmp_path, monkeypatch):
    import replay as replay_module

    monkeypatch.setattr(replay_module, "set_retarget_hand_side", lambda side: None)
    monkeypatch.setattr(replay_module, "reset_retarget_state", lambda: None)
    monkeypatch.setattr(
        replay_module, "retarget_hand",
        lambda points, **kwargs: {"right_hand_index_MCP_joint": 0.3 if kwargs["frame_id"] < 31 else 1.2},
    )
    logger = SessionJsonlLogger(
        tmp_path / "logs",
        session_metadata={"hand_side": "right", "requested_camera_fps": 30, "self_collision_mode": "off"},
        recording_mode="full",  # retain the explicit simulated frame timestamps
    )
    for frame_id in range(1, 151):
        logger.record_frame({
            "frame_id": frame_id,
            "session_elapsed_s": frame_id / 30.0,
            "detected": True,
            "held_last_valid": 31 <= frame_id <= 90,
            "detected_label": "Right",
            "wuji_points_21_m": _hand_points(),
        })
    logger.close(reason="test")
    output = tmp_path / "replayed.jsonl"
    summary = replay_recording(logger.path, output_path=output)
    rows = [json.loads(line) for line in output.read_text(encoding="utf-8").splitlines()]
    assert summary["ok"]
    assert summary["held_frames"] == 60
    assert summary["retarget_frames"] == 86  # 90 valid minus two confirmations twice
    assert summary["tracking_transition"]["loss_count"] == 1
    assert summary["tracking_transition"]["recovery_count"] == 2
    assert summary["tracking_phase_counts"]["holding"] > 0
    assert summary["max_sample_joint_step_rad"] < 0.008001
    assert rows[89]["ruckig_joint_angles_rad"] == rows[90]["ruckig_joint_angles_rad"]
    assert rows[90]["tracking_transition"]["phase"] == "confirming"
    assert rows[92]["tracking_transition"]["phase"] == "recovering"
