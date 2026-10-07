"""Thumb flexion must be observable and survive the right-hand runtime boundary."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from TsenseI_retargeting.fk import RobotKeypointFK, UrdfModel
from TsenseI_retargeting.toporetarget import TopoRetargeter
from utils import pipeline_log, retarget_runtime as runtime

ASSETS = ROOT / "assets/hands/TsenseI_hand"


@pytest.fixture
def fk():
    return RobotKeypointFK(ASSETS / "urdf/TsenseI__hand_left_v1.urdf",
                           ASSETS / "robot_keypoints_21.yaml")


def bend(points, ids=(2, 3, 4)):
    a, b, c = points[list(ids)]
    u, v = b-a, c-b
    return np.arccos(np.clip(u @ v / np.linalg.norm(u) / np.linalg.norm(v), -1, 1))


def test_reparenting_preserves_calibrated_zero_positions(fk):
    poses = fk.model.forward_kinematics({})
    # Original annotations, before correcting their moving-link ownership.
    original = {
        1: ("left_hand_thumb_MCP_link", [.0016260920517402223, -.004357295143621932, -.03515053326127253]),
        2: ("left_hand_thumb_PIP_link", [.000746501508499333, .004652203635844184, -.05595040277284186]),
    }
    points = fk.compute({})
    for idx, (link, xyz) in original.items():
        np.testing.assert_allclose(points[idx], (poses[link] @ [*xyz, 1])[:3], atol=1e-14)


@pytest.mark.parametrize("joint,stationary,angle_ids", [
    ("thumb_PIP", [0, 1, 2], (2, 3, 4)),
    ("thumb_MCP", [0, 1], (1, 2, 3)),
])
def test_flexion_moves_only_downstream_markers_and_changes_bone_angle(fk, joint, stationary, angle_ids):
    a = fk.compute({f"left_hand_{joint}_joint": .5})
    b = fk.compute({f"left_hand_{joint}_joint": 1.})
    np.testing.assert_allclose(a[stationary], b[stationary], atol=1e-14)
    assert .4 < bend(b, angle_ids) - bend(a, angle_ids) < .6
    assert np.linalg.norm(b[4] - a[4]) > .01


@pytest.mark.parametrize("swing", [0., .3, 1., 1.7, 2.])
def test_right_runtime_keeps_solver_pose_and_warm_start(monkeypatch, fk, swing):
    q = {name: .3 for name in fk.model.joint_names()}
    q.update(left_hand_thumb_swing_joint=swing, left_hand_pinky_rota_joint=0.)
    previous = {n.replace("left_hand_", "right_hand_"): v for n, v in q.items()}
    points = fk.compute(q)
    captured = {}

    def retarget(inp, **kwargs):
        captured.update(points=inp.copy(), **kwargs)
        return SimpleNamespace(joint_values=q, success=True, diagnostics={}, loss={})

    monkeypatch.setattr(runtime, "_topo_retargeter", SimpleNamespace(retarget=retarget))
    monkeypatch.setattr(runtime, "_ensure_topo_ready", lambda: True)
    monkeypatch.setattr(runtime, "_sync_topo_config", lambda: None)
    monkeypatch.setattr(pipeline_log, "enabled", lambda: False)
    monkeypatch.setattr(pipeline_log, "log_retarget_diag", lambda *args, **kwargs: None)
    mirrored_input = points * [-1, 1, 1]
    actual = runtime.retarget_hand(mirrored_input, hand_side="right", previous_joint_values=previous)
    np.testing.assert_allclose(captured["points"], points)
    assert captured["previous_joint_values"] == pytest.approx(q)
    assert actual == pytest.approx(previous)
    np.testing.assert_allclose(mirrored_input, points * [-1, 1, 1])

    # Independently load the real right URDF: equal q must mirror the actual
    # articulated positions, not merely pass a reversible scalar mapping.
    right = UrdfModel(ASSETS / "urdf/TsenseI__hand_right_v1.urdf")
    poses = right.forward_kinematics(actual)
    right_points = []
    for kp in fk.keypoints.keypoints:
        local = np.array(kp.local_xyz) * [1, -1, 1]
        link = kp.link.replace("left_hand_", "right_hand_")
        right_points.append((poses[link] @ [*local, 1])[:3])
    np.testing.assert_allclose(right_points, points * [1, -1, 1], atol=1e-12)


@pytest.mark.parametrize("backend", ["auto", "numpy"])
def test_corrected_markers_have_consistent_jacobian_and_recover_bent_thumb(fk, backend):
    config = replace(runtime._build_topo_config(), kinematics_backend=backend)
    solver = TopoRetargeter(config)
    q = (solver.lower + solver.upper) * .4
    points, jac = solver._analytic_kinematics.compute_points_and_jacobian(q)
    compute = lambda v: fk.compute(dict(zip(solver.joint_names, v)))
    np.testing.assert_allclose(points, compute(q), atol=1e-12)
    eps = 1e-6
    numerical = np.stack([(compute(q+d)-compute(q-d))/(2*eps)
                          for d in np.eye(len(q))*eps], axis=-1)
    np.testing.assert_allclose(jac, numerical, atol=1e-8, rtol=1e-5)
    target = dict(left_hand_thumb_rota_joint=.8, left_hand_thumb_swing_joint=1.6,
                  left_hand_thumb_MCP_joint=.45, left_hand_thumb_PIP_joint=.75)
    source = fk.compute(target)
    for _ in range(120):
        result = solver.retarget(source, include_artifacts=False)
    assert abs(result.joint_values["left_hand_thumb_PIP_joint"] - .75) < .1
    assert abs(result.joint_values["left_hand_thumb_swing_joint"] - 1.6) < .1
    assert abs(bend(fk.compute(result.joint_values)) - bend(source)) < np.radians(5)
