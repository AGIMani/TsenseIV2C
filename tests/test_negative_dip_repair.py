from __future__ import annotations

import sys
from pathlib import Path

import numpy as np


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from utils.retarget_runtime import (
    _build_topo_config,
    _prepare_keypoints_for_solver,
    _to_right_joint_names,
    _with_fixed_left_joints,
)


def test_left_hand_geometry_is_not_modified() -> None:
    raw = np.arange(63, dtype=np.float64).reshape(21, 3)

    solver_points = _prepare_keypoints_for_solver(raw, "left")

    assert np.array_equal(solver_points, raw)
    assert solver_points is not raw


def test_right_wrist_frame_is_mirrored_once_for_shared_left_solver() -> None:
    right_wrist_frame = np.arange(63, dtype=np.float64).reshape(21, 3)

    solver_points = _prepare_keypoints_for_solver(right_wrist_frame, "right")

    np.testing.assert_array_equal(solver_points[:, 0], -right_wrist_frame[:, 0])
    np.testing.assert_array_equal(solver_points[:, 1:], right_wrist_frame[:, 1:])
    np.testing.assert_array_equal(
        right_wrist_frame,
        np.arange(63, dtype=np.float64).reshape(21, 3),
    )
    assert solver_points is not right_wrist_frame


def test_pinky_rotation_is_not_a_topo_variable_and_is_reinserted_as_zero() -> None:
    config = _build_topo_config()
    assert config.joint_names is not None
    assert len(config.joint_names) == 20
    assert "left_hand_pinky_rota_joint" not in config.joint_names

    left = _with_fixed_left_joints({"left_hand_pinky_rota_joint": 0.8})
    right = _to_right_joint_names(left)
    assert left["left_hand_pinky_rota_joint"] == 0.0
    assert right["right_hand_pinky_rota_joint"] == 0.0
