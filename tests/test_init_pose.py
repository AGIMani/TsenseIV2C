"""All entry points must agree on one zero init pose, independent of hand side."""
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from joint_schema import URDF_JOINT_MAP, default_joint_position, get_init_pose
from utils.hand_init import get_init_pose as shared_init_pose


@pytest.mark.parametrize("side", ["left", "right"])
def test_init_is_zero_and_returned_poses_are_independent(side):
    init = get_init_pose(side)
    assert shared_init_pose is get_init_pose
    assert len(init) == 21
    assert all(name.startswith(f"{side}_hand_") for name in init)
    assert set(init.values()) == {0.0}
    for name in URDF_JOINT_MAP:
        assert default_joint_position(name) == 0
    init[next(iter(init))] = .8
    assert set(get_init_pose(side).values()) == {0.0}






def test_retarget_cold_start_uses_zero_init():
    from TsenseI_retargeting.toporetarget import TopoRetargeter
    retargeter = TopoRetargeter()
    np.testing.assert_array_equal(retargeter.init_q, np.zeros(len(retargeter.joint_names)))


def test_synchronized_ruckig_initial_and_reset_positions_are_zero():
    from utils.ruckig_interpolator import PhaseSynchronizedRuckigInterpolator
    interpolator = PhaseSynchronizedRuckigInterpolator()
    assert set(interpolator._input.current_position) == {0.0}
    target = {name: .2 for name in get_init_pose()}
    interpolator.seed_ramp(target, target)
    interpolator.reset()
    assert set(interpolator._input.current_position) == {0.0}
    assert set(interpolator._input.target_position) == {0.0}
