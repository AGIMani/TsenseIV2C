"""A3.0 preserves retained A3.2 linkages and replaces DIP with direct angles."""
from pathlib import Path
import math
import sys
from fractions import Fraction

import numpy as np
import pytest

sys.path[:0] = [str(Path(__file__).resolve().parents[1] / "src")]
from a30_kinematics import A30StrokeMapper
from a32_kinematics.a32_angles_to_inputs import load_configuration, load_kinematic_model
from a32_kinematics.a32_fast import A32Converter
from hand_profiles import load_hand_profile


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("method", ["exact", "lut"])
def test_retained_channels_match_original_a32_geometry_and_four_dips_are_direct(side, method):
    mapper = A30StrokeMapper(side, method=method)
    # A3.0 retains the original unscaled geometry. A3.2's application mapper
    # now uses SDK 1.1.9 normalized strokes, which must not change A3.0 motion.
    model = load_kinematic_model()
    indices = (3, 5, 7, 9, 11, 13, 15, 17, 19)
    reference = A32Converter(model, load_configuration()).prepare(indices, method=method)
    reference_names = tuple(model.q_names[i - 1].replace("left_hand_", f"{side}_hand_")
                            for i in indices)
    assert mapper._prepared.n == 9
    assert not set(mapper._prepared.indices).intersection({4, 8, 12, 16, 20})
    assert set(mapper.joint_names) == set(load_hand_profile("TsenseI_v3_0").joint_limits(side))
    dips = {f"{side}_hand_{f}_DIP_joint" for f in ("index", "mid", "ring", "pinky")}
    rng = np.random.default_rng(30)
    for _ in range(40):
        pose = {n: rng.uniform(lo, hi) for n, (lo, hi) in zip(mapper.joint_names, mapper._limits)}
        before = dict(pose)
        target = mapper.convert(pose)
        ref = reference([math.degrees(pose[name]) for name in reference_names])
        assert pose == before and target.clamped_joints_rad == {}
        assert len(target.linear_strokes_mm) == 9 and len(target.rotary_positions_deg) == 11
        assert not dips.intersection(target.linear_strokes_mm)
        assert set(target.linear_strokes_mm) == set(reference_names)
        for name, meters in zip(reference_names, ref):
            assert target.linear_strokes_mm[name] == pytest.approx(meters * 1000., abs=1e-9)
        for name in target.rotary_positions_deg:
            assert target.rotary_positions_deg[name] == pytest.approx(math.degrees(pose[name]), abs=1e-12)
        assert f"{side}_hand_pinky_MCP_joint" in target.linear_strokes_mm
        assert f"{side}_hand_pinky_swing_joint" in target.rotary_positions_deg


def test_dip_only_motion_has_no_linear_coupling_and_does_not_use_a32_limits():
    m = A30StrokeMapper()
    zero = dict.fromkeys(m.joint_names, 0.)
    before = m.convert(zero)
    for suffix in ("index_DIP", "mid_DIP", "ring_DIP", "pinky_DIP", "thumb_PIP"):
        pose = dict(zero); dip = f"left_hand_{suffix}_joint"
        pose[dip] = 1.57
        after = m.convert(pose)
        assert after.linear_strokes_mm == before.linear_strokes_mm
        assert after.rotary_positions_deg[dip] == pytest.approx(math.degrees(1.57))
        assert all(after.rotary_positions_deg[n] == v for n, v in before.rotary_positions_deg.items() if n != dip)


def test_strict_limits_and_preview_clamps_are_explicit():
    m = A30StrokeMapper()
    pose = dict.fromkeys(m.joint_names, 0.)
    pose["left_hand_index_MCP_joint"] = 1.57
    pose["left_hand_index_DIP_joint"] = 2.
    with pytest.raises(ValueError):
        m.convert(pose)
    p = m.preview(pose)
    assert p["valid"] and not p["hardware_enabled"] and p["conversion_ready"]
    assert p["clamped_joints_rad"]["left_hand_index_MCP_joint"] == pytest.approx(1.5229)
    assert p["clamped_joints_rad"]["left_hand_index_DIP_joint"] == 1.57
    assert p["rotary_positions_deg"]["left_hand_index_DIP_joint"] == pytest.approx(math.degrees(1.57))
    assert pose["left_hand_index_DIP_joint"] == 2.


@pytest.mark.parametrize("invalid", [float("nan"), float("inf"), True, "0", None, complex(1, 2)])
def test_invalid_input_never_produces_partial_or_stale_targets(invalid):
    m = A30StrokeMapper()
    pose = dict.fromkeys(m.joint_names, 0.)
    m.preview(pose)
    pose["left_hand_mid_DIP_joint"] = invalid
    with pytest.raises(ValueError):
        m.convert(pose)
    p = m.preview(pose)
    assert not p["valid"] and not p["linear_strokes_mm"] and not p["rotary_positions_deg"]


def test_wrong_side_and_incomplete_pose_rejected_and_mirror_uses_equal_angles():
    left, right = A30StrokeMapper(), A30StrokeMapper("right")
    pose = {n: (lo + hi) / 2 for n, (lo, hi) in zip(left.joint_names, left._limits)}
    with pytest.raises(ValueError):
        left.convert({})
    with pytest.raises(ValueError):
        right.convert(pose)
    a = left.convert(pose); b = right.convert({n.replace("left_", "right_"): v for n, v in pose.items()})
    for attr in ("linear_strokes_mm", "rotary_positions_deg"):
        assert {n.replace("left_", "right_"): v for n, v in getattr(a, attr).items()} == getattr(b, attr)


@pytest.mark.parametrize("value", [0, .25, np.float32(.25), np.float64(.25), np.int64(0), Fraction(1, 4)])
def test_builtin_fast_path_preserves_other_real_numeric_inputs(value):
    mapper = A30StrokeMapper()
    pose = dict.fromkeys(mapper.joint_names, 0.)
    name = "left_hand_index_DIP_joint"
    pose[name] = value
    actual = mapper.convert(pose)
    pose[name] = float(value)
    assert actual == mapper.convert(pose)


@pytest.mark.parametrize("value", [False, np.bool_(True), float("-inf"), 10**400])
def test_builtin_fast_path_does_not_admit_bool_nonfinite_or_overflow(value):
    mapper = A30StrokeMapper()
    pose = dict.fromkeys(mapper.joint_names, 0.)
    pose[mapper.joint_names[0]] = value
    with pytest.raises(ValueError):
        mapper.convert(pose)
