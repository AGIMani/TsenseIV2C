"""Positive swing coordinates preserve geometry and invert SDK feedback."""
from pathlib import Path
import math
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src"), str(ROOT / "tools")]
from a32_driver import A32Device
from a32_kinematics import A32Error, A32StrokeMapper
from a32_kinematics.joint_coordinates import (
    COORDINATE_VERSION, SWING_AXIS_ALIGNMENT, SWING_COMMAND_LIMITS, is_finger_swing,
    requested_limits, legacy_to_model, model_to_legacy,
    thumb_rotaback_origin_offset,
)
from hand_profiles import A32_MODEL_IDS, load_hand_profile
from TsenseI_retargeting.fk.urdf import UrdfModel
from apply_a32_joint_ranges import apply_urdf_coordinates


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
@pytest.mark.parametrize("side", ["left", "right"])
def test_new_limits_initial_pose_and_retarget_seed(model_id, side):
    profile = load_hand_profile(model_id)
    device = A32Device(side, model_id=model_id)
    limits = profile.joint_limits(side)
    mechanism_limits = {c["joint"]: c["limits_rad"] for c in device.mapper.description()["channels"]}
    for name, bounds in limits.items():
        expected = requested_limits(name)
        if expected:
            assert bounds == expected
            if name.endswith("_DIP_joint"):
                assert device.joint_limits()[name][0] == pytest.approx(mechanism_limits[name][0])
                assert device.joint_limits()[name][1] < mechanism_limits[name][1]
            else:
                np.testing.assert_allclose(device.joint_limits()[name],
                                           SWING_COMMAND_LIMITS if is_finger_swing(name) else mechanism_limits[name],
                                           atol=1e-12, rtol=0)
    pose = profile.init_pose(side)
    suffix = "" if side == "left" else "_right"
    assert yaml.safe_load((profile.root / f"q_init{suffix}.yaml").read_text())["q"] == pose
    for name, value in pose.items():
        assert value == (0. if is_finger_swing(name) else .005 if requested_limits(name) else 0.)
    solver = profile.create_retargeter(side)
    np.testing.assert_array_equal(solver.init_q, [pose[name] for name in solver.joint_names])
    assert device._backend is None


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("method", ["exact", "lut"])
def test_four_finger_dip_original_geometry_with_eight_mm_hardware_limit(model_id, side, method):
    mapper = A32StrokeMapper(side, method=method, model_id=model_id)
    device = A32Device(side, model_id=model_id)
    pose = load_hand_profile(model_id).init_pose(side)
    dips = {f"{side}_hand_{finger}_DIP_joint" for finger in ("index", "mid", "ring", "pinky")}
    for name in dips:
        pose[name] = 1.565
    converted = mapper.convert(pose)
    limited = mapper.convert(device.constrain_pose(pose))
    for name in dips:
        assert converted.linear_strokes_mm[name] > 8.
        assert limited.linear_strokes_mm[name] == pytest.approx(8., abs=2e-6)


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
@pytest.mark.parametrize("side", ["left", "right"])
def test_transformed_urdf_preserves_original_pose_and_rebuild_is_idempotent(model_id, side):
    profile = load_hand_profile(model_id)
    source = (ROOT / f"TsenseI_V3.2_hand_withid_fixed/urdf/TsenseI_{side}.urdf" if model_id.endswith("withid")
              else Path(os.environ.get("A32_REFERENCE_SDK", ROOT / "vendor/a3-cpp-sdk")) /
              f"daxian_hand_v3_2/urdf/daxian_hand_{side}_v3_2.urdf")
    old, new = UrdfModel(source), UrdfModel(profile.path("urdf", side))
    for fraction in (0., .25, .5, .75, 1.):
        q = {name: lo + fraction * (hi - lo) for name, (lo, hi) in new.joint_limits().items()}
        a = old.forward_kinematics({name: model_to_legacy(model_id, name, angle)
                                   + thumb_rotaback_origin_offset(model_id, name)
                                   for name, angle in q.items()})
        b = new.forward_kinematics(q)
        for name in a:
            np.testing.assert_allclose(a[name], b[name], atol=2e-12)
    root = ET.parse(profile.path("urdf", side)).getroot()
    before = ET.tostring(root)
    assert ET.tostring(apply_urdf_coordinates(root, model_id=model_id)) == before
    assert root.get("a32_joint_coordinates") == COORDINATE_VERSION
    if model_id == "TsenseI_v3_2":
        assert root.get("a32_swing_axis_alignment") == SWING_AXIS_ALIGNMENT


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
@pytest.mark.parametrize("fraction", [0., .2, .5, .8, 1.])
def test_all_twenty_endpoints_send_within_limits_and_feedback_inverts(model_id, fraction):
    device = A32Device(model_id=model_id)
    pose = {name: lo + fraction * (hi - lo) for name, (lo, hi) in device.joint_limits().items()}
    targets = device.prepare_targets(pose, clamp=False)
    for channel in device.channels:
        value = targets["self" if channel["channel"] == "self" else "mit_pos"][channel["index"]]
        assert channel["limits"][0] <= value <= channel["limits"][1]
        assert device._inverse(channel, value) == pytest.approx(pose[channel["joint"]], abs=2e-7)
        if is_finger_swing(channel["joint"]):
            q = pose[channel["joint"]]
            assert value == pytest.approx(math.degrees(q))
            assert .005 - 1e-12 <= math.radians(value) <= .695 + 1e-12
    assert device._backend is None  # No DLL or connection used for any sample.


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
@pytest.mark.parametrize("old, bare_value, id_value", [(.35, .7, 0.), (0., .35, .35), (-.35, 0., .7)])
def test_swing_source_coordinates_then_hardware_clip(model_id, old, bare_value, id_value):
    profile = load_hand_profile(model_id)
    device = A32Device(model_id=model_id)
    translated = bare_value if model_id == "TsenseI_v3_2" else id_value
    sent = min(.695, max(.005, translated))
    pose = profile.init_pose()
    for name in pose:
        if is_finger_swing(name):
            pose[name] = legacy_to_model(model_id, name, old)
            assert pose[name] == pytest.approx(translated)
    model_targets = device.mapper.convert(pose)
    command = device.prepare_targets(pose)
    for channel in device.channels:
        name = channel["joint"]
        if is_finger_swing(name):
            assert model_targets.rotary_positions_deg[name] == pytest.approx(math.degrees(translated))
            assert command["self"][channel["index"]] == pytest.approx(math.degrees(sent))
            assert command["applied_pose"][name] == pytest.approx(sent)
            assert device._inverse(channel, command["self"][channel["index"]]) == pytest.approx(sent)
    assert device._backend is None
