"""Reference signed swing coordinates preserve geometry and invert SDK feedback."""
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
    thumb_origin_offset,
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
def test_all_mcp_models_extend_to_1565_with_sdk_scaled_strokes(model_id, side):
    profile = load_hand_profile(model_id)
    device = A32Device(side, model_id=model_id)
    pose = profile.init_pose(side)
    for finger in ("thumb", "index", "mid", "ring", "pinky"):
        name = f"{side}_hand_{finger}_MCP_joint"
        assert profile.joint_limits(side)[name] == (.005, 1.565)
        expected_upper = 1.565
        assert device.joint_limits()[name] == pytest.approx((.005, expected_upper), abs=1e-10, rel=0)
        pose[name] = 1.565
    targets = device.prepare_targets(pose, clamp=True)
    for finger in ("thumb", "index", "mid", "ring", "pinky"):
        name = f"{side}_hand_{finger}_MCP_joint"
        expected_upper = 1.565
        assert targets["applied_pose"][name] == pytest.approx(expected_upper, abs=1e-10, rel=0)
        assert pose[name] == 1.565
    for channel in device.channels:
        value = targets["self" if channel["channel"] == "self" else "mit_pos"][channel["index"]]
        assert channel["limits"][0] <= value <= channel["limits"][1]
    assert device._backend is None


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("method", ["exact", "lut"])
def test_four_finger_dip_original_geometry_and_hardware_limits(model_id, side, method):
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
        assert limited.linear_strokes_mm[name] == pytest.approx(converted.linear_strokes_mm[name], abs=2e-6)


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
@pytest.mark.parametrize("side", ["left", "right"])
def test_transformed_urdf_preserves_original_pose_and_rebuild_is_idempotent(model_id, side):
    profile = load_hand_profile(model_id)
    source = (ROOT / f"assets/hands/TsenseI_hand_v3_2_withid/source_20_outputs/TsenseI_hand_{side}_v3_2_withid.urdf" if model_id.endswith("withid")
              else Path(os.environ.get("A32_REFERENCE_SDK", ROOT / "vendor/a3-cpp-sdk")) /
              f"daxian_hand_v3_2/urdf/daxian_hand_{side}_v3_2.urdf")
    old, new = UrdfModel(source), UrdfModel(profile.path("urdf", side))
    for fraction in (0., .25, .5, .75, 1.):
        q = {name: lo + fraction * (hi - lo) for name, (lo, hi) in new.joint_limits().items()}
        source_q = ({name: model_to_legacy(model_id, name, angle)
                     + thumb_origin_offset(model_id, name) for name, angle in q.items()}
                    if model_id == "TsenseI_v3_2" else
                    {name: -angle if name.endswith('_thumb_rotaback_joint2') and old.joint_by_name[name].lower < 0
                     else angle for name, angle in q.items()})
        a = old.forward_kinematics(source_q)
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
@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("fraction", [0., .2, .5, .8, 1.])
def test_all_twenty_endpoints_send_within_limits_and_feedback_inverts(model_id, side, fraction):
    device = A32Device(side, model_id=model_id)
    pose = {name: lo + fraction * (hi - lo) for name, (lo, hi) in device.joint_limits().items()}
    targets = device.prepare_targets(pose, clamp=False)
    for channel in device.channels:
        value = targets["self" if channel["channel"] == "self" else "mit_pos"][channel["index"]]
        assert channel["limits"][0] <= value <= channel["limits"][1]
        assert device._inverse(channel, value) == pytest.approx(pose[channel["joint"]], abs=2e-7)
        if is_finger_swing(channel["joint"]):
            q = pose[channel["joint"]]
            assert value == pytest.approx(math.degrees(q))
            assert -.34 - 1e-12 <= math.radians(value) <= .34 + 1e-12
    assert device._backend is None  # No DLL or connection used for any sample.


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
@pytest.mark.parametrize("old, bare_value, id_value", [(.35, -.35, .35), (0., 0., 0.), (-.35, .35, -.35)])
def test_swing_source_coordinates_then_hardware_clip(model_id, old, bare_value, id_value):
    profile = load_hand_profile(model_id)
    device = A32Device(model_id=model_id)
    translated = bare_value if model_id == "TsenseI_v3_2" else id_value
    sent = min(.34, max(-.34, translated))
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


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
@pytest.mark.parametrize("side", ["left", "right"])
def test_positive_asset_migration_preserves_fk_and_is_idempotent(tmp_path, model_id, side):
    from scipy.spatial.transform import Rotation
    from a32_kinematics.joint_coordinates import (
        POSITIVE_COORDINATE_VERSION, POSITIVE_SWING_AXIS_ALIGNMENT, migrate_positive_pose,
    )
    profile = load_hand_profile(model_id)
    root = ET.parse(profile.path("urdf", side)).getroot()
    # Reconstruct the previous positive coordinate from its defining transform.
    for joint in root.findall("joint"):
        if not is_finger_swing(joint.get("name", "")):
            continue
        axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
        origin = joint.find("origin")
        rotation = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy"), sep=" "))
        rotation *= Rotation.from_rotvec(axis / np.linalg.norm(axis) * .35)
        origin.set("rpy", " ".join(map(str, rotation.as_euler("xyz"))))
        joint.find("axis").set("xyz", " ".join(map(str, -axis)))
        joint.find("limit").set("lower", "0")
        joint.find("limit").set("upper", "0.7")
    root.set("a32_joint_coordinates", POSITIVE_COORDINATE_VERSION)
    root.set("a32_swing_axis_alignment", POSITIVE_SWING_AXIS_ALIGNMENT)
    old_path = tmp_path / "old.urdf"
    ET.ElementTree(root).write(old_path)
    old = UrdfModel(old_path)
    apply_urdf_coordinates(root, model_id=model_id)
    after = ET.tostring(root)
    assert ET.tostring(apply_urdf_coordinates(root, model_id=model_id)) == after
    new_path = tmp_path / "new.urdf"
    ET.ElementTree(root).write(new_path)
    new = UrdfModel(new_path)
    installed = UrdfModel(profile.path("urdf", side))
    for value in (0., .1, .35, .6, .7):
        pose = profile.init_pose(side)
        pose.update({name: value for name in pose if is_finger_swing(name)})
        migrated = migrate_positive_pose(pose, profile.joint_limits(side))
        a, b, c = old.forward_kinematics(pose), new.forward_kinematics(migrated), installed.forward_kinematics(migrated)
        for name in a:
            np.testing.assert_allclose(a[name], b[name], atol=2e-12)
            np.testing.assert_allclose(b[name], c[name], atol=2e-12)
