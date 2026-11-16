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
    COORDINATE_VERSION, SWING_AXIS_ALIGNMENT, SWING_COMMAND_LIMITS, BARE_SWING_COMMAND_LIMITS, is_finger_swing,
    requested_limits, legacy_to_model, model_to_legacy,
    thumb_origin_offset, migrate_signed_pose,
    WITHID_CAD_VERSION,
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
        expected = requested_limits(name, model_id=model_id)
        if expected:
            assert bounds == expected
            if "_thumb_rota" not in name:
                np.testing.assert_allclose(device.joint_limits()[name],
                                           (SWING_COMMAND_LIMITS if model_id.endswith("withid") else BARE_SWING_COMMAND_LIMITS)
                                           if is_finger_swing(name) else mechanism_limits[name],
                                           atol=1e-12, rtol=0)
    pose = profile.init_pose(side)
    suffix = "" if side == "left" else "_right"
    assert yaml.safe_load((profile.root / f"q_init{suffix}.yaml").read_text())["q"] == pose
    for name, value in pose.items():
        assert value == 0.
    solver = profile.create_retargeter(side)
    np.testing.assert_array_equal(solver.init_q, [pose[name] for name in solver.joint_names])
    assert device._backend is None


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
@pytest.mark.parametrize("side", ["left", "right"])
def test_all_mcp_models_use_authored_limits_with_sdk_scaled_strokes(model_id, side):
    profile = load_hand_profile(model_id)
    device = A32Device(side, model_id=model_id)
    pose = profile.init_pose(side)
    for finger in ("thumb", "index", "mid", "ring", "pinky"):
        name = f"{side}_hand_{finger}_MCP_joint"
        expected_upper = math.pi / 2 if model_id.endswith("withid") and finger == "thumb" else 1.57
        assert profile.joint_limits(side)[name] == (0., expected_upper)
        assert device.joint_limits()[name] == pytest.approx((0., expected_upper), abs=1e-10, rel=0)
        pose[name] = expected_upper
    targets = device.prepare_targets(pose, clamp=True)
    for finger in ("thumb", "index", "mid", "ring", "pinky"):
        name = f"{side}_hand_{finger}_MCP_joint"
        expected_upper = math.pi / 2 if model_id.endswith("withid") and finger == "thumb" else 1.57
        assert targets["applied_pose"][name] == pytest.approx(expected_upper, abs=1e-10, rel=0)
        assert pose[name] == expected_upper
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
        pose[name] = 1.57
    converted = mapper.convert(pose)
    limited = mapper.convert(device.constrain_pose(pose))
    for name in dips:
        assert converted.linear_strokes_mm[name] > 8.
        assert limited.linear_strokes_mm[name] == pytest.approx(converted.linear_strokes_mm[name], abs=2e-6)


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
@pytest.mark.parametrize("side", ["left", "right"])
def test_transformed_urdf_preserves_original_pose_and_rebuild_is_idempotent(model_id, side):
    profile = load_hand_profile(model_id)
    # Source fidelity is checked independently from the import manifest; a
    # second on-disk URDF is intentionally no longer kept in the repository.
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
            bound = math.pi / 9 if model_id.endswith("withid") else .34
            assert -bound - 1e-12 <= math.radians(value) <= bound + 1e-12
    assert device._backend is None  # No DLL or connection used for any sample.


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
@pytest.mark.parametrize("old, bare_value, id_value", [(.35, -.35, .35), (0., 0., 0.), (-.35, .35, -.35)])
def test_swing_source_coordinates_then_hardware_clip(model_id, old, bare_value, id_value):
    profile = load_hand_profile(model_id)
    device = A32Device(model_id=model_id)
    translated = bare_value if model_id == "TsenseI_v3_2" else id_value
    command_bound = math.pi / 9 if model_id.endswith("withid") else .34
    sent = min(command_bound, max(-command_bound, translated))
    pose = profile.init_pose()
    for name in pose:
        if is_finger_swing(name):
            pose[name] = legacy_to_model(model_id, name, old)
            assert pose[name] == pytest.approx(translated)
    pose = migrate_signed_pose(pose, profile.joint_limits())
    translated = min(math.pi / 9, max(-math.pi / 9, translated))
    model_targets = device.mapper.convert(pose, clamp=True)
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
        # The new exact 20-degree shell clips the old rounded endpoints.
        # Compare FK at the same retained angle in each coordinate system.
        retained_old = {name: .35 - angle if is_finger_swing(name) else angle
                        for name, angle in migrated.items()}
        a, b, c = old.forward_kinematics(retained_old), new.forward_kinematics(migrated), installed.forward_kinematics(migrated)
        for name in a:
            np.testing.assert_allclose(a[name], b[name], atol=2e-12)
            np.testing.assert_allclose(b[name], c[name], atol=2e-12)


@pytest.mark.parametrize("side", ["left", "right"])
def test_customer_cad_limits_match_all_hardware_command_endpoints(side):
    profile = load_hand_profile("TsenseI_v3_2_withid")
    limits = profile.joint_limits(side)
    device = A32Device(side, model_id=profile.model_id)
    prefix = f"{side}_hand_"
    expected = {
        "thumb_rota_block_joint": math.radians(130),
        "thumb_rotaback_joint2": math.radians(100),
        "thumb_MCP_joint": math.pi / 2,
        "thumb_PIP_joint": math.pi / 2,
    }
    for suffix, upper in expected.items():
        assert limits[prefix + suffix] == pytest.approx((0., upper), abs=1e-14)
    for finger in ("index", "mid", "ring", "pinky"):
        assert limits[prefix + finger + "_swing_joint"] == pytest.approx((-math.pi / 9, math.pi / 9))
        for part in ("MCP", "PIP", "DIP"):
            assert limits[prefix + finger + "_" + part + "_joint"] == (0., 1.57)
    endpoints = {name: hi for name, (lo, hi) in limits.items()}
    result = device.prepare_targets(endpoints, clamp=True)
    assert result["mit_pos"] == pytest.approx([100., 130.])
    assert result["applied_pose"][prefix + "thumb_MCP_joint"] == math.pi / 2
    assert result["applied_pose"][prefix + "thumb_PIP_joint"] == math.pi / 2
    assert result["applied_pose"] == pytest.approx(endpoints, abs=1e-12)
    for name, bounds in limits.items():
        assert device.joint_limits()[name] == pytest.approx(bounds, abs=1e-12)
    assert device._backend is None


@pytest.mark.parametrize("side", ["left", "right"])
def test_bare_hardware_rotary_limits_are_preserved(side):
    device = A32Device(side, model_id="TsenseI_v3_2")
    limits = device.joint_limits()
    assert limits[f"{side}_hand_thumb_rota_block_joint"] == pytest.approx((0., math.radians(119)))
    assert limits[f"{side}_hand_thumb_rotaback_joint2"] == pytest.approx((0., math.radians(99)))
    for name, bounds in limits.items():
        if is_finger_swing(name):
            assert bounds == pytest.approx((-.34, .34))


@pytest.mark.parametrize("side", ["left", "right"])
def test_customer_cad_synchronization_preserves_unmarked_source_joint_data(side):
    profile = load_hand_profile("TsenseI_v3_2_withid")
    root = ET.parse(profile.path("urdf", side)).getroot()
    for key in ("a32_joint_coordinates", "a32_swing_axis_alignment", "a32_thumb_zero_alignment"):
        root.attrib.pop(key, None)
    root.set("a32_cad_source", WITHID_CAD_VERSION)
    before = [ET.tostring(joint) for joint in root.findall("joint")]
    apply_urdf_coordinates(root, model_id=profile.model_id)
    assert [ET.tostring(joint) for joint in root.findall("joint")] == before


@pytest.mark.parametrize("old_angle", [-.35, -.2, 0., .2, .35])
def test_signed_motion_migration_clamps_only_old_swing_endpoints(old_angle):
    limits = {"left_hand_index_swing_joint": (-math.pi / 9, math.pi / 9),
              "left_hand_thumb_rota_block_joint": (0., math.radians(130)),
              "left_hand_thumb_MCP_joint": (0., math.pi / 2)}
    pose = {name: old_angle if is_finger_swing(name) else hi for name, (lo, hi) in limits.items()}
    before = dict(pose)
    migrated = migrate_signed_pose(pose, limits)
    assert pose == before
    assert migrated["left_hand_index_swing_joint"] == min(math.pi / 9, max(-math.pi / 9, old_angle))
    assert migrated["left_hand_thumb_rota_block_joint"] == math.radians(130)
    assert migrated["left_hand_thumb_MCP_joint"] == math.pi / 2
    assert migrate_signed_pose(migrated, limits) == migrated
    for invalid in (-.351, .351, float("nan"), True):
        with pytest.raises(ValueError):
            migrate_signed_pose({**pose, "left_hand_index_swing_joint": invalid}, limits)
    with pytest.raises(ValueError):
        migrate_signed_pose({**pose, "left_hand_thumb_MCP_joint": 1.58}, limits)
