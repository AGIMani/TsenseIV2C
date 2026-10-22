"""The supplied CAD defines thumb zero; origin calibration never moves motors."""
from pathlib import Path
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src"), str(ROOT / "tools")]
from a32_driver import A32Device
from a32_kinematics import A32StrokeMapper
from a32_kinematics.joint_coordinates import (
    THUMB_ZERO_REFERENCE, THUMB_ZERO_REFERENCE_SHA256, THUMB_ZERO_ALIGNMENT,
    LEGACY_THUMB_ZERO_ALIGNMENT, LEGACY_THUMB_ROTABACK_ORIGIN_OFFSET, THUMB_ORIGIN_OFFSETS,
)
from hand_profiles import A32_MODEL_IDS, load_hand_profile
from TsenseI_retargeting.fk.urdf import UrdfModel
from apply_a32_joint_ranges import apply_urdf_coordinates
from measure_a32_thumb_zero import measure


def test_measured_offsets_reproduce_reference_and_export_precision():
    report = measure()
    assert report["reference_sha256"] == THUMB_ZERO_REFERENCE_SHA256
    assert report["thumb_middle_opening_deg"] == pytest.approx(46.893124983914774, abs=1e-9)
    for model_id, result in report["models"].items():
        assert result["orientation_residual_deg"] < .001
        for name, angle in result["origin_offsets_rad"].items():
            suffix = name.removeprefix("left_hand_")
            assert THUMB_ORIGIN_OFFSETS[model_id][suffix] == pytest.approx(angle, abs=1e-8)


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
@pytest.mark.parametrize("side", ["left", "right"])
def test_zero_thumb_matches_reference_orientation_and_preserves_mirror(model_id, side):
    ref = UrdfModel(ROOT / THUMB_ZERO_REFERENCE)
    profile = load_hand_profile(model_id)
    model = UrdfModel(profile.path("urdf", side))
    a, b = ref.forward_kinematics({}), model.forward_kinematics({})
    palm = "Left_hand_palm_link" if side == "left" else "Right_hand_palm_link"
    mirror = np.eye(3) if side == "left" else np.diag([-1, 1, 1] if model_id == "TsenseI_v3_2" else [1, -1, 1])
    for part in ("MCP", "PIP"):
        target = a["Left_hand_palm_link"][:3, :3].T @ a[f"left_hand_thumb_{part}_link"][:3, :3]
        actual = b[palm][:3, :3].T @ b[f"{side}_hand_thumb_{part}_link"][:3, :3]
        error = Rotation.from_matrix(actual.T @ mirror @ target @ mirror).magnitude()
        assert math.degrees(error) < .001
    if model_id == "TsenseI_v3_2":
        reflection = np.eye(4)
        reflection[:3, :3] = mirror
        for name, transform in a.items():
            if "thumb" in name:
                np.testing.assert_allclose(b[name.replace("left_hand_", f"{side}_hand_")],
                                           reflection @ transform @ reflection, atol=2e-12)
    root = ET.parse(profile.path("urdf", side)).getroot()
    assert root.get("a32_thumb_zero_alignment") == THUMB_ZERO_ALIGNMENT
    before = ET.tostring(root)
    assert ET.tostring(apply_urdf_coordinates(root, model_id=model_id)) == before


@pytest.mark.parametrize("side", ["left", "right"])
def test_previous_stop_contact_zero_is_undone_once(side):
    profile = load_hand_profile("TsenseI_v3_2")
    root = ET.parse(profile.path("urdf", side)).getroot()
    name = f"{side}_hand_thumb_rotaback_joint2"
    joint = root.find(f"./joint[@name='{name}']")
    origin = joint.find("origin")
    expected = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy"), sep=" "))
    axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
    previous = expected * Rotation.from_rotvec(axis * LEGACY_THUMB_ROTABACK_ORIGIN_OFFSET)
    origin.set("rpy", " ".join(map(str, previous.as_euler("xyz"))))
    root.set("a32_thumb_zero_alignment", LEGACY_THUMB_ZERO_ALIGNMENT)
    apply_urdf_coordinates(root, model_id=profile.model_id)
    actual = Rotation.from_euler("xyz", np.fromstring(origin.get("rpy"), sep=" "))
    np.testing.assert_allclose(actual.as_matrix(), expected.as_matrix(), atol=1e-14)
    before = ET.tostring(root)
    assert ET.tostring(apply_urdf_coordinates(root, model_id=profile.model_id)) == before


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("angle", [0., .6, 1.74])
def test_origin_calibration_does_not_offset_motor_commands(model_id, side, angle):
    profile = load_hand_profile(model_id)
    names = (f"{side}_hand_thumb_rotaback_joint2", f"{side}_hand_thumb_rota_block_joint")
    pose = profile.init_pose(side)
    pose.update(dict.fromkeys(names, angle))
    result = A32StrokeMapper(side, model_id=model_id).convert(pose)
    device = A32Device(side, model_id=model_id)
    targets = device.prepare_targets(pose)
    for name in names:
        assert result.rotary_positions_deg[name] == pytest.approx(math.degrees(angle), abs=1e-10)
        channel = next(c for c in device.channels if c["joint"] == name)
        value = targets["mit_pos"][channel["index"]]
        expected = min(channel["limits"][1], math.degrees(angle))
        assert value == pytest.approx(expected, abs=1e-10)
        assert device._inverse(channel, value) == pytest.approx(math.radians(expected), abs=1e-10)
    assert device._backend is None
