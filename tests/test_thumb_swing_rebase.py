"""Rebasing must preserve physical poses, mirrored geometry and wire commands."""
from pathlib import Path
import math
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from protocol.hand_angle_protocol import (
    UI_RANGES, convert_ui_to_hardware, remap_motion_cfg_file_to_ui,
    remap_motion_cfg_ui_to_file, thumb_swing_hw_to_ui, thumb_swing_ui_to_hw,
)
from protocol.hand_display_protocol import convert_ui_to_display
from utils.hand_init import get_init_pose
from usb.offline import ROTARY_JOINT_SLOTS, joint_angles_to_hand_positions

JOINT = "left_hand_thumb_swing_joint"

def thumb_joint(side):
    path = ROOT / f"assets/hands/TsenseI_hand/urdf/TsenseI__hand_{side}_v1.urdf"
    return ET.parse(path).find(f"joint[@name='{side}_hand_thumb_swing_joint']")

@pytest.mark.parametrize("side,old_axis,old_yaw", [("left", -1, -.29927), ("right", 1, .29927)])
@pytest.mark.parametrize("old_q", [-1, -.4, 0, .6, 1])
def test_urdf_rebase_preserves_joint_transform(side, old_axis, old_yaw, old_q):
    joint = thumb_joint(side)
    origin = joint.find("origin")
    axis = np.fromstring(joint.find("axis").get("xyz"), sep=" ")
    rpy = np.fromstring(origin.get("rpy"), sep=" ")
    actual = Rotation.from_euler("xyz", rpy).as_matrix() @ Rotation.from_rotvec(axis * (1-old_q)).as_matrix()
    expected = Rotation.from_euler("z", old_yaw + old_axis * old_q).as_matrix()
    np.testing.assert_allclose(actual, expected, atol=1e-12)
    np.testing.assert_allclose(np.fromstring(origin.get("xyz"), sep=" "), [-.0003, -.00885 * old_axis, .03775])
    assert (float(joint.find("limit").get("lower")), float(joint.find("limit").get("upper"))) == UI_RANGES["thumb_swing"] == (0, 2)

def test_mjcf_matches_rebased_left_urdf():
    root = ET.parse(ROOT / "assets/hands/TsenseI_hand/mjcf/mjmodel.xml")
    body = root.find(".//body[@name='left_hand_thumb_swing_link']")
    quat = np.fromstring(body.get("quat"), sep=" ")
    actual = Rotation.from_quat(np.roll(quat, -1)).as_matrix()
    expected = Rotation.from_euler("z", -1.29927).as_matrix()
    np.testing.assert_allclose(actual, expected, atol=2e-12)
    assert body.find("joint").get("axis") == "0 0 1"
    assert body.find("joint").get("range") == "0 2"
    assert root.find(".//position[@joint='left_hand_thumb_swing_joint']").get("ctrlrange") == "0 2"

@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("file_value", [-1, -.4, 0, .6, 1])
def test_legacy_motion_keeps_pose_and_h7_r05(side, file_value):
    old_ui = file_value if side == "left" else -file_value
    expected = 1-old_ui
    ui = remap_motion_cfg_file_to_ui({JOINT: file_value}, side)
    assert ui[JOINT] == pytest.approx(expected)
    display = convert_ui_to_display(ui, side)
    assert display[f"{side}_hand_thumb_swing_joint"] == pytest.approx(expected)
    hardware = convert_ui_to_hardware(ui, side)
    _, rotary = joint_angles_to_hand_positions(hardware)
    assert rotary[ROTARY_JOINT_SLOTS[JOINT]] == pytest.approx(math.degrees(expected))
    assert thumb_swing_ui_to_hw(expected, side) == pytest.approx(expected)
    assert thumb_swing_hw_to_ui(expected, side) == pytest.approx(expected)
    assert remap_motion_cfg_ui_to_file(ui, side)[JOINT] == pytest.approx(file_value)

def test_init_is_zero_for_both_hands():
    assert get_init_pose("left")[JOINT] == 0
    assert get_init_pose('right')['right_hand_thumb_swing_joint'] == 0

def test_ruckig_reset_starts_at_zero_init():
    from utils.ruckig_interpolator import RuckigJointInterpolator
    interpolator = RuckigJointInterpolator()
    assert interpolator._inputs[JOINT].current_position == [0.0]
    interpolator.reset()
    assert interpolator._inputs[JOINT].current_position == [0.0]
