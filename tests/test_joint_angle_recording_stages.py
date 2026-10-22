"""Only real pipeline stages produce joint-angle series; diagnostics stay intact."""
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]

from a32_driver.recording import A32SessionRecorder
from utils.control_session_recorder import ControlSessionRecorder


def rows(path):
    return [json.loads(line) for line in path.read_text().splitlines()]


def test_sdk_angle_requires_confirmed_success_and_uses_applied_joint_pose(tmp_path):
    recorder = A32SessionRecorder(tmp_path)
    path = recorder.start()
    joint = "left_hand_index_MCP_joint"
    targets = {"applied_pose": {joint: .2}, "self": [1.] * 18, "mit_pos": [2.] * 2}
    recorder.record_command(targets, pose={joint: .5}, frame_id=8, sent=False, error="failed")
    recorder.record_command(targets, pose={joint: .5}, frame_id=9, sent=True)
    recorder.stop()
    events = rows(path)
    angles = [row for row in events if row["type"] == "joint_angles"]
    assert len(angles) == 1
    assert angles[0]["stage"] == "sdk" and angles[0]["sent"] is True
    assert angles[0]["joints"] == {joint: .2} and angles[0]["frame_id"] == 9
    writes = [row for row in events if row["type"] == "downlink"]
    assert [row["sent"] for row in writes] == [False, True]
    assert writes[0]["sdk_targets"] == targets
    assert len([row for row in events if row["type"] == "conversion"]) == 2


def test_stage_events_do_not_drop_retarget_and_collision_context(tmp_path):
    recorder = ControlSessionRecorder()
    path = recorder.start(output_dir=tmp_path)
    pose = {"right_hand_index_MCP_joint": .3}
    recorder.record_retarget(pose, frame_id=5, hand_side="right", solve_ms=3.2,
                             landmark_repairs=[{"finger": "index"}])
    recorder.record_stage("control_target", pose, frame_id=5, hand_side="right",
                          self_collision={"limited": True})
    recorder.record_joint_angles("sdk", pose, frame_id=5, hand_side="right", sent=False)
    recorder.record_joint_angles("sdk", pose, frame_id=5, hand_side="right")
    recorder.record_stage("direct", pose, frame_id=5, hand_side="right")
    recorder.record_h7(joints_ui=pose, joints_hw=pose, frame_id=5, hand_side="right",
                       sent=False, usb_connected=True, joints_ruckig_ui=None)
    recorder.stop()
    events = rows(path)
    assert [row["stage"] for row in events if row["type"] == "joint_angles"] == ["retarget", "direct"]
    context = next(row for row in events if row["type"] == "retarget")
    assert "joints" not in context and context["solve_ms"] == 3.2 and context["landmark_repairs"]
    collision = next(row for row in events if row["type"] == "control_target")
    assert "joints" not in collision and collision["self_collision"]["limited"]


def test_ruckig_capture_precedes_collision_replanning():
    ruckig = pytest.importorskip("ruckig")
    from online_control.online_motion_track.safe_trajectory import SafeTrajectoryMixin

    joint = "left_hand_index_MCP_joint"
    captured = []
    class Trajectory(SafeTrajectoryMixin):
        def record_ruckig_output(self, pose):
            captured.append(dict(pose))

    item = Trajectory()
    item.names, item.side, item.frame_id = [joint], "left", 3
    item.pose, item.target = {joint: 0.}, {joint: .2}
    item.limits = {joint: (0., 1.)}
    item.input = SimpleNamespace(current_position=[0.], current_velocity=[0.],
                                 current_acceleration=[0.], target_position=[.2])
    item.output = SimpleNamespace(new_position=[0.], trajectory=SimpleNamespace(
        position_extrema=[SimpleNamespace(min=0., max=.2)]))
    def update(input, output):
        output.new_position = list(input.target_position)
        return ruckig.Result.Working
    item.otg = SimpleNamespace(update=update)
    item.guard = SimpleNamespace(project=lambda pose: {joint: 0.},
        diagnostics={"contacts": ["index"]}, guard=SimpleNamespace(_contact_fingers=lambda contacts: {"index"}))
    item.finger_holds, item.finger_hold_context, item.sample_collision = {}, {}, {}
    safe = item._checked_step(.002)
    assert safe == {joint: 0.}
    assert item.ruckig_output_joint_angles_rad == {joint: .2}
    assert captured == [{joint: .2}]  # Do not relabel the collision-held sample.
