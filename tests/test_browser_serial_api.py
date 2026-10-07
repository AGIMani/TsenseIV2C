"""Public sessions expose computation only, including browser pose rebasing."""
import sys
import struct
from pathlib import Path

import pytest

sys.path[:0] = [str(Path(__file__).resolve().parents[1]), str(Path(__file__).resolve().parents[1] / "src")]
from online_control.online_motion_track.main import Application
from test_web_v2 import packet


@pytest.mark.parametrize("side", ["left", "right"])
def test_public_browser_rebase_is_finite_complete_and_never_arms(tmp_path, side):
    app = Application(tmp_path, public=True)
    try:
        desc = app.create({"model_id": "TsenseI_v2", "side": side, "recording": False, "collision_mode": "stop"})
        token = desc["session"]
        s = app.get(token)
        command = lambda action, **kw: app.v2_command({"session": token, "action": action, **kw})
        pose = desc["init_pose"]
        assert len(pose) == len(desc["joint_limits"]) == 21
        command("tracking_start")
        state = command("browser_rebase", pose=pose)
        assert state["browser_epoch"] == 1
        assert state["ruckig_joint_angles_rad"] == pose
        assert not state["armed"] and not state["connected"] and not state["target_ready"]
        for invalid in ({}, {**pose, "unknown": 0}, {**pose, next(iter(pose)): float("nan")},
                        {**pose, next(iter(pose)): 100}, {**pose, f"{side}_hand_pinky_rota_joint": .1}):
            with pytest.raises(ValueError):
                command("browser_rebase", pose=invalid)
        for i in range(5):
            result = s.process(packet(s, i))
        assert result["retarget_joint_angles_rad"]
        assert result["collision_safe_target_joint_angles_rad"]
        for action in ("ports", "home", "connect", "arm"):
            with pytest.raises(ValueError, match="公共服务"):
                command(action)
        assert not list(tmp_path.rglob("*.jsonl"))
    finally:
        app.close()


@pytest.mark.parametrize("side", ["left", "right"])
def test_partial_homing_uses_feedback_pose_instead_of_assuming_zero(tmp_path, side):
    app = Application(tmp_path, public=True)
    try:
        desc = app.create({"model_id": "TsenseI_v2", "side": side, "recording": False, "collision_mode": "stop"})
        payload = bytearray(780)
        payload[457] = int(side == "right")
        # Index swing feedback is nonzero even though home_label remains 0.
        struct.pack_into("<f", payload, 456 + 24, 20.0)
        command = lambda feedback: app.v2_command({"session": desc["session"], "action": "browser_rebase", "telemetry": feedback})
        state = command(payload.hex())
        assert state["ruckig_joint_angles_rad"][f"{side}_hand_index_swing_joint"] > .1
        assert not state["connected"] and not state["armed"]
        struct.pack_into("<f", payload, 12, float("nan"))
        with pytest.raises(ValueError, match="无效数值"):
            command(payload.hex())
        with pytest.raises(ValueError):
            command("not a telemetry payload")
    finally:
        app.close()
