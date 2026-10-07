"""A3 direct targets bypass smoothing/collision; limits and mode epochs remain."""
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from hand_profiles import load_hand_profile
from online_control.online_motion_track.a30.control import A30Control
from online_control.online_motion_track.a30.session import A30WebSession
from online_control.online_motion_track.main import Application, WebSession


@pytest.mark.parametrize("model", ["TsenseI_v3_0", "TsenseI_v3_0_withid"])
@pytest.mark.parametrize("side", ["left", "right"])
def test_switch_revokes_old_stream_and_direct_target_arrives_in_one_tick(model, side):
    control = A30Control(load_hand_profile(model), side, "off", start=False)
    joint = f"{side}_hand_mid_DIP_joint"
    try:
        control.step(.002)
        stream = control.open_stream(control.epoch)
        assert len(next(stream)) == 216
        control.set_direct_mode(True)
        assert list(stream) == []
        control.rebase(bytes(484))
        target = {**control.pose, joint: .7}
        control.push(target, 12)
        control.step(.002)
        assert control.valid, control.error
        assert control.pose == target
        assert control.snapshot()['direct_filter'] is None
        assert control.snapshot()['direct_mode'] and not control.snapshot()['ruckig_enabled']
        assert not control.snapshot()['motion_limits_enabled']
        for preview in (False, True):
            stream = control.open_stream(control.epoch, preview=preview)
            assert len(next(stream)) == (108 if preview else 216)
            stream.close()
        control.set_direct_mode(False)
        assert control.target_at == 0 and control.frame_id == control.sample_frame_id == -1
        control.push({**target, joint: 0.}, 13)
        control.step(.002)
        assert control.valid, control.error
        assert 0 < control.pose[joint] < .7
        assert control.snapshot()["ruckig_enabled"]
    finally:
        control.close()


@pytest.mark.parametrize("side", ["left", "right"])
def test_a30_direct_response_is_target_not_trajectory(monkeypatch, side):
    session = A30WebSession(side, collision_mode="off")
    try:
        target = session.pipeline.session.profile.init_pose(side)
        target[f"{side}_hand_mid_DIP_joint"] = .7
        monkeypatch.setattr(WebSession, "process", lambda self, packet: {
            "tracking": True, "pose": dict(target), "sequence": 1,
            "collision_safe_target_joint_angles_rad": dict(target), "actuation": {"direct": True}})
        session.command("set_direct_mode", {"enabled": True})
        result = session.process({})
        assert result["pose"] == target and result["actuation"] == {"direct": True}
        assert not result["hardware"]["ruckig_enabled"]
        assert result["hardware"]["target_joint_angles_rad"] == target
        session.command("set_direct_mode", {"enabled": False})
        assert not session.status()["direct_mode"]
    finally:
        session.close()


@pytest.mark.parametrize("model", ["TsenseI_v3_0", "TsenseI_v2"])
def test_direct_session_allows_rebase_and_survives_tracking_restart(model):
    app = Application()
    try:
        created = app.create({"model_id": model, "collision_mode": "off", "recording": False, "direct_mode": True})
        session = app.get(created["session"])
        assert created["hardware"]["direct_mode"]
        session.command("tracking_start", {})
        assert not session.status()["ruckig_enabled"]
        if model == "TsenseI_v3_0":
            session.command("browser_rebase", {"telemetry": bytes(484).hex()})
        if model == "TsenseI_v2":
            session.command("browser_rebase", {"pose": session.profile.init_pose()})
            q = session.solver.fk.compute().tolist()
            for i in range(10):
                result = session.process({"sequence": i+1, "timestamp_ms": 1000+i*33, "capture_unix_ms": 0,
                    "hand": {"label": "Left", "score": .99, "world": q, "normalized": q}})
            assert result["tracking"], result["message"]
            assert result["pose"] == result["collision_safe_target_joint_angles_rad"]
        session.command("set_direct_mode", {"enabled": False})
        assert session.status()["ruckig_enabled"]
        assert not session.status()["armed"]
    finally:
        app.close()


@pytest.mark.parametrize("model", ["TsenseI_v3_0", "TsenseI_v3_0_withid"])
@pytest.mark.parametrize("side", ["left", "right"])
def test_direct_step_bypasses_collision_but_preserves_finite_and_joint_limits(model, side, monkeypatch):
    control = A30Control(load_hand_profile(model), side, "stop", start=False)
    joint = f"{side}_hand_mid_DIP_joint"
    original_guard = control.guard
    def forbidden(*args, **kwargs):
        pytest.fail("A3 direct command entered trajectory or collision processing")
    monkeypatch.setattr(original_guard, "project", forbidden)
    monkeypatch.setattr(control, "_checked_step", forbidden)
    try:
        control.set_direct_mode(True)
        assert control.guard is None
        low, high = control.limits[joint]
        for frame_id, requested in enumerate((high + 1., low - 1., .7)):
            control.push({**control.pose, joint: requested}, frame_id)
            accepted = dict(control.target)
            assert accepted[joint] == min(high, max(low, requested))
            control.step(.002)
            assert control.valid, control.error
            assert control.pose == accepted
            assert len(control.wire) == 108
        before = (dict(control.target), control.frame_id, control.wire)
        for invalid in (float('nan'), float('inf'), -float('inf')):
            with pytest.raises(ValueError):
                control.push({**control.pose, joint: invalid}, 99)
            assert (control.target, control.frame_id, control.wire) == before
        assert not control.snapshot()['sample_collision']['checked']
        control.target[joint] = float('nan')
        control.step(.002)
        assert not control.valid and control.wire == b''
        control.set_direct_mode(False)
        assert control.guard is original_guard
        assert control.snapshot()['ruckig_enabled']
    finally:
        control.close()


def test_direct_sample_keeps_original_frame_id_when_new_target_arrives():
    import struct
    control = A30Control(load_hand_profile("TsenseI_v3_0"), "left", "off", start=False)
    try:
        control.set_direct_mode(True)
        control.push(dict(control.pose), 12)
        control.step(.002)
        control.push({**control.pose,'left_hand_mid_DIP_joint':.7}, 13)
        stream = control.open_stream(control.epoch)
        try:
            assert struct.unpack_from('<i',next(stream),12)[0] == 12
        finally:
            stream.close()
        assert control.snapshot()['sample_frame_id'] == 12
        assert control.snapshot()['frame_id'] == 13
    finally:
        control.close()
