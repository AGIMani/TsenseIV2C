"""WebXR metric landmarks through existing solvers; no physical devices."""
from pathlib import Path
import json
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]

from online_control.online_motion_track.main import StaleFrameError, WebSession
from utils.mediapipe_frame import apply_mediapipe_transformations


def webxr_packet(world, side="left", sequence=0):
    return {"sequence": sequence, "timestamp_ms": 1000 + sequence * 50,
            "capture_unix_ms": 1789600000000 + sequence * 50, "source": "webxr",
            "source_metadata": {"coordinate_space": "webxr_wrist_relative", "unit": "m",
                                "native_joint_count": 25, "normalized_is_image": False},
            "raw_webxr": {"type": "hand_skeleton", "joint_set": "webxr-hand-25",
                          "sequence": sequence, "hands": [{"handedness": side, "joints": []}]},
            # No normalized image coordinates exist for WebXR.
            "hand": {"label": side.title(), "world": world.tolist()}}


def rows(path):
    return [json.loads(line) for line in Path(path).read_text(encoding="utf-8").splitlines()]


@pytest.mark.parametrize("model_id", ["TsenseI_v3_0", "TsenseI_v3_0_withid",
                                     "TsenseI_v3_2", "TsenseI_v3_2_withid"])
@pytest.mark.parametrize("side", ["left", "right"])
def test_webxr_metric_geometry_and_recording_across_profiles(model_id, side, tmp_path):
    session = WebSession(side, True, tmp_path, model_id=model_id, input_source="webxr")
    world = session.pipeline.session.retargeter.fk.compute() * .75
    request = webxr_packet(world, side)
    try:
        result = session.process(request)
        assert result["tracking"], result["message"]
        assert result["source"] == "webxr" and result["depth_source"] == "webxr_metric"
        assert "WebXR" in result["message"]
        topo = np.asarray(result["points"])
        np.testing.assert_allclose(topo, apply_mediapipe_transformations(world, side), atol=1e-8)
        # Canonicalization is a rotation, with neither bone normalization nor reflection.
        np.testing.assert_allclose(np.linalg.norm(topo[1:] - topo[:-1], axis=1),
                                   np.linalg.norm(world[1:] - world[:-1], axis=1), atol=1e-8)
        lost = webxr_packet(world, side, 1)
        lost["hand"] = None
        lost["raw_webxr"] = {"type": "tracking_lost", "handedness": side}
        held = session.process(lost)
        assert not held["tracking"] and held["retarget_joint_angles_rad"] is None
        assert held["pose"] == result["pose"]
    finally:
        session.close()
    records = rows(session.recorder.path)
    assert records[0]["input_source"] == "webxr"
    assert records[0]["camera_source"] == "quest3-webxr"
    tracked, missing = [row for row in records if row["type"] == "frame"]
    assert tracked["depth_source"] == "webxr_metric"
    assert tracked["raw_webxr"] == request["raw_webxr"]
    assert tracked["source_metadata"] == request["source_metadata"]
    assert tracked["raw_world_points_21_m"] == world.tolist()
    assert tracked["raw_normalized_points_21"] is None
    assert tracked["smoothed_normalized_points_21"] is None
    assert tracked["camera_size"] is None
    assert missing["input_source"] == "webxr" and missing["depth_source"] == "none"
    assert missing["raw_webxr"] == lost["raw_webxr"]


def test_switching_source_resets_metric_filter_and_keeps_mediapipe_default(tmp_path):
    session = WebSession(recording=True, log_dir=tmp_path)
    world = session.pipeline.session.retargeter.fk.compute()
    try:
        camera = webxr_packet(world)
        camera.pop("source")
        camera["hand"]["normalized"] = np.zeros((21, 3)).tolist()
        result = session.process(camera)
        assert result["source"] == "mediapipe" and result["depth_source"] == "mediapipe_world"
        quest = webxr_packet(world * .65, sequence=1)
        result = session.process(quest)
        np.testing.assert_allclose(result["points"], apply_mediapipe_transformations(world * .65), atol=1e-8)
    finally:
        session.close()
    records = rows(session.recorder.path)
    frames = [row for row in records if row["type"] == "frame"]
    assert frames[0]["raw_normalized_points_21"] == camera["hand"]["normalized"]
    assert frames[1]["raw_normalized_points_21"] is None


@pytest.mark.parametrize("side", ["left", "right"])
def test_v2_webxr_bypasses_wuji_and_preserves_control_gates(side, tmp_path, monkeypatch):
    pytest.importorskip("ruckig")
    from online_control.online_motion_track.v2 import session as module
    from test_web_v2 import MemoryUsb
    monkeypatch.setattr(module, "UsbController", MemoryUsb)
    def unexpected_wuji(*args, **kwargs):
        pytest.fail("Metric WebXR bones must never be normalized by Wuji")
    monkeypatch.setattr(module, "preprocess_wuji_world_points", unexpected_wuji)
    session = module.V2WebSession(side, True, tmp_path, collision_mode="off", input_source="webxr")
    world = session.solver.fk.compute() * .8
    if side == "right":
        world[:, 0] *= -1
    observed = []
    original_observe = session.control.observe_tracking_frame
    def observe(valid, **kwargs):
        observed.append((valid, kwargs.get("reason")))
        return original_observe(valid, **kwargs)
    monkeypatch.setattr(session.control, "observe_tracking_frame", observe)
    try:
        session.command("tracking_start", {})
        for i in range(5):
            result = session.process(webxr_packet(world, side, i))
        assert result["tracking"], result["message"]
        np.testing.assert_allclose(result["points"], apply_mediapipe_transformations(world, side), atol=1e-8)
        assert result["depth_source"] == "webxr_metric"
        assert session.control.snapshot()["target_ready"]
        with pytest.raises(StaleFrameError):
            session.process(webxr_packet(world, side, 3))
        assert session.control.snapshot()["target_ready"]
        lost = webxr_packet(world, side, 5)
        lost["hand"] = None
        lost["raw_webxr"] = {"type": "tracking_lost", "handedness": side}
        assert not session.process(lost)["tracking"]
        assert observed[-1] == (False, "hand_missing")
        wrong = webxr_packet(world, "right" if side == "left" else "left", 6)
        assert not session.process(wrong)["tracking"]
        assert not session.control.snapshot()["target_ready"]
        assert not session.control.snapshot()["armed"]
    finally:
        session.close()
    native = rows(session.recorder.path)
    assert native[0]["metadata"]["input_source"] == "webxr"
    assert {"webxr_raw", "webxr", "retarget", "control_target"} <= {r["type"] for r in native}
    assert not {"mediapipe", "mediapipe_raw"} & {r["type"] for r in native}
    frames = [row for row in rows(session.frame_logger.path) if row["type"] == "frame"]
    assert all(frame["wuji_points_21_m"] is None for frame in frames)
    assert all(frame["raw_normalized_points_21"] is None for frame in frames)
    accepted = next(frame for frame in frames if frame["retarget_ran"])
    np.testing.assert_allclose(accepted["source_points_21_m"], world - world[0], atol=1e-8)
    assert frames[-2]["raw_webxr"] == lost["raw_webxr"]


def test_a30_webxr_loss_uses_existing_hold_and_does_not_clear_fault(monkeypatch):
    pytest.importorskip("ruckig")
    from online_control.online_motion_track.a30 import session as module
    from online_control.online_motion_track.a30.control import A30Control
    monkeypatch.setattr(module, "A30Control", lambda profile, side, mode, **kwargs:
                        A30Control(profile, side, mode, start=False, **kwargs))
    session = module.A30WebSession(collision_mode="off", input_source="webxr")
    world = session.pipeline.session.retargeter.fk.compute()
    try:
        assert session.control.preview_only
        assert session.process(webxr_packet(world))["tracking"]
        previous = dict(session.control.target)
        lost = webxr_packet(world, sequence=1)
        lost["hand"] = None
        assert not session.process(lost)["tracking"]
        assert session.control.phase == "holding" and session.control.target == previous
        broken = webxr_packet(np.zeros((21, 3)), sequence=2)
        assert not session.process(broken)["tracking"]
        assert not session.control.valid
        lost["sequence"], lost["timestamp_ms"] = 3, 1150
        session.process(lost)
        assert not session.control.valid
    finally:
        session.close()


def test_a32_webxr_native_recording_and_lost_hand_hold(tmp_path, monkeypatch):
    pytest.importorskip("ruckig")
    from test_a32_tracking_recording import make_session, records
    session, _ = make_session(tmp_path, monkeypatch)
    session.input_source = "webxr"
    world = session.pipeline.session.retargeter.fk.compute()
    try:
        session.command("connect", {"port": "SIM_ONLY"})
        request = webxr_packet(world)
        assert session.process(request)["tracking"]
        session.command("arm", {"enabled": True})
        # Arming seeds a hold pose. Accept a fresh target afterwards so loss
        # cannot accidentally pass by observing only the arm-time reset.
        assert session.process(webxr_packet(world, sequence=1))["tracking"]
        accepted_target = dict(session.control.target)
        accepted_at = session.control.target_at
        assert accepted_at > 0.
        lost = webxr_packet(world, sequence=2)
        lost["hand"] = None
        assert not session.process(lost)["tracking"]
        assert session.control.target_at == accepted_at
        assert session.control.target == accepted_target
        state = session.status()
        assert state["armed"] and state["target_ready"]
        assert state["tracking_transition"]["phase"] == "holding"
        assert not state["last_error"]
        # A3.2 retains the last checked target on loss. Observation freshness
        # expires independently; it is not a fault or a disabled target.
        from online_control.online_motion_track.a32 import control as control_module
        with monkeypatch.context() as patch:
            patch.setattr(control_module.time, "monotonic",
                          lambda: accepted_at + session.control.INPUT_TIMEOUT + .01)
            expired = session.status()
        assert not expired["hand_recent"]
        assert expired["armed"] and expired["target_ready"]
        assert session.control.target_at == accepted_at
        assert session.control.target == accepted_target
    finally:
        session.close()
    saved = records(session)
    assert saved[0]["metadata"]["camera_source"] == "quest3-webxr"
    assert {"webxr_raw", "webxr", "retarget", "control_target"} <= {r["type"] for r in saved}
    assert not {"mediapipe", "mediapipe_raw"} & {r["type"] for r in saved}
    raw = next(row for row in saved if row["type"] == "webxr_raw")
    assert raw["raw_webxr"] == request["raw_webxr"]
    assert raw["source_metadata"] == request["source_metadata"]
    topo = next(row for row in saved if row["type"] == "webxr")
    assert topo["depth_source"] == "webxr_metric"
