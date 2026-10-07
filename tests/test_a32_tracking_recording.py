"""Viewer-compatible tracking logs, with no native transport or real log folder."""
import json
from datetime import datetime
from pathlib import Path
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
pytest.importorskip("ruckig")

from a32_driver import A32Device
from hand_profiles import A32_MODEL_IDS
from online_control.online_motion_track.a32 import session as session_module
from online_control.online_motion_track.a32.control import A32Control
from test_a32_driver import SimulatedNativeHand
from test_a32_tracking_control import PassGuard
from test_web_v3_2 import packet


def make_session(tmp_path, monkeypatch, model_id="TsenseI_v3_2", *, default_dir=False):
    backend = SimulatedNativeHand()
    device = A32Device(model_id=model_id, backend=backend)

    def control(profile, side, mode, **options):
        item = A32Control(profile, side, mode, device=device, start=False, **options)
        item.guard = PassGuard()
        item.start_tracking()
        return item

    monkeypatch.setattr(session_module, "A32Control", control)
    session = session_module.A32WebSession(log_dir=None if default_dir else tmp_path, model_id=model_id)
    return session, backend


def records(session):
    path = session.control.recorder.writer.path
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]


def test_input_failure_keeps_original_error_when_emergency_stop_also_fails(tmp_path, monkeypatch):
    session, _ = make_session(tmp_path, monkeypatch)
    original = ValueError("simulated solver failure")

    def fail_solve(*args):
        raise original

    def fail_stop():
        raise OSError("simulated estop I/O failure")

    try:
        session.command("connect", {"port": "SIM_ONLY"})
        session.control.armed = True
        with monkeypatch.context() as patch:
            patch.setattr(session_module.WebSession, "process", fail_solve)
            patch.setattr(session.control.device, "stop", fail_stop)
            with pytest.raises(ValueError) as error:
                session.process({"sequence": 17})
            assert error.value is original
            assert not session.control.armed and session.control.stop_pending
            assert "solver failure" in session.control.last_error
            assert "estop I/O failure" in session.control.last_error
    finally:
        session.close()
    row = next(row for row in records(session) if row["type"] == "sdk_error")
    assert row["stage"] == "骨架求解" and row["exception_type"] == "ValueError"
    assert row["frame_id"] == 17


def test_next_tracking_recording_respects_updated_viewer_parent(tmp_path, monkeypatch):
    parent = [tmp_path / "first-root"]
    monkeypatch.setattr("utils.control_session_recorder.control_log_parent", lambda: parent[0])
    session, _ = make_session(tmp_path, monkeypatch, default_dir=True)
    try:
        session.command("connect", {"port": "SIM_ONLY"})
        first = Path(session.status()["recording"]["path"])
        session.command("disconnect", {})
        parent[0] = tmp_path / "next-root"
        session.command("connect", {"port": "SIM_ONLY"})
        second = Path(session.status()["recording"]["path"])
        assert first.parent == tmp_path / "first-root" / "A32"
        assert second.parent == parent[0] / "A32"
        assert first.is_file() and second.is_file()
    finally:
        session.close()


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
def test_tracking_records_all_viewer_stages_and_wall_times(tmp_path, monkeypatch, model_id):
    session, backend = make_session(tmp_path, monkeypatch, model_id)
    try:
        assert not list(tmp_path.glob("*.jsonl"))
        state = session.command("connect", {"transport": "usb", "port": "SIM_ONLY"})
        status = state["recording"]
        assert status["active"] and Path(status["path"]).parent == tmp_path.resolve()
        assert backend.calls == [("usb", "SIM_ONLY")]
        world = session.pipeline.session.retargeter.fk.compute()
        request = packet(world)
        request["raw_detection"] = {"world": [world.tolist()],
            "normalized": [request["hand"]["normalized"]], "labels": [{"label": "Left", "score": .98}]}
        request["inference_ms"] = 4.2
        result = session.process(request)
        assert result["tracking"]
        session.command("arm", {"enabled": True})
        for _ in range(3):
            session.control.step(.005)
        session.command("disconnect", {})
        assert not session.status()["recording"]["active"]
    finally:
        session.close()
    rows = records(session)
    assert rows[0]["schema"] == "hand-control-session/v2"
    assert rows[0]["metadata"]["platform"] == "motion_track"
    assert rows[0]["metadata"]["model_id"] == model_id
    assert rows[-1]["reason"] == "disconnected"
    assert {"mediapipe_raw", "mediapipe", "retarget", "control_target", "conversion", "downlink",
            "h7", "telemetry", "sdk_status", "sdk_action"} <= {row["type"] for row in rows}
    raw = next(row for row in rows if row["type"] == "mediapipe_raw")
    assert raw["world_landmarks"] == [world.tolist()]
    assert raw["source_scope"] == "all_detected_hands"
    assert raw["capture_unix_ms"] == request["capture_unix_ms"]
    assert raw["infer_ms"] == 4.2
    writes = [row for row in rows if row["type"] == "downlink"]
    assert len(writes) == 4  # Enable seed plus each successful control sample.
    assert all(row["sent"] and row["frame_id"] == 0 for row in writes)
    assert all(len(row["linear_mm"]) == 14 and len(row["rotary_deg"]) == 6 for row in writes)
    assert all(isinstance(row["t"], (int, float)) and row["wall_time_unix_ms"] > 0 for row in rows)
    assert all(datetime.fromisoformat(row["wall_time"]).tzinfo is not None for row in rows)
    assert [row["t"] for row in rows] == sorted(row["t"] for row in rows)


def test_idle_worker_records_telemetry_without_browser_status(tmp_path, monkeypatch):
    session, backend = make_session(tmp_path, monkeypatch)
    try:
        session.command("connect", {"port": "SIM_ONLY"})
        backend.state["rx_ok"] += 1
        backend.state["self"][8]["online"] = False
        session.control.step(.005)
        backend.state["rx_ok"] += 1
        backend.state["self"][0]["position"] = .12
        session.control._recording_state_at = 0
        session.control.step(.005)
        assert not session.control.armed
        assert [call[0] for call in backend.calls] == ["usb"]
    finally:
        session.close()
    frames = [row for row in records(session) if row["type"] == "telemetry"]
    assert len(frames) == 3
    assert frames[-1]["linear"][0]["pos_act_mm"] == -.12
    assert frames[-1]["linear"][0]["position"] == .12
    assert frames[-1]["linear_feedback_sign"] == -1
    offline = next(row for row in frames[-1]["linear"] if row["slot"] == 8)
    assert not offline["measurement_valid"] and offline["pos_act_mm"] is None


def test_late_sample_records_elapsed_and_planner_time_separately(tmp_path, monkeypatch):
    session, _ = make_session(tmp_path, monkeypatch)
    try:
        session.command("connect", {"port": "SIM_ONLY"})
        session.control.push(session.control.device.constrain_pose(session.control.pose), 9)
        session.command("arm", {"enabled": True})
        session.control.step(.25)
        assert session.control.armed
        assert not session.control.recording_error
    finally:
        session.close()
    sample = next(row for row in records(session) if row["type"] == "h7" and row.get("source") == "a32_tracking")
    assert sample["sent"] and sample["control_dt_s"] == .25
    assert sample["control_period_ms"] == 250
    assert sample["planner_dt_s"] == .004
    assert sample['control_timing']['planner_lost_ms'] == pytest.approx(246.)


def test_failed_write_is_distinguished_from_successful_command(tmp_path, monkeypatch):
    session, backend = make_session(tmp_path, monkeypatch)
    try:
        session.command("connect", {"port": "SIM_ONLY"})
        session.control.push(session.control.device.constrain_pose(session.control.pose), 9)
        session.command("arm", {"enabled": True})
        backend.fail_send = True
        session.control.step(.005)
        assert not session.control.armed
    finally:
        session.close()
    rows = records(session)
    failed = [row for row in rows if row["type"] == "downlink" and not row["sent"]]
    assert len(failed) == 1 and failed[0]["frame_id"] == 9
    assert "link loss" in failed[0]["error"]
    assert len(failed[0]["sdk_targets"]["self"]) == 18
    assert any(row["type"] == "sdk_error" and "link loss" in row["error"] for row in rows)


def test_connection_failure_log_is_finished_without_commands(tmp_path, monkeypatch):
    session, backend = make_session(tmp_path, monkeypatch)
    def fail(port):
        raise RuntimeError("simulated connect failure")
    monkeypatch.setattr(backend, "connect_usb", fail)
    try:
        with pytest.raises(RuntimeError, match="connect failure"):
            session.command("connect", {"port": "SIM_ONLY"})
        assert not session.status()["recording"]["active"]
    finally:
        session.close()
    rows = records(session)
    assert rows[-1]["reason"] == "connect_failed"
    assert not any(row["type"] == "downlink" for row in rows)
    assert any(row["type"] == "sdk_error" and row["action"] == "connect" for row in rows)


def test_status_heartbeat_does_not_wait_for_retarget_lock(tmp_path, monkeypatch):
    import threading
    session, _ = make_session(tmp_path, monkeypatch)
    replies = []
    worker = threading.Thread(target=lambda: replies.append(session.command('status', {})))
    try:
        with session.lock:
            worker.start()
            worker.join(timeout=1)
            assert not worker.is_alive()
        assert len(replies) == 1
    finally:
        worker.join(timeout=2)
        session.close()


@pytest.mark.parametrize('kind', ['missing', 'wrong_side', 'rejected'])
def test_session_distinguishes_missing_hand_from_rejected_target(tmp_path, monkeypatch, kind):
    session, _ = make_session(tmp_path, monkeypatch)
    try:
        session.command('connect', {'port': 'SIM_ONLY'})
        session.command('tracking_start', {})
        session.command('arm', {'enabled': True})
        target = {**session.control.pose, 'left_hand_index_PIP_joint': .6}
        session.control.push(target, 0)
        result = {'tracking': False, 'sequence': 1, 'message': 'injected solve rejection',
                  'pose': dict(session.control.pose)}
        monkeypatch.setattr(session_module.WebSession, 'process', lambda self, packet: dict(result))
        monkeypatch.setattr(session, '_record_frame', lambda packet, result: None)
        hand = None if kind == 'missing' else {'label': 'Right' if kind == 'wrong_side' else 'Left'}
        session.process({'hand': hand})
        if kind == 'rejected':
            assert not session.control.armed
            assert session.control.last_error == result['message']
        else:
            assert session.control.armed and session.control.target == target
            assert not session.control.last_error
    finally:
        session.close()
