"""Camera previews remain useful while stale observations cannot retarget H7.

Real sessions, retargeting, control and actuator conversion use the simulated
native SDK only. No camera, transport library or physical device is opened.
"""
import json
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
pytest.importorskip("ruckig")

from online_control.online_motion_track.a32 import session as session_module
from test_a32_tracking_recording import make_session, records
from test_web_v3_2 import packet as original_packet


@pytest.fixture
def camera(tmp_path, monkeypatch):
    session, backend = make_session(tmp_path, monkeypatch)
    clock = SimpleNamespace(elapsed=100., wall=2_000_000_000.)
    monkeypatch.setattr(session_module, "time", SimpleNamespace(
        perf_counter=lambda: clock.elapsed, time=lambda: clock.wall,
        monotonic=time.monotonic))

    def packet(age_ms=150., sequence=0, *, monotonic_age=True, flexion=.6):
        pose = {n: flexion if "PIP" in n else 0. for n in session.control.names}
        world = session.pipeline.session.retargeter.fk.compute(pose)
        value = original_packet(world, sequence=sequence)
        value.update(capture_unix_ms=clock.wall*1000-age_ms,
                     mediapipe_filter_enabled=False)
        if monotonic_age:
            value["source_age_ms"] = age_ms
        return value

    try:
        yield SimpleNamespace(session=session, backend=backend, clock=clock, packet=packet)
    finally:
        session.close()


@pytest.mark.parametrize("direct", [False, True])
def test_150ms_result_updates_preview_and_target_without_arming(camera, direct):
    session = camera.session
    session.set_direct_mode(direct)
    assert session.description()["camera_freshness_limit_ms"] == 400
    request = camera.packet()
    result = session.process(request)
    assert result["tracking"] and result["control_accepted"]
    assert result["control_skip_reason"] is None
    assert result["source_age_at_solve_ms"] == pytest.approx(150.)
    assert len(result["optimized_points"]) == 21 and len(result["pose"]) == 20
    assert session.control.frame_id == 0 and session.control.target_at > 0
    assert not session.control.armed and camera.backend.calls == []
    assert request["capture_unix_ms"] == camera.clock.wall*1000-150.


@pytest.mark.parametrize("age_ms", [400., 401., 2_000.])
def test_expired_result_updates_preview_without_refreshing_control(camera, age_ms):
    control = camera.session.control
    before = (control.target.copy(), control.target_at, control.frame_id)
    result = camera.session.process(camera.packet(age_ms))
    assert result["tracking"] and not result["control_accepted"]
    assert result["control_skip_reason"] == "stale_camera_observation"
    assert len(result["optimized_points"]) == 21
    assert "仅更新预览" in result["control_message"]
    assert (control.target, control.target_at, control.frame_id) == before
    assert not camera.backend.calls


def test_solver_delay_uses_monotonic_elapsed_even_if_wall_clock_moves_back(camera, monkeypatch):
    process = session_module.WebSession.process
    def delayed(session, packet, **kwargs):
        result = process(session, packet, **kwargs)
        camera.clock.elapsed += .3
        camera.clock.wall -= 100.
        return result
    monkeypatch.setattr(session_module.WebSession, "process", delayed)
    result = camera.session.process(camera.packet())
    assert result["tracking"] and not result["control_accepted"]
    assert result["source_age_at_solve_ms"] == pytest.approx(450.)
    assert camera.session.control.target_at == 0


@pytest.mark.parametrize("lock_owner", ["session", "control"])
def test_waiting_for_a_lock_counts_against_observation_age(camera, lock_owner):
    owner = camera.session if lock_owner == "session" else camera.session.control
    original = owner.lock
    class DelayedLock:
        pending = True
        def __enter__(self):
            original.acquire()
            if self.pending:
                camera.clock.elapsed += .3
                self.pending = False
            return self
        def __exit__(self, *args):
            original.release()
    owner.lock = DelayedLock()
    result = camera.session.process(camera.packet())
    assert result["tracking"] and not result["control_accepted"]
    assert result["source_age_at_solve_ms"] == pytest.approx(450.)
    assert camera.session.control.target_at == 0


@pytest.mark.parametrize("age", [-1., float("nan"), float("inf"), True, "1", None])
def test_invalid_monotonic_age_is_preview_only_and_never_emits_nan(camera, age):
    request = camera.packet()
    request["source_age_ms"] = age
    result = camera.session.process(request)
    assert result["tracking"] and not result["control_accepted"]
    assert result["control_skip_reason"] == "invalid_source_age"
    assert result["source_age_at_solve_ms"] is None
    json.dumps(result, allow_nan=False)
    assert camera.session.control.target_at == 0


def test_future_capture_cannot_be_made_fresh_by_a_client_age(camera):
    request = camera.packet()
    request.update(capture_unix_ms=camera.clock.wall*1000+1000., source_age_ms=0.)
    result = camera.session.process(request)
    assert result["tracking"] and not result["control_accepted"]
    assert result["control_skip_reason"] == "future_capture_timestamp"
    assert result["source_age_at_solve_ms"] is None
    assert camera.session.control.frame_id == -1


@pytest.mark.parametrize("age_ms,accepted", [(150., True), (450., False)])
def test_legacy_client_without_monotonic_age_uses_original_capture_time(camera, age_ms, accepted):
    request = camera.packet(age_ms, monotonic_age=False)
    result = camera.session.process(request)
    assert result["tracking"] and result["control_accepted"] is accepted
    assert result["source_age_at_solve_ms"] == pytest.approx(age_ms)


def test_neither_age_source_can_hide_the_older_observation(camera):
    request = camera.packet(450.)
    request["source_age_ms"] = 1.
    result = camera.session.process(request)
    assert not result["control_accepted"]
    request = camera.packet(1., sequence=1)
    request["source_age_ms"] = 450.
    result = camera.session.process(request)
    assert not result["control_accepted"]
    assert result["source_age_at_solve_ms"] == pytest.approx(450.)


@pytest.mark.parametrize("direct", [False, True])
def test_armed_stale_frame_cannot_replace_target_or_write_it_to_sdk(camera, direct):
    session = camera.session
    session.set_direct_mode(direct)
    session.command("connect", {"port": "SIM_ONLY"})
    session.command("arm", {"enabled": True})
    assert session.process(camera.packet())["control_accepted"]
    session.control.step(.002)
    before = (session.control.target.copy(), session.control.target_at, session.control.frame_id)
    calls = len(camera.backend.calls)
    result = session.process(camera.packet(500., sequence=1, flexion=.9))
    assert result["tracking"] and not result["control_accepted"]
    assert session.control.armed
    assert result["pose"] != result["command_joint_angles_rad"]
    assert result["command_joint_angles_rad"] == session.control.pose
    assert (session.control.target, session.control.target_at, session.control.frame_id) == before
    assert len(camera.backend.calls) == calls
    session.control.step(.002)
    assert session.control.target == before[0] and session.control.frame_id == before[2]
    assert session.process(camera.packet(150., sequence=2))["control_accepted"]
    assert session.control.frame_id == 2
    session.command("disconnect", {})
    logged = [r for r in records(session) if r.get("frame_id") == 1]
    assert not any(r["type"] == "control_target" for r in logged)
    assert any(r["type"] == "retarget" and not r["control_accepted"] for r in logged)
    assert any(r["type"] == "control_target_skipped" and r["reason"] == "stale_camera_observation"
               for r in logged)


def test_no_hand_does_not_change_existing_hold_semantics(camera):
    request = camera.packet()
    request["hand"] = None
    request["source_age_ms"] = float("nan")
    result = camera.session.process(request)
    assert not result["tracking"] and not result["control_accepted"]
    assert not result["control_skip_reason"]
    assert camera.session.control.phase == "holding"


def test_metric_commit_remains_the_freshness_authority(camera):
    camera.session.input_source = "webxr"
    request = camera.packet(5_000.)
    request.update(source="webxr", source_age_ms=float("nan"))
    def commit(action):
        action()
        return True
    result = camera.session.process_metric(request, commit)
    assert result["tracking"] and result["control_accepted"]
    assert "source_age_at_solve_ms" not in result
