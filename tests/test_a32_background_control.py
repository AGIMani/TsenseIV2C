"""Lease-safe background solves and presentation caching, using fake hardware."""
import os
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src')]
from online_control.online_motion_track.a32 import session as session_module
from online_control.online_motion_track.a32.control import A32Control
from test_a32_tracking_control import control, FakeDevice, PassGuard
from test_a32_tracking_recording import make_session
from test_web_v3_2 import packet


def accept(action):
    action()
    return True


def install_result(session, *, tracking=True, message='fake rejection', process=None):
    result = dict(tracking=tracking, pose=dict(session.control.pose), sequence=1, message=message)
    session.solver = SimpleNamespace(process=process or (lambda _: result.copy()), close=lambda: None)


def test_background_push_and_read_only_status_do_not_renew_pc_lease(tmp_path, monkeypatch):
    session, _ = make_session(tmp_path, monkeypatch)
    try:
        install_result(session)
        session.last_seen, session.control.client_at = 111., 222.
        result = session.process_background({'hand': {}, 'sequence': 1}, accept)
        assert result['tracking'] and session.control.frame_id == 1
        assert session.control.target_at > 222.
        assert session.last_seen == 111. and session.control.client_at == 222.
        session.status(touch_client=False)
        assert session.last_seen == 111. and session.control.client_at == 222.
        session.status()
        assert session.last_seen > 111. and session.control.client_at > 222.
    finally:
        session.close()


@pytest.mark.parametrize('outcome', ['push', 'hold', 'reject'])
def test_rejected_background_commit_cannot_mutate_control(outcome, tmp_path, monkeypatch):
    session, _ = make_session(tmp_path, monkeypatch)
    try:
        install_result(session, tracking=outcome == 'push')
        calls = []
        for name in ('push', 'hold', 'reject_input'):
            monkeypatch.setattr(session.control, name, lambda *args, n=name: calls.append(n))
        hand = None if outcome == 'hold' else {'label': 'Left'}
        result = session.process_background({'hand': hand}, lambda action: False)
        assert not result['tracking'] and result['background_discarded']
        assert result['message'] == '输入已取消或过期，等待新数据'
        assert calls == []
    finally:
        session.close()


def test_cancel_during_solve_blocks_late_target(tmp_path, monkeypatch):
    session, _ = make_session(tmp_path, monkeypatch)
    began, release = threading.Event(), threading.Event()
    gate, cancelled = threading.Lock(), [False]
    results = []
    def solve(_):
        began.set()
        assert release.wait(2)
        return dict(tracking=True, pose=dict(session.control.pose), sequence=5)
    def commit(action):
        with gate:
            if cancelled[0]:
                return False
            return accept(action)
    install_result(session, process=solve)
    worker = threading.Thread(target=lambda: results.append(session.process_background({}, commit)))
    try:
        original = (session.control.frame_id, session.control.target_at, session.control.target.copy())
        worker.start()
        assert began.wait(1)
        with gate:
            cancelled[0] = True
        release.set()
        worker.join(2)
        assert not worker.is_alive() and results[0]['background_discarded']
        assert (session.control.frame_id, session.control.target_at, session.control.target) == original
    finally:
        release.set()
        worker.join(2)
        session.close()


def test_cancelled_solver_error_cannot_stop_current_control(tmp_path, monkeypatch):
    session, _ = make_session(tmp_path, monkeypatch)
    def fail(_):
        raise ValueError('late solve error')
    try:
        install_result(session, process=fail)
        session.control.last_error = 'current epoch'
        with pytest.raises(ValueError, match='late solve error'):
            session.process_background({}, lambda action: False)
        assert session.control.last_error == 'current epoch'
    finally:
        session.close()


def test_background_inline_path_does_not_renew_base_session_lease(tmp_path, monkeypatch):
    session, _ = make_session(tmp_path, monkeypatch)
    try:
        session.last_seen, session.control.client_at = 111., 222.
        request = dict(sequence=1, timestamp_ms=1., capture_unix_ms=1., hand=None)
        assert not session.process_background(request, accept)['tracking']
        assert session.last_seen == 111. and session.control.client_at == 222.
    finally:
        session.close()


@pytest.mark.parametrize('isolated', [False, True])
def test_trusted_source_reset_clears_estimation_without_motion_or_lease(isolated, tmp_path, monkeypatch):
    session, _ = make_session(tmp_path, monkeypatch)
    try:
        if isolated:
            session.solver = Mock()
        resetters = [Mock(), Mock(), Mock()]
        for owner, reset in zip((session.filter, session.normalized_filter, session.pipeline.session.retargeter), resetters):
            monkeypatch.setattr(owner, 'reset', reset)
        session.last_seen, session.control.client_at = 111., 222.
        original = (session.control.frame_id, session.control.target_at, session.control.target.copy())
        session.reset_metric_source()
        for reset in resetters:
            reset.assert_called_once_with()
        if isolated:
            session.solver.call.assert_called_once_with('reset')
        assert session.last_seen == 111. and session.control.client_at == 222.
        assert (session.control.frame_id, session.control.target_at, session.control.target) == original
    finally:
        session.close()


def test_cache_reuses_only_presentation_and_advances_telemetry_age(monkeypatch):
    item, device = control()
    try:
        now = [100.]
        monkeypatch.setattr('online_control.online_motion_track.a32.control.time.monotonic', lambda: now[0])
        item.snapshot()
        device.state['telemetry_age_ms'] = 400
        now[0] += .025
        cached = item.snapshot(refresh_device=False)
        assert cached['telemetry_age_ms'] == pytest.approx(25.)
        assert cached['sdk_snapshot_age_ms'] == pytest.approx(25.)
        assert item.snapshot()['telemetry_age_ms'] == 400
        now[0] += .2
        device.state['telemetry_age_ms'] = 600
        assert item.snapshot(refresh_device=False)['telemetry_age_ms'] == 600
    finally:
        item.close()


def test_cached_presentation_never_hides_current_control_or_recording_error():
    item, _ = control()
    try:
        item.snapshot()
        item.last_error = 'current motor stop'
        item.recorder.writer._error = 'disk full'
        result = item.snapshot(refresh_device=False)
        assert result['last_error'] == 'current motor stop'
        assert result['recording']['error'] == 'disk full'
        item._state_cache_error = RuntimeError('observer failed')
        with pytest.raises(RuntimeError, match='observer failed'):
            item.snapshot(refresh_device=False)
    finally:
        item.close()


def test_step_uses_live_fault_even_when_presentation_cache_is_healthy():
    item, device = control()
    try:
        item.arm(True)
        item.snapshot()
        device.state['mit'] = [{'online': True, 'fault': 1}]
        item.step(.002)
        assert not item.armed and not device.writes and '故障' in item.last_error
    finally:
        item.close()


def test_full_step_timing_includes_log_producer_and_post_write_work(monkeypatch):
    item, _ = control()
    try:
        item.arm(True)
        record = item._record
        def slow_record(method, *args, **kwargs):
            if method == 'record_command':
                time.sleep(.005)
            return record(method, *args, **kwargs)
        monkeypatch.setattr(item, '_record', slow_record)
        item.step(.002)
        timing = item.snapshot()['control_timing']
        assert timing['command_record_ms'] >= 4.
        assert timing['step_total_ms'] >= timing['post_send_ms'] >= timing['command_record_ms']
        assert item.samples[-1]['control_timing'] == timing
    finally:
        item.close()


@pytest.mark.parametrize('source', ['mediapipe', 'webxr', 'wuji_glove'])
def test_all_live_sources_solve_in_child_without_hardware_or_lease_renewal(source, tmp_path, monkeypatch):
    def fake_control(profile, side, mode, **options):
        return A32Control(profile, side, mode, device=FakeDevice(profile.model_id), start=True, **options)
    monkeypatch.setattr(session_module, 'A32Control', fake_control)
    session = session_module.A32WebSession(input_source=source, log_dir=tmp_path)
    try:
        assert session.solver.process_handle.pid != os.getpid()
        world = session.pipeline.session.retargeter.fk.compute()
        request = packet(world)
        request['source'] = source
        session.last_seen, session.control.client_at = 111., 222.
        result = session.process_background(request, accept)
        assert result['tracking'] and result['source'] == source
        assert len(result['pose']) == 20 and session.control.frame_id == request['sequence']
        assert session.last_seen == 111. and session.control.client_at == 222.
        assert not session.control.device.writes
    finally:
        session.close()
    assert not session.solver.process_handle.is_alive()


def test_legacy_angle_recording_keeps_inline_writer(tmp_path, monkeypatch):
    def fake_control(profile, side, mode, **options):
        return A32Control(profile, side, mode, device=FakeDevice(profile.model_id), start=True, **options)
    monkeypatch.setattr(session_module, 'A32Control', fake_control)
    session = session_module.A32WebSession(recording=True, input_source='webxr', log_dir=tmp_path)
    try:
        assert session.solver is None and session.recorder is not None
        request = packet(session.pipeline.session.retargeter.fk.compute())
        request['source'] = 'webxr'
        assert session.process_background(request, accept)['tracking']
    finally:
        session.close()
    assert session.recorder.path.exists() and session.recorder.path.stat().st_size > 0
