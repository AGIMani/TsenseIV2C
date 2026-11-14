"""Leap epochs/expiry changing during real solves cannot replace robot targets."""
from pathlib import Path
import sys
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src')]
from online_control.online_motion_track.main import Application
from online_control.online_motion_track.leapmotion_input import LeapMotionInputRelay
from online_control.online_motion_track.leapmotion_source import FINGERS
from online_control.online_motion_track.a32 import session as a32_module
from online_control.online_motion_track.a32.control import A32Control
from test_a32_tracking_control import FakeDevice
from test_online_leapmotion_input import FakeSdk, hand, frame, wait_until


def native_hand(points, identity=1):
    result = hand(identity=identity)
    def vector(point):
        return NS(**dict(zip(('x', 'y', 'z'), (points[point] * 1000).tolist())))
    result.arm.next_joint = vector(0)
    for index, name in enumerate(FINGERS):
        digit = getattr(result, name)
        joints = [1 + index * 4 + offset for offset in range(4)]
        digit.metacarpal.prev_joint = vector(joints[0] if index == 0 else 0)
        digit.metacarpal.next_joint = vector(joints[0])
        for bone, start, end in zip(('proximal', 'intermediate', 'distal'), joints, joints[1:]):
            getattr(digit, bone).prev_joint = vector(start)
            getattr(digit, bone).next_joint = vector(end)
    return result


@pytest.fixture
def setup(tmp_path, monkeypatch):
    monkeypatch.setattr(a32_module, 'A32Control', lambda profile, side, mode, **kwargs:
        A32Control(profile, side, mode, device=FakeDevice(profile.model_id, side), start=False, **kwargs))
    session = a32_module.A32WebSession(recording=False, log_dir=tmp_path, input_source='leapmotion')
    session.command('tracking_start', {})
    now = [10.]
    sdk = FakeSdk()
    relay = LeapMotionInputRelay(clock=lambda: now[0], sdk_loader=lambda: sdk, retry_delay=.01)
    app = Application(tmp_path)
    app.leapmotion = relay
    app.sessions['test'] = session
    relay.create('test', 'left')
    relay.latest('test')
    wait_until(lambda: sdk.connections and sdk.connections[-1].ready)
    solver = session.pipeline
    method = 'process'
    points = session.pipeline.session.retargeter.fk.compute()
    push_name = 'push'
    push = Mock(wraps=getattr(session.control, push_name))
    monkeypatch.setattr(session.control, push_name, push)
    native_sequence = 0
    def emit(identity=1):
        nonlocal native_sequence
        native_sequence += 1
        sdk.native_now += 1000
        sdk.emit(frame(native_sequence, [native_hand(points, identity)], timestamp=sdk.native_now))
    def submit(index):
        snapshot = relay.latest('test')
        return app.frame(dict(session='test', source='leapmotion', leapmotion_revision=snapshot['revision'],
            sequence=index, timestamp_ms=1000 + index * 16, capture_unix_ms=1789700000000 + index * 16))
    emit()
    initial = submit(0)
    assert initial['tracking'] and push.call_count == 1
    push.reset_mock()
    try:
        yield NS(app=app, session=session, relay=relay, sdk=sdk, now=now,
                 emit=emit, submit=submit, solver=solver, method=method, push=push)
    finally:
        app.close()
        relay.client.close()


@pytest.mark.parametrize('change', ['expired', 'identity_replaced'])
def test_actual_solver_result_rechecks_native_source_before_target_commit(setup, change, monkeypatch):
    h = setup
    h.emit()
    solve = getattr(h.solver, h.method)
    def changed_during_solve(*args, **kwargs):
        result = solve(*args, **kwargs)
        if change == 'expired':
            h.now[0] += .4
        else:
            h.emit(identity=2)
        return result
    monkeypatch.setattr(h.solver, h.method, changed_during_solve)
    result = h.submit(1)
    assert not result['tracking']
    assert result.get('display_discarded') or result.get('background_discarded')
    assert result['discard_reason'] == 'source_cancelled_or_expired'
    assert result['retarget_joint_angles_rad'], 'keep the completed solve for diagnostics'
    h.push.assert_not_called()
    assert not h.session.control.device.writes


def test_old_epoch_solve_failure_does_not_invalidate_newer_control(setup, monkeypatch):
    h = setup
    h.emit()
    invalidation_name = 'reject_input'
    invalidation = Mock(wraps=getattr(h.session.control, invalidation_name))
    monkeypatch.setattr(h.session.control, invalidation_name, invalidation)
    solve = getattr(h.solver, h.method)
    def failed_old_solve(*args, **kwargs):
        solve(*args, **kwargs)
        h.emit(identity=2)
        raise RuntimeError('old source solve failed')
    monkeypatch.setattr(h.solver, h.method, failed_old_solve)
    with pytest.raises(RuntimeError, match='old source solve failed'):
        h.submit(1)
    invalidation.assert_not_called()
    h.push.assert_not_called()
