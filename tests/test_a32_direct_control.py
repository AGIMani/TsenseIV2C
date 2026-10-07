"""A3.2 direct command mode keeps device gates without trajectory processing."""
from types import SimpleNamespace
from unittest.mock import Mock

import pytest

from test_a32_tracking_control import control
from test_a32_tracking_recording import make_session, records
from online_control.online_motion_track.a32.solver_process import _worker


def forbidden(*args, **kwargs):
    raise AssertionError("direct mode entered a trajectory or collision filter")


@pytest.mark.parametrize('normal_mode', ['stop', 'mink'])
@pytest.mark.parametrize('direct_mode', [False, True])
def test_recording_metadata_and_state_describe_effective_mode(normal_mode, direct_mode):
    item, _ = control()
    try:
        item.mode = normal_mode
        item._normal_guard = item.guard
        recorder = item.recorder = Mock(writer=SimpleNamespace(active=True))
        item.set_direct_mode(direct_mode)
        item.start_recording(camera_source='quest3-webxr', collision_mode='monitor',
                             direct_mode=not direct_mode, ruckig_enabled=direct_mode)
        metadata = recorder.start.call_args.kwargs['metadata']
        assert metadata['camera_source'] == 'quest3-webxr'
        assert metadata['collision_mode'] == ('off' if direct_mode else normal_mode)
        assert metadata['direct_mode'] is direct_mode
        assert metadata['ruckig_enabled'] is (not direct_mode)
        assert metadata['collision_protection_enabled'] is (not direct_mode)
        item._record_state({'connected': True})
        state = recorder.record_state.call_args.args[0]
        for field in ('collision_mode', 'direct_mode', 'ruckig_enabled', 'collision_protection_enabled'):
            assert state[field] == metadata[field]
    finally:
        item.close()


def test_direct_sends_latest_bounded_target_without_planner_or_step_limit(monkeypatch):
    item, device = control()
    try:
        item.set_direct_mode(True)
        monkeypatch.setattr(item, '_checked_step', forbidden)
        monkeypatch.setattr(item.output, 'pass_to_input', forbidden)
        item.arm(True)
        item.step(.002)
        assert device.writes == []  # A seed is not a new observation.
        first = {**device.feedback, 'left_hand_index_PIP_joint': .8}
        latest = {**first, 'left_hand_index_PIP_joint': 99.}
        item.push(first, 10)
        item.push(latest, 11)
        item.step(.002)
        bounded = device.constrain_pose(latest)
        assert device.writes == [bounded]
        assert item.pose == bounded and item.frame_id == 11
        state = item.snapshot(after_tick=0)
        assert state['direct_mode'] and not state['ruckig_enabled']
        assert not state['collision_protection_enabled']
        assert state['collision_mode'] == 'off'
        assert state['command_joint_angles_rad'] == state['ruckig_joint_angles_rad'] == bounded
        assert state['control_timing']['planner_dt_ms'] is None
        assert state['command_samples'][0]['direct_mode']
        assert not item.recording_error
    finally:
        item.close()


def test_toggle_preserves_guard_and_clears_targets_derivatives_and_holds():
    item, device = control()
    try:
        normal = item._normal_guard = item.guard
        item.mode = 'stop'
        item.input.current_velocity = [1.] * 20
        item.input.current_acceleration = [2.] * 20
        item.finger_holds = {'index': {'left_hand_index_PIP_joint': .2}}
        item.finger_hold_context = {'index': {'left_hand_mid_PIP_joint': .3}}
        item.target_at, item.frame_id = 1., 3
        item.set_direct_mode(True)
        assert item.guard is None and not item.target_at and item.frame_id == -1
        assert item.target == item.pose
        assert not any(item.input.current_velocity) and not any(item.input.current_acceleration)
        assert not item.finger_holds and not item.finger_hold_context
        item.arm(True)
        with pytest.raises(ValueError, match='关闭下发'):
            item.set_direct_mode(False)
        assert item.direct_mode and item.guard is None
        item.stop()
        item.set_direct_mode(False)
        assert item.guard is normal and not item.direct_mode
        assert item.snapshot()['ruckig_enabled']
        assert item.snapshot()['collision_protection_enabled']
        assert item.snapshot()['collision_mode'] == 'stop'
    finally:
        item.close()


@pytest.mark.parametrize('value', [None, 0, 1, 'true'])
def test_direct_mode_requires_boolean(value):
    item, _ = control()
    try:
        with pytest.raises(ValueError, match='布尔值'):
            item.set_direct_mode(value)
        assert not item.direct_mode
    finally:
        item.close()


@pytest.mark.parametrize('failure', ['heartbeat', 'fault', 'disconnect', 'write'])
def test_direct_preserves_stop_gates_and_commits_only_successful_writes(failure):
    item, device = control()
    try:
        item.set_direct_mode(True)
        item.arm(True)
        accepted = dict(item.pose)
        item.push({**device.feedback, 'left_hand_index_PIP_joint': .8}, 1)
        if failure == 'heartbeat':
            item.client_at -= 3.
        elif failure == 'fault':
            device.state['mit'] = [{'online': True, 'fault': True}]
        elif failure == 'disconnect':
            device.is_connected = False
        else:
            device.fail_write = True
        item.step(.002)
        assert not item.armed and item.last_error
        assert item.pose == accepted and not device.writes and item.ticks == 0
    finally:
        item.close()


def test_direct_rejects_nonfinite_target_without_replacing_current_target():
    item, device = control()
    try:
        item.set_direct_mode(True)
        original = dict(item.target)
        with pytest.raises(ValueError):
            item.push({**device.feedback, 'left_hand_index_PIP_joint': float('nan')}, 3)
        assert item.target == original and item.frame_id == -1
        assert not device.writes
    finally:
        item.close()


@pytest.mark.parametrize('background', [False, True])
@pytest.mark.parametrize('accept', [False, True])
def test_session_propagates_direct_mode_and_labels_process_results(tmp_path, monkeypatch, background, accept):
    session, _ = make_session(tmp_path, monkeypatch)
    try:
        solver = session.solver = Mock()
        session.command('set_direct_mode', {'enabled': True})
        solver.call.assert_called_once_with('set_direct_mode', True)
        assert session.pipeline.collision_bypass and session.control.guard is None
        assert session.description()['direct_mode']
        assert session.description()['collision_mode'] == 'off'
        solver.process.return_value = dict(tracking=True, pose=dict(session.control.pose), sequence=7)
        if background:
            def commit(action):
                if accept:
                    action()
                return accept
            result = session.process_background({'hand': {}}, commit)
            assert result.get('background_discarded', False) == (not accept)
        else:
            result = session.process({'hand': {}})
        assert result['direct_mode']
        if not background or accept:
            assert not result['hardware']['ruckig_enabled']
            assert result['command_joint_angles_rad'] == session.control.pose
        session.control.armed = True
        with pytest.raises(ValueError, match='关闭下发'):
            session.command('set_direct_mode', {'enabled': False})
        solver.call.assert_called_once_with('set_direct_mode', True)
        session.control.armed = False
        session.command('set_direct_mode', {'enabled': False})
        assert not session.pipeline.collision_bypass
        assert solver.call.call_args.args == ('set_direct_mode', False)
    finally:
        session.close()


def test_direct_recording_names_the_actual_command_path(tmp_path, monkeypatch):
    session, _ = make_session(tmp_path, monkeypatch)
    try:
        session.command('set_direct_mode', {'enabled': True})
        session.command('connect', {'port': 'SIM_ONLY'})
        session.command('arm', {'enabled': True})
        target = {**session.control.pose, 'left_hand_index_PIP_joint': .7}
        session.control.push(target, 8)
        session.control.step(.002)
        assert session.control.pose == target
        assert not session.control.recording_error
    finally:
        session.close()
    commands = [r for r in records(session) if r.get('frame_id') == 8 and r['type'] in ('downlink', 'h7')]
    assert {r['type'] for r in commands} == {'downlink', 'h7'}
    assert all(r['source'] == 'a32_tracking_direct' for r in commands)
    assert all(r.get('planner_dt_s') is None for r in commands)


def test_solver_mode_and_seed_bypass_guard_without_skipping_normal_seed(monkeypatch):
    from online_control.online_motion_track import main
    model_session = SimpleNamespace(set_pose=Mock(), retargeter=SimpleNamespace(reset=Mock()))
    guard = SimpleNamespace(current={}, guard=SimpleNamespace(rebase=Mock()))
    pipeline = SimpleNamespace(session=model_session, mink_guard=guard, collision_bypass=False)
    fake = SimpleNamespace(pipeline=pipeline, filter=SimpleNamespace(reset=Mock()),
                           normalized_filter=SimpleNamespace(reset=Mock()),
                           process=lambda packet: {'tracking': True}, close=Mock())
    monkeypatch.setattr(main, 'WebSession', lambda **kwargs: fake)
    requests = iter([('set_direct_mode', True), ('seed', {'q': .5}), ('process', {}),
                     ('set_direct_mode', False), ('seed', {'q': .6}), ('close', None)])
    sent = []
    pipe = SimpleNamespace(recv=lambda: next(requests), send=sent.append, close=lambda: None)
    _worker(pipe, {})
    assert not any(kind == 'error' for kind, value in sent)
    assert sent[3] == ('ok', {'tracking': True, 'direct_mode': True})
    guard.guard.rebase.assert_called_once_with({'q': .6})
    assert model_session.set_pose.call_count == 2
    assert not pipeline.collision_bypass
