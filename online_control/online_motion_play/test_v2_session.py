"""V2 file, dialog, planner and stop contracts without Qt or physical devices."""
import ast
import json
import math
from pathlib import Path
import sys
import threading
import time

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from v2_session import JOINTS, PERIOD, V2Session, _SweepTransport

JOINT = 'left_hand_mid_MCP_joint'
SWING = 'left_hand_mid_swing_joint'


@pytest.mark.parametrize('disconnect_fails', [False, True])
def test_close_releases_native_planners(tmp_path, monkeypatch, disconnect_fails):
    state = V2Session(tmp_path)
    planner = state._interpolator
    assert planner._otg and planner._inputs and planner._outputs
    if disconnect_fails:
        def fail_disconnect():
            raise OSError('disconnect failed')
        monkeypatch.setattr(state.usb, 'disconnect', fail_disconnect)
        with pytest.raises(OSError, match='disconnect failed'):
            state.close()
    else:
        state.close()
        state.close()
    # Keep the session alive: cleanup must not depend on cyclic GC or shutdown
    # ordering between Python signal handlers and the native binding module.
    assert not planner._otg
    assert not planner._inputs
    assert not planner._outputs


class Clock:
    now = 0.

    def __call__(self):
        return self.now

    def advance(self, seconds=PERIOD):
        self.now += seconds


class FakeUsb:
    def __init__(self, connected=True):
        self.is_connected = connected
        self.frames, self.commands = [], []
        self.seed = None
        self.fail_next = False
        self.send_hex_log_enabled = False
        self.telemetry = None

    @property
    def should_process_downlink(self):
        return self.is_connected

    def send_joint_angles(self, cfg, **options):
        if self.fail_next:
            self.fail_next = False
            return False
        self.frames.append((dict(cfg), dict(options)))
        return True

    def read_latest_joint_angles(self, **kwargs):
        return self.seed

    def send_hand_type(self, side):
        self.commands.append(('side', side))
        return True

    def send_all_lm_homing(self, command, current):
        self.commands.append(('linear', command, current))
        return 14

    def send_rm_drive_motor(self, bus, motor, enable):
        self.commands.append(('enable', bus, motor, enable))
        return True

    def send_rm_zero(self, bus, motor):
        self.commands.append(('zero', bus, motor))
        return True

    def send_rm_pos_motor(self, bus, motor, value):
        self.commands.append(('position', bus, motor, value))
        return True

    def read_latest_telemetry(self):
        return self.telemetry

    def _write_frame(self, frame, **kwargs):
        self.commands.append(('raw', frame))
        return True

    def connect_port(self, port):
        self.is_connected = True
        return True

    def disconnect(self):
        self.is_connected = False


@pytest.fixture
def session(tmp_path):
    clock, usb = Clock(), FakeUsb()
    state = V2Session(tmp_path, usb=usb, clock=clock, start=False)
    yield state, usb, clock
    state.close()


def control(state, key, value=None, **extra):
    snapshot = state.snapshot()
    candidates = list(snapshot['controls'].values()) + snapshot['joints'] + [
        item for dialog in snapshot['dialogs'] for item in dialog['items']]
    spec = next(item for item in candidates if item['id'].rsplit(':', 1)[-1] == key)
    return state.widget({'id': spec['id'], 'value': value, **extra})


def import_motions(state, count=1, *, transition=.01, hold=0., side='left'):
    state.dispatch('import', {'text': json.dumps({
        '_hand_side': side,
        **{f'm{i}': {JOINT: .1 + i * .001, '_transition_s': transition, '_interval_s': hold}
           for i in range(count)}})})


def run_until_done(state, clock, limit=4000):
    for _ in range(limit):
        clock.advance()
        state.tick()
        if state._task is None:
            return
    pytest.fail('trajectory did not complete')


def save(state, name):
    control(state, 'savemotion')
    control(state, 'motionName', name)
    control(state, 'confirm')


def test_initial_schema_defaults_and_invalid_values(session):
    state, usb, _ = session
    snapshot = state.snapshot()
    assert len(snapshot['joints']) == 21
    assert all(value == 0. for value in snapshot['pose'].values())
    assert snapshot['controls']['transition']['min'] == .001
    assert snapshot['controls']['transition']['max'] == 5.
    assert snapshot['controls']['interval']['max'] == 30.
    assert snapshot['controls']['motiondirectlysend']['text'] == '五指往返'
    for value in (float('nan'), float('inf'), True, -1., 10.):
        with pytest.raises(ValueError):
            control(state, JOINT, value)
    assert not usb.frames


def test_save_insert_overwrite_confirm_delete_and_disk_roundtrip(session):
    state, _, _ = session
    for name in ('first', 'last'):
        save(state, name)
        control(state, 'confirm')  # saved message
    control(state, 'choosemotion', 0)
    control(state, JOINT, .4)
    save(state, 'middle')
    control(state, 'confirm')
    assert list(state.motions) == ['first', 'middle', 'last']
    control(state, JOINT, .7)
    save(state, 'middle')
    assert state.dialog['action'] == 'overwrite'
    assert state.motions['middle'][JOINT] == .4
    control(state, 'cancel')
    assert state.motions['middle'][JOINT] == .4
    save(state, 'middle')
    control(state, 'confirm')
    control(state, 'confirm')
    assert list(state.motions) == ['first', 'middle', 'last']
    assert state.motions['middle'][JOINT] == .7
    control(state, 'deletemotion')
    control(state, 'confirm')
    assert list(state.motions) == ['first', 'last']
    assert state.selected == 1
    assert state.pose[JOINT] == 0.
    exported = state.dispatch('export', {})
    assert json.loads(state.path.read_text()) == exported
    clone = V2Session(state.data_dir, usb=FakeUsb(False), start=False)
    try:
        assert clone.dispatch('export', {}) == exported
    finally:
        clone.close()


def test_right_file_legacy_list_and_bad_import_are_atomic(session):
    state, usb, _ = session
    thumb, pinky = 'left_hand_thumb_swing_joint', 'left_hand_pinky_rota_joint'
    payload = {'_hand_side': 'right', 'r': {thumb: -.4, pinky: -.3}}
    state.dispatch('import', {'text': '\ufeff' + json.dumps(payload)})
    assert state.side == 'right'
    assert state.motions['r'][thumb] == pytest.approx(.6)
    assert state.motions['r'][pinky] == pytest.approx(.3)
    assert state.dispatch('export', {}) == payload
    old_bytes = state.path.read_bytes()
    for broken in ({'bad': {JOINT: float('nan')}},
                   {'bad': {'_transition_s': -1}},
                   {'_model_id': 'TsenseI_v3_2', 'motions': {}}):
        with pytest.raises(ValueError):
            state.dispatch('import', {'text': json.dumps(broken)})
        assert state.path.read_bytes() == old_bytes
    state.dispatch('import', {'text': json.dumps([{'name': 'legacy', JOINT: .2}])})
    assert state.side == 'left'
    assert state.motions['legacy'][JOINT] == .2
    assert not usb.frames


def test_stale_dialog_and_hand_side_ids_are_rejected(session):
    state, _, _ = session
    control(state, 'savemotion')
    stale = state.snapshot()['dialogs'][0]['items'][-2]['id']
    control(state, 'cancel')
    control(state, 'savemotion')
    with pytest.raises(ValueError):
        state.widget({'id': stale})
    state.stop()
    old_id = state.snapshot()['joints'][0]['id']
    control(state, 'handSide', 1)
    with pytest.raises(ValueError):
        state.widget({'id': old_id, 'value': .1})


def test_selection_never_moves_and_load_without_sync_is_display_only(session):
    state, usb, _ = session
    import_motions(state, 2)
    control(state, 'choosemotion', 1)
    assert state.pose[JOINT] == 0.
    control(state, 'loadmotion')
    assert state.pose[JOINT] == pytest.approx(.101)
    assert not usb.frames
    control(state, 'motionstreamplay')
    assert state.live[JOINT] == pytest.approx(.101)


def test_realtime_yes_no_remember_cancel_and_exclusivity(session):
    state, usb, clock = session
    control(state, 'realtimeJointControl', True)
    control(state, 'cancel')
    assert not state.realtime
    usb.seed = {JOINT: .6}
    control(state, 'realtimeJointControl', True)
    control(state, 'rememberRead', True)
    control(state, 'confirm')
    assert state.realtime and state.pose[JOINT] == .6
    control(state, JOINT, .8, commit=True)
    clock.advance()
    state.tick()
    assert usb.frames and .6 <= usb.frames[-1][0][JOINT] < .8
    control(state, 'isConnectOffline', True)
    assert state.sync and not state.realtime
    control(state, 'realtimeJointControl', True)
    assert state.realtime and not state.sync and state.dialog is None
    control(state, 'openSendLog', True)
    assert usb.send_hex_log_enabled


def test_realtime_declining_telemetry_preserves_other_joint_pose(session):
    state, usb, clock = session
    other = 'left_hand_index_MCP_joint'
    control(state, other, .5)
    usb.seed = {other: 0.}
    control(state, 'realtimeJointControl', True)
    control(state, 'no')
    control(state, JOINT, .4)
    clock.advance()
    state.tick()
    assert usb.frames[-1][0][other] == pytest.approx(.5)


def test_failed_telemetry_requires_visible_acknowledgement_then_falls_back(session):
    state, usb, clock = session
    control(state, JOINT, .6)
    control(state, 'realtimeJointControl', True)
    control(state, 'confirm')
    assert state.dialog['title'] == '读取失败'
    assert not state.realtime
    clock.advance()
    state.tick()
    assert not usb.frames
    control(state, 'confirm')
    assert state.realtime and state.pose[JOINT] == .6


def test_realtime_target_updates_do_not_bypass_two_millisecond_deadline(session):
    state, usb, clock = session
    control(state, 'realtimeJointControl', True)
    control(state, 'no')
    state.tick()
    assert len(usb.frames) == 1
    for index in range(10):
        control(state, JOINT, .1 + index * .01, commit=False)
        clock.advance(.0001)
        state.tick()
    assert len(usb.frames) == 1
    clock.advance(.001)
    state.tick()
    assert len(usb.frames) == 2
    assert state._realtime_target[JOINT] == pytest.approx(.19)


def test_timeline_does_not_scan_full_motion_dictionary_during_tick(session):
    state, _, clock = session
    import_motions(state, 40, transition=.002)
    control(state, 'motionstreamplay')
    class NoScan(dict):
        def __iter__(self):
            raise AssertionError('per-tick full sequence scan')
        def values(self):
            raise AssertionError('per-tick full sequence scan')
    state.motions = NoScan(state.motions)
    for _ in range(45):
        clock.advance()
        state.tick()
    assert state.selected == 39
    assert state._task is None


def test_large_target_is_jerk_limited_and_protocol_floor_preserves_ui_zero(session):
    state, usb, clock = session
    state._seed(dict(state.pose))
    target = {**state.pose, JOINT: 1.57}
    positions = []
    for _ in range(80):
        clock.advance()
        ok, _ = state._send_step(target, source='test', stream=True)
        assert ok
        positions.append(usb.frames[-1][0][JOINT])
    assert 0 < positions[0] < 1.57
    assert positions == sorted(positions)
    assert max(b-a for a, b in zip([0.] + positions, positions)) <= 20. * PERIOD + 1e-9
    state._seed({name: 0. for name in JOINTS})
    clock.advance()
    ok, output = state._send_step({name: 0. for name in JOINTS}, source='zero', stream=False)
    assert ok and output[SWING] == 0.
    assert usb.frames[-1][0][SWING] == pytest.approx(math.radians(.5))


def test_write_failure_rolls_back_ui_and_hw_baselines(session):
    state, usb, clock = session
    state._seed(dict(state.pose))
    target = {**state.pose, JOINT: 1.2}
    for _ in range(20):
        clock.advance()
        state._send_step(target, source='test', stream=True)
    last_hw, last_ui = dict(state._last_success_hw), dict(state._last_success_ui)
    usb.fail_next = True
    clock.advance()
    ok, output = state._send_step(target, source='test', stream=True)
    assert not ok and output == last_ui
    assert state._last_success_hw == last_hw
    clock.advance()
    ok, output = state._send_step(target, source='test', stream=True)
    assert ok and 0 <= output[JOINT] - last_ui[JOINT] <= 20. * PERIOD
    assert output[SWING] == 0.


def test_guard_clamp_resynchronizes_interpolator_without_expected_jump(session):
    state, usb, clock = session
    state._seed(dict(state.pose))
    original = state._guard.filter
    jumps, resynced = [], []
    def limited(cfg, **options):
        jumps.append(options.get('expected_jump', False))
        result = original(cfg, **options)
        result[JOINT] = 0.
        return result
    state._guard.filter = limited
    sync_pose = state._interpolator.sync_current_pose
    def sync(cfg, **options):
        resynced.append(dict(cfg))
        return sync_pose(cfg, **options)
    state._interpolator.sync_current_pose = sync
    clock.advance()
    ok, output = state._send_step({**state.pose, JOINT: 1.}, source='test', stream=True)
    assert ok and output[JOINT] == 0.
    assert resynced and resynced[-1][JOINT] == 0.
    assert jumps == [False]


def test_settle_timeout_holds_last_safe_pose_and_never_sends_absolute_target(session):
    state, usb, clock = session
    import_motions(state, transition=.001)
    control(state, 'isConnectOffline', True)
    control(state, 'motionstreamplay')
    # A device refusing all writes cannot be treated as having reached the target.
    attempts = []
    def refuse(cfg, **options):
        attempts.append((dict(cfg), options))
        return False
    usb.send_joint_angles = refuse
    run_until_done(state, clock)
    assert '超时' in state.message
    assert state.pose[JOINT] == 0.
    assert 5. <= state.playback_timing['settle_s'] < 5.003
    assert all(cfg[JOINT] < .1 for cfg, _ in attempts)
    assert not usb.frames


def test_long_idle_does_not_skip_ruckig_first_step(session):
    state, usb, clock = session
    state._seed(dict(state.pose))
    clock.advance(120.)
    ok, output = state._send_step({**state.pose, JOINT: 1.57}, source='test', stream=False)
    assert ok and 0. < output[JOINT] < .01
    assert usb.frames[-1][0][JOINT] == output[JOINT]


@pytest.mark.parametrize('hold', [0., .000001])
def test_dense_timeline_final_convergence_without_per_segment_flush(session, hold):
    state, usb, clock = session
    import_motions(state, 120, transition=.002, hold=hold)
    control(state, 'isConnectOffline', True)
    control(state, 'motionstreamplay')
    run_until_done(state, clock)
    stats = state.playback_timing
    assert stats['nominal_s'] == pytest.approx(120 * (.002 + hold))
    assert stats['timeline_s'] < stats['nominal_s'] + .00201
    assert stats['sent'] > 0 and not stats['stopped']
    assert state.selected == 119
    assert state.pose[JOINT] == pytest.approx(.219, abs=1e-5)
    streams = [options['stream'] for _, options in usb.frames]
    assert all(streams[:stats['sent']])
    assert len(streams) - stats['sent'] >= 3
    assert all(not stream for stream in streams[stats['sent']:])
    assert all('最终态' in options['source'] for _, options in usb.frames[stats['sent']:])


def test_late_sender_skips_expired_ticks_without_bursting(session):
    state, usb, clock = session
    import_motions(state, 100, transition=.002)
    original = usb.send_joint_angles
    def delayed(cfg, **kw):
        result = original(cfg, **kw)
        clock.advance(.005)
        return result
    usb.send_joint_angles = delayed
    control(state, 'isConnectOffline', True)
    control(state, 'motionstreamplay')
    run_until_done(state, clock)
    assert state.playback_timing['missed_ticks'] > 0
    assert state.playback_timing['ticks'] < 50
    assert state.playback_timing['timeline_s'] < .213


def test_loop_repeats_selected_subsequence_and_stop_never_flushes(session):
    state, usb, clock = session
    import_motions(state, 3, transition=.004)
    control(state, 'choosemotion', 1)
    control(state, 'isConnectOffline', True)
    control(state, 'motionStreamLoop', True)
    control(state, 'motionstreamplay')
    for _ in range(16):
        clock.advance()
        state.tick()
    assert state.playing
    assert state._task['entries'][0][0] == 'm1'
    assert all(options['stream'] for _, options in usb.frames)
    state.stop()
    sent = len(usb.frames)
    for _ in range(10):
        clock.advance()
        state.tick()
    assert len(usb.frames) == sent
    assert state.playback_timing['stopped']


def test_unchecking_loop_finishes_and_load_uses_edited_transition(session):
    state, usb, clock = session
    import_motions(state)
    control(state, 'isConnectOffline', True)
    control(state, 'transition', .04)
    control(state, 'loadmotion')
    assert state._task['stats']['nominal_s'] == .04
    run_until_done(state, clock)
    control(state, 'motionStreamLoop', True)
    control(state, 'motionstreamplay')
    clock.advance(.02)
    state.tick()
    control(state, 'motionStreamLoop', False)
    run_until_done(state, clock)
    assert not state.playing


@pytest.mark.parametrize('hold', [0., .000001])
def test_single_offline_segment_finishes_at_exact_subtick_deadline(session, hold):
    state, usb, clock = session
    import_motions(state, transition=.002, hold=hold)
    control(state, 'motionstreamplay')
    while state._task:
        clock.now = state._task['next_tick']
        state.tick()
    assert clock.now == pytest.approx(.002 + hold, abs=1e-12)
    assert state.playback_timing['timeline_s'] == pytest.approx(.002 + hold)
    assert not usb.frames


def test_loop_cycle_starts_from_last_actual_output_not_target(session):
    state, usb, clock = session
    import_motions(state, transition=.004)
    control(state, 'isConnectOffline', True)
    control(state, 'motionStreamLoop', True)
    control(state, 'motionstreamplay')
    clock.advance(.004)
    state.tick()
    origin = state._task['segments'][0][3]
    assert origin == state.live
    assert 0. < origin[JOINT] < .1
    assert all(options['stream'] for _, options in usb.frames)


def test_stop_before_first_deadline_cannot_send_or_flush(session):
    state, usb, clock = session
    import_motions(state)
    control(state, 'isConnectOffline', True)
    control(state, 'motionstreamplay')
    state.stop()
    clock.advance(100.)
    state.tick()
    assert not usb.frames
    assert state.playback_timing['stopped']


def test_sequence_and_load_seed_latest_device_pose_before_trajectory(session):
    state, usb, clock = session
    import_motions(state)
    usb.seed = {JOINT: .7, 'left_hand_index_MCP_joint': .4}
    control(state, 'isConnectOffline', True)
    control(state, 'loadmotion')
    assert state.live[JOINT] == .7
    clock.advance()
    state.tick()
    assert .1 < usb.frames[-1][0][JOINT] <= .7
    state.stop()
    usb.seed = {JOINT: .55}
    control(state, 'isConnectOffline', True)
    control(state, 'motionstreamplay')
    assert state.live[JOINT] == .55


@pytest.mark.parametrize('operation', ['stop', 'disconnect', 'import', 'side'])
def test_replacement_cancels_old_tasks_and_pending_dialogs(session, operation):
    state, usb, clock = session
    import_motions(state)
    control(state, 'isConnectOffline', True)
    control(state, 'motionstreamplay')
    clock.advance()
    state.tick()
    if operation == 'side':
        control(state, 'handSide', 1)
    elif operation == 'import':
        import_motions(state, 2)
    else:
        state.dispatch(operation, {})
    frame_count = len(usb.frames)
    clock.advance(10.)
    state.tick()
    assert len(usb.frames) == frame_count
    assert not state.playing and not state.realtime
    assert state.sync is (operation == 'side')
    assert not state.snapshot()['dialogs']


def test_hand_side_restores_realtime_with_new_planner_and_rejects_stale_controls(session):
    state, usb, clock = session
    control(state, JOINT, .3)
    control(state, 'realtimeJointControl', True)
    control(state, 'no')
    clock.advance()
    state.tick()
    old_id = state.snapshot()['joints'][0]['id']
    old_epoch = state._epoch
    control(state, 'handSide', 1)
    assert state.realtime and not state.sync and state._task is None
    assert state._epoch > old_epoch and state.side == 'right'
    assert usb.commands[-1] == ('side', 'right')
    assert state.live[JOINT] == .3
    with pytest.raises(ValueError):
        state.widget({'id': old_id, 'value': .1})
    clock.advance()
    state.tick()
    assert usb.frames[-1][1]['hand_side'] == 'right'
    assert usb.frames[-1][0][JOINT] == pytest.approx(.3)


def test_hand_type_write_failure_does_not_restore_old_control_mode(session):
    state, usb, clock = session
    control(state, 'realtimeJointControl', True)
    control(state, 'no')
    state.tick()
    usb.send_hand_type = lambda side: False
    with pytest.raises(ValueError, match='配置写入失败'):
        control(state, 'handSide', 1)
    assert state.side == 'left' and not state.realtime and not state.sync
    sent = len(usb.frames)
    clock.advance(1.)
    state.tick()
    assert len(usb.frames) == sent


def test_onekey_homing_excludes_motion_and_cancels_future_commands(session):
    state, usb, clock = session
    control(state, 'oneKeyZero')
    control(state, 'homingIq', 350.)
    control(state, 'confirm')
    assert state.homing.active
    assert usb.commands[0] == ('side', 'left')
    assert usb.commands[1][0] == 'linear'
    clock.advance(1.)
    state.tick()
    assert usb.commands[-1] == ('enable', 1, 1, True)
    state.stop()
    sent = len(usb.commands)
    clock.advance(30.)
    state.tick()
    assert len(usb.commands) == sent and not state.homing.active
    assert not state.snapshot()['dialogs']


def test_linear_home_requires_confirmation_and_reset_cancels_it(session):
    state, usb, _ = session
    control(state, 'allzero')
    control(state, 'homingIq', 450.)
    assert not usb.commands
    control(state, 'cancel')
    assert not usb.commands
    control(state, 'allzero')
    control(state, 'confirm')
    assert usb.commands[-1][0] == 'linear' and usb.commands[-1][-1] == 450.
    control(state, 'allreset')
    assert usb.commands[-1][-1] == 0.
    assert usb.commands[-1][1] != usb.commands[-2][1]


def test_lost_connection_cancels_homing_and_calibration_dialogs(session):
    state, usb, clock = session
    control(state, 'calibrationRotateMotor')
    control(state, 'confirm')
    assert state.calibration
    usb.is_connected = False
    state.tick()
    assert not state.calibration and not state.snapshot()['dialogs']
    usb.is_connected = True
    control(state, 'oneKeyZero')
    control(state, 'confirm')
    usb.is_connected = False
    count = len(usb.commands)
    clock.advance(5.)
    state.tick()
    assert not state.homing.active and len(usb.commands) == count


def test_calibration_dialog_is_nonmodal_and_last_step_requires_manual_navigation(session):
    state, usb, _ = session
    control(state, 'calibrationRotateMotor')
    control(state, 'confirm')
    assert not state.snapshot()['dialogs'][0]['modal']
    control(state, 'calibrationPosition', -42.)
    control(state, 'calibrationSend')
    assert usb.commands[-1] == ('position', 0, 0, -42.)
    for index in range(7):
        control(state, 'calibrationZero')
        assert state.calibration.index == index
        if index < 6:
            control(state, 'calibrationNext')
    assert state.calibration.completed
    control(state, 'confirm')
    control(state, 'calibrationClose')
    assert state.calibration is None


def test_sweep_transport_fences_old_worker_after_stop(session):
    state, usb, _ = session
    transport = _SweepTransport(state, state._epoch)
    assert transport._write_frame(b'frame')
    state.stop()
    assert not transport._write_frame(b'stale')
    assert usb.commands == [('raw', b'frame')]


def test_background_stop_waits_for_inflight_write_and_no_more_writes(tmp_path):
    entered, release = threading.Event(), threading.Event()
    usb = FakeUsb()
    original = usb.send_joint_angles
    def blocked(cfg, **kwargs):
        entered.set()
        assert release.wait(3.)
        return original(cfg, **kwargs)
    usb.send_joint_angles = blocked
    state = V2Session(tmp_path, usb=usb)
    try:
        import_motions(state)
        control(state, 'isConnectOffline', True)
        control(state, 'motionstreamplay')
        assert entered.wait(2.)
        stopped = threading.Event()
        stopper = threading.Thread(target=lambda: (state.stop(), stopped.set()))
        stopper.start()
        assert not stopped.wait(.02)
        release.set()
        assert stopped.wait(2.)
        count = len(usb.frames)
        time.sleep(.025)
        assert len(usb.frames) == count
        stopper.join()
    finally:
        release.set()
        state.close()
    assert not state._worker.is_alive()


def test_all_joint_angles_have_one_ruckig_guarded_exit():
    tree = ast.parse((Path(__file__).parent / 'v2_session.py').read_text())
    methods = [node for cls in tree.body if isinstance(cls, ast.ClassDef)
               for node in cls.body if isinstance(node, ast.FunctionDef)]
    exits = [node for node in methods if any(isinstance(call, ast.Call)
             and isinstance(call.func, ast.Attribute) and call.func.attr == 'send_joint_angles'
             for call in ast.walk(node))]
    assert [node.name for node in exits] == ['_send_step']
    names = {node.attr for node in ast.walk(exits[0]) if isinstance(node, ast.Attribute)}
    assert {'push', 'sample', 'filter', 'send_joint_angles'} <= names
