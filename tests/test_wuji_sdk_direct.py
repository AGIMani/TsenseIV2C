"""Offline SDK lifecycle tests: no installed SDK import, network or robot commands."""
from copy import deepcopy
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from online_control.online_motion_track.wuji_input import WujiInputRelay, resolve_wuji_config
from online_control.online_motion_track.wuji_sdk_source import SkeletonStore
from online_control.online_motion_track.wuji_transport import LatestWujiClient, LatestWujiSdkClient


def wait_until(predicate):
    deadline = time.monotonic() + 2
    while not predicate():
        assert time.monotonic() < deadline, 'worker did not reach expected state'
        time.sleep(.002)


def frame(sequence=1, side='left'):
    return SimpleNamespace(header=SimpleNamespace(seq=sequence, timestamp_us=123,
        frame_id='l_wrist' if side == 'left' else 'r_wrist'),
        joints=[SimpleNamespace(pose=SimpleNamespace(position=[i * .001, i * .002, 0.]),
                                confidence=1.) for i in range(21)])


class FakeGlove:
    serial_number = 'WG_TEST'

    def __init__(self, side='left'):
        self.side, self.is_connected = side, True
        self.callback = None
        self.subscription_closed = False

    def hand_side(self):
        return SimpleNamespace(get=lambda: self.side)

    def hand_skeleton(self):
        return SimpleNamespace(subscribe_with_callback=self.subscribe)

    def subscribe(self, callback):
        self.callback = callback
        return SimpleNamespace(close=lambda: setattr(self, 'subscription_closed', True))


class FakeManager:
    def __init__(self):
        self.scan_calls = 0
        self.connect_calls, self.disconnections, self.gloves = [], [], []
        self.scan_gate = self.connect_gate = None
        self.connect_error = None

    def scan(self):
        self.scan_calls += 1
        if self.scan_gate:
            assert self.scan_gate.wait(2)
        return [SimpleNamespace(sn='WG_TEST', device_type='glove'),
                SimpleNamespace(sn='ROBOT', device_type='robot')]

    def connect(self, **kwargs):
        self.connect_calls.append(kwargs)
        if self.connect_gate:
            assert self.connect_gate.wait(2)
        if self.connect_error:
            raise self.connect_error
        glove = FakeGlove()
        self.gloves.append(glove)
        return glove

    def disconnect(self, *, device_name):
        assert device_name.startswith('hand_track_glove_')
        self.disconnections.append(device_name)

    def disconnect_all(self):
        pytest.fail('must never disconnect another SDK owner')

    def sdk(self):
        return SimpleNamespace(SdkManager=SimpleNamespace(instance=lambda: self),
            DeviceType=SimpleNamespace(WujiGlove='glove'), ConnectOptions=lambda **kw: SimpleNamespace(**kw))


def client_for(manager, **kwargs):
    return LatestWujiSdkClient(sdk_loader=manager.sdk, poll_interval=.002, retry_delay=.01,
                              max_retry_delay=.01, close_timeout=.02, **kwargs)


@pytest.fixture(autouse=True)
def isolated_config(monkeypatch):
    for key in ('WUJI_SOURCE', 'WUJI_ADDRESS', 'WUJI_SERIAL', 'WUJI_BRIDGE_URL'):
        monkeypatch.delenv(key, raising=False)


def test_sdk_default_ignores_old_bridge_environment_and_is_lazy(monkeypatch):
    monkeypatch.setenv('WUJI_BRIDGE_URL', 'http://old-node:8787')
    relay = WujiInputRelay()
    assert isinstance(relay.client, LatestWujiSdkClient)
    relay.create('session', 'left')
    assert relay.client.worker is None
    relay.close('session')
    assert resolve_wuji_config()['source'] == 'sdk'
    assert isinstance(WujiInputRelay('http://127.0.0.1:8787').client, LatestWujiClient)
    assert resolve_wuji_config(source='bridge')['url'] == 'http://old-node:8787'
    with pytest.raises(ValueError):
        resolve_wuji_config('http://old-node:8787', source='sdk')
    monkeypatch.delenv('WUJI_BRIDGE_URL')
    with pytest.raises(ValueError):
        resolve_wuji_config(source='bridge')


def test_callback_metric_shape_age_duplicate_and_delayed_consumption():
    now, manager = [10.], FakeManager()
    client = client_for(manager, clock=lambda: now[0], address='192.168.1.100:50001', serial='WG_TEST')
    relay = WujiInputRelay(clock=lambda: now[0])
    relay.client = client
    relay.create('s', 'left')
    try:
        relay.latest('s')
        wait_until(lambda: manager.gloves and manager.gloves[0].callback)
        manager.gloves[0].callback(frame())
        initial = relay.latest('s')
        assert initial['hand']['world'][0] == [0., 0., 0.]
        assert initial['source_metadata']['source_joint_count'] == 21
        assert manager.scan_calls == 0
        connection = manager.connect_calls[0]
        assert connection['address'] == '192.168.1.100:50001'
        assert not connection['options'].enable_bridge
        assert connection['options'].timeout_ms == 1000
        now[0] += .2
        manager.gloves[0].callback(frame())
        duplicate = relay.latest('s')
        assert duplicate['revision'] == initial['revision']
        assert duplicate['age_ms'] == pytest.approx(200.)
        now[0] += .151
        assert relay.consume('s', initial['revision'])['hand'] is None
        assert relay.latest('s')['hand'] is None
        manager.gloves[0].callback(frame(2))
        assert relay.latest('s')['hand'] is not None
    finally:
        relay.close('s')
        wait_until(lambda: not client.worker['thread'].is_alive())
    assert manager.gloves[0].subscription_closed
    assert len(manager.disconnections) == 1
    manager.gloves[0].callback(frame(999))
    assert client.worker['store'].latest('left')['frame'] is None


def test_disconnect_reconnect_rejects_old_callbacks_and_only_closes_own_connections():
    manager = FakeManager()
    client = client_for(manager)
    try:
        client.latest('left')
        wait_until(lambda: manager.gloves and manager.gloves[0].callback)
        old = manager.gloves[0]
        old.callback(frame())
        old_stream = client.latest('left')['frame']['stream_id']
        old.is_connected = False
        wait_until(lambda: len(manager.gloves) == 2 and manager.gloves[1].callback)
        new = manager.gloves[1]
        new.callback(frame())
        new_snapshot = client.latest('left')
        assert new_snapshot['frame']['stream_id'] != old_stream
        old.callback(frame(999))
        assert client.latest('left')['frame'] == new_snapshot['frame']
        assert old.subscription_closed
        assert manager.connect_calls[0]['sn'] == 'WG_TEST'
        assert all(c.get('sn') != 'ROBOT' for c in manager.connect_calls)
    finally:
        client.close()
        wait_until(lambda: not client.worker['thread'].is_alive())
    assert len(set(manager.disconnections)) == 2


@pytest.mark.parametrize('stage', ['scan', 'connect'])
def test_close_during_blocked_native_call_cannot_subscribe_or_reconnect(stage):
    manager, gate = FakeManager(), threading.Event()
    setattr(manager, stage + '_gate', gate)
    client = client_for(manager)
    client.latest('left')
    wait_until(lambda: manager.scan_calls if stage == 'scan' else manager.connect_calls)
    client.close()
    # A new input request cannot overlap the still-finishing cancelled owner.
    assert client.latest('left')['state'] == 'stopping'
    gate.set()
    wait_until(lambda: not client.worker['thread'].is_alive())
    assert manager.scan_calls == 1
    assert len(manager.connect_calls) == (0 if stage == 'scan' else 1)
    assert all(g.callback is None for g in manager.gloves)
    assert len(manager.disconnections) == (0 if stage == 'scan' else 1)


def test_last_session_close_cancels_worker_but_other_session_keeps_it_alive():
    manager = FakeManager()
    client = client_for(manager)
    relay = WujiInputRelay()
    relay.client = client
    relay.create('one', 'left')
    relay.create('two', 'right')
    relay.latest('one')
    wait_until(lambda: manager.gloves and manager.gloves[0].callback)
    relay.close('one')
    assert not client.worker['stop'].is_set()
    relay.close('two')
    wait_until(lambda: not client.worker['thread'].is_alive())
    assert client.worker['stop'].is_set()
    with pytest.raises(ValueError):
        relay.latest('one')
    relay.create('three', 'left')
    relay.latest('three')
    wait_until(lambda: len(manager.gloves) == 2 and manager.gloves[1].callback)
    relay.close('three')


def test_sdk_import_and_connection_failures_are_visible_and_cancellation_stops_retry(monkeypatch):
    monkeypatch.delenv('WUJI_TELEMETRY_DISABLE', raising=False)
    def missing_sdk():
        import os
        assert os.environ['WUJI_TELEMETRY_DISABLE'] == '1'
        raise ImportError('fake missing SDK')
    client = LatestWujiSdkClient(sdk_loader=missing_sdk)
    client.latest('left')
    wait_until(lambda: client.latest('left')['state'] == 'sdk_error')
    assert 'fake missing SDK' in client.latest('left')['error']
    client.close()
    manager = FakeManager()
    manager.connect_error = RuntimeError('fake cable disconnected')
    client = client_for(manager, address='192.168.1.100:50001')
    client.latest('left')
    wait_until(lambda: client.latest('left')['error'])
    assert 'fake cable disconnected' in client.latest('left')['error']
    client.close()
    wait_until(lambda: not client.worker['thread'].is_alive())
    assert client.worker['store'].latest('left')['frame'] is None


def test_invalid_callback_clears_data_without_refreshing_duplicate_frame():
    now, manager = [10.], FakeManager()
    client = client_for(manager, clock=lambda: now[0])
    try:
        client.latest('left')
        wait_until(lambda: manager.gloves and manager.gloves[0].callback)
        callback = manager.gloves[0].callback
        callback(frame())
        broken = frame(2)
        broken.joints[0].pose.position[0] = float('nan')
        callback(broken)
        bad = client.latest('left')
        assert bad['frame'] is None and bad['state'] == 'invalid_frame' and bad['error']
        now[0] += 1
        callback(frame())
        assert client.latest('left')['frame'] is None
        callback(frame(3))
        assert client.latest('left')['frame']['sequence'] == 3
    finally:
        client.close()


def test_serial_mismatch_releases_connection_without_subscribing():
    manager = FakeManager()
    client = client_for(manager, address='192.168.1.100:50001', serial='DIFFERENT')
    try:
        client.latest('left')
        wait_until(lambda: client.latest('left')['error'])
        assert '序列号' in client.latest('left')['error']
        assert all(g.callback is None for g in manager.gloves)
        assert manager.disconnections
    finally:
        client.close()


def test_failed_cleanup_blocks_reconnect_and_close_retries_only_original_owner():
    manager = FakeManager()
    broken = [True]
    disconnect = manager.disconnect
    def fail_then_recover(*, device_name):
        if broken[0]:
            raise RuntimeError('fake disconnect failure')
        disconnect(device_name=device_name)
    manager.disconnect = fail_then_recover
    client = client_for(manager)
    client.latest('left')
    wait_until(lambda: manager.gloves and manager.gloves[0].callback)
    manager.gloves[0].is_connected = False
    wait_until(lambda: not client.worker['thread'].is_alive())
    assert len(manager.connect_calls) == 1
    failed = client.latest('left')
    assert failed['state'] == 'disconnect_error' and 'fake disconnect failure' in failed['error']
    assert client.worker['store'].pending_cleanup
    client.close()
    wait_until(lambda: not client.worker['thread'].is_alive())
    assert client.latest('left')['state'] == 'disconnect_error'
    assert len(manager.connect_calls) == 1
    broken[0] = False
    client.close()
    wait_until(lambda: not client.worker['thread'].is_alive())
    assert not client.worker['store'].pending_cleanup
    assert manager.disconnections == [manager.connect_calls[0]['device_name']]


def test_default_sdk_callback_reaches_real_application_a32_solver_without_bridge(monkeypatch, tmp_path):
    from online_control.online_motion_track.main import Application
    from a32_driver.native import NativeHand

    def forbidden_bridge(*args, **kwargs):
        pytest.fail('the default SDK input must never construct an HTTP bridge client')
    def forbidden_robot_connection(*args, **kwargs):
        pytest.fail('skeleton preview must never open robot hardware')
    monkeypatch.setattr(LatestWujiClient, '__init__', forbidden_bridge)
    monkeypatch.setattr(NativeHand, 'connect_usb', forbidden_robot_connection)
    monkeypatch.setattr(NativeHand, 'connect_eth', forbidden_robot_connection)

    manager = FakeManager()
    app = Application(log_dir=tmp_path)
    assert app.wuji.source == 'sdk'
    assert isinstance(app.wuji.client, LatestWujiSdkClient)
    client = app.wuji.client
    client.sdk_loader = manager.sdk
    try:
        token = app.create({'model_id': 'TsenseI_v3_2_withid', 'side': 'left',
                            'input_source': 'wuji_glove', 'recording': False})['session']
        session = app.get(token)
        world = session.pipeline.session.retargeter.fk.compute().tolist()
        app.wuji_create({'session': token})
        app.wuji_latest({'session': token})
        worker = app.metric_workers[token]
        client_seen, control_seen = session.last_seen, session.control.client_at
        wait_until(lambda: manager.gloves and manager.gloves[0].callback)
        observation = frame()
        for joint, position in zip(observation.joints, world, strict=True):
            joint.pose.position = position
        manager.gloves[0].callback(observation)

        # No further browser request is needed to solve or commit the target.
        wait_until(lambda: worker.read().get('tracking'))
        assert session.last_seen == client_seen
        assert session.control.client_at == control_seen

        result = app.wuji_frame({'session': token, 'sequence': 0, 'timestamp_ms': 1000.,
            'capture_unix_ms': 1_700_000_000_000,
            'hand': {'label': 'Right', 'world': [[999., 999., 999.]] * 21}})
        assert result['tracking'] and result['update']['tracking'], result['update'].get('message')
        assert result['source_metadata']['transport']['mode'] == 'sdk'
        assert result['raw_wuji']['positions_xyz'] == world
        assert result['raw_wuji']['serial_number'] == 'WG_TEST'
        assert result['hand']['label'] == 'Left'
        points = result['hand']['world']
        assert points[2] == [(points[1][axis] + points[3][axis]) * .5 for axis in range(3)]
        solved_input = result['update']['points']
        for axis in range(3):
            assert abs(solved_input[2][axis] - (solved_input[1][axis] + solved_input[3][axis]) * .5) < 1e-10
        assert result['update']['source'] == 'wuji_glove'
        assert result['update']['depth_source'] == 'wuji_glove_metric'
        assert result['update']['input_path'] == 'server_metric_worker'
        assert len(result['update']['retarget_joint_angles_rad']) == 20
        assert app.hardware_owner is None
        assert not session.control.armed and not session.control.device.is_connected
    finally:
        app.close()
    wait_until(lambda: not client.worker['thread'].is_alive())
    assert not client.worker['store'].pending_cleanup
    assert manager.gloves[0].subscription_closed
    assert manager.disconnections == [manager.connect_calls[0]['device_name']]
