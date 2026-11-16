"""Exercise Manus snapshots through the real A3.2 solver without opening USB."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import urllib.request

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src')]
from online_control.online_motion_track.manus_input import ManusInputRelay
from online_control.online_motion_track.manus_source import FINGER_SPEC, hand_frame, mediapipe_rows
from online_control.online_motion_track.source_worker import SourceWorker


class Clock:
    value = 100.

    def __call__(self):
        return self.value


def skeleton(points, clock, *, side='left', sequence=1, glove_id=42, timestamp=None):
    nodes = [SimpleNamespace(node_id=70, parent_id=70, chain_type=13, finger_joint_type=0, side=1 if side == 'left' else 2)]
    nodes.extend(SimpleNamespace(node_id=100+i, parent_id=70, chain_type=chain, finger_joint_type=joint, side=1 if side == 'left' else 2)
                 for i, (chain, joint) in enumerate(FINGER_SPEC))
    data = np.zeros((21, 10))
    data[:, :3] = np.asarray(points) + [1., 2., 3.]
    data[:, 3] = 1.
    data[:, 7:] = 1.
    order = np.random.default_rng(71).permutation(21)
    return [nodes[i] for i in order], dict(skeleton=data[order], node_ids=[nodes[i].node_id for i in order], valid=True, sequence=sequence,
        host_arrival_ns=int(clock()*1e9), glove_id=glove_id, source_timestamp_raw=123455+sequence if timestamp is None else timestamp,
        source_timestamp_ns=None, source_clock_domain='manus_sdk_compressed_timestamp', tracking_valid=None)


class FakeGlove:
    def __init__(self, clock):
        self.clock = clock
        self.lock = threading.Lock()
        self.frames = {}
        self.connections = self.disconnections = 0
        self.connected = False
        self.calibrations = []
        self.calibration_result = 1

    def publish(self, points, side='left', **options):
        with self.lock:
            self.frames[side] = skeleton(points, self.clock, side=side, **options)

    def connect(self, **options):
        assert options['world_coordinates'] is True
        self.connected = True
        self.connections += 1
        return True

    def disconnect(self):
        self.connected = False
        self.disconnections += 1

    def is_connected(self):
        return self.connected

    def get_glove_id(self, side):
        with self.lock:
            return self.frames[side][1]['glove_id'] if side in self.frames else 0

    def get_node_info(self, side):
        with self.lock:
            return deepcopy(self.frames[side][0])

    def get_raw_skeleton_snapshot(self, side):
        with self.lock:
            return deepcopy(self.frames[side][1])

    def set_calibration(self, side, data):
        self.calibrations.append((side, data))
        return self.calibration_result


def wait_for(read, predicate=lambda value: value['hand'] is not None):
    deadline = time.monotonic()+3
    while time.monotonic() < deadline:
        value = read()
        if predicate(value):
            return value
        time.sleep(.005)
    pytest.fail(f'Timed out waiting for Manus: {value}')


@pytest.fixture
def device():
    clock = Clock()
    glove = FakeGlove(clock)
    relay = ManusInputRelay(clock=clock, sdk_loader=lambda: SimpleNamespace(ManusGlove=lambda: glove), retry_delay=.01)
    yield clock, glove, relay
    for session in list(relay.channels):
        relay.close(session)
    relay.client.close()


def test_semantic_mapping_preserves_metres_and_chirality():
    points = np.random.default_rng(12).normal(size=(21, 3))*.03
    for side in ('left', 'right'):
        nodes, snapshot = skeleton(points, Clock(), side=side)
        raw, hand = hand_frame(snapshot, nodes, side)
        np.testing.assert_allclose(hand['world'], points-points[0], atol=1e-14)
        assert hand['label'] == side.title()
        assert hand['normalized_is_image'] is False
        assert raw['source_timestamp_raw'] == '123456'
        assert raw['tracking_valid'] is None
        assert raw['joint_mapping'] != list(range(21))


def test_missing_ambiguous_and_nonfinite_nodes_are_rejected():
    nodes, snapshot = skeleton(np.zeros((21, 3)), Clock())
    with pytest.raises(ValueError, match='缺少'):
        mediapipe_rows(nodes[:-1])
    with pytest.raises(ValueError, match='重复'):
        duplicate = SimpleNamespace(**vars(nodes[0]))
        duplicate.node_id = 999
        mediapipe_rows(nodes+[duplicate])
    snapshot['skeleton'][0, 1] = np.nan
    with pytest.raises(ValueError, match='有限'):
        hand_frame(snapshot, nodes, 'left')


def test_untyped_wrist_root_is_supported():
    nodes, _ = skeleton(np.zeros((21, 3)), Clock())
    wrist = next(node for node in nodes if node.node_id == 70)
    wrist.chain_type = 0
    assert nodes[mediapipe_rows(nodes)[0]].node_id == 70


def test_repeated_frames_age_out_and_recovery_changes_epoch(device):
    clock, glove, relay = device
    glove.publish(np.zeros((21, 3)))
    relay.create('a', 'left')
    first = wait_for(lambda: relay.latest('a'))
    assert relay.consume('a', first['revision'])['hand'] is not None
    assert relay.consume('a', first['revision'])['hand'] is None
    clock.value += .15
    repeated = relay.latest('a')
    assert repeated['revision'] == first['revision']
    assert repeated['age_ms'] == pytest.approx(150.)
    clock.value += .06
    assert relay.latest('a')['hand'] is None
    called = []
    assert not relay.commit_if_current('a', first, lambda: called.append(True))
    assert not called
    glove.publish(np.zeros((21, 3)), sequence=2)
    recovered = wait_for(lambda: relay.latest('a'))
    assert recovered['source_metadata']['source_epoch'] > first['source_metadata']['source_epoch']
    relay.close('a')
    assert not relay.commit_if_current('a', recovered, lambda: called.append(True))


def test_sides_share_connection_and_right_updates_do_not_refresh_left(device):
    clock, glove, relay = device
    glove.publish(np.zeros((21, 3)))
    glove.publish(np.ones((21, 3))*.01, side='right', glove_id=43)
    relay.create('left', 'left')
    relay.create('right', 'right')
    left = wait_for(lambda: relay.latest('left'))
    wait_for(lambda: relay.latest('right'))
    assert glove.connections == 1
    clock.value += .25
    glove.publish(np.ones((21, 3))*.02, side='right', sequence=2, glove_id=43)
    wait_for(lambda: relay.latest('right'))
    assert relay.latest('left')['revision'] == left['revision']
    assert relay.latest('left')['hand'] is None
    relay.close('left')
    assert glove.connected
    relay.close('right')
    relay.client.close()
    assert glove.disconnections == 1


def test_old_callback_and_replaced_glove_cannot_reuse_saved_frame(device):
    clock, glove, relay = device
    glove.publish(np.zeros((21, 3)), sequence=5)
    relay.create('a', 'left')
    first = wait_for(lambda: relay.latest('a'))
    clock.value += .1
    glove.publish(np.ones((21, 3)), sequence=4)
    time.sleep(.03)
    assert relay.latest('a')['revision'] == first['revision']
    assert relay.latest('a')['age_ms'] == pytest.approx(100)
    glove.publish(np.zeros((21, 3)), glove_id=99)
    replaced = wait_for(lambda: relay.latest('a'), lambda x:x['source_metadata']['sequence']==1)
    assert replaced['source_metadata']['source_epoch'] > first['source_metadata']['source_epoch']
    assert relay.consume('a', first['revision'])['hand'] is None


@pytest.mark.parametrize('result', [1, 2])
def test_calibration_applied_once_and_requires_new_callback(tmp_path, result):
    clock = Clock()
    glove = FakeGlove(clock)
    glove.calibration_result = result
    glove.publish(np.zeros((21, 3)))
    calibration = tmp_path/'left.mcal'
    calibration.write_bytes(b'test calibration')
    relay = ManusInputRelay(clock=clock, left_calibration=calibration,
        sdk_loader=lambda: SimpleNamespace(ManusGlove=lambda: glove))
    relay.create('a', 'left')
    try:
        first = wait_for(lambda: relay.latest('a'), lambda x:len(glove.calibrations)==1)
        assert first['hand'] is None
        clock.value += .01
        glove.publish(np.zeros((21, 3)), sequence=2)
        if result == 1:
            wait_for(lambda: relay.latest('a'))
        else:
            failed = wait_for(lambda: relay.latest('a'), lambda x:x['source_metadata']['error'] is not None)
            assert 'code=2' in failed['source_metadata']['error']
            assert failed['hand'] is None
        time.sleep(.03)
        assert glove.calibrations == [('left', b'test calibration')]
    finally:
        relay.close('a')
        relay.client.close()


def test_cancel_during_connect_does_not_overlap_sdk_owners():
    clock = Clock()
    entered, release = threading.Event(), threading.Event()
    first, second = FakeGlove(clock), FakeGlove(clock)
    original_connect = first.connect
    def blocked_connect(**options):
        entered.set()
        assert release.wait(2)
        return original_connect(**options)
    first.connect = blocked_connect
    a = ManusInputRelay(clock=clock, sdk_loader=lambda:SimpleNamespace(ManusGlove=lambda:first))
    b = ManusInputRelay(clock=clock, sdk_loader=lambda:SimpleNamespace(ManusGlove=lambda:second))
    a.create('a', 'left')
    b.create('b', 'left')
    try:
        a.latest('a')
        assert entered.wait(2)
        a.close('a')
        b.latest('b')
        time.sleep(.03)
        assert second.connections == 0
        release.set()
        wait_for(lambda:b.latest('b'), lambda x:second.connections==1)
        assert first.disconnections == 1
    finally:
        release.set()
        a.close('a')
        a.client.close()
        b.close('b')
        b.client.close()


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('thumb_joint', [3, 4])
def test_atomic_node_ids_join_independently_ordered_metadata_and_preserve_thumb(side, thumb_joint):
    points = np.random.default_rng(14).normal(size=(21, 3)) * .04
    nodes, snapshot = skeleton(points, Clock(), side=side)
    # The callback array and SDK hierarchy may have entirely different orders.
    nodes.reverse()
    thumb = next(node for node in nodes if node.chain_type == 5 and node.finger_joint_type == 4)
    thumb.finger_joint_type = thumb_joint
    original = snapshot['skeleton'].copy()
    raw, hand = hand_frame(snapshot, nodes, side)
    np.testing.assert_allclose(hand['world'], points - points[0], atol=1e-14)
    assert not np.allclose(hand['world'][2], (np.array(hand['world'][1])+hand['world'][3])/2)
    assert hand['label'] == side.title()
    assert raw['node_ids'] == snapshot['node_ids']
    assert 'confidences' not in raw and 'confidences' not in hand
    np.testing.assert_array_equal(original, snapshot['skeleton'])


def test_native_25_node_skeleton_omits_only_four_finger_metacarpals():
    points = np.random.default_rng(23).normal(size=(21, 3)) * .03
    nodes, snapshot = skeleton(points, Clock())
    for chain in (6, 7, 8, 9):
        node_id = 900 + chain
        nodes.append(SimpleNamespace(node_id=node_id, parent_id=70, chain_type=chain,
                                     finger_joint_type=1, side=1))
        next(node for node in nodes if node.chain_type == chain and node.finger_joint_type == 2).parent_id = node_id
        snapshot['node_ids'].append(node_id)
        snapshot['skeleton'] = np.vstack([snapshot['skeleton'], [[1.2, 2.3, 3.4, 1, 0, 0, 0, 1, 1, 1]]])
    nodes.reverse()
    raw, hand = hand_frame(snapshot, nodes, 'left')
    assert len(raw['skeleton']) == 25 and len(raw['joint_mapping']) == 21
    np.testing.assert_allclose(hand['world'], points - points[0], atol=1e-14)


@pytest.mark.parametrize('cause', ['missing_ids', 'duplicate_ids', 'unmatched_ids', 'wrong_side', 'cycle'])
def test_invalid_atomic_node_identity_and_topology_are_rejected(cause):
    nodes, snapshot = skeleton(np.zeros((21, 3)), Clock())
    if cause == 'missing_ids':
        snapshot.pop('node_ids')
    elif cause == 'duplicate_ids':
        snapshot['node_ids'][0] = snapshot['node_ids'][1]
    elif cause == 'unmatched_ids':
        snapshot['node_ids'][0] = 999999
    elif cause == 'wrong_side':
        nodes[0].side = 2
    else:
        a, b = [node for node in nodes if node.chain_type == 6][:2]
        a.parent_id, b.parent_id = b.node_id, a.node_id
    with pytest.raises(ValueError):
        hand_frame(snapshot, nodes, 'left')


@pytest.mark.parametrize('timestamp', [(1 << 64)-1, str((1 << 64)-1)])
def test_packed_timestamp_is_lossless_decimal_string(timestamp):
    nodes, snapshot = skeleton(np.zeros((21, 3)), Clock(), timestamp=timestamp)
    raw, _ = hand_frame(snapshot, nodes, 'left')
    assert raw['source_timestamp_raw'] == '18446744073709551615'
    assert raw['source_timestamp_ns'] is None
    assert snapshot['source_timestamp_raw'] == timestamp


@pytest.mark.parametrize('timestamp', [True, -1, 1 << 64, '01', '-1', '１', ' 1'])
def test_invalid_packed_timestamp_is_rejected(timestamp):
    nodes, snapshot = skeleton(np.zeros((21, 3)), Clock(), timestamp=timestamp)
    with pytest.raises(ValueError):
        hand_frame(snapshot, nodes, 'left')


def test_repeated_sdk_timestamp_cannot_renew_host_age_or_revision(device):
    clock, glove, relay = device
    glove.publish(np.zeros((21, 3)), timestamp=(1 << 60))
    relay.create('a', 'left')
    first = wait_for(lambda: relay.latest('a'))
    clock.value += .12
    glove.publish(np.zeros((21, 3)), sequence=2, timestamp=(1 << 60))
    time.sleep(.03)
    duplicate = relay.latest('a')
    assert duplicate['revision'] == first['revision']
    assert duplicate['age_ms'] == pytest.approx(120)
    clock.value += .09
    glove.publish(np.zeros((21, 3)), sequence=3, timestamp=(1 << 60))
    time.sleep(.03)
    assert relay.latest('a')['hand'] is None
    # Packed values are identities, never compared as numeric capture times.
    glove.publish(np.zeros((21, 3)), sequence=4, timestamp=1)
    assert wait_for(lambda: relay.latest('a'))['hand'] is not None


def test_old_solve_cannot_commit_while_newer_frames_are_fresh(device):
    clock, glove, relay = device
    glove.publish(np.zeros((21, 3)))
    relay.create('a', 'left')
    first = wait_for(lambda: relay.latest('a'))
    for sequence in (2, 3, 4):
        clock.value += .08
        glove.publish(np.zeros((21, 3)), sequence=sequence)
        current = wait_for(lambda: relay.latest('a'), lambda x:x['source_metadata']['sequence'] == sequence)
    assert current['hand'] is not None
    assert current['source_metadata']['source_epoch'] == first['source_metadata']['source_epoch']
    assert not relay.commit_if_current('a', first, lambda: pytest.fail('stale solved target'))
    assert relay.commit_if_current('a', current, lambda: None)


@pytest.mark.parametrize('cause', ['incomplete', 'nonfinite', 'future_arrival', 'wrong_side'])
def test_invalid_new_snapshot_immediately_clears_valid_frame(device, cause):
    clock, glove, relay = device
    glove.publish(np.zeros((21, 3)))
    relay.create('a', 'left')
    first = wait_for(lambda: relay.latest('a'))
    with glove.lock:
        nodes, snapshot = skeleton(np.zeros((21, 3)), clock, sequence=2)
        if cause == 'incomplete':
            snapshot['skeleton'] = snapshot['skeleton'][:-1]
        elif cause == 'nonfinite':
            snapshot['skeleton'][0, 0] = float('nan')
        elif cause == 'future_arrival':
            snapshot['host_arrival_ns'] += 1_000_000
        else:
            nodes[0].side = 2
        glove.frames['left'] = (nodes, snapshot)
    lost = wait_for(lambda: relay.latest('a'), lambda x:x['source_metadata']['event'] == 'invalid_frame')
    assert lost['hand'] is None
    assert relay.consume('a', first['revision'])['hand'] is None
    assert not relay.commit_if_current('a', first, lambda: pytest.fail('invalid input target'))


def test_reconnect_resets_sequence_but_old_owner_cannot_publish(device):
    clock, glove, relay = device
    glove.publish(np.zeros((21, 3)), sequence=50)
    relay.create('a', 'left')
    first = wait_for(lambda: relay.latest('a'))
    old_worker = relay.client.worker
    relay.close('a')
    relay.client.close()
    clock.value += .01
    glove.publish(np.ones((21, 3))*.01, sequence=1)
    relay.create('b', 'left')
    fresh = wait_for(lambda: relay.latest('b'))
    assert fresh['source_metadata']['stream_id'] != first['source_metadata']['stream_id']
    assert fresh['source_metadata']['sequence'] == 1
    nodes, snapshot = skeleton(np.ones((21, 3)), clock, sequence=999)
    relay.client._publish(old_worker, 'left', snapshot, nodes)
    assert relay.latest('b')['revision'] == fresh['revision']


def test_cancel_during_sdk_import_never_constructs_a_native_owner():
    clock = Clock()
    entered, release = threading.Event(), threading.Event()
    calls = []
    def loader():
        entered.set()
        assert release.wait(2)
        return SimpleNamespace(ManusGlove=lambda: calls.append('constructed'))
    relay = ManusInputRelay(clock=clock, sdk_loader=loader)
    relay.create('a', 'left')
    try:
        relay.latest('a')
        assert entered.wait(2)
        relay.close('a')
        release.set()
        relay.client.close()
        assert calls == []
    finally:
        release.set()
        relay.client.close()


def test_failed_cleanup_keeps_sdk_owner_until_disconnect_succeeds():
    clock = Clock()
    first, second = FakeGlove(clock), FakeGlove(clock)
    release = threading.Event()
    attempts = []
    original = first.disconnect
    def failing_disconnect():
        attempts.append(1)
        if not release.is_set():
            raise RuntimeError('temporarily busy')
        original()
    first.disconnect = failing_disconnect
    a = ManusInputRelay(clock=clock, sdk_loader=lambda:SimpleNamespace(ManusGlove=lambda:first), retry_delay=.01)
    b = ManusInputRelay(clock=clock, sdk_loader=lambda:SimpleNamespace(ManusGlove=lambda:second), retry_delay=.01)
    a.create('a', 'left')
    b.create('b', 'left')
    try:
        wait_for(lambda:a.latest('a'), lambda x:first.connections == 1)
        a.close('a')
        wait_for(lambda:b.latest('b'), lambda x:bool(attempts))
        assert second.connections == 0
        release.set()
        wait_for(lambda:b.latest('b'), lambda x:second.connections == 1)
        assert first.disconnections == 1
    finally:
        release.set()
        a.close('a'); a.client.close()
        b.close('b'); b.client.close()


def test_close_requests_native_cancellation_without_waiting_for_connect():
    clock = Clock()
    glove = FakeGlove(clock)
    entered, cancelled = threading.Event(), threading.Event()
    def connect(**options):
        entered.set()
        assert cancelled.wait(2)
        return False
    glove.connect = connect
    glove.request_disconnect = cancelled.set
    relay = ManusInputRelay(clock=clock, sdk_loader=lambda:SimpleNamespace(ManusGlove=lambda:glove))
    relay.create('a', 'left')
    try:
        relay.latest('a')
        assert entered.wait(2)
        relay.close('a')
        assert cancelled.is_set()
        relay.client.close()
        assert glove.disconnections == 1
    finally:
        cancelled.set()
        relay.client.close()


def test_cancel_after_blocked_device_query_does_not_apply_calibration(tmp_path):
    clock = Clock()
    glove = FakeGlove(clock)
    glove.publish(np.zeros((21, 3)))
    entered, release = threading.Event(), threading.Event()
    original = glove.get_glove_id
    def blocked(side):
        entered.set()
        assert release.wait(2)
        return original(side)
    glove.get_glove_id = blocked
    path = tmp_path/'left.mcal'
    path.write_bytes(b'explicit calibration')
    relay = ManusInputRelay(clock=clock, left_calibration=path,
        sdk_loader=lambda:SimpleNamespace(ManusGlove=lambda:glove))
    relay.create('a', 'left')
    try:
        relay.latest('a')
        assert entered.wait(2)
        relay.close('a')
        release.set()
        relay.client.close()
        assert glove.calibrations == [] and glove.disconnections == 1
    finally:
        release.set()
        relay.client.close()


def test_mode_timeout_and_calibration_options_are_preserved(tmp_path):
    for mode in ('integrated', 'local', 'remote'):
        relay = ManusInputRelay(mode=mode, connect_timeout=17)
        assert relay.client.mode == mode and relay.client.connect_timeout == 17
    with pytest.raises(ValueError):
        ManusInputRelay(mode='bridge')
    with pytest.raises(ValueError):
        ManusInputRelay(connect_timeout=True)
    with pytest.raises(ValueError):
        ManusInputRelay(left_calibration=tmp_path/'missing.mcal')


def test_browser_lease_expires_independently_of_new_sdk_frames(device):
    clock, glove, relay = device
    glove.publish(np.zeros((21, 3)))
    relay.create('a', 'left')
    wait_for(lambda: relay.latest('a'))
    worker = SourceWorker('a', object(), relay, 'manus_glove', clock=clock)
    clock.value += 2.01
    glove.publish(np.zeros((21, 3)), sequence=2)
    snapshot = wait_for(lambda: relay.latest('a'))
    assert not worker._commit(snapshot, clock(), lambda: pytest.fail('expired UI lease'))
