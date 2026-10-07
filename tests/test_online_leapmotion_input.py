"""Offline tests of the official LeapC callback adapter; no device/robot access."""
from copy import deepcopy
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace as NS

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from online_control.online_motion_track.leapmotion_input import LeapMotionInputRelay
from online_control.online_motion_track.leapmotion_source import FINGERS, BONES, hand_frame


def wait_until(predicate):
    deadline = time.monotonic() + 3
    while not predicate():
        assert time.monotonic() < deadline, 'Leap worker did not reach expected state'
        time.sleep(.002)


def vector(i):
    return NS(x=100+i, y=200+2*i, z=-300+3*i)


def hand(side='left', identity=1):
    fingers = {name: NS(**{bone: NS(prev_joint=vector(i*10+j), next_joint=vector(i*10+j+1))
                          for j, bone in enumerate(BONES)}) for i, name in enumerate(FINGERS)}
    return NS(id=identity, type='HandType.'+side.title(), confidence=1.,
              arm=NS(prev_joint=vector(-50), next_joint=vector(0)), **fingers)


def frame(sequence=1, hands=None, timestamp=1_000_000):
    return NS(tracking_frame_id=sequence, timestamp=timestamp,
              hands=[hand()] if hands is None else hands)


class FakeSdk:
    Listener = object
    exceptions = NS(LeapTimeoutError=type('LeapTimeoutError', (Exception,), {}))
    TrackingMode = NS(Desktop='desktop')

    def __init__(self):
        self.connections = []
        self.native_now = 1_000_000
        self.connect_gate = None
        self.connect_error = None
        self.disconnect_errors = 0

    def get_now(self):
        return self.native_now

    def Connection(self, *, poll_timeout):
        sdk = self

        class Connection:
            def __init__(self):
                self.listener = None
                self.ready = False
                self.closed = False
                self.disconnect_calls = 0
                self.tracking_mode_calls = 0

            def add_listener(self, listener):
                self.listener = listener

            def remove_listener(self, listener):
                assert listener is self.listener
                self.listener = None

            def connect(self, *, timeout):
                assert timeout <= 1.
                if sdk.connect_gate:
                    assert sdk.connect_gate.wait(2)
                if sdk.connect_error:
                    raise sdk.connect_error
                self.listener.on_connection_event(NS())

            def set_tracking_mode(self, mode):
                assert mode == 'desktop'
                self.tracking_mode_calls += 1
                self.ready = True

            def disconnect(self):
                self.disconnect_calls += 1
                if sdk.disconnect_errors:
                    sdk.disconnect_errors -= 1
                    raise RuntimeError('temporary cleanup failure')
                self.closed = True

        connection = Connection()
        self.connections.append(connection)
        return connection

    def emit(self, event):
        self.connections[-1].listener.on_tracking_event(event)


@pytest.fixture
def reader():
    now = [10.]
    sdk = FakeSdk()
    relay = LeapMotionInputRelay(clock=lambda: now[0], sdk_loader=lambda: sdk, retry_delay=.01)
    relay.create('left', 'left')
    relay.latest('left')
    wait_until(lambda: sdk.connections and sdk.connections[-1].ready)
    yield relay, sdk, now
    for session in list(relay.channels):
        relay.close(session)
    relay.client.close()
    assert not relay.client.worker['thread'].is_alive()


def test_mapping_thumb_cmc_order_units_axes_and_raw_bones():
    side, raw, metric = hand_frame(hand(), sequence=1, timestamp_us=42, stream_id='stream')
    assert side == 'left'
    assert metric['world'][0] == [0., 0., 0.]
    expected = [0] + [i*10+j for i in range(5) for j in (1, 2, 3, 4)]
    for point, source in zip(metric['world'], expected):
        assert point == pytest.approx([source*.001, source*.002, source*.003])
    assert raw['units'] == 'mm'
    assert raw['digits']['thumb']['metacarpal']['prev_joint'] == [100, 200, -300]
    assert raw['digits']['thumb']['proximal']['prev_joint'] == [101, 202, -297]
    assert metric['normalized_is_image'] is False
    assert metric['normalized'] == metric['world']


def test_optional_import_and_reader_are_lazy():
    calls = []
    relay = LeapMotionInputRelay(sdk_loader=lambda: calls.append(True))
    relay.create('one', 'left')
    assert relay.supports_background
    assert calls == [] and relay.client.worker is None
    relay.close('one')
    with pytest.raises(ValueError):
        relay.create('one', 'unknown')


def test_both_sides_share_reader_and_close_only_last_session(reader):
    relay, sdk, _ = reader
    relay.create('right', 'right')
    sdk.emit(frame(hands=[hand('left'), hand('right', 2)]))
    assert relay.latest('left')['hand']['label'] == 'Left'
    assert relay.latest('right')['hand']['label'] == 'Right'
    assert len(sdk.connections) == 1
    relay.close('left')
    assert not sdk.connections[0].closed
    relay.close('right')
    relay.client.close()
    assert sdk.connections[0].closed


def test_service_latency_polling_duplicates_and_expiry_do_not_refresh_age(reader):
    relay, sdk, now = reader
    sdk.native_now = 1_100_000
    sdk.emit(frame())
    first = relay.latest('left')
    assert first['age_ms'] == pytest.approx(100)
    assert first['tracking']
    now[0] += .1
    sdk.emit(frame())
    repeated = relay.latest('left')
    assert repeated['revision'] == first['revision']
    assert repeated['age_ms'] == pytest.approx(200)
    now[0] += .16
    expired = relay.latest('left')
    assert expired['hand'] is None
    assert expired['age_ms'] == pytest.approx(360)
    assert expired['raw_leapmotion'] == first['raw_leapmotion']
    assert not relay.commit_if_current('left', first, lambda: pytest.fail('expired commit'))


def test_repeated_and_out_of_order_frame_or_timestamp_is_ignored(reader):
    relay, sdk, now = reader
    sdk.native_now = 1_010_000
    sdk.emit(frame(sequence=10, timestamp=1_005_000))
    before = relay.latest('left')
    for sequence, timestamp in ((10, 1_006_000), (9, 1_007_000), (11, 1_004_000)):
        sdk.emit(frame(sequence=sequence, timestamp=timestamp))
        assert relay.latest('left')['revision'] == before['revision']
    now[0] += .4
    assert not relay.latest('left')['tracking']


def test_selected_side_loss_identity_and_recovery_invalidate_saved_work(reader):
    relay, sdk, now = reader
    sdk.emit(frame())
    first = relay.latest('left')
    epoch = first['source_metadata']['source_epoch']
    sdk.native_now += 1000
    sdk.emit(frame(2, [hand('right')], timestamp=sdk.native_now))
    lost = relay.latest('left')
    assert lost['hand'] is None and lost['source_metadata']['event'] == 'tracking_lost'
    assert lost['source_metadata']['source_epoch'] != epoch
    assert relay.consume('left', first['revision'])['hand'] is None
    assert not relay.commit_if_current('left', first, lambda: pytest.fail('lost commit'))
    sdk.native_now += 1000
    sdk.emit(frame(3, [hand(identity=7)], timestamp=sdk.native_now))
    recovered = relay.latest('left')
    assert recovered['tracking']
    sdk.native_now += 1000
    sdk.emit(frame(4, [hand(identity=8)], timestamp=sdk.native_now))
    replaced = relay.latest('left')
    assert replaced['source_metadata']['source_epoch'] != recovered['source_metadata']['source_epoch']
    assert not relay.commit_if_current('left', recovered, lambda: pytest.fail('old identity'))
    now[0] += .36
    sdk.native_now += 1000
    sdk.emit(frame(5, [hand(identity=8)], timestamp=sdk.native_now))
    assert relay.latest('left')['source_metadata']['source_epoch'] != replaced['source_metadata']['source_epoch']


def test_consume_exactly_once_copies_data_and_rechecks_expiry(reader):
    relay, sdk, now = reader
    sdk.emit(frame())
    first = relay.latest('left')
    first['hand']['world'][1][0] = 99
    first['raw_leapmotion']['digits'].clear()
    consumed = relay.consume('left', first['revision'])
    assert consumed['hand']['world'][1][0] != 99
    assert consumed['raw_leapmotion']['digits']
    assert consumed['source_metadata']['raw_leapmotion']['digits']
    assert relay.consume('left', first['revision'])['hand'] is None
    sdk.native_now += 1000
    sdk.emit(frame(2, timestamp=sdk.native_now))
    second = relay.latest('left')
    now[0] += .36
    assert relay.consume('left', second['revision'])['hand'] is None


@pytest.mark.parametrize('mutation', [
    lambda f: setattr(f.hands[0].thumb.proximal.prev_joint, 'x', float('nan')),
    lambda f: setattr(f.hands[0].index.distal.next_joint, 'z', float('inf')),
    lambda f: setattr(f.hands[0].middle.intermediate.next_joint, 'y', 20000),
    lambda f: setattr(f.hands[0], 'type', 'unknown'),
    lambda f: setattr(f.hands[0], 'confidence', 1.1),
    lambda f: setattr(f, 'tracking_frame_id', True),
    lambda f: setattr(f, 'timestamp', -1),
    lambda f: setattr(f, 'timestamp', 9_000_000),
    lambda f: f.hands.append(deepcopy(f.hands[0])),
])
def test_malformed_frames_clear_previous_observation(reader, mutation):
    relay, sdk, _ = reader
    sdk.emit(frame())
    old = relay.latest('left')
    sdk.native_now += 1000
    bad = frame(2, timestamp=sdk.native_now)
    mutation(bad)
    sdk.emit(bad)
    result = relay.latest('left')
    assert result['hand'] is None
    assert result['source_metadata']['event'] == 'invalid_frame'
    assert result['source_metadata']['error']
    assert not relay.commit_if_current('left', old, lambda: pytest.fail('invalid commit'))


def test_disconnect_reconnect_epoch_and_old_callbacks(reader):
    relay, sdk, _ = reader
    sdk.emit(frame())
    old = relay.latest('left')
    listener = sdk.connections[0].listener
    listener.on_connection_lost_event(NS())
    assert relay.latest('left')['hand'] is None
    wait_until(lambda: len(sdk.connections) == 2 and sdk.connections[-1].ready)
    assert sdk.connections[0].closed
    sdk.emit(frame())  # Sequence may restart in a new LeapC connection.
    new = relay.latest('left')
    assert new['tracking']
    assert new['source_metadata']['stream_id'] != old['source_metadata']['stream_id']
    assert not relay.commit_if_current('left', old, lambda: pytest.fail('old stream'))
    listener.on_connection_lost_event(NS())
    listener.on_tracking_event(frame(99))
    assert relay.latest('left')['revision'] == new['revision']


def test_timeout_is_idle_but_sdk_failure_invalidates_and_reconnects(reader):
    relay, sdk, _ = reader
    sdk.emit(frame())
    first = relay.latest('left')
    sdk.connections[0].listener.on_error(sdk.exceptions.LeapTimeoutError())
    assert relay.latest('left')['revision'] == first['revision']
    sdk.connections[0].listener.on_error(RuntimeError('broken connection'))
    assert relay.latest('left')['hand'] is None
    wait_until(lambda: len(sdk.connections) == 2 and sdk.connections[-1].ready)


def test_missing_binding_is_reported_without_breaking_session():
    def missing():
        raise ImportError('No module named leap')
    relay = LeapMotionInputRelay(sdk_loader=missing, retry_delay=.01)
    relay.create('left', 'left')
    relay.latest('left')
    wait_until(lambda: relay.latest('left')['source_metadata']['event'] == 'sdk_unavailable')
    assert 'leapc-python-bindings' in relay.latest('left')['source_metadata']['error']
    relay.close('left')
    relay.client.close()


def test_connect_failure_still_closes_allocated_connection():
    sdk = FakeSdk()
    sdk.connect_error = RuntimeError('service unavailable')
    relay = LeapMotionInputRelay(sdk_loader=lambda: sdk, retry_delay=.02)
    relay.create('left', 'left')
    relay.latest('left')
    wait_until(lambda: sdk.connections and sdk.connections[0].closed)
    assert relay.latest('left')['hand'] is None
    relay.close('left')
    relay.client.close()


def test_close_during_connect_does_not_publish_or_overlap_new_reader():
    sdk = FakeSdk()
    sdk.connect_gate = threading.Event()
    relay = LeapMotionInputRelay(sdk_loader=lambda: sdk, retry_delay=.01)
    relay.create('left', 'left')
    relay.latest('left')
    wait_until(lambda: len(sdk.connections) == 1)
    stale_listener = sdk.connections[0].listener
    relay.close('left')
    relay.create('new', 'left')
    assert not relay.latest('new')['tracking']
    assert len(sdk.connections) == 1
    stale_listener.on_tracking_event(frame())
    assert relay.latest('new')['hand'] is None
    sdk.connect_gate.set()
    wait_until(lambda: sdk.connections[0].closed and not relay.client.worker['thread'].is_alive())
    assert sdk.connections[0].tracking_mode_calls == 0
    relay.latest('new')
    wait_until(lambda: len(sdk.connections) == 2 and sdk.connections[1].ready)
    sdk.emit(frame())
    assert relay.latest('new')['tracking']
    relay.close('new')
    relay.client.close()


def test_failed_cleanup_finishes_before_reconnect(reader):
    relay, sdk, _ = reader
    sdk.disconnect_errors = 1
    sdk.connections[0].listener.on_device_lost_event(NS())
    wait_until(lambda: len(sdk.connections) == 2 and sdk.connections[-1].ready)
    assert sdk.connections[0].closed and sdk.connections[0].disconnect_calls == 2


def test_subscribe_and_commit_guard(reader):
    relay, sdk, _ = reader
    callbacks = []
    unsubscribe = relay.subscribe('left', lambda: callbacks.append(True))
    sdk.emit(frame())
    result = relay.latest('left')
    actions = []
    assert callbacks
    assert relay.commit_if_current('left', result, lambda: actions.append('committed'))
    assert actions == ['committed']
    unsubscribe()
    count = len(callbacks)
    sdk.native_now += 1000
    sdk.emit(frame(2, timestamp=sdk.native_now))
    assert len(callbacks) == count
    relay.close('left')
    assert not relay.commit_if_current('left', result, lambda: pytest.fail('closed commit'))


class BackgroundSession:
    def __init__(self):
        self.lock = threading.RLock()
        self.control = NS(touch=lambda: None, armed=False)
        self.committed = []
        self.processed = []
        self.resets = 0
        self.gate = None
        self.entered = threading.Event()

    def reset_metric_source(self):
        self.resets += 1

    def process_background(self, packet, commit):
        self.processed.append(deepcopy(packet))
        if packet['hand'] is not None and self.gate:
            self.entered.set()
            assert self.gate.wait(2)
        accepted = commit(lambda: self.committed.append(deepcopy(packet)))
        return dict(tracking=accepted and packet['hand'] is not None,
                    background_discarded=not accepted)


def background_reader():
    from online_control.online_motion_track.source_worker import SourceWorker
    sdk = FakeSdk()
    relay = LeapMotionInputRelay(sdk_loader=lambda: sdk, retry_delay=.01)
    relay.create('left', 'left')
    session = BackgroundSession()
    worker = SourceWorker('left', session, relay, 'leapmotion')
    worker.touch_client()
    wait_until(lambda: sdk.connections and sdk.connections[-1].ready)
    return sdk, relay, session, worker


def test_background_callbacks_process_raw_source_and_expiry_without_ui_poll():
    sdk, relay, session, worker = background_reader()
    try:
        sdk.emit(frame())
        wait_until(lambda: any(p['hand'] is not None for p in session.committed))
        committed = [p for p in session.committed if p['hand'] is not None]
        assert len(committed) == 1
        assert committed[0]['source'] == 'leapmotion'
        assert committed[0]['raw_leapmotion']['sequence'] == 1
        assert committed[0]['source_metadata']['raw_leapmotion']['digits']['thumb']
        sdk.emit(frame())
        assert len([p for p in session.committed if p['hand'] is not None]) == 1
        count = len(session.committed)
        wait_until(lambda: len(session.committed) > count and session.committed[-1]['hand'] is None)
        assert session.committed[-1]['source_metadata']['age_ms'] >= 350
        assert not worker.read()['tracking']
        assert not session.control.armed
    finally:
        worker.close()
        relay.close('left')
        relay.client.close()


def test_background_hand_loss_recovery_discards_inflight_old_pose():
    sdk, relay, session, worker = background_reader()
    session.gate = threading.Event()
    try:
        sdk.emit(frame())
        assert session.entered.wait(1)
        sdk.native_now += 1000
        sdk.emit(frame(2, [], sdk.native_now))
        sdk.native_now += 1000
        sdk.emit(frame(3, [hand(identity=2)], sdk.native_now))
        session.gate.set()
        wait_until(lambda: any(p['hand'] is not None for p in session.committed))
        assert [p['raw_leapmotion']['sequence'] for p in session.committed if p['hand'] is not None] == [3]
        assert session.resets >= 2
    finally:
        session.gate.set()
        worker.close()
        relay.close('left')
        relay.client.close()


def test_background_cancel_during_solve_blocks_commit_and_stops_reader():
    sdk, relay, session, worker = background_reader()
    session.gate = threading.Event()
    try:
        sdk.emit(frame())
        assert session.entered.wait(1)
        worker.cancel()
        relay.close('left')
        session.gate.set()
        worker.close()
        relay.client.close()
        assert not any(p['hand'] is not None for p in session.committed)
        assert sdk.connections[0].closed
        assert not worker.thread.is_alive()
    finally:
        session.gate.set()
        worker.close()
        relay.close('left')
        relay.client.close()

def test_dead_sdk_poll_thread_reconnects_and_invalidates_previous_frame(reader):
    relay, sdk, _ = reader
    sdk.emit(frame())
    first = relay.latest('left')
    original = sdk.connections[0]
    original._poll_thread = NS(is_alive=lambda: False)
    wait_until(lambda: len(sdk.connections) >= 2 and sdk.connections[-1].ready)
    assert original.closed
    assert relay.latest('left')['hand'] is None
    assert not relay.commit_if_current('left', first, lambda: pytest.fail('dead reader commit'))
    sdk.native_now += 1000
    sdk.emit(frame(2, timestamp=sdk.native_now))
    assert relay.latest('left')['tracking']

def test_no_tracking_frames_reports_stall_without_reopening_healthy_empty_hand_stream(reader):
    relay, sdk, now = reader
    sdk.emit(frame(hands=[]))
    now[0] += 4
    assert relay.latest('left')['source_metadata']['event'] == 'tracking_lost'
    now[0] += 2
    wait_until(lambda: relay.latest('left')['source_metadata']['event'] == 'frame_timeout')
    stalled = relay.latest('left')
    assert stalled['source_metadata']['error']
    assert len(sdk.connections) == 1
    sdk.native_now += 1000
    sdk.emit(frame(2, timestamp=sdk.native_now))
    assert relay.latest('left')['tracking']
    assert relay.latest('left')['source_metadata']['error'] is None
