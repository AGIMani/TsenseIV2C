"""Metric input scheduling/lease/epoch contracts using real Quest relay, no hardware."""
from copy import deepcopy
from pathlib import Path
import sys
import threading
import time

sys.path[:0] = [str(Path(__file__).resolve().parents[1]),
               str(Path(__file__).resolve().parents[1]/'src/tracking')]
from online_control.online_motion_track.source_worker import SourceWorker
from online_control.online_motion_track.webxr_input import WebXRInputRelay
from test_online_webxr_input import event, skeleton
from test_wuji_sdk_direct import wait_until
from test_wuji_sdk_direct import FakeManager, frame
from online_control.online_motion_track.wuji_input import WujiInputRelay


class Control:
    def __init__(self):
        self.client_at = 0.
        self.armed = False

    def touch(self):
        self.client_at = time.monotonic()


class Session:
    def __init__(self):
        self.lock = threading.RLock()
        self.control = Control()
        self.last_seen = 0.
        self.processed, self.committed = [], []
        self.entered = threading.Event()
        self.block = None
        self.resets = 0

    def reset_metric_source(self):
        self.resets += 1

    def process_background(self, packet, commit):
        self.processed.append(deepcopy(packet))
        if self.block is not None and packet.get('hand') is not None:
            self.entered.set()
            assert self.block.wait(2)
        accepted = commit(lambda: self.committed.append(deepcopy(packet)))
        return dict(tracking=accepted and packet.get('hand') is not None,
                    source=packet['source'], sequence=packet['sequence'],
                    background_discarded=not accepted,
                    retarget_joint_angles_rad={'test': 0.} if accepted else None)


def setup(session_id='pc', relay=None, session=None):
    relay = relay or WebXRInputRelay()
    session = session or Session()
    token = relay.create(session_id, 'left')['token']
    relay.publish(token, event())
    worker = SourceWorker(session_id, session, relay, 'webxr')
    worker.touch_client()
    return relay, token, session, worker


def valid_commits(session):
    return [p for p in session.committed if p['hand'] is not None]


def test_read_marks_only_expired_tracking_pose_for_display_discard():
    relay = WebXRInputRelay()
    worker = SourceWorker('pc', Session(), relay, 'webxr', clock=lambda: 1.)
    worker.result_at = 0.
    worker.result = dict(age_ms=0., hand={'points': []}, tracking=True,
                        update=dict(tracking=True, retarget_joint_angles_rad={'joint': .2}))
    stale = worker.read()
    assert stale['update']['display_discarded'] is True
    assert stale['update']['tracking'] is False
    assert stale['update']['retarget_joint_angles_rad'] == {'joint': .2}
    assert worker.result['update']['tracking'] is True
    worker.result['update']['tracking'] = False
    hold = worker.read()
    assert 'display_discarded' not in hold['update']


def test_publish_drives_worker_without_polling_and_result_reads_do_not_consume_again():
    relay, token, session, worker = setup()
    try:
        seen, client_at = session.last_seen, session.control.client_at
        relay.publish(token, skeleton(1))
        wait_until(lambda: worker.read()['tracking'])
        result = worker.read()
        assert len(valid_commits(session)) == 1
        assert session.last_seen == seen and session.control.client_at == client_at
        assert result['update_revision'] > 0
        assert result['raw_webxr']['sequence'] == 1
        assert worker.read(result['update_revision'])['update'] is None
        relay.publish(token, skeleton(1))
        assert len(valid_commits(session)) == 1
        relay.publish(token, skeleton(2))
        wait_until(lambda: len(valid_commits(session)) == 2)
        assert worker.read()['update_revision'] > result['update_revision']
        assert not session.control.armed
    finally:
        worker.close()
        relay.close('pc')


def test_350ms_loss_is_committed_without_any_new_publish_or_browser_poll():
    relay, token, session, worker = setup()
    try:
        relay.publish(token, skeleton(1))
        wait_until(lambda: worker.read()['tracking'])
        initial = worker.read()['update_revision']
        holds = len([p for p in session.committed if p['hand'] is None])
        wait_until(lambda: len([p for p in session.committed if p['hand'] is None]) > holds)
        result = worker.read()
        assert not result['tracking'] and result['hand'] is None
        assert result['age_ms'] >= 350
        assert result['update_revision'] > initial
    finally:
        worker.close()


def test_cancel_during_solve_discards_result_before_target_commit():
    session = Session(); session.block = threading.Event()
    relay, token, session, worker = setup(session=session)
    try:
        relay.publish(token, skeleton(1))
        assert session.entered.wait(1)
        worker.cancel()
        session.block.set()
        worker.close()
        assert not valid_commits(session)
        assert not worker.thread.is_alive()
    finally:
        session.block.set(); worker.close()


def test_stream_replacement_during_solve_cannot_commit_old_result():
    session = Session(); session.block = threading.Event()
    relay, token, session, worker = setup(session=session)
    try:
        relay.publish(token, skeleton(1))
        assert session.entered.wait(1)
        relay.publish(token, event('session_started', 0, 'quest-new'))
        relay.publish(token, skeleton(1, stream='quest-new'))
        session.block.set()
        wait_until(lambda: valid_commits(session))
        assert all(p['raw_webxr']['stream_id'] == 'quest-new' for p in valid_commits(session))
    finally:
        session.block.set(); worker.close()


def test_same_quest_stream_loss_and_recovery_invalidates_inflight_result():
    session = Session(); session.block = threading.Event()
    relay, token, session, worker = setup(session=session)
    try:
        relay.publish(token, skeleton(1))
        assert session.entered.wait(1)
        resets = session.resets
        relay.publish(token, event('tracking_lost', 2, handedness='left'))
        relay.publish(token, skeleton(3))
        session.block.set()
        wait_until(lambda: valid_commits(session))
        assert all(p['raw_webxr']['sequence'] == 3 for p in valid_commits(session))
        assert session.resets > resets
    finally:
        session.block.set(); worker.close()


def test_same_sdk_stream_invalid_frame_and_recovery_invalidates_inflight_result():
    session = Session(); session.block = threading.Event()
    manager, relay = FakeManager(), WujiInputRelay(source='sdk')
    relay.client.sdk_loader = manager.sdk
    relay.create('pc', 'left')
    worker = SourceWorker('pc', session, relay, 'wuji_glove')
    worker.touch_client()
    try:
        wait_until(lambda: manager.gloves and manager.gloves[0].callback)
        callback = manager.gloves[0].callback
        callback(frame(1))
        assert session.entered.wait(1)
        resets = session.resets
        invalid = frame(2)
        invalid.joints[0].pose.position[0] = float('nan')
        callback(invalid)
        callback(frame(3))
        session.block.set()
        wait_until(lambda: valid_commits(session))
        assert all(p['raw_wuji']['sequence'] == 3 for p in valid_commits(session))
        assert session.resets > resets
    finally:
        session.block.set(); worker.close(); relay.close('pc')


def test_new_fresh_frame_does_not_starve_valid_inflight_result():
    session = Session(); session.block = threading.Event()
    relay, token, session, worker = setup(session=session)
    try:
        relay.publish(token, skeleton(1))
        assert session.entered.wait(1)
        resets = session.resets
        relay.publish(token, skeleton(2))
        session.block.set()
        wait_until(lambda: len(valid_commits(session)) >= 2)
        assert [p['raw_webxr']['sequence'] for p in valid_commits(session)] == [1, 2]
        assert session.resets == resets
    finally:
        session.block.set(); worker.close()


def test_client_lease_expiry_during_solve_blocks_commit_and_source_does_not_renew_it():
    session = Session(); session.block = threading.Event()
    relay, token, session, worker = setup(session=session)
    worker.CLIENT_TIMEOUT = .05
    try:
        seen, control_at = session.last_seen, session.control.client_at
        relay.publish(token, skeleton(1))
        assert session.entered.wait(1)
        time.sleep(.07)
        session.block.set()
        wait_until(lambda: worker.read()['update'] and worker.read()['update'].get('background_discarded'))
        relay.publish(token, skeleton(2))
        assert not valid_commits(session)
        assert session.last_seen == seen and session.control.client_at == control_at
        worker.touch_client()
        wait_until(lambda: valid_commits(session))
        assert not session.control.armed
    finally:
        session.block.set(); worker.close()


def test_observation_expiry_during_solve_blocks_old_target():
    session = Session(); session.block = threading.Event()
    relay, token, session, worker = setup(session=session)
    try:
        relay.publish(token, skeleton(1))
        assert session.entered.wait(1)
        time.sleep(.37)
        session.block.set()
        wait_until(lambda: len(session.processed) >= 2)
        assert not valid_commits(session)
    finally:
        session.block.set(); worker.close()


def test_two_sessions_have_independent_workers_and_closing_one_does_not_stop_other():
    relay, token_a, session_a, worker_a = setup('a')
    _, token_b, session_b, worker_b = setup('b', relay)
    try:
        relay.publish(token_a, skeleton(1))
        wait_until(lambda: valid_commits(session_a))
        assert not valid_commits(session_b)
        worker_a.close(); relay.close('a')
        relay.publish(token_b, skeleton(1))
        wait_until(lambda: valid_commits(session_b))
        assert not worker_a.thread.is_alive() and worker_b.thread.is_alive()
    finally:
        worker_a.close(); worker_b.close()


def test_replacing_worker_keeps_server_sequence_monotonic_and_cancels_old_epoch():
    session = Session(); session.block = threading.Event()
    relay, token, session, old = setup(session=session)
    new = None
    try:
        relay.publish(token, skeleton(1))
        assert session.entered.wait(1)
        old.cancel()
        _, new_token, _, new = setup(relay=relay, session=session)
        relay.publish(new_token, skeleton(1, stream='quest-a'))
        session.block.set()
        wait_until(lambda: valid_commits(session))
        assert len(valid_commits(session)) == 1
        ids = [p['sequence'] for p in session.processed]
        assert ids == sorted(set(ids))
    finally:
        session.block.set(); old.close()
        if new: new.close()
