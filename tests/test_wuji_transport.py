"""Slow remote HTTP must not block local snapshots or extend stale frame age."""
import sys
from pathlib import Path
import threading
import time

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from online_control.online_motion_track.wuji_transport import LatestWujiClient


def wait_until(predicate):
    deadline = time.monotonic() + 2
    while not predicate():
        assert time.monotonic() < deadline
        time.sleep(.005)


def test_cache_is_nonblocking_ages_and_expires_through_network_failure(monkeypatch):
    now = [10.]
    client = LatestWujiClient('http://127.0.0.1:8787', clock=lambda: now[0], interval=.01)
    gate = threading.Event()
    called = threading.Event()
    calls = []
    def request(connection, side):
        calls.append(connection)
        if len(calls) == 1:
            return dict(schema='tsensei.wuji_glove.skeleton.v1', connected=True,
                        frame={'sequence':1}, age_ms=10., state='receiving')
        called.set()
        gate.wait(1)
        raise TimeoutError('test outage')
    monkeypatch.setattr(client, '_request', request)
    try:
        client.latest('left')
        wait_until(lambda: client.latest('left')['frame'] is not None)
        assert called.wait(1)
        now[0] += .1
        t = time.monotonic()
        snapshot = client.latest('left')
        assert time.monotonic() - t < .05
        assert snapshot['age_ms'] == pytest.approx(110)
        gate.set()
        wait_until(lambda: client.latest('left')['transport']['errors'] > 0)
        assert client.latest('left')['connected']
        now[0] += .3
        stale = client.latest('left')
        assert stale['age_ms'] == pytest.approx(410)
        assert not stale['connected'] and stale['state'] == 'transport_error'
        assert len({id(c) for c in calls}) == 1
    finally:
        gate.set()
        client.close()
    assert not client.streams


def test_age_includes_full_remote_request_and_local_cache_wait(monkeypatch):
    now = [10.]
    client = LatestWujiClient('http://127.0.0.1:8787', clock=lambda: now[0], interval=10)
    def request(*_):
        now[0] += .05
        return dict(connected=True, frame={'sequence':1}, age_ms=7)
    monkeypatch.setattr(client, '_request', request)
    try:
        client.latest('left')
        wait_until(lambda: client.latest('left')['frame'] is not None)
        now[0] += .02
        snapshot = client.latest('left')
        assert snapshot['age_ms'] == pytest.approx(77)
        assert snapshot['transport']['bridge_age_ms'] == 7
        assert snapshot['transport']['request_ms'] == pytest.approx(50)
    finally:
        client.close()
