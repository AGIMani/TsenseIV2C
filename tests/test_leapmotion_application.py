"""Leap source routes exercise the real solver without SDK or robot hardware."""
import json
from pathlib import Path
import sys
import threading
import urllib.error
import urllib.request

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src')]
from online_control.online_motion_track.main import Application, WebSession, make_server
from run_leapmotion_browser_fixture import SyntheticLeapRelay


@pytest.fixture
def app(tmp_path):
    application = Application(tmp_path, public=True)
    application.leapmotion = SyntheticLeapRelay()
    yield application
    application.close()


def create(app, **kwargs):
    session = app.create(dict(input_source='leapmotion', recording=False, **kwargs))['session']
    app.leapmotion_create(dict(session=session))
    return session


@pytest.mark.parametrize('side', ['left', 'right'])
def test_combined_frame_solves_once_with_server_time_and_selected_hand(app, side):
    session = create(app, side=side)
    app.leapmotion_latest(dict(session=session))
    app.leapmotion.paused = True
    request = dict(session=session, sequence=-900, timestamp_ms=-1,
                   hand={'world': 'forged'}, raw_leapmotion={'forged': True})
    snapshot = app.leapmotion_frame(request)
    assert snapshot['update']['tracking']
    assert snapshot['update']['source'] == 'leapmotion'
    assert snapshot['update']['depth_source'] == 'leapmotion_metric'
    assert snapshot['hand']['label'] == side.title()
    assert snapshot['update']['sequence'] >= 0
    assert snapshot['update']['input_path'] == 'server_leapmotion_frame'
    assert app.leapmotion_frame(request)['update'] is None
    app.leapmotion.channels[session]['at'] -= 1
    stale = app.leapmotion_frame(request)
    assert stale['hand'] is None
    assert not stale['update']['tracking']


def test_generic_frame_never_uses_browser_supplied_skeleton(app):
    session = create(app)
    snapshot = app.leapmotion_latest(dict(session=session))
    app.leapmotion.paused = True
    request = dict(session=session, source='leapmotion', leapmotion_revision=snapshot['revision'],
                   sequence=0, timestamp_ms=10, capture_unix_ms=1000,
                   hand={'world': 'forged'}, source_metadata={'age_ms': -100},
                   raw_leapmotion={'forged': True})
    result = app.frame(request)
    assert result['tracking']
    assert result['source_metadata']['raw_leapmotion']['simulation_only']
    assert app.frame(request | dict(sequence=1, timestamp_ms=20))['tracking'] is False
    with pytest.raises(ValueError, match='不一致'):
        app.frame(request | dict(source='mediapipe'))


@pytest.mark.parametrize('cause', ['closed', 'expired'])
def test_preview_rechecks_source_and_original_age_after_solve(app, monkeypatch, cause):
    from online_control.online_motion_track import main
    token = create(app)
    session = app.get(token)
    clock = [10.]
    monkeypatch.setattr(main.time, 'perf_counter', lambda: clock[0])
    original = session.pipeline.process
    def delayed(observation):
        result = original(observation)
        assert result.tracking
        if cause == 'closed':
            app.leapmotion.close(token)
        else:
            # New source frames may be fresh; this solve's original input isn't.
            clock[0] += .36
        return result
    session.pipeline.process = delayed
    snapshot = app.leapmotion_frame(dict(session=token))
    result = snapshot['update']
    assert snapshot['hand'] is None and not snapshot['tracking']
    assert not result['tracking']
    assert result['display_discarded']
    assert result['discard_reason'] == 'source_cancelled_or_expired'


def test_calibration_uses_native_revision_and_freeze_closes_source(app):
    session = app.create(dict(mode='calibration', input_source='leapmotion', side='left'))['session']
    enabled = app.calibration_action(dict(session=session, source='leapmotion', enabled=True), 'input')
    app.leapmotion_create(dict(session=session))
    snapshot = app.leapmotion_latest(dict(session=session))
    result = app.frame(dict(session=session, source='leapmotion', input_epoch=enabled['input_epoch'],
        leapmotion_revision=snapshot['revision'], sequence=0, timestamp_ms=10, capture_unix_ms=1000))
    assert result['tracking']
    app.calibration_action(dict(session=session, enabled=False), 'input')
    assert session not in app.leapmotion.channels


def test_recording_preserves_raw_leap_data_and_metric_source(tmp_path):
    session = WebSession(recording=True, log_dir=tmp_path, input_source='leapmotion')
    path = session.recorder.path
    points = session.pipeline.session.retargeter.fk.compute()
    raw = dict(timestamp_us=12345, hand_id=17, bones_mm=[1, 2, 3])
    try:
        session.process(dict(source='leapmotion', sequence=0, timestamp_ms=10, capture_unix_ms=1000,
            hand=dict(world=points.tolist(), label='Left'), raw_leapmotion=raw,
            source_metadata=dict(source='leapmotion', raw_leapmotion=raw)))
    finally:
        session.close()
    frame = next(row for line in Path(path).read_text().splitlines()
                 if (row := json.loads(line)).get('type') == 'frame')
    assert frame['raw_leapmotion'] == raw
    assert frame['depth_source'] == 'leapmotion_metric'
    assert frame['raw_normalized_points_21'] is None


def test_http_discovery_routes_same_origin_and_stop(tmp_path):
    server = make_server(0, tmp_path, public=True)
    server.application.leapmotion = SyntheticLeapRelay()
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f'http://127.0.0.1:{server.server_port}'
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def request(path, data=None, origin=base):
        req = urllib.request.Request(base + path, data=None if data is None else json.dumps(data).encode(),
            headers={'Origin': origin, 'Content-Type': 'application/json'})
        with opener.open(req, timeout=10) as response:
            return json.load(response)

    try:
        assert 'leapmotion' in request('/api/health')['input_sources']
        session = request('/api/session', dict(input_source='leapmotion', recording=False))['session']
        created = request('/api/leapmotion/create', dict(session=session))
        assert created['combined_frame'] and not created['background_processing']
        with pytest.raises(urllib.error.HTTPError) as error:
            request('/api/leapmotion/latest', dict(session=session), 'https://unrelated.invalid')
        assert error.value.code == 403
        assert request('/api/leapmotion/latest', dict(session=session))['tracking']
        assert request('/api/leapmotion/frame', dict(session=session))['update']['tracking']
        request('/api/stop', dict(session=session))
        assert session not in server.application.leapmotion.channels
        with pytest.raises(urllib.error.HTTPError) as error:
            request('/api/leapmotion/latest', dict(session=session))
        assert error.value.code == 400
    finally:
        server.shutdown()
        server.server_close()
        server.application.close()
        thread.join(timeout=5)
