"""Manus source routes exercise the real solver without SDK or robot hardware."""
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
from test_manus_input import device, wait_for


@pytest.fixture
def app(device, tmp_path):
    clock, glove, relay = device
    application = Application(tmp_path, public=True)
    application.manus = relay
    application.test_clock, application.test_glove = clock, glove
    yield application
    application.close()


def create(app, **kwargs):
    session = app.create(dict(input_source='manus_glove', recording=False, **kwargs))['session']
    app.manus_create(dict(session=session))
    points = app.get(session).pipeline.session.retargeter.fk.compute()
    app.test_glove.publish(points, side=kwargs.get('side', 'left'))
    wait_for(lambda: app.manus_latest(dict(session=session)))
    return session


@pytest.mark.parametrize('side', ['left', 'right'])
def test_combined_frame_solves_once_with_server_time_and_selected_hand(app, side):
    session = create(app, side=side)
    app.manus_latest(dict(session=session))
    request = dict(session=session, sequence=-900, timestamp_ms=-1,
                   hand={'world': 'forged'}, raw_manus={'forged': True})
    snapshot = app.manus_frame(request)
    assert snapshot['update']['tracking']
    assert snapshot['update']['source'] == 'manus_glove'
    assert snapshot['update']['depth_source'] == 'manus_glove_metric'
    assert snapshot['hand']['label'] == side.title()
    assert snapshot['update']['sequence'] >= 0
    assert snapshot['update']['input_path'] == 'server_manus_frame'
    assert app.manus_frame(request)['update'] is None
    app.test_clock.value += .21
    stale = app.manus_frame(request)
    assert stale['hand'] is None
    assert not stale['update']['tracking']


def test_generic_frame_never_uses_browser_supplied_skeleton(app):
    session = create(app)
    snapshot = app.manus_latest(dict(session=session))
    request = dict(session=session, source='manus_glove', manus_revision=snapshot['revision'],
                   sequence=0, timestamp_ms=10, capture_unix_ms=1000,
                   hand={'world': 'forged'}, source_metadata={'age_ms': -100},
                   raw_manus={'forged': True})
    result = app.frame(request)
    assert result['tracking']
    assert result['source_metadata']['raw_manus']['source_timestamp_raw'] == '123456'
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
            app.manus.close(token)
        else:
            # New source frames may be fresh; this solve's original input isn't.
            clock[0] += .36
        return result
    session.pipeline.process = delayed
    snapshot = app.manus_frame(dict(session=token))
    result = snapshot['update']
    assert snapshot['hand'] is None and not snapshot['tracking']
    assert not result['tracking']
    assert result['display_discarded']
    assert result['discard_reason'] == 'source_cancelled_or_expired'


def test_calibration_uses_native_revision_and_freeze_closes_source(app):
    session = app.create(dict(mode='calibration', input_source='manus_glove', side='left'))['session']
    enabled = app.calibration_action(dict(session=session, source='manus_glove', enabled=True), 'input')
    app.manus_create(dict(session=session))
    app.test_glove.publish(app.get(session).input_points, side='left')
    snapshot = wait_for(lambda: app.manus_latest(dict(session=session)))
    result = app.frame(dict(session=session, source='manus_glove', input_epoch=enabled['input_epoch'],
        manus_revision=snapshot['revision'], sequence=0, timestamp_ms=10, capture_unix_ms=1000))
    assert result['tracking']
    app.calibration_action(dict(session=session, enabled=False), 'input')
    assert session not in app.manus.channels


def test_recording_preserves_raw_manus_data_and_metric_source(tmp_path):
    session = WebSession(recording=True, log_dir=tmp_path, input_source='manus_glove')
    path = session.recorder.path
    points = session.pipeline.session.retargeter.fk.compute()
    raw = dict(timestamp_raw='1152921504606846977', glove_id=17, nodes=[1, 2, 3])
    try:
        session.process(dict(source='manus_glove', sequence=0, timestamp_ms=10, capture_unix_ms=1000,
            hand=dict(world=points.tolist(), label='Left'), raw_manus=raw,
            source_metadata=dict(source='manus_glove', raw_manus=raw)))
    finally:
        session.close()
    frame = next(row for line in Path(path).read_text().splitlines()
                 if (row := json.loads(line)).get('type') == 'frame')
    assert frame['raw_manus'] == raw
    assert frame['depth_source'] == 'manus_glove_metric'
    assert frame['raw_normalized_points_21'] is None


def test_http_discovery_routes_same_origin_and_stop(tmp_path, device):
    server = make_server(0, tmp_path, public=True)
    clock, glove, relay = device
    server.application.manus = relay
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
        assert 'manus_glove' in request('/api/health')['input_sources']
        session = request('/api/session', dict(input_source='manus_glove', recording=False))['session']
        created = request('/api/manus/create', dict(session=session))
        assert created['combined_frame'] and not created['background_processing']
        with pytest.raises(urllib.error.HTTPError) as error:
            request('/api/manus/latest', dict(session=session), 'https://unrelated.invalid')
        assert error.value.code == 403
        glove.publish(server.application.get(session).pipeline.session.retargeter.fk.compute())
        assert wait_for(lambda: request('/api/manus/latest', dict(session=session)))['tracking']
        assert request('/api/manus/frame', dict(session=session))['update']['tracking']
        request('/api/stop', dict(session=session))
        assert session not in server.application.manus.channels
        with pytest.raises(urllib.error.HTTPError) as error:
            request('/api/manus/latest', dict(session=session))
        assert error.value.code == 400
    finally:
        server.shutdown()
        server.server_close()
        server.application.close()
        thread.join(timeout=5)


def test_customer_source_alias_and_constructor_options(device, tmp_path):
    _, glove, relay = device
    app = Application(tmp_path, public=True, manus_options=dict(mode='local', connect_timeout=3))
    assert app.manus.client.mode == 'local'
    assert app.manus.client.connect_timeout == 3
    app.manus = relay
    try:
        token = app.create(dict(input_source='manus', recording=False))['session']
        assert app.get(token).input_source == 'manus_glove'
        app.manus_create(dict(session=token))
        glove.publish(app.get(token).pipeline.session.retargeter.fk.compute())
        snapshot = wait_for(lambda: app.manus_latest(dict(session=token)))
        result = app.frame(dict(session=token, source='manus', manus_revision=snapshot['revision'],
            sequence=0, timestamp_ms=10, capture_unix_ms=1000))
        assert result['tracking'] and result['source'] == 'manus_glove'
    finally:
        app.close()


def test_cli_exposes_customer_manus_options_without_loading_sdk():
    import subprocess
    result = subprocess.run([sys.executable, '-m', 'online_control.online_motion_track.main', '--help'],
                            cwd=ROOT, text=True, capture_output=True, timeout=10)
    assert result.returncode == 0, result.stderr
    for flag in ('--manus-mode', '--manus-module-dir', '--manus-left-calibration',
                 '--manus-right-calibration', '--manus-connect-timeout', '--wuji-source'):
        assert flag in result.stdout
