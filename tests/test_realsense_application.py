"""RGB-D to real web retarget/calibration and HTTP, without camera or robot."""
import json
from pathlib import Path
import struct
import sys
import threading
import urllib.request

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src')]
from online_control.online_motion_track.main import Application, WebSession, make_server
from online_control.online_motion_track.realsense_source import encode_frame
from test_rgbd_fusion import CAMERA, hand_scene


class StoredCamera:
    def __init__(self):
        self.sessions = set()
        self.consumed = set()
        self.expired = False
        self.sequence = 0
        self.normalized, self.prior, self.depth, self.truth = hand_scene(prior_wrong_z=True)
        self.capture_unix_ms = 1791590400123.
        self.read_hook = None

    def create(self, token):
        self.sessions.add(token)

    def close(self, token):
        self.sessions.discard(token)

    def close_all(self):
        self.sessions.clear()

    def frame(self, token, after_revision=-1):
        assert token in self.sessions
        self.sequence += 1
        if self.read_hook:
            self.read_hook()
        return encode_frame(dict(has_frame=True, format='rgb8', width=640, height=480,
            frame_id=self.sequence, stream_id='capture-stream', revision=self.sequence,
            capture_unix_ms=self.capture_unix_ms, timestamp_ms=3000+self.sequence,
            age_ms=5, stale_after_ms=500, connected=True), bytes(640*480*3))

    def consume(self, token, frame_id, stream_id):
        if (self.expired or token not in self.sessions or stream_id != 'capture-stream'
                or frame_id != self.sequence or (token, frame_id) in self.consumed):
            raise ValueError('expired, duplicate or mismatched capture')
        self.consumed.add((token, frame_id))
        return dict(depth_m=self.depth, intrinsics=CAMERA, frame_id=frame_id,
                    stream_id=stream_id, timestamp_ms=3000+frame_id,
                    capture_unix_ms=self.capture_unix_ms)

    def commit_if_current(self, token, snapshot, callback):
        if self.expired or token not in self.sessions:
            raise ValueError('expired during solve')
        callback()


@pytest.fixture
def app(tmp_path):
    value = Application(tmp_path, public=True)
    value.realsense = StoredCamera()
    yield value
    value.close()


def create(app, **kwargs):
    return app.create(dict(input_source='realsense_rgbd', camera_source='realsense_rgbd',
                           recording=False, **kwargs))['session']


def packet(app, token, *, sequence=0, hand=True):
    app.realsense_frame({'session': token})
    source = app.realsense
    return dict(session=token, source='realsense_rgbd', sequence=sequence,
                timestamp_ms=1000+sequence*30, capture_unix_ms=1,
                realsense_frame_id=source.sequence, realsense_stream_id='capture-stream',
                hand=dict(label='Left', normalized=source.normalized.tolist(),
                          world=source.prior.tolist()) if hand else None)


def test_rgbd_fits_depth_then_uses_existing_21_point_solver(app):
    token = create(app)
    result = app.frame(packet(app, token))
    assert result['tracking'], result['message']
    assert result['source'] == 'realsense_rgbd'
    assert result['depth_source'] == 'realsense_rgbd_fused'
    assert len(result['points']) == len(result['fused_world']) == 21
    assert result['depth_fusion']['measured_count'] == 21
    assert result['source_metadata']['capture_unix_ms'] == app.realsense.capture_unix_ms
    assert result['source_metadata']['mediapipe_world_prior'] == app.realsense.prior.tolist()
    assert result['fused_world'][8][2] < -.010 < app.realsense.prior[8, 2]


@pytest.mark.parametrize('bad', ['holes', 'wrong_stream', 'duplicate', 'missing_id', 'expired', 'wrong_hand'])
def test_unavailable_depth_never_passes_monocular_prior_as_metric_input(app, bad):
    token = create(app)
    frame = packet(app, token)
    if bad == 'holes':
        app.realsense.depth.fill(0)
    elif bad == 'wrong_stream':
        frame['realsense_stream_id'] = 'previous-stream'
    elif bad == 'duplicate':
        assert app.frame(frame)['tracking']
        frame.update(sequence=1, timestamp_ms=1030)
    elif bad == 'missing_id':
        del frame['realsense_frame_id']
    elif bad == 'expired':
        app.realsense.expired = True
    elif bad == 'wrong_hand':
        frame['hand']['label'] = 'Right'
    result = app.frame(frame)
    assert not result['tracking']
    assert result['fused_world'] is None
    assert result['depth_source'] == 'none'
    assert result['depth_fusion']['status'] != 'fused'


def test_missing_camera_frame_is_a_hold_and_recovers(app):
    token = create(app)
    result = app.frame(dict(session=token, source='realsense_rgbd', sequence=0,
                           timestamp_ms=1000, hand=None))
    assert not result['tracking']
    assert app.frame(packet(app, token, sequence=1))['tracking']


def test_depth_fused_preview_is_discarded_when_capture_expires_during_retarget(app):
    token = create(app)
    session = app.get(token)
    process = session.pipeline.process
    def expire_after_solve(observation):
        result = process(observation)
        app.realsense.expired = True
        return result
    session.pipeline.process = expire_after_solve
    result = app.frame(packet(app, token))
    assert not result['tracking'] and result['display_discarded']
    assert result['fused_world'] is None


def test_calibration_can_switch_to_rgbd_freeze_and_release_camera(app):
    token = create(app, mode='calibration')
    active = app.calibration_action(dict(session=token, enabled=True, source='realsense_rgbd',
        camera_source='realsense_rgbd', side='left'), 'input')
    frame = packet(app, token)
    frame['input_epoch'] = active['input_epoch']
    result = app.frame(frame)
    assert result['tracking'] and len(result['input_points']) == 21
    app.calibration_action(dict(session=token, enabled=False, source='realsense_rgbd', side='left'), 'input')
    assert token not in app.realsense.sessions
    with pytest.raises(ValueError, match='冻结'):
        app.frame(frame)


def test_existing_rgb_webcam_path_never_opens_depth_source(app):
    token = app.create(dict(input_source='mediapipe', camera_source='browser', recording=False))['session']
    session = app.get(token)
    world = session.pipeline.session.retargeter.fk.compute().tolist()
    result = app.frame(dict(session=token, sequence=0, timestamp_ms=1000, capture_unix_ms=1000,
        hand=dict(label='Left', world=world, normalized=np.zeros((21, 3)).tolist())))
    assert result['tracking'] and result['depth_source'] == 'mediapipe_world'
    assert not app.realsense.sessions
    with pytest.raises(ValueError, match='深度融合'):
        app.realsense_frame({'session': token})


def test_stop_during_camera_read_cannot_return_a_retired_frame(app):
    token = create(app)
    app.realsense.read_hook = lambda: app.remove(token)
    with pytest.raises(ValueError, match='停止'):
        app.realsense_frame({'session': token})
    assert not app.realsense.sessions


def test_rgbd_recording_keeps_depth_diagnostics_and_image_landmarks(tmp_path):
    camera = StoredCamera()
    normalized, prior, depth, _ = hand_scene()
    from online_control.online_motion_track.rgbd_fusion import fuse_hand
    fused = fuse_hand(normalized, prior, depth, CAMERA)
    session = WebSession('left', True, tmp_path, input_source='realsense_rgbd')
    try:
        session.process(dict(source='realsense_rgbd', sequence=0, timestamp_ms=1000,
            capture_unix_ms=camera.capture_unix_ms, camera_size=[640,480],
            source_metadata={'depth_fusion': fused['diagnostics'], 'mediapipe_world_prior': prior.tolist()},
            hand=dict(world=fused['world'], normalized=fused['normalized'], label='Left')))
    finally:
        session.close()
    records = [json.loads(line) for line in session.recorder.path.read_text().splitlines()]
    assert records[0]['camera_source'] == 'realsense-rgbd'
    record = next(row for row in records if row['type'] == 'frame')
    assert record['depth_source'] == 'realsense_rgbd_fused'
    assert record['raw_normalized_points_21'] == normalized.tolist()
    assert record['source_metadata']['depth_fusion']['measured_count'] == 21


def test_real_http_rgb_binary_to_fused_json_contract(tmp_path):
    server = make_server(0, tmp_path, public=True)
    server.application.realsense = StoredCamera()
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f'http://127.0.0.1:{server.server_port}'
    def request(path, data):
        req = urllib.request.Request(base+path, json.dumps(data).encode(),
            headers={'Origin': base, 'Content-Type': 'application/json'})
        with urllib.request.urlopen(req, timeout=10) as response:
            return response.headers, response.read()
    try:
        _, body = request('/api/session', dict(input_source='realsense_rgbd', camera_source='realsense_rgbd', recording=False))
        token = json.loads(body)['session']
        headers, body = request('/api/realsense/frame', {'session': token})
        length = struct.unpack('<I', body[:4])[0]
        metadata = json.loads(body[4:4+length])
        assert headers['Content-Type'] == 'application/octet-stream'
        assert metadata['format'] == 'rgb8' and len(body)-4-length == 640*480*3
        source = server.application.realsense
        _, body = request('/api/frame', dict(session=token, source='realsense_rgbd', sequence=0,
            timestamp_ms=1000, capture_unix_ms=1, realsense_frame_id=metadata['frame_id'],
            realsense_stream_id=metadata['stream_id'], hand=dict(label='Left',
                world=source.prior.tolist(), normalized=source.normalized.tolist())))
        assert json.loads(body)['depth_fusion']['status'] == 'fused'
        request('/api/stop', {'session': token})
        assert not source.sessions
    finally:
        server.shutdown()
        server.server_close()
        server.application.close()
        thread.join(timeout=2)
