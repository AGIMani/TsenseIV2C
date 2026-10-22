"""IR buffer lifetime, real capture rate, cancellation and HTTP contracts."""
import json
from pathlib import Path
import struct
import sys
import threading
from types import SimpleNamespace as NS
import urllib.error
import urllib.request

import cffi
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src')]
from online_control.online_motion_track.leap_camera import LeapCameraSource
from online_control.online_motion_track.main import Application, make_server
from test_online_leapmotion_input import FakeSdk, frame, wait_until


class ImageSdk(FakeSdk):
    def __init__(self):
        super().__init__()
        self.ffi = cffi.FFI()
        self.policies = []
        self.libleapc.eLeapPolicyFlag_Images = 2
        self.libleapc.LeapSetPolicyFlagsEx = self.set_policy

    def set_policy(self, connection, device, enable, disable):
        assert connection.subscribed and connection.primary is device
        self.policies.append((enable, disable))
        return 0

    def set_hints(self, connection, device, hints):
        assert connection.primary_tracking_mode == 'hmd'
        assert hints[2] == self.ffi.NULL
        self.hint_calls.append([self.ffi.string(hints[i]) for i in range(2)])
        return 0

    def image(self, sequence=1, timestamp=1_000_000, **props):
        # Padding verifies use of LEAP_IMAGE.offset, not the start of data.
        buffer = self.ffi.new('unsigned char[]', b'PAD\x00\x7f\x80\xff')
        image = NS(properties=NS(**(dict(width=2, height=2, bpp=1) | props)), data=buffer, offset=3)
        return NS(image=[NS(c_data=image)], c_data=NS(info=NS(frame_id=sequence, timestamp=timestamp)))

    def emit_image(self, event):
        self.connections[-1].listener.on_image_event(event)


@pytest.fixture
def camera():
    now = [10.]
    sdk = ImageSdk()
    source = LeapCameraSource(clock=lambda: now[0], sdk_loader=lambda: sdk, retry_delay=.01)
    source.start()
    wait_until(lambda: sdk.hint_calls)
    yield source, sdk, now
    source.close()


def test_image_is_copied_before_next_leap_poll_and_never_uses_skeleton(camera):
    source, sdk, now = camera
    event = sdk.image()
    sdk.emit_image(event)
    event.image[0].c_data.data[3] = 222
    meta, pixels = source.read_image()
    assert pixels == b'\x00\x7f\x80\xff'
    assert (meta['width'], meta['height'], meta['format']) == (2, 2, 'gray8')
    assert meta['source'] == 'leapmotion_camera' and meta['has_frame']
    assert sdk.policies == [(2, 0)]
    assert sdk.hint_calls == [[b'fast_hand_motion', b'high_hand_fidelity']]
    sdk.emit(frame(sequence=1000))
    assert source.sequence == 1 and not source.frames
    assert source.read_image(meta['revision'], timeout=0)[1] == b''
    now[0] += .4
    sdk.emit_image(event)  # Duplicate cannot extend its original lifetime.
    assert not source.read_image()[0]['has_frame']


@pytest.mark.parametrize('fps', [60, 90, 120])
def test_fps_counts_actual_received_images_not_frame_id_gaps(camera, fps):
    source, sdk, now = camera
    for i in range(fps * 2 + 1):
        timestamp = 1_000_000 + round(i * 1_000_000 / fps)
        sdk.native_now = timestamp
        now[0] = 10 + i / fps
        sdk.emit_image(sdk.image(sequence=i*2, timestamp=timestamp))
    meta, _ = source.read_image()
    assert meta['capture_fps'] == pytest.approx(fps, abs=.01)
    assert meta['fps_status'] == ('ok' if fps == 120 else 'below_target')
    assert meta['target_fps'] == 120


@pytest.mark.parametrize('props', [dict(bpp=2), dict(width=0), dict(height=4096)])
def test_invalid_image_clears_previous_output(camera, props):
    source, sdk, _ = camera
    sdk.emit_image(sdk.image())
    sdk.emit_image(sdk.image(sequence=2, **props))
    meta, pixels = source.read_image()
    assert meta['event'] == 'image_error' and not meta['has_frame'] and not pixels


def test_native_delay_counts_towards_expiry(camera):
    source, sdk, _ = camera
    sdk.native_now += 400_000
    sdk.emit_image(sdk.image())
    meta, pixels = source.read_image()
    assert meta['age_ms'] == 400 and not pixels


def test_reconnect_rejects_old_callback_and_accepts_new_frame_clock(camera):
    source, sdk, _ = camera
    sdk.emit_image(sdk.image(sequence=99))
    previous_stream = source.stream_id
    old = sdk.connections[-1].listener
    old.on_device_lost_event(NS())
    assert not source.read_image()[1]
    wait_until(lambda: len(sdk.hint_calls) == 2)
    assert source.stream_id != previous_stream
    old.on_image_event(sdk.image(sequence=100))
    assert not source.read_image()[1]
    sdk.emit_image(sdk.image(sequence=0, timestamp=1))
    sdk.native_now = 1
    sdk.emit_image(sdk.image(sequence=1, timestamp=2))
    assert source.read_image()[0]['event'] == 'image_error'  # Future native clock rejected.
    sdk.native_now = 10
    sdk.emit_image(sdk.image(sequence=2, timestamp=10))
    assert source.read_image()[0]['has_frame']


def test_close_wakes_long_poll_without_reopening(camera):
    source, sdk, _ = camera
    sdk.emit_image(sdk.image())
    revision = source.revision
    outputs = []
    waiter = threading.Thread(target=lambda: outputs.append(source.read_image(revision, timeout=2, start=False)))
    waiter.start()
    source.close()
    waiter.join(timeout=1)
    assert not waiter.is_alive() and not outputs[0][1]
    assert not outputs[0][0]['connected']
    assert len(sdk.connections) == 1


class StubCamera:
    def __init__(self):
        self.running = False
        self.read_hook = None
    def start(self):
        self.running = True
    def read_image(self, after, *, start):
        assert start is False and self.running
        if self.read_hook:
            self.read_hook()
        return dict(format='gray8', width=2, height=2, has_frame=True, revision=1), b'\x00\x01\xfe\xff'
    def close(self, **kwargs):
        self.running = False


@pytest.fixture
def app(tmp_path):
    app = Application(tmp_path, public=True)
    app.leap_camera = StubCamera()
    yield app
    app.close()


def create(app, **kwargs):
    return app.create(dict(input_source='mediapipe', camera_source='leapmotion_camera', recording=False, **kwargs))['session']


def test_camera_is_mediapipe_session_and_last_owner_releases_reader(app):
    one, two = create(app), create(app)
    assert app.get(one).input_source == 'mediapipe'
    payload = app.leap_camera_frame(dict(session=one))
    size, = struct.unpack('<I', payload[:4])
    assert json.loads(payload[4:4+size])['has_frame']
    assert payload[4+size:] == b'\x00\x01\xfe\xff'
    app.remove(one)
    assert app.leap_camera.running
    app.remove(two)
    assert not app.leap_camera.running
    with pytest.raises(ValueError):
        app.leap_camera_frame(dict(session=two))


def test_stop_racing_poll_never_returns_frame(app):
    token = create(app)
    app.leap_camera.read_hook = lambda: app.remove(token)
    with pytest.raises(ValueError, match='已停止'):
        app.leap_camera_frame(dict(session=token))
    assert not app.leap_camera.running


def test_calibration_freeze_releases_camera_and_rejects_inflight_image(app):
    token = create(app, mode='calibration')
    app.calibration_action(dict(session=token, enabled=True, source='mediapipe', camera_source='leapmotion_camera'), 'input')
    app.leap_camera.read_hook = lambda: app.calibration_action(dict(session=token, enabled=False), 'input')
    with pytest.raises(ValueError, match='已停止'):
        app.leap_camera_frame(dict(session=token))
    assert not app.leap_camera.running


def test_non_camera_session_and_invalid_revision_do_not_open_device(app):
    token = app.create(dict(recording=False))['session']
    with pytest.raises(ValueError, match='相机 / MediaPipe'):
        app.leap_camera_frame(dict(session=token))
    token = create(app)
    for bad in (True, '1', -2, None):
        with pytest.raises(ValueError, match='revision'):
            app.leap_camera_frame(dict(session=token, after_revision=bad))
    assert not app.leap_camera.running
    with pytest.raises(ValueError, match='仅用于 MediaPipe'):
        app.create(dict(input_source='leapmotion', camera_source='leapmotion_camera'))


def test_http_binary_no_cache_same_origin_and_cancel(tmp_path):
    server = make_server(0, tmp_path, public=True)
    server.application.leap_camera = StubCamera()
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f'http://127.0.0.1:{server.server_port}'
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    token = create(server.application)
    def request(origin=base):
        return opener.open(urllib.request.Request(base+'/api/leap-camera/frame',
            data=json.dumps(dict(session=token)).encode(), headers={'Origin':origin, 'Content-Type':'application/json'}), timeout=5)
    try:
        with request() as response:
            assert response.headers['Content-Type'] == 'application/octet-stream'
            assert response.headers['Cache-Control'] == 'no-store'
            assert response.read().endswith(b'\x00\x01\xfe\xff')
        with pytest.raises(urllib.error.HTTPError) as error:
            request('https://unrelated.invalid')
        assert error.value.code == 403
        server.application.remove(token)
        with pytest.raises(urllib.error.HTTPError) as error:
            request()
        assert error.value.code == 400
    finally:
        server.shutdown()
        server.server_close()
        server.application.close()
        thread.join(timeout=5)
