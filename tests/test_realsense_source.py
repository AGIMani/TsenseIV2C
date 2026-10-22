"""Native RGB-D capture pairing and lifecycle without a camera or SDK import."""
import json
from pathlib import Path
import queue
import struct
import sys
import threading
import time
from types import SimpleNamespace as NS

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src')]
from online_control.online_motion_track.realsense_source import RealSenseSource


def wait_until(predicate, timeout=2.):
    end = time.monotonic() + timeout
    while not predicate():
        if time.monotonic() >= end:
            raise AssertionError('capture thread did not reach expected state')
        time.sleep(.005)


def decode(payload):
    size, = struct.unpack('<I', payload[:4])
    return json.loads(payload[4:size + 4]), payload[size + 4:]


class Profile:
    def __init__(self, stream, format, fps=30, width=2, height=2):
        self.kind, self.encoding, self.rate = stream, format, fps
        self.intrinsics = NS(width=width, height=height, fx=100., fy=101., ppx=.5, ppy=.6,
                             model='modified_brown_conrady', coeffs=[.1, 0., 0., 0., 0.])
    def stream_type(self): return self.kind
    def format(self): return self.encoding
    def fps(self): return self.rate
    def width(self): return self.intrinsics.width
    def height(self): return self.intrinsics.height
    def as_video_stream_profile(self): return self
    def get_intrinsics(self): return self.intrinsics


class FakeSDK:
    stream = NS(depth='depth', color='color')
    format = NS(z16='z16', rgb8='rgb8')
    camera_info = NS(serial_number='serial', name='name')
    timestamp_domain = NS(hardware_clock='hardware', system_time='system', global_time='global')

    def __init__(self):
        self.depth_profile = Profile('depth', 'z16')
        self.color_profile = Profile('color', 'rgb8')
        self.profiles = [self.depth_profile, self.color_profile, Profile('depth', 'z16', 90)]
        self.device = NS(get_info=lambda key: {'serial': 'test-serial', 'name': 'D455 test'}[key],
                         query_sensors=lambda: [NS(get_stream_profiles=lambda: self.profiles)],
                         first_depth_sensor=lambda: NS(get_depth_scale=lambda: .001))
        self.devices = [self.device]
        self.pipelines, self.configs, self.alignments, self.deprojections = [], [], [], []
        self.fail_starts = 0

    def context(self): return NS(query_devices=lambda: self.devices)

    def config(self):
        class Config:
            def __init__(self): self.streams = []
            def enable_device(self, serial): self.serial = serial
            def enable_stream(self, *args): self.streams.append(args)
        config = Config()
        self.configs.append(config)
        return config

    def pipeline(self):
        sdk = self
        class Pipeline:
            def __init__(self):
                self.queue = queue.Queue()
                self.started = self.stopped = False
            def start(self, config):
                if sdk.fail_starts:
                    sdk.fail_starts -= 1
                    raise RuntimeError('unsupported combination')
                self.started = True
                return NS(get_device=lambda: sdk.device,
                          get_stream=lambda stream: sdk.depth_profile if stream == 'depth' else sdk.color_profile)
            def wait_for_frames(self, timeout):
                assert timeout <= 250
                try:
                    result = self.queue.get(timeout=timeout/1000)
                except queue.Empty:
                    raise RuntimeError("Frame didn't arrive within 250")
                if isinstance(result, Exception): raise result
                return result
            def stop(self): self.stopped = True
        pipeline = Pipeline()
        self.pipelines.append(pipeline)
        return pipeline

    def align(self, stream):
        assert stream == self.stream.color
        def process(frames):
            self.alignments.append(frames)
            return frames
        return NS(process=process)

    def rs2_deproject_pixel_to_point(self, intrinsics, uv, z):
        self.deprojections.append((intrinsics, uv, z))
        return [(uv[0] - intrinsics.ppx) / intrinsics.fx * z,
                (uv[1] - intrinsics.ppy) / intrinsics.fy * z, z]

    def emit(self, frame_id=1, depth_value=600, timestamp=1000., domain='hardware', pipeline=None,
             depth_timestamp=None, depth_frame_id=None):
        rgb = np.full((2, 2, 3), frame_id, dtype=np.uint8)
        depth = np.full((2, 2), depth_value, dtype=np.uint16)
        color_frame = NS(get_frame_number=lambda: frame_id, get_timestamp=lambda: timestamp,
                         get_frame_timestamp_domain=lambda: domain, get_data=lambda: rgb,
                         profile=self.color_profile)
        depth_frame = NS(get_data=lambda: depth, profile=self.depth_profile,
                         get_frame_number=lambda: frame_id if depth_frame_id is None else depth_frame_id,
                         get_timestamp=lambda: timestamp if depth_timestamp is None else depth_timestamp,
                         get_frame_timestamp_domain=lambda: domain)
        frames = NS(get_depth_frame=lambda: depth_frame, get_color_frame=lambda: color_frame)
        (pipeline or self.pipelines[-1]).queue.put(frames)
        return depth, rgb


@pytest.fixture
def capture():
    sdk, now = FakeSDK(), [10.]
    source = RealSenseSource(sdk_loader=lambda: sdk, clock=lambda: now[0],
                            unix_clock=lambda: 1_700_000_000. + now[0], retry_delay=.01)
    source.create('one')
    wait_until(lambda: source.event == 'waiting')
    yield source, sdk, now
    source.close_all()


def delivered(source, sdk, frame_id=1, **kwargs):
    sdk.emit(frame_id, **kwargs)
    wait_until(lambda: frame_id in source.frames)
    meta, pixels = decode(source.frame('one'))
    assert meta['has_frame'] and meta['frame_id'] == frame_id
    return meta, pixels


def test_lazy_optional_sdk_does_not_open_ordinary_camera():
    calls = []
    def missing():
        calls.append(True)
        raise ImportError('optional module missing')
    source = RealSenseSource(sdk_loader=missing)
    assert not calls
    with pytest.raises(RuntimeError, match='track.*pyrealsense2'):
        source.create('one')
    assert calls == [True] and not source.sessions and source.worker is None


def test_copies_paired_frame_and_preserves_sdk_intrinsics(capture):
    source, sdk, _ = capture
    depth, rgb = sdk.emit()
    wait_until(lambda: source.frames)
    depth[:] = 999
    rgb[:] = 99
    meta, pixels = decode(source.frame('one'))
    assert pixels == bytes([1]) * 12
    assert meta['format'] == 'rgb8' and meta['source'] == 'realsense_rgbd'
    assert meta['device_name'] == 'D455 test'
    assert meta['depth_mode']['fps'] == meta['color_mode']['fps'] == 30
    snapshot = source.consume('one', meta['frame_id'], meta['stream_id'])
    assert np.allclose(snapshot['depth_m'], .6)
    with pytest.raises(ValueError): snapshot['depth_m'][0, 0] = 3
    with pytest.raises(ValueError): snapshot['depth_m'].flags.writeable = True
    with pytest.raises(TypeError): snapshot['intrinsics']['fx'] = 1
    with pytest.raises(TypeError): snapshot['stream_id'] = 'changed'
    assert snapshot['deproject']((.5, .6), .6) == [0., 0., .6]
    assert sdk.deprojections[0][0] is sdk.depth_profile.intrinsics
    assert sdk.alignments
    result = source.commit_if_current('one', snapshot, lambda: 'committed')
    assert result == 'committed'
    assert source.commit_if_current('one', snapshot, lambda: 'recorded') == 'recorded'


def test_retains_depth_for_older_delivered_image_not_latest(capture):
    source, sdk, _ = capture
    first, _ = delivered(source, sdk, 1, depth_value=410)
    delivered(source, sdk, 2, depth_value=790, timestamp=1033.)
    old = source.consume('one', first['frame_id'], first['stream_id'])
    assert source.commit_if_current('one', old, lambda: 'observed') == 'observed'
    assert source.commit_if_current('one', old, lambda: 'target') == 'target'
    assert np.allclose(old['depth_m'], .410)
    with pytest.raises(ValueError, match='已使用'):
        source.consume('one', first['frame_id'], first['stream_id'])


def test_undelivered_cross_session_and_out_of_order_frames_are_rejected(capture):
    source, sdk, _ = capture
    meta, _ = delivered(source, sdk)
    source.create('two')
    with pytest.raises(ValueError, match='session'):
        source.consume('two', meta['frame_id'], meta['stream_id'])
    two, _ = decode(source.frame('two'))
    assert source.consume('two', two['frame_id'], two['stream_id'])
    later, _ = delivered(source, sdk, 2, timestamp=1033.)
    source.consume('one', later['frame_id'], later['stream_id'])
    with pytest.raises(ValueError, match='乱序'):
        source.consume('one', meta['frame_id'], meta['stream_id'])


def test_expiry_checked_both_before_fitting_and_at_commit(capture):
    source, sdk, now = capture
    meta, _ = delivered(source, sdk)
    snapshot = source.consume('one', meta['frame_id'], meta['stream_id'])
    now[0] += .501
    stale, pixels = decode(source.frame('one'))
    assert stale['stale'] and not stale['has_frame'] and not pixels
    with pytest.raises(ValueError, match='过期'):
        source.commit_if_current('one', snapshot, lambda: pytest.fail('stale fit committed'))


def test_newer_fit_prevents_older_inflight_result_committing(capture):
    source, sdk, _ = capture
    first, _ = delivered(source, sdk)
    old = source.consume('one', first['frame_id'], first['stream_id'])
    second, _ = delivered(source, sdk, 2, timestamp=1033.)
    new = source.consume('one', second['frame_id'], second['stream_id'])
    assert source.commit_if_current('one', new, lambda: 'new') == 'new'
    with pytest.raises(ValueError):
        source.commit_if_current('one', old, lambda: pytest.fail('older fit committed'))


@pytest.mark.parametrize('kwargs', [dict(depth_timestamp=100.), dict(depth_frame_id=0)])
def test_mismatched_or_repeated_depth_is_not_paired_with_new_rgb(capture, kwargs):
    source, sdk, _ = capture
    delivered(source, sdk)
    source.retry_delay = 10.
    sdk.emit(2, timestamp=1033., **kwargs)
    wait_until(lambda: source.event == 'capture_error')
    meta, pixels = decode(source.frame('one'))
    assert '不同步' in meta['error'] and not pixels


def test_system_timestamp_delay_includes_initial_capture_age(capture):
    source, sdk, now = capture
    sdk.emit(timestamp=(1_700_000_000. + now[0]) * 1000 - 700, domain='global')
    wait_until(lambda: source.frames)
    meta, pixels = decode(source.frame('one'))
    assert meta['age_ms'] == pytest.approx(700)
    assert not meta['has_frame'] and not pixels


def test_hardware_timestamp_queue_delay_expires_old_capture(capture):
    source, sdk, now = capture
    delivered(source, sdk)
    now[0] += .8
    sdk.emit(2, timestamp=1033.)
    wait_until(lambda: 2 in source.frames)
    meta, pixels = decode(source.frame('one'))
    assert meta['age_ms'] == pytest.approx(767)
    assert not pixels


def test_disconnect_and_reconnect_invalidate_fit_and_change_stream(capture):
    source, sdk, _ = capture
    first, _ = delivered(source, sdk)
    snapshot = source.consume('one', first['frame_id'], first['stream_id'])
    previous = sdk.pipelines[-1]
    previous.queue.put(RuntimeError('USB disconnected'))
    wait_until(lambda: len(sdk.pipelines) >= 2 and source.event == 'waiting')
    assert previous.stopped and source.stream_id != first['stream_id']
    with pytest.raises(ValueError):
        source.commit_if_current('one', snapshot, lambda: pytest.fail('disconnected fit committed'))
    second, _ = delivered(source, sdk)
    assert second['stream_id'] != first['stream_id']
    assert source.consume('one', second['frame_id'], second['stream_id'])


def test_history_is_bounded_and_evicted_frames_cannot_be_consumed(capture):
    source, sdk, _ = capture
    source.history_size = 2
    first, _ = delivered(source, sdk, 1)
    delivered(source, sdk, 2, timestamp=1033.)
    delivered(source, sdk, 3, timestamp=1066.)
    assert len(source.frames) == 2
    with pytest.raises(ValueError, match='过期'):
        source.consume('one', first['frame_id'], first['stream_id'])


def test_last_session_closes_pipeline_and_pending_poll(capture):
    source, sdk, _ = capture
    source.create('two')
    assert len(sdk.pipelines) == 1
    source.close('two')
    assert not sdk.pipelines[-1].stopped
    meta, _ = delivered(source, sdk)
    results = []
    def poll():
        try: source.frame('one', meta['revision'])
        except ValueError as exc: results.append(str(exc))
    waiter = threading.Thread(target=poll)
    waiter.start()
    source.close('one')
    waiter.join(timeout=1.)
    assert not waiter.is_alive() and results
    assert sdk.pipelines[-1].stopped and source.worker is None
    source.close('one')
    with pytest.raises(ValueError): source.frame('one')


def test_missing_device_reports_health_without_busy_spin():
    sdk = FakeSDK()
    sdk.devices = []
    source = RealSenseSource(sdk_loader=lambda: sdk, retry_delay=10.)
    try:
        source.create('one')
        wait_until(lambda: source.error is not None)
        meta, pixels = decode(source.frame('one'))
        assert not pixels and not meta['connected']
        assert meta['event'] == 'capture_error' and 'udev' in meta['error']
    finally:
        source.close_all()


def test_invalid_requests_never_open_a_camera(capture):
    source, _, _ = capture
    for value in (-2, True, 1.5, '1', None):
        with pytest.raises(ValueError, match='revision'): source.frame('one', value)
    for value in (-1, True, 1.5, '1', None):
        with pytest.raises(ValueError, match='frame_id'): source.consume('one', value, 'stream')
