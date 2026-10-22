"""The browser proposes landmarks; captured RGB-D evidence stays server-owned."""

from copy import deepcopy

import numpy as np
import pytest

from online_control.online_motion_track import rgbd_input
from online_control.online_motion_track.rgbd_fusion import FusionUnavailable


class Source:
    def __init__(self, snapshot=None, error=None):
        self.snapshot = snapshot
        self.error = error
        self.calls = []

    def consume(self, *args):
        self.calls.append(args)
        if self.error:
            raise ValueError(self.error)
        return self.snapshot


@pytest.fixture
def packet():
    return dict(session='session-1', sequence=7, timestamp_ms=1000, source='realsense_rgbd',
                realsense_frame_id=42, realsense_stream_id='stream-1', capture_unix_ms=1,
                camera_size=[999, 999], raw_webxr={'spoof': True}, raw_wuji={'spoof': True},
                raw_leapmotion={'spoof': True}, source_metadata={'device_id': 'spoof'},
                hand={'label': 'Left', 'world': [[.01, .02, .03]] * 21,
                      'normalized': [[.5, .5, 0]] * 21})


@pytest.fixture
def snapshot():
    return dict(frame_id=42, stream_id='stream-1', revision=90, capture_unix_ms=1760000000123,
                timestamp_ms=3210, intrinsics={'width': 640, 'height': 480, 'fx': 600,
                                             'fy': 600, 'ppx': 320, 'ppy': 240},
                depth_m=np.zeros((480, 640)), deproject=lambda uv, z: [0, 0, z],
                device_id='native-device', device_name='D455f', age_ms=12,
                depth_mode={'width': 640, 'height': 480, 'fps': 30},
                color_mode={'width': 640, 'height': 480, 'fps': 30})


def test_native_depth_intrinsics_and_identity_replace_client_metadata(monkeypatch, packet, snapshot):
    calls = []
    corrected = np.arange(63).reshape(21, 3).tolist()

    def fit(normalized, world, depth, intrinsics, **kwargs):
        calls.append((normalized, world, depth, intrinsics, kwargs))
        return {'world': corrected, 'normalized': normalized,
                'diagnostics': {'measured_count': 15, 'method': 'test'}}

    monkeypatch.setattr(rgbd_input, 'fuse_hand', fit)
    source = Source(snapshot)
    original = deepcopy(packet)
    trusted, captured, fused, diagnostics = rgbd_input.prepare_packet(source, packet, 'left')
    assert source.calls == [('session-1', 42, 'stream-1')]
    assert captured is snapshot
    assert calls[0][2] is snapshot['depth_m']
    assert calls[0][3] is snapshot['intrinsics']
    assert calls[0][4]['deproject'] is snapshot['deproject']
    assert trusted['capture_unix_ms'] == snapshot['capture_unix_ms']
    assert trusted['camera_size'] == [640, 480]
    assert trusted['source_metadata']['device_id'] == 'native-device'
    assert trusted['source_metadata']['revision'] == 90
    assert trusted['source_metadata']['output_coordinate_space'] == 'camera_axes_wrist_relative'
    assert trusted['source_metadata']['mediapipe_world_prior'] == original['hand']['world']
    assert trusted['source_metadata']['mediapipe_normalized'] == original['hand']['normalized']
    assert trusted['hand'] is fused
    assert fused['world'] == corrected
    assert diagnostics['status'] == 'fused'
    assert diagnostics['fusion_ms'] >= 0
    assert trusted['raw_webxr'] is trusted['raw_wuji'] is trusted['raw_leapmotion'] is None
    assert packet == original


@pytest.mark.parametrize('reason', ['frame expired', 'stream restarted', 'frame already consumed', 'source closed'])
def test_rejected_capture_never_runs_fusion_or_returns_client_skeleton(monkeypatch, packet, reason):
    def unexpected(*args, **kwargs):
        pytest.fail('Fusion must not run without retained same-frame depth')

    monkeypatch.setattr(rgbd_input, 'fuse_hand', unexpected)
    trusted, captured, fused, diagnostics = rgbd_input.prepare_packet(Source(error=reason), packet, 'left')
    assert trusted['hand'] is captured is fused is None
    assert diagnostics == {'status': 'unavailable', 'measured_count': 0, 'reason': reason}
    assert trusted['source_metadata']['source'] == 'realsense_rgbd'
    assert 'device_id' not in trusted['source_metadata']


@pytest.mark.parametrize('missing', ['realsense_frame_id', 'realsense_stream_id'])
def test_missing_identity_never_uses_client_hand(packet, missing):
    del packet[missing]
    source = Source()
    trusted, snapshot, fused, diagnostics = rgbd_input.prepare_packet(source, packet, 'left')
    assert not source.calls
    assert trusted['hand'] is snapshot is fused is None
    assert diagnostics['reason'] == 'missing_rgbd_frame'


def test_invalid_depth_retains_original_prior_only_as_diagnostic(monkeypatch, packet, snapshot):
    def fail(*args, **kwargs):
        raise FusionUnavailable('all depth invalid')

    monkeypatch.setattr(rgbd_input, 'fuse_hand', fail)
    trusted, captured, fused, diagnostics = rgbd_input.prepare_packet(Source(snapshot), packet, 'left')
    assert trusted['hand'] is fused is None
    assert captured is snapshot
    assert diagnostics['status'] == 'unavailable'
    assert diagnostics['reason'] == 'all depth invalid'
    assert trusted['source_metadata']['mediapipe_world_prior'] == packet['hand']['world']


def test_wrong_hand_is_not_fused(monkeypatch, packet, snapshot):
    def unexpected(*args, **kwargs):
        pytest.fail('Wrong-side hand must not enter fusion')

    monkeypatch.setattr(rgbd_input, 'fuse_hand', unexpected)
    trusted, captured, fused, diagnostics = rgbd_input.prepare_packet(Source(snapshot), packet, 'right')
    assert captured is snapshot
    assert trusted['hand'] is fused is None
    assert diagnostics['status'] == 'no_hand'
    assert diagnostics['reason'] == 'wrong_hand'


def test_no_hand_still_uses_native_capture_time(packet, snapshot):
    packet['hand'] = None
    trusted, captured, fused, diagnostics = rgbd_input.prepare_packet(Source(snapshot), packet, 'left')
    assert captured is snapshot
    assert trusted['hand'] is fused is None
    assert diagnostics == {'status': 'no_hand', 'measured_count': 0}
    assert trusted['capture_unix_ms'] == snapshot['capture_unix_ms']


@pytest.mark.parametrize('bad_hand', [[], 'hand', {'label': 'Unknown'}])
def test_invalid_hand_object_is_rejected(packet, snapshot, bad_hand):
    packet['hand'] = bad_hand
    with pytest.raises(ValueError, match='手别'):
        rgbd_input.prepare_packet(Source(snapshot), packet, 'left')


def test_real_fitter_rejects_all_invalid_depth_without_fallback(packet, snapshot):
    packet['hand']['world'] = [[i * .005, 0, 0] for i in range(21)]
    trusted, _, fused, diagnostics = rgbd_input.prepare_packet(Source(snapshot), packet, 'left')
    assert trusted['hand'] is fused is None
    assert diagnostics['status'] == 'unavailable'
    assert 'insufficient depth' in diagnostics['reason']
