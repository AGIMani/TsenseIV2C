"""Shared camera alignment, gating and optional Python camera capture."""
import copy
import multiprocessing as mp
import os
from pathlib import Path
from queue import Queue
import sys
import threading
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src'), str(ROOT / 'tools')]

import numpy as np
import pytest

from hand_profiles import load_hand_profile
from keypoint_mediapipe_input import (MediaPipeInput, align_world_landmarks,
                                      capture_loop, observation_packet)


@pytest.mark.parametrize('model', ['TsenseI_v3_2', 'TsenseI_v3_2_withid'])
@pytest.mark.parametrize('side', ['left', 'right'])
def test_alignment_removes_camera_pose_and_scale_and_matches_chirality(model, side):
    reference = load_hand_profile(model).create_retargeter(side).fk.compute({})
    rotation = np.array([[0, 0, 1], [1, 0, 0], [0, 1, 0]])
    camera = reference @ rotation * 1.4 + [.2, -.4, .7]
    np.testing.assert_allclose(align_world_landmarks(camera, reference, side, side), reference, atol=1e-12)
    reflected = camera * [-1, 1, 1]
    other = 'right' if side == 'left' else 'left'
    np.testing.assert_allclose(align_world_landmarks(reflected, reference, other, side), reference, atol=1e-12)
    camera[5] = camera[17]
    with pytest.raises(ValueError, match='退化'):
        align_world_landmarks(camera, reference, side, side)


def observation(points):
    return SimpleNamespace(held_last_valid=False, used_fallback_hand=False,
                           expected_label='Left', detected_label='Left', world_points=points)


def test_world_landmark_gate_does_not_use_wrong_hand_held_or_normalized_fallback():
    points = load_hand_profile('TsenseI_v3_2_withid').create_retargeter().fk.compute({})
    source = observation(points)
    packet = observation_packet(source, 'Left')
    np.testing.assert_array_equal(packet['points'], points)
    assert packet['points'] is not points
    for key, value in [('held_last_valid', True), ('used_fallback_hand', True),
                       ('detected_label', 'Right'), ('expected_label', 'Right'), ('world_points', None)]:
        bad = copy.copy(source)
        setattr(bad, key, value)
        assert observation_packet(bad, 'Left')['points'] is None
    source.world_points = np.full((21, 3), np.nan)
    assert observation_packet(source, 'Left')['points'] is None
    assert observation_packet(None, 'Left')['points'] is None


def test_capture_failure_releases_detector_and_camera():
    pytest.importorskip("cv2", reason="Optional Python camera capture dependencies")
    class Capture:
        released = False
        def isOpened(self): return True
        def get(self, prop): return 0
        def read(self): return True, np.zeros((16, 16, 3), dtype=np.uint8)
        def release(self): self.released = True

    class Tracker:
        closed = False
        expected_mp_label = 'Left'
        def process_bgr_detailed(self, frame): raise RuntimeError('test detector failure')
        def close(self): self.closed = True

    capture, tracker, queue = Capture(), Tracker(), Queue(maxsize=1)
    capture_loop('0', 'left', queue, threading.Event(), lambda _: capture, lambda _: tracker)
    packet = queue.get_nowait()
    assert packet['terminal'] and 'test detector failure' in packet['message']
    assert capture.released and tracker.closed


def test_camera_requests_mjpeg_before_size_and_reports_actual_format(monkeypatch):
    cv2 = pytest.importorskip("cv2", reason="Optional Python camera capture dependencies")
    from keypoint_mediapipe_input import open_capture, capture_description
    calls = []
    class Capture:
        def isOpened(self): return True
        def set(self, prop, value): calls.append((prop, value)); return True
        def get(self, prop):
            return {cv2.CAP_PROP_FRAME_WIDTH: 640, cv2.CAP_PROP_FRAME_HEIGHT: 480,
                    cv2.CAP_PROP_FOURCC: cv2.VideoWriter_fourcc(*'MJPG')}[prop]
    capture = Capture()
    monkeypatch.setattr(cv2, 'VideoCapture', lambda source: capture)
    assert open_capture('0') is capture
    assert calls[0] == (cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    assert capture_description(capture) == '640×480 MJPG'
    calls.clear()
    open_capture('/dev/video0')
    assert calls[0][0] == cv2.CAP_PROP_FOURCC
    calls.clear()
    open_capture('/tmp/recorded.avi')
    assert not calls


def test_owned_camera_process_can_be_stopped_even_when_read_is_blocked():
    source = MediaPipeInput.__new__(MediaPipeInput)
    source.context = mp.get_context('spawn')
    source.queue = source.context.Queue(maxsize=1)
    source.stop_event = source.context.Event()
    # Simulate a driver that never observes stop_event.
    source.process = source.context.Process(target=time.sleep, args=(60,), daemon=True)
    source.process.start()
    started = time.monotonic()
    source.close()
    assert time.monotonic() - started < 2
    assert source.process._closed


def test_real_mediapipe_initialization_and_video_eof(tmp_path):
    pytest.importorskip("mediapipe", reason="Optional Python camera detector")
    # Real Tasks detector and bundled model, using synthetic blank video.
    # This verifies runtime integration, not real-camera hand accuracy.
    cv2 = pytest.importorskip("cv2", reason="Optional Python camera capture dependencies")
    video = tmp_path / 'blank.avi'
    writer = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*'MJPG'), 10, (64, 64))
    assert writer.isOpened()
    for _ in range(3):
        writer.write(np.zeros((64, 64, 3), dtype=np.uint8))
    writer.release()
    packets = []
    class Recorder:
        def put_nowait(self, packet): packets.append(packet)
    capture_loop(str(video), 'left', Recorder(), threading.Event())
    assert any('未检测到' in packet['message'] for packet in packets)
    assert packets[-1]['terminal'] and '视频已结束' in packets[-1]['message']
    assert all(packet['points'] is None for packet in packets)

