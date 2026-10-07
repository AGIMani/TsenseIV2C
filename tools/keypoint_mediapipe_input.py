"""MediaPipe camera input for the calibration editor, with bounded shutdown.

Capture/detection live in an owned child process: even a blocked camera read
cannot strand a Qt thread on close. Only the newest 21-point packet is kept.
"""
from __future__ import annotations

import multiprocessing as mp
from pathlib import Path
from queue import Empty, Full
import sys
import time

import numpy as np


def palm_frame(points):
    points = np.asarray(points, dtype=float)
    if points.shape != (21, 3) or not np.isfinite(points).all():
        raise ValueError('需要 21 个有限三维关键点')
    across, forward = points[5] - points[17], points[9] - points[0]
    width, length = np.linalg.norm(across), np.linalg.norm(forward)
    if min(width, length) < 1e-6:
        raise ValueError('掌宽或腕部参考骨段退化')
    forward /= length
    normal = np.cross(across / width, forward)
    norm = np.linalg.norm(normal)
    if norm < .05:
        raise ValueError('掌部关键点接近共线')
    normal /= norm
    return np.column_stack((np.cross(forward, normal), forward, normal)), width


def align_world_landmarks(points, reference, source_side, model_side):
    """Wrist-centered, uniformly scaled palm alignment; preserve bone ratios.

    A changed source hand is explicitly reflected in palm-local depth, while
    the displayed URDF/its calibration remain the selected model and side.
    """
    if source_side not in ('left', 'right') or model_side not in ('left', 'right'):
        raise ValueError('手别必须为 left / right')
    points, reference = np.asarray(points, dtype=float), np.asarray(reference, dtype=float)
    source_frame, source_width = palm_frame(points)
    target_frame, target_width = palm_frame(reference)
    local = (points - points[0]) @ source_frame
    if source_side != model_side:
        local[:, 2] *= -1
    return local @ target_frame.T * (target_width / source_width) + reference[0]


def observation_packet(observation, expected_label):
    if observation is None or observation.held_last_valid:
        return {'points': None, 'message': '未检测到所选手，保持最后姿态'}
    if (observation.used_fallback_hand or observation.detected_label != expected_label
            or observation.expected_label != expected_label):
        return {'points': None, 'message': f'检测到 {observation.detected_label or "未知"}，等待 {expected_label}'}
    if observation.world_points is None:
        return {'points': None, 'message': '缺少三维 world landmarks，保持最后姿态'}
    try:
        points = np.asarray(observation.world_points, dtype=float)
        palm_frame(points)
    except (ValueError, TypeError) as exc:
        return {'points': None, 'message': f'关键点无效：{exc}'}
    return {'points': points.copy(), 'message': f'MediaPipe · {expected_label} · 21 点实时输入'}


def _publish(queue, packet):
    packet = dict(packet, timestamp=packet.get('timestamp', time.monotonic()))
    for _ in range(2):
        try:
            queue.put_nowait(packet)
            return
        except Full:
            try:
                queue.get(timeout=.02)
            except Empty:
                pass


def create_tracker(side):
    # Bypass tracking/__init__, which imports optional torch-based trackers.
    tracking = str(Path(__file__).resolve().parents[1] / 'src' / 'tracking')
    if tracking not in sys.path:
        sys.path.insert(0, tracking)
    from wuji_mediapipe import WujiMediaPipe21, WujiMediaPipeConfig
    return WujiMediaPipe21(side, config=WujiMediaPipeConfig(
        max_num_hands=2, input_is_mirrored=False, swap_handedness=False,
        correct_segments=False, hold_last_valid=False, smoothing_enabled=True))


def open_capture(source):
    import cv2
    source = int(source) if str(source).isdecimal() else str(source)
    capture = cv2.VideoCapture(source)
    if capture.isOpened() and (isinstance(source, int) or str(source).startswith('/dev/video')):
        # Prefer compressed USB video before negotiating size/rate. Some USB
        # transports deliver incomplete YUYV frames even when read() succeeds.
        # This is standard V4L2/OpenCV negotiation, also used on native Linux.
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        capture.set(cv2.CAP_PROP_FPS, 30)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    return capture


def capture_description(capture):
    import cv2
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    code = int(capture.get(cv2.CAP_PROP_FOURCC))
    fourcc = ''.join(chr((code >> (8 * i)) & 255) for i in range(4)).strip('\x00 ')
    return f'{width}×{height} {fourcc or "未知格式"}'


def capture_loop(source, side, queue, stop, capture_factory=None, tracker_factory=None):
    capture = tracker = None
    try:
        _publish(queue, {'points': None, 'message': '正在打开相机并初始化 MediaPipe…'})
        capture = (capture_factory or open_capture)(source)
        if not capture.isOpened():
            raise RuntimeError(f'无法打开相机/视频 {source}；请检查 Linux 视频设备或文件路径')
        format_description = capture_description(capture)
        if stop.is_set():
            return
        tracker = (tracker_factory or create_tracker)(side)
        import cv2
        is_file = Path(str(source)).is_file()
        fps = capture.get(cv2.CAP_PROP_FPS) if is_file else 30
        period = 1 / fps if np.isfinite(fps) and fps > 0 else 1 / 30
        while not stop.is_set():
            start = time.monotonic()
            ok, frame = capture.read()
            if not ok:
                message = '视频已结束，已冻结最后姿态' if is_file else '相机读取失败，已冻结最后姿态'
                _publish(queue, {'points': None, 'message': message, 'terminal': True})
                return
            observation = tracker.process_bgr_detailed(frame)
            packet = observation_packet(observation, tracker.expected_mp_label)
            packet['message'] += f' · {format_description}'
            packet['detect_ms'] = (time.monotonic() - start) * 1000
            packet['timestamp'] = start
            _publish(queue, packet)
            stop.wait(max(0, period - (time.monotonic() - start)))
    except Exception as exc:
        _publish(queue, {'points': None, 'message': f'MediaPipe 接入失败：{exc}', 'terminal': True})
    finally:
        for resource, method in ((capture, 'release'), (tracker, 'close')):
            if resource is not None:
                try:
                    getattr(resource, method)()
                except Exception:
                    pass


class MediaPipeInput:
    def __init__(self, source, side):
        self.context = mp.get_context('spawn')
        self.queue = self.context.Queue(maxsize=1)
        self.stop_event = self.context.Event()
        self.process = self.context.Process(target=capture_loop,
                                            args=(source, side, self.queue, self.stop_event), daemon=True)
        try:
            self.process.start()
        except Exception:
            self.queue.close()
            raise

    def take_latest(self):
        packet = None
        try:
            while True:
                packet = self.queue.get_nowait()
        except Empty:
            return packet

    def is_alive(self):
        return self.process.is_alive()

    def close(self):
        self.stop_event.set()
        self.process.join(.2)
        if self.process.is_alive():
            self.process.terminate()
            self.process.join(.5)
        if self.process.is_alive():
            self.process.kill()
            self.process.join(.5)
        self.process.close()
        self.queue.cancel_join_thread()
        self.queue.close()
