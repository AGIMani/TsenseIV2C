"""Native, optional RealSense RGB-D capture for browser MediaPipe detection.

Only explicit RGB-D sessions open the device. A single capture thread aligns
depth to RGB; a bounded history retains the exact depth paired with each image
sent to the browser. Neither OpenCV nor Qt is needed. The SDK's distortion-aware
deprojection uses the aligned frame's intrinsics, not a guessed pinhole model.
"""
from collections import OrderedDict, deque
import importlib
import json
import math
import struct
import threading
import time
from types import MappingProxyType
import uuid

import numpy as np

STALE_AFTER_MS = 500
FRAME_WAIT_MS = 250


def encode_frame(metadata, pixels):
    """Little-endian uint32 JSON length, UTF-8 JSON, then packed RGB8 pixels."""
    header = json.dumps(metadata, ensure_ascii=False, allow_nan=False).encode('utf-8')
    return struct.pack('<I', len(header)) + header + pixels


def _session_key(session):
    if not isinstance(session, str) or not session:
        raise ValueError('RealSense session 必须为非空字符串')
    return session


def _mode(profile):
    video = profile.as_video_stream_profile()
    return dict(width=int(video.width()), height=int(video.height()), fps=int(profile.fps()))


def _profile_pairs(sdk, device):
    """Prefer 640x480/30; only request RGB8/Z16 modes the device advertises.

    Color/depth dimensions may differ before alignment, but FPS must match.
    Avoid assuming D455 color supports the depth sensor's 90 FPS profile.
    """
    depth, color = [], []
    for sensor in device.query_sensors():
        for profile in sensor.get_stream_profiles():
            if profile.stream_type() == sdk.stream.depth and profile.format() == sdk.format.z16:
                depth.append(_mode(profile))
            elif profile.stream_type() == sdk.stream.color and profile.format() == sdk.format.rgb8:
                color.append(_mode(profile))
    pairs = {(d['width'], d['height'], c['width'], c['height'], d['fps'])
             for d in depth for c in color if d['fps'] == c['fps'] and 0 < d['fps'] <= 60
             and max(d['width'], c['width']) <= 1280 and max(d['height'], c['height']) <= 800}
    def rank(pair):
        dw, dh, cw, ch, fps = pair
        return (abs(fps - 30), abs(cw - 640) + abs(ch - 480),
                abs(dw - 640) + abs(dh - 480), dw * dh + cw * ch)
    return sorted(pairs, key=rank)


class RealSenseSource:
    """One SDK pipeline shared by explicitly registered RGB-D sessions.

    create/close are idempotent. frame grants a session access to a particular
    capture, consume claims that capture once, and commit_if_current validates
    it again after fitting. Disconnect/restart/close invalidate in-flight fits.
    SDK import happens synchronously in create so a missing optional dependency
    is an actionable request error instead of an endless waiting camera.
    """
    def __init__(self, *, sdk_loader=None, clock=time.monotonic, unix_clock=time.time,
                 history_size=24, stale_after_ms=STALE_AFTER_MS, retry_delay=1.):
        if history_size < 2 or stale_after_ms <= 0 or retry_delay <= 0:
            raise ValueError('RealSense history、超时与重试间隔必须为正值')
        self.sdk_loader = sdk_loader or (lambda: importlib.import_module('pyrealsense2'))
        self.clock, self.unix_clock = clock, unix_clock
        self.history_size, self.stale_after_ms = int(history_size), float(stale_after_ms)
        self.retry_delay = retry_delay
        self.lock = threading.RLock()
        self.changed = threading.Condition(self.lock)
        # Serialize starting/stopping pipelines, without holding the capture lock
        # while joining the worker (its finally block also needs that lock).
        self.lifecycle_lock = threading.Lock()
        self.sessions = {}
        self.frames = OrderedDict()
        self.worker = None
        self.stream_id = None
        self.revision = 0
        self.connected = False
        self.event, self.error = 'stopped', None
        self.device_id = self.device_name = None
        self.depth_mode = self.color_mode = None
        self.image_times = deque(maxlen=121)

    def create(self, session):
        session = _session_key(session)
        with self.lifecycle_lock:
            with self.lock:
                if session in self.sessions:
                    return
                sdk = self.worker['sdk'] if self.worker else None
            if sdk is None:
                try:
                    sdk = self.sdk_loader()
                except (ImportError, OSError) as exc:
                    raise RuntimeError('RealSense RGB-D 需要 track 环境中的 pyrealsense2；'
                                       '请运行 .venvs/track/bin/python -m pip install pyrealsense2，'
                                       '并按 README-UBUNTU.txt 配置原生 Linux USB 权限。') from exc
            with self.changed:
                self.sessions[session] = dict(token=object(), delivered=set(), consumed=-1, claimed={})
                if self.worker is not None:
                    return
                worker = dict(stop=threading.Event(), sdk=sdk)
                worker['thread'] = threading.Thread(target=self._run, args=(worker,),
                                                     name='realsense-rgbd', daemon=True)
                self.worker = worker
                self.event, self.error = 'starting', None
                self.revision += 1
                worker['thread'].start()
                self.changed.notify_all()

    def close(self, session):
        session = _session_key(session)
        with self.lifecycle_lock:
            with self.changed:
                self.sessions.pop(session, None)
                worker = self._stop_locked() if not self.sessions else None
                self.changed.notify_all()
            if worker:
                worker['thread'].join(timeout=3.)

    def close_all(self):
        with self.lifecycle_lock:
            with self.changed:
                self.sessions.clear()
                worker = self._stop_locked()
                self.changed.notify_all()
            if worker:
                worker['thread'].join(timeout=3.)

    def _stop_locked(self):
        worker, self.worker = self.worker, None
        if worker:
            worker['stop'].set()
        self._invalidate_locked('stopped', None)
        return worker

    def _invalidate_locked(self, event, error):
        self.connected = False
        self.event, self.error = event, error
        self.frames.clear()
        self.image_times.clear()
        self.stream_id = uuid.uuid4().hex
        self.revision += 1
        for state in self.sessions.values():
            state['delivered'].clear()
            state['claimed'].clear()
            state['consumed'] = -1
        self.changed.notify_all()

    def _current(self, worker):
        return self.worker is worker and not worker['stop'].is_set()

    def _open(self, sdk, worker):
        devices = list(sdk.context().query_devices())
        if not devices:
            raise RuntimeError('未发现 RealSense 深度相机；请检查 USB 3 连接与 Linux udev 权限')
        last_error = None
        for device in devices:
            serial = str(device.get_info(sdk.camera_info.serial_number))
            name = str(device.get_info(sdk.camera_info.name))
            # Bound expensive startup attempts when a device is occupied or its
            # advertised modes exceed the current USB connection's bandwidth.
            for dw, dh, cw, ch, fps in _profile_pairs(sdk, device)[:6]:
                if worker['stop'].is_set():
                    raise RuntimeError('RealSense 输入已停止')
                pipeline = sdk.pipeline()
                config = sdk.config()
                config.enable_device(serial)
                config.enable_stream(sdk.stream.depth, dw, dh, sdk.format.z16, fps)
                config.enable_stream(sdk.stream.color, cw, ch, sdk.format.rgb8, fps)
                try:
                    active = pipeline.start(config)
                    scale = float(active.get_device().first_depth_sensor().get_depth_scale())
                    if not math.isfinite(scale) or scale <= 0:
                        raise ValueError('设备返回了无效深度单位')
                    depth_mode = _mode(active.get_stream(sdk.stream.depth))
                    color_mode = _mode(active.get_stream(sdk.stream.color))
                    return pipeline, sdk.align(sdk.stream.color), scale, serial, name, depth_mode, color_mode
                except Exception as exc:
                    last_error = exc
                    try:
                        pipeline.stop()
                    except Exception:
                        pass
        raise RuntimeError('无法启动 RealSense 同步 RGB/深度流；请释放普通相机占用，'
                           '检查 USB 3 与 udev 权限。' + (f' {last_error}' if last_error else ' 未找到共同帧率的 RGB8/Z16 模式。'))

    def _run(self, worker):
        sdk = worker['sdk']
        while not worker['stop'].is_set():
            pipeline = None
            try:
                pipeline, align, scale, serial, name, depth_mode, color_mode = self._open(sdk, worker)
                with self.changed:
                    if not self._current(worker):
                        return
                    self._invalidate_locked('waiting', None)
                    generation = self.stream_id
                    self.device_id, self.device_name = serial, name
                    self.depth_mode, self.color_mode = depth_mode, color_mode
                timestamp_offset = None
                previous_frame = -1
                previous_depth_frame = -1
                last_arrival = time.monotonic()
                while not worker['stop'].is_set():
                    try:
                        frames = pipeline.wait_for_frames(FRAME_WAIT_MS)
                    except RuntimeError as exc:
                        # First exposure can take longer than one HTTP poll.
                        # Missing data still expires at stale_after_ms; a bounded
                        # two-second grace avoids repeatedly restarting startup.
                        message = str(exc).lower()
                        is_timeout = any(part in message for part in ("frame didn't arrive", 'timed out', 'timeout'))
                        if is_timeout and time.monotonic() - last_arrival < 2.:
                            worker['stop'].wait(.005)
                            continue
                        raise
                    last_arrival = time.monotonic()
                    receipt, unix_ms = self.clock(), self.unix_clock() * 1000
                    aligned = align.process(frames)
                    depth, color = aligned.get_depth_frame(), aligned.get_color_frame()
                    if not depth or not color:
                        # Missing pairs are an acquisition error, not fresh RGB.
                        raise RuntimeError('RealSense 没有返回配对的彩色与深度帧')
                    frame_id = int(color.get_frame_number())
                    timestamp_ms = float(color.get_timestamp())
                    depth_frame_id = int(depth.get_frame_number())
                    depth_timestamp_ms = float(depth.get_timestamp())
                    if frame_id <= previous_frame or not math.isfinite(timestamp_ms) or timestamp_ms < 0:
                        raise RuntimeError('RealSense 帧编号或时间戳回退')
                    if (depth_frame_id <= previous_depth_frame or not math.isfinite(depth_timestamp_ms)
                            or abs(depth_timestamp_ms - timestamp_ms) > max(50., 1500. / color_mode['fps'])
                            or depth.get_frame_timestamp_domain() != color.get_frame_timestamp_domain()):
                        raise RuntimeError('RealSense 彩色与深度帧不同步')
                    previous_frame = frame_id
                    previous_depth_frame = depth_frame_id
                    observed_offset = receipt - timestamp_ms / 1000
                    timestamp_offset = observed_offset if timestamp_offset is None else min(timestamp_offset, observed_offset)
                    observed_at = min(receipt, timestamp_ms / 1000 + timestamp_offset)
                    # Native hardware time has an arbitrary origin. System/global
                    # time, when available, also accounts for the first frame's age.
                    domain = color.get_frame_timestamp_domain()
                    if domain in (sdk.timestamp_domain.system_time, sdk.timestamp_domain.global_time):
                        observed_at = receipt - max(0., unix_ms - timestamp_ms) / 1000
                    snapshot, pixels = self._copy_frame(sdk, depth, color, scale, frame_id,
                                                       timestamp_ms, observed_at, receipt, unix_ms, generation)
                    with self.changed:
                        if not self._current(worker) or self.stream_id != generation:
                            return
                        self.revision += 1
                        snapshot['revision'] = self.revision
                        snapshot.update(device_id=serial, device_name=name,
                                        depth_mode=MappingProxyType(dict(depth_mode)),
                                        color_mode=MappingProxyType(dict(color_mode)))
                        frozen = MappingProxyType(snapshot)
                        self.frames[frame_id] = (frozen, pixels)
                        self.image_times.append(observed_at)
                        self.connected, self.event, self.error = True, 'frames', None
                        while len(self.frames) > self.history_size:
                            evicted, _ = self.frames.popitem(last=False)
                            for state in self.sessions.values():
                                state['delivered'].discard(evicted)
                                state['claimed'].pop(evicted, None)
                        self.changed.notify_all()
            except Exception as exc:
                with self.changed:
                    if self._current(worker):
                        self._invalidate_locked('capture_error', str(exc))
            finally:
                if pipeline is not None:
                    try:
                        pipeline.stop()
                    except Exception:
                        pass
            # Event.wait permits immediate shutdown and prevents an empty busy loop.
            worker['stop'].wait(self.retry_delay)

    @staticmethod
    def _copy_frame(sdk, depth, color, scale, frame_id, timestamp_ms, observed_at, receipt, unix_ms, generation):
        native_intrinsics = depth.profile.as_video_stream_profile().get_intrinsics()
        rgb = np.asanyarray(color.get_data())
        raw_depth = np.asanyarray(depth.get_data())
        width, height = int(native_intrinsics.width), int(native_intrinsics.height)
        if (not 0 < width <= 1280 or not 0 < height <= 800 or rgb.dtype != np.uint8
                or rgb.shape != (height, width, 3) or raw_depth.dtype != np.uint16
                or raw_depth.shape != (height, width)):
            raise ValueError('RealSense 对齐后的 RGB8 / Z16 图像大小无效')
        intrinsics = dict(width=width, height=height, fx=float(native_intrinsics.fx),
                          fy=float(native_intrinsics.fy), ppx=float(native_intrinsics.ppx),
                          ppy=float(native_intrinsics.ppy))
        if not all(math.isfinite(v) for v in intrinsics.values()) or min(intrinsics['fx'], intrinsics['fy']) <= 0:
            raise ValueError('RealSense 相机内参无效')
        # A bytes-backed NumPy view cannot have its writeable flag re-enabled.
        depth_m = np.frombuffer((raw_depth.astype(np.float32) * scale).tobytes(),
                                dtype=np.float32).reshape(height, width)
        def deproject(uv, z):
            return sdk.rs2_deproject_pixel_to_point(native_intrinsics, [float(uv[0]), float(uv[1])], float(z))
        return dict(depth_m=depth_m, intrinsics=MappingProxyType(intrinsics), deproject=deproject,
                    capture_unix_ms=unix_ms - max(0., receipt - observed_at) * 1000,
                    timestamp_ms=timestamp_ms, frame_id=frame_id, stream_id=generation,
                    observed_at=observed_at), rgb.tobytes()

    def _state_locked(self, session):
        state = self.sessions.get(_session_key(session))
        if state is None or self.worker is None:
            raise ValueError('RealSense 输入已停止或 session 不匹配')
        return state

    def frame(self, session, after_revision=-1):
        if type(after_revision) is not int or after_revision < -1:
            raise ValueError('RealSense revision 必须为 -1 或非负整数')
        with self.changed:
            state = self._state_locked(session)
            token = state['token']
            if not self.frames or self.revision == after_revision:
                self.changed.wait(FRAME_WAIT_MS / 1000)
            state = self._state_locked(session)
            if state['token'] is not token:
                raise ValueError('RealSense 输入已重新启动')
            item = next(reversed(self.frames.values()), None)
            snapshot, pixels = item if item else ({}, b'')
            age = max(0., (self.clock() - snapshot['observed_at']) * 1000) if item else None
            fresh = self.connected and age is not None and age < self.stale_after_ms
            send = fresh and snapshot['revision'] > after_revision
            if send:
                state['delivered'].add(snapshot['frame_id'])
            elapsed = self.image_times[-1] - self.image_times[0] if len(self.image_times) > 1 else 0.
            fps = (len(self.image_times) - 1) / elapsed if elapsed >= 1. else None
            meta = dict(source='realsense_rgbd', format='rgb8', has_frame=bool(send),
                        revision=self.revision, stream_id=self.stream_id, connected=self.connected,
                        event=self.event, error=self.error, stale_after_ms=self.stale_after_ms,
                        age_ms=round(age, 2) if age is not None else None,
                        stale=not fresh, device_id=self.device_id, device_name=self.device_name,
                        depth_mode=self.depth_mode, color_mode=self.color_mode,
                        target_fps=self.color_mode['fps'] if self.color_mode else None,
                        capture_fps=round(fps, 2) if fps is not None else None,
                        width=snapshot['intrinsics']['width'] if item else 0,
                        height=snapshot['intrinsics']['height'] if item else 0,
                        frame_id=snapshot.get('frame_id', -1), timestamp_ms=snapshot.get('timestamp_ms'),
                        capture_unix_ms=snapshot.get('capture_unix_ms'))
            return encode_frame(meta, pixels if send else b'')

    def consume(self, session, frame_id, stream_id):
        if type(frame_id) is not int or frame_id < 0 or not isinstance(stream_id, str):
            raise ValueError('RealSense frame_id / stream_id 无效')
        with self.lock:
            state = self._state_locked(session)
            if stream_id != self.stream_id:
                raise ValueError('RealSense 流已断开或重新启动')
            if frame_id <= state['consumed']:
                raise ValueError('RealSense 帧已使用或乱序')
            item = self.frames.get(frame_id)
            if item is None or frame_id not in state['delivered']:
                raise ValueError('RealSense 帧不存在、已过期或未发送给此 session')
            snapshot = item[0]
            self._validate_locked(session, snapshot, require_claimed=False)
            state['consumed'] = frame_id
            state['claimed'].clear()
            state['claimed'][frame_id] = snapshot
            return snapshot

    def _validate_locked(self, session, snapshot, *, require_claimed=True):
        state = self._state_locked(session)
        item = self.frames.get(snapshot['frame_id'])
        if (not self.connected or snapshot['stream_id'] != self.stream_id or item is None
                or item[0] is not snapshot
                or (require_claimed and (state['claimed'].get(snapshot['frame_id']) is not snapshot
                                         or state['consumed'] != snapshot['frame_id']))):
            raise ValueError('RealSense 帧已失效、断开或不属于此 session')
        if self.clock() - snapshot['observed_at'] >= self.stale_after_ms / 1000:
            raise ValueError('RealSense 帧已过期，请等待新的同步 RGB-D 图像')

    def validate_if_current(self, session, snapshot):
        with self.lock:
            self._validate_locked(session, snapshot)
            return True

    def commit_if_current(self, session, snapshot, callback):
        """Validate each pipeline commit; callback must be short/nonblocking.

        A metric pipeline may observe tracking, push a target and record that
        same fit via separate callbacks. The capture itself is consumed once;
        callbacks remain valid until a newer consume or lifecycle invalidation.
        """
        with self.lock:
            self._validate_locked(session, snapshot)
            return callback()
