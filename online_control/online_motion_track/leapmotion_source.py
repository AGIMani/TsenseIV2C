"""Read the local Ultraleap service through its optional official Python binding.

All LeapC objects are copied inside their callback. No camera, browser, robot or
WSL APIs participate in acquisition; the same reader runs on native Ubuntu.
"""
from copy import deepcopy
from contextlib import ExitStack
import importlib
import logging
import math
import threading
import time
import uuid

STALE_AFTER_MS = 350
FRAME_TIMEOUT_SECONDS = 5.
TRACKING_MODE = 'hmd'
# LEAP_HINT_HIGH_HAND_FIDELITY from LeapC.h (not exported by the Python binding).
DEVICE_HINTS = ('high_hand_fidelity',)
LOGGER = logging.getLogger(__name__)
FINGERS = ('thumb', 'index', 'middle', 'ring', 'pinky')
BONES = ('metacarpal', 'proximal', 'intermediate', 'distal')
JOINT_MAPPING = ['arm.next_joint'] + [
    f'{finger}.{joint}' for finger in FINGERS for joint in
    ('proximal.prev_joint', 'proximal.next_joint', 'intermediate.next_joint', 'distal.next_joint')]


def _number(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f'Leap Motion {label} 必须为有限数值')
    return value


def _integer(value, label):
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f'Leap Motion {label} 必须为非负整数')
    return value


def _vector(value):
    point = [_number(getattr(value, axis), '坐标') for axis in ('x', 'y', 'z')]
    if any(abs(v) > 10000 for v in point):
        raise ValueError('Leap Motion 坐标超出 10 米范围')
    return point


def _bone(value):
    return {'prev_joint': _vector(value.prev_joint), 'next_joint': _vector(value.next_joint)}


def hand_frame(hand, *, sequence, timestamp_us, stream_id):
    """LeapC millimetres -> wrist-relative MediaPipe-order 21 points in metres.

    The thumb's metacarpal is zero length in LeapC. Its proximal bone is the
    anatomical thumb metacarpal, so proximal.prev_joint is the thumb CMC.
    Keep the original coordinate handedness (no reflection or image projection).
    """
    kind = str(hand.type)
    if kind not in ('HandType.Left', 'HandType.Right'):
        raise ValueError('Leap Motion 手别无效')
    side = 'left' if kind == 'HandType.Left' else 'right'
    arm = _bone(hand.arm)
    digits = {finger: {bone: _bone(getattr(getattr(hand, finger), bone)) for bone in BONES}
              for finger in FINGERS}
    points = [arm['next_joint']]
    for digit in digits.values():
        points.extend((digit['proximal']['prev_joint'], digit['proximal']['next_joint'],
                       digit['intermediate']['next_joint'], digit['distal']['next_joint']))
    world = [[(p[i] - points[0][i]) / 1000 for i in range(3)] for p in points]
    if any(abs(v) > 10 for point in world for v in point):
        raise ValueError('Leap Motion 手部相对坐标超出 10 米范围')
    confidence = _number(hand.confidence, '置信度')
    if not 0 <= confidence <= 1:
        raise ValueError('Leap Motion 置信度超出范围')
    raw = dict(schema='ultraleap.leapc.hand.v1', side=side,
               stream_id=stream_id, sequence=sequence, timestamp_us=timestamp_us,
               hand_id=_integer(hand.id, '手 ID'), units='mm',
               coordinate_space='leap_device', arm=arm, digits=digits,
               confidence=confidence, positions_xyz=deepcopy(points))
    metric = dict(world=world, normalized=deepcopy(world), label=side.title(),
                  source='leapmotion', coordinate_space='leap_wrist_relative',
                  normalized_is_image=False)
    return side, raw, metric


class LeapMotionSource:
    """One reconnecting service connection shared by both hands and all sessions."""
    device_hints = DEVICE_HINTS
    def __init__(self, *, clock=time.monotonic, sdk_loader=None, retry_delay=1.):
        self.clock = clock
        self.sdk_loader = sdk_loader or (lambda: importlib.import_module('leap'))
        self.retry_delay = retry_delay
        self.lock = threading.RLock()
        self.worker = None
        self.on_change = None
        self.revision = 0
        self.epoch = 0
        self.stream_id = None
        self.sequence = -1
        self.timestamp_us = -1
        self.connected = False
        self.event = 'waiting'
        self.error = None
        self.frames = {}
        self.side_epochs = {'left': 0, 'right': 0}
        self.hint_status = 'waiting_for_device'
        self.hint_error = None
        self.device_id = None

    def _notify(self):
        if self.on_change:
            self.on_change()

    def _current(self, worker):
        return self.worker is worker and not worker['stop'].is_set()

    def _clear(self, worker, event, error=None, *, connected=False, restart=False, generation=None):
        with self.lock:
            if (not self._current(worker) or
                    (generation is not None and generation is not worker.get('generation'))):
                return
            self.frames.clear()
            self.connected, self.event, self.error = connected, event, error
            self.revision += 1
            self.epoch += 1
            self.side_epochs = dict.fromkeys(('left', 'right'), self.epoch)
            if restart:
                worker['restart'].set()
        self._notify()

    def _tracking(self, worker, sdk, event, generation):
        try:
            sequence = _integer(event.tracking_frame_id, '帧序号')
            timestamp = _integer(event.timestamp, '时间戳')
            native_now = _integer(sdk.get_now(), 'SDK 时间')
            # LeapGetNow and the tracking timestamp share the native microsecond
            # clock. Include service queue delay; reception alone is not age.
            if timestamp > native_now:
                raise ValueError('Leap Motion 帧时间戳晚于 SDK 时钟')
            age = (native_now - timestamp) / 1000
            with self.lock:
                if (not self._current(worker) or worker['restart'].is_set()
                        or generation is not worker.get('generation')):
                    return
                if sequence <= self.sequence or timestamp <= self.timestamp_us:
                    return  # Duplicate/out-of-order callbacks cannot refresh age.
                stream_id = self.stream_id
            frames = {}
            for hand in event.hands:
                side, raw, metric = hand_frame(hand, sequence=sequence,
                    timestamp_us=timestamp, stream_id=stream_id)
                if side in frames:
                    raise ValueError('Leap Motion 同一帧出现重复手别')
                frames[side] = (raw, metric)
            with self.lock:
                if (not self._current(worker) or worker['restart'].is_set()
                        or generation is not worker.get('generation')
                        or stream_id != self.stream_id or sequence <= self.sequence):
                    return
                now = self.clock()
                worker['last_tracking_at'] = now
                for side in ('left', 'right'):
                    previous = self.frames.get(side)
                    item = frames.get(side)
                    # Loss, identity replacement and a gap beyond expiry reset
                    # downstream filters; ordinary consecutive frames do not.
                    if ((previous is not None and item is None) or
                            (item is not None and (previous is None or
                             (now - previous['observed_at']) * 1000 >= STALE_AFTER_MS or
                             previous['raw']['hand_id'] != item[0]['hand_id']))):
                        self.epoch += 1
                        self.side_epochs[side] = self.epoch
                self.frames = {side: dict(raw=raw, hand=metric, observed_at=now-age/1000)
                               for side, (raw, metric) in frames.items()}
                self.sequence, self.timestamp_us = sequence, timestamp
                self.connected, self.event, self.error = True, 'tracking', None
                self.revision += 1
        except Exception as exc:
            self._clear(worker, 'invalid_frame', str(exc), connected=True, generation=generation)
            return
        self._notify()

    def _listener(self, sdk, worker, generation):
        owner = self
        # The official package only exports LeapError at its top level; the
        # concrete timeout class lives in leap.exceptions.
        timeout_type = getattr(getattr(sdk, 'exceptions', None), 'LeapTimeoutError', ())

        class Listener(sdk.Listener):
            def on_connection_event(self, event):
                owner._clear(worker, 'connected', connected=True, generation=generation)

            def on_connection_lost_event(self, event):
                owner._clear(worker, 'disconnected', restart=True, generation=generation)

            def on_device_lost_event(self, event):
                owner._clear(worker, 'device_lost', restart=True, generation=generation)

            def on_device_failure_event(self, event):
                owner._clear(worker, 'device_error', 'Ultraleap 设备报告故障', restart=True, generation=generation)

            def on_tracking_event(self, event):
                owner._tracking(worker, sdk, event, generation)

            def on_error(self, error):
                # The official poll loop reports normal idle timeouts here.
                if isinstance(error, timeout_type):
                    return
                owner._clear(worker, 'sdk_error', str(error), restart=True, generation=generation)

        return Listener()

    def _check_frame_timeout(self, worker, connected_at):
        # Empty-hand TrackingEvents are healthy. Do not repeatedly reopen a
        # camera whose service is stalled. Notify outside the source lock.
        with self.lock:
            last_at = worker.get('last_tracking_at')
            if last_at is None:
                last_at = connected_at
            if (not self._current(worker)
                    or self.clock() - last_at <= FRAME_TIMEOUT_SECONDS
                    or self.event == 'frame_timeout'):
                return
            self.frames.clear()
            self.connected, self.event = True, 'frame_timeout'
            self.error = 'Ultraleap 服务已连接，但超过 5 秒没有追踪帧；请检查设备/服务'
            self.revision += 1
            self.epoch += 1
            self.side_epochs = dict.fromkeys(('left', 'right'), self.epoch)
        self._notify()

    def _configure_device(self, sdk, connection, device):
        """Submit preferences, never claim that the service adopted them."""
        try:
            strings = [sdk.ffi.new('char[]', hint.encode('ascii')) for hint in self.device_hints]
            # LeapC requires both NUL-terminated strings and a NULL array sentinel.
            # Keep the allocated strings alive until the native call returns.
            hints = sdk.ffi.new('const char*[]', strings + [sdk.ffi.NULL])
            result = sdk.libleapc.LeapSetDeviceHints(
                connection.get_connection_ptr(), device.c_data_device, hints)
            if result != 0:
                raise RuntimeError(f'LeapSetDeviceHints returned 0x{result:08x}')
        except Exception as exc:
            self.hint_status, self.hint_error = 'request_failed', str(exc)
            LOGGER.warning('Leap device hints request failed: %s', exc)
        else:
            self.hint_status, self.hint_error = 'submitted_unverified', None
            LOGGER.info('Leap device hints submitted (effective modes unverified): %s', self.device_hints)

    def _run(self, worker):
        while not worker['stop'].is_set():
            connection = listener = None
            devices = ExitStack()
            device = None
            try:
                try:
                    sdk = self.sdk_loader()
                except Exception as exc:
                    # leap.__init__ also raises a plain Exception when the
                    # native leapc_cffi directory is absent.
                    raise ImportError(str(exc)) from exc
                with self.lock:
                    if not self._current(worker):
                        return
                    self.stream_id = uuid.uuid4().hex
                    self.sequence = self.timestamp_us = -1
                    worker['generation'] = object()
                    worker['restart'].clear()
                    worker['last_tracking_at'] = None
                    self.hint_status, self.hint_error = 'waiting_for_device', None
                    self.device_id = None
                self._clear(worker, 'connecting')
                connection = sdk.Connection(poll_timeout=.1, multi_device_aware=True)
                listener = self._listener(sdk, worker, worker['generation'])
                connection.add_listener(listener)
                # Explicit finally is needed: the binding's open() context does
                # not disconnect when its initial connect() raises a timeout.
                connection.connect(timeout=1.)
                with self.lock:
                    # Connecting can outlive the last session. Serialize this
                    # side effect with close(), then let finally release the
                    # owned connection without configuring a cancelled reader.
                    if not self._current(worker) or worker['restart'].is_set():
                        continue
                    connection.set_tracking_mode(sdk.TrackingMode.HMD)
                connected_at = self.clock()
                while not worker['stop'].is_set() and not worker['restart'].wait(.1):
                    # A binding conversion exception can silently kill its
                    # poll thread without calling Listener.on_error.
                    poll_thread = getattr(connection, '_poll_thread', None)
                    if poll_thread is not None and not poll_thread.is_alive():
                        raise RuntimeError('Ultraleap SDK 采集线程已退出，正在重连')
                    if device is None:
                        # Select one camera; merging two cameras would mix frame
                        # clocks and skeleton identities. Discover late arrivals
                        # here rather than blocking the SDK callback thread.
                        available = connection.get_devices()
                        if available:
                            with self.lock:
                                if not self._current(worker) or worker['restart'].is_set():
                                    continue
                                device = devices.enter_context(available[0].open())
                                self.device_id = device.id
                                connection.subscribe_events(device)
                                connection.set_primary_device(device)
                                # Reapply HMD to the selected primary camera before
                                # submitting its hints, including after hot-plug.
                                connection.set_tracking_mode(sdk.TrackingMode.HMD)
                                self._configure_device(sdk, connection, device)
                                self.revision += 1
                            self._notify()
                    self._check_frame_timeout(worker, connected_at)
            except (ImportError, OSError) as exc:
                self._clear(worker, 'sdk_unavailable',
                    '请安装 Ultraleap Linux Hand Tracking Service、LeapC 和官方 '
                    f'leapc-python-bindings（Python 模块 leap）：{exc}')
            except Exception as exc:
                self._clear(worker, 'sdk_error', str(exc))
            finally:
                if connection is not None:
                    while True:
                        try:
                            connection.disconnect()
                            break
                        except Exception as exc:
                            self._clear(worker, 'sdk_error', f'Ultraleap 连接关闭失败：{exc}')
                            # Keep this resource owner until cleanup succeeds;
                            # even a rapid new session must not overlap readers.
                            time.sleep(max(.01, self.retry_delay))
                    if listener is not None:
                        try:
                            connection.remove_listener(listener)
                        except Exception as exc:
                            self._clear(worker, 'sdk_error', f'Ultraleap 监听器清理失败：{exc}')
                # Polling is stopped before closing the device handle, including
                # failed setup, disconnects, and cancelled sessions.
                devices.close()
            if worker['stop'].wait(self.retry_delay):
                return

    def _start(self):
        worker = self.worker
        if worker is None or not worker['thread'].is_alive():
            self.frames.clear()
            self.connected, self.event = False, 'connecting'
            worker = dict(stop=threading.Event(), restart=threading.Event())
            worker['thread'] = threading.Thread(target=self._run, args=(worker,),
                                                name='leapmotion-input', daemon=True)
            self.worker = worker
            worker['thread'].start()

    def _snapshot(self, side):
        item = self.frames.get(side)
        age = None if item is None else max(0., (self.clock()-item['observed_at'])*1000)
        stopped = self.worker is not None and self.worker['stop'].is_set()
        fresh = not stopped and self.connected and age is not None and age < STALE_AFTER_MS
        metadata = dict(source='leapmotion', source_joint_set='leapc-mediapipe-21',
            source_joint_count=21, native_representation='leapc-bones',
            joint_mapping=list(JOINT_MAPPING), units='m', native_units='mm',
            coordinate_space='leap_device', output_coordinate_space='leap_wrist_relative',
            normalized_is_image=False, stream_id=self.stream_id,
            source_epoch=self.side_epochs[side], sequence=self.sequence,
            age_ms=age, stale_after_ms=STALE_AFTER_MS, tracking=fresh, stale=not fresh,
            event='stopping' if stopped else ('tracking_lost' if self.event == 'tracking' and item is None else self.event),
            error=self.error, transport='leapc', tracking_mode=TRACKING_MODE)
        metadata['raw_leapmotion'] = deepcopy(item['raw']) if item else None
        metadata['device_hints'] = dict(requested=list(self.device_hints),
            status='inactive' if stopped or not self.connected else self.hint_status,
            error=self.hint_error, effective=None, device_id=self.device_id)
        return dict(source='leapmotion', revision=self.revision, age_ms=age,
            hand=deepcopy(item['hand']) if fresh else None, tracking=fresh,
            connected=self.connected and not stopped, source_metadata=metadata,
            raw_leapmotion=deepcopy(item['raw']) if item else None)

    def latest(self, side):
        with self.lock:
            self._start()
            return self._snapshot(side)

    def commit_if_current(self, side, snapshot, action):
        with self.lock:
            if self.worker is None or self.worker['stop'].is_set():
                return False
            metadata = snapshot.get('source_metadata', {})
            if (metadata.get('source_epoch') != self.side_epochs[side]
                    or metadata.get('stream_id') != self.stream_id):
                return False
            current = self._snapshot(side)
            if (snapshot.get('hand') is not None) != current['tracking']:
                return False
            action()
            return True

    def close(self, *, wait=True):
        with self.lock:
            worker = self.worker
            if worker is None:
                return
            worker['stop'].set()
            worker['restart'].set()
            self.frames.clear()
            self.connected, self.event = False, 'stopped'
            self.revision += 1
            self.epoch += 1
            self.side_epochs = dict.fromkeys(('left', 'right'), self.epoch)
        self._notify()
        if wait and worker['thread'] is not threading.current_thread():
            worker['thread'].join(timeout=1.5)
