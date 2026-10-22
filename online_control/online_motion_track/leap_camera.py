"""Latest-only LeapC infrared images for the browser's MediaPipe detector.

The native reader copies camera 0 before LeapC can reuse its buffer. Capture runs
independently of HTTP, display refresh and inference; no synthetic frames or Qt.
"""
from collections import deque
import json
import struct
import threading

from .leapmotion_source import LeapMotionSource, STALE_AFTER_MS

TARGET_FPS = 120


def encode_frame(metadata, pixels):
    """Little-endian JSON length, UTF-8 metadata, then width*height gray8 bytes."""
    header = json.dumps(metadata, ensure_ascii=False, allow_nan=False).encode('utf-8')
    return struct.pack('<I', len(header)) + header + pixels


class LeapCameraSource(LeapMotionSource):
    device_hints = ('fast_hand_motion', 'high_hand_fidelity')

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.image_changed = threading.Condition(self.lock)
        self.image = None
        self.image_times = deque(maxlen=241)

    def _notify(self):
        with self.image_changed:
            self.image_changed.notify_all()

    def _tracking(self, worker, sdk, event, generation):
        # No Leap skeleton enters this input. All landmarks come from MediaPipe.
        pass

    def _listener(self, sdk, worker, generation):
        parent = super()._listener(sdk, worker, generation)
        owner = self

        class Listener(type(parent)):
            def on_image_event(self, event):
                owner._image_event(worker, sdk, event, generation)

        return Listener()

    def _configure_device(self, sdk, connection, device):
        # The base reader calls this under its lock. Use the nonblocking native
        # call; the Python set_policy_flags waits for a callback and deadlocks.
        result = sdk.libleapc.LeapSetPolicyFlagsEx(connection.get_connection_ptr(),
            device.c_data_device, sdk.libleapc.eLeapPolicyFlag_Images, 0)
        if result != 0:
            raise RuntimeError(f'Leap 图像订阅失败：0x{result:08x}')
        super()._configure_device(sdk, connection, device)

    def _image_event(self, worker, sdk, event, generation):
        try:
            with self.lock:
                if (not self._current(worker) or worker['restart'].is_set()
                        or generation is not worker.get('generation')):
                    return
                stream_id = self.stream_id
            data = event.image[0].c_data
            width, height, bpp = (int(data.properties.width),
                                  int(data.properties.height), int(data.properties.bpp))
            if bpp != 1 or not (1 <= width <= 2048 and 1 <= height <= 2048):
                raise ValueError('Leap 相机需要有效的单通道 8 位红外图像')
            frame_id, timestamp = int(event.c_data.info.frame_id), int(event.c_data.info.timestamp)
            age_ms = (int(sdk.get_now()) - timestamp) / 1000
            if frame_id < 0 or timestamp < 0 or age_ms < 0 or int(data.offset) < 0:
                raise ValueError('Leap 图像时间戳或缓冲区无效')
            if data.data == sdk.ffi.NULL:
                raise ValueError('Leap 图像缓冲区为空')
            pixels = bytes(sdk.ffi.buffer(sdk.ffi.cast('char *', data.data) + data.offset, width * height))
            with self.lock:
                if (not self._current(worker) or worker['restart'].is_set()
                        or generation is not worker.get('generation') or stream_id != self.stream_id):
                    return
                if frame_id <= self.sequence or timestamp <= self.timestamp_us:
                    return
                if self.image is None or self.image['stream_id'] != stream_id:
                    self.image_times.clear()
                self.image_times.append(timestamp)
                now = self.clock()
                self.sequence, self.timestamp_us = frame_id, timestamp
                self.connected, self.event, self.error = True, 'images', None
                worker['last_tracking_at'] = now
                self.revision += 1
                self.image = dict(stream_id=stream_id, frame_id=frame_id, timestamp_us=timestamp,
                    width=width, height=height, pixels=pixels, observed_at=now-age_ms/1000,
                    revision=self.revision)
            self._notify()
        except Exception as exc:
            self._clear(worker, 'image_error', str(exc), connected=True, generation=generation)

    def start(self):
        with self.lock:
            self._start()

    def read_image(self, after_revision=-1, timeout=.25, *, start=True):
        if type(after_revision) is not int or after_revision < -1:
            raise ValueError('图像 revision 必须为 -1 或非负整数')
        with self.image_changed:
            if start:
                self._start()
            if self.revision == after_revision:
                self.image_changed.wait(timeout)
            stopped = self.worker is None or self.worker['stop'].is_set()
            item = self.image
            current = item is not None and item['stream_id'] == self.stream_id
            age = max(0., (self.clock()-item['observed_at'])*1000) if current else None
            fresh = (not stopped and self.connected and self.event == 'images'
                     and age is not None and age < STALE_AFTER_MS)
            times = self.image_times if current else ()
            duration = (times[-1]-times[0])/1e6 if len(times) > 1 else 0.
            fps = (len(times)-1)/duration if duration > 0 else None
            measured = duration >= 1.
            status = ('ok' if fps >= TARGET_FPS*.98 else 'below_target') if measured and fresh else 'measuring'
            meta = dict(source='leapmotion_camera', camera_index=0, format='gray8',
                revision=self.revision, stream_id=self.stream_id, frame_id=self.sequence,
                timestamp_us=self.timestamp_us, target_fps=TARGET_FPS,
                capture_fps=round(fps, 2) if measured else None, fps_status=status,
                age_ms=round(age, 2) if age is not None else None, stale_after_ms=STALE_AFTER_MS,
                connected=self.connected and not stopped, event=self.event, error=self.error,
                width=item['width'] if current else 0, height=item['height'] if current else 0,
                device_id=self.device_id, hints=dict(requested=list(self.device_hints),
                    status=self.hint_status, error=self.hint_error), has_frame=False)
            pixels = b''
            if fresh and item['revision'] > after_revision:
                meta['has_frame'] = True
                pixels = item['pixels']
            return meta, pixels
