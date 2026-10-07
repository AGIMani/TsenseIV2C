"""Session-bound Leap Motion 2 input. Browser requests never supply its skeleton."""
from copy import deepcopy
import threading
import time

from .leapmotion_source import LeapMotionSource, STALE_AFTER_MS


class LeapMotionInputRelay:
    stale_after_ms = STALE_AFTER_MS
    supports_background = True

    def __init__(self, *, clock=time.monotonic, sdk_loader=None, retry_delay=1.):
        self.clock = clock
        self.lock = threading.RLock()
        self.channels = {}
        self.listeners = {}
        self.client = LeapMotionSource(clock=clock, sdk_loader=sdk_loader, retry_delay=retry_delay)
        self.client.on_change = self._notify

    def _notify(self):
        with self.lock:
            callbacks = list(self.listeners.values())
        for callback in callbacks:
            callback()

    def subscribe(self, session, callback):
        with self.lock:
            self.listeners[session] = callback
        def unsubscribe():
            with self.lock:
                if self.listeners.get(session) is callback:
                    self.listeners.pop(session, None)
        return unsubscribe

    def create(self, session, side):
        if side not in ('left', 'right'):
            raise ValueError('Leap Motion 手别必须为 left 或 right')
        with self.lock:
            self.channels[session] = dict(side=side, consumed=-1, snapshots={})
        return dict(source='leapmotion', transport='leapc', tracking_mode='desktop',
                    stale_after_ms=self.stale_after_ms)

    def close(self, session):
        with self.lock:
            self.channels.pop(session, None)
            self.listeners.pop(session, None)
            if not self.channels:
                # The SDK callback may be waiting for this lock. Join only in
                # explicit process teardown/tests, never while closing a relay.
                self.client.close(wait=False)

    def _channel(self, session):
        channel = self.channels.get(session)
        if channel is None:
            raise ValueError('请先创建 Leap Motion 输入会话')
        return channel

    def latest(self, session):
        with self.lock:
            channel = self._channel(session)
            result = self.client.latest(channel['side'])
            saved = channel['snapshots']
            if result['hand'] is None:
                saved.clear()
            saved[result['revision']] = (deepcopy(result), self.clock())
            while len(saved) > 4:
                del saved[next(iter(saved))]
            return result

    def consume(self, session, revision):
        with self.lock:
            channel = self._channel(session)
            current = self.client.latest(channel['side'])
            item = channel['snapshots'].get(revision) if type(revision) is int else None
            valid = item is not None and revision > channel['consumed']
            if valid:
                result = deepcopy(item[0])
                channel['consumed'] = revision
                age = result['age_ms']
                if age is not None:
                    age += max(0., self.clock()-item[1])*1000
                meta, current_meta = result['source_metadata'], current['source_metadata']
                valid = (meta['source_epoch'] == current_meta['source_epoch'] and
                         meta['stream_id'] == current_meta['stream_id'] and
                         age is not None and age < self.stale_after_ms and current['tracking'])
                result['age_ms'] = age
                meta['age_ms'] = age
            else:
                result = current
            if not valid:
                result.update(hand=None, tracking=False)
                result['source_metadata'].update(tracking=False, stale=True,
                    discard_reason='obsolete_or_repeated_revision')
            return result

    def commit_if_current(self, session, snapshot, action):
        with self.lock:
            channel = self.channels.get(session)
            return channel is not None and self.client.commit_if_current(channel['side'], snapshot, action)
