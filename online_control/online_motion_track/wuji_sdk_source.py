"""Read-only, cancellable Wuji SDK skeleton acquisition shared by both transports."""
from copy import deepcopy
import importlib
import math
import os
import threading
import time
from uuid import uuid4

SCHEMA = "tsensei.wuji_glove.skeleton.v1"
STALE_AFTER_MS = 350


class SkeletonStore:
    def __init__(self, clock=time.monotonic, on_change=None):
        self.clock = clock
        self.lock = threading.Lock()
        self.frames = {}
        self.last_sequences = {}
        self.state, self.error = "waiting", None
        self.generation = 0
        self.source_epoch = 0
        self.errors = 0
        self.pending_cleanup = []
        self.on_change = on_change

    def changed(self):
        if self.on_change:
            self.on_change()

    def clear(self, state, error=None, *, invalidate=False):
        with self.lock:
            if invalidate:
                self.generation += 1
                self.last_sequences.clear()
            self.frames.clear()
            self.source_epoch += 1
            self.state, self.error = state, error
            self.errors += int(error is not None)
            generation = self.generation
        self.changed()
        return generation

    def offer(self, skeleton, *, side, serial, stream_id, generation=None):
        positions = [[float(v) for v in j.pose.position] for j in skeleton.joints]
        confidence = [float(j.confidence) for j in skeleton.joints]
        if (side not in ("left", "right") or len(positions) != 21 or len(confidence) != 21
                or any(len(p) != 3 for p in positions)
                or any(not math.isfinite(v) or abs(v) > 10 for p in positions for v in p)
                or any(not math.isfinite(v) or not 0 <= v <= 1 for v in confidence)):
            raise ValueError("invalid SDK skeleton")
        frame_id = str(skeleton.header.frame_id)
        if frame_id != ("l_wrist" if side == "left" else "r_wrist"):
            raise ValueError("unexpected SDK wrist frame: " + frame_id)
        sequence = skeleton.header.seq
        if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
            raise ValueError("invalid SDK skeleton sequence")
        raw = {"schema": SCHEMA, "stream_id": stream_id, "serial_number": serial,
               "sequence": sequence, "timestamp_us": int(skeleton.header.timestamp_us),
               "side": side, "frame_id": frame_id, "units": "m",
               "positions_xyz": positions, "confidences": confidence}
        with self.lock:
            if generation is not None and generation != self.generation:
                return
            key = (side, stream_id)
            if sequence <= self.last_sequences.get(key, -1):
                return
            self.last_sequences[key] = sequence
            previous = self.frames.get(side)
            if previous and (previous[0]['stream_id'] != stream_id or
                             (self.clock()-previous[1])*1000 >= STALE_AFTER_MS):
                self.source_epoch += 1
            self.frames[side] = (raw, self.clock())
            self.state, self.error = "receiving", None
        self.changed()

    def callback_error(self, error, generation):
        with self.lock:
            if generation == self.generation:
                self.frames.clear()
                self.source_epoch += 1
                self.state, self.error = "invalid_frame", str(error)
                self.errors += 1
        self.changed()

    def latest(self, side):
        with self.lock:
            item = self.frames.get(side)
            age = max(0., (self.clock() - item[1]) * 1000) if item else None
            return {"schema": SCHEMA, "state": self.state, "error": self.error,
                    "side": side, "age_ms": age, "frame": deepcopy(item[0]) if item else None,
                    "connected": item is not None and age < STALE_AFTER_MS,
                    "transport": {"mode": "sdk", "request_ms": 0., "cache_ms": age,
                                  "generation": self.generation,
                                  "source_epoch": self.source_epoch,
                                  "errors": self.errors, "last_error": self.error}}


def cleanup_resources(store, resources=None):
    """Retry only owned resources and retain failures so they cannot be forgotten."""
    with store.lock:
        pending = list(store.pending_cleanup if resources is None else resources)
    errors = []
    for _ in range(3):
        failed, errors = [], []
        for label, close in pending:
            try:
                close()
            except Exception as exc:
                failed.append((label, close))
                errors.append(label + ': ' + str(exc))
        pending = failed
        if not pending:
            break
    with store.lock:
        store.pending_cleanup = pending
    if pending:
        store.clear('disconnect_error', '; '.join(errors), invalidate=True)
    return not pending


def acquire(store, stopped, address=None, serial=None, *, sdk_loader=None,
            poll_interval=.1, reconnect_after=3., first_frame_timeout=10.,
            retry_delay=1., max_retry_delay=10.):
    """Subscribe only while requested; never publish commands or disconnect other owners.

    Native scan/connect calls cannot be interrupted by Python. Cancellation is
    checked after every such call, before another SDK operation or reconnect.
    A connection returning after cancellation is immediately closed by its owner.
    """
    try:
        os.environ.setdefault('WUJI_TELEMETRY_DISABLE', '1')
        sdk = (sdk_loader or (lambda: importlib.import_module("wuji_sdk")))()
        manager = sdk.SdkManager.instance()
        options = sdk.ConnectOptions(timeout_ms=1000, retry_count=1, enable_bridge=False)
    except Exception as exc:
        store.clear("stopped" if stopped.is_set() else "sdk_error", str(exc), invalidate=True)
        return
    backoff = retry_delay
    while not stopped.is_set():
        subscriptions, owned, sides = [], [], []
        generation = store.clear("connecting" if address or serial else "discovering", invalidate=True)
        try:
            if stopped.is_set():
                break
            if address or serial:
                locators = [{"address": address} if address else {"sn": serial}]
            else:
                devices = manager.scan()
                if stopped.is_set():
                    break
                locators = [{"sn": d.sn} for d in devices if d.device_type == sdk.DeviceType.WujiGlove]
            if not locators:
                raise RuntimeError("未发现 Wuji 手套，请检查电源、以太网和 WSL 网络，或指定 --wuji-address")
            for locator in locators:
                if stopped.is_set():
                    break
                name = "hand_track_glove_" + uuid4().hex
                glove = manager.connect(**locator, device_name=name, options=options)
                owned.append((name, glove))
                if stopped.is_set():
                    break
                actual_serial = str(glove.serial_number)
                if serial and actual_serial != serial:
                    raise ValueError("连接的手套序列号与 --wuji-serial 不符")
                side_value = str(glove.hand_side().get()).lower()
                if stopped.is_set():
                    break
                side = "left" if "left" in side_value else "right" if "right" in side_value else None
                if side is None or side in sides:
                    raise ValueError("手套手别无效或同侧有多台设备，请指定 --wuji-serial / --wuji-address")
                sides.append(side)

                def callback(frame, side=side, sn=actual_serial, stream=uuid4().hex, generation=generation):
                    if stopped.is_set():
                        return
                    try:
                        store.offer(frame, side=side, serial=sn, stream_id=stream, generation=generation)
                    except Exception as exc:
                        store.callback_error(exc, generation)

                subscriptions.append(glove.hand_skeleton().subscribe_with_callback(callback))
            began = store.clock()
            while not stopped.wait(poll_interval):
                if any(not glove.is_connected for _, glove in owned):
                    raise RuntimeError("Wuji 手套连接已断开")
                samples = [store.latest(side) for side in sides]
                if samples and all(s["connected"] for s in samples):
                    backoff = retry_delay
                if any((s["age_ms"] is None and store.clock() - began > first_frame_timeout)
                       or (s["age_ms"] is not None and s["age_ms"] > reconnect_after * 1000)
                       for s in samples):
                    raise RuntimeError("Wuji 手套骨架流超时，正在重新连接")
        except Exception as exc:
            store.clear("stopped" if stopped.is_set() else "disconnected", str(exc), invalidate=True)
        finally:
            # Reject callbacks already queued by an old native subscription.
            with store.lock:
                store.generation += 1
                store.frames.clear()
            resources = [('subscription', sub.close) for sub in subscriptions]
            resources += [(name, lambda name=name: manager.disconnect(device_name=name))
                          for name, _ in owned]
            cleaned = cleanup_resources(store, resources)
            if stopped.is_set() and cleaned:
                store.clear("stopped", invalidate=True)
        if not cleaned:
            # Leave aliases/closers reachable and the failure visible. A later
            # close() may retry cleanup; never open a second owner meanwhile.
            return
        if stopped.wait(backoff):
            break
        backoff = min(backoff * 2, max_retry_delay)
