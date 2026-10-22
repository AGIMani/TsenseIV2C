"""Session-bound, latest-only Quest input using separate 25/21-point protocols.

The WebXR endpoint only accepts native 25 joints; ADB input explicitly supplies
21 points.  The server derives the compatibility view itself; its ``normalized``
slot contains metric points and must never be treated as image coordinates.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import dataclass, field
import json
import math
import secrets
import threading
import time
from typing import Callable

# main.py adds src/tracking to sys.path.  A standalone import deliberately avoids
# tracking.__init__, which loads optional camera inference backends.
from webxr import (
    WEBXR_25_TO_MEDIAPIPE_21, WEBXR_PROTOCOL_SCHEMA, WEBXR_STALE_TIMEOUT_S,
    WebXRHandFrame, parse_webxr_message,
)
from .quest_adb_input import (
    QUEST_ADB_JOINT_SET, QUEST_ADB_PROTOCOL_SCHEMA, QuestADBHandFrame,
    parse_quest_adb_message,
)


def metric_hand(frame: WebXRHandFrame | QuestADBHandFrame) -> dict | None:
    """Produce 21 metric points, retaining WebXR axes and metres.

    Downstream pipelines already perform the hand-side MANO wrist alignment.
    Subtracting the wrist here removes large room offsets before filtering;
    applying an axis reflection here would change hand chirality.
    """
    landmarks = frame.mediapipe_landmarks()
    if not landmarks:
        return None
    wrist = landmarks[0]
    world = [[point[i] - wrist[i] for i in range(3)] for point in landmarks]
    if any(not math.isfinite(v) or abs(v) > 10 for point in world for v in point):
        raise ValueError("WebXR 手部相对坐标超出有限的 10 米范围")
    return {"world": world, "normalized": deepcopy(world), "label": frame.hand_side.title(),
            "source": "webxr", "coordinate_space": "webxr_wrist_relative",
            "normalized_is_image": False}


@dataclass
class _Channel:
    token: str
    side: str
    revision: int = 0
    consumed_revision: int = -1
    stream_id: str | None = None
    protocol_schema: str = WEBXR_PROTOCOL_SCHEMA
    retired_streams: set[str] = field(default_factory=set)
    sequence: int = -1
    ended: bool = False
    received_at: float | None = None
    observation_at: float | None = None
    event: str = "waiting"
    hand: dict | None = None
    raw: dict | None = None
    polled_snapshot: dict | None = None
    polled_at: float | None = None
    minimum_clock_offset_ms: float | None = None
    last_client_perf_ms: float | None = None
    transport_delay_ms: float | None = None
    discard_reason: str | None = None
    source_epoch: int = 0


class WebXRInputRelay:
    """One bounded observation slot per existing control/preview session."""
    stale_after_ms = int(WEBXR_STALE_TIMEOUT_S * 1000)
    pairing_code = "123456"

    def __init__(self, *, clock: Callable[[], float] = time.monotonic):
        self.clock = clock
        self.lock = threading.Lock()
        self.channels: dict[str, _Channel] = {}
        self.tokens: dict[str, str] = {}
        self.pair_attempts: dict[str, list[float]] = {}
        self.global_pair_attempts: list[float] = []
        self.listeners = {}

    def subscribe(self, session, callback):
        with self.lock:
            self.listeners[session] = callback
        def unsubscribe():
            with self.lock:
                if self.listeners.get(session) is callback:
                    self.listeners.pop(session, None)
        return unsubscribe

    def commit_if_current(self, session, snapshot, action):
        with self.lock:
            channel = self.channels.get(session)
            if channel is None:
                return False
            expected = snapshot.get('source_metadata', {}).get('stream_id')
            if channel.stream_id != expected:
                return False
            if channel.source_epoch != snapshot.get('source_metadata', {}).get('source_epoch'):
                return False
            fresh = (channel.hand is not None and not channel.ended and
                channel.observation_at is not None and
                (self.clock()-channel.observation_at)*1000 < self.stale_after_ms)
            if (snapshot.get('hand') is not None) != fresh:
                return False
            action()
            return True

    def _pairing(self) -> dict:
        code = self.pairing_code
        return {"pairing_code": code, "pairing_expires_in_s": None,
                "entry_path": "/q", "capture_path": f"/q#code={code}"}

    def renew_code(self, session: str) -> dict:
        with self.lock:
            channel = self.channels.get(session)
            if channel is None:
                raise ValueError("请先在电脑开始 Quest 识别")
            return self._pairing()

    def pair(self, code, peer: str) -> dict:
        """The permanent human code resolves the single active Quest session."""
        with self.lock:
            now = self.clock()
            self.pair_attempts = {key: [t for t in times if now-t < 60]
                                  for key, times in self.pair_attempts.items() if times and now-times[-1] < 60}
            self.global_pair_attempts = [t for t in self.global_pair_attempts if now-t < 60]
            attempts = self.pair_attempts.get(peer, [])
            if len(attempts) >= 10 or len(self.global_pair_attempts) >= 100:
                raise ValueError("配对尝试过于频繁，请一分钟后重试")
            self.pair_attempts[peer] = [*attempts, now]
            self.global_pair_attempts.append(now)
            if not isinstance(code, str) or len(code) != 6 or not code.isascii() or not code.isdigit():
                raise ValueError("请输入电脑上显示的 6 位数字码")
            if code != self.pairing_code:
                raise ValueError("配对码无效，请使用固定配对码 123456")
            if not self.channels:
                raise ValueError("请先在电脑选择 Quest 输入并开始识别，再点击连接电脑")
            if len(self.channels) != 1:
                raise ValueError("有多个 Quest 控制会话，请在电脑停止多余会话后重新连接")
            channel = next(iter(self.channels.values()))
            return {"token": channel.token}

    def create(self, session: str, side: str) -> dict:
        if side not in ("left", "right"):
            raise ValueError("WebXR 手别必须为 left 或 right")
        with self.lock:
            previous = self.channels.pop(session, None)
            if previous:
                self.tokens.pop(previous.token, None)
            token = secrets.token_urlsafe(32)
            self.channels[session] = _Channel(token, side)
            self.tokens[token] = session
            pairing = self._pairing()
        return {"token": token, **pairing,
                "stale_after_ms": self.stale_after_ms, "source": "webxr"}

    def close(self, session: str) -> None:
        with self.lock:
            channel = self.channels.pop(session, None)
            self.listeners.pop(session, None)
            if channel:
                self.tokens.pop(channel.token, None)

    def publish(self, token: str, message: dict) -> dict:
        """Accept only the unchanged native 25-joint WebXR protocol."""
        return self._publish(token, message, WEBXR_PROTOCOL_SCHEMA, parse_webxr_message)

    def publish_quest_adb(self, token: str, message: dict) -> dict:
        """Accept native 21-point input through the same bounded relay slot."""
        return self._publish(token, message, QUEST_ADB_PROTOCOL_SCHEMA, parse_quest_adb_message)

    def _publish(self, token: str, message: dict, schema: str, parser) -> dict:
        if not isinstance(message, dict) or message.get("schema") != schema:
            raise ValueError(f"Quest schema 必须为 {schema}")
        stream_id, sequence = message.get("stream_id"), message.get("sequence")
        if not isinstance(stream_id, str) or not 1 <= len(stream_id) <= 128:
            raise ValueError("WebXR stream_id 必须为 1..128 字符")
        if isinstance(sequence, bool) or not isinstance(sequence, int) or not 0 <= sequence <= 2**53 - 1:
            raise ValueError("WebXR sequence 必须为非负安全整数")
        # Raw metadata is retained too; reject non-JSON NaN/Infinity in optional
        # fields so one malformed sample cannot poison later HTTP responses.
        json.dumps(message, allow_nan=False)
        # Validate before changing the latest slot, including native joint names,
        # completeness, finite coordinates, handedness and duplicate-hand checks.
        parsed = parser(message)
        client_perf = message.get("client_perf_ms")
        if client_perf is not None and (isinstance(client_perf, bool)
                or not isinstance(client_perf, (int, float)) or not math.isfinite(client_perf) or client_perf < 0):
            raise ValueError("WebXR client_perf_ms 必须为有限非负单调时间")
        if parsed.type == "hand_skeleton" and client_perf is None:
            raise ValueError("WebXR 骨架必须带有 client_perf_ms")
        with self.lock:
            session = self.tokens.get(token) if isinstance(token, str) else None
            channel = self.channels.get(session)
            if channel is None:
                raise ValueError("WebXR 配对已失效，请在控制页面重新创建")
            reason = None
            new_stream = stream_id != channel.stream_id
            if not new_stream and schema != channel.protocol_schema:
                raise ValueError("同一 Quest stream_id 不能切换 25 点与 21 点协议，请创建新的 stream_id")
            if stream_id in channel.retired_streams:
                reason = "retired_stream"
            elif new_stream:
                if parsed.type not in ("client_hello", "session_started"):
                    raise ValueError("新的 WebXR stream 必须先发送 client_hello 或 session_started")
                if len(channel.retired_streams) >= 128:
                    raise ValueError("WebXR 重启次数过多，请重新创建配对")
            elif sequence <= channel.sequence:
                reason = "stale_sequence"
            elif channel.ended:
                reason = "stream_ended"
            if reason:
                return {"ok": True, "accepted": False, "reason": reason, "revision": channel.revision}

            chosen = next((frame for frame in parsed.frames if frame.hand_side == channel.side), None)
            hand = metric_hand(chosen) if chosen is not None else None
            if new_stream:
                channel.source_epoch += 1
                if channel.stream_id is not None:
                    channel.retired_streams.add(channel.stream_id)
                channel.stream_id = stream_id
                channel.protocol_schema = schema
                channel.hand = None
                channel.ended = False
                channel.polled_snapshot = None
                channel.minimum_clock_offset_ms = None
                channel.last_client_perf_ms = None
            now = self.clock()
            clock_regressed = (client_perf is not None and channel.last_client_perf_ms is not None
                               and client_perf < channel.last_client_perf_ms)
            transport_delay = None
            if client_perf is not None and not clock_regressed:
                offset = now * 1000 - client_perf
                baseline = channel.minimum_clock_offset_ms
                channel.minimum_clock_offset_ms = offset if baseline is None else min(baseline, offset)
                transport_delay = max(0., offset - channel.minimum_clock_offset_ms)
                channel.last_client_perf_ms = client_perf
            channel.sequence = sequence
            channel.received_at = now
            channel.revision += 1
            channel.event = parsed.type
            if parsed.type == "hand_skeleton":
                # Compare relative to the fastest observed delivery in this
                # stream; Quest and PC wall clocks need not agree. Absolute
                # first-message latency remains unknown without a clock sync.
                channel.discard_reason = ("client_clock_regressed" if clock_regressed else
                    "late_transport" if transport_delay >= self.stale_after_ms else None)
                if hand is None or channel.discard_reason is not None or (
                        channel.observation_at is not None and
                        now-channel.observation_at >= WEBXR_STALE_TIMEOUT_S):
                    channel.source_epoch += 1
                channel.hand = hand if channel.discard_reason is None else None
                channel.transport_delay_ms = transport_delay
                channel.observation_at = now - (transport_delay or 0.) / 1000
                channel.raw = deepcopy(message)
            elif parsed.type in ("client_hello", "session_started", "session_ended", "client_error") or (
                    parsed.type == "tracking_lost" and parsed.handedness in (None, channel.side)):
                channel.source_epoch += 1
                channel.hand = None
                channel.observation_at = now
                channel.raw = deepcopy(message)
                channel.transport_delay_ms = transport_delay
                channel.discard_reason = None
            if parsed.type in ("session_ended", "client_error"):
                channel.ended = True
            if channel.hand is None:
                channel.polled_snapshot = None
            result = {"ok": True, "accepted": True, "revision": channel.revision}
            notify = self.listeners.get(session)
        if notify:
            notify()
        return result

    def _snapshot(self, channel: _Channel) -> dict:
        now = self.clock()
        age = None if channel.observation_at is None else max(0, (now - channel.observation_at) * 1000)
        connected = (channel.received_at is not None and not channel.ended
                     and now - channel.received_at < WEBXR_STALE_TIMEOUT_S)
        fresh = age is not None and age < self.stale_after_ms
        hand = deepcopy(channel.hand) if connected and fresh else None
        raw = deepcopy(channel.raw)
        raw = raw if raw is not None else {}
        metadata = {"source": "webxr", "source_joint_set": "webxr-hand-25", "source_joint_count": 25,
                    "joint_mapping": list(WEBXR_25_TO_MEDIAPIPE_21), "units": "m",
                    "coordinate_space": raw.get("coordinate_space", "webxr_local_floor"),
                    "output_coordinate_space": "webxr_wrist_relative", "normalized_is_image": False,
                    "sequence": raw.get("sequence"), "stream_id": channel.stream_id,
                    "source_epoch": channel.source_epoch,
                    "timestamp_ms": raw.get("timestamp_ms"), "client_perf_ms": raw.get("client_perf_ms"),
                    "xr_predicted_display_time_ms": raw.get("xr_predicted_display_time_ms"),
                    "transport_delay_ms": channel.transport_delay_ms,
                    "age_ms": age, "stale_after_ms": self.stale_after_ms,
                    "stale": not fresh, "event": channel.event, "tracking": hand is not None}
        if channel.protocol_schema == QUEST_ADB_PROTOCOL_SCHEMA:
            metadata.update(transport="adb", source_joint_set=QUEST_ADB_JOINT_SET,
                            source_joint_count=21, joint_mapping=list(range(21)))
        if channel.discard_reason:
            metadata["discard_reason"] = channel.discard_reason
        return {"revision": channel.revision, "age_ms": age, "connected": connected,
                "tracking": hand is not None, "hand": hand, "source": "webxr",
                "source_metadata": metadata, "raw_webxr": raw or None}

    def latest(self, session: str) -> dict:
        with self.lock:
            channel = self.channels.get(session)
            if channel is None:
                raise ValueError("尚未创建 WebXR 配对")
            snapshot = self._snapshot(channel)
            # A new Quest sample can arrive between the PC's poll and frame
            # submission. Retain exactly the last polled sample until consumed.
            channel.polled_snapshot = deepcopy(snapshot)
            channel.polled_at = channel.observation_at
            return snapshot

    def consume(self, session: str, revision) -> dict:
        """Recheck freshness when a PC frame reaches the solver, exactly once."""
        with self.lock:
            channel = self.channels.get(session)
            if channel is None:
                raise ValueError("尚未创建 WebXR 配对")
            snapshot = self._snapshot(channel)
            polled = channel.polled_snapshot
            if (polled is not None and revision == polled["revision"]
                    and revision != channel.revision):
                snapshot = deepcopy(polled)
                age = None if channel.polled_at is None else max(0, (self.clock() - channel.polled_at) * 1000)
                fresh = age is not None and age < self.stale_after_ms and not channel.ended
                snapshot["age_ms"] = age
                snapshot["source_metadata"]["age_ms"] = age
                snapshot["source_metadata"]["stale"] = not fresh
                if not fresh:
                    snapshot["connected"] = False
                    snapshot["hand"] = None
                    snapshot["tracking"] = False
                    snapshot["source_metadata"]["tracking"] = False
            if (isinstance(revision, bool) or not isinstance(revision, int)
                    or revision != snapshot["revision"] or revision <= channel.consumed_revision):
                snapshot["hand"] = None
                snapshot["tracking"] = False
                snapshot["source_metadata"]["tracking"] = False
                snapshot["source_metadata"]["discard_reason"] = "obsolete_or_repeated_revision"
            else:
                channel.consumed_revision = revision
            return snapshot
