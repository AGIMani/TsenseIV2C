"""Session-bound local SDK reader; browsers cannot supply robot observations."""
from copy import deepcopy
import math
import os
import threading
import time
from urllib.parse import urlsplit
from .wuji_transport import LatestWujiClient, LatestWujiSdkClient

SCHEMA = "tsensei.wuji_glove.skeleton.v1"


def resolve_wuji_config(url=None, *, source=None, address=None, serial=None):
    # An explicitly supplied legacy URL opts into bridge mode. Merely retaining
    # WUJI_BRIDGE_URL in an old shell must not silently select a remote backend.
    source = source or ('bridge' if url else os.environ.get('WUJI_SOURCE', 'sdk'))
    if source not in ('sdk', 'bridge'):
        raise ValueError('Wuji 来源必须为 sdk 或 bridge')
    if source == 'sdk':
        if url:
            raise ValueError('--wuji-url 仅用于 --wuji-source bridge')
        return dict(source=source, url=None, address=address or os.environ.get('WUJI_ADDRESS'),
                    serial=serial or os.environ.get('WUJI_SERIAL'))
    if address or serial:
        raise ValueError('--wuji-address / --wuji-serial 仅用于 SDK 直连')
    url = (url or os.environ.get('WUJI_BRIDGE_URL') or '').rstrip('/')
    parsed = urlsplit(url)
    if (parsed.scheme not in ('http', 'https') or not parsed.hostname or parsed.query
            or parsed.fragment or parsed.username or parsed.password):
        raise ValueError('桥接模式必须显式配置有效的 --wuji-url 或 WUJI_BRIDGE_URL')
    return dict(source=source, url=url, address=None, serial=None)


def validate_frame(raw, side):
    if not isinstance(raw, dict) or raw.get("schema") != SCHEMA:
        raise ValueError("Wuji 骨架协议不匹配")
    if raw.get("side") != side or raw.get("frame_id") != ("l_wrist" if side == "left" else "r_wrist"):
        raise ValueError("Wuji 手别或坐标系不匹配")
    if raw.get("units") != "m":
        raise ValueError("Wuji 骨架单位必须为米")
    seq = raw.get("sequence")
    if isinstance(seq, bool) or not isinstance(seq, int) or seq < 0:
        raise ValueError("Wuji 序号无效")
    if not isinstance(raw.get("stream_id"), str) or not 1 <= len(raw["stream_id"]) <= 128:
        raise ValueError("Wuji 流标识无效")
    pts, conf = raw.get("positions_xyz"), raw.get("confidences")
    def number(v):
        return not isinstance(v, bool) and isinstance(v, (float, int)) and math.isfinite(v)
    if (not isinstance(pts, list) or len(pts) != 21 or any(
            not isinstance(p, list) or len(p) != 3 or any(not number(v) or abs(v) > 10 for v in p) for p in pts)):
        raise ValueError("Wuji 需要有限的 21×3 骨架点")
    if not isinstance(conf, list) or len(conf) != 21 or any(not number(v) or not 0 <= v <= 1 for v in conf):
        raise ValueError("Wuji 置信度无效")
    wrist = pts[0]
    return {"world": [[p[i] - wrist[i] for i in range(3)] for p in pts],
            "label": side.title(), "source": "wuji_glove", "normalized_is_image": False}


class WujiInputRelay:
    stale_after_ms = 350

    def __init__(self, url=None, *, source=None, address=None, serial=None,
                 clock=time.perf_counter, fetch=None):
        config = resolve_wuji_config(url, source=source, address=address, serial=serial)
        self.source, self.url = config['source'], config['url']
        self.clock, self.fetch = clock, fetch or self._fetch
        self.client = (LatestWujiSdkClient(config['address'], config['serial'], clock=clock)
                       if self.source == 'sdk' else LatestWujiClient(self.url, clock=clock))
        self.lock = threading.RLock()
        self.channels = {}
        self.listeners = {}
        if self.source == 'sdk':
            self.client.on_change = self._notify

    @property
    def supports_background(self):
        return self.source == 'sdk' and getattr(self.fetch, '__self__', None) is self

    def _notify(self):
        with self.lock:
            listeners = list(self.listeners.values())
        for callback in listeners:
            callback()

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
            return self.client.commit_if_current(channel['side'], snapshot, action)

    def _fetch(self, side):
        # Serialize lazy worker creation with closing the last session.
        with self.lock:
            if not any(c['side'] == side for c in self.channels.values()):
                raise ValueError("Wuji 会话已停止")
            return self.client.latest(side)

    def create(self, session, side):
        if side not in ("left", "right"):
            raise ValueError("Wuji 手别无效")
        with self.lock:
            self.channels[session] = {"side": side, "key": None, "revision": 0, "consumed": -1,
                                      "observed": None, "snapshots": {}, "retired": set()}
        return {"source": "wuji_glove", "transport": self.source, "stale_after_ms": self.stale_after_ms}

    def close(self, session):
        with self.lock:
            self.channels.pop(session, None)
            self.listeners.pop(session, None)
            if not self.channels:
                # Never join the SDK while holding the relay lock: its final
                # source-change notification may be waiting for this lock.
                self.client.close(wait=False)

    def latest(self, session):
        with self.lock:
            channel = self.channels.get(session)
            if channel is None:
                raise ValueError("请先创建 Wuji 手套输入")
            side = channel["side"]
        began = self.clock()
        raw, hand, age, error, event, transport = None, None, None, None, "waiting", {}
        try:
            response = self.fetch(side)
            if response.get("schema") != SCHEMA:
                raise ValueError("Wuji 采集服务协议不匹配")
            error, event = response.get("error"), response.get("state", "waiting")
            transport = response.get("transport", {})
            raw = response.get("frame")
            if raw is not None:
                hand = validate_frame(raw, side)
                age = response.get("age_ms")
                if isinstance(age, bool) or not isinstance(age, (float, int)) or not math.isfinite(age) or age < 0:
                    raise ValueError("Wuji 数据龄期无效")
                age += (self.clock() - began) * 1000
                if not response.get("connected"):
                    hand = None
        except Exception as exc:
            raw, hand, age, error, event = None, None, None, str(exc), "transport_error"
        with self.lock:
            if self.channels.get(session) is not channel:
                raise ValueError("Wuji 会话已停止")
            if raw is not None:
                key = (raw["stream_id"], raw["sequence"])
                previous = channel["key"]
                if key[0] in channel["retired"] or (previous and key[0] == previous[0] and key[1] < previous[1]):
                    hand, event = None, "out_of_order"
                elif key != previous:
                    if previous and key[0] != previous[0]:
                        channel["retired"].add(previous[0])
                        if len(channel["retired"]) > 128:
                            raise ValueError("手套重连次数过多，请重新开始会话")
                    channel["key"] = key
                    channel["revision"] += 1
                    channel["observed"] = self.clock() - age / 1000
                else:
                    # A repeated frame can never become younger when polled again.
                    channel["observed"] = min(channel["observed"], self.clock() - age / 1000)
                age = max(age, (self.clock() - channel["observed"]) * 1000) if channel["observed"] is not None else age
            if age is None or age >= self.stale_after_ms:
                hand = None
            metadata = {"source": "wuji_glove", "units": "m", "source_joint_count": 21,
                        "coordinate_space": "wuji_wrist", "normalized_is_image": False,
                        "age_ms": age, "stale_after_ms": self.stale_after_ms,
                        "tracking": hand is not None, "stale": hand is None, "event": event,
                        "error": error, "raw_wuji": raw}
            metadata["transport"] = transport
            result = {"revision": channel["revision"], "source": "wuji_glove", "hand": hand,
                      "age_ms": age, "connected": hand is not None, "tracking": hand is not None,
                      "source_metadata": metadata, "raw_wuji": raw}
            if hand is None:
                channel["snapshots"].clear()
            channel["snapshots"][channel["revision"]] = (deepcopy(result), self.clock())
            while len(channel["snapshots"]) > 4:
                del channel["snapshots"][next(iter(channel["snapshots"]))]
            return result

    def consume(self, session, revision):
        with self.lock:
            channel = self.channels.get(session)
            if channel is None:
                raise ValueError("Wuji 会话已停止")
            item = channel["snapshots"].get(revision) if type(revision) is int else None
            if item is None or revision <= channel["consumed"]:
                return {"hand": None, "source_metadata": {"source": "wuji_glove", "tracking": False,
                        "discard_reason": "obsolete_or_repeated_revision"}, "raw_wuji": None}
            result = deepcopy(item[0])
            channel["consumed"] = revision
            age = result["age_ms"]
            if age is not None:
                age += (self.clock() - item[1]) * 1000
            if age is None or age >= self.stale_after_ms:
                result.update(hand=None, tracking=False, connected=False)
            result["age_ms"] = age
            result["source_metadata"].update(age_ms=age, tracking=result["hand"] is not None,
                                             stale=result["hand"] is None)
            return result
