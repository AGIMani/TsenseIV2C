"""Bounded, expiring in-memory serial transport. Never opens a host device.

The control planners produce protocol frames; only the owning browser can
exchange them with its locally authorized Web Serial port. Adjacent position
samples are coalesced, while configuration/enable/home order is preserved.
"""
import base64
import binascii
import struct
import threading
import time
import uuid
from online_control.shared.stream import StreamLease


class BrowserSerial:
    LEASE = .35
    MAX_RX = 65536
    MAX_TX = 256

    def __init__(self, clock=time.perf_counter):
        self.clock = clock
        self.lock = threading.RLock()
        self.ready = threading.Condition(self.lock)
        self.exchanging = False
        self.streaming = False
        self.is_open = False
        self.token = ""
        self.rx = bytearray()
        self.tx = []
        self.last_exchange = 0.
        self.exchange_id = 0
        self.port = "browser"

    def open(self, port, **_kwargs):
        if port != "browser":
            raise ValueError("只能连接打开页面的电脑上的浏览器串口")
        with self.lock:
            if self.is_open:
                raise ValueError("浏览器串口已连接")
            self.token = uuid.uuid4().hex
            self.rx.clear()
            self.tx.clear()
            self.exchange_id = 0
            self.last_exchange = self.clock()
            self.is_open = True
            self.streaming = False
        return self

    @property
    def expired(self):
        return self.is_open and self.clock() - self.last_exchange > self.LEASE

    @property
    def in_waiting(self):
        with self.lock:
            return len(self.rx)

    @property
    def out_waiting(self):
        return 0  # Positions are coalesced, never queued at the planner's rate.

    def read(self, count):
        with self.lock:
            result = bytes(self.rx[:count])
            del self.rx[:count]
            return result

    def write(self, data):
        data = bytes(data)
        with self.lock:
            if not self.is_open or self.expired:
                raise OSError("浏览器串口链路中断")
            # Both supported H7 protocols use 0x20 for full-hand positions.
            def position(frame):
                return frame.startswith(b'JQP1') or len(frame) > 5 and frame[5] == 0x20
            if self.tx and position(data) and position(self.tx[-1][1]):
                self.tx.pop()
            if len(self.tx) >= self.MAX_TX:
                self.close()
                raise OSError("浏览器下发队列拥塞，已断开")
            self.tx.append((self.clock(), data))
            self.ready.notify_all()
        return len(data)

    def write_trajectory(self, frame, q):
        # Transport-only metadata, stripped before physical USB. Preserve the
        # planned joint coordinates for the browser's last-successful-write guard.
        if len(q) != 20 or len(frame) != 108 or frame[5] != 0x20:
            raise ValueError('Invalid A3 trajectory metadata')
        self.write(b'JQP1' + struct.pack('<20f', *q) + frame)
        return len(frame)

    def exchange(self, body):
        encoded = body.get("rx", "")
        if not isinstance(encoded, str) or len(encoded) > 87384:
            raise ValueError("串口遥测数据过大")
        try:
            incoming = base64.b64decode(encoded, validate=True)
        except (ValueError, binascii.Error):
            raise ValueError("串口遥测编码无效") from None
        frames, now = self.exchange_raw(body.get("token"), body.get("seq"), incoming)
        return {"token": body["token"], "seq": body["seq"],
                "frames": [{"data": base64.b64encode(data).decode(), "age_ms": (now-at)*1000}
                           for at, data in frames]}

    def exchange_binary(self, body, *, receive_only=False):
        # MSR1 + 32 ASCII token bytes + uint32 sequence + unencoded telemetry.
        if len(body) < 40 or len(body) > 40 + self.MAX_RX or body[:4] != b"MSR1":
            raise ValueError("二进制串口请求格式无效")
        try:
            token = body[4:36].decode("ascii")
        except UnicodeDecodeError:
            raise ValueError("串口会话无效") from None
        seq = struct.unpack_from("<I", body, 36)[0]
        frames, now = self.exchange_raw(token, seq, body[40:], wait=0. if receive_only else .01, receive_only=receive_only)
        return self.packet(token, seq, frames, now)

    @staticmethod
    def packet(token, seq, frames, now):
        result = bytearray(struct.pack("<4s32sIH", b"MST1", token.encode("ascii"), seq, len(frames)))
        for at, data in frames:
            result.extend(struct.pack("<Hf", len(data), (now-at)*1000))
            result.extend(data)
        return bytes(result)

    def open_stream(self, token):
        with self.lock:
            if not self.is_open or self.expired or token != self.token or self.streaming:
                raise ValueError("串口流无效或已有页面正在读取")
            self.streaming = True
        return StreamLease(self._stream(token), lambda: self._release_stream(token))

    def _release_stream(self, token):
        with self.lock:
            if self.token == token:
                self.streaming = False
                self.close()

    def _stream(self, token):
        try:
            while True:
                with self.lock:
                    self.ready.wait_for(lambda: self.tx or not self.is_open or self.token != token, timeout=.02)
                    if not self.is_open or self.token != token or self.expired:
                        return
                    now = self.clock()
                    if self.tx and now-self.tx[0][0] > self.LEASE:
                        return
                    frames, self.tx = self.tx, []
                    packet = self.packet(token, self.exchange_id, frames, now)
                yield packet
        finally:
            pass  # StreamLease releases even an iterator that never started.

    def exchange_raw(self, token, seq, incoming, *, wait=0., receive_only=False):
        with self.lock:
            if not self.is_open or token != self.token:
                raise ValueError("浏览器串口会话无效，请重新连接")
            if self.expired:
                self.close()
                raise ValueError("浏览器串口链路超时，请重新连接")
            if self.exchanging or type(seq) is not int or seq != self.exchange_id + 1:
                raise ValueError("串口请求重复或乱序")
            if self.streaming and not receive_only:
                raise ValueError("推送模式禁止并行拉取下发帧")
            if len(self.rx) + len(incoming) > self.MAX_RX:
                self.close()
                raise ValueError("串口遥测拥塞，已断开")
            now = self.clock()
            if self.tx and now - self.tx[0][0] > self.LEASE:
                self.close()
                raise ValueError("下发命令过期，已断开")
            self.rx.extend(incoming)
            self.last_exchange, self.exchange_id = now, seq
            self.exchanging = True
            try:
                # Releases the lock while waiting. A planner write wakes this
                # request immediately; idle connections heartbeat every 10 ms.
                if wait and not self.tx:
                    self.ready.wait_for(lambda: self.tx or not self.is_open or token != self.token, timeout=wait)
                if not self.is_open or token != self.token:
                    raise ValueError("浏览器串口会话已断开")
                if receive_only:
                    return [], self.clock()
                frames, self.tx = self.tx, []
                return frames, self.clock()
            finally:
                self.exchanging = False

    def reset_input_buffer(self):
        with self.lock:
            self.rx.clear()

    def reset_output_buffer(self):
        with self.lock:
            self.tx.clear()

    def flush(self):
        pass

    def close(self):
        with self.lock:
            self.is_open = False
            self.rx.clear()
            self.tx.clear()
            self.ready.notify_all()
