"""Real .so integration against local TCP and PTY only. Never opens hardware.

Run: python3 test_offline.py /absolute/path/to/liba32_native.so
Wire golden values come from the unmodified Windows vendor SDK oracle.
"""
import ctypes
import importlib.util
import json
import math
import os
from pathlib import Path
import pty
import queue
import select
import socket
import struct
import sys
import threading
import time
import unittest

LIBRARY = Path(sys.argv.pop(1)).resolve()
spec = importlib.util.spec_from_file_location("a32_native_under_test", Path(__file__).parents[2] / "native.py")
native = importlib.util.module_from_spec(spec)
spec.loader.exec_module(native)
HOST_QUERY = bytes.fromhex("aa550800030101010001000000003d72")
ENABLE = bytes.fromhex("aa5504000321ffff01036984")
DISABLE = bytes.fromhex("aa5504000321ffff02036974")
HOME17 = bytes.fromhex("aa550c000322020201000000a04200000000e26d")
RESET17 = bytes.fromhex("aa550c00032202020200000000000000000072cd")


def crc(data):
    value = 0xffff
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = (value >> 1) ^ (0xa001 if value & 1 else 0)
    return value


def frame(fun, payload, dev=3):
    data = b"\xaa\x55" + struct.pack("<HBB", len(payload), dev, fun) + payload
    return data + struct.pack("<H", crc(data))


def telemetry(sequence=0x1234, nan=False):
    data = bytearray(484)
    struct.pack_into("<HBBII", data, 0, sequence, 3, 0, 0x78563412, 0x3ffff)
    for i in range(18):
        off = 12 + i * 24
        data[off:off + 4] = bytes([0, 0, 0, i % 5])
        struct.pack_into("<ffffhh", data, off + 4, math.nan if nan and i == 0 else i + .125,
                         i - 8.5, i * .25, i + .75, -100 + i, 2500 + i * 10)
    for i in range(2):
        # Confirmed SDK: centered uint32 position; centered uint16 velocity/current.
        sign = 1 if i == 0 else -1
        struct.pack_into("<HIHHHIBBBB", data, 444 + i * 20, 0, 0x80000000 + sign * 2097152,
                         32768 + sign * 500, 32768 + sign * 1024, 240 + i, 1 << i,
                         2 + i, 3 + i, 4 + i, 5 + i)
    return bytes(data)


class Device:
    """A deterministic protocol peer, using either loopback or a pseudo terminal."""
    def __init__(self, kind="tcp", firmware=b"\x01\x00\x01\x02\x03\x04\x00\x00", reply=True):
        self.kind, self.firmware, self.reply = kind, firmware, reply
        self.frames = queue.Queue()
        self.stop = threading.Event()
        self.peer = None
        self.writer = threading.Lock()
        self.error = None
        if kind == "tcp":
            self.listener = socket.socket()
            self.listener.bind(("127.0.0.1", 0))
            self.port = self.listener.getsockname()[1]
            self.listener.listen(1)
            self.listener.settimeout(2)
        else:
            self.master, self.slave = pty.openpty()
            self.path = os.ttyname(self.slave)
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def send(self, data, fragmented=False):
        with self.writer:
            parts = [data[i:i+3] for i in range(0, len(data), 3)] if fragmented else [data]
            for part in parts:
                if self.kind == "tcp":
                    self.peer.sendall(part)
                else:
                    position = 0
                    while position < len(part):
                        position += os.write(self.master, part[position:])
                if fragmented:
                    time.sleep(.0002)

    def run(self):
        pending = bytearray()
        try:
            if self.kind == "tcp":
                self.peer, _ = self.listener.accept()
                fd = self.peer
            else:
                fd = self.master
            while not self.stop.is_set():
                if not select.select([fd], [], [], .05)[0]:
                    continue
                chunk = self.peer.recv(2048) if self.kind == "tcp" else os.read(fd, 2048)
                if not chunk:
                    break
                pending.extend(chunk)
                while len(pending) >= 8:
                    if pending[:2] != b"\xaa\x55":
                        raise AssertionError("host transmitted malformed magic")
                    size = struct.unpack_from("<H", pending, 2)[0] + 8
                    if len(pending) < size:
                        break
                    packet = bytes(pending[:size])
                    del pending[:size]
                    assert crc(packet[:-2]) == struct.unpack("<H", packet[-2:])[0]
                    self.frames.put(packet)
                    if packet[5] == 1 and self.reply:
                        # Noise, bad CRC, oversized length, wrong device, then split valid reply.
                        self.send(b"junk\xaa\x55\xff\xff\x03\x01" + frame(1, self.firmware)[:-1] + b"\xff")
                        self.send(frame(1, self.firmware, dev=4))
                        self.send(frame(1, self.firmware), fragmented=True)
        except (OSError, ValueError) as exc:
            if not self.stop.is_set():
                self.error = exc
        except BaseException as exc:
            self.error = exc

    def connect(self, hand):
        if self.kind == "tcp":
            hand.connect_eth("127.0.0.1", self.port)
        else:
            hand.connect_usb(self.path)

    def next(self):
        return self.frames.get(timeout=1)

    def close(self):
        self.stop.set()
        if self.kind == "tcp":
            if self.peer:
                try:
                    self.peer.shutdown(socket.SHUT_RDWR)
                except OSError:
                    pass
                self.peer.close()
            self.listener.close()
        else:
            os.close(self.master)
            os.close(self.slave)
        self.thread.join(timeout=2)


class OfflineDriverTest(unittest.TestCase):
    def setUp(self):
        self.hand = native.NativeHand(LIBRARY)
        self.addCleanup(self.hand.close)

    def device(self, **kwargs):
        peer = Device(**kwargs)
        self.addCleanup(peer.close)
        return peer

    def eventually(self, predicate, timeout=1):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(.005)
        self.fail("condition did not become true before timeout")

    def exercise_transport(self, kind):
        peer = self.device(kind=kind)
        peer.connect(self.hand)
        self.assertEqual(peer.next(), HOST_QUERY)
        self.assertTrue(peer.frames.empty(), "connect must not send power or targets")
        with self.assertRaises(native.A32DriverError):
            self.hand.send_once()
        peer.send(frame(2, telemetry()) + frame(3, struct.pack("<HhhH", 0, -250, 3250, 24000)), fragmented=True)
        self.eventually(lambda: self.hand.get_state()["sequence"] == 0x1234)
        state = self.hand.get_state()
        self.assertEqual(len(state["self"]), 18)
        self.assertEqual(len(state["mit"]), 2)
        self.assertEqual(state["self"][17]["position"], 17.125)
        self.assertEqual(state["self"][17]["temperature_centi"], 2670)
        self.assertAlmostEqual(state["mit"][0]["position"], 7.2, places=5)
        self.assertEqual(state["mit"][0]["velocity_rpm"], 1)
        self.assertEqual(state["mit"][1]["iq"], -2.75)
        self.eventually(lambda: self.hand.get_state()["board"]["valid"])
        self.assertEqual(self.hand.get_state()["board"]["vin24_mv"], 24000)
        fast = self.hand.get_control_state()
        self.assertTrue(fast["connected"])
        self.assertTrue(fast["homing_active"])
        self.assertTrue(fast["mit"][0]["fault"])
        self.hand.enable_all()
        self.assertEqual(peer.next(), ENABLE)
        self.hand.home_slot(17, 80)
        self.assertEqual(peer.next(), HOME17)
        self.hand.home_reset_slot(17)
        self.assertEqual(peer.next(), RESET17)
        positions = [i + 1.125 for i in range(18)]
        first = self.hand.send_targets(positions, [1, -2])
        packet = peer.next()
        self.assertEqual(len(packet), 108)
        self.assertEqual(packet[5:7], b"\x20\x00")
        self.assertEqual(struct.unpack_from("<18f", packet, 7), tuple(positions))
        self.assertEqual(struct.unpack_from("<2f", packet, 79), (50, -100))
        self.assertEqual(packet[87:106], bytes(19))
        second = self.hand.send_targets(positions, [1, -2])
        self.assertEqual(peer.next()[6], 1)
        self.assertGreaterEqual(second["started_ns"] - first["started_ns"], 1_000_000)
        self.assertGreaterEqual(second["finished_ns"], second["started_ns"])
        cached = self.hand.get_state()["targets"]
        with self.assertRaises(native.A32DriverError):
            self.hand.set_targets([math.nan] + positions[1:], [1, 2])
        with self.assertRaises(native.A32DriverError):
            self.hand.set_targets(positions, [math.inf, 2])
        self.assertEqual(self.hand.get_state()["targets"], cached)
        self.assertTrue(peer.frames.empty())
        self.hand.estop()
        self.assertEqual(peer.next(), DISABLE)
        self.hand.enable_all(False)
        self.assertEqual(peer.next(), DISABLE)
        peer.send(frame(2, telemetry(sequence=0x1235, nan=True)))
        self.eventually(lambda: self.hand.get_state()["sequence"] == 0x1235)
        self.assertIsNone(self.hand.get_state()["self"][0]["position"])
        self.hand.disconnect()
        self.assertFalse(self.hand.get_state()["connected"])
        self.assertEqual(self.hand.get_state()["diagnostics"]["disconnect_count"], 0)
        self.assertTrue(peer.frames.empty())
        self.assertIsNone(peer.error)

    def test_tcp(self):
        self.exercise_transport("tcp")

    def test_pty(self):
        self.exercise_transport("pty")

    def test_pty_exclusive_ownership(self):
        peer = self.device(kind="pty")
        peer.connect(self.hand)
        self.assertEqual(peer.next(), HOST_QUERY)
        other = native.NativeHand(LIBRARY)
        self.addCleanup(other.close)
        with self.assertRaisesRegex(native.A32DriverError, "open serial port"):
            peer.connect(other)
        self.assertTrue(peer.frames.empty())
        self.hand.disconnect()
        peer.connect(other)
        self.assertEqual(peer.next(), HOST_QUERY)
        other.disconnect()

    def test_handshake_timeout(self):
        peer = self.device(reply=False)
        before = time.monotonic()
        with self.assertRaisesRegex(native.A32DriverError, "timed out"):
            peer.connect(self.hand)
        self.assertLess(time.monotonic() - before, 3)
        self.assertFalse(self.hand.get_state()["connected"])
        self.assertEqual(peer.next(), HOST_QUERY)
        self.assertTrue(peer.frames.empty())

    def test_incompatible_firmware(self):
        peer = self.device(firmware=b"\x02\x00\x01\x00\x00\x00\x00\x00")
        with self.assertRaisesRegex(native.A32DriverError, "incompatible"):
            peer.connect(self.hand)
        self.assertFalse(self.hand.get_state()["connected"])
        self.assertEqual(peer.next(), HOST_QUERY)
        self.assertTrue(peer.frames.empty())

    def test_peer_loss_and_reconnect(self):
        peer = Device()
        try:
            peer.connect(self.hand)
            self.assertEqual(peer.next(), HOST_QUERY)
            self.hand.send_targets([0] * 18, [0, 0])
            self.assertEqual(peer.next()[6], 0)
        finally:
            peer.close()
        self.eventually(lambda: not self.hand.get_state()["connected"])
        state = self.hand.get_state()
        self.assertEqual(state["diagnostics"]["disconnect_count"], 1)
        with self.assertRaises(native.A32DriverError):
            self.hand.send_targets([0] * 18, [0, 0])
        peer2 = self.device()
        peer2.connect(self.hand)
        self.assertEqual(peer2.next(), HOST_QUERY)
        with self.assertRaises(native.A32DriverError):
            self.hand.send_once()
        self.hand.send_targets([0] * 18, [0, 0])
        # This implementation counts attempted sends, including a failed send.
        self.assertEqual(peer2.next()[6], 2)

    def test_invalid_arguments_and_json_capacity(self):
        for slot in (-1, 18):
            with self.assertRaises(native.A32DriverError):
                self.hand.home_slot(slot, 80)
        for value in (0, -1, math.nan, math.inf):
            with self.assertRaises(native.A32DriverError):
                self.hand.home_slot(0, value)
        buffer = ctypes.create_string_buffer(b"sentinel", 9)
        required = self.hand._dll.a32_state(self.hand._handle, buffer, 1)
        self.assertGreater(required, 1)
        self.assertEqual(buffer.value, b"sentinel")
        self.assertEqual(self.hand.get_state()["sdk_version"], "independent-linux-abi2/1.0")


if __name__ == "__main__":
    unittest.main(verbosity=2)
