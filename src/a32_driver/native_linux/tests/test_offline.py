"""Real .so integration against local TCP and PTY only. Never opens hardware.

Run: python3 test_offline.py /absolute/path/to/liba32_native.so
Wire golden values retain the original vendor oracle fixtures, with SDK 1.1.15
host metadata, slot limits, homing classification and current units verified
through the real native adapter against independent raw telemetry fixtures.
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
HOST_QUERY = bytes.fromhex("aa550800030101010f01000000003d8d")
ENABLE = bytes.fromhex("aa5504000321ffff01036984")
HOME17 = bytes.fromhex("aa550c000322020201000000a04200000000e26d")
RESET17 = bytes.fromhex("aa550c00032202020200000000000000000072cd")
# Explicit expectations include the SDK 1.1.15 four-link stroke caps.
SELF_MAX = (12.437476671033064, 11.761489775595662, 19.627778913882647,
            12.437476671033064, 16.939021296767194, 19.627778913882647,
            20., 20., 12.437476671033064, 16.939021296767194,
            19.627778913882647, 12.437476671033064, 16.939021296767194,
            19.627778913882647, 20., 20., 12.437476671033064, 16.939021296767194)
SELF_MIN = tuple(-20. if i in (6, 7, 14, 15) else 0. for i in range(18))


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
        self.received_bytes = 0
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
                self.received_bytes += len(chunk)
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

    def assert_quiet(self, peer, received_bytes):
        # Waiting on the peer prevents a just-written command from escaping an
        # immediate Queue.empty() assertion; count even incomplete wire frames.
        with self.assertRaises(queue.Empty):
            peer.frames.get(timeout=.12)
        self.assertEqual(peer.received_bytes, received_bytes)
        self.assertIsNone(peer.error)

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
        positions = [i * .5 + 1.125 for i in range(18)]
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
        received_bytes = peer.received_bytes
        self.hand.estop()
        self.hand.enable_all(False)
        self.assert_quiet(peer, received_bytes)
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

    def test_sdk_self_currents_are_amperes_once_for_all_slots(self):
        peer = self.device(reply=False)
        peer.connect(self.hand)
        self.assertEqual(peer.next(), HOST_QUERY)
        rotary_slots = {6, 7, 14, 15}
        for sequence, (measured, reference) in enumerate(((819., -80.), (-819., 80.), (0., 0.)), 1):
            with self.subTest(measured_counts=measured, reference_counts=reference):
                payload = bytearray(telemetry(sequence=sequence))
                for slot in range(18):
                    struct.pack_into("<ff", payload, 12 + 24 * slot + 8, measured, reference)
                peer.send(frame(2, payload))
                self.eventually(lambda: self.hand.get_state()["sequence"] == sequence)
                state = self.hand.get_state()
                self.assertEqual(state["telemetry_format"], "a32-native-sdk-state/v2")
                self.assertEqual(state["telemetry_units"], {"self_current": "A", "external_current": "A"})
                for slot, row in enumerate(state["self"]):
                    scale = 1. / 819. if slot in rotary_slots else .001374
                    self.assertAlmostEqual(row["iq"], measured * scale, places=6)
                    self.assertAlmostEqual(row["iq_ref"], reference * scale, places=6)
                    self.assertEqual(row["current_unit"], "A")
                self.assertEqual(state["mit"][0]["iq"], 2.75)
                self.assertEqual(state["mit"][1]["iq"], -2.75)
                self.assertTrue(all(row["current_unit"] == "A" for row in state["mit"]))

        # Invalid readings must remain JSON null; never turn them into zero A.
        payload = bytearray(telemetry(sequence=4))
        for slot, readings in ((0, (math.nan, math.inf)), (6, (-math.inf, math.nan)),
                               (14, (math.inf, -math.inf)), (17, (math.nan, math.nan))):
            struct.pack_into("<ff", payload, 12 + 24 * slot + 8, *readings)
        peer.send(frame(2, payload))
        self.eventually(lambda: self.hand.get_state()["sequence"] == 4)
        state = self.hand.get_state()
        for slot in (0, 6, 14, 17):
            self.assertIsNone(state["self"][slot]["iq"])
            self.assertIsNone(state["self"][slot]["iq_ref"])

        # Feedback unit conversion must never alter homing's count-based command.
        self.hand.home_slot(17, 80)
        self.assertEqual(peer.next(), HOME17)

    def test_sdk_homing_distinguishes_float_low_bytes_from_open_spin(self):
        peer = self.device(reply=False)
        peer.connect(self.hand)
        self.assertEqual(peer.next(), HOST_QUERY)
        cases = [(bytes((0, 0, 0, code)), code) for code in range(5)]
        # Ordinary floats near 1 rad used to be mislabeled OpenSpin solely
        # because their mantissa low byte happened to be 01..04.
        cases += [(bytes((low, 0, 0x80, 0x3f)), 254) for low in range(1, 5)]
        cases += [(bytes((code, 0xe8, 3, 4)), 255) for code in range(1, 5)]
        cases += [(bytes((1, 0xe8, 3, 5)), 254),  # High byte outside a homing code.
                  (bytes((0, 0, 0, 0x40)), 0),  # Retain the SDK's exact-float fallback.
                  (bytes((0, 0, 0x80, 0x3f)), 254),
                  (bytes(4), 0), (bytes((0, 0, 0, 2)), 2)]
        payload = bytearray(telemetry(sequence=1))
        for slot, (raw, expected) in enumerate(cases):
            payload[12 + 24 * slot:16 + 24 * slot] = raw
        peer.send(frame(2, payload))
        self.eventually(lambda: self.hand.get_state()["sequence"] == 1)
        state = self.hand.get_state()
        for slot, (raw, expected) in enumerate(cases):
            with self.subTest(slot=slot, raw=raw.hex()):
                self.assertEqual(state["self"][slot]["homing"], expected)
                self.assertEqual(state["self"][slot]["open_spin"], expected == 255)
        self.assertTrue(self.hand.get_control_state()["homing_active"])
        # An all-theta frame clears SEEK/RELEASE in the fast control snapshot.
        struct.pack_into("<H", payload, 0, 2)
        for slot in range(18):
            payload[12 + 24 * slot:16 + 24 * slot] = bytes((1, 0, 0x80, 0x3f))
        peer.send(frame(2, payload))
        self.eventually(lambda: self.hand.get_state()["sequence"] == 2)
        self.assertFalse(self.hand.get_control_state()["homing_active"])

    def test_sdk_slot_limits_match_cached_and_transmitted_targets(self):
        peer = self.device()
        peer.connect(self.hand)
        self.assertEqual(peer.next(), HOST_QUERY)
        for requested, expected, mit_requested, mit_expected in (
                ([100.] * 18, SELF_MAX, [200., -200.], [180., -180.]),
                ([-100.] * 18, SELF_MIN, [-200., 200.], [-180., 180.]),
                (list(SELF_MAX), SELF_MAX, [180., -180.], [180., -180.])):
            with self.subTest(requested=requested[0]):
                self.hand.send_targets(requested, mit_requested)
                packet = peer.next()
                # Compare float32 bytes so rounding at SDK decimal limits is
                # checked independently of JSON snapshot formatting.
                self.assertEqual(packet[7:79], struct.pack("<18f", *expected))
                self.assertEqual(packet[79:87], struct.pack("<2f", *(x * 50. for x in mit_expected)))
                cached = self.hand.get_state()["targets"]
                for slot, value in enumerate(expected):
                    self.assertAlmostEqual(cached["self"][slot], value, places=5)
                self.assertEqual(cached["mit_pos"], mit_expected)

    def test_invalid_target_batches_never_partially_replace_or_send(self):
        peer = self.device()
        peer.connect(self.hand)
        self.assertEqual(peer.next(), HOST_QUERY)
        self.hand.send_targets([2.] * 18, [3., 4.])
        peer.next()
        cached = self.hand.get_state()["targets"]
        received_bytes = peer.received_bytes
        for operation in (self.hand.set_targets, self.hand.send_targets):
            for positions, mit in (([7.] * 17 + [math.nan], [8., 9.]),
                                   ([7.] * 18, [8., math.inf]),
                                   ([7.] * 18, [8., -math.inf])):
                with self.subTest(operation=operation.__name__, invalid=mit[-1]):
                    with self.assertRaises(native.A32DriverError):
                        operation(positions, mit)
                    self.assertEqual(self.hand.get_state()["targets"], cached)
                    self.assert_quiet(peer, received_bytes)
        self.hand.send_once()
        packet = peer.next()
        self.assertEqual(struct.unpack_from("<18f", packet, 7), (2.,) * 18)
        self.assertEqual(struct.unpack_from("<2f", packet, 79), (150., 200.))

    def test_firmware_hand_side_metadata(self):
        peer = self.device(reply=False)
        peer.connect(self.hand)
        self.assertEqual(peer.next(), HOST_QUERY)
        for sequence, (wire_side, expected) in enumerate(((0, 0), (1, 1), (2, 2), (255, 0)), 1):
            with self.subTest(wire_side=wire_side):
                peer.send(frame(1, bytes((1, 0, 1, 1, 9, sequence, wire_side, 0))), fragmented=True)
                self.eventually(lambda: self.hand.get_state()["firmware"]["app_build"] == sequence)
                state = self.hand.get_state()
                self.assertEqual(state["firmware"]["hand_side"], expected)
                self.assertTrue(state["connected"])
                self.assertTrue(peer.frames.empty(), "metadata must not enable or send targets")

    def test_mit_identity_validity_is_separate_from_online_and_keeps_precision(self):
        peer = self.device(reply=False)
        peer.connect(self.hand)
        self.assertEqual(peer.next(), HOST_QUERY)
        payload = bytearray(telemetry(sequence=1))
        struct.pack_into("<H", payload, 2, 0)  # Both offline, identity may still be valid.
        payload[444:446] = bytes((0x05, 0x05))
        struct.pack_into("<I", payload, 446, 0x80000001)
        peer.send(frame(2, payload))
        self.eventually(lambda: self.hand.get_state()["sequence"] == 1)
        state = self.hand.get_state()
        self.assertTrue(state["mit"][0]["valid"])
        self.assertFalse(state["mit"][0]["online"])
        self.assertAlmostEqual(state["mit"][0]["position"], 360. / 2097152. / 50., places=12)
        self.assertFalse(state["mit"][1]["valid"])
        self.assertFalse(state["mit"][1]["online"])
        self.assertEqual(state["mit"][1]["fault"], 0)
        self.assertFalse(any(row["fault"] for row in self.hand.get_control_state()["mit"]))
        # An online mask is sufficient even when old firmware omits identity.
        struct.pack_into("<HH", payload, 0, 2, 2)
        struct.pack_into("<I", payload, 466, 0x7fffffff)
        peer.send(frame(2, payload))
        self.eventually(lambda: self.hand.get_state()["sequence"] == 2)
        state = self.hand.get_state()
        self.assertTrue(state["mit"][1]["valid"])
        self.assertTrue(state["mit"][1]["online"])
        self.assertAlmostEqual(state["mit"][1]["position"], -360. / 2097152. / 50., places=12)
        self.assertTrue(self.hand.get_control_state()["mit"][1]["fault"])

    def test_native_stop_calls_cancel_cached_targets_without_wire_commands(self):
        # Call the C ABI directly: a Python compatibility guard must not hide a
        # regression in a32_estop or a32_enable_all(false) inside the library.
        stop_sending = self.hand._dll.a32_stop_sending
        stop_sending.argtypes = [ctypes.c_void_p]
        stop_sending.restype = ctypes.c_int
        for kind in ("tcp", "pty"):
            peer = self.device(kind=kind)
            peer.connect(self.hand)
            self.assertEqual(peer.next(), HOST_QUERY)
            for name, args in (("a32_estop", ()), ("a32_enable_all", (0,)), ("a32_stop_sending", ())):
                with self.subTest(kind=kind, operation=name):
                    self.hand.set_targets([12.] * 18, [1., -2.])
                    received_bytes = peer.received_bytes
                    self.assertEqual(getattr(self.hand._dll, name)(self.hand._handle, *args), 1)
                    self.assertTrue(self.hand.get_state()["connected"])
                    self.assertEqual(self.hand._dll.a32_send_once(self.hand._handle), 0)
                    self.assert_quiet(peer, received_bytes)
                    # Enabling power remains explicit and cannot replay the
                    # cancelled target; a fresh target resumes normal sending.
                    self.assertEqual(self.hand._dll.a32_enable_all(self.hand._handle, 1), 1)
                    self.assertEqual(peer.next(), ENABLE)
                    received_bytes = peer.received_bytes
                    self.assertEqual(self.hand._dll.a32_send_once(self.hand._handle), 0)
                    self.assert_quiet(peer, received_bytes)
                    self.hand.send_targets([3.] * 18, [0., 0.])
                    packet = peer.next()
                    self.assertEqual(packet[5], 0x20)
                    self.assertEqual(struct.unpack_from("<18f", packet, 7), (3.,) * 18)
            peer.send(frame(2, telemetry()))
            self.eventually(lambda: self.hand.get_state()["sequence"] == 0x1234)
            self.hand.disconnect()

    def test_disconnect_and_close_never_send_commands(self):
        for kind in ("tcp", "pty"):
            for action in ("disconnect", "close"):
                with self.subTest(kind=kind, action=action):
                    peer = self.device(kind=kind)
                    hand = native.NativeHand(LIBRARY)
                    self.addCleanup(hand.close)
                    peer.connect(hand)
                    self.assertEqual(peer.next(), HOST_QUERY)
                    hand.enable_all()
                    self.assertEqual(peer.next(), ENABLE)
                    hand.set_targets([9.] * 18, [1., 2.])
                    received_bytes = peer.received_bytes
                    getattr(hand, action)()
                    self.assert_quiet(peer, received_bytes)

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

    def test_connection_without_firmware_reply(self):
        for kind in ("tcp", "pty"):
            with self.subTest(kind=kind):
                peer = self.device(kind=kind, reply=False)
                before = time.monotonic()
                peer.connect(self.hand)
                self.assertLess(time.monotonic() - before, 1)
                self.assertEqual(peer.next(), HOST_QUERY)
                state = self.hand.get_state()
                self.assertTrue(state["connected"])
                self.assertFalse(state["firmware"]["valid"])
                # Missing version metadata does not prevent valid telemetry.
                peer.send(frame(2, telemetry()))
                self.eventually(lambda: self.hand.get_state()["sequence"] == 0x1234)
                self.assertTrue(peer.frames.empty(), "connect must not send power or targets")
                self.hand.disconnect()

    def test_late_firmware_reply_is_metadata(self):
        peer = self.device(reply=False)
        peer.connect(self.hand)
        self.assertEqual(peer.next(), HOST_QUERY)
        time.sleep(1.6)  # Beyond the former handshake deadline.
        self.assertTrue(self.hand.get_state()["connected"])
        self.assertFalse(self.hand.get_state()["firmware"]["valid"])
        peer.send(frame(1, peer.firmware), fragmented=True)
        self.eventually(lambda: self.hand.get_state()["firmware"]["valid"])
        self.assertTrue(self.hand.get_state()["connected"])
        self.assertTrue(peer.frames.empty())

    def test_other_firmware_versions_do_not_block_connection(self):
        for major_index in (0, 2):
            firmware = bytearray(b"\x01\x00\x01\x00\x00\x00\x00\x00")
            firmware[major_index] = 2
            peer = self.device(firmware=bytes(firmware))
            peer.connect(self.hand)
            self.assertEqual(peer.next(), HOST_QUERY)
            self.eventually(lambda: self.hand.get_state()["firmware"]["valid"])
            state = self.hand.get_state()
            self.assertTrue(state["connected"])
            self.assertEqual(state["firmware"]["proto_major" if major_index == 0 else "app_major"], 2)
            self.assertTrue(peer.frames.empty())
            self.hand.disconnect()

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
        state = self.hand.get_state()
        self.assertEqual(state["sdk_version"], "1.1.15")
        self.assertEqual(state["native_implementation"], "a3-sdk-protocol-linux-abi2/1.0")


if __name__ == "__main__":
    unittest.main(verbosity=2)
