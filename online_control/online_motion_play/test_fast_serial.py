"""Binary transport ordering, ownership, wakeups and actual HTTP reuse."""
import concurrent.futures
import http.client
import json
import struct
import threading
from urllib.parse import urlsplit

import pytest
from browser_serial import BrowserSerial
from test_server import bridge
from test_deployment import live_server, encoded
from a30_kinematics.downlink import pack_aa55


def request_bytes(token, seq=1, rx=b""):
    return struct.pack("<4s32sI", b"MSR1", token.encode(), seq) + rx


def response_frames(data):
    magic, token, seq, count = struct.unpack_from("<4s32sIH", data)
    assert magic == b"MST1"
    offset, frames = 42, []
    for _ in range(count):
        length, age = struct.unpack_from("<Hf", data, offset)
        offset += 6
        frames.append((data[offset:offset+length], age))
        offset += length
    assert offset == len(data)
    return token.decode(), seq, frames


def test_binary_roundtrip_preserves_telemetry_bytes_and_control_order():
    port = BrowserSerial().open("browser")
    pose1, pose2 = pack_aa55(0x20, b"one"), pack_aa55(0x20, b"two")
    enable = pack_aa55(0x21, b"enable")
    for data in (pose1, pose2, enable, pose1, pose2):
        port.write(data)
    rx = bytes(range(256)) * 256
    token, seq, frames = response_frames(port.exchange_binary(request_bytes(port.token, rx=rx)))
    assert token == port.token and seq == 1
    assert [frame for frame, age in frames] == [pose2, enable, pose2]
    assert all(0 <= age < 100 for frame, age in frames)
    assert port.read(65536) == rx


def test_trajectory_metadata_coalesces_only_adjacent_positions_and_keeps_wire_bytes():
    port = BrowserSerial().open('browser')
    pose = pack_aa55(0x20, bytes(100))
    enable = pack_aa55(0x21, b'enable')
    assert port.write_trajectory(pose, [0.] * 20) == 108
    port.write_trajectory(pose, [.001] * 20)
    port.write(enable)
    port.write_trajectory(pose, [.002] * 20)
    _, _, frames = response_frames(port.exchange_binary(request_bytes(port.token)))
    assert len(frames) == 3 and frames[1][0] == enable
    for item, value in ((frames[0][0], .001), (frames[2][0], .002)):
        assert item[:4] == b'JQP1' and len(item) == 192 and item[84:] == pose
        assert struct.unpack_from('<20f', item, 4) == pytest.approx([value] * 20)


@pytest.mark.parametrize("end", ["write", "close", "reopen"])
def test_wait_releases_lock_and_wakes_without_replaying_a_new_session(end):
    port = BrowserSerial().open("browser")
    entered = threading.Event()
    original_wait = port.ready.wait_for
    def waiting(*args, **kwargs):
        entered.set()
        return original_wait(*args, **kwargs)
    port.ready.wait_for = waiting
    old = port.token
    frame = pack_aa55(0x20, b"pose")
    with concurrent.futures.ThreadPoolExecutor() as pool:
        result = pool.submit(port.exchange_raw, old, 1, b"", wait=1.)
        assert entered.wait(1)
        with pytest.raises(ValueError, match="重复"):
            port.exchange_raw(old, 2, b"")
        if end == "write":
            port.write(frame)
            assert result.result(timeout=.5)[0][0][1] == frame
        else:
            with port.lock:
                port.close()
                if end == "reopen":
                    port.open("browser")
                    port.write(frame)
            with pytest.raises(ValueError, match="断开"):
                result.result(timeout=.5)
            if end == "reopen":
                assert port.exchange_raw(port.token, 1, b"")[0][0][1] == frame


def test_fast_exchange_bypasses_control_queue_and_enforces_owner(bridge):
    bridge.dispatch("control", {}, client="owner")
    token = bridge.dispatch("connect", {"port":"browser"}, client="owner")["token"]
    body = request_bytes(token)
    with concurrent.futures.ThreadPoolExecutor() as pool:
        # No Bridge.run() here: a queued call would time out.
        data = pool.submit(bridge.call, "serial_binary", body, "owner").result(timeout=.5)
    assert response_frames(data)[2][0][0][5] == 0x2b
    last_seen = bridge.last_seen
    with pytest.raises(ValueError, match="只读"):
        bridge.call("serial_binary", request_bytes(token, 2), "observer")
    with pytest.raises(ValueError, match="无效"):
        bridge.call("serial_binary", request_bytes("0"*32, 2), "owner")
    assert bridge.last_seen == last_seen
    bridge.last_seen -= 6
    with pytest.raises(ValueError, match="只读"):
        bridge.call("serial_binary", request_bytes(token, 2), "owner")


@pytest.mark.parametrize("data", [b"", b"MSR1" + bytes(35), b"BAD1" + bytes(36), b"MSR1" + b"\xff"*36, b"MSR1" + bytes(65577)],
                         ids=["empty", "short", "magic", "token", "oversized"])
def test_invalid_binary_requests_do_not_advance_sequence(data):
    port = BrowserSerial().open("browser")
    with pytest.raises(ValueError):
        port.exchange_binary(data)
    assert port.exchange_id == 0


def test_http_binary_reuses_connection_and_checks_origin(live_server):
    url = urlsplit(live_server)
    connection = http.client.HTTPConnection(url.hostname, url.port, timeout=3)
    headers = {"Content-Type":"application/json", "X-Motion-Client":"f"*32}
    def post(action, body, extra=None):
        raw = isinstance(body, bytes)
        connection.request("POST", "/api/"+action, body if raw else json.dumps(body),
                           {**headers, **({"Content-Type":"application/octet-stream"} if raw else {}), **(extra or {})})
        response = connection.getresponse()
        data = response.read()
        return response.status, data
    try:
        assert post("control", {})[0] == 200
        status, data = post("connect", {"port":"browser"})
        connected = json.loads(data)["result"]
        assert connected["transport"] == "binary-v1"
        socket = connection.sock
        token = connected["token"]
        status, data = post("serial_binary", request_bytes(token))
        assert status == 200 and response_frames(data)[2]
        assert connection.sock is socket
        assert post("serial_binary", request_bytes(token, 2), {"Origin":"https://evil.invalid"})[0] == 403
        assert connection.sock is None
        assert post("serial_binary", request_bytes(token, 2))[0] == 200
    finally:
        post("disconnect", {})
        post("release", {})
        connection.close()


def test_released_owner_and_old_stream_cannot_drain_new_transport(bridge):
    bridge.dispatch("control", {}, client="old-owner")
    old = bridge.dispatch("connect", {"port": "browser"}, client="old-owner")["token"]
    stream = bridge.call("serial_stream", {"token": old}, client="old-owner")
    bridge.dispatch("release", {}, client="old-owner")
    bridge.dispatch("control", {}, client="new-owner")
    new = bridge.dispatch("connect", {"port": "browser"}, client="new-owner")["token"]
    with pytest.raises(ValueError, match="只读"):
        bridge.call("serial_binary", request_bytes(new), "old-owner")
    with pytest.raises(ValueError, match="无效"):
        bridge.call("serial_binary", request_bytes(old), "new-owner")
    # Finishing an old HTTP stream must neither close nor consume the new port.
    assert list(stream) == []
    stream.close()
    assert bridge.serial.is_open
    data = bridge.call("serial_binary", request_bytes(new), "new-owner")
    assert response_frames(data)[2][0][0][5] == 0x2b
