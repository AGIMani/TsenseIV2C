from __future__ import annotations

import base64
import json
import os
from pathlib import Path
import socket
import ssl
import struct
import sys
import threading
import time

import pytest


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from tracking.webxr import (
    QuestWebXRServer,
    WEBXR_25_TO_MEDIAPIPE_21,
    WEBXR_HAND_JOINT_NAMES,
    WebXRCalibrationTransform,
    WebXRProtocolError,
    derive_openxr26_joints,
    parse_webxr_message,
)


def _hand(side: str = "left", *, invalid_index: int | None = None) -> dict:
    joints = []
    for index, name in enumerate(WEBXR_HAND_JOINT_NAMES):
        valid = index != invalid_index
        joints.append(
            {
                "name": name,
                "valid": valid,
                "position": [index * 0.01, index * 0.01 + 0.1, index * 0.01 + 0.2]
                if valid
                else None,
                "orientation": [0.0, 0.0, 0.0, 1.0] if valid else None,
                "radius": 0.005 if valid else None,
            }
        )
    return {
        "handedness": side,
        "tracked": invalid_index is None,
        "valid_joint_count": 25 if invalid_index is None else 24,
        "joints": joints,
    }


def _message(*hands: dict, sequence: int = 7) -> dict:
    return {
        "schema": "teleop.webxr.hand_skeleton.v1",
        "type": "hand_skeleton",
        "joint_set": "webxr-hand-25",
        "joint_names": list(WEBXR_HAND_JOINT_NAMES),
        "sequence": sequence,
        "timestamp_ms": 1_700_000_000_123,
        "client_perf_ms": 321.5,
        "xr_predicted_display_time_ms": 330.0,
        "coordinate_space": "webxr_local_floor",
        "connection_state": "open",
        "hands": list(hands),
    }


def _wait_until(predicate, timeout: float = 2.0) -> bool:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(0.01)
    return bool(predicate())


def _masked_frame(opcode: int, payload: bytes) -> bytes:
    first = 0x80 | opcode
    length = len(payload)
    if length < 126:
        header = bytes((first, 0x80 | length))
    elif length <= 0xFFFF:
        header = bytes((first, 0x80 | 126)) + struct.pack("!H", length)
    else:
        header = bytes((first, 0x80 | 127)) + struct.pack("!Q", length)
    mask = os.urandom(4)
    masked = bytes(value ^ mask[index & 3] for index, value in enumerate(payload))
    return header + mask + masked


def _connect_websocket(port: int) -> ssl.SSLSocket:
    context = ssl.SSLContext(ssl.PROTOCOL_TLS_CLIENT)
    context.check_hostname = False
    context.verify_mode = ssl.CERT_NONE
    raw = socket.create_connection(("127.0.0.1", port), timeout=2.0)
    sock = context.wrap_socket(raw, server_hostname="localhost")
    sock.settimeout(2.0)
    key = base64.b64encode(os.urandom(16)).decode("ascii")
    request = (
        "GET /hand-skeleton HTTP/1.1\r\n"
        f"Host: 127.0.0.1:{port}\r\n"
        f"Origin: https://127.0.0.1:{port}\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        f"Sec-WebSocket-Key: {key}\r\n"
        "Sec-WebSocket-Version: 13\r\n\r\n"
    )
    sock.sendall(request.encode("ascii"))
    response = bytearray()
    while b"\r\n\r\n" not in response:
        response.extend(sock.recv(4096))
    assert response.startswith(b"HTTP/1.1 101")
    return sock


def test_preserves_native_webxr25_and_explicitly_maps_to_21() -> None:
    parsed = parse_webxr_message(_message(_hand("left")))
    assert len(parsed.frames) == 1
    frame = parsed.frames[0]
    assert frame.tracked is True
    assert frame.source_joint_count == 25
    assert frame.valid_joint_count == 25
    assert [joint.name for joint in frame.joints] == list(WEBXR_HAND_JOINT_NAMES)
    assert frame.coordinate_space == "webxr_local_floor"
    expected = tuple(frame.joints[index].position for index in WEBXR_25_TO_MEDIAPIPE_21)
    assert frame.mediapipe_landmarks() == expected
    compat = frame.to_compat_frame()
    assert compat.source_joint_count == 25
    assert compat.tracked is True
    assert compat.landmarks == expected


def test_partial_joint_validity_is_preserved_but_not_used_as_live_pose() -> None:
    frame = parse_webxr_message(_message(_hand("right", invalid_index=12))).frames[0]
    assert frame.source_joint_count == 25
    assert frame.valid_joint_count == 24
    assert frame.joints[12].valid is False
    assert frame.tracked is False
    assert frame.mediapipe_landmarks() == ()
    assert frame.to_compat_frame().tracked is False


def test_rejects_wrong_joint_order_and_openxr26_mislabel() -> None:
    wrong_order = _message(_hand())
    wrong_order["hands"][0]["joints"][1]["name"] = "thumb-tip"
    with pytest.raises(WebXRProtocolError, match="order must be fixed"):
        parse_webxr_message(wrong_order)

    wrong_set = _message(_hand())
    wrong_set["joint_set"] = "openxr-hand-26"
    with pytest.raises(WebXRProtocolError, match="do not label WebXR data as OpenXR-26"):
        parse_webxr_message(wrong_set)


def test_derived_palm_is_marked_and_does_not_mutate_raw_frame() -> None:
    frame = parse_webxr_message(_message(_hand())).frames[0]
    compatibility = derive_openxr26_joints(frame)
    assert len(frame.joints) == 25
    assert len(compatibility) == 26
    assert compatibility[0].name == "palm"
    assert compatibility[0].derived is True
    assert all(joint.derived is False for joint in frame.joints)
    source_indices = (0, 1, 5, 10, 15, 20)
    expected_x = sum(frame.joints[index].position[0] for index in source_indices) / 6
    assert compatibility[0].position[0] == pytest.approx(expected_x)


def test_fixed_calibration_is_a_derived_view_and_raw_local_floor_is_unchanged() -> None:
    frame = parse_webxr_message(_message(_hand())).frames[0]
    raw_wrist = frame.joints[0]
    calibration = WebXRCalibrationTransform(
        yaw_degrees=180.0,
        translation_m=(1.0, 0.0, 2.0),
    )
    derived = calibration.apply(frame)

    assert frame.coordinate_space == "webxr_local_floor"
    assert frame.joints[0] is raw_wrist
    assert frame.joints[0].derived is False
    assert derived.coordinate_space.startswith("derived(webxr_local_floor")
    assert all(joint.derived for joint in derived.joints)
    assert derived.joints[0].position == pytest.approx((1.0, 0.1, 1.8))
    compat = frame.to_compat_frame(calibration)
    assert compat.coordinate_space.startswith("derived(webxr_local_floor")
    assert compat.source_joint_count == 25


def test_tls_wss_latest_data_disconnect_cleanup_and_ten_reconnects(tmp_path: Path) -> None:
    delivered = []
    delivered_lock = threading.Lock()

    def on_frame(frame) -> None:
        with delivered_lock:
            delivered.append(frame)

    server = QuestWebXRServer(on_frame=on_frame)
    host, port = server.start(
        "127.0.0.1",
        0,
        cert_path=tmp_path / "server.crt",
        key_path=tmp_path / "server.key",
        advertise_hosts=("127.0.0.1",),
    )
    assert host == "127.0.0.1"
    assert port > 0
    assert (tmp_path / "quest_webxr_ca.crt").is_file()
    try:
        for cycle in range(10):
            sock = _connect_websocket(port)
            payload = json.dumps(
                _message(_hand("left"), sequence=cycle),
                separators=(",", ":"),
            ).encode("utf-8")
            sock.sendall(_masked_frame(0x1, payload))
            expected_tracked = cycle * 2 + 1
            assert _wait_until(lambda: len(delivered) >= expected_tracked)
            assert delivered[expected_tracked - 1].tracked is True
            sock.sendall(_masked_frame(0x8, struct.pack("!H", 1000)))
            sock.close()
            expected_lost = cycle * 2 + 2
            assert _wait_until(lambda: len(delivered) >= expected_lost)
            assert delivered[expected_lost - 1].tracked is False
            assert delivered[expected_lost - 1].event == "tracking_lost"
            assert _wait_until(lambda: server.health_snapshot()["connections"] == 0)

        health = server.health_snapshot()
        assert health["status"] == "ok"
        assert health["joint_set"] == "webxr-hand-25"
        assert health["native_joint_count"] == 25
        assert health["frames_received"] == 10
        assert health["invalid_messages"] == 0

        # A server-side stop must also clear a currently tracked hand; it may
        # never leave the last good frame pretending to be live.
        sock = _connect_websocket(port)
        sock.sendall(
            _masked_frame(
                0x1,
                json.dumps(_message(_hand("right"), sequence=99)).encode("utf-8"),
            )
        )
        assert _wait_until(lambda: len(delivered) >= 21)
        server.stop()
        assert _wait_until(lambda: len(delivered) >= 22)
        assert delivered[-1].hand_side == "right"
        assert delivered[-1].tracked is False
        sock.close()
    finally:
        server.stop()


def test_continuous_50hz_stream_meets_rate_and_p95_metrics(tmp_path: Path) -> None:
    server = QuestWebXRServer(on_frame=lambda _frame: None)
    _host, port = server.start(
        "127.0.0.1",
        0,
        cert_path=tmp_path / "timing.crt",
        key_path=tmp_path / "timing.key",
        advertise_hosts=("127.0.0.1",),
    )
    sock = _connect_websocket(port)
    try:
        for sequence in range(45):
            payload = json.dumps(
                _message(_hand("left"), sequence=sequence), separators=(",", ":")
            ).encode("utf-8")
            sock.sendall(_masked_frame(0x1, payload))
            time.sleep(0.02)
        assert _wait_until(lambda: server.health_snapshot()["frames_received"] == 45)
        health = server.health_snapshot()
        assert health["timing_sample_count"] == 44
        assert health["effective_rate_hz"] >= 30.0
        assert health["frame_interval_p95_ms"] < 80.0
    finally:
        try:
            sock.sendall(_masked_frame(0x8, struct.pack("!H", 1000)))
        except OSError:
            pass
        sock.close()
        server.stop()
