from __future__ import annotations

import json
import socket
import sys
import threading
from pathlib import Path

import pytest


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from tracking.cloudxr import (
    CloudXRLandmarkReceiver,
    CloudXRPacketError,
    OPENXR_26_TO_MEDIAPIPE_21,
    parse_cloudxr_packet,
)


def _points(count: int) -> list[list[float]]:
    return [[float(i), float(i) + 0.1, float(i) + 0.2] for i in range(count)]


def test_parses_mediapipe_21_packet_without_reordering() -> None:
    points = _points(21)
    frame = parse_cloudxr_packet(
        {
            "type": "hand_landmarks",
            "sequence": 42,
            "timestamp_ns": 12_345_000_000,
            "hand": "right",
            "space": "openxr_local",
            "unit": "m",
            "landmarks": points,
        }
    )

    assert frame.tracked is True
    assert frame.hand_side == "right"
    assert frame.sequence == 42
    assert frame.timestamp_ms == 12_345
    assert frame.source_joint_count == 21
    assert frame.landmarks == tuple(tuple(point) for point in points)


def test_maps_openxr_26_order_to_mediapipe_21() -> None:
    points = _points(26)
    frame = parse_cloudxr_packet(
        {"handedness": "left", "joints": points, "unit": "meters"}
    )

    assert frame.source_joint_count == 26
    assert len(frame.landmarks) == 21
    assert frame.landmarks == tuple(tuple(points[i]) for i in OPENXR_26_TO_MEDIAPIPE_21)


def test_nested_hands_selects_tracked_high_confidence_and_converts_cm() -> None:
    points = [
        {"position": {"x": i, "y": i + 1, "z": i + 2}}
        for i in range(21)
    ]
    frame = parse_cloudxr_packet(
        {
            "unit": "cm",
            "hands": [
                {"tracked": False, "side": "left"},
                {
                    "tracked": True,
                    "confidence": 0.9,
                    "side": "right",
                    "points": points,
                },
            ],
        }
    )

    assert frame.hand_side == "right"
    assert frame.landmarks[3] == pytest.approx((0.03, 0.04, 0.05))


def test_explicit_tracking_loss_packet_needs_no_landmarks() -> None:
    frame = parse_cloudxr_packet(
        {"tracked": False, "hand_side": "left", "sequence": 9}
    )
    assert frame.tracked is False
    assert frame.landmarks == ()
    assert frame.sequence == 9

    empty_frame = parse_cloudxr_packet({"tracked": False, "joints": []})
    assert empty_frame.tracked is False
    assert empty_frame.landmarks == ()


def test_rejects_invalid_timestamp_without_stopping_receiver_parser() -> None:
    with pytest.raises(CloudXRPacketError, match="timestamp"):
        parse_cloudxr_packet(
            {"timestamp_ns": "not-a-number", "landmarks": _points(21)}
        )


@pytest.mark.parametrize("count", [0, 20, 22, 25, 27])
def test_rejects_unsupported_joint_counts(count: int) -> None:
    with pytest.raises(CloudXRPacketError, match="expected 21 MediaPipe or 26 OpenXR"):
        parse_cloudxr_packet({"landmarks": _points(count)})


def test_udp_receiver_delivers_valid_frame() -> None:
    delivered = []
    ready = threading.Event()
    receiver = CloudXRLandmarkReceiver(
        on_frame=lambda frame: (delivered.append(frame), ready.set()),
    )
    host, port = receiver.start("127.0.0.1", 0)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sender.sendto(b'{"timestamp_ns":"bad"}', (host, port))
        sender.sendto(
            json.dumps({"sequence": 3, "landmarks": _points(21)}).encode("utf-8"),
            (host, port),
        )
        assert ready.wait(1.5)
        assert delivered[0].sequence == 3
    finally:
        sender.close()
        receiver.stop()
