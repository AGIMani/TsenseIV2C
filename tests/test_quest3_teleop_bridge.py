from __future__ import annotations

import json
import sys
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from quest3_teleop_bridge import (
    OPENXR_26_TO_FORM_21,
    OPENXR_HAND_JOINT_NAMES,
    TeleopHandPacketEncoder,
    encode_udp_packet,
    prepare_teleop_hand_snapshot,
)
from tracking.cloudxr import (
    OPENXR_26_TO_MEDIAPIPE_21,
    parse_cloudxr_packet,
)


def _snapshot() -> dict[str, object]:
    return {
        "joint_names": list(OPENXR_HAND_JOINT_NAMES),
        "joint_positions_xyz": [
            [float(index), float(index) + 0.1, float(index) + 0.2]
            for index in range(26)
        ],
        "joint_valid": [True] * 26,
    }


def test_bridge_mapping_stays_identical_to_form_receiver() -> None:
    assert OPENXR_26_TO_FORM_21 == OPENXR_26_TO_MEDIAPIPE_21


def test_valid_teleop_snapshot_round_trips_through_form_parser() -> None:
    encoder = TeleopHandPacketEncoder(hand_side="right")
    packet = encoder.packet_for_snapshot(
        _snapshot(), timestamp_ns=123_000_000, monotonic_s=1.0
    )

    assert packet is not None
    assert packet["tracked"] is True
    payload = encode_udp_packet(packet)
    assert json.loads(payload)["schema"] == "hand.quest3.openxr26.v1"

    frame = parse_cloudxr_packet(payload)
    assert frame.hand_side == "right"
    assert frame.timestamp_ms == 123
    assert frame.source_joint_count == 26
    assert frame.landmarks == tuple(
        tuple(_snapshot()["joint_positions_xyz"][index])  # type: ignore[index]
        for index in OPENXR_26_TO_FORM_21
    )


def test_snapshot_joint_names_control_reordering() -> None:
    canonical = _snapshot()
    order = list(reversed(range(26)))
    shuffled = {
        "joint_names": [OPENXR_HAND_JOINT_NAMES[index] for index in order],
        "joint_positions_xyz": [
            canonical["joint_positions_xyz"][index]  # type: ignore[index]
            for index in order
        ],
        "joint_valid": [True] * 26,
    }

    prepared = prepare_teleop_hand_snapshot(shuffled)
    assert [point[0] for point in prepared.positions] == [float(index) for index in range(26)]


def test_invalid_required_joint_uses_grace_then_emits_one_loss_packet() -> None:
    encoder = TeleopHandPacketEncoder(hand_side="left", loss_timeout_s=0.35)
    assert encoder.packet_for_snapshot(_snapshot(), monotonic_s=10.0) is not None

    invalid = _snapshot()
    invalid["joint_valid"][10] = False  # type: ignore[index]
    assert encoder.packet_for_snapshot(invalid, monotonic_s=10.10) is None

    loss = encoder.packet_for_snapshot(invalid, monotonic_s=10.36)
    assert loss is not None
    assert loss["tracked"] is False
    assert loss["handedness"] == "left"
    assert encoder.packet_for_snapshot(invalid, monotonic_s=10.80) is None


def test_invalid_dropped_openxr_joint_is_sanitized_but_frame_remains_tracked() -> None:
    snapshot = _snapshot()
    snapshot["joint_positions_xyz"][0] = [float("nan"), 0.0, 0.0]  # type: ignore[index]
    encoder = TeleopHandPacketEncoder()

    packet = encoder.packet_for_snapshot(snapshot, monotonic_s=2.0)

    assert packet is not None
    assert packet["tracked"] is True
    assert packet["joint_valid"][0] is False
    assert packet["joints"][0] == [0.0, 0.0, 0.0]
    encode_udp_packet(packet)


def test_missing_hand_emits_loss_once_and_recovers() -> None:
    encoder = TeleopHandPacketEncoder()

    first_loss = encoder.packet_for_snapshot(None, monotonic_s=0.0)
    assert first_loss is not None and first_loss["tracked"] is False
    assert encoder.packet_for_snapshot(None, monotonic_s=1.0) is None

    recovered = encoder.packet_for_snapshot(_snapshot(), monotonic_s=2.0)
    assert recovered is not None and recovered["tracked"] is True
    final_loss = encoder.force_tracking_lost(timestamp_ns=999)
    assert final_loss is not None and final_loss["tracked"] is False
    assert final_loss["timestamp_ns"] == 999
