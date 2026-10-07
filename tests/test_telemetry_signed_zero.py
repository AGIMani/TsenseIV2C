"""IEEE-754 negative zero must not become a million-mm legacy integer."""
import json
import math
import struct
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from usb.telemetry import decode_telem_slot_float_mm, parse_hand_telem_v2
from utils.control_session_recorder import ControlSessionRecorder
from test_usb_v3_protocol_sync import _telem_payload


@pytest.mark.parametrize("raw,expected", [
    (bytes.fromhex("00000080"), -0.0),
    (struct.pack("<f", 0.0), 0.0),
    (struct.pack("<f", -2.75), -2.75),
    (struct.pack("<f", 16.25), 16.25),
    (bytes.fromhex("27f70000"), 63.271),  # Documented old integer encoding.
    (struct.pack("<i", -1234), -1.234),
])
def test_position_decoder_preserves_signed_zero_and_legacy_integers(raw, expected):
    assert decode_telem_slot_float_mm(raw) == pytest.approx(expected)


def test_negative_zero_in_actual_and_reference_positions(tmp_path):
    payload = bytearray(_telem_payload(6937))
    for index in range(14):
        struct.pack_into("<f", payload, 8 + 32 * index + 4, -0.0)
        struct.pack_into("<f", payload, 8 + 32 * index + 16, -0.0)
    frame = parse_hand_telem_v2(bytes(payload))
    for slot in frame["linear"]:
        assert slot.pos_act_mm == slot.pos_ref_mm == slot.pos_err_mm == 0
        assert math.copysign(1, slot.pos_ref_mm) == -1
    recorder = ControlSessionRecorder()
    path = recorder.start(output_dir=tmp_path)
    recorder.record_telemetry_payload(bytes(payload), accepted=True)
    recorder.stop()
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    assert all(s["pos_ref_mm"] == s["pos_act_mm"] == 0 for s in rows[1]["linear"])
    assert rows[1]["payload_hex"] == bytes(payload).hex()
    assert recorder.error is None
