"""Exercise production conversion, serial I/O and RX framing without hardware."""
import json
import struct
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from usb import offline
from usb.telemetry import LEGACY_TELEM_FRAME_SIZE
from utils.control_session_recorder import ControlSessionRecorder
from test_usb_v3_protocol_sync import _telem_payload, _uplink_frame


class SerialStub:
    is_open = True
    out_waiting = 0

    def __init__(self):
        self.frames = []

    def write(self, frame):
        self.frames.append(bytes(frame))
        return len(frame)

    def flush(self):
        pass

    def close(self):
        self.is_open = False


def read_rows(path):
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]


def test_usb_conversion_tx_and_all_rx_frames_are_correlated(tmp_path, monkeypatch):
    recorder = ControlSessionRecorder()
    monkeypatch.setattr(offline, "_control_session_recorder", recorder)
    ctrl = offline.UsbController()
    ctrl._ser = serial = SerialStub()
    ctrl._connected = True
    ctrl._hand_type_cache = 1
    path = recorder.start(output_dir=tmp_path)
    joints = {"left_hand_index_MCP_joint": 0.3, "left_hand_thumb_rota_joint": 0.2}
    assert ctrl.send_joint_angles(joints, source="test", hand_side="right", frame_id=42)
    # Backpressure must produce a failed attempt, never replace successful command correlation.
    serial.out_waiting = offline.SERIAL_TX_BACKPRESSURE_BYTES + 1
    assert not ctrl.send_joint_angles(joints, stream=True, hand_side="right", frame_id=43)
    serial.out_waiting = 0
    payload = bytearray(_telem_payload(65535))
    struct.pack_into("<I", payload, 8 + 28, 3650 | (2 << 16) | (3 << 24))
    frame = _uplink_frame(bytes(payload))
    # Fragmentation, sequence wrap, duplicates and multiple frames in one chunk.
    ctrl._process_uplink_chunk(frame[:13])
    ctrl._process_uplink_chunk(frame[13:] + _uplink_frame(_telem_payload(0)) + _uplink_frame(_telem_payload(0)))
    ctrl._process_uplink_chunk(_uplink_frame(_telem_payload(1)))
    corrupt = bytearray(_uplink_frame(_telem_payload(2)))
    corrupt[-1] ^= 1
    ctrl._process_uplink_chunk(corrupt)
    for _ in range(3):
        assert ctrl.read_latest_telemetry()["seq"] == 1
    recorder.stop(reason="test")
    rows = read_rows(path)
    conversion = [r for r in rows if r["type"] == "conversion"]
    downlink = [r for r in rows if r["type"] == "downlink"]
    feedback = [r for r in rows if r["type"] == "telemetry"]
    assert len(conversion) == len(downlink) == 2
    assert downlink[0]["command_id"] == conversion[0]["seq"]
    assert downlink[0]["frame_id"] == 42
    assert [r["sent"] for r in downlink] == [True, False]
    assert bytes.fromhex(downlink[0]["wire_hex"]) == serial.frames[0]
    expected = struct.unpack_from("<B14f7f", serial.frames[0], 6)
    assert downlink[0]["linear_mm"] == pytest.approx(expected[1:15])
    assert downlink[0]["rotary_deg"] == pytest.approx(expected[15:22])
    assert [r["telemetry_seq"] for r in feedback] == [65535, 0, 0, 1]
    assert [r["accepted"] for r in feedback] == [True, True, False, True]
    assert all(r["latest_host_command"]["frame_id"] == 42 for r in feedback)
    first = feedback[0]["linear"][0]
    assert first["pos_act_mm"] == 10
    assert first["pos_ref_mm"] == 12
    assert first["iq_meas"] == 100
    assert first["iq_ref"] == 110
    assert first["grav_iq"] == 1.5
    assert first["mcu_temp_c"] == 36.5
    assert first["can_rx_error_cnt"] == 2
    assert first["can_tx_error_cnt"] == 3
    assert feedback[0]["rotary"][0]["pos_ref_mm"] == 20
    assert "mcu_temp_c" not in feedback[0]["rotary"][0]
    assert feedback[0]["board"]["imu_roll_deg"] == 1
    assert rows[-1]["dropped_events"] == 0
    assert recorder.error is None


def test_legacy_telemetry_and_nonfinite_values_are_not_invented(tmp_path):
    recorder = ControlSessionRecorder()
    path = recorder.start(output_dir=tmp_path)
    slot = struct.pack("<B3x10f", 2, float("nan"), 0, 1.5, 0, 2, 1, 1, 0, 0, 0)
    payload = struct.pack("<HHI", 9, 0, 500) + slot * 21
    assert len(payload) == LEGACY_TELEM_FRAME_SIZE
    recorder.record_telemetry_payload(payload, accepted=True)
    recorder.stop()
    row = read_rows(path)[1]
    assert row["linear"][0]["iq_meas"] is None
    assert row["linear"][0]["mcu_temp_c"] is None
    assert row["latest_host_command"] is None
    assert row["payload_hex"] == payload.hex()
    assert "NaN" not in path.read_text()


def test_raw_mediapipe_keeps_all_hands_and_empty_detections(tmp_path):
    recorder = ControlSessionRecorder()
    path = recorder.start(output_dir=tmp_path)
    point = SimpleNamespace(x=0.1, y=0.2, z=0.3)
    result = SimpleNamespace(hand_landmarks=[[point] * 21] * 2,
                             hand_world_landmarks=[[point] * 21] * 2,
                             handedness=[[SimpleNamespace(category_name="Right", score=0.9)]] * 2)
    recorder.record_mediapipe_raw(result, frame_id=1, timestamp_ms=33, infer_ms=4.5)
    result.hand_landmarks = result.hand_world_landmarks = result.handedness = []
    recorder.record_mediapipe_raw(result, frame_id=2, timestamp_ms=66, infer_ms=4.0)
    recorder.stop()
    rows = read_rows(path)
    assert len(rows[1]["world_landmarks"]) == 2
    assert len(rows[1]["normalized_landmarks"][0]) == 21
    assert rows[2]["world_landmarks"] == []


def test_short_write_is_not_reported_as_success(tmp_path, monkeypatch):
    recorder = ControlSessionRecorder()
    monkeypatch.setattr(offline, "_control_session_recorder", recorder)
    ctrl = offline.UsbController()
    ctrl._ser = SerialStub()
    ctrl._ser.write = lambda frame: len(frame) - 1
    ctrl._connected = True
    path = recorder.start(output_dir=tmp_path)
    assert not ctrl.send_hand_positions([0.0] * 14, [0.0] * 7)
    recorder.stop()
    rows = read_rows(path)
    assert next(r for r in rows if r["type"] == "downlink")["sent"] is False
    assert next(r for r in rows if r["type"] == "usb_tx")["sent"] is False
