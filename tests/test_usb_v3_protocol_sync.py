"""deprecated/main_app_v3.py 与当前 USB 控制链的关键协议回归。"""
from __future__ import annotations

import struct
import sys
import threading
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from usb.offline import (  # noqa: E402
    DEV_TELEM,
    FUN_GRAVITY_ALLOW_NO_IMU,
    FUN_GRAVITY_MASTER_ENABLE,
    FUN_RM_GRAVITY_ENABLE,
    FUN_RM_ELEC_CALIB_SP,
    FUN_RM_ELEC_OFFSET_SP,
    FUN_HAND_POS_BATCH,
    FUN_HEARTBEAT,
    FUN_TELEM,
    UsbController,
    crc16_modbus,
    pack_gravity_allow_no_imu,
    pack_gravity_master_enable,
    pack_hand_pos_batch,
    pack_rm_elec_calib_sp,
    pack_rm_elec_offset_sp,
    pack_rm_gravity_enable,
    build_downlink,
)
from usb.telemetry import (  # noqa: E402
    HEARTBEAT_MAGIC,
    LINEAR_MOTOR_NUM,
    ROTARY_MOTOR_NUM,
    TELEM_FRAME_SIZE,
    materialize_telem_frame,
    parse_currents_raw_slot11,
    parse_hand_telem_lazy,
    parse_heartbeat,
    telem_linear_fingerprint,
    telem_seq_accept,
)


def _telem_payload(seq: int, *, tick_ms: int = 1234, hand_type: int = 1) -> bytes:
    payload = bytearray(struct.pack("<HHI", seq & 0xFFFF, 0x0201, tick_ms))
    for idx in range(LINEAR_MOTOR_NUM):
        payload.extend(
            struct.pack(
                "<8f",
                0.0,
                10.0 + idx,
                100.0 + idx,
                110.0 + idx,
                12.0 + idx,
                1.5 + idx,
                0.0,
                0.0,
            )
        )
    rotary_off = len(payload)
    for idx in range(ROTARY_MOTOR_NUM):
        payload.extend(
            struct.pack(
                "<B3x10f",
                idx % 4,
                200.0 + idx,
                0.0,
                210.0 + idx,
                0.0,
                20.0 + idx,
                21.0 + idx,
                1.0,
                0.0,
                0.0,
                0.0,
            )
        )
    # rotary[0] reserved[0] 由固件用作 hand_type。
    payload[rotary_off + 1] = hand_type & 0xFF
    payload.extend(struct.pack("<HHfff", 1, 7, 1.0, 2.0, 3.0))
    assert len(payload) == TELEM_FRAME_SIZE
    return bytes(payload)


def _uplink_frame(payload: bytes, *, fun: int = FUN_TELEM) -> bytes:
    body = struct.pack("<BBHBB", 0x55, 0xAA, len(payload), DEV_TELEM, fun) + payload
    return body + struct.pack("<H", crc16_modbus(body))


def test_new_gravity_commands_match_v3_wire_layout() -> None:
    assert FUN_RM_GRAVITY_ENABLE == 0x1B
    assert FUN_GRAVITY_ALLOW_NO_IMU == 0x1C
    assert pack_rm_gravity_enable(1) == b"\x01\x00\x00\x00"
    assert pack_rm_gravity_enable(2) == b"\x02\x00\x00\x00"
    assert pack_gravity_allow_no_imu(1) == b"\x01\x00\x00\x00"
    assert FUN_GRAVITY_MASTER_ENABLE == 0x1D
    assert pack_gravity_master_enable(2) == b"\x02\x00\x00\x00"


def test_rotary_electric_calibration_commands_match_v3_wire_layout() -> None:
    assert FUN_RM_ELEC_CALIB_SP == 0x2C
    assert FUN_RM_ELEC_OFFSET_SP == 0x2D
    assert pack_rm_elec_calib_sp(3, 1, 2) == b"\x03\x01\x02\x00"
    assert pack_rm_elec_offset_sp(3, 1, 2, 12.5, -7.25) == struct.pack(
        "<BBBx2f", 3, 1, 2, 12.5, -7.25,
    )


def test_lazy_v2_telemetry_materializes_all_slots_and_imu() -> None:
    lazy = parse_hand_telem_lazy(_telem_payload(42))
    assert lazy is not None
    assert lazy["seq"] == 42
    assert lazy["linear"] is None

    frame = materialize_telem_frame(lazy)
    assert frame is not None
    assert frame["_parsed"] is True
    assert frame["hand_type"] == 1
    assert len(frame["linear"]) == LINEAR_MOTOR_NUM
    assert len(frame["rotary"]) == ROTARY_MOTOR_NUM
    assert frame["linear"][3].pos_act_mm == 13.0
    assert frame["linear"][3].iq_meas == 103.0
    assert frame["rotary"][4].pos_actual_mm == 25.0
    assert frame["imu_valid"] == 1
    assert frame["imu_yaw_deg"] == 3.0
    assert telem_linear_fingerprint(frame) != 0


def test_heartbeat_raw_currents_and_sequence_rules() -> None:
    hb = parse_heartbeat(struct.pack("<4I", HEARTBEAT_MAGIC, 9, 100, 11))
    assert hb == {"seq": 9, "tick_ms": 100, "hb_count": 11}
    assert parse_heartbeat(b"\x00" * 16) is None
    assert parse_currents_raw_slot11(0xFFFE0003) == (3, -2)

    assert telem_seq_accept(-1, 7)
    assert not telem_seq_accept(7, 7)
    assert telem_seq_accept(7, 8)
    assert not telem_seq_accept(7, 700)
    assert telem_seq_accept(0xFFFE, 1)


def test_usb_controller_rejects_duplicate_and_stale_telem_and_reports_once() -> None:
    ctrl = UsbController()
    ctrl._process_uplink_chunk(_uplink_frame(_telem_payload(100, tick_ms=1)))
    assert ctrl._telem_generation == 1

    # 同 seq 即使 payload 变了也不得覆盖最新权威帧。
    ctrl._process_uplink_chunk(_uplink_frame(_telem_payload(100, tick_ms=2)))
    assert ctrl._telem_generation == 1
    assert ctrl._telem_duplicate_or_stale == 1

    class Reporter:
        def __init__(self) -> None:
            self.frames = []

        def record_telemetry(self, frame, *, hand_side: str) -> None:
            self.frames.append((frame["seq"], hand_side))

    reporter = Reporter()
    ctrl.set_data_reporter(reporter)
    first = ctrl.read_latest_telemetry()
    second = ctrl.read_latest_telemetry()
    assert first is second
    assert reporter.frames == [(100, "right")]

    ctrl._process_uplink_chunk(_uplink_frame(_telem_payload(101, tick_ms=3)))
    assert ctrl.read_latest_telemetry()["seq"] == 101
    assert reporter.frames == [(100, "right"), (101, "right")]

    ctrl._process_uplink_chunk(_uplink_frame(_telem_payload(700, tick_ms=4)))
    assert ctrl._telem_generation == 2
    assert ctrl._telem_duplicate_or_stale == 2

    hb_payload = struct.pack("<4I", HEARTBEAT_MAGIC, 8, 900, 17)
    ctrl._process_uplink_chunk(_uplink_frame(hb_payload, fun=FUN_HEARTBEAT))
    heartbeat = ctrl.read_latest_heartbeat()
    assert heartbeat is not None
    assert heartbeat["seq"] == 8
    assert heartbeat["hb_count"] == 17


def test_rx_loop_parses_every_followup_drain_batch(monkeypatch) -> None:
    """回归：连续排空得到的 extra 不能只统计后直接丢弃。"""
    import usb.offline as offline

    chunks = [
        _uplink_frame(_telem_payload(200, tick_ms=10)),
        _uplink_frame(_telem_payload(201, tick_ms=20)),
    ]
    ctrl = UsbController()

    class FakeSerial:
        is_open = True

        @property
        def in_waiting(self) -> int:
            return 1 if chunks else 0

    fake = FakeSerial()
    ctrl._ser = fake
    ctrl._connected = True
    ctrl._rx_stop = threading.Event()

    def fake_drain(_ser):
        if chunks:
            return chunks.pop(0)
        ctrl._rx_stop.set()
        return b""

    monkeypatch.setattr(offline, "_drain_serial_port", fake_drain)
    monkeypatch.setattr(offline, "record_usb_rx", lambda **_kwargs: None)
    ctrl._rx_loop()

    assert ctrl._telem_generation == 2
    assert ctrl.read_latest_telemetry()["seq"] == 201


def test_controller_preallocated_hand_frame_is_wire_equivalent() -> None:
    class FakeSerial:
        is_open = True
        out_waiting = 0

        def __init__(self) -> None:
            self.frames = []

        def write(self, frame: bytes) -> int:
            self.frames.append(bytes(frame))
            return len(frame)

        def flush(self) -> None:
            return None

    ctrl = UsbController()
    fake = FakeSerial()
    ctrl._ser = fake
    ctrl._connected = True
    linear = [float(i) for i in range(LINEAR_MOTOR_NUM)]
    rotary = [float(-i) for i in range(ROTARY_MOTOR_NUM)]

    assert ctrl.send_hand_positions(linear, rotary, seq=23)
    expected = build_downlink(
        DEV_TELEM,
        FUN_HAND_POS_BATCH,
        pack_hand_pos_batch(23, linear, rotary),
    )
    assert fake.frames[-1] == expected

    # 相同位置走 seq-only 路径，线帧仍必须完全等价。
    assert ctrl.send_hand_positions(linear, rotary, seq=24)
    expected2 = build_downlink(
        DEV_TELEM,
        FUN_HAND_POS_BATCH,
        pack_hand_pos_batch(24, linear, rotary),
    )
    assert fake.frames[-1] == expected2


def test_emergency_stop_respects_thumb_h7_lower_bounds(monkeypatch) -> None:
    ctrl = UsbController()
    captured = {}

    def fake_send(linear, rotary, **kwargs):
        captured["linear"] = list(linear)
        captured["rotary"] = list(rotary)
        captured["source"] = kwargs.get("source")
        return True

    monkeypatch.setattr(ctrl, "send_hand_positions", fake_send)

    assert ctrl.emergency_stop()
    assert captured["linear"] == [0.0] * LINEAR_MOTOR_NUM
    assert captured["rotary"][:4] == [0.5] * 4  # 左手四指 swing / R00-R03 下限
    assert captured["rotary"][4] == 0.5  # thumb_rota / R04 下限
    assert captured["rotary"][5] == 0.0  # thumb_swing / R05 下限
    assert captured["source"] == "UsbController/急停"
