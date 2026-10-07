"""Verify actual online wire frames and displayed ranges without hardware."""
from dataclasses import replace
import math
import struct
from types import SimpleNamespace

import pytest
import server
from test_server import bridge
from downlink_policy import OnlineUsbController, encode_online_targets, negative_linear, v2_ranges
from a30_kinematics import A30StrokeMapper
from a30_kinematics.downlink import channel_mapping, crc16_modbus
from a30_session import A30Session
from protocol.hand_angle_protocol import convert_ui_to_hardware
from usb import offline


@pytest.mark.parametrize("side", ["left", "right"])
def test_v2_joint_frame_forces_negative_and_preserves_rotary(bridge, monkeypatch, side):
    usb = bridge.v2.usb
    assert isinstance(usb, OnlineUsbController)
    usb._connected, usb._ser = True, SimpleNamespace(is_open=True)
    usb._hand_type_cache = int(side == "right")
    frames, conversions = [], []
    monkeypatch.setattr(usb, "_write_frame", lambda frame, **kwargs: frames.append(frame) or True)
    monkeypatch.setattr(usb._session_recorder, "record_conversion", lambda angles, linear, rotary, **kw: conversions.append(linear[:]))
    linear = [0., -1., 2., -3., 4., -5., 6., -7., 8., -9., 10., -11., 12., -13.]
    rotary = [-20., 21., -22., 23., 24., 25., -26.]
    monkeypatch.setattr(offline, "joint_angles_to_hand_positions", lambda pose: (linear, rotary))
    try:
        for stream in (False, True):
            assert usb.send_joint_angles({"left_hand_index_MCP_joint": .5}, hand_side=side, stream=stream)
        for frame in frames:
            assert struct.unpack_from("<14f", frame, 7) == pytest.approx([-abs(v) for v in linear])
            assert struct.unpack_from("<7f", frame, 63) == pytest.approx(rotary)
            assert struct.unpack_from("<H", frame, len(frame)-2)[0] == crc16_modbus(frame[:-2])
        assert conversions == [[-abs(v) for v in linear]] * 2
        assert offline.UsbController().normalize_linear_positions(linear) == linear
    finally:
        usb._connected, usb._ser = False, None


@pytest.mark.parametrize("side", ["left", "right"])
def test_v2_ranges_match_wire_and_do_not_change_live_mapper(bridge, monkeypatch, side):
    joints = bridge.snapshot()["joints"]
    mapper = offline.get_hand_stroke_mapper()
    before = {name: mapper.last_stroke_mm(name) for name in offline.LINEAR_JOINT_SLOTS}
    ranges = v2_ranges(side, joints)
    assert {name: mapper.last_stroke_mm(name) for name in before} == before
    for fraction in (0., .25, .5, .75, 1.):
        pose = {j["name"]: j["min"] + (j["max"] - j["min"]) * fraction for j in joints}
        linear, rotary = offline.joint_angles_to_hand_positions(convert_ui_to_hardware(pose, hand_side=side))
        for name, spec in ranges.items():
            value = negative_linear(linear[offline.LINEAR_JOINT_SLOTS[name]]) if spec["unit"] == "mm" else rotary[offline.ROTARY_JOINT_SLOTS[name]]
            assert spec["min"] - 1e-6 <= value <= spec["max"] + 1e-6
            if fraction in (0., 1.):
                assert value == pytest.approx(spec["at_ui_min" if fraction == 0 else "at_ui_max"])
    assert ranges["left_hand_pinky_rota_joint"]["min" if side == "right" else "max"] == pytest.approx(-50 if side == "right" else 50)


@pytest.mark.parametrize("side", ["left", "right"])
def test_a30_negative_strokes_ranges_and_external_reduction(tmp_path, side):
    session = A30Session(tmp_path, side)
    try:
        mapper = session.mapper
        specs = {j["name"]: j for j in session.snapshot()["joints"]}
        for fraction in (0., .25, .5, .75, 1.):
            targets = mapper.convert({name: lo + (hi-lo)*fraction for name, (lo, hi) in zip(mapper.joint_names, mapper._limits)})
            for sign in (1, -1):
                signed = replace(targets, linear_strokes_mm={n: sign*v for n, v in targets.linear_strokes_mm.items()})
                frame = encode_online_targets(signed, hand_side=side)
                assert struct.unpack_from("<H", frame, 106)[0] == crc16_modbus(frame[:106])
                for row in channel_mapping(side):
                    offset = 7 + 4*(row["slot"] if row["field"] == "self_pos" else 18 + row["slot"])
                    value = struct.unpack_from("<f", frame, offset)[0]
                    expected = -abs(targets.linear_strokes_mm[row["joint"]]) if row["unit"] == "mm" else targets.rotary_positions_deg[row["joint"]] * row["gear_ratio"]
                    assert value == pytest.approx(expected)
                    spec = specs[row["joint"]]["downlink"]
                    assert spec["min"] - 1e-5 <= value <= spec["max"] + 1e-5
    finally:
        session.close()


@pytest.mark.parametrize("value", [math.nan, math.inf, -math.inf, True, "1"])
def test_invalid_linear_values_rejected(value):
    with pytest.raises(ValueError):
        negative_linear(value)
