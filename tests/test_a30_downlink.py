"""Source wire compatibility without importing/running the Tk/USB application."""
from pathlib import Path
import ast
import math
import struct
import sys
from fractions import Fraction
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from a30_kinematics import downlink as wire
from hand_profiles import load_hand_profile


@pytest.fixture(scope="module")
def source():
    tree = ast.parse((ROOT / "tests/fixtures/a30_protocol_reference.py").read_text(encoding="utf-8-sig"))
    names = {"_crc16_modbus_table", "crc16_modbus", "pack_aa55", "pack_self_pwr",
             "pack_self_homing", "pack_self_pos_pid", "pack_self_cur_pid",
             "pack_self_open_spin", "pack_hand_batch_into", "pack_hand_batch"}
    constants = {"SELF_MOTOR_NUM", "MIT_MOTOR_NUM", "DEV_TELEM", "FUN_HAND_POS_BATCH",
                 "FUN_SELF_PWR", "FUN_SELF_HOMING", "FUN_SELF_POS_PID", "FUN_SELF_CUR_PID",
                 "FUN_SELF_OPEN_SPIN", "HAND_BATCH_PAYLOAD_LEN", "HAND_BATCH_WIRE_LEN",
                 "MIT_GEAR_RATIO", "CRC16_MODBUS_TAB", "HAND_BATCH_HDR"}
    selected = [n for n in tree.body if (isinstance(n, ast.FunctionDef) and n.name in names)
                or (isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id in constants for t in n.targets))]
    ns = {"struct": struct}
    exec(compile(ast.Module(body=selected, type_ignores=[]), "source_pure_functions", "exec"), ns)
    return SimpleNamespace(**ns)


@pytest.mark.parametrize("side", ["left", "right"])
def test_named_order_and_complete_bytes_match_original(source, side):
    linear_order = ["pinky_PIP", "pinky_MCP", "ring_PIP", "ring_MCP",
                    "mid_PIP", "mid_MCP", "index_PIP", "index_MCP", "thumb_MCP"]
    rotary_order = ["pinky_DIP", "pinky_swing", "ring_DIP", "ring_swing", "mid_DIP",
                    "mid_swing", "index_DIP", "index_swing", "thumb_PIP",
                    "thumb_rotaback_joint2", "thumb_rota_block_joint"]
    name = lambda suffix: f"{side}_hand_" + (suffix if "joint" in suffix else suffix + "_joint")
    linear = {name(n): i + .25 for i, n in enumerate(linear_order)}
    rotary = {name(n): -(i + 1) * 2.5 for i, n in enumerate(rotary_order)}
    targets = SimpleNamespace(linear_strokes_mm=linear, rotary_positions_deg=rotary)
    for seq in (0, 255, 256):
        actual = wire.encode_targets(targets, hand_side=side, seq=seq,
                                     mit_vel_rpm=(1.5, -2), mit_iq_ff_a=(.2, -.3))
        expected_self = [-2.5, -5, -7.5, -10, -.25, -1.25, -2.25, -3.25,
                         -12.5, -15, -17.5, -20, -4.25, -5.25, -6.25, -7.25, -22.5, -8.25]
        expected = source.pack_hand_batch(seq, expected_self, [-25*50, -27.5*30], [75, -60], [.2, -.3])
        assert wire.MIT_GEAR_RATIOS == (50, 30)
        assert actual == expected and len(actual) == 108
        assert actual[:6] == b"\xaa\x55\x64\x00\x03\x20"
        assert actual[103:106] == bytes(3)
        assert struct.unpack_from("<H", actual, 106)[0] == source.crc16_modbus(actual[:106])
    rows = wire.channel_mapping(side)
    assert [r["bus"] for r in rows] == [0]*8 + [1]*8 + [2]*4
    assert [r["motor"] for r in rows] == list(range(1, 9))*2 + [1, 2, 1, 2]
    assert [r["can_id"] for r in rows[-2:]] == [0x505, 0x509]


@pytest.mark.parametrize("fn,args", [
    ("pack_self_pwr", (0, 1, 1)), ("pack_self_pwr", (255, 255, 0, 3)),
    ("pack_self_homing", (1, 6, 1, 120.)), ("pack_self_pos_pid", (2, 2, 1., 2., 3.)),
    ("pack_self_cur_pid", (0, 5, 1., 2., 3., 4., 5., 6.)),
    ("pack_self_open_spin", (1, 2, 1, .5, 2.)),
])
def test_single_motor_builders_match_original(source, fn, args):
    assert getattr(wire, fn)(*args) == getattr(source, fn)(*args)










@pytest.mark.parametrize("value", [0, .25, Fraction(1, 4)])
def test_encoder_builtin_fast_path_preserves_real_numeric_values(value):
    assert wire._finite_float32(value) == float(value)


@pytest.mark.parametrize("bad", [False, None, complex(1, 2), -float("inf")])
def test_encoder_builtin_fast_path_keeps_invalid_value_rejection(bad):
    with pytest.raises(ValueError):
        wire._finite_float32(bad)
