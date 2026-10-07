"""Pure A3 wire builders extracted from A3_USB_Downlink.pyw.

No Tk, serial connection, background sender, OTA or firmware update code.
Batch builder inputs are already wire-side (signed linear strokes, MIT angles/speeds).
Use encode_targets for named A3.0 joint outputs and exactly-once reduction.
"""
from __future__ import annotations
from functools import lru_cache
import math
from numbers import Real
import struct
from typing import Mapping

SELF_MOTOR_NUM = 18
MIT_MOTOR_NUM = 2
DEV_TELEM = 3
FUN_HAND_POS_BATCH = 0x20
FUN_SELF_PWR = 0x21
FUN_SELF_HOMING = 0x22
FUN_SELF_POS_PID = 0x23
FUN_SELF_CUR_PID = 0x24
FUN_SELF_OPEN_SPIN = 0x25
HAND_BATCH_PAYLOAD_LEN = 100
HAND_BATCH_WIRE_LEN = 108
MIT_GEAR_RATIOS = (50.0, 30.0)  # CAN3 M1 ROTABACK / M2 ROTA_BLOCK
HAND_BATCH_HDR = bytes([0xAA, 0x55]) + struct.pack("<HBB", 100, DEV_TELEM, FUN_HAND_POS_BATCH)

def slot_to_bus_node(slot: int) -> tuple[int, int]:
    """USB 槽 #00..#17 → (bus 0/1/2, 二进制 node_id)。node 不是 ASCII。"""
    if 0 <= slot <= 7:
        return 0, slot + 1
    if 8 <= slot <= 15:
        return 1, slot - 7
    if slot == 16:
        return 2, 1
    if slot == 17:
        return 2, 2
    raise ValueError(f"无效槽 #{slot}")

def pack_aa55(fun: int, payload: bytes = b"") -> bytes:
    plen = len(payload)
    hdr = bytes([0xAA, 0x55]) + struct.pack("<HBB", plen, DEV_TELEM, fun)
    body = hdr + payload
    return body + struct.pack("<H", crc16_modbus(body))

def pack_self_pwr(bus: int, node: int, cmd: int, drv: int = 0x03) -> bytes:
    """0x21 使能/断电。drv: 1=DRV1 2=DRV2 3=全部。"""
    return pack_aa55(FUN_SELF_PWR, bytes((bus & 0xFF, node & 0xFF, cmd & 0xFF, drv & 0xFF)))

def pack_self_homing(bus: int, node: int, cmd: int, iq_seek: float) -> bytes:
    """0x22 回零。iq_seek 为 Iq 码值（float32），不是安培。"""
    return pack_aa55(FUN_SELF_HOMING, struct.pack("<BBBB f 4x", bus, node, cmd, 0, iq_seek))

def pack_self_pos_pid(bus: int, node: int, kp: float, ki: float, kd: float) -> bytes:
    return pack_aa55(FUN_SELF_POS_PID, struct.pack("<BBBB 3f", bus, node, 0, 0, kp, ki, kd))

def pack_self_cur_pid(
    bus: int,
    node: int,
    id_kp: float,
    id_ki: float,
    id_kd: float,
    iq_kp: float,
    iq_ki: float,
    iq_kd: float,
) -> bytes:
    return pack_aa55(
        FUN_SELF_CUR_PID,
        struct.pack("<BBBB 6f", bus, node, 0, 0, id_kp, id_ki, id_kd, iq_kp, iq_ki, iq_kd),
    )

def pack_self_open_spin(bus: int, node: int, cmd: int, freq_hz: float, half_s: float) -> bytes:
    """0x25 开环往复。cmd: 1=开始 2=停止。"""
    return pack_aa55(
        FUN_SELF_OPEN_SPIN,
        struct.pack("<BBBB 2f", bus, node, cmd, 0, freq_hz, half_s),
    )

def _crc16_modbus_table() -> tuple[int, ...]:
    tab = []
    for i in range(256):
        crc = i
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
        tab.append(crc)
    return tuple(tab)

def crc16_modbus(data: bytes | bytearray | memoryview) -> int:
    """Modbus CRC16，初值 0xFFFF，与 H7 tsensei_app_usb_crc16_modbus 一致。"""
    crc = 0xFFFF
    tab = CRC16_MODBUS_TAB
    for b in data:
        crc = (crc >> 8) ^ tab[(crc ^ b) & 0xFF]
    return crc & 0xFFFF

def pack_hand_batch_into(
    buf: bytearray,
    seq: int,
    self_pos: tuple[float, ...] | list[float],
    mit_pos_deg: tuple[float, ...] | list[float],
    mit_vel_rpm: tuple[float, ...] | list[float],
    mit_iq_ff_a: tuple[float, ...] | list[float],
) -> None:
    """组 USB 0x20 整包到 108 字节缓冲，避免每帧新分配。"""
    if len(buf) < HAND_BATCH_WIRE_LEN:
        raise ValueError("buf 需要 108 字节")
    if len(self_pos) != SELF_MOTOR_NUM:
        raise ValueError(f"self_pos 需要 {SELF_MOTOR_NUM} 个")
    if len(mit_pos_deg) != MIT_MOTOR_NUM:
        raise ValueError("mit_pos_deg 需要 2 个")
    buf[0:6] = HAND_BATCH_HDR
    struct.pack_into(
        "<B18f2f2f2f3x",
        buf,
        6,
        seq & 0xFF,
        *self_pos,
        *mit_pos_deg,
        *mit_vel_rpm,
        *mit_iq_ff_a,
    )
    struct.pack_into("<H", buf, 106, crc16_modbus(memoryview(buf)[:106]))

def pack_hand_batch(
    seq: int,
    self_pos: list[float],
    mit_pos_deg: list[float],
    mit_vel_rpm: list[float],
    mit_iq_ff_a: list[float],
) -> bytes:
    """组 USB 0x20 整包，返回 108 字节。"""
    buf = bytearray(HAND_BATCH_WIRE_LEN)
    pack_hand_batch_into(buf, seq, self_pos, mit_pos_deg, mit_vel_rpm, mit_iq_ff_a)
    return bytes(buf)

def mit_out_to_motor(out_val: float, slot: int) -> float:
    """界面输出轴 → USB 电机端（deg 或 rpm）。"""
    return out_val * MIT_GEAR_RATIOS[slot]

def mit_motor_to_out(motor_val: float, slot: int) -> float:
    """电机端 → 界面输出轴（deg 或 rpm）。"""
    return motor_val / MIT_GEAR_RATIOS[slot]

CRC16_MODBUS_TAB = _crc16_modbus_table()

# Confirmed physical slot order. The two external motors use separate fields.
SELF_JOINTS = (
    "pinky_DIP_joint", "pinky_swing_joint", "ring_DIP_joint", "ring_swing_joint",
    "pinky_PIP_joint", "pinky_MCP_joint", "ring_PIP_joint", "ring_MCP_joint",
    "mid_DIP_joint", "mid_swing_joint", "index_DIP_joint", "index_swing_joint",
    "mid_PIP_joint", "mid_MCP_joint", "index_PIP_joint", "index_MCP_joint",
    "thumb_PIP_joint", "thumb_MCP_joint",
)
EXTERNAL_JOINTS = ("thumb_rotaback_joint2", "thumb_rota_block_joint")
SELF_ANGLE_SLOTS = frozenset({0, 1, 2, 3, 8, 9, 10, 11, 16})


def channel_mapping(hand_side="left"):
    if hand_side not in ("left", "right"):
        raise ValueError("hand_side must be left or right")
    rows = []
    for slot, suffix in enumerate(SELF_JOINTS):
        bus, node = slot_to_bus_node(slot)
        rows.append({"joint": f"{hand_side}_hand_{suffix}", "bus": bus,
                     "can": f"CAN{bus + 1}", "motor": node,
                     "field": "self_pos", "slot": slot,
                     "unit": "deg" if slot in SELF_ANGLE_SLOTS else "mm",
                     "gear_ratio": 1.0, "position_sign": 1 if slot in SELF_ANGLE_SLOTS else -1,
                     "can_id": 0x211})
    for slot, suffix in enumerate(EXTERNAL_JOINTS):
        rows.append({"joint": f"{hand_side}_hand_{suffix}", "bus": 2,
                     "can": "CAN3", "motor": slot + 1, "field": "mit_pos_deg",
                     "slot": slot, "unit": "deg", "gear_ratio": MIT_GEAR_RATIOS[slot],
                     "position_sign": 1,
                     "can_id": (0x505, 0x509)[slot]})
    return rows


def _finite_float32(value):
    if (type(value) not in (float, int) and (isinstance(value, bool) or not isinstance(value, Real))) or not math.isfinite(value):
        raise ValueError("Motor target must be a finite real number")
    value = float(value)
    try:
        struct.pack("<f", value)
    except (OverflowError, struct.error) as exc:
        raise ValueError("Motor target exceeds float32 range") from exc
    return value


def _pair(values, *, factors=(1.0, 1.0)):
    if len(values) != 2:
        raise ValueError("External motor fields require exactly two values")
    return tuple(_finite_float32(_finite_float32(v) * factor) for v, factor in zip(values, factors))


@lru_cache(maxsize=2)
def _encoder_layout(hand_side):
    """Immutable internal layout; public channel_mapping stays freely mutable."""
    rows = channel_mapping(hand_side)
    return (tuple((r["joint"], r["unit"] == "mm", r["field"] == "self_pos",
                   r["position_sign"], r["slot"]) for r in rows),
            frozenset(r["joint"] for r in rows if r["unit"] == "mm"),
            frozenset(r["joint"] for r in rows if r["unit"] == "deg"))


def encode_targets(targets, *, hand_side="left", seq=0,
                   mit_vel_rpm=(0.0, 0.0), mit_iq_ff_a=(0.0, 0.0)) -> bytes:
    """Named mm/output-shaft deg -> one 108-byte frame, without transmitting.

    External speed is also OUTPUT-shaft rpm; position/speed are multiplied by
    50 (M1 ROTABACK) / 30 (M2 ROTA_BLOCK) exactly once here. Linear strokes are negated once.
    Current feedforward is in A and is never multiplied.
    The protocol carries no left/right selector: names validate the chosen hand.
    """
    if hand_side not in ("left", "right"):
        raise ValueError("hand_side must be left or right")
    rows, linear_names, rotary_names = _encoder_layout(hand_side)
    if getattr(targets, "hand_side", hand_side) != hand_side:
        raise ValueError("Target hand_side does not match encoder")
    if isinstance(seq, bool) or not isinstance(seq, int) or seq < 0:
        raise ValueError("seq must be a nonnegative integer (wrapped to uint8)")
    linear, rotary = targets.linear_strokes_mm, targets.rotary_positions_deg
    for values, unit, expected in ((linear, "mm", linear_names), (rotary, "deg", rotary_names)):
        if not isinstance(values, Mapping) or values.keys() != expected:
            raise ValueError(f"A3.0 requires complete named {unit} targets for {hand_side}")
    slots, external = [], []
    for name, is_linear, is_self, sign, slot in rows:
        values = linear if is_linear else rotary
        value = _finite_float32(values[name])
        if is_self:
            slots.append(value * sign)
        else:
            external.append(_finite_float32(mit_out_to_motor(value, slot)))
    return pack_hand_batch(seq, slots, external,
                           _pair(mit_vel_rpm, factors=MIT_GEAR_RATIOS), _pair(mit_iq_ff_a))
