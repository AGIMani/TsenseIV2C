#!/usr/bin/env python3
"""Assemble standalone CLI hx_display.py (no project imports)."""
from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
FL = (ROOT / "src" / "kinematics" / "finger_linkage.py").read_text(encoding="utf-8")
V3 = (ROOT / "deprecated" / "main_app_v3.py").read_text(encoding="utf-8")
OLD = (ROOT / "hx_display.py").read_text(encoding="utf-8")

m_hi = re.search(r"_AUCH_CRC_HI = bytes\(\[(.*?)\]\)", V3, re.S)
m_lo = re.search(r"_AUCH_CRC_LO = bytes\(\[(.*?)\]\)", V3, re.S)
if not m_hi or not m_lo:
    raise SystemExit("CRC not found")
crc_block = m_hi.group(0) + "\n" + m_lo.group(0)

# Strip finger_linkage optional utils import and protocol import block
fl = FL
# Drop module header / duplicate imports (file already has them in HEADER)
fl = re.sub(r'^"""[\s\S]*?"""\s*', '', fl, count=1)
fl = re.sub(r'^from __future__ import annotations\s*', '', fl, count=1)
fl = re.sub(
    r'^import math\nimport json\nimport os\nimport time\n'
    r'from dataclasses import dataclass, field\n'
    r'from typing import Dict, List, Optional, Sequence, Tuple\n\n',
    '',
    fl,
    count=1,
)
fl = re.sub(
    r"# 可选工程日志.*?def register_tag\(tag: str, description: str = \"\", \*, enabled: bool = True\) -> None:.*?return None\n\n",
    (
        "def log_emit(msg: str, tag: str = \"\") -> None:\n"
        "    return None\n\n"
        "def register_tag(tag: str, description: str = \"\", *, enabled: bool = True) -> None:\n"
        "    return None\n\n"
    ),
    fl,
    count=1,
    flags=re.S,
)
# Remove protocol import that raises; remap will be provided above MotionPlaylist usage
fl = re.sub(
    r"# ── 动作播放用关节重映射.*?from None\n\n"
    r"(?:_PLAYLIST_STROKE_MAPPER: Optional\[HandStrokeMapper\] = None\n\n)?",
    (
        "# remap_* 由下方内嵌 protocol 提供\n\n"
        "_PLAYLIST_STROKE_MAPPER: Optional[HandStrokeMapper] = None\n\n"
    ),
    fl,
    count=1,
    flags=re.S,
)

# Extract USB/session portion from current hx_display (from USB constants through MotionPlayer)
# We'll rewrite CLI completely; reuse classes by extracting from OLD after CRC.
# Simpler: keep protocol+USB+CLI as written below; only embed kinematics from fl.

HEADER = '''#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""hx_display.py — 自包含 H7 命令行工具（串口 / 整手找零 / 动作下发）

本文件可单独拷贝运行，不依赖仓库内其它模块。
内嵌：
  · src/kinematics/finger_linkage.py（含 LUT 逆解）
  · protocol UI↔Hardware 符号映射（动作文件所需最小子集）
  · USB 协议 / 整手找零 / 0x20 动作下发（源自 deprecated/main_app_v3）

外部依赖仅：
  pip install pyserial

示例：
  python hx_display.py ports
  python hx_display.py zero --port COM7
  python hx_display.py motion --port COM7 --file motion/ok.txt --loop --hz 100
  python hx_display.py run --zero --file motion/ok.txt --port COM7
"""

from __future__ import annotations

import argparse
import json
import math
import os
import struct
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import (
    Callable,
    Dict,
    List,
    Mapping,
    Optional,
    Sequence,
    Tuple,
)

try:
    import serial
    from serial.tools import list_ports
except ImportError as exc:  # pragma: no cover
    raise SystemExit("请先安装 pyserial: pip install pyserial") from exc


# ═══════════════════════════════════════════════════════════════════════════
# 一、内嵌 finger_linkage（新逆解）+ UI↔HW 映射
# ═══════════════════════════════════════════════════════════════════════════

'''

PROTOCOL_EMBED = r'''
# ── UI ↔ Hardware（动作文件 / 下发符号规则，自 hand_angle_protocol 精简）──
Range = Tuple[float, float]
Cfg = Dict[str, float]

SHORT_TO_URDF: Dict[str, str] = {
    "thumb_rota": "left_hand_thumb_rota_joint",
    "thumb_swing": "left_hand_thumb_swing_joint",
    "thumb_mcp": "left_hand_thumb_MCP_joint",
    "thumb_pip": "left_hand_thumb_PIP_joint",
    "index_swing": "left_hand_index_swing_joint",
    "index_mcp": "left_hand_index_MCP_joint",
    "index_pip": "left_hand_index_PIP_joint",
    "index_dip": "left_hand_index_DIP_joint",
    "mid_swing": "left_hand_mid_swing_joint",
    "mid_mcp": "left_hand_mid_MCP_joint",
    "mid_pip": "left_hand_mid_PIP_joint",
    "mid_dip": "left_hand_mid_DIP_joint",
    "ring_swing": "left_hand_ring_swing_joint",
    "ring_mcp": "left_hand_ring_MCP_joint",
    "ring_pip": "left_hand_ring_PIP_joint",
    "ring_dip": "left_hand_ring_DIP_joint",
    "pinky_rota": "left_hand_pinky_rota_joint",
    "pinky_swing": "left_hand_pinky_swing_joint",
    "pinky_mcp": "left_hand_pinky_MCP_joint",
    "pinky_pip": "left_hand_pinky_PIP_joint",
    "pinky_dip": "left_hand_pinky_DIP_joint",
}
URDF_TO_SHORT: Dict[str, str] = {v: k for k, v in SHORT_TO_URDF.items()}

_DEG_SWING_MAX = 0.696
_FINGER_SWING_HW_LEFT_MIN = math.radians(0.5)
_FINGER_SWING_HW_LEFT_MAX = math.radians(39.5)
_DEG_50 = math.radians(50.0)
_THUMB_ROTA_HW_MIN = math.radians(0.5)
_THUMB_ROTA_HW_MAX = math.radians(89.5)
_FLEX_MAX = 1.57
_THUMB_SWING_MAX = 2.0
_THUMB_SWING_HW_MIN = 0.0
_THUMB_SWING_HW_MAX = 2.0
_FOUR_FINGER_SWING_SHORTS = (
    "index_swing", "mid_swing", "ring_swing", "pinky_swing"
)

UI_RANGES: Dict[str, Range] = {
    "thumb_rota": (0.0, _FLEX_MAX),
    "thumb_swing": (0.0, _THUMB_SWING_MAX),
    "thumb_mcp": (0.0, _FLEX_MAX),
    "thumb_pip": (0.0, _FLEX_MAX),
    "index_swing": (0.0, _DEG_SWING_MAX),
    "index_mcp": (0.0, _FLEX_MAX),
    "index_pip": (0.0, _FLEX_MAX),
    "index_dip": (0.0, _FLEX_MAX),
    "mid_swing": (0.0, _DEG_SWING_MAX),
    "mid_mcp": (0.0, _FLEX_MAX),
    "mid_pip": (0.0, _FLEX_MAX),
    "mid_dip": (0.0, _FLEX_MAX),
    "ring_swing": (0.0, _DEG_SWING_MAX),
    "ring_mcp": (0.0, _FLEX_MAX),
    "ring_pip": (0.0, _FLEX_MAX),
    "ring_dip": (0.0, _FLEX_MAX),
    "pinky_rota": (0.0, _DEG_50),
    "pinky_swing": (0.0, _DEG_SWING_MAX),
    "pinky_mcp": (0.0, _FLEX_MAX),
    "pinky_pip": (0.0, _FLEX_MAX),
    "pinky_dip": (0.0, _FLEX_MAX),
}
HW_RANGES_LEFT: Dict[str, Range] = {
    "thumb_rota": (_THUMB_ROTA_HW_MIN, _THUMB_ROTA_HW_MAX),
    "thumb_swing": (_THUMB_SWING_HW_MIN, _THUMB_SWING_HW_MAX),
    "thumb_mcp": (0.0, _FLEX_MAX),
    "thumb_pip": (0.0, _FLEX_MAX),
    "index_swing": (_FINGER_SWING_HW_LEFT_MIN, _FINGER_SWING_HW_LEFT_MAX),
    "index_mcp": (0.0, _FLEX_MAX),
    "index_pip": (0.0, _FLEX_MAX),
    "index_dip": (0.0, _FLEX_MAX),
    "mid_swing": (_FINGER_SWING_HW_LEFT_MIN, _FINGER_SWING_HW_LEFT_MAX),
    "mid_mcp": (0.0, _FLEX_MAX),
    "mid_pip": (0.0, _FLEX_MAX),
    "mid_dip": (0.0, _FLEX_MAX),
    "ring_swing": (_FINGER_SWING_HW_LEFT_MIN, _FINGER_SWING_HW_LEFT_MAX),
    "ring_mcp": (0.0, _FLEX_MAX),
    "ring_pip": (0.0, _FLEX_MAX),
    "ring_dip": (0.0, _FLEX_MAX),
    "pinky_rota": (0.0, _DEG_50),
    "pinky_swing": (_FINGER_SWING_HW_LEFT_MIN, _FINGER_SWING_HW_LEFT_MAX),
    "pinky_mcp": (0.0, _FLEX_MAX),
    "pinky_pip": (0.0, _FLEX_MAX),
    "pinky_dip": (0.0, _FLEX_MAX),
}
HW_RANGES_RIGHT: Dict[str, Range] = {
    "thumb_rota": (_THUMB_ROTA_HW_MIN, _THUMB_ROTA_HW_MAX),
    "thumb_swing": (_THUMB_SWING_HW_MIN, _THUMB_SWING_HW_MAX),
    "thumb_mcp": (0.0, _FLEX_MAX),
    "thumb_pip": (0.0, _FLEX_MAX),
    "index_swing": (0.0, _DEG_SWING_MAX),
    "index_mcp": (0.0, _FLEX_MAX),
    "index_pip": (0.0, _FLEX_MAX),
    "index_dip": (0.0, _FLEX_MAX),
    "mid_swing": (0.0, _DEG_SWING_MAX),
    "mid_mcp": (0.0, _FLEX_MAX),
    "mid_pip": (0.0, _FLEX_MAX),
    "mid_dip": (0.0, _FLEX_MAX),
    "ring_swing": (0.0, _DEG_SWING_MAX),
    "ring_mcp": (0.0, _FLEX_MAX),
    "ring_pip": (0.0, _FLEX_MAX),
    "ring_dip": (0.0, _FLEX_MAX),
    "pinky_rota": (-_DEG_50, 0.0),
    "pinky_swing": (0.0, _DEG_SWING_MAX),
    "pinky_mcp": (0.0, _FLEX_MAX),
    "pinky_pip": (0.0, _FLEX_MAX),
    "pinky_dip": (0.0, _FLEX_MAX),
}
def clamp(x: float, lo: float, hi: float) -> float:
    if x < lo:
        return lo
    if x > hi:
        return hi
    return x


def normalize_hand_side(hand_side: Optional[str]) -> str:
    s = (hand_side or "left").strip().lower()
    if s in ("right", "r", "右手", "rh"):
        return "right"
    return "left"


def normalize_cfg_keys(cfg: Mapping[str, float], *, as_urdf: bool = True) -> Cfg:
    out: Cfg = {}
    for key, val in cfg.items():
        k = str(key)
        if k in URDF_TO_SHORT:
            urdf = k
        elif k in SHORT_TO_URDF:
            urdf = SHORT_TO_URDF[k]
        elif k.startswith("right_hand_"):
            urdf = "left_hand_" + k[len("right_hand_") :]
        else:
            continue
        try:
            out[urdf] = float(val)
        except (TypeError, ValueError):
            continue
    return out


def convert_ui_to_hardware(
    cfg: Mapping[str, float],
    hand_side: Optional[str] = "left",
) -> Cfg:
    """UI → HW。thumb_swing 左右手与 UI 同值；H7 范围 [0,2] rad。"""
    side = normalize_hand_side(hand_side)
    ui = normalize_cfg_keys(cfg, as_urdf=True)
    out: Cfg = dict(ui)
    for urdf, value in ui.items():
        short = URDF_TO_SHORT.get(urdf)
        if short is None or short not in UI_RANGES:
            continue
        ulo, uhi = UI_RANGES[short]
        v = clamp(float(value), ulo, uhi)
        if side == "right" and short == "pinky_rota":
            v = -v
        hw = HW_RANGES_RIGHT[short] if side == "right" else HW_RANGES_LEFT[short]
        out[urdf] = clamp(v, hw[0], hw[1])
    return out


def convert_hardware_to_ui(
    cfg: Mapping[str, float],
    hand_side: Optional[str] = "left",
) -> Cfg:
    side = normalize_hand_side(hand_side)
    hw_cfg = normalize_cfg_keys(cfg, as_urdf=True)
    out: Cfg = dict(hw_cfg)
    for urdf, value in hw_cfg.items():
        short = URDF_TO_SHORT.get(urdf)
        if short is None or short not in UI_RANGES:
            continue
        v = float(value)
        if short in ("thumb_rota", "thumb_swing"):
            hw = HW_RANGES_RIGHT[short] if side == "right" else HW_RANGES_LEFT[short]
            v = clamp(v, hw[0], hw[1])
        elif short in _FOUR_FINGER_SWING_SHORTS:
            hw = HW_RANGES_RIGHT[short] if side == "right" else HW_RANGES_LEFT[short]
            v = clamp(v, hw[0], hw[1])
        elif side == "right" and short == "pinky_rota":
            v = -v
        ulo, uhi = UI_RANGES[short]
        out[urdf] = clamp(v, ulo, uhi)
    return out


def remap_joint_angles_for_downlink(
    joint_angles: Mapping[str, float],
    hand_side: Optional[str] = "left",
) -> Cfg:
    return convert_ui_to_hardware(joint_angles, hand_side=hand_side)


def remap_motion_cfg_file_to_ui(
    cfg: Mapping[str, float],
    hand_side: Optional[str] = "left",
) -> Cfg:
    normalized = normalize_cfg_keys(cfg, as_urdf=True)
    ui = convert_hardware_to_ui(normalized, hand_side=hand_side)
    thumb = SHORT_TO_URDF["thumb_rota"]
    if thumb in normalized:
        lo, hi = UI_RANGES["thumb_rota"]
        ui[thumb] = clamp(normalized[thumb], lo, hi)
    thumb_swing = SHORT_TO_URDF["thumb_swing"]
    if thumb_swing in normalized:
        value = clamp(normalized[thumb_swing], -1.0, 1.0)
        ui[thumb_swing] = 1.0 + value if normalize_hand_side(hand_side) == "right" else 1.0 - value
    for short in _FOUR_FINGER_SWING_SHORTS:
        urdf = SHORT_TO_URDF[short]
        if urdf in normalized:
            lo, hi = UI_RANGES[short]
            ui[urdf] = clamp(normalized[urdf], lo, hi)
    return ui

'''

# Insert PROTOCOL_EMBED before joint_angles_to_hand_positions / MotionPlaylist
marker = "def joint_angles_to_hand_positions("
if marker not in fl:
    raise SystemExit("joint_angles_to_hand_positions not found")
fl_kin, fl_play = fl.split(marker, 1)
fl_play = marker + fl_play

USB_AND_CLI = r'''

# ═══════════════════════════════════════════════════════════════════════════
# 二、USB 协议 / 找零 / 动作下发 / CLI
# ═══════════════════════════════════════════════════════════════════════════

USB_UPLINK_HDR = (0x55, 0xAA)
USB_DOWNLINK_HDR = (0xAA, 0x55)
DEV_TELEM = 0x03
FUN_HEARTBEAT = 0x01
FUN_TELEM = 0x02
FUN_LM_MECH_HOMING = 0x12
FUN_HAND_POS_BATCH = 0x20
FUN_RM_DRIVE_SP = 0x22
FUN_RM_ZERO_SP = 0x25

LM_HOMING_START = 0x01
DEFAULT_HOMING_IQ_SEEK = 700.0
HEARTBEAT_MAGIC = 0x42483748

LINEAR_MOTOR_NUM = 14
ROTARY_MOTOR_NUM = 7
LINEAR_BUS_COUNTS = (6, 6, 2)
ROTARY_BUS_LABELS = ("SPI1", "SPI2", "SPI3", "SPI4")
ROTARY_BUS_COUNTS = (2, 2, 1, 2)
ROTARY_BUS_BASE = (0, 2, 6, 4)

RM_ONE_CLICK_ZERO_SEQ: Tuple[Tuple[int, int], ...] = (
    (1, 1), (1, 0), (0, 1), (0, 0), (3, 0), (3, 1),
)
RM_ONE_CLICK_ZERO_TIMEOUT_S = 15.0
RM_ONE_CLICK_ZERO_ENABLE_DELAY_S = 0.2
RM_ONE_CLICK_ZERO_IQ_SEEK_ABS = 200.0
RM_ONE_CLICK_ZERO_MIN_SEEK_S = 1.2
RM_ONE_CLICK_ZERO_MOTION_DEG = 0.5
HAND_ONE_CLICK_LM_DELAY_S = 1.0

LINEAR_SLOT_SIZE = 32
ROTARY_SLOT_SIZE = 44
IMU_TELEM_SIZE = 16
TELEM_FRAME_SIZE = (
    8
    + LINEAR_SLOT_SIZE * LINEAR_MOTOR_NUM
    + ROTARY_SLOT_SIZE * ROTARY_MOTOR_NUM
    + IMU_TELEM_SIZE
)
ROTARY_SLOT_FMT = "B3x10f"

STM_VID = 0x0483
STM_USB_CDC_PID = 0x5740
STM_STLINK_VCP_PID = 0x374F

SERIAL_WRITE_TIMEOUT_S = 0.05
SERIAL_READ_TIMEOUT_S = 0.002
RX_DRAIN_MAX_BYTES_PER_LOOP = 262144
RX_DRAIN_CHUNK_BYTES = 65536
RX_IDLE_SLEEP_S = 0.0005
PARSER_BUF_MAX = 65536
PARSER_SYNC_BUDGET = 4096
DOWNLINK_HZ_DEFAULT = 100

''' + crc_block + r'''


def crc16_modbus(data: bytes, init: int = 0xFFFF) -> int:
    hi = (init >> 8) & 0xFF
    lo = init & 0xFF
    for b in data:
        idx = hi ^ b
        hi = lo ^ _AUCH_CRC_HI[idx]
        lo = _AUCH_CRC_LO[idx]
    return ((lo << 8) | hi) & 0xFFFF


def build_downlink(dev: int, fun: int, payload: bytes) -> bytes:
    plen = len(payload)
    body = bytes(USB_DOWNLINK_HDR) + struct.pack("<HBB", plen, dev & 0xFF, fun & 0xFF) + payload
    return body + struct.pack("<H", crc16_modbus(body))


def pack_lm_mech_homing(fdcan_bus: int, motor_id: int, cmd: int, iq_seek: float) -> bytes:
    return struct.pack("<BBBBf", fdcan_bus & 0xFF, motor_id & 0xFF, cmd & 0xFF, 0, float(iq_seek))


def pack_rm_drive_sp(spi_bus: int, en0: int, en1: int) -> bytes:
    return struct.pack("<BBBB", spi_bus & 0xFF, en0 & 0xFF, en1 & 0xFF, 0)


def pack_rm_zero_sp(spi_bus: int, zero0: int, zero1: int) -> bytes:
    return struct.pack("<BBBB", spi_bus & 0xFF, zero0 & 0xFF, zero1 & 0xFF, 0)


def pack_hand_pos_batch(seq: int, linear: Sequence[float], rotary: Sequence[float]) -> bytes:
    lin = [float(v) for v in linear]
    rot = [float(v) for v in rotary]
    while len(lin) < LINEAR_MOTOR_NUM:
        lin.append(0.0)
    while len(rot) < ROTARY_MOTOR_NUM:
        rot.append(0.0)
    return struct.pack("<B14f7f3x", seq & 0xFF, *lin[:LINEAR_MOTOR_NUM], *rot[:ROTARY_MOTOR_NUM])


def bus_motor_count(fdcan_bus: int) -> int:
    if 0 <= fdcan_bus < len(LINEAR_BUS_COUNTS):
        return LINEAR_BUS_COUNTS[fdcan_bus]
    return 0


def rotary_bus_motor_count(spi_bus: int) -> int:
    if 0 <= spi_bus < len(ROTARY_BUS_COUNTS):
        return ROTARY_BUS_COUNTS[spi_bus]
    return 0


def rotary_bus_slot_index(spi_bus: int, local_motor: int) -> Optional[int]:
    if spi_bus < 0 or spi_bus >= len(ROTARY_BUS_COUNTS):
        return None
    if local_motor < 0 or local_motor >= ROTARY_BUS_COUNTS[spi_bus]:
        return None
    return ROTARY_BUS_BASE[spi_bus] + local_motor


def rm_one_click_zero_seq_text() -> str:
    return " → ".join(f"{ROTARY_BUS_LABELS[b]} M{m}" for b, m in RM_ONE_CLICK_ZERO_SEQ)


def drain_serial_port(ser: serial.Serial, *, max_bytes: int = RX_DRAIN_MAX_BYTES_PER_LOOP) -> bytes:
    parts: List[bytes] = []
    total = 0
    while total < max_bytes:
        waiting = ser.in_waiting
        if waiting > 0:
            to_read = min(waiting, RX_DRAIN_CHUNK_BYTES, max_bytes - total)
            block = ser.read(to_read)
            if not block:
                break
            parts.append(block)
            total += len(block)
            continue
        if total > 0:
            break
        block = ser.read(1)
        if not block:
            break
        parts.append(block)
        break
    return b"".join(parts)


def format_port_entry(p) -> str:
    if p.vid == STM_VID and p.pid == STM_USB_CDC_PID:
        role = "H7 USB CDC"
    elif p.vid == STM_VID and p.pid == STM_STLINK_VCP_PID:
        role = "ST-Link"
    else:
        role = (p.description or "未知")[:40]
    return f"{p.device} — {role}"


def device_from_port_entry(entry: str) -> str:
    return entry.split(" — ", 1)[0].strip()


def pick_h7_port(ports: Optional[list] = None) -> Optional[str]:
    ports = list(ports if ports is not None else list_ports.comports())
    for p in ports:
        if p.vid == STM_VID and p.pid == STM_USB_CDC_PID:
            return p.device
    return None


def resolve_port(port_arg: str = "") -> str:
    """命令行 --port：空则自动选 H7 CDC；可写 COMx 或完整 entry。"""
    ports = list(list_ports.comports())
    if port_arg:
        raw = port_arg.strip()
        if " — " in raw:
            return device_from_port_entry(raw)
        for p in ports:
            if p.device.upper() == raw.upper():
                return p.device
        return raw
    h7 = pick_h7_port(ports)
    if h7:
        return h7
    if ports:
        return ports[0].device
    raise SystemExit("未找到可用串口；请插上 H7 USB CDC 或用 --port COMx 指定")


@dataclass
class RotaryMotorSlot:
    home_label: int
    iq: float
    iq_ref: float
    pos_actual_mm: float


def parse_rotary_slots(payload: bytes) -> Optional[List[RotaryMotorSlot]]:
    if len(payload) != TELEM_FRAME_SIZE:
        return None
    off = 8 + LINEAR_SLOT_SIZE * LINEAR_MOTOR_NUM
    out: List[RotaryMotorSlot] = []
    for _ in range(ROTARY_MOTOR_NUM):
        chunk = payload[off : off + ROTARY_SLOT_SIZE]
        if len(chunk) < ROTARY_SLOT_SIZE:
            return None
        t = struct.unpack("<" + ROTARY_SLOT_FMT, chunk)
        out.append(
            RotaryMotorSlot(
                home_label=int(t[0]),
                iq=float(t[1]),
                iq_ref=float(t[3]),
                pos_actual_mm=float(t[6]),
            )
        )
        off += ROTARY_SLOT_SIZE
    return out


def parse_heartbeat(payload: bytes) -> Optional[dict]:
    if len(payload) < 16:
        return None
    magic, seq, tick_ms, hb_count = struct.unpack("<4I", payload[:16])
    if magic != HEARTBEAT_MAGIC:
        return None
    return {"seq": seq, "tick_ms": tick_ms, "hb_count": hb_count}


class UplinkFrameParser:
    def __init__(self, on_frame: Callable[[int, int, bytes], None]) -> None:
        self._on_frame = on_frame
        self._buf = bytearray()
        self.crc_fail = 0

    def feed(self, chunk: bytes) -> None:
        self._buf.extend(chunk)
        if len(self._buf) > PARSER_BUF_MAX:
            del self._buf[: len(self._buf) - PARSER_BUF_MAX]
        sync_budget = PARSER_SYNC_BUDGET
        frames_budget = 32
        while frames_budget > 0 and sync_budget > 0:
            if len(self._buf) < 4:
                return
            if self._buf[0] != USB_UPLINK_HDR[0] or self._buf[1] != USB_UPLINK_HDR[1]:
                del self._buf[0]
                sync_budget -= 1
                continue
            plen = self._buf[2] | (self._buf[3] << 8)
            total = plen + 8
            if total > 4096 or plen > 2048:
                del self._buf[0]
                sync_budget -= 1
                continue
            if len(self._buf) < total:
                return
            frame = bytes(self._buf[:total])
            del self._buf[:total]
            body = frame[: plen + 6]
            crc_rx = frame[plen + 6] | (frame[plen + 7] << 8)
            if crc16_modbus(body) != crc_rx:
                self.crc_fail += 1
                sync_budget -= 1
                continue
            self._on_frame(frame[4], frame[5], frame[6 : 6 + plen])
            frames_budget -= 1


def log(msg: str) -> None:
    print(f"{time.strftime('%H:%M:%S')} {msg}", flush=True)


class H7Session:
    def __init__(self) -> None:
        self._ser: Optional[serial.Serial] = None
        self._stop = threading.Event()
        self._rx_thread: Optional[threading.Thread] = None
        self._tx_lock = threading.Lock()
        self._seq = 0
        self._telem_lock = threading.Lock()
        self._rotary: Optional[List[RotaryMotorSlot]] = None
        self.on_telem: Optional[Callable[[], None]] = None
        self.hb_count = 0
        self.telem_count = 0

    @property
    def connected(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def connect(self, port: str, *, baud: int = 115200) -> None:
        if self.connected:
            return
        ser = serial.Serial(
            port,
            baudrate=int(baud),
            timeout=SERIAL_READ_TIMEOUT_S,
            write_timeout=SERIAL_WRITE_TIMEOUT_S,
        )
        ser.dtr = True
        ser.rts = False
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        self._ser = ser
        self._stop.clear()
        self.hb_count = 0
        self.telem_count = 0
        self._rx_thread = threading.Thread(target=self._rx_loop, name="HxRx", daemon=True)
        self._rx_thread.start()
        log(f"已连接 {port} baud={baud} DTR=1")

    def disconnect(self) -> None:
        self._stop.set()
        t = self._rx_thread
        if t is not None and t.is_alive():
            t.join(timeout=1.0)
        self._rx_thread = None
        ser = self._ser
        self._ser = None
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass
        log("已断开")

    def send_fun(self, fun: int, payload: bytes) -> bool:
        if not self.connected or self._ser is None:
            return False
        frame = build_downlink(DEV_TELEM, fun, payload)
        with self._tx_lock:
            try:
                self._ser.write(frame)
                return True
            except serial.SerialException as exc:
                log(f"写串口失败: {exc}")
                return False

    def send_hand_batch(self, linear: Sequence[float], rotary: Sequence[float]) -> bool:
        self._seq = (self._seq + 1) & 0xFF
        return self.send_fun(FUN_HAND_POS_BATCH, pack_hand_pos_batch(self._seq, linear, rotary))

    def latest_rotary(self) -> Optional[List[RotaryMotorSlot]]:
        with self._telem_lock:
            return list(self._rotary) if self._rotary is not None else None

    def _on_frame(self, dev: int, fun: int, payload: bytes) -> None:
        if dev != DEV_TELEM:
            return
        if fun == FUN_HEARTBEAT:
            if parse_heartbeat(payload):
                self.hb_count += 1
            return
        if fun == FUN_TELEM:
            slots = parse_rotary_slots(payload)
            if slots is None:
                return
            with self._telem_lock:
                self._rotary = slots
            self.telem_count += 1
            cb = self.on_telem
            if cb is not None:
                try:
                    cb()
                except Exception:
                    pass

    def _rx_loop(self) -> None:
        parser = UplinkFrameParser(self._on_frame)
        while not self._stop.is_set():
            ser = self._ser
            if ser is None or not ser.is_open:
                break
            try:
                data = drain_serial_port(ser)
                if data:
                    parser.feed(data)
                else:
                    time.sleep(RX_IDLE_SLEEP_S)
            except serial.SerialException as exc:
                log(f"读串口失败: {exc}")
                break
            except Exception as exc:
                log(f"读线程异常: {exc}")
                time.sleep(0.01)


class HandZeroController:
    def __init__(self, session: H7Session) -> None:
        self.session = session
        self.active = False
        self.phase = ""
        self._t0 = 0.0
        self._iq_seek = DEFAULT_HOMING_IQ_SEEK
        self._lm_delay_s = HAND_ONE_CLICK_LM_DELAY_S
        self._rm_active = False
        self._rm_queue: List[Tuple[int, int]] = []
        self._rm_step: Optional[Tuple[int, int]] = None
        self._rm_phase = ""
        self._rm_t0 = 0.0
        self._rm_enable_at = 0.0
        self._rm_step_i = 0
        self._rm_total = 0
        self._rm_fail_caps: List[str] = []
        self._rm_en: Dict[int, Tuple[int, int]] = {}
        self._rm_saw_clear = False
        self._rm_saw_seek_iq = False
        self._rm_saw_motion = False
        self._rm_pos0 = 0.0
        self._rm_timeout_s = RM_ONE_CLICK_ZERO_TIMEOUT_S
        self._rm_enable_delay_s = RM_ONE_CLICK_ZERO_ENABLE_DELAY_S

    def start(
        self,
        *,
        iq_seek: float = DEFAULT_HOMING_IQ_SEEK,
        lm_delay_s: float = HAND_ONE_CLICK_LM_DELAY_S,
        rm_timeout_s: float = RM_ONE_CLICK_ZERO_TIMEOUT_S,
        rm_enable_delay_s: float = RM_ONE_CLICK_ZERO_ENABLE_DELAY_S,
    ) -> bool:
        if self.active:
            log("整手找零已在进行中")
            return False
        if not self.session.connected:
            log("未连接")
            return False
        self._iq_seek = float(iq_seek)
        self._lm_delay_s = float(lm_delay_s)
        self._rm_timeout_s = float(rm_timeout_s)
        self._rm_enable_delay_s = float(rm_enable_delay_s)
        sent = 0
        for bus in range(len(LINEAR_BUS_COUNTS)):
            for mid in range(1, bus_motor_count(bus) + 1):
                if self.session.send_fun(
                    FUN_LM_MECH_HOMING,
                    pack_lm_mech_homing(bus, mid, LM_HOMING_START, self._iq_seek),
                ):
                    sent += 1
        if sent <= 0:
            log("直线回零命令未能发出")
            return False
        self.active = True
        self.phase = "lm_wait"
        self._t0 = time.monotonic()
        log(
            f"整手找零开始: 直线 {sent} 帧 iq_seek={self._iq_seek:g}，"
            f"{self._lm_delay_s:g}s 后旋转 ({rm_one_click_zero_seq_text()})"
        )
        return True

    def stop(self, reason: str = "已停止") -> None:
        was = self.active
        self.active = False
        self.phase = ""
        self._rm_stop(reason)
        if was:
            log(f"整手找零{reason}")

    def tick(self) -> None:
        if not self.active:
            return
        now = time.monotonic()
        if self.phase == "lm_wait":
            if now - self._t0 < self._lm_delay_s:
                return
            log("直线延时结束，开始旋转找零")
            self.phase = "rm_run"
            if not self._rm_start():
                self.stop(reason="失败：旋转找零未能启动")
            return
        if self.phase == "rm_run":
            self._rm_tick()
            if self.active and not self._rm_active and self.phase == "rm_run":
                self.active = False
                self.phase = ""
                if self._rm_fail_caps:
                    log(f"整手找零结束（有失败）: {'、'.join(self._rm_fail_caps)}")
                else:
                    log("整手找零全部完成")

    def _rm_start(self) -> bool:
        self._rm_queue = list(RM_ONE_CLICK_ZERO_SEQ)
        self._rm_total = len(self._rm_queue)
        self._rm_step_i = 0
        self._rm_fail_caps = []
        self._rm_active = True
        self._rm_step = None
        self._rm_phase = ""
        self._rm_en.clear()
        self._rm_kick_next()
        return self._rm_active

    def _rm_stop(self, reason: str) -> None:
        was = self._rm_active
        self._rm_active = False
        self._rm_queue.clear()
        self._rm_step = None
        self._rm_phase = ""
        if was:
            log(f"旋转找零{reason}")

    def _rm_cap(self, bus: int, motor: int) -> str:
        return f"{ROTARY_BUS_LABELS[bus]} M{motor}"

    def _rm_send_drive(self, bus: int, en0: int, en1: int) -> bool:
        n = rotary_bus_motor_count(bus)
        e0 = en0 & 1
        e1 = (en1 & 1) if n >= 2 else 0
        self._rm_en[bus] = (e0, e1)
        return self.session.send_fun(FUN_RM_DRIVE_SP, pack_rm_drive_sp(bus, e0, e1))

    def _rm_send_zero(self, bus: int, motor: int) -> bool:
        n = rotary_bus_motor_count(bus)
        if motor == 0:
            z0, z1 = 1, 0
        elif motor == 1 and n >= 2:
            z0, z1 = 0, 1
        else:
            return False
        return self.session.send_fun(FUN_RM_ZERO_SP, pack_rm_zero_sp(bus, z0, z1))

    def _rm_slot(self, bus: int, motor: int) -> Optional[RotaryMotorSlot]:
        idx = rotary_bus_slot_index(bus, motor)
        rotary = self.session.latest_rotary()
        if idx is None or not rotary or idx >= len(rotary):
            return None
        return rotary[idx]

    def _rm_kick_next(self) -> None:
        if not self._rm_active:
            return
        if not self._rm_queue:
            self._rm_active = False
            self._rm_step = None
            self._rm_phase = ""
            if self._rm_fail_caps:
                log(f"旋转找零结束（有失败）: {'、'.join(self._rm_fail_caps)}")
            else:
                log("旋转找零全部完成")
            return
        bus, motor = self._rm_queue.pop(0)
        self._rm_step_i += 1
        self._rm_step = (bus, motor)
        self._rm_saw_clear = False
        self._rm_saw_seek_iq = False
        self._rm_saw_motion = False
        self._rm_pos0 = 0.0
        cap = self._rm_cap(bus, motor)
        log(f"旋转找零 [{self._rm_step_i}/{self._rm_total}] {cap} 使能…")
        e0, e1 = self._rm_en.get(bus, (0, 0))
        e0 = e0 | (1 if motor == 0 else 0)
        e1 = e1 | (1 if motor == 1 else 0)
        if not self._rm_send_drive(bus, e0, e1):
            self.stop(reason=f"失败：{cap} 使能发送失败")
            return
        self._rm_phase = "enable_wait"
        self._rm_enable_at = time.monotonic() + self._rm_enable_delay_s

    def _rm_issue_zero(self) -> None:
        if not self._rm_active or self._rm_step is None:
            return
        bus, motor = self._rm_step
        cap = self._rm_cap(bus, motor)
        slot0 = self._rm_slot(bus, motor)
        self._rm_pos0 = float(slot0.pos_actual_mm) if slot0 is not None else 0.0
        if not self._rm_send_zero(bus, motor):
            self.stop(reason=f"失败：{cap} 回零发送失败")
            return
        self._rm_phase = "seeking"
        self._rm_t0 = time.monotonic()
        self._rm_saw_clear = False
        self._rm_saw_seek_iq = False
        self._rm_saw_motion = False
        if slot0 is not None and int(slot0.home_label) == 0:
            self._rm_saw_clear = True
        log(f"旋转找零 [{self._rm_step_i}/{self._rm_total}] {cap} 寻零中…")

    def _rm_on_success(self) -> None:
        if self._rm_step is None:
            return
        bus, motor = self._rm_step
        log(f"旋转找零成功: {self._rm_cap(bus, motor)}")
        self._rm_step = None
        self._rm_phase = ""
        self._rm_kick_next()

    def _rm_on_timeout_skip(self) -> None:
        if self._rm_step is None:
            return
        bus, motor = self._rm_step
        cap = self._rm_cap(bus, motor)
        if cap not in self._rm_fail_caps:
            self._rm_fail_caps.append(cap)
        log(
            f"旋转找零失败（超时）: {cap} "
            f"[{self._rm_step_i}/{self._rm_total}]，跳过并继续"
        )
        self._rm_step = None
        self._rm_phase = ""
        self._rm_saw_clear = False
        self._rm_saw_seek_iq = False
        self._rm_saw_motion = False
        self._rm_kick_next()

    def _rm_tick(self) -> None:
        if not self._rm_active or self._rm_step is None:
            return
        now = time.monotonic()
        bus, motor = self._rm_step
        cap = self._rm_cap(bus, motor)
        if self._rm_phase == "enable_wait":
            if now >= self._rm_enable_at:
                self._rm_issue_zero()
            return
        if self._rm_phase != "seeking":
            return
        elapsed = now - self._rm_t0
        if elapsed > self._rm_timeout_s:
            self._rm_on_timeout_skip()
            return
        slot = self._rm_slot(bus, motor)
        if slot is None:
            return
        home = int(slot.home_label)
        iq_ref = float(slot.iq_ref)
        pos = float(slot.pos_actual_mm)
        if abs(iq_ref) >= RM_ONE_CLICK_ZERO_IQ_SEEK_ABS:
            self._rm_saw_seek_iq = True
        if abs(pos - self._rm_pos0) >= RM_ONE_CLICK_ZERO_MOTION_DEG:
            self._rm_saw_motion = True
        if home == 0:
            self._rm_saw_clear = True
            return
        if home >= 1 and self._rm_saw_clear:
            self._rm_on_success()
            return
        if home >= 1 and elapsed >= RM_ONE_CLICK_ZERO_MIN_SEEK_S:
            if self._rm_saw_seek_iq and abs(iq_ref) < 50.0:
                self._rm_on_success()
                return
            if self._rm_saw_motion and abs(pos) < 2.0:
                self._rm_on_success()


class MotionPlayer:
    def __init__(self, session: H7Session) -> None:
        self.session = session
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._playlist: Optional[MotionPlaylist] = None

    @property
    def playing(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def load(self, path: str, *, loop: bool = False, hand_side: str = "") -> MotionPlaylist:
        pl = load_motion_file(path)
        if hand_side:
            pl.hand_side = normalize_hand_side(hand_side)
        pl.loop = bool(loop)
        self._playlist = pl
        log(
            f"已加载动作 {os.path.basename(path)} "
            f"关键帧={len(pl.keyframes)} 时长={pl.duration_s:.2f}s side={pl.hand_side} loop={pl.loop}"
        )
        return pl

    def start(self, *, hz: float = DOWNLINK_HZ_DEFAULT) -> bool:
        if not self.session.connected:
            log("未连接")
            return False
        if self._playlist is None:
            log("请先加载动作")
            return False
        if self.playing:
            log("动作已在播放")
            return False
        self._hz = max(1.0, float(hz))
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="HxMotion", daemon=True)
        self._thread.start()
        log(f"开始下发 @ {self._hz:g}Hz")
        return True

    def stop(self) -> None:
        self._stop.set()
        t = self._thread
        if t is not None and t.is_alive():
            t.join(timeout=1.0)
        self._thread = None
        log("动作下发已停止")

    def _run(self) -> None:
        pl = self._playlist
        if pl is None:
            return
        interval = 1.0 / self._hz
        t0 = time.perf_counter()
        next_tick = t0
        last_report = 0.0
        while not self._stop.is_set():
            now = time.perf_counter()
            if now < next_tick:
                time.sleep(min(0.002, next_tick - now))
                continue
            elapsed = now - t0
            if (not pl.loop) and elapsed >= pl.duration_s:
                linear, rotary = pl.hand_positions_at(pl.duration_s)
                self.session.send_hand_batch(linear, rotary)
                log("动作播放完成")
                break
            linear, rotary = pl.hand_positions_at(elapsed)
            if not self.session.send_hand_batch(linear, rotary):
                log("写串口失败，停止播放")
                break
            if now - last_report >= 1.0:
                show_t = elapsed % pl.duration_s if pl.loop and pl.duration_s > 0 else elapsed
                log(f"播放 t={show_t:.2f}/{pl.duration_s:.2f}s seg={pl.segment_label_at(elapsed)}")
                last_report = now
            next_tick += interval
            if next_tick < now - interval:
                next_tick = now


def cmd_ports(_args: argparse.Namespace) -> int:
    ports = list(list_ports.comports())
    if not ports:
        print("（无串口）")
        return 1
    h7 = pick_h7_port(ports)
    for p in ports:
        mark = " <-- H7" if (h7 and p.device == h7) else ""
        print(f"{format_port_entry(p)}{mark}")
    return 0


def cmd_zero(args: argparse.Namespace) -> int:
    port = resolve_port(args.port)
    session = H7Session()
    zero = HandZeroController(session)
    session.on_telem = zero.tick
    try:
        session.connect(port, baud=args.baud)
        # 可选：等几帧遥测再找零
        if args.wait_telem > 0:
            t0 = time.monotonic()
            while time.monotonic() - t0 < args.wait_telem and session.telem_count < 1:
                time.sleep(0.05)
            log(f"心跳={session.hb_count} 遥测={session.telem_count}")
        if not zero.start(
            iq_seek=args.iq_seek,
            lm_delay_s=args.lm_delay,
            rm_timeout_s=args.rm_timeout,
            rm_enable_delay_s=args.rm_enable_delay,
        ):
            return 2
        t_end = time.monotonic() + float(args.timeout)
        while zero.active and time.monotonic() < t_end:
            zero.tick()
            time.sleep(0.05)
        if zero.active:
            zero.stop(reason="超时未完成")
            return 3
        return 0
    except serial.SerialException as exc:
        log(f"连接失败: {exc}")
        return 2
    except KeyboardInterrupt:
        log("用户中断")
        zero.stop(reason="用户中断")
        return 130
    finally:
        zero.stop()
        session.disconnect()


def cmd_motion(args: argparse.Namespace) -> int:
    port = resolve_port(args.port)
    session = H7Session()
    player = MotionPlayer(session)
    try:
        session.connect(port, baud=args.baud)
        if args.wait_telem > 0:
            t0 = time.monotonic()
            while time.monotonic() - t0 < args.wait_telem and session.telem_count < 1:
                time.sleep(0.05)
        player.load(args.file, loop=bool(args.loop), hand_side=args.hand_side or "")
        if not player.start(hz=args.hz):
            return 2
        if args.loop:
            log("循环播放中，Ctrl+C 停止")
            while player.playing:
                time.sleep(0.2)
        else:
            while player.playing:
                time.sleep(0.05)
            if args.hold > 0:
                log(f"终态保持 {args.hold:g}s")
                time.sleep(float(args.hold))
        return 0
    except serial.SerialException as exc:
        log(f"连接失败: {exc}")
        return 2
    except Exception as exc:
        log(f"失败: {exc}")
        return 2
    except KeyboardInterrupt:
        log("用户中断")
        return 130
    finally:
        player.stop()
        session.disconnect()


def cmd_run(args: argparse.Namespace) -> int:
    """先找零再播动作（可选）。"""
    port = resolve_port(args.port)
    session = H7Session()
    zero = HandZeroController(session)
    player = MotionPlayer(session)
    session.on_telem = zero.tick
    try:
        session.connect(port, baud=args.baud)
        if args.zero:
            if not zero.start(
                iq_seek=args.iq_seek,
                lm_delay_s=args.lm_delay,
                rm_timeout_s=args.rm_timeout,
                rm_enable_delay_s=args.rm_enable_delay,
            ):
                return 2
            t_end = time.monotonic() + float(args.timeout)
            while zero.active and time.monotonic() < t_end:
                zero.tick()
                time.sleep(0.05)
            if zero.active:
                zero.stop(reason="超时未完成")
                return 3
        if args.file:
            player.load(args.file, loop=bool(args.loop), hand_side=args.hand_side or "")
            if not player.start(hz=args.hz):
                return 2
            if args.loop:
                log("循环播放中，Ctrl+C 停止")
                while player.playing:
                    time.sleep(0.2)
            else:
                while player.playing:
                    time.sleep(0.05)
        return 0
    except serial.SerialException as exc:
        log(f"连接失败: {exc}")
        return 2
    except KeyboardInterrupt:
        log("用户中断")
        return 130
    finally:
        player.stop()
        zero.stop()
        session.disconnect()


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="hx_display.py",
        description="自包含 H7 命令行：扫口 / 整手找零 / 动作下发",
    )
    sub = p.add_subparsers(dest="cmd", required=True)

    sp = sub.add_parser("ports", help="列出串口（标记 H7 CDC）")
    sp.set_defaults(func=cmd_ports)

    def add_common(ap: argparse.ArgumentParser) -> None:
        ap.add_argument("--port", default="", help="串口名，如 COM7；省略则自动选 H7 CDC")
        ap.add_argument("--baud", type=int, default=115200, help="波特率（CDC 通常无关，默认 115200）")
        ap.add_argument(
            "--wait-telem",
            type=float,
            default=1.0,
            help="连接后等待首帧遥测的秒数（0=不等）",
        )

    def add_zero_opts(ap: argparse.ArgumentParser) -> None:
        ap.add_argument("--iq-seek", type=float, default=DEFAULT_HOMING_IQ_SEEK, help="直线找零 iq_seek")
        ap.add_argument("--lm-delay", type=float, default=HAND_ONE_CLICK_LM_DELAY_S, help="直线发完后等待秒数再转旋转找零")
        ap.add_argument("--rm-timeout", type=float, default=RM_ONE_CLICK_ZERO_TIMEOUT_S, help="单轴旋转找零超时秒数")
        ap.add_argument("--rm-enable-delay", type=float, default=RM_ONE_CLICK_ZERO_ENABLE_DELAY_S, help="旋转使能后发回零前延时")
        ap.add_argument("--timeout", type=float, default=120.0, help="整手找零总超时秒数")

    def add_motion_opts(ap: argparse.ArgumentParser) -> None:
        ap.add_argument("--file", "-f", required=False, default="", help="动作文件 .txt/.json")
        ap.add_argument("--loop", action="store_true", help="循环播放")
        ap.add_argument("--hz", type=float, default=DOWNLINK_HZ_DEFAULT, help="下发频率 Hz")
        ap.add_argument("--hand-side", default="", choices=["left", "right"], help="覆盖动作文件中的左右手")
        ap.add_argument("--hold", type=float, default=0.0, help="非循环播放结束后保持终态秒数")

    sz = sub.add_parser("zero", help="整手一键找零")
    add_common(sz)
    add_zero_opts(sz)
    sz.set_defaults(func=cmd_zero)

    sm = sub.add_parser("motion", help="加载并下发动作文件")
    add_common(sm)
    add_motion_opts(sm)
    sm.add_argument("--file", "-f", required=True, help="动作文件路径")
    sm.set_defaults(func=cmd_motion)

    sr = sub.add_parser("run", help="可选找零后播放动作")
    add_common(sr)
    add_zero_opts(sr)
    add_motion_opts(sr)
    sr.add_argument("--zero", action="store_true", help="先执行整手找零")
    sr.set_defaults(func=cmd_run)

    return p


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    return int(args.func(args))


if __name__ == "__main__":
    raise SystemExit(main())
'''

# Deduplicate MotionPlaylist file arg - motion subparser has required --file
# Fix: in add_motion_opts I also add --file optional; for motion I re-add required.
# The second add_argument for --file on same parser may conflict. Fix USB_AND_CLI:
USB_AND_CLI = USB_AND_CLI.replace(
    '''    sm = sub.add_parser("motion", help="加载并下发动作文件")
    add_common(sm)
    add_motion_opts(sm)
    sm.add_argument("--file", "-f", required=True, help="动作文件路径")
    sm.set_defaults(func=cmd_motion)
''',
    '''    sm = sub.add_parser("motion", help="加载并下发动作文件")
    add_common(sm)
    sm.add_argument("--file", "-f", required=True, help="动作文件路径")
    sm.add_argument("--loop", action="store_true", help="循环播放")
    sm.add_argument("--hz", type=float, default=DOWNLINK_HZ_DEFAULT, help="下发频率 Hz")
    sm.add_argument("--hand-side", default="", choices=["left", "right"], help="覆盖动作文件中的左右手")
    sm.add_argument("--hold", type=float, default=0.0, help="非循环播放结束后保持终态秒数")
    sm.set_defaults(func=cmd_motion)
''',
)

# Also fix run: --file optional
USB_AND_CLI = USB_AND_CLI.replace(
    '''    def add_motion_opts(ap: argparse.ArgumentParser) -> None:
        ap.add_argument("--file", "-f", required=False, default="", help="动作文件 .txt/.json")
        ap.add_argument("--loop", action="store_true", help="循环播放")
        ap.add_argument("--hz", type=float, default=DOWNLINK_HZ_DEFAULT, help="下发频率 Hz")
        ap.add_argument("--hand-side", default="", choices=["left", "right"], help="覆盖动作文件中的左右手")
        ap.add_argument("--hold", type=float, default=0.0, help="非循环播放结束后保持终态秒数")
''',
    '''    def add_motion_opts(ap: argparse.ArgumentParser) -> None:
        ap.add_argument("--file", "-f", default="", help="动作文件 .txt/.json")
        ap.add_argument("--loop", action="store_true", help="循环播放")
        ap.add_argument("--hz", type=float, default=DOWNLINK_HZ_DEFAULT, help="下发频率 Hz")
        ap.add_argument("--hand-side", default="", choices=["left", "right"], help="覆盖动作文件中的左右手")
        ap.add_argument("--hold", type=float, default=0.0, help="非循环播放结束后保持终态秒数")
''',
)

out = HEADER + fl_kin + "\n" + PROTOCOL_EMBED + "\n" + fl_play + "\n" + USB_AND_CLI
# Remove shebang/docstring duplicate from embedded finger_linkage top if present inside fl_kin
# fl_kin still starts with """ finger_linkage... — keep as comment block is fine inside file.

path = ROOT / "hx_display.py"
path.write_text(out, encoding="utf-8")
print(f"wrote {path} ({path.stat().st_size} bytes)")
