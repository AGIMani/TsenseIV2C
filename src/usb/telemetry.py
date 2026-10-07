"""H7 聚合遥测帧解析（从 deprecated/main_app_v3.py 抽出）。

主工程勿再 ``import main_app_v3``：该文件已迁入 deprecated/，
会导致 Form5 / 一键找零 / 遥测读角静默失败。
"""
from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass
from typing import List, Optional, Tuple

LINEAR_MOTOR_NUM = 14
ROTARY_MOTOR_NUM = 7

BENCH_ACTIVE_LINEAR_SLOTS: Optional[frozenset[int]] = None

LINEAR_SLOT_FMT = "8f"
LINEAR_SLOT_SIZE = struct.calcsize("<" + LINEAR_SLOT_FMT)  # 32
ROTARY_SLOT_FMT = "B3x10f"
ROTARY_SLOT_SIZE = struct.calcsize("<" + ROTARY_SLOT_FMT)  # 44
IMU_TELEM_FMT = "HHfff"
IMU_TELEM_SIZE = struct.calcsize("<" + IMU_TELEM_FMT)  # 16
TELEM_FRAME_SIZE = (
    8
    + LINEAR_SLOT_SIZE * LINEAR_MOTOR_NUM
    + ROTARY_SLOT_SIZE * ROTARY_MOTOR_NUM
    + IMU_TELEM_SIZE
)  # 780
LEGACY_TELEM_SLOT_SIZE = ROTARY_SLOT_SIZE
LEGACY_TELEM_FRAME_SIZE = 8 + LEGACY_TELEM_SLOT_SIZE * (
    LINEAR_MOTOR_NUM + ROTARY_MOTOR_NUM
)  # 932

HEARTBEAT_MAGIC = 0x42483748
IA_IB_COUNTS_TO_AMP = 0.001374
MECH_HOME_LABELS = ("WAIT", "SEEK", "DONE", "ERR")


def mcu_temp_plausible(centi: int) -> bool:
    return 500 <= abs(int(centi)) <= 8500


def decode_status_a(status_a: int) -> dict:
    u = int(status_a) & 0xFFFFFFFF
    return {
        "status_valid": bool((u >> 31) & 1),
        "ntc_valid": bool((u >> 30) & 1),
        "drv2_nfault": bool((u >> 25) & 1),
        "drv1_nfault": bool((u >> 24) & 1),
        "drv1_reg0x01": (u >> 16) & 0xFF,
        "drv1_reg0x02": (u >> 8) & 0xFF,
        "drv1_reg0x03": u & 0xFF,
    }


def decode_status_b(status_b: int) -> dict:
    u = int(status_b) & 0xFFFFFFFF
    return {
        "drv2_reg0x01": (u >> 16) & 0xFF,
        "drv2_reg0x02": (u >> 8) & 0xFF,
        "drv2_reg0x03": u & 0xFF,
    }


def home_label_str(label: int, elec_angle_rad: float) -> str:
    if label <= 3:
        return MECH_HOME_LABELS[label]
    return f"θe={elec_angle_rad:.4f} rad"


def parse_homing_or_theta_slot1(slot1: bytes) -> Tuple[int, int, float]:
    if len(slot1) < 4:
        return 0, 0, 0.0
    b0, b1, b2, b3 = slot1[0], slot1[1], slot1[2], slot1[3]
    if b0 == 0 and b1 == 0 and b2 == 0 and b3 <= 3:
        return b3, 0, 0.0
    if b0 <= 3 and b2 == 0 and b3 == 0:
        return b0, b1, 0.0
    elec, = struct.unpack("<f", slot1[:4])
    return 0xFF, b1, elec


def u32_le(data: bytes) -> int:
    return struct.unpack("<I", data[:4])[0]


def decode_telem_slot_float_mm(raw4: bytes) -> float:
    if len(raw4) < 4:
        return 0.0
    f = struct.unpack("<f", raw4[:4])[0]
    # IEEE-754 -0.0 is 0x80000000. It must not fall through to the
    # legacy int32 micrometre decoder (-2147483648 -> -2147483.648 mm).
    if f == 0.0 or abs(f) >= 1e-6:
        return f
    i = struct.unpack("<i", raw4[:4])[0]
    if i == 0:
        return 0.0
    return i * 0.001


def parse_status_c_slot10(u32: int) -> Tuple[int, int, int]:
    u32 &= 0xFFFFFFFF

    def _sign16(v: int) -> int:
        return v - 0x10000 if v >= 0x8000 else v

    temp_lo = _sign16(u32 & 0xFFFF)
    temp_hi = _sign16((u32 >> 16) & 0xFFFF)
    rx_lo, tx_lo = (u32 >> 16) & 0xFF, (u32 >> 24) & 0xFF
    rx_hi, tx_hi = u32 & 0xFF, (u32 >> 8) & 0xFF

    def _plausible_temp(t: int) -> bool:
        return 500 <= abs(t) <= 12000

    if _plausible_temp(temp_lo) and not _plausible_temp(temp_hi):
        return rx_lo, tx_lo, temp_lo
    if _plausible_temp(temp_hi) and not _plausible_temp(temp_lo):
        return rx_hi, tx_hi, temp_hi
    if temp_lo != 0:
        return rx_lo, tx_lo, temp_lo
    if temp_hi != 0:
        return rx_hi, tx_hi, temp_hi
    return rx_lo, tx_lo, temp_lo


def parse_currents_raw_slot11(u32: int) -> Tuple[int, int]:
    """将一个 u32 按两个小端 int16 解析为 ia/ib ADC 原始计数。"""
    ia = u32 & 0xFFFF
    ib = (u32 >> 16) & 0xFFFF
    if ia >= 0x8000:
        ia -= 0x10000
    if ib >= 0x8000:
        ib -= 0x10000
    return ia, ib


@dataclass
class LinearMotorSlot:
    mech_home_state: int
    drv_auto_recover: int
    pos_act_mm: float
    iq_meas: float
    iq_ref: float
    pos_ref_mm: float
    pos_err_mm: float
    grav_iq: float
    status_b_u32: int
    can_rx_error_cnt: int
    can_tx_error_cnt: int
    mcu_temp_centi: int
    elec_angle_rad: float
    app_version: int = 0
    id_meas: float = 0.0
    id_ref: float = 0.0
    status_a_u32: int = 0
    ia_counts_raw: int = 0
    ib_counts_raw: int = 0
    vd_out: float = 0.0
    vq_out: float = 0.0
    slot_phys_bus: Optional[int] = None
    slot_motor_id: Optional[int] = None
    slot_linear_idx: Optional[int] = None
    slot_tag_v2: bool = False
    slot_id_tag_ok: bool = False

    @property
    def app_ver_str(self) -> str:
        # 当前 32B 槽布局不上报 app_version，与 main_app_v3 显示逻辑一致。
        return "—"

    @property
    def home_str(self) -> str:
        return home_label_str(self.mech_home_state, self.elec_angle_rad)

    @property
    def mcu_temp_c(self) -> float:
        return self.mcu_temp_centi * 0.01

    @property
    def ia_amp(self) -> float:
        return self.ia_counts_raw * IA_IB_COUNTS_TO_AMP

    @property
    def ib_amp(self) -> float:
        return self.ib_counts_raw * IA_IB_COUNTS_TO_AMP

    @property
    def drv_fault_str(self) -> str:
        if self.status_b_u32 == 0:
            return "—"
        sb = decode_status_b(self.status_b_u32)
        return (
            "DRV2: "
            f"R01/02/03=0x{sb['drv2_reg0x01']:02X}/"
            f"0x{sb['drv2_reg0x02']:02X}/0x{sb['drv2_reg0x03']:02X}"
        )


@dataclass
class RotaryMotorSlot:
    home_label: int
    iq: float
    id_: float
    iq_ref: float
    id_ref: float
    pos_ref_mm: float
    pos_actual_mm: float
    pos_err_mm: float
    vq_cmd: float
    force_n: float
    elec_angle_rad: float

    @property
    def home_str(self) -> str:
        return home_label_str(self.home_label, self.elec_angle_rad)


def _linear_slot_has_data(slot: LinearMotorSlot) -> bool:
    if mcu_temp_plausible(slot.mcu_temp_centi):
        return True
    if abs(slot.pos_act_mm) > 0.01:
        return True
    if abs(slot.iq_meas) > 0.01 or abs(slot.iq_ref) > 0.01:
        return True
    if abs(slot.grav_iq) > 1e-6:
        return True
    if 0 < slot.mech_home_state <= 3:
        return True
    return False


def slot_is_active_linear(slot: LinearMotorSlot, idx: Optional[int] = None) -> bool:
    if idx is None:
        return False
    if BENCH_ACTIVE_LINEAR_SLOTS is not None and idx not in BENCH_ACTIVE_LINEAR_SLOTS:
        return False
    if slot.slot_id_tag_ok:
        return True
    return _linear_slot_has_data(slot)


def _rotary_slot_has_data(slot: RotaryMotorSlot) -> bool:
    return (
        abs(slot.iq) > 1e-6
        or abs(slot.id_) > 1e-6
        or abs(slot.pos_actual_mm) > 1e-3
        or abs(slot.pos_ref_mm) > 1e-3
        or abs(slot.iq_ref) > 1e-6
        or abs(slot.vq_cmd) > 1e-6
        or abs(slot.elec_angle_rad) > 1e-6
        or slot.home_label != 0
    )


def slot_is_active_rotary(slot: RotaryMotorSlot) -> bool:
    return _rotary_slot_has_data(slot)


def parse_heartbeat(payload: bytes) -> Optional[dict]:
    """解析 16B H7 心跳 payload；magic 不匹配时返回 None。"""
    if len(payload) < 16:
        return None
    magic, seq, tick_ms, hb_count = struct.unpack("<4I", payload[:16])
    if magic != HEARTBEAT_MAGIC:
        return None
    return {"seq": seq, "tick_ms": tick_ms, "hb_count": hb_count}


def _parse_linear_slot(data: bytes) -> LinearMotorSlot:
    chunk = data[:LINEAR_SLOT_SIZE].ljust(LINEAR_SLOT_SIZE, b"\x00")
    t = struct.unpack("<" + LINEAR_SLOT_FMT, chunk)
    home, drv, elec = parse_homing_or_theta_slot1(chunk[0:4])
    pos_act_mm = decode_telem_slot_float_mm(chunk[4:8])
    pos_ref_mm = decode_telem_slot_float_mm(chunk[16:20])
    status_b_u32 = u32_le(chunk[24:28])
    can_rx_error_cnt, can_tx_error_cnt, mcu_temp_centi = parse_status_c_slot10(
        u32_le(chunk[28:32])
    )
    return LinearMotorSlot(
        mech_home_state=home,
        drv_auto_recover=drv,
        pos_act_mm=pos_act_mm,
        iq_meas=t[2],
        iq_ref=t[3],
        pos_ref_mm=pos_ref_mm,
        pos_err_mm=pos_ref_mm - pos_act_mm,
        grav_iq=t[5],
        status_b_u32=status_b_u32,
        can_rx_error_cnt=can_rx_error_cnt,
        can_tx_error_cnt=can_tx_error_cnt,
        mcu_temp_centi=mcu_temp_centi,
        elec_angle_rad=elec,
    )


def _parse_rotary_slot(data: bytes) -> RotaryMotorSlot:
    t = struct.unpack("<" + ROTARY_SLOT_FMT, data)
    return RotaryMotorSlot(
        home_label=t[0],
        iq=t[1],
        id_=t[2],
        iq_ref=t[3],
        id_ref=t[4],
        pos_ref_mm=t[5],
        pos_actual_mm=t[6],
        pos_err_mm=t[7],
        vq_cmd=t[8],
        force_n=t[9],
        elec_angle_rad=t[10],
    )


def _legacy_slot_to_linear(data: bytes) -> LinearMotorSlot:
    t = struct.unpack("<" + ROTARY_SLOT_FMT, data)
    return LinearMotorSlot(
        mech_home_state=t[0],
        drv_auto_recover=0,
        pos_act_mm=t[6],
        iq_meas=t[1],
        iq_ref=t[3],
        pos_ref_mm=t[5],
        pos_err_mm=t[7],
        grav_iq=0.0,
        status_b_u32=0,
        can_rx_error_cnt=0,
        can_tx_error_cnt=0,
        mcu_temp_centi=0,
        elec_angle_rad=t[10],
        id_meas=t[2],
        id_ref=t[4],
        vd_out=0.0,
        vq_out=t[8],
    )


def parse_hand_telem_v2(payload: bytes) -> Optional[dict]:
    if len(payload) != TELEM_FRAME_SIZE:
        return None
    seq, diag_u16, tick_ms = struct.unpack("<HHI", payload[:8])
    off = 8
    linear: List[LinearMotorSlot] = []
    for _ in range(LINEAR_MOTOR_NUM):
        linear.append(_parse_linear_slot(payload[off : off + LINEAR_SLOT_SIZE]))
        off += LINEAR_SLOT_SIZE

    hand_type = int(payload[off + 1]) if (off + 1) < len(payload) else 0
    rotary: List[RotaryMotorSlot] = []
    for _ in range(ROTARY_MOTOR_NUM):
        rotary.append(_parse_rotary_slot(payload[off : off + ROTARY_SLOT_SIZE]))
        off += ROTARY_SLOT_SIZE
    imu_valid, imu_flag, roll, pitch, yaw = struct.unpack(
        "<" + IMU_TELEM_FMT, payload[off : off + IMU_TELEM_SIZE]
    )
    return {
        "seq": seq,
        "tick_ms": tick_ms,
        "imu_ok_cnt": int(diag_u16 & 0xFF),
        "imu_hdr_cnt": int((diag_u16 >> 8) & 0xFF),
        "hand_type": hand_type,
        "linear": linear,
        "rotary": rotary,
        "imu_valid": int(imu_valid),
        "imu_flag": int(imu_flag),
        "imu_roll_deg": float(roll),
        "imu_pitch_deg": float(pitch),
        "imu_yaw_deg": float(yaw),
        "fmt": "v2",
        "_parsed": True,
    }


def parse_hand_telem_lazy(payload: bytes) -> Optional[dict]:
    """RX 热路轻量解析：只解帧头，槽位与 IMU 延后到读取线程。"""
    if len(payload) != TELEM_FRAME_SIZE:
        return None
    seq, _diag_u16, tick_ms = struct.unpack("<HHI", payload[:8])
    return {
        "seq": seq,
        "tick_ms": tick_ms,
        "payload": payload,
        "linear": None,
        "rotary": None,
        "imu_valid": 0,
        "imu_flag": 0,
        "imu_roll_deg": 0.0,
        "imu_pitch_deg": 0.0,
        "imu_yaw_deg": 0.0,
        "fmt": "v2",
        "_parsed": False,
    }


def empty_linear_slot() -> LinearMotorSlot:
    return LinearMotorSlot(
        mech_home_state=0,
        drv_auto_recover=0,
        pos_act_mm=0.0,
        iq_meas=0.0,
        iq_ref=0.0,
        pos_ref_mm=0.0,
        pos_err_mm=0.0,
        grav_iq=0.0,
        status_b_u32=0,
        can_rx_error_cnt=0,
        can_tx_error_cnt=0,
        mcu_temp_centi=0,
        elec_angle_rad=0.0,
    )


def remap_linear_slots_by_tag(linear: List[LinearMotorSlot]) -> List[LinearMotorSlot]:
    """当前 32B 协议不携带身份标签，槽位数组下标即为权威映射。"""
    return list(linear)


def sanitize_telem_linear_slots(linear: List[LinearMotorSlot]) -> None:
    """保留与 main_app_v3 一致的扩展点；当前无标签布局无需改写槽位。"""
    _ = linear


def validate_telem_frame_v2(frame: dict) -> bool:
    """校验聚合遥测的结构，单槽扩展校验不得丢弃整帧。"""
    linear = frame.get("linear")
    rotary = frame.get("rotary")
    if not isinstance(linear, list) or len(linear) != LINEAR_MOTOR_NUM:
        return False
    if not isinstance(rotary, list) or len(rotary) != ROTARY_MOTOR_NUM:
        return False
    frame["linear"] = remap_linear_slots_by_tag(linear)
    sanitize_telem_linear_slots(frame["linear"])
    return True


def materialize_telem_frame(frame: dict) -> Optional[dict]:
    """将 parse_hand_telem_lazy 的轻量帧完整解析并校验。"""
    if frame.get("_parsed"):
        return frame if validate_telem_frame_v2(frame) else None
    payload = frame.get("payload")
    if not isinstance(payload, (bytes, bytearray)) or len(payload) != TELEM_FRAME_SIZE:
        return None
    full = parse_hand_telem_v2(bytes(payload))
    if full is None or not validate_telem_frame_v2(full):
        return None
    full["_parsed"] = True
    return full


def parse_hand_telem_legacy(payload: bytes) -> Optional[dict]:
    if len(payload) < LEGACY_TELEM_FRAME_SIZE:
        return None
    seq, _reserved_h, tick_ms = struct.unpack("<HHI", payload[:8])
    off = 8
    linear: List[LinearMotorSlot] = []
    for _ in range(LINEAR_MOTOR_NUM):
        linear.append(
            _legacy_slot_to_linear(payload[off : off + LEGACY_TELEM_SLOT_SIZE])
        )
        off += LEGACY_TELEM_SLOT_SIZE
    rotary: List[RotaryMotorSlot] = []
    for _ in range(ROTARY_MOTOR_NUM):
        rotary.append(_parse_rotary_slot(payload[off : off + LEGACY_TELEM_SLOT_SIZE]))
        off += LEGACY_TELEM_SLOT_SIZE
    return {
        "seq": seq,
        "tick_ms": tick_ms,
        "linear": linear,
        "rotary": rotary,
        "fmt": "legacy",
    }


def parse_hand_telem(payload: bytes) -> Optional[dict]:
    """解析当前 v2 聚合遥测；旧 932B 布局请显式调用 legacy 解析器。"""
    frame = parse_hand_telem_v2(payload)
    if frame is None or not validate_telem_frame_v2(frame):
        return None
    return frame


def telem_seq_accept(prev_seq: int, seq: int) -> bool:
    """seq 单调前进；拒绝重复/旧帧，允许 uint16 回绕或重连重同步。"""
    if prev_seq < 0:
        return True
    delta = (int(seq) - int(prev_seq)) & 0xFFFF
    if delta == 0:
        return False
    if delta <= 512:
        return True
    return delta >= 0xF000


def telem_linear_fingerprint(frame: dict) -> int:
    """返回直线槽 pos/iq 的 16-bit 调试指纹。"""
    linear = frame.get("linear")
    if isinstance(linear, list) and linear and frame.get("_parsed"):
        blob = bytearray()
        for slot in linear:
            blob.extend(struct.pack("<ff", slot.pos_act_mm, slot.iq_meas))
        return zlib.crc32(blob) & 0xFFFF
    payload = frame.get("payload")
    if isinstance(payload, (bytes, bytearray)):
        return zlib.crc32(payload[8:]) & 0xFFFF
    return 0
