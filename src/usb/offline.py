"""
usb_offline.py — USB 串口下位机通信（Dexterous Hand V3 协议）

协议与 main_app_v3.py / UsbH7App_User.h 一致。
Form2 动作播放 / 手势同步下位机时，默认发送整手批量位置帧（FUN_HAND_POS_BATCH）。
另含五指顺序往返（FingerSeqSweepWorker）与轻量 SerialController（原 test_usb）。

依赖: pip install pyserial

────────────────────────────────────────────────────────────────────────
一、USB 下行帧通用格式（共 96 字节，整手批量）
────────────────────────────────────────────────────────────────────────

  偏移     长度   字段              说明
  ────────────────────────────────────────────────────────────────────
  [0-1]     2B    AA 55             帧头魔数
  [2-3]     2B    len (u16 LE)      payload 长度 = 0x0058 (88)
  [4]       1B    Device            0x03 = TELEM（遥测/控制设备）
  [5]       1B    Fun               0x20 = 整手批量位置 (HAND_POS_BATCH)
  [6]       1B    seq               帧序号 0~255，每发一帧 +1
  [7-62]   56B    linear[14]        14 × float32 LE，单位 mm
  [63-90]  28B    rotary[7]         7 × float32 LE，单位 deg
  [91-93]   3B    pad               固定填 0（对齐用）
  [94-95]   2B    CRC16             Modbus CRC，覆盖 [0..93]

  payload 布局（88 B，pack_hand_pos_batch）:
    offset +0      seq           u8
    offset +1~+56  linear[14]    14 × f32 LE (mm)
    offset +57~+84 rotary[7]     7 × f32 LE (deg)
    offset +85~+87 pad           3 × u8 = 0

  示意图:
    ┌────────────────────────────────────────────────────────────────┐
    │ AA 55 │ len │ 03 │ 20 │ seq │ linear[14]×4B │ rotary[7]×4B │ pad │ CRC │
    │  2B   │ 2B  │ 1B │ 1B │ 1B │   56B (mm)    │  28B (deg)   │ 3B  │ 2B  │
    └────────────────────────────────────────────────────────────────┘

────────────────────────────────────────────────────────────────────────
二、直线电机槽位 L00~L13（帧 [7]~[62]，payload +1~+56，单位 mm）
────────────────────────────────────────────────────────────────────────

  槽位   帧字节      硬件总线        电机   URDF 关节                         90°行程
  ────────────────────────────────────────────────────────────────────────────────────
  L00    [7-10]      FDCAN1 (CAN1)   M1     left_hand_ring_MCP_joint  无名指 MCP   16 mm
  L01    [11-14]     FDCAN1          M2     left_hand_ring_PIP_joint  无名指 PIP   14 mm  # DIP/PIP 已实机验证
  L02    [15-18]     FDCAN1          M3     left_hand_ring_DIP_joint  无名指 DIP   12 mm  # DIP/PIP 已实机验证
  L03    [19-22]     FDCAN1          M4     left_hand_pinky_MCP_joint 小指 MCP    16 mm
  L04    [23-26]     FDCAN1          M5     left_hand_pinky_PIP_joint 小指 PIP    14 mm  # DIP/PIP 已实机验证
  L05    [27-30]     FDCAN1          M6     left_hand_pinky_DIP_joint 小指 DIP    12 mm  # DIP/PIP 已实机验证
  L06    [31-34]     FDCAN2 (CAN2)   M1     left_hand_index_MCP_joint 食指 MCP    16 mm
  L07    [35-38]     FDCAN2          M2     left_hand_index_PIP_joint 食指 PIP    14 mm  # DIP/PIP 已实机验证
  L08    [39-42]     FDCAN2          M3     left_hand_index_DIP_joint 食指 DIP    12 mm  # DIP/PIP 已实机验证
  L09    [43-46]     FDCAN2          M4     left_hand_mid_MCP_joint   中指 MCP    16 mm
  L10    [47-50]     FDCAN2          M5     left_hand_mid_PIP_joint   中指 PIP    14 mm  # DIP/PIP 已实机验证
  L11    [51-54]     FDCAN2          M6     left_hand_mid_DIP_joint   中指 DIP    12 mm  # DIP/PIP 已实机验证
  L12    [55-58]     FDCAN3 (CAN3)   M1     left_hand_thumb_MCP_joint 拇指 MCP    16 mm
  L13    [59-62]     FDCAN3          M2     left_hand_thumb_PIP_joint 拇指 PIP    12 mm

  总线分组（LINEAR_BUS_COUNTS=(6,6,2), LINEAR_BUS_BASE=(0,6,12)）:
    CAN1 / FDCAN1 → L00~L05（无名指 + 小指，6 台）
    CAN2 / FDCAN2 → L06~L11（食指 + 中指，6 台）
    CAN3 / FDCAN3 → L12~L13（拇指，2 台）

────────────────────────────────────────────────────────────────────────
三、旋转电机槽位 R00~R06（帧 [63]~[90]，payload +57~+84，单位 deg）
────────────────────────────────────────────────────────────────────────

  槽位   帧字节      硬件 SPI   电机   URDF 关节
  ────────────────────────────────────────────────────────────────────────
  R00    [63-66]     SPI1       M1     left_hand_index_swing_joint   食指摆动
  R01    [67-70]     SPI1       M2     left_hand_mid_swing_joint     中指摆动
  R02    [71-74]     SPI2       M1     left_hand_ring_swing_joint    无名指摆动
  R03    [75-78]     SPI2       M2     left_hand_pinky_swing_joint   小指摆动
  R04    [79-82]     SPI4       M1     left_hand_thumb_rota_joint    拇指旋转
  R05    [83-86]     SPI4       M2     left_hand_thumb_swing_joint   拇指摆动
  R06    [87-90]     SPI3       M1     left_hand_pinky_rota_joint    小指旋转

  注意: USB 槽顺序为 SPI1 → SPI2 → SPI4 → SPI3（与 main_app_v3 rotary_caption 一致），
        不是按 SPI1/2/3/4 连续编号。ROTARY_BUS_BASE = (0, 2, 6, 4)。

────────────────────────────────────────────────────────────────────────
四、关节角空间与全部转换（左右手说明）
────────────────────────────────────────────────────────────────────────

  H7 协议文档约定：旋转指令正值=顺时针，负值=逆时针。
  这里的顺/逆时针是固件/电机视角约定；URDF 的 q 正方向则遵循关节局部轴右手定则，
  两者通过 protocol.hand_angle_protocol 中的 UI→HW 映射衔接。

  ***********************  左 手 (LEFT)  ***********************
  *                                                             *
  *     [thumb_rota]              [thumb_swing]                 *
  *          |                         |                        *
  *          +--------- 拇指 ----------+                        *
  *          CW/CCW:__CCW__               CW/CCW:__CCW__              *
  *                                                             *
  *  [index_swing]  [mid_swing]  [ring_swing]  [pinky_swing]    *
  *       |              |            |              |          *
  *     食指           中指         无名指          小指         *
  *   CW/CCW:__CW__    CW/CCW:__CW__  CW/CCW:__CW__    CW/CCW:__CW__    *
  *                                                  |          *
  *                                           [pinky_rota]      *
  *                                           CW/CCW:__CW__       *
  *                                                             *
  *  下发范围：                                                  *
  *    四指 swing : [+0.5°, +39.5°] ← 同号，H7边界截断              *
  *    pinky_rota : [  0°, +50°]                                *
  *    thumb_rota : [+0.5°, +89.5°] ← 与 UI 同号，协议层直接钳位       *
  *    thumb_swing: [0°, 114.59°]（H7 [0,2] rad；UI 同值）    *
  *    四指/拇指直线 MCP·PIP·DIP : 负 mm（stroke_sign=-1）       *
  ***************************************************************

  ***********************  右 手 (RIGHT)  **********************
  *                                                             *
  *     [thumb_rota]              [thumb_swing]                 *
  *          |                         |                        *
  *          +--------- 拇指 ----------+                        *
  *          CW/CCW:__CW__               CW/CCW:__CW__              *
  *                                                             *
  *  [index_swing]  [mid_swing]  [ring_swing]  [pinky_swing]    *
  *       |              |            |              |          *
  *     食指           中指         无名指          小指         *
  *   CW/CCW:__CW__    CW/CCW:__CW__  CW/CCW:__CW__    CW/CCW:__CW__    *
  *                                                  |          *
  *                                           [pinky_rota]      *
  *                                           CW/CCW:__CCW__       *
  *                                                             *
  *  下发范围：                                                  *
  *    四指 swing : [  0°, +39.88°] ← 与 UI 同号，不取负          *
  *    pinky_rota : [-50°,  0]                                  *
  *    thumb_rota : [+0.5°, +89.5°] ← 与 UI 同号，协议层直接钳位       *
  *    thumb_swing: [0°, 114.59°]（H7 [0,2] rad；UI 同值）    *
  *    四指/拇指直线 MCP·PIP·DIP : 负 mm（与左手相同，不另反号） *
  ***************************************************************

  ┌─ 空间定义 ─────────────────────────────────────────────────────────┐
  │ A. 界面/内存（Form2 滑块）：键名一律 left_hand_*；旋转量均为正区间   │
  │ B. 动作文件 / USB 下发 / 遥测：硬件约定（按下图左右手符号）         │
  │ C. URDF 显示：左手 left_hand_*；右手映射为 right_hand_*             │
  └────────────────────────────────────────────────────────────────────┘

  角度变号（UI ↔ Hardware）已迁至 protocol.hand_angle_protocol：
    convert_ui_to_hardware / convert_hardware_to_ui
    （旧名 remap_joint_angles_for_downlink / remap_motion_cfg_* 仍可从 protocol 导入）

  本模块只负责：Hardware(rad) → mm/deg 打包 → 串口下发。
  调用约定：先 protocol 转换，再 UsbController.send_joint_angles(hw_cfg)。

  Form2 显示：同一 UI 数值只切换左右手 URDF 键名前缀；镜像方向由 URDF 关节轴承担。
  0x2B hand_type 只配置固件左右手/找零 Iq，不额外反转 0x20 位置数组。
  未映射关节（如拇指 DIP 虚拟关节）不写入 payload，对应槽保持 0。

────────────────────────────────────────────────────────────────────────
五、示例帧（全零姿态）
────────────────────────────────────────────────────────────────────────

  AA 55 58 00 03 20 00 00 00 00 00 ... 00 00 00 00 35 35
  │    │     │  │    │   └─ L00~L13 全 0 mm
  │    │     │  │    └─ seq=0
  │    │     │  └─ Fun=0x20 整手批量
  │    │     └─ Dev=0x03 TELEM
  │    └─ len=88 (0x0058)
  └─ 帧头

  含义: 序号 0，14 个直线电机目标 0 mm，7 个旋转电机目标 0°，CRC=0x3535。

────────────────────────────────────────────────────────────────────────
六、其他下行 Fun Code（Device=TELEM/3）
────────────────────────────────────────────────────────────────────────

  0x10  FUN_LM_POS_REF      单点直线位置
  0x11  FUN_LM_MOTOR_PWR    直线电机使能/断电
  0x12  FUN_LM_MECH_HOMING  直线电机机械回零
  0x13~0x17                直线 PID / 正弦 / PWM / Vq / Boot
  0x18 / 0x19              直线 / 旋转批量位置
  0x1A~0x1D                直线/旋转重力开关、无IMU策略、重力总开关
  0x20  FUN_HAND_POS_BATCH  整手批量位置（本模块主要使用）
  0x21  FUN_RM_POS_SET_SP   旋转 SPI 单点位置
  0x22  FUN_RM_DRIVE_SP     旋转 SPI 使能/断电
  0x23~0x27                旋转位置 PID / 正弦 / 回零 / PWM / Vq
  0x28~0x2A                旋转电流 PID / 电流环 / 开环旋转
  0x2B  FUN_RM_HAND_TYPE_SET_SP  H7 Flash 左右手配置
  0x2C  FUN_RM_ELEC_CALIB_SP     旋转电角度离线标定
  0x2D  FUN_RM_ELEC_OFFSET_SP    旋转电角度偏置
"""

from __future__ import annotations

from kinematics.finger_linkage import (
    FingerLinkageParams,
    MOTOR_F1,
    MOTOR_F3,
    HandStrokeMapper,
    ThumbStrokeMapper,
    _log_kin_failure,
    cf_to_phi_deg,
    create_default_hand_stroke_mapper,
    finalize_linkage_params,
    joint_deg_to_phi_rad,
)
import math
import os
import struct
import sys
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, List, Mapping, Optional, Sequence, Tuple

from utils.comm_diag import (
    info as comm_diag_info,
    record_sweep_tick,
    record_usb_rx,
    record_usb_serial_state,
    record_usb_tx,
    snapshot_usb_port,
    warn as comm_diag_warn,
)
from utils.log_control import emit as log_emit, is_enabled as tag_enabled
from utils.control_session_recorder import RECORDER as _control_session_recorder
from protocol import (
    normalize_hand_side as _normalize_hand_side,
    pinky_rota_hw_to_ui as _pinky_rota_hw_to_ui,
    remap_joint_angles_for_downlink,
    remap_motion_cfg_file_to_ui,
    remap_motion_cfg_ui_to_file,
    swing_hw_to_urdf,
    swing_urdf_to_hw,
    thumb_rota_hw_to_ui as _thumb_rota_hw_to_ui,
    thumb_swing_hw_to_ui,
    thumb_swing_ui_to_hw,
)

# 流式下发背压阈值（字节）：超过此值时跳帧，避免 TX 缓冲积压引起位置滞后
SERIAL_TX_BACKPRESSURE_BYTES = 4096
# 最新聚合遥测线长 788B（payload 780B）；须尽快 drain OS RX，避免 H7 CDC 背压
RX_DRAIN_MAX_BYTES_PER_LOOP = 262144
RX_DRAIN_CHUNK_BYTES = 65536
RX_PARSER_BACKLOG_BYTES = 8192
RX_IDLE_SLEEP_S = 0.0005

try:
    import serial
    from serial.tools import list_ports
    _SERIAL_AVAILABLE = True
    _SERIAL_TIMEOUT_EXC: tuple = (serial.SerialTimeoutException,)
except ImportError:
    serial = None
    list_ports = None
    _SERIAL_AVAILABLE = False
    _SERIAL_TIMEOUT_EXC = ()

# ── 协议常量 ──
USB_DOWNLINK_HDR = (0xAA, 0x55)
DEV_TELEM     = 0x03
DEV_FDCAN     = 0x00
DEV_SPI       = 0x01
DEV_IMU       = 0x02

# 下行 Fun Code（Device=TELEM/3）
FUN_HEARTBEAT       = 0x01
FUN_TELEM           = 0x02
FUN_LM_POS_REF      = 0x10  # 单点位置
FUN_LM_MOTOR_PWR    = 0x11  # 使能/断电
FUN_LM_MECH_HOMING  = 0x12  # 机械回零
FUN_LM_PID_GAIN     = 0x13
FUN_LM_SINE_MOTION  = 0x14
FUN_LM_PWM_REF      = 0x15
FUN_LM_VQ_REF       = 0x16
FUN_LM_BOOT_REQ     = 0x17
FUN_LM_POS_BATCH    = 0x18
FUN_RM_POS_BATCH    = 0x19
FUN_LM_GRAVITY_ENABLE = 0x1A
FUN_RM_GRAVITY_ENABLE = 0x1B
FUN_GRAVITY_ALLOW_NO_IMU = 0x1C
FUN_GRAVITY_MASTER_ENABLE = 0x1D
FUN_HAND_POS_BATCH  = 0x20  # 整手批量位置
FUN_RM_POS_SET_SP   = 0x21  # 旋转 SPI 单点位置 → SPI 0x21
FUN_RM_DRIVE_SP     = 0x22  # 旋转 SPI 使能/断电 → SPI 0x22
FUN_RM_PID_SP       = 0x23
FUN_RM_SINE_SP      = 0x24
FUN_RM_ZERO_SP      = 0x25  # 旋转 SPI 机械回零 → SPI 0x25
FUN_RM_PWM_SP       = 0x26
FUN_RM_VQ_SP        = 0x27
FUN_RM_CUR_PID_SP   = 0x28
FUN_RM_CUR_LOOP_SP  = 0x29
FUN_RM_OPEN_SPIN_SP = 0x2A
FUN_RM_HAND_TYPE_SET_SP = 0x2B
FUN_RM_ELEC_CALIB_SP = 0x2C
FUN_RM_ELEC_OFFSET_SP = 0x2D

RM_CUR_LOOP_FLAG_ENABLE = 0x01
RM_OPEN_SPIN_FLAG_ENABLE = 0x01
RM_ELEC_CALIB_FLAG_RUN = 0x01
RM_ELEC_OFFSET_FLAG_SAVE = 0x01

LM_GRAVITY_ENABLE_ON = 0x01
LM_GRAVITY_ENABLE_OFF = 0x02
LM_GRAVITY_ENABLE_BROADCAST_ID = 0xFF
BOOT_REQ_MAGIC = 0x21475055

# 旋转 SPI 总线（与 main_app_v3 ROTARY_BUS_* 一致）
# spi_bus: 0=SPI1, 1=SPI2, 2=SPI3(仅M0), 3=SPI4
ROTARY_SPI_BUS_COUNTS = (2, 2, 1, 2)
ROTARY_HW_SPI_LABELS = ("SPI1", "SPI2", "SPI3", "SPI4")
ROTARY_BUS_BASE = (0, 2, 6, 4)

# 与 deprecated/main_app_v3.py 同步的旋转位置符号开关。当前项目的左右手
# 变换已集中在 protocol.convert_ui_to_hardware，因此这里必须保持 False，
# pack 层只保留参考程序的兼容接口，避免主机与 H7 重复取反。
ROTARY_DOWNLINK_SIGN_ON_HOST = False
ROTARY_SIGN_NEG_SLOTS_LEFT: Tuple[int, ...] = (0, 1, 2, 3, 5)
ROTARY_SIGN_NEG_SLOTS_RIGHT: Tuple[int, ...] = (4,)
_rotary_downlink_hand_type = 0

# USB 旋转槽 → 手指与关节部位（校准界面显示用）
ROTARY_SLOT_FINGER_PART: Dict[int, Tuple[str, str]] = {
    0: ("食指", "摆动关节"),
    1: ("中指", "摆动关节"),
    2: ("无名指", "摆动关节"),
    3: ("小指", "摆动关节"),
    4: ("拇指", "旋转关节"),
    5: ("拇指", "摆动关节"),
    6: ("小指", "旋转关节"),
}

# 校准步骤英文标识（与 URDF 关节名对应）
ROTARY_SLOT_CALIB_ID: Dict[int, str] = {
    0: "index_swing",
    1: "mid_swing",
    2: "ring_swing",
    3: "pinky_swing",
    4: "thumb_rota",
    5: "thumb_swing",
    6: "pinky_rota",
}


def rotary_bus_slot_index(spi_bus: int, local_motor: int) -> Optional[int]:
    """SPI 总线 + 本板电机号 → USB rotary 槽下标（与 main_app_v3 一致）。"""
    if spi_bus < 0 or spi_bus >= len(ROTARY_SPI_BUS_COUNTS):
        return None
    if local_motor < 0 or local_motor >= ROTARY_SPI_BUS_COUNTS[spi_bus]:
        return None
    return ROTARY_BUS_BASE[spi_bus] + local_motor


def set_rotary_downlink_hand_type(hand_type: int) -> None:
    """同步参考程序的 pack 缺省手型：0=左手，1=右手。"""
    global _rotary_downlink_hand_type
    _rotary_downlink_hand_type = 0 if int(hand_type) == 0 else 1


def get_rotary_downlink_hand_type() -> int:
    return int(_rotary_downlink_hand_type)


def rotary_slot_needs_neg(usb_slot: int, hand_type: Optional[int] = None) -> bool:
    """返回参考 H7 符号表中指定旋转槽是否需要取反。"""
    ht = (
        get_rotary_downlink_hand_type()
        if hand_type is None
        else (0 if int(hand_type) == 0 else 1)
    )
    if usb_slot < 0 or usb_slot >= ROTARY_MOTOR_NUM:
        return False
    slots = ROTARY_SIGN_NEG_SLOTS_LEFT if ht == 0 else ROTARY_SIGN_NEG_SLOTS_RIGHT
    return usb_slot in slots


def apply_rotary_downlink_sign(
    positions: Sequence[float],
    hand_type: Optional[int] = None,
    *,
    enabled: Optional[bool] = None,
) -> List[float]:
    """按参考程序符号表转换旋转槽；默认关闭，当前由 protocol 统一转换。"""
    out = [float(v) for v in positions]
    if not (ROTARY_DOWNLINK_SIGN_ON_HOST if enabled is None else bool(enabled)):
        return out
    while len(out) < ROTARY_MOTOR_NUM:
        out.append(0.0)
    for slot in range(min(len(out), ROTARY_MOTOR_NUM)):
        if rotary_slot_needs_neg(slot, hand_type):
            out[slot] = -out[slot]
    return out


def apply_rotary_bus_pos_sign(
    spi_bus: int,
    pos0_deg: float,
    pos1_deg: float,
    hand_type: Optional[int] = None,
    *,
    enabled: Optional[bool] = None,
) -> Tuple[float, float]:
    """同步参考程序的SPI双电机符号转换兼容接口。"""
    if not (ROTARY_DOWNLINK_SIGN_ON_HOST if enabled is None else bool(enabled)):
        return float(pos0_deg), float(pos1_deg)
    p0, p1 = float(pos0_deg), float(pos1_deg)
    slot0 = rotary_bus_slot_index(spi_bus, 0)
    slot1 = rotary_bus_slot_index(spi_bus, 1)
    if slot0 is not None and rotary_slot_needs_neg(slot0, hand_type):
        p0 = -p0
    if slot1 is not None and rotary_slot_needs_neg(slot1, hand_type):
        p1 = -p1
    return p0, p1


def rotary_finger_part_label(spi_bus: int, local_motor: int) -> Tuple[str, str]:
    """返回 (手指名, 部位名)；未知槽位时回退为 SPI 标注。"""
    idx = rotary_bus_slot_index(spi_bus, local_motor)
    if idx is not None and idx in ROTARY_SLOT_FINGER_PART:
        return ROTARY_SLOT_FINGER_PART[idx]
    hw = ROTARY_HW_SPI_LABELS[spi_bus] if 0 <= spi_bus < len(ROTARY_HW_SPI_LABELS) else f"SPI{spi_bus}"
    return (hw, f"M{local_motor}")


def rotary_calib_step_label(spi_bus: int, local_motor: int) -> str:
    """校准步骤英文标注，如 index_swing（SPI0 M0）。"""
    idx = rotary_bus_slot_index(spi_bus, local_motor)
    calib_id = (
        ROTARY_SLOT_CALIB_ID[idx]
        if idx is not None and idx in ROTARY_SLOT_CALIB_ID
        else f"slot{idx if idx is not None else '?'}"
    )
    return f"{calib_id}（SPI{spi_bus} M{local_motor}）"

# 直线电机使能/断电命令字
LM_MOTOR_PWR_ENABLE  = 0x01
LM_MOTOR_PWR_DISABLE = 0x02

# 机械回零命令字
LM_HOMING_START = 0x01   # 开始寻零
LM_HOMING_RESET = 0x02   # 复位回零状态

# 默认寻零电流（mA 量级，与 main_app_v3 一致）
DEFAULT_HOMING_IQ_SEEK = 700.0

# CAN 总线布局（与 main_app_v3 LINEAR_BUS_COUNTS / LINEAR_BUS_BASE 一致）
LINEAR_BUS_COUNTS = (6, 6, 2)   # CAN1/2/3 各 N 台电机
LINEAR_BUS_BASE   = (0, 6, 12)  # 对应 linear 槽起始

LINEAR_MOTOR_NUM = 14
ROTARY_MOTOR_NUM = 7
HAND_POS_BATCH_SIZE = 88
LM_POS_BATCH_SIZE = 64
RM_POS_BATCH_SIZE = 33

# 上行聚合遥测（与 main_app_v3 一致）：payload 780B，线帧约 788B
LINEAR_SLOT_SIZE = 32
ROTARY_SLOT_SIZE = 44
IMU_TELEM_SIZE = 16
TELEM_FRAME_SIZE = (
    8 + LINEAR_SLOT_SIZE * LINEAR_MOTOR_NUM
    + ROTARY_SLOT_SIZE * ROTARY_MOTOR_NUM
    + IMU_TELEM_SIZE
)  # 780
LEGACY_TELEM_FRAME_SIZE = 8 + ROTARY_SLOT_SIZE * (LINEAR_MOTOR_NUM + ROTARY_MOTOR_NUM)  # 932

STM_VID = 0x0483
STM_USB_CDC_PID = 0x5740
STM_STLINK_VCP_PID = 0x374F

# 波特率：USB CDC 下多为虚拟值，但 Windows 驱动仍会参考；可用环境变量 HAND_SERIAL_BAUD 提高（如 921600）
SERIAL_BAUDRATE = int(os.environ.get("HAND_SERIAL_BAUD", "115200"))
SERIAL_WRITE_TIMEOUT_S = 0.05
SERIAL_READ_TIMEOUT_S = 0.002

# 蓝牙 SPP 等虚拟串口在链路拥塞/短暂干扰时，单次 write/read 超时或
# WinError 很常见，但物理连接通常并未真的断开。若一遇到就整段 disconnect()，
# 会表现为“非常容易断开”。这里改成连续失败计数：只有连续超过阈值次数
# （对应 500Hz 流式约 20~30ms）才判定为真正掉线，单次抖动只记日志/跳帧。
SERIAL_TX_FAIL_STREAK_LIMIT = int(os.environ.get("HAND_SERIAL_TX_FAIL_LIMIT", "10"))
SERIAL_RX_FAIL_STREAK_LIMIT = int(os.environ.get("HAND_SERIAL_RX_FAIL_LIMIT", "20"))
SERIAL_RX_FAIL_RETRY_SLEEP_S = 0.02
# connect() 重试：蓝牙 SPP 刚配对/唤醒时打开端口可能瞬时失败，重试几次再放弃。
SERIAL_CONNECT_RETRIES = int(os.environ.get("HAND_SERIAL_CONNECT_RETRIES", "3"))
SERIAL_CONNECT_RETRY_DELAY_S = 0.25
HAND_POS_FRAME_LEN = 6 + HAND_POS_BATCH_SIZE + 2
USB_UPLINK_HDR = (0x55, 0xAA)
_UPLINK_MAX_FRAME = 4096
_TELEM_ACCEPT_PLEN = frozenset({TELEM_FRAME_SIZE, LEGACY_TELEM_FRAME_SIZE})
HEARTBEAT_MAGIC = 0x42483748

# 手势/动作/五指往返共用的下行定频（可用 HAND_SEND_HZ 覆盖）
# 500Hz 对控制足够；若 GUI 仍卡可先设 HAND_SEND_HZ=200
DOWNLINK_HZ_DEFAULT = float(os.environ.get("HAND_SEND_HZ", "500"))
DL_SWEEP_PERIOD_S_MIN = 0.05
# wait_until 末段自旋窗口（秒）。旧值 2ms 会在 500Hz 下整段忙等，吃满一核。
_WAIT_SPIN_S = float(os.environ.get("HAND_WAIT_SPIN_S", "0.00025"))

# 五指顺序往返：USB 直线槽分组（与 main_app_v3 DL_FINGER_SEQUENTIAL_SWEEP_GROUPS 一致）
FINGER_SEQ_GROUPS: Tuple[Tuple[str, Tuple[int, ...]], ...] = (
    ("拇指", (12, 13)),
    ("食指", (6, 7, 8)),
    ("中指", (9, 10, 11)),
    ("无名指", (0, 1, 2)),
    ("小指", (3, 4, 5)),
)


def theoretical_line_max_hz(baud: int = SERIAL_BAUDRATE) -> float:
    """8N1 下按波特率估算的 96B 帧线速上限（参考值，USB CDC 实际可能更高）。"""
    return baud / (HAND_POS_FRAME_LEN * 10)


def wait_until(deadline: float) -> None:
    """精确等待并释放 GIL，避免饿死 Qt 主线程与相机线程。

    策略：大部分时间 ``time.sleep``；仅在最后 ``HAND_WAIT_SPIN_S``（默认 250µs）
    内短自旋，兼顾 500Hz 定频与 GUI 流畅。切勿把自旋窗口设到 ≥ 下行周期。
    """
    spin_s = max(_WAIT_SPIN_S, 0.0)
    while True:
        remain = deadline - time.perf_counter()
        if remain <= 0.0:
            return
        if remain > spin_s:
            # 提前一点醒来再进入短自旋，避免 sleep 过冲
            time.sleep(remain - spin_s * 0.5)
        else:
            # 末段短暂让出 GIL；窗口很小，不会再占满 CPU
            time.sleep(0)


def verify_hand_pos_frame(frame: bytes) -> bool:
    """校验整手批量下行帧（96 B）结构与 CRC。"""
    if len(frame) != HAND_POS_FRAME_LEN:
        return False
    if frame[0:2] != bytes(USB_DOWNLINK_HDR):
        return False
    plen = frame[2] | (frame[3] << 8)
    if plen != HAND_POS_BATCH_SIZE:
        return False
    if frame[4] != DEV_TELEM or frame[5] != FUN_HAND_POS_BATCH:
        return False
    body = frame[:-2]
    crc_rx = frame[-2] | (frame[-1] << 8)
    return crc16_modbus(body) == crc_rx


def sine_sweep_pos(elapsed_s: float, pos_start: float, pos_end: float, period_s: float) -> float:
    """整周期正弦往返：pos_start → pos_end → pos_start，端点速度为 0。

    blend = (1 - cos(2π·t/T)) / 2
    """
    if period_s < DL_SWEEP_PERIOD_S_MIN:
        period_s = DL_SWEEP_PERIOD_S_MIN
    span = pos_end - pos_start
    if span == 0.0:
        return pos_start
    t = elapsed_s % period_s
    blend = (1.0 - math.cos(2.0 * math.pi * t / period_s)) * 0.5
    return pos_start + span * blend


def _set_thread_priority(name: str) -> None:
    if sys.platform != "win32":
        return
    try:
        import ctypes
        k32 = ctypes.windll.kernel32  # type: ignore[attr-defined]
        pri_map = {"above_normal": 1, "below_normal": -1, "normal": 0}
        k32.SetThreadPriority(k32.GetCurrentThread(), pri_map.get(name, 0))
    except Exception:
        pass


def _drain_serial_port(
    ser: "serial.Serial",
    *,
    max_bytes: int = RX_DRAIN_MAX_BYTES_PER_LOOP,
) -> bytes:
    """排空驱动 RX 缓冲；仅在完全空闲时做一次短 timeout 阻塞 read。"""
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
        block = ser.read(min(RX_DRAIN_CHUNK_BYTES, max_bytes))
        if block:
            parts.append(block)
            total += len(block)
        break
    return b"".join(parts)


# 21 个 URDF 关节名（与 skeleton_tracker / form2 一致）
JOINT_ORDER: List[str] = [
    "left_hand_thumb_rota_joint",
    "left_hand_thumb_swing_joint",
    "left_hand_thumb_MCP_joint",
    "left_hand_thumb_PIP_joint",
    "left_hand_index_swing_joint",
    "left_hand_index_MCP_joint",
    "left_hand_index_PIP_joint",
    "left_hand_index_DIP_joint",
    "left_hand_mid_swing_joint",
    "left_hand_mid_MCP_joint",
    "left_hand_mid_PIP_joint",
    "left_hand_mid_DIP_joint",
    "left_hand_ring_swing_joint",
    "left_hand_ring_MCP_joint",
    "left_hand_ring_PIP_joint",
    "left_hand_ring_DIP_joint",
    "left_hand_pinky_rota_joint",
    "left_hand_pinky_swing_joint",
    "left_hand_pinky_MCP_joint",
    "left_hand_pinky_PIP_joint",
    "left_hand_pinky_DIP_joint",
]

# 旋转关节 → USB rotary 槽 R00~R06（帧 [63]~[90]，单位 deg）
# 详见文件顶部文档「三、旋转电机槽位」
ROTARY_JOINT_SLOTS: Dict[str, int] = {
    "left_hand_index_swing_joint": 0,   # R00  SPI1 M1  帧[63-66]  食指摆动
    "left_hand_mid_swing_joint": 1,     # R01  SPI1 M2  帧[67-70]  中指摆动
    "left_hand_ring_swing_joint": 2,    # R02  SPI2 M1  帧[71-74]  无名指摆动
    "left_hand_pinky_swing_joint": 3,   # R03  SPI2 M2  帧[75-78]  小指摆动
    "left_hand_thumb_rota_joint": 4,   # R04  SPI4 M1  帧[79-82]  拇指旋转
    "left_hand_thumb_swing_joint": 5,  # R05  SPI4 M2  帧[83-86]  拇指摆动
    "left_hand_pinky_rota_joint": 6,   # R06  SPI3 M1  帧[87-90]  小指旋转
}

# 直线关节 → USB linear 槽 L00~L13（帧 [7]~[62]，单位 mm）
# 总线: CAN1/FDCAN1 L00~L05 | CAN2/FDCAN2 L06~L11 | CAN3/FDCAN3 L12~L13
# 详见文件顶部文档「二、直线电机槽位」
LINEAR_JOINT_SLOTS: Dict[str, int] = {
    # CAN1 / FDCAN1
    # 四指 DIP/PIP 槽位：相对早期映射曾对调（L01↔L02 等），已实机验证无误，作为正式映射
    "left_hand_ring_MCP_joint":  0,   # L00  M1  帧[7-10]   无名指 MCP  16mm@90°
    "left_hand_ring_PIP_joint":  1,   # L01  M2  帧[11-14]  无名指 PIP（已实机验证）
    "left_hand_ring_DIP_joint":  2,   # L02  M3  帧[15-18]  无名指 DIP（已实机验证）
    "left_hand_pinky_MCP_joint": 3,   # L03  M4  帧[19-22]  小指 MCP   16mm@90°
    "left_hand_pinky_PIP_joint": 4,   # L04  M5  帧[23-26]  小指 PIP（已实机验证）
    "left_hand_pinky_DIP_joint": 5,   # L05  M6  帧[27-30]  小指 DIP（已实机验证）
    # CAN2 / FDCAN2
    "left_hand_index_MCP_joint": 6,   # L06  M1  帧[31-34]  食指 MCP   16mm@90°
    "left_hand_index_PIP_joint": 7,   # L07  M2  帧[35-38]  食指 PIP（已实机验证）
    "left_hand_index_DIP_joint": 8,   # L08  M3  帧[39-42]  食指 DIP（已实机验证）
    "left_hand_mid_MCP_joint":   9,   # L09  M4  帧[43-46]  中指 MCP   16mm@90°
    "left_hand_mid_PIP_joint":   10,  # L10  M5  帧[47-50]  中指 PIP（已实机验证）
    "left_hand_mid_DIP_joint":   11,  # L11  M6  帧[51-54]  中指 DIP（已实机验证）
    # CAN3 / FDCAN3
    "left_hand_thumb_MCP_joint": 12,  # L12  M1  帧[55-58]  拇指 MCP   16mm@90°
    "left_hand_thumb_PIP_joint": 13,  # L13  M2  帧[59-62]  拇指 PIP   12mm@90°
}

# ── 四指 swing：UI / 模型 / 文件为 [0, 0.696rad]，init 为 0 ──
# H7 下发符号由 protocol 处理：左手 [0.5°,39.5°]，右手 [0,+0.696rad]。
# 0.348rad只是UI数值量程中点，不是当前 URDF 的 q=0。
# 完整说明见本文件顶部「四、关节角空间与全部转换」。
_DEG_SWING_MAX = 0.696
_SWING_HW_MAX_DEG = math.degrees(_DEG_SWING_MAX)
_SWING_HW_MAX = _DEG_SWING_MAX
_SWING_CLAMP = (0.0, _DEG_SWING_MAX)                  # UI / 模型 / 文件
_SWING_UI_MIN = _SWING_CLAMP[0]
_SWING_UI_MAX = _SWING_CLAMP[1]
_SWING_UI_MID = _SWING_HW_MAX * 0.5                   # ≈ 0.2269（13°）
# 兼容旧名（曾表示 URDF± 与 mid 仿射）；现与 UI/HW 同空间
_SWING_URDF_MIN = _SWING_UI_MIN
_SWING_URDF_MAX = _SWING_UI_MAX
_SWING_MID_MAX = _SWING_UI_MID
_SWING_MID_MIN = -_SWING_UI_MID  # 仅保留符号对称常量，映射不再使用
_SWING_MID_SPAN = _SWING_HW_MAX
_SWING_HW_SPAN = _SWING_HW_MAX
_SWING_URDF_TO_MID_SCALE = 1.0

_FINGER_SWING_JOINTS = frozenset({
    "left_hand_index_swing_joint",
    "left_hand_mid_swing_joint",
    "left_hand_ring_swing_joint",
    "left_hand_pinky_swing_joint",
})

_NON_THUMB_FLEX_CLAMP = (0.0, 1.57)
_NON_THUMB_FLEX_JOINTS = frozenset(
    name for name in LINEAR_JOINT_SLOTS if "thumb" not in name
)
_PINKY_ROTA_JOINT = "left_hand_pinky_rota_joint"
_THUMB_ROTA_JOINT = "left_hand_thumb_rota_joint"
_THUMB_SWING_JOINT = "left_hand_thumb_swing_joint"

# UI 正区间（左右手滑块相同）；thumb_rota 使用模型完整 [0,1.57rad]
_PINKY_ROTA_UI_CLAMP = (0.0, math.radians(50.0))          # [0, 50°]
_THUMB_ROTA_UI_CLAMP = (0.0, 1.57)                        # [0, 89.95°]
_THUMB_SWING_UI_CLAMP = (0.0, 2.0)                        # 0..2 rad ≈ 0..114.59°
# 硬件覆盖左右两侧符号
_PINKY_ROTA_HW_CLAMP = (-math.radians(50.0), math.radians(50.0))
_THUMB_ROTA_HW_CLAMP = (math.radians(0.5), math.radians(89.5))
_THUMB_SWING_HW_CLAMP = (0.0, 2.0)
# 兼容旧名（曾用 1.57 / ±1,1]+1 映射）
_PINKY_ROTA_CLAMP = _PINKY_ROTA_UI_CLAMP


def _clamp_rad(rad: float, lo: float, hi: float) -> float:
    if rad < lo:
        return lo
    if rad > hi:
        return hi
    return rad


_AUCH_CRC_HI = bytes([
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40,
])
_AUCH_CRC_LO = bytes([
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
    0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
    0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
    0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
    0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
    0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
    0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
    0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
    0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
    0x40,
])

_SERIAL_AVAILABLE_LOGGED = False


# ── 直线电机单点下行打包（与 main_app_v3 pack_lm_* 完全一致） ──

def pack_lm_pos_ref(fdcan_bus: int, motor_id: int, pos_mm: float) -> bytes:
    """8B：bus / motor_id / pad / pos_mm(f32) → FUN_LM_POS_REF (0x10)"""
    return struct.pack("<BBxxf", fdcan_bus & 0xFF, motor_id & 0xFF, float(pos_mm))


def pack_linear_pos_batch(seq: int, positions: Sequence[float]) -> bytes:
    """64B → FUN_LM_POS_BATCH (0x18)：seq + 15×f32 + 3 pad。"""
    pos = [float(v) for v in positions]
    while len(pos) < 15:
        pos.append(0.0)
    return struct.pack("<B15f3x", seq & 0xFF, *pos[:15])


def pack_rotary_pos_batch(
    seq: int,
    positions: Sequence[float],
    hand_type: Optional[int] = None,
) -> bytes:
    """33B → FUN_RM_POS_BATCH (0x19)：seq + 7×f32 + 4 pad。"""
    pos = apply_rotary_downlink_sign(positions, hand_type)
    while len(pos) < ROTARY_MOTOR_NUM:
        pos.append(0.0)
    return struct.pack("<B7f4x", seq & 0xFF, *pos[:ROTARY_MOTOR_NUM])


def pack_lm_motor_pwr(fdcan_bus: int, motor_id: int, cmd: int) -> bytes:
    """4B：bus / motor_id / cmd / 0x00 → FUN_LM_MOTOR_PWR (0x11)"""
    return struct.pack("<BBBB", fdcan_bus & 0xFF, motor_id & 0xFF, cmd & 0xFF, 0)


def pack_lm_mech_homing(fdcan_bus: int, motor_id: int, cmd: int,
                         iq_seek: float = DEFAULT_HOMING_IQ_SEEK) -> bytes:
    """8B：bus / motor_id / cmd / 0x00 / iq_seek(f32) → FUN_LM_MECH_HOMING (0x12)
    cmd: LM_HOMING_START(1)=开始寻零  LM_HOMING_RESET(2)=复位回零状态"""
    return struct.pack("<BBBBf", fdcan_bus & 0xFF, motor_id & 0xFF,
                       cmd & 0xFF, 0, float(iq_seek))


def pack_rm_zero_sp(spi_bus: int, zero0: int, zero1: int) -> bytes:
    """4B：spi_bus / zero0 / zero1 / 0 → FUN_RM_ZERO_SP (0x25)，与 main_app_v3 一致。"""
    return struct.pack("<BBBB", spi_bus & 0xFF, zero0 & 0xFF, zero1 & 0xFF, 0)


def pack_rm_pos_set_sp(
    spi_bus: int,
    pos0_deg: float,
    pos1_deg: float,
    motor_mask: int = 0,
    hand_type: Optional[int] = None,
) -> bytes:
    """12B → FUN_RM_POS_SET_SP (0x21)；motor_mask bit0=M0 bit1=M1。"""
    p0, p1 = apply_rotary_bus_pos_sign(spi_bus, pos0_deg, pos1_deg, hand_type)
    return struct.pack("<BB2x2f", spi_bus & 0xFF, motor_mask & 0xFF, p0, p1)


def pack_rm_drive_sp(spi_bus: int, en0: int, en1: int) -> bytes:
    """4B → FUN_RM_DRIVE_SP (0x22)。"""
    return struct.pack("<BBBB", spi_bus & 0xFF, en0 & 0xFF, en1 & 0xFF, 0)


def pack_lm_gravity_enable(fdcan_bus: int, motor_id: int, cmd: int) -> bytes:
    """4B → FUN_LM_GRAVITY_ENABLE (0x1A) / CAN 0x208。"""
    return struct.pack("<BBBB", fdcan_bus & 0xFF, motor_id & 0xFF, cmd & 0xFF, 0)


def pack_rm_gravity_enable(cmd: int) -> bytes:
    """4B → FUN_RM_GRAVITY_ENABLE (0x1B)；1=允许自动开启，2=强制关闭。"""
    return struct.pack("<B3x", cmd & 0xFF)


def pack_gravity_allow_no_imu(cmd: int) -> bytes:
    """4B → FUN_GRAVITY_ALLOW_NO_IMU (0x1C)；1=无 IMU 固定姿态，2=必须 IMU。"""
    return struct.pack("<B3x", cmd & 0xFF)


def pack_gravity_master_enable(cmd: int) -> bytes:
    """4B → FUN_GRAVITY_MASTER_ENABLE (0x1D)；1=允许，2=强制关闭。"""
    return struct.pack("<B3x", cmd & 0xFF)


def pack_lm_pid_gain(
    fdcan_bus: int, motor_id: int, id_kp: float, id_ki: float,
    iq_kp: float, iq_ki: float, pos_kp: float, pos_ki: float,
) -> bytes:
    return struct.pack(
        "<BBxx6f", fdcan_bus & 0xFF, motor_id & 0xFF,
        id_kp, id_ki, iq_kp, iq_ki, pos_kp, pos_ki,
    )


def pack_lm_sine_motion(
    fdcan_bus: int, motor_id: int, cmd: int,
    low_mm: float, high_mm: float, period_s: float,
) -> bytes:
    return struct.pack(
        "<BBBB3f", fdcan_bus & 0xFF, motor_id & 0xFF, cmd & 0xFF, 0,
        low_mm, high_mm, period_s,
    )


def pack_lm_pwm_ref(
    fdcan_bus: int, motor_id: int, cmd: int,
    pa0_cc: int, pa1_cc: int, pa25_cc: int, pa22_cc: int,
) -> bytes:
    return struct.pack(
        "<BBBB4H", fdcan_bus & 0xFF, motor_id & 0xFF, cmd & 0xFF, 0,
        pa0_cc & 0xFFFF, pa1_cc & 0xFFFF, pa25_cc & 0xFFFF, pa22_cc & 0xFFFF,
    )


def pack_lm_vq_ref(fdcan_bus: int, motor_id: int, cmd: int, vq_ref: float) -> bytes:
    return struct.pack(
        "<BBBBf", fdcan_bus & 0xFF, motor_id & 0xFF, cmd & 0xFF, 0, vq_ref,
    )


def pack_lm_boot_req(fdcan_bus: int, magic: int = BOOT_REQ_MAGIC) -> bytes:
    return struct.pack("<B3xI", fdcan_bus & 0xFF, magic & 0xFFFFFFFF)


def pack_rm_pid_sp(
    spi_bus: int, kp0: float, ki0: float, kd0: float,
    kp1: float, ki1: float, kd1: float,
) -> bytes:
    return struct.pack("<B3x6f", spi_bus & 0xFF, kp0, ki0, kd0, kp1, ki1, kd1)


def pack_rm_cur_pid_sp(
    spi_bus: int, motor_mask: int,
    id_kp0: float, id_ki0: float, id_kd0: float,
    iq_kp0: float, iq_ki0: float, iq_kd0: float,
    id_kp1: float, id_ki1: float, id_kd1: float,
    iq_kp1: float, iq_ki1: float, iq_kd1: float,
) -> bytes:
    """52B；线序与 G431 一致，id_kd 位于每路六参数末尾。"""
    return struct.pack(
        "<BB2x12f", spi_bus & 0xFF, motor_mask & 0xFF,
        id_kp0, id_ki0, iq_kp0, iq_ki0, iq_kd0, id_kd0,
        id_kp1, id_ki1, iq_kp1, iq_ki1, iq_kd1, id_kd1,
    )


def pack_rm_sine_sp(
    spi_bus: int, max0: float, min0: float, freq0: float,
    max1: float, min1: float, freq1: float,
    hand_type: Optional[int] = None,
) -> bytes:
    max0, max1 = apply_rotary_bus_pos_sign(spi_bus, max0, max1, hand_type)
    min0, min1 = apply_rotary_bus_pos_sign(spi_bus, min0, min1, hand_type)
    return struct.pack("<B3x6f", spi_bus & 0xFF, max0, min0, freq0, max1, min1, freq1)


def pack_rm_pwm_sp(
    spi_bus: int, ccr0_a: int, ccr0_b: int, ccr0_c: int,
    ccr1_a: int, ccr1_b: int, ccr1_c: int,
) -> bytes:
    return struct.pack(
        "<B3x3H3H", spi_bus & 0xFF,
        ccr0_a & 0xFFFF, ccr0_b & 0xFFFF, ccr0_c & 0xFFFF,
        ccr1_a & 0xFFFF, ccr1_b & 0xFFFF, ccr1_c & 0xFFFF,
    )


def pack_rm_vq_sp(spi_bus: int, vq0: float, vq1: float) -> bytes:
    return struct.pack("<B3x2f", spi_bus & 0xFF, vq0, vq1)


def pack_rm_cur_loop_sp(
    spi_bus: int, flags: int, motor_mask: int, iq_ref0: float, iq_ref1: float,
) -> bytes:
    return struct.pack(
        "<BBBx2f", spi_bus & 0xFF, flags & 0xFF, motor_mask & 0xFF,
        iq_ref0, iq_ref1,
    )


def pack_rm_open_spin_sp(
    spi_bus: int, flags: int, motor_mask: int,
    uq0: float, uq1: float, freq_hz0: float, freq_hz1: float,
) -> bytes:
    return struct.pack(
        "<BBBx4f", spi_bus & 0xFF, flags & 0xFF, motor_mask & 0xFF,
        uq0, uq1, freq_hz0, freq_hz1,
    )


def pack_rm_elec_calib_sp(spi_bus: int, flags: int, motor_mask: int) -> bytes:
    """4B → FUN_RM_ELEC_CALIB_SP (0x2C)：离线电角度标定。"""
    return struct.pack(
        "<BBBx",
        spi_bus & 0xFF,
        flags & 0xFF,
        motor_mask & 0xFF,
    )


def pack_rm_elec_offset_sp(
    spi_bus: int,
    flags: int,
    motor_mask: int,
    offset0_deg: float,
    offset1_deg: float,
) -> bytes:
    """12B → FUN_RM_ELEC_OFFSET_SP (0x2D)：写机械域电角度偏置。"""
    return struct.pack(
        "<BBBx2f",
        spi_bus & 0xFF,
        flags & 0xFF,
        motor_mask & 0xFF,
        float(offset0_deg),
        float(offset1_deg),
    )


def pack_rm_hand_type_set(hand_type: int, cmd: int = 1) -> bytes:
    """4B → FUN_RM_HAND_TYPE_SET_SP (0x2B)，0=左手，1=右手。"""
    return struct.pack("<BB2x", cmd & 0xFF, hand_type & 0xFF)


def _log(msg: str):
    if tag_enabled("UsbController"):
        log_emit(msg, "UsbController")


def _format_joint_angles_for_log(joint_angles: Dict[str, float]) -> str:
    """格式化关节角日志（rad）。"""
    parts: List[str] = []
    for name in JOINT_ORDER:
        if name in joint_angles:
            short = name.replace("left_hand_", "").replace("_joint", "")
            parts.append(f"{short}={float(joint_angles[name]):+.4f}")
    extra = sorted(k for k in joint_angles if k not in JOINT_ORDER)
    for name in extra:
        short = name.replace("left_hand_", "").replace("_joint", "")
        parts.append(f"{short}={float(joint_angles[name]):+.4f}")
    return " | ".join(parts)


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


def pack_hand_pos_batch(
    seq: int,
    linear: Sequence[float],
    rotary: Sequence[float],
    hand_type: Optional[int] = None,
) -> bytes:
    """打包整手批量 payload（88 B），布局见文件顶部文档「一、USB 下行帧」。"""
    lin = [float(v) for v in linear]
    rot = apply_rotary_downlink_sign(rotary, hand_type)
    while len(lin) < LINEAR_MOTOR_NUM:
        lin.append(0.0)
    while len(rot) < ROTARY_MOTOR_NUM:
        rot.append(0.0)
    return struct.pack("<B14f7f3x", seq & 0xFF, *lin[:LINEAR_MOTOR_NUM], *rot[:ROTARY_MOTOR_NUM])


# joint_angles_to_hand_positions 结果缓存（高频下发时角度常几乎不变）
_JA_POS_CACHE_LOCK = threading.Lock()
_JA_POS_CACHE_KEY: Optional[Tuple[float, ...]] = None
_JA_POS_CACHE_LINEAR: Optional[List[float]] = None
_JA_POS_CACHE_ROTARY: Optional[List[float]] = None
_JA_POS_CACHE_EPS = float(os.environ.get("HAND_LINKAGE_CACHE_EPS", "1e-5"))


def _joint_angles_cache_key(joint_angles: Dict[str, float]) -> Tuple[float, ...]:
    eps = max(_JA_POS_CACHE_EPS, 1e-9)
    inv = 1.0 / eps
    out: List[float] = []
    for name in JOINT_ORDER:
        try:
            v = float(joint_angles.get(name, 0.0))
        except (ValueError, TypeError, OverflowError) as exc:
            raise ValueError(f"Invalid V2 angle: {name}") from exc
        if not math.isfinite(v):
            # Nonfinite keys must never collide with any finite input angle.
            out.append(math.nan if math.isnan(v) else v)
            continue
        scaled = v * inv
        out.append(round(scaled) * eps if math.isfinite(scaled) else v)
    return tuple(out)


def clear_joint_angles_pos_cache() -> None:
    """行程映射参数变化后清空缓存（如 stroke_scale）。"""
    global _JA_POS_CACHE_KEY, _JA_POS_CACHE_LINEAR, _JA_POS_CACHE_ROTARY
    with _JA_POS_CACHE_LOCK:
        _JA_POS_CACHE_KEY = None
        _JA_POS_CACHE_LINEAR = None
        _JA_POS_CACHE_ROTARY = None


def joint_angles_to_hand_positions(
    joint_angles: Dict[str, float],
) -> Tuple[List[float], List[float]]:
    """URDF 关节角（rad）→ 整手批量下行数组（14×mm + 7×deg）。

    四指 MCP/PIP/DIP 使用 finger_linkage 闭环逆解；拇指走 ThumbStrokeMapper。
    闭环覆盖的关节若逆解失败或暂时缺失，保持上次有效行程（或 0），
    **绝不**回退线性 rad→mm 系数，避免与闭环结果之间的突变。

    高频路径下对几乎相同的角度复用上次结果，避免重复连杆逆解。
    """
    global _JA_POS_CACHE_KEY, _JA_POS_CACHE_LINEAR, _JA_POS_CACHE_ROTARY
    if not isinstance(joint_angles, Mapping):
        raise ValueError("V2 pose must be a joint-name mapping")
    key = _joint_angles_cache_key(joint_angles)
    with _JA_POS_CACHE_LOCK:
        if (
            key == _JA_POS_CACHE_KEY
            and _JA_POS_CACHE_LINEAR is not None
            and _JA_POS_CACHE_ROTARY is not None
        ):
            return list(_JA_POS_CACHE_LINEAR), list(_JA_POS_CACHE_ROTARY)

    linear = [0.0] * LINEAR_MOTOR_NUM
    rotary = [0.0] * ROTARY_MOTOR_NUM
    get = joint_angles.get
    for slot, name in _ROTARY_JOINTS_BY_SLOT:
        degrees = float(get(name, 0.0)) * _RAD_TO_DEG
        if not math.isfinite(degrees) or abs(degrees) > 3.4028234663852886e38:
            raise ValueError(f"V2 rotary target is not a finite float32: {name}")
        rotary[slot] = degrees
    mapper = _ensure_hand_stroke_mapper()
    linkage_strokes = mapper.linear_strokes_mm(joint_angles)
    managed = set(mapper.managed_linear_joints())
    for slot, name in _LINEAR_JOINTS_BY_SLOT:
        if name in linkage_strokes:
            mm = linkage_strokes[name]
        elif name in managed:
            held = mapper.last_stroke_mm(name)
            mm = 0.0 if held is None else held
        else:
            # 非闭环管理关节（当前默认配置下不应出现）保持 0
            mm = 0.0
        linear[slot] = float(mm) if math.isfinite(mm) else 0.0
        if abs(linear[slot]) > 3.4028234663852886e38:
            raise ValueError(f"V2 linear target is not representable as float32: {name}")
    with _JA_POS_CACHE_LOCK:
        _JA_POS_CACHE_KEY = key
        _JA_POS_CACHE_LINEAR = list(linear)
        _JA_POS_CACHE_ROTARY = list(rotary)
    return linear, rotary


_LINEAR_SLOT_JOINT = {slot: name for name, slot in LINEAR_JOINT_SLOTS.items()}
_ROTARY_SLOT_JOINT = {slot: name for name, slot in ROTARY_JOINT_SLOTS.items()}
_FINGER_CN = {
    "thumb": "拇指",
    "index": "食指",
    "mid": "中指",
    "ring": "无名指",
    "pinky": "小指",
}


def joint_cfg_from_telemetry_frame(
    frame: dict,
    *,
    use_ref: bool = False,
    hand_side: Optional[str] = "left",
    strict: bool = False,
) -> Dict[str, float]:
    """将遥测帧反解为 URDF 关节角（rad）；use_ref=True 时用 pos_ref。"""
    linear = frame.get("linear")
    rotary = frame.get("rotary")
    if strict:
        if not isinstance(linear, list) or len(linear) != 14 or not isinstance(rotary, list) or len(rotary) != 7:
            raise ValueError("遥测需要完整的 14 路直线和 7 路旋转位置")
        values = [getattr(s, "pos_ref_mm" if use_ref else "pos_act_mm", None) for s in linear]
        values += [getattr(s, "pos_ref_mm" if use_ref else "pos_actual_mm", None) for s in rotary]
        if any(not isinstance(v, (int, float)) or not math.isfinite(v) for v in values):
            raise ValueError("遥测位置包含无效数值")
    if not isinstance(linear, list) or not isinstance(rotary, list):
        return {name: 0.0 for name in JOINT_ORDER}

    cfg: Dict[str, float] = {name: 0.0 for name in JOINT_ORDER}
    mm_by_joint: Dict[str, float] = {}
    for slot, joint in _LINEAR_SLOT_JOINT.items():
        if slot >= len(linear):
            continue
        slot_obj = linear[slot]
        if use_ref:
            mm = getattr(slot_obj, "pos_ref_mm", None)
        else:
            mm = getattr(slot_obj, "pos_act_mm", None)
        if isinstance(mm, (int, float)) and math.isfinite(float(mm)):
            mm_by_joint[joint] = float(mm)

    mapper = _ensure_hand_stroke_mapper()
    for finger in ("index", "mid", "ring", "pinky"):
        j_mcp = f"left_hand_{finger}_MCP_joint"
        j_pip = f"left_hand_{finger}_PIP_joint"
        j_dip = f"left_hand_{finger}_DIP_joint"
        if j_mcp in mm_by_joint and j_pip in mm_by_joint and j_dip in mm_by_joint:
            cf1 = -float(mm_by_joint[j_dip])
            cf2 = -float(mm_by_joint[j_pip])
            cf3 = -float(mm_by_joint[j_mcp])
            try:
                mcp, pip, dip = mapper.four_finger.cf_mm_to_joint_rad(cf1, cf2, cf3)
                cfg[j_mcp] = float(mcp)
                cfg[j_pip] = float(pip)
                cfg[j_dip] = float(dip)
            except Exception as exc:
                if strict:
                    raise ValueError(f"{finger} 遥测位置无法反解") from exc
                finger_cn = _FINGER_CN.get(finger, finger)
                _log_kin_failure(
                    f"telem_{finger}",
                    f"遥测正运动学失败({finger_cn}): {exc}",
                    direction="fk",
                    cf1_mm=cf1,
                    cf2_mm=cf2,
                    cf3_mm=cf3,
                    use_ref=use_ref,
                )

    thumb_mcp_mm = mm_by_joint.get("left_hand_thumb_MCP_joint")
    thumb_pip_mm = mm_by_joint.get("left_hand_thumb_PIP_joint")
    if thumb_mcp_mm is not None:
        try:
            cf3 = -float(thumb_mcp_mm)
            phi = cf_to_phi_deg(cf3, MOTOR_F3, mapper.thumb.params, clamp=True)
            cfg["left_hand_thumb_MCP_joint"] = joint_deg_to_phi_rad(phi)
        except Exception as exc:
            if strict:
                raise ValueError("拇指 MCP 遥测位置无法反解") from exc
            _log_kin_failure(
                "telem_thumb_mcp",
                f"遥测拇指MCP正运动学失败: {exc}",
                direction="fk",
                cf3_mm=-float(thumb_mcp_mm),
                use_ref=use_ref,
            )
    if thumb_pip_mm is not None:
        try:
            cf1 = -float(thumb_pip_mm)
            phi = cf_to_phi_deg(cf1, MOTOR_F1, mapper.thumb.params, clamp=True)
            cfg["left_hand_thumb_PIP_joint"] = joint_deg_to_phi_rad(phi)
        except Exception as exc:
            if strict:
                raise ValueError("拇指 PIP 遥测位置无法反解") from exc
            _log_kin_failure(
                "telem_thumb_pip",
                f"遥测拇指PIP正运动学失败: {exc}",
                direction="fk",
                cf1_mm=-float(thumb_pip_mm),
                use_ref=use_ref,
            )

    for slot, joint in _ROTARY_SLOT_JOINT.items():
        if slot >= len(rotary):
            continue
        slot_obj = rotary[slot]
        if use_ref:
            deg = getattr(slot_obj, "pos_ref_mm", None)
        else:
            deg = getattr(slot_obj, "pos_actual_mm", None)
        if not isinstance(deg, (int, float)) or not math.isfinite(float(deg)):
            continue
        rad_hw = float(deg) / _RAD_TO_DEG
        if joint in _FINGER_SWING_JOINTS:
            # 遥测为硬件空间；对外返回 UI/URDF 空间
            cfg[joint] = swing_hw_to_urdf(rad_hw, hand_side)
        elif joint == _THUMB_SWING_JOINT:
            cfg[joint] = thumb_swing_hw_to_ui(rad_hw, hand_side)
        elif joint == _THUMB_ROTA_JOINT:
            cfg[joint] = _thumb_rota_hw_to_ui(rad_hw, hand_side)
        elif joint == _PINKY_ROTA_JOINT:
            cfg[joint] = _pinky_rota_hw_to_ui(rad_hw, hand_side)
        else:
            cfg[joint] = rad_hw
    return cfg

# 直线/旋转关节按槽位排序（避免热路径里再做 .items() 迭代）
_LINEAR_JOINTS_BY_SLOT: List[Tuple[int, str]] = sorted(
    [(slot, name) for name, slot in LINEAR_JOINT_SLOTS.items()],
    key=lambda x: x[0],
)
_ROTARY_JOINTS_BY_SLOT: List[Tuple[int, str]] = sorted(
    [(slot, name) for name, slot in ROTARY_JOINT_SLOTS.items()],
    key=lambda x: x[0],
)
_RAD_TO_DEG = 180.0 / math.pi

_HAND_STROKE_MAPPER: Optional[HandStrokeMapper] = None


def _ensure_hand_stroke_mapper() -> HandStrokeMapper:
    global _HAND_STROKE_MAPPER
    if _HAND_STROKE_MAPPER is None:
        _HAND_STROKE_MAPPER = create_default_hand_stroke_mapper()
    return _HAND_STROKE_MAPPER


def get_hand_stroke_mapper() -> HandStrokeMapper:
    """返回当前整手行程映射器（四指闭环 + 拇指独立）。"""
    return _ensure_hand_stroke_mapper()


def configure_hand_stroke_mapper(mapper: HandStrokeMapper) -> None:
    """注入自定义 HandStrokeMapper（例如替换拇指标定或单指几何参数）。"""
    global _HAND_STROKE_MAPPER
    mapper.ensure_stroke_scales()
    _HAND_STROKE_MAPPER = mapper
    clear_joint_angles_pos_cache()


def configure_finger_linkage_params(params: FingerLinkageParams) -> None:
    """更新四指/拇指共用闭环几何参数（MATLAB defaultParameters 对应字段）。"""
    finalized = finalize_linkage_params(params)
    mapper = _ensure_hand_stroke_mapper()
    mapper.four_finger.params = finalized
    mapper.thumb.params = finalized
    clear_joint_angles_pos_cache()
    try:
        from kinematics.finger_linkage import clear_phi_cf_lut_cache

        clear_phi_cf_lut_cache()
    except Exception:
        pass


def configure_thumb_stroke_mapper(**kwargs) -> None:
    """配置拇指行程映射（默认 F3+MCP / F1+PIP 闭环；use_linkage=False 为线性）。"""
    mapper = _ensure_hand_stroke_mapper()
    mapper.thumb.configure(**kwargs)
    mapper.ensure_stroke_scales()
    clear_joint_angles_pos_cache()


def get_stroke_scales() -> Dict[str, float]:
    """读取各直线关节角度→行程缩放系数（默认 1.0）。"""
    return _ensure_hand_stroke_mapper().get_stroke_scales()


def set_stroke_scale(joint_name: str, scale: float) -> None:
    """设置单个关节的行程缩放系数。"""
    _ensure_hand_stroke_mapper().set_stroke_scale(joint_name, scale)
    clear_joint_angles_pos_cache()


def configure_stroke_scales(scales: Dict[str, float]) -> None:
    """批量设置行程缩放系数（可只传部分关节）。"""
    _ensure_hand_stroke_mapper().set_stroke_scales(scales)
    clear_joint_angles_pos_cache()


def _short_joint_name(name: str) -> str:
    name = name.replace("left_hand_", "")
    if name.endswith("_joint"):
        name = name[: -len("_joint")]
    return name


def format_hand_downlink_log(
    source: str,
    seq: int,
    linear: Sequence[float],
    rotary: Sequence[float],
    joint_angles: Optional[Dict[str, float]] = None,
    frame: Optional[bytes] = None,
    *,
    connected: bool = False,
    sent: bool = False,
) -> str:
    """格式化整手批量下发日志（含来源窗体与当前线程）。"""
    thread = threading.current_thread()
    if sent:
        serial_status = "已连接并已发送"
    elif connected:
        serial_status = "已连接但发送失败"
    else:
        serial_status = "未连接（仅预览）"
    lines = [
        "──────── 下位机下发 ────────",
        f"来源窗体: {source}",
        f"发送线程: {thread.name} (id={thread.ident})",
        f"串口状态: {serial_status}",
        f"协议: DEV=0x{DEV_TELEM:02X} FUN=0x{FUN_HAND_POS_BATCH:02X} seq={seq}",
        f"直线电机 mm [{LINEAR_MOTOR_NUM}]:",
    ]
    for i in range(LINEAR_MOTOR_NUM):
        joint = _short_joint_name(_LINEAR_SLOT_JOINT.get(i, f"linear_{i}"))
        val = float(linear[i]) if i < len(linear) else 0.0
        lines.append(f"  L{i:02d} {joint:<22} = {val:8.3f}")
    lines.append(f"旋转电机 deg [{ROTARY_MOTOR_NUM}]:")
    for i in range(ROTARY_MOTOR_NUM):
        joint = _short_joint_name(_ROTARY_SLOT_JOINT.get(i, f"rotary_{i}"))
        val = float(rotary[i]) if i < len(rotary) else 0.0
        lines.append(f"  R{i:02d} {joint:<22} = {val:8.3f}")
    if joint_angles:
        lines.append(f"关节角 rad [{len(joint_angles)}]:")
        for name in JOINT_ORDER:
            if name in joint_angles:
                lines.append(f"  {_short_joint_name(name):<22} = {float(joint_angles[name]):8.9f}")
        extra = sorted(k for k in joint_angles if k not in JOINT_ORDER)
        for name in extra:
            lines.append(f"  {_short_joint_name(name):<22} = {float(joint_angles[name]):8.9f}")
    if frame is not None:
        all_bytes = frame.hex(" ").upper().split(" ")   # ['AA', '55', ...]
        n = len(all_bytes)
        chunk = (n + 2) // 3                            # 每行字节数（三等分）
        row1 = " ".join(all_bytes[0        : chunk    ])
        row2 = " ".join(all_bytes[chunk    : chunk * 2])
        row3 = " ".join(all_bytes[chunk * 2:           ])
        lines.append(f"帧长度: {n} B")
        lines.append(f"帧(hex):")
        lines.append(f"  {row1}")
        lines.append(f"  {row2}")
        lines.append(f"  {row3}")
    lines.append("────────────────────────────")
    return "\n".join(lines)


_DOWNLINK_FUN_LABELS = {
    FUN_LM_POS_REF: "直线单点位置",
    FUN_LM_MOTOR_PWR: "直线使能",
    FUN_LM_MECH_HOMING: "直线机械回零",
    FUN_LM_PID_GAIN: "直线 PID",
    FUN_LM_SINE_MOTION: "直线正弦",
    FUN_LM_PWM_REF: "直线 PWM",
    FUN_LM_VQ_REF: "直线 Vq",
    FUN_LM_BOOT_REQ: "Boot 升级",
    FUN_LM_POS_BATCH: "直线批量位置",
    FUN_RM_POS_BATCH: "旋转批量位置",
    FUN_LM_GRAVITY_ENABLE: "直线重力开关",
    FUN_RM_GRAVITY_ENABLE: "旋转重力开关",
    FUN_GRAVITY_ALLOW_NO_IMU: "无IMU重力开关",
    FUN_GRAVITY_MASTER_ENABLE: "重力补偿总开关",
    FUN_HAND_POS_BATCH: "整手批量",
    FUN_RM_POS_SET_SP: "旋转单点位置",
    FUN_RM_DRIVE_SP: "旋转使能",
    FUN_RM_PID_SP: "旋转位置 PID",
    FUN_RM_SINE_SP: "旋转正弦",
    FUN_RM_ZERO_SP: "旋转机械回零",
    FUN_RM_PWM_SP: "旋转 PWM",
    FUN_RM_VQ_SP: "旋转 Vq",
    FUN_RM_CUR_PID_SP: "旋转电流 PID",
    FUN_RM_CUR_LOOP_SP: "旋转电流环",
    FUN_RM_OPEN_SPIN_SP: "旋转开环",
    FUN_RM_HAND_TYPE_SET_SP: "手型配置",
    FUN_RM_ELEC_CALIB_SP: "旋转电角度标定",
    FUN_RM_ELEC_OFFSET_SP: "旋转电角度偏置",
}


def format_downlink_wire_hex(frame: bytes, *, sent: bool) -> str:
    """格式化单次下行帧完整十六进制（与 main_app_v3 下行监视一致）。"""
    dev = frame[4] if len(frame) > 4 else 0
    fun = frame[5] if len(frame) > 5 else 0
    fun_lab = _DOWNLINK_FUN_LABELS.get(fun, f"0x{fun:02X}")
    status = "已发送" if sent else "发送失败/未连接"
    all_bytes = frame.hex(" ").upper().split(" ")
    n = len(all_bytes)
    chunk = max(1, (n + 2) // 3)
    rows = [
        " ".join(all_bytes[i: i + chunk])
        for i in range(0, n, chunk)
    ]
    lines = [
        "──────── 下行帧 HEX ────────",
        f"状态: {status}",
        f"协议: DEV=0x{dev:02X} FUN=0x{fun:02X} ({fun_lab})  长度={n} B",
        "帧(hex):",
    ]
    lines.extend(f"  {row}" for row in rows)
    lines.append("────────────────────────────")
    return "\n".join(lines)


def _log_downlink(
    source: str,
    seq: int,
    linear: Sequence[float],
    rotary: Sequence[float],
    *,
    joint_angles: Optional[Dict[str, float]] = None,
    frame: Optional[bytes] = None,
    connected: bool = False,
    sent: bool = False,
):
    body = format_hand_downlink_log(
        source, seq, linear, rotary, joint_angles, frame,
        connected=connected, sent=sent,
    )
    if tag_enabled("UsbController"):
        for line in body.splitlines():
            log_emit(line, "UsbController")


def format_port_entry(port_info) -> str:
    if port_info.vid == STM_VID and port_info.pid == STM_USB_CDC_PID:
        role = "H7 USB CDC（推荐）"
    elif port_info.vid == STM_VID and port_info.pid == STM_STLINK_VCP_PID:
        role = "ST-Link 调试口"
    else:
        role = (port_info.description or "未知")[:40]
    return f"{port_info.device} — {role}"


def device_from_port_entry(entry: str) -> str:
    return entry.split(" — ", 1)[0].strip()


class UsbDeviceInfo:
    """串口设备描述。"""

    def __init__(self, port: str, description: str = "", vid: int = 0, pid: int = 0):
        self.port = port
        self.description = description
        self.vid = vid
        self.pid = pid
        # 兼容旧 HID 字段
        self.path = port.encode("utf-8", errors="ignore")

    @property
    def display_name(self) -> str:
        if self.description:
            return f"{self.port} — {self.description}"
        return self.port

    def __repr__(self):
        return f"<UsbDeviceInfo {self.display_name}>"


class UsbController:
    """H7 USB CDC 下位机控制器。"""

    def __init__(self, *, recorder=None):
        # Desktop retains the global recorder; web sessions own isolated logs.
        self._session_recorder = _control_session_recorder if recorder is None else recorder
        self._ser: Optional[serial.Serial] = None
        self._connected = False
        self._current_info: Optional[UsbDeviceInfo] = None
        self._write_lock = threading.Lock()
        self._hand_frame_builder_lock = threading.Lock()
        self._hand_frame_builder: Optional[DownlinkFrameBuilder] = None
        self._downlink_seq = 0
        self.downlink_log_enabled = False
        self.send_hex_log_enabled = False
        self._rm_drive_cache: Dict[int, Dict[str, int]] = {}
        self._hand_type_cache: Optional[int] = None
        self._rx_stop = threading.Event()
        self._rx_thread: Optional[threading.Thread] = None
        self._uplink_buf = bytearray()
        self._telem_lock = threading.Lock()
        self._telem_payload: Optional[bytes] = None
        self._telem_recv_mono: float = 0.0
        self._telem_generation = 0
        self._telem_parsed_generation = -1
        self._telem_reported_generation = -1
        self._telem_frame_cache: Optional[dict] = None
        self._last_telem_seq = -1
        self._telem_duplicate_or_stale = 0
        self._heartbeat_cache: Optional[dict] = None
        self._telem_rx_count = 0
        self._telem_rx_rate_t0 = time.perf_counter()
        self._telem_rx_rate_hz = 0.0
        self._conn_listeners: List[Callable[[bool], None]] = []
        self._conn_listeners_lock = threading.Lock()
        self._port_lost = threading.Event()
        # 连续失败计数：吸收蓝牙 SPP 等虚拟串口的瞬时抖动，避免误判掉线
        self._tx_fail_streak = 0
        self._rx_fail_streak = 0
        self._data_reporter = None

    @property
    def should_process_downlink(self) -> bool:
        """是否应处理下发（需已连接串口）。"""
        return self.is_connected

    def set_data_reporter(self, reporter) -> None:
        """注册可选数据报告器（需提供 record_command/record_telemetry）。"""
        self._data_reporter = reporter

    @property
    def is_connected(self) -> bool:
        """真实连通：标志位 + 句柄 + pyserial.is_open。

        拔线/对端关闭后 is_open 会变 False，但旧逻辑只看 ``_connected``，
        导致 UI 仍显示「已连接」。
        """
        if not self._connected:
            return False
        ser = self._ser
        if ser is None:
            return False
        try:
            return bool(ser.is_open)
        except Exception:
            return False

    def add_connection_listener(self, callback: Callable[[bool], None]) -> None:
        """注册连接状态回调 ``callback(connected: bool)``（可能在非 UI 线程触发）。"""
        with self._conn_listeners_lock:
            if callback not in self._conn_listeners:
                self._conn_listeners.append(callback)

    def remove_connection_listener(self, callback: Callable[[bool], None]) -> None:
        with self._conn_listeners_lock:
            try:
                self._conn_listeners.remove(callback)
            except ValueError:
                pass

    def _notify_connection_changed(self, connected: bool) -> None:
        with self._conn_listeners_lock:
            listeners = list(self._conn_listeners)
        for cb in listeners:
            try:
                cb(bool(connected))
            except Exception as e:
                _log(f"connection listener 异常: {e}")

    def reconcile_connection(self) -> bool:
        """若标志仍为已连接但端口已死，执行断开并通知 UI。返回当前是否连通。"""
        if self._port_lost.is_set() and (self._connected or self._ser is not None):
            _log("reconcile_connection: RX 标记端口丢失，同步断开")
            self.disconnect()
            return False
        if self.is_connected:
            return True
        if self._connected or self._ser is not None:
            _log("reconcile_connection: 端口已失效，同步断开")
            self.disconnect()
        return False

    @property
    def current_device(self) -> Optional[UsbDeviceInfo]:
        return self._current_info

    @property
    def serial_available(self) -> bool:
        return _SERIAL_AVAILABLE

    def scan_devices(self) -> List[UsbDeviceInfo]:
        global _SERIAL_AVAILABLE_LOGGED
        if not _SERIAL_AVAILABLE or list_ports is None:
            if not _SERIAL_AVAILABLE_LOGGED:
                _log("pyserial 未安装，串口不可用")
                _SERIAL_AVAILABLE_LOGGED = True
            return []
        try:
            ports = list(list_ports.comports())
            result: List[UsbDeviceInfo] = []
            for p in ports:
                desc = format_port_entry(p).split(" — ", 1)[-1]
                result.append(UsbDeviceInfo(p.device, desc, p.vid or 0, p.pid or 0))
            # 优先 H7 CDC
            result.sort(key=lambda d: 0 if d.pid == STM_USB_CDC_PID else 1)
            _log(f"scan_devices: 找到 {len(result)} 个串口")
            return result
        except Exception as e:
            _log(f"scan_devices 失败: {e}")
            return []

    def connect(self, device_info: UsbDeviceInfo) -> bool:
        if not _SERIAL_AVAILABLE or serial is None:
            _log("connect: pyserial 不可用")
            return False
        if self._connected or self._ser is not None:
            self.disconnect()
        port = device_info.port
        last_exc: Optional[Exception] = None
        for attempt in range(1, SERIAL_CONNECT_RETRIES + 1):
            try:
                ser = getattr(self, "serial_factory", serial.Serial)(
                    port,
                    baudrate=SERIAL_BAUDRATE,
                    timeout=SERIAL_READ_TIMEOUT_S,
                    write_timeout=SERIAL_WRITE_TIMEOUT_S,
                )
                ser.dtr = True
                ser.rts = False
                ser.reset_input_buffer()
                ser.reset_output_buffer()
                self._port_lost.clear()
                self._tx_fail_streak = 0
                self._rx_fail_streak = 0
                self._ser = ser
                self._connected = True
                self._current_info = device_info
                self._downlink_seq = 0
                with self._hand_frame_builder_lock:
                    self._hand_frame_builder = None
                self._hand_type_cache = None
                with self._telem_lock:
                    self._telem_payload = None
                    self._telem_recv_mono = 0.0
                    self._telem_generation = 0
                    self._telem_parsed_generation = -1
                    self._telem_reported_generation = -1
                    self._telem_frame_cache = None
                    self._last_telem_seq = -1
                    self._telem_duplicate_or_stale = 0
                    self._heartbeat_cache = None
                self._start_rx_drain()
                comm_diag_info(f"USB connect {device_info.display_name} baud={SERIAL_BAUDRATE} "
                               f"线速参考≈{theoretical_line_max_hz():.0f}Hz RX drain 已启动")
                _log(f"connect: 已连接 {device_info.display_name}"
                     + (f"（第 {attempt} 次尝试成功）" if attempt > 1 else ""))
                self._notify_connection_changed(True)
                return True
            except Exception as e:
                last_exc = e
                self._ser = None
                self._connected = False
                self._current_info = None
                if attempt < SERIAL_CONNECT_RETRIES:
                    # 蓝牙 SPP 刚配对/唤醒时打开端口常见瞬时失败，短暂等待后重试
                    _log(f"connect 第 {attempt} 次尝试失败，{SERIAL_CONNECT_RETRY_DELAY_S}s 后重试: {e}")
                    time.sleep(SERIAL_CONNECT_RETRY_DELAY_S)
        _log(f"connect 失败（已重试 {SERIAL_CONNECT_RETRIES} 次）: {last_exc}")
        return False

    def connect_port(self, port: str) -> bool:
        return self.connect(UsbDeviceInfo(port))

    def _rx_loop(self) -> None:
        """后台排空上行遥测，避免 CDC RX 积压导致 H7 发送阻塞与 CAN 时序劣化。"""
        _set_thread_priority("above_normal")
        idle_loops = 0
        while not self._rx_stop.is_set():
            ser = self._ser
            if ser is None or not getattr(ser, "is_open", False):
                # 不可在本线程 join 自己：只打标，由 UI/下发路径 reconcile
                if self._connected:
                    self._port_lost.set()
                time.sleep(0.01)
                continue
            try:
                t0 = time.perf_counter()
                chunk = _drain_serial_port(ser)
                drain_ms = (time.perf_counter() - t0) * 1000.0
                if not chunk:
                    idle_loops += 1
                    if idle_loops % 2000 == 0:
                        snapshot_usb_port(ser)
                    time.sleep(RX_IDLE_SLEEP_S)
                    continue
                idle_loops = 0
                self._process_uplink_chunk(chunk)
                in_wait = 0
                try:
                    in_wait = ser.in_waiting
                except Exception:
                    pass
                record_usb_rx(nbytes=len(chunk), in_waiting_after=in_wait, drain_ms=drain_ms)
                while not self._rx_stop.is_set():
                    # 与 main_app_v3.rx_loop 一致：OS 缓冲或解析缓存仍有
                    # 明显积压时连续读取，并且每一批都必须送入解析器。
                    # 旧实现只统计 extra 后就丢弃，恰好会漏掉找零状态边沿，
                    # 同时令 Form5 看起来跳帧、卡顿。
                    if (
                        ser.in_waiting <= 0
                        and len(self._uplink_buf) <= RX_PARSER_BACKLOG_BYTES
                    ):
                        break
                    extra = _drain_serial_port(ser)
                    if not extra:
                        break
                    self._process_uplink_chunk(extra)
                    record_usb_rx(
                        nbytes=len(extra),
                        in_waiting_after=ser.in_waiting,
                    )
            except serial.SerialException as exc:
                self._rx_fail_streak += 1
                streak = self._rx_fail_streak
                if streak < SERIAL_RX_FAIL_STREAK_LIMIT:
                    # 蓝牙 SPP 等虚拟串口偶发 WinError/超时很常见，物理连接通常
                    # 未真的断开：连续少数次只记日志重试，避免整段误判掉线。
                    comm_diag_warn(f"USB_RX drain 异常（第{streak}次，重试中）: {exc}")
                    time.sleep(SERIAL_RX_FAIL_RETRY_SLEEP_S)
                    continue
                comm_diag_warn(f"USB_RX drain 异常（连续{streak}次，判定掉线）: {exc}")
                _log(f"RX drain 异常（连续{streak}次）: {exc}")
                self._port_lost.set()
                break
            except Exception as exc:
                comm_diag_warn(f"USB_RX drain 未预期: {exc}")
                time.sleep(0.01)
                continue
            else:
                self._rx_fail_streak = 0

    def _process_uplink_chunk(self, chunk: bytes) -> None:
        """轻量解析上行帧，缓存最新遥测 payload（参照 main_app_v3 UplinkFrameParser）。"""
        if not chunk:
            return
        self._uplink_buf.extend(chunk)
        if len(self._uplink_buf) > 131072:
            del self._uplink_buf[:-65536]
        while len(self._uplink_buf) >= 8:
            if self._uplink_buf[0] != USB_UPLINK_HDR[0] or self._uplink_buf[1] != USB_UPLINK_HDR[1]:
                del self._uplink_buf[0]
                continue
            plen = int(self._uplink_buf[2] | (self._uplink_buf[3] << 8))
            total = plen + 8
            if plen <= 0 or total > _UPLINK_MAX_FRAME:
                del self._uplink_buf[0]
                continue
            if len(self._uplink_buf) < total:
                break
            frame = bytes(self._uplink_buf[:total])
            del self._uplink_buf[:total]
            body = frame[: plen + 6]
            crc_rx = int(frame[plen + 6] | (frame[plen + 7] << 8))
            if crc16_modbus(body) != crc_rx:
                continue
            dev = int(frame[4])
            fun = int(frame[5])
            if dev == DEV_TELEM and fun == FUN_HEARTBEAT and plen >= 16:
                payload = bytes(frame[6 : 6 + plen])
                magic, seq, tick_ms, hb_count = struct.unpack("<4I", payload[:16])
                if magic == HEARTBEAT_MAGIC:
                    with self._telem_lock:
                        self._heartbeat_cache = {
                            "seq": int(seq),
                            "tick_ms": int(tick_ms),
                            "hb_count": int(hb_count),
                            "recv_mono": time.perf_counter(),
                        }
                continue
            if dev == DEV_TELEM and fun == FUN_TELEM:
                # 仅缓存权威 780B（或旧固件 932B）遥测，避免脏长度污染解析
                if plen not in _TELEM_ACCEPT_PLEN:
                    continue
                payload = bytes(frame[6 : 6 + plen])
                seq = int(payload[0] | (payload[1] << 8))
                with self._telem_lock:
                    # 与 main_app_v3 一致：重复帧与旧帧不覆盖最新遥测。
                    # 允许 uint16 回绕和 USB 长时丢帧/重连后的大跳变。
                    if self._last_telem_seq >= 0:
                        delta = (seq - self._last_telem_seq) & 0xFFFF
                        if delta == 0 or (delta > 512 and delta < 0xF000):
                            self._telem_duplicate_or_stale += 1
                            if self._session_recorder.active:
                                self._session_recorder.record_telemetry_payload(payload, accepted=False)
                            continue
                    if self._session_recorder.active:
                        self._session_recorder.record_telemetry_payload(payload, accepted=True)
                    self._last_telem_seq = seq
                    self._telem_payload = payload
                    self._telem_recv_mono = time.perf_counter()
                    self._telem_generation += 1
                    self._telem_frame_cache = None
                    self._telem_rx_count += 1
                    now = time.perf_counter()
                    elapsed = now - self._telem_rx_rate_t0
                    if elapsed >= 1.0:
                        self._telem_rx_rate_hz = self._telem_rx_count / elapsed
                        self._telem_rx_count = 0
                        self._telem_rx_rate_t0 = now

    def _start_rx_drain(self) -> None:
        self._stop_rx_drain()
        self._rx_stop.clear()
        self._rx_thread = threading.Thread(
            target=self._rx_loop, name="UsbRxDrain", daemon=True,
        )
        self._rx_thread.start()

    def _stop_rx_drain(self) -> None:
        self._rx_stop.set()
        t = self._rx_thread
        if t is not None:
            t.join(timeout=1.0)
        self._rx_thread = None

    def disconnect(self) -> None:
        was_connected = bool(self._connected or self._ser is not None)
        self._port_lost.clear()
        # 从 RX 线程不能 join 自己：若当前就是 RX 线程，只关口并退出循环
        in_rx = (
            self._rx_thread is not None
            and threading.current_thread() is self._rx_thread
        )
        if not in_rx:
            self._stop_rx_drain()
        else:
            self._rx_stop.set()
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception as e:
                _log(f"disconnect: 关闭异常（已忽略）: {e}")
            finally:
                self._ser = None
        name = self._current_info.display_name if self._current_info else "未知设备"
        self._connected = False
        self._current_info = None
        self._hand_type_cache = None
        if was_connected:
            _log(f"disconnect: 已断开 → {name}")
            self._notify_connection_changed(False)

    def _rm_drive_cache_bus(self, spi_bus: int) -> Dict[str, int]:
        return self._rm_drive_cache.setdefault(spi_bus, {"en0": 0, "en1": 0})

    def _log_wire_hex(self, frame: bytes, *, sent: bool, stream: bool = False) -> None:
        """打印下发帧 HEX。Form2「显示下发HEX」开启时，流式（实时关节/动作同步）也打印。"""
        if not self.send_hex_log_enabled:
            return
        if not tag_enabled("UsbController"):
            return
        body = format_downlink_wire_hex(frame, sent=sent)
        if stream:
            log_emit("[stream] 高频下发帧", "UsbController")
        for line in body.splitlines():
            log_emit(line, "UsbController")

    def _write_frame(
        self, frame: bytes, flush: bool = True,
        *, stream: bool = False, ignore_backpressure: bool = False,
    ) -> bool:
        sent = False
        try:
            sent = self._write_frame_impl(frame, flush, stream=stream,
                                          ignore_backpressure=ignore_backpressure)
            return sent
        finally:
            self._session_recorder.record_usb_tx(frame, sent=sent)

    def _write_frame_impl(
        self, frame: bytes, flush: bool = True,
        *, stream: bool = False, ignore_backpressure: bool = False,
    ) -> bool:
        """写入一帧数据到串口。
        stream=True 时不等串口清空（flush=False），靠 OS TX 缓冲区异步发送；
        默认检查 out_waiting 背压，积压过多时跳帧，避免时序漂移（与 main_app_v3 一致）。
        ignore_backpressure=True 时完全跳过 out_waiting 检查，让 OS / USB 驱动
        自己处理 TX 队列，热路径省一次系统调用。
        """
        if not self.is_connected or self._ser is None or self._port_lost.is_set():
            if self._connected or self._ser is not None or self._port_lost.is_set():
                self.disconnect()
            self._log_wire_hex(frame, sent=False, stream=stream)
            return False
        if stream:
            if not self._write_lock.acquire(blocking=False):
                record_usb_tx(ok=False, skip_lock=True)
                return False
        elif not self._write_lock.acquire(timeout=0.1):
            record_usb_tx(ok=False, skip_lock=True)
            self._log_wire_hex(frame, sent=False, stream=stream)
            return False
        try:
            ser = self._ser
            if ser is None:
                record_usb_tx(ok=False)
                return False
            out_waiting = 0
            try:
                out_waiting = ser.out_waiting
            except (AttributeError, Exception):
                out_waiting = 0
            # 背压检查：TX 缓冲积压时跳帧（仅流式模式且未禁用时）
            if stream and not ignore_backpressure:
                if out_waiting > SERIAL_TX_BACKPRESSURE_BYTES:
                    record_usb_tx(ok=False, skip_bp=True, out_waiting=out_waiting)
                    return False
            t0 = time.perf_counter()
            try:
                written = ser.write(frame)
                if written != len(frame):
                    raise IOError(f"short serial write: {written}/{len(frame)} bytes")
                if flush and not stream:
                    ser.flush()
            except Exception as e:
                write_ms = (time.perf_counter() - t0) * 1000.0
                record_usb_tx(ok=False, write_ms=write_ms, out_waiting=out_waiting)
                self._tx_fail_streak += 1
                is_timeout = isinstance(e, getattr(serial, "SerialTimeoutException", ()))
                streak = self._tx_fail_streak
                self._log_wire_hex(frame, sent=False, stream=stream)
                if is_timeout and streak < SERIAL_TX_FAIL_STREAK_LIMIT:
                    # 蓝牙 SPP 等虚拟串口链路瞬时拥塞很常见，物理连接通常未断，
                    # 单次/少数几次超时只跳帧+记日志，不误判掉线。
                    comm_diag_warn(
                        f"USB_TX 写超时（第{streak}次，未断开）: {e} out_waiting={out_waiting}"
                    )
                    return False
                comm_diag_warn(
                    f"USB_TX 写入异常（连续{streak}次，判定掉线）: {e} out_waiting={out_waiting}"
                )
                _log(f"串口写入失败（连续{streak}次）: {e}")
                self.disconnect()
                return False
            self._tx_fail_streak = 0
            write_ms = (time.perf_counter() - t0) * 1000.0
            record_usb_tx(
                ok=True, nbytes=len(frame), write_ms=write_ms,
                out_waiting=out_waiting,
            )
            self._log_wire_hex(frame, sent=True, stream=stream)
            return True
        finally:
            self._write_lock.release()

    def send_downlink(self, fun: int, payload: bytes) -> bool:
        frame = build_downlink(DEV_TELEM, fun, payload)
        return self._write_frame(frame)

    def normalize_linear_positions(self, linear: Sequence[float]) -> List[float]:
        """Transport-specific position policy; desktop preserves signed inputs."""
        return [float(v) for v in linear]

    def send_hand_positions(
        self,
        linear: Sequence[float],
        rotary: Sequence[float],
        seq: Optional[int] = None,
        source: str = "unknown",
        joint_angles: Optional[Dict[str, float]] = None,
        stream: bool = False,
        ignore_backpressure: bool = False,
        frame_id: Optional[int] = None,
        command_id: Optional[int] = None,
        hand_side: Optional[str] = None,
    ) -> bool:
        """下发整手批量位置帧。
        stream=True：1kHz 高频流式路径，不 flush，默认有背压检测；
        stream=False：单次命令路径，flush 确保立即发出。
        ignore_backpressure=True：跳过 out_waiting 检查，让 OS 缓冲/USB 驱动排队。
        """
        if not self.should_process_downlink:
            return False
        if seq is None:
            seq = self._downlink_seq
            self._downlink_seq = (self._downlink_seq + 1) & 0xFF
        # 复用 main_app_v3 的预分配帧思路：高频路径只更新 payload/CRC，
        # 位置完全未变时更是只改 seq/CRC。帧字节与旧 build_downlink 一致。
        linear_pos = self.normalize_linear_positions(linear)
        rotary_pos = [float(v) for v in rotary]
        with self._hand_frame_builder_lock:
            builder = self._hand_frame_builder
            if builder is None:
                builder = DownlinkFrameBuilder(linear_pos, rotary_pos)
                self._hand_frame_builder = builder
            if builder.linear == linear_pos and builder.rotary == rotary_pos:
                frame = builder.build_seq_only(seq)
            else:
                frame = builder.build(seq, linear_pos, rotary_pos)
        sent = False
        if self.is_connected:
            sent = self._write_frame(
                frame, flush=not stream, stream=stream,
                ignore_backpressure=ignore_backpressure,
            )
        if self._session_recorder.active:
            self._session_recorder.record_downlink(
                frame, sent=sent, source=source, frame_id=frame_id, command_id=command_id,
                hand_side=_normalize_hand_side(hand_side) if hand_side is not None
                else ("right" if self._hand_type_cache == 1 else "left"),
                usb_connected=self.is_connected,
            )
        if not sent:
            return False
        if self.downlink_log_enabled:
            _log_downlink(
                source, seq, linear_pos, rotary,
                joint_angles=joint_angles, frame=frame,
                connected=self.is_connected, sent=sent,
            )
        return True

    def send_joint_angles(
        self,
        joint_angles: Dict[str, float],
        source: str = "unknown",
        stream: bool = False,
        hand_side: Optional[str] = "left",
        frame_id: Optional[int] = None,
    ) -> bool:
        """将 **Hardware 空间** 关节角（rad）打包为整手批量位置并下发。

        不做 UI↔HW 变号。调用方须先经 ``protocol.convert_ui_to_hardware``
        （或 ``convert_external_to_hardware``）再传入。

        stream=True：高频流式路径；stream=False：单次命令立即 flush。
        hand_side：写入 H7 手型配置（0x2B），不影响本函数内角度符号。
        """
        if not isinstance(joint_angles, Mapping) or not joint_angles or not self.should_process_downlink:
            return False
        try:
            linear, rotary = joint_angles_to_hand_positions(joint_angles)
            linear = self.normalize_linear_positions(linear)
        except (ValueError, TypeError, OverflowError) as exc:
            _log(f"send_joint_angles: rejected invalid target: {exc}")
            return False
        hand_type = 1 if _normalize_hand_side(hand_side) == "right" else 0
        if self._hand_type_cache != hand_type:
            # 每次连接/手别变化仅发一次，避免 1kHz 热路径反复写 H7 Flash。
            self.send_hand_type(hand_side)
        command_id = self._session_recorder.record_conversion(
            joint_angles, linear, rotary, frame_id=frame_id,
            hand_side=_normalize_hand_side(hand_side), source=source,
        )
        ok = self.send_hand_positions(
            linear, rotary, source=source, joint_angles=joint_angles,
            stream=stream, frame_id=frame_id, command_id=command_id, hand_side=hand_side,
        )
        if ok and self._data_reporter is not None:
            try:
                self._data_reporter.record_command(
                    dict(joint_angles),
                    hand_side=_normalize_hand_side(hand_side),
                    source=source,
                )
            except Exception:
                pass
        return ok

    def send_hand_type(self, hand_side: Optional[str]) -> bool:
        """写入 H7 手型配置（0x2B）：0=左手，1=右手；影响旋转电机找零电流方向。"""
        hand_type = 1 if _normalize_hand_side(hand_side) == "right" else 0
        set_rotary_downlink_hand_type(hand_type)
        if not self.is_connected:
            return False
        sent = self.send_downlink(
            FUN_RM_HAND_TYPE_SET_SP,
            pack_rm_hand_type_set(hand_type),
        )
        if sent:
            self._hand_type_cache = hand_type
        return sent

    def send_lm_gravity_enable(
        self, fdcan_bus: int, motor_id: int, enabled: bool,
    ) -> bool:
        """设置单个/广播直线电机重力补偿（0x1A → CAN 0x208）。"""
        if not self.is_connected:
            return False
        cmd = LM_GRAVITY_ENABLE_ON if enabled else LM_GRAVITY_ENABLE_OFF
        return self.send_downlink(
            FUN_LM_GRAVITY_ENABLE,
            pack_lm_gravity_enable(fdcan_bus, motor_id, cmd),
        )

    def send_rm_gravity_enable(self, enabled: bool) -> bool:
        """设置全部旋转电机重力补偿（0x1B）。

        enabled=True 允许固件在 IMU/雅可比有效时自动开启；
        False 强制关闭旋转前馈。
        """
        if not self.is_connected:
            return False
        cmd = LM_GRAVITY_ENABLE_ON if enabled else LM_GRAVITY_ENABLE_OFF
        return self.send_downlink(
            FUN_RM_GRAVITY_ENABLE,
            pack_rm_gravity_enable(cmd),
        )

    def send_gravity_allow_no_imu(self, enabled: bool) -> bool:
        """设置无 IMU 时的重力补偿策略（0x1C）。

        enabled=True 使用固定姿态；False 表示必须有效 IMU 才允许前馈。
        """
        if not self.is_connected:
            return False
        cmd = LM_GRAVITY_ENABLE_ON if enabled else LM_GRAVITY_ENABLE_OFF
        return self.send_downlink(
            FUN_GRAVITY_ALLOW_NO_IMU,
            pack_gravity_allow_no_imu(cmd),
        )

    def send_gravity_master_enable(self, enabled: bool) -> bool:
        """设置H7重力补偿总开关（0x1D，直线与旋转前馈总门控）。"""
        if not self.is_connected:
            return False
        cmd = LM_GRAVITY_ENABLE_ON if enabled else LM_GRAVITY_ENABLE_OFF
        return self.send_downlink(
            FUN_GRAVITY_MASTER_ENABLE,
            pack_gravity_master_enable(cmd),
        )

    def send_lm_homing(self, fdcan_bus: int, motor_id: int, cmd: int,
                        iq_seek: float = DEFAULT_HOMING_IQ_SEEK) -> bool:
        """向单台直线电机发送机械回零命令（0x12）。"""
        if not self.is_connected:
            return False
        payload = pack_lm_mech_homing(fdcan_bus, motor_id, cmd, iq_seek)
        frame = build_downlink(DEV_TELEM, FUN_LM_MECH_HOMING, payload)
        return self._write_frame(frame)

    def send_all_lm_homing(self, cmd: int,
                            iq_seek: float = DEFAULT_HOMING_IQ_SEEK) -> int:
        """向全部 14 台直线电机广播机械回零命令，返回成功发送帧数。

        cmd: LM_HOMING_START(1)=开始寻零  LM_HOMING_RESET(2)=复位回零状态
        layout: CAN1→bus0 M1~6, CAN2→bus1 M1~6, CAN3→bus2 M1~2
        """
        if not self.is_connected:
            _log("send_all_lm_homing: 串口未连接")
            return 0
        lab = "开始寻零" if cmd == LM_HOMING_START else "复位回零状态"
        sent = 0
        for bus, count in enumerate(LINEAR_BUS_COUNTS):
            for mid in range(1, count + 1):
                payload = pack_lm_mech_homing(bus, mid, cmd, iq_seek)
                frame = build_downlink(DEV_TELEM, FUN_LM_MECH_HOMING, payload)
                if self._write_frame(frame):
                    sent += 1
        _log(f"send_all_lm_homing [{lab}] iq_seek={iq_seek:g} "
             f"发送 {sent}/{LINEAR_MOTOR_NUM} 帧")
        return sent

    def send_rm_zero(self, spi_bus: int, motor: Optional[int] = None) -> bool:
        """旋转 SPI 机械回零（0x25），与 main_app_v3 _sp_send_rm_zero 一致。

        motor: None=本总线全部；0=M0；1=M1（若存在）
        """
        if not self.is_connected:
            _log("send_rm_zero: 串口未连接")
            return False
        if spi_bus < 0 or spi_bus >= len(ROTARY_SPI_BUS_COUNTS):
            _log(f"send_rm_zero: 无效 spi_bus={spi_bus}")
            return False
        n = ROTARY_SPI_BUS_COUNTS[spi_bus]
        if motor is None:
            z0 = 1 if n >= 1 else 0
            z1 = 1 if n >= 2 else 0
        elif motor == 0:
            z0, z1 = 1, 0
        elif motor == 1 and n >= 2:
            z0, z1 = 0, 1
        else:
            _log(f"send_rm_zero: 无效 motor={motor} for spi_bus={spi_bus}")
            return False
        payload = pack_rm_zero_sp(spi_bus, z0, z1)
        frame = build_downlink(DEV_TELEM, FUN_RM_ZERO_SP, payload)
        ok = self._write_frame(frame)
        who = "M0+M1" if motor is None else f"M{motor}"
        hw = ROTARY_HW_SPI_LABELS[spi_bus]
        _log(f"send_rm_zero [{hw} {who}] spi_bus={spi_bus} z0={z0} z1={z1} ok={ok}")
        return ok

    def send_rm_drive(self, spi_bus: int, en0: int, en1: int) -> bool:
        """旋转 SPI 使能/断电（0x22），与 main_app_v3 _sp_send_rm_drive 一致。"""
        if spi_bus < 0 or spi_bus >= len(ROTARY_SPI_BUS_COUNTS):
            _log(f"send_rm_drive: 无效 spi_bus={spi_bus}")
            return False
        n = ROTARY_SPI_BUS_COUNTS[spi_bus]
        cache = self._rm_drive_cache_bus(spi_bus)
        cache["en0"] = en0 & 1
        cache["en1"] = en1 & 1
        if n < 2:
            cache["en1"] = 0
        payload = pack_rm_drive_sp(spi_bus, cache["en0"], cache["en1"])
        frame = build_downlink(DEV_TELEM, FUN_RM_DRIVE_SP, payload)
        if not self.is_connected:
            _log("send_rm_drive: 串口未连接")
            self._log_wire_hex(frame, sent=False, stream=False)
            return False
        ok = self._write_frame(frame)
        hw = ROTARY_HW_SPI_LABELS[spi_bus]
        _log(
            f"send_rm_drive [{hw}] spi_bus={spi_bus} "
            f"M0={'使能' if cache['en0'] else '断电'} "
            f"M1={'使能' if cache['en1'] else '断电'} ok={ok}"
        )
        return ok

    def send_rm_drive_motor(self, spi_bus: int, motor: int, enable: bool) -> bool:
        """单路旋转 SPI 使能/断电（与 main_app_v3 _sp_send_rm_drive_motor 一致，保留同总线另一路状态）。"""
        if spi_bus < 0 or spi_bus >= len(ROTARY_SPI_BUS_COUNTS):
            _log(f"send_rm_drive_motor: 无效 spi_bus={spi_bus}")
            return False
        cache = self._rm_drive_cache_bus(spi_bus)
        en0 = cache.get("en0", 0)
        en1 = cache.get("en1", 0)
        bit = 1 if enable else 0
        if motor == 0:
            en0 = bit
        elif motor == 1 and ROTARY_SPI_BUS_COUNTS[spi_bus] >= 2:
            en1 = bit
        else:
            _log(f"send_rm_drive_motor: 无效 motor={motor} for spi_bus={spi_bus}")
            return False
        return self.send_rm_drive(spi_bus, en0, en1)

    def send_rm_pos_motor(self, spi_bus: int, motor: int, pos_deg: float) -> bool:
        """旋转 SPI 单点位置（0x21），仅下发指定 M0/M1。"""
        if not self.is_connected:
            _log("send_rm_pos_motor: 串口未连接")
            return False
        if spi_bus < 0 or spi_bus >= len(ROTARY_SPI_BUS_COUNTS):
            _log(f"send_rm_pos_motor: 无效 spi_bus={spi_bus}")
            return False
        n = ROTARY_SPI_BUS_COUNTS[spi_bus]
        if motor == 0:
            motor_mask = 0x01
            pos0, pos1 = float(pos_deg), 0.0
        elif motor == 1 and n >= 2:
            motor_mask = 0x02
            pos0, pos1 = 0.0, float(pos_deg)
        else:
            _log(f"send_rm_pos_motor: 无效 motor={motor} for spi_bus={spi_bus}")
            return False
        payload = pack_rm_pos_set_sp(spi_bus, pos0, pos1, motor_mask)
        frame = build_downlink(DEV_TELEM, FUN_RM_POS_SET_SP, payload)
        ok = self._write_frame(frame)
        hw = ROTARY_HW_SPI_LABELS[spi_bus]
        _log(
            f"send_rm_pos_motor [{hw} M{motor}] spi_bus={spi_bus} "
            f"pos={pos_deg:g}° mask=0x{motor_mask:X} ok={ok}"
        )
        return ok

    def send_rm_elec_calib(
        self,
        spi_bus: int,
        motor_mask: int,
        *,
        run: bool = True,
    ) -> bool:
        """触发旋转电机离线电角度标定（0x2C），线格式与main_app_v3一致。"""
        if not self.is_connected or not 0 <= spi_bus < len(ROTARY_SPI_BUS_COUNTS):
            return False
        flags = RM_ELEC_CALIB_FLAG_RUN if run else 0
        return self.send_downlink(
            FUN_RM_ELEC_CALIB_SP,
            pack_rm_elec_calib_sp(spi_bus, flags, motor_mask),
        )

    def send_rm_elec_offset(
        self,
        spi_bus: int,
        motor_mask: int,
        offset0_deg: float,
        offset1_deg: float,
        *,
        save: bool = True,
    ) -> bool:
        """写旋转电机机械域电角度偏置（0x2D），线格式与main_app_v3一致。"""
        if not self.is_connected or not 0 <= spi_bus < len(ROTARY_SPI_BUS_COUNTS):
            return False
        flags = RM_ELEC_OFFSET_FLAG_SAVE if save else 0
        return self.send_downlink(
            FUN_RM_ELEC_OFFSET_SP,
            pack_rm_elec_offset_sp(
                spi_bus,
                flags,
                motor_mask,
                offset0_deg,
                offset1_deg,
            ),
        )

    def send_raw(self, data: bytes) -> bool:
        return self._write_frame(data)

    def ping(self) -> bool:
        return self.is_connected

    def read_latest_telemetry(self) -> Optional[dict]:
        """返回最新解析后的遥测帧 dict（linear/rotary 槽位对象列表）。

        RX 线程只解帧头并缓存 payload；首个读者延后完整解析，
        同一上行帧之后直接复用缓存，避免 Form5/日志计时器重复解析和重复记录。
        """
        with self._telem_lock:
            generation = self._telem_generation
            if (
                self._telem_frame_cache is not None
                and self._telem_parsed_generation == generation
            ):
                frame = self._telem_frame_cache
                needs_report = self._telem_reported_generation != generation
            else:
                frame = None
                needs_report = False
            payload = bytes(self._telem_payload) if self._telem_payload else None

        if frame is None:
            if payload is None or len(payload) not in _TELEM_ACCEPT_PLEN:
                return None
            try:
                from usb.telemetry import (
                    materialize_telem_frame,
                    parse_hand_telem_lazy,
                    parse_hand_telem_legacy,
                )

                if len(payload) == TELEM_FRAME_SIZE:
                    lazy = parse_hand_telem_lazy(payload)
                    frame = materialize_telem_frame(lazy) if lazy is not None else None
                else:
                    frame = parse_hand_telem_legacy(payload)
            except Exception as exc:
                _log(f"read_latest_telemetry 解析失败: {exc}")
                return None
            if not isinstance(frame, dict):
                return None
            linear = frame.get("linear")
            rotary = frame.get("rotary")
            if not isinstance(linear, list) or not isinstance(rotary, list):
                return None
            with self._telem_lock:
                # 解析期间若新帧已到，不用旧帧覆盖最新缓存。
                if generation == self._telem_generation:
                    self._telem_frame_cache = frame
                    self._telem_parsed_generation = generation
                    needs_report = self._telem_reported_generation != generation

        if needs_report and self._data_reporter is not None:
            try:
                side = "right" if int(frame.get("hand_type", 0)) == 1 else "left"
                self._data_reporter.record_telemetry(frame, hand_side=side)
            except Exception:
                pass
            finally:
                with self._telem_lock:
                    if generation == self._telem_generation:
                        self._telem_reported_generation = generation
        return frame

    def get_telemetry_rx_rate_hz(self) -> float:
        with self._telem_lock:
            return float(self._telem_rx_rate_hz)

    def read_latest_heartbeat(self) -> Optional[dict]:
        """返回最新 H7 心跳快照；尚未接收或 magic 无效时返回 None。"""
        with self._telem_lock:
            return dict(self._heartbeat_cache) if self._heartbeat_cache is not None else None

    def read_latest_joint_angles(
        self,
        hand_side: Optional[str] = "left",
    ) -> Optional[Dict[str, float]]:
        """从最新上行遥测反解当前关节角（UI 空间 rad）；无可用遥测返回 None。

        hand_side 必须与当前真实手型一致，否则四指 swing / rota 的硬件符号无法正确还原，
        常见结果是 swing 被钳到 0，表现为手指并拢。
        """
        frame = self.read_latest_telemetry()
        if frame is None:
            return None
        cfg = joint_cfg_from_telemetry_frame(
            frame, use_ref=False, hand_side=hand_side
        )
        _log(f"遥测解算关节角(rad): {_format_joint_angles_for_log(cfg)}")
        return cfg

    def read_status(self) -> Optional[dict]:
        return None

    def set_control_mode(self, mode: str) -> bool:
        _log(f"set_control_mode({mode!r}): 未实现")
        return False

    def set_joint_limits(self, limits: Dict[str, tuple]) -> bool:
        _log(f"set_joint_limits: 未实现")
        return False

    def emergency_stop(self) -> bool:
        """下发安全下限姿态，避免急停帧绕过 H7 关节范围。"""
        zero = [0.0] * LINEAR_MOTOR_NUM
        rot = [0.0] * ROTARY_MOTOR_NUM
        # send_hand_positions 接收的是线上角度（deg），不会再经过 UI→HW
        # 协议转换。因此急停也必须显式遵守 thumb_rota 的 0.5° 下限；
        # thumb_swing 的 H7 下限本来就是 0°，保留初始化值即可。
        rot[ROTARY_JOINT_SLOTS[_THUMB_ROTA_JOINT]] = math.degrees(
            _THUMB_ROTA_HW_CLAMP[0]
        )
        # 左手四指 swing 的 H7 下限现为 0.5°；右手仍允许 0°。
        if self._hand_type_cache != 1:
            for joint in _FINGER_SWING_JOINTS:
                rot[ROTARY_JOINT_SLOTS[joint]] = 0.5
        return self.send_hand_positions(zero, rot, source="UsbController/急停")


# 兼容旧代码：hidapi 探测
try:
    import hid as _hid
    _HID_AVAILABLE = True
except ImportError:
    _hid = None
    _HID_AVAILABLE = False

_global_usb: Optional[UsbController] = None


def get_usb_controller() -> UsbController:
    global _global_usb
    if _global_usb is None:
        _global_usb = UsbController()
    return _global_usb


# ────────────────────────────────────────────────────────────────────────
# 五指顺序往返 / 简易串口控制器（原 test_usb.py）
# ────────────────────────────────────────────────────────────────────────


class _SerialRxDrain:
    """SerialController 专用 RX 排空（UsbController 自带 drain，勿重复启动）。"""

    def __init__(self, get_ser: Callable[[], Optional["serial.Serial"]]) -> None:
        self._get_ser = get_ser
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, name="SerialRxDrain", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        t = self._thread
        if t is not None:
            t.join(timeout=1.0)

    def _loop(self) -> None:
        _set_thread_priority("above_normal")
        idle = 0
        while not self._stop.is_set():
            ser = self._get_ser()
            if ser is None or not ser.is_open:
                time.sleep(0.01)
                continue
            try:
                chunk = _drain_serial_port(ser)
                if not chunk:
                    idle += 1
                    if idle % 2000 == 0:
                        snapshot_usb_port(ser)
                    time.sleep(RX_IDLE_SLEEP_S)
                    continue
                idle = 0
                t0 = time.perf_counter()
                in_wait = ser.in_waiting
                record_usb_rx(
                    nbytes=len(chunk),
                    in_waiting_after=in_wait,
                    drain_ms=(time.perf_counter() - t0) * 1000.0,
                )
                while not self._stop.is_set():
                    if ser.in_waiting <= 0:
                        break
                    extra = _drain_serial_port(ser)
                    if not extra:
                        break
                    record_usb_rx(nbytes=len(extra), in_waiting_after=ser.in_waiting)
            except Exception:
                time.sleep(0.01)


class DownlinkFrameBuilder:
    """缓存整手批量帧，仅更新 payload / CRC，减少高频路径分配。"""

    USB_HDR_LEN = 6

    def __init__(self, linear: List[float], rotary: List[float]) -> None:
        self.linear = list(linear)
        self.rotary = list(rotary)
        payload = pack_hand_pos_batch(0, linear, rotary)
        self.frame = bytearray(build_downlink(DEV_TELEM, FUN_HAND_POS_BATCH, payload))
        self._payload_off = self.USB_HDR_LEN
        self._crc_off = self.USB_HDR_LEN + HAND_POS_BATCH_SIZE

    def build(self, seq: int, linear: List[float], rotary: List[float]) -> bytes:
        self.linear = list(linear)
        self.rotary = list(rotary)
        payload = pack_hand_pos_batch(seq, linear, rotary)
        self.frame[self._payload_off : self._payload_off + HAND_POS_BATCH_SIZE] = payload
        crc = crc16_modbus(memoryview(self.frame)[: self._crc_off])
        struct.pack_into("<H", self.frame, self._crc_off, crc)
        return bytes(self.frame)

    def build_seq_only(self, seq: int) -> bytes:
        self.frame[self._payload_off] = seq & 0xFF
        crc = crc16_modbus(memoryview(self.frame)[: self._crc_off])
        struct.pack_into("<H", self.frame, self._crc_off, crc)
        return bytes(self.frame)


class SerialController:
    """轻量串口控制器（CLI / 独立测试用）。GUI 路径请优先用 UsbController。"""

    def __init__(self, port: str = "", baudrate: int = SERIAL_BAUDRATE):
        self._port = port
        self._baudrate = baudrate
        self._ser: Optional["serial.Serial"] = None
        self._write_lock = threading.Lock()
        self._is_connected = False
        self._rx_drain: Optional[_SerialRxDrain] = None

    @property
    def is_connected(self) -> bool:
        return self._is_connected and self._ser is not None and self._ser.is_open

    def connect(self) -> bool:
        if self.is_connected:
            return True
        if not _SERIAL_AVAILABLE or serial is None:
            print("[SerialController] pyserial 不可用")
            return False
        try:
            self._ser = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                timeout=SERIAL_READ_TIMEOUT_S,
                write_timeout=SERIAL_WRITE_TIMEOUT_S,
            )
            self._ser.dtr = True
            self._ser.rts = False
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
            self._is_connected = True
            self._rx_drain = _SerialRxDrain(lambda: self._ser)
            self._rx_drain.start()
            return True
        except Exception as e:
            print(f"[SerialController] 连接失败: {e}")
            return False

    def disconnect(self) -> None:
        if self._rx_drain is not None:
            self._rx_drain.stop()
            self._rx_drain = None
        if self._ser is not None:
            try:
                self._ser.close()
            except Exception:
                pass
            self._ser = None
        self._is_connected = False

    def send_hand_positions(
        self,
        linear: Sequence[float],
        rotary: Sequence[float],
        seq: int = 0,
        stream: bool = False,
        source: str = "",
    ) -> bool:
        if not self.is_connected:
            return False
        try:
            payload = pack_hand_pos_batch(seq, linear, rotary)
            frame = build_downlink(DEV_TELEM, FUN_HAND_POS_BATCH, payload)
            return self._write_downlink_bytes(frame, stream=stream)
        except Exception as e:
            print(f"[SerialController] 发送失败: {e}")
            return False

    def _write_downlink_bytes(self, frame: bytes, *, stream: bool = False) -> bool:
        ser = self._ser
        if ser is None:
            record_usb_tx(ok=False)
            return False
        if stream:
            if not self._write_lock.acquire(blocking=False):
                record_usb_tx(ok=False, skip_lock=True)
                return False
        elif not self._write_lock.acquire(timeout=0.1):
            record_usb_tx(ok=False, skip_lock=True)
            return False
        try:
            ser = self._ser
            if ser is None:
                record_usb_tx(ok=False)
                return False
            try:
                out_waiting = ser.out_waiting
            except (AttributeError, Exception):
                out_waiting = 0
            if out_waiting > SERIAL_TX_BACKPRESSURE_BYTES:
                if stream:
                    record_usb_tx(ok=False, skip_bp=True, out_waiting=out_waiting)
                    return False
                time.sleep(0.01)
                try:
                    out_waiting = ser.out_waiting
                except (AttributeError, Exception):
                    out_waiting = 0
                if out_waiting > SERIAL_TX_BACKPRESSURE_BYTES:
                    record_usb_tx(ok=False, skip_bp=True, out_waiting=out_waiting)
                    return False
            t0 = time.perf_counter()
            try:
                ser.write(frame)
            except Exception as exc:
                comm_diag_warn(f"SerialController 写失败: {exc}")
                record_usb_tx(
                    ok=False,
                    write_ms=(time.perf_counter() - t0) * 1000.0,
                    out_waiting=out_waiting,
                )
                return False
            write_ms = (time.perf_counter() - t0) * 1000.0
            record_usb_tx(
                ok=True,
                nbytes=len(frame),
                write_ms=write_ms,
                out_waiting=out_waiting,
            )
        finally:
            self._write_lock.release()
        return True

    @staticmethod
    def list_ports() -> List[str]:
        ports: List[str] = []
        if list_ports is None:
            return ports
        try:
            for p in list_ports.comports():
                ports.append(f"{p.device} — {p.description or '未知'}")
        except Exception:
            pass
        return ports


@dataclass
class FingerSweepParams:
    pos_start: float = 0.0
    pos_end: float = -16.0
    period_s: float = 4.0
    send_hz: float = DOWNLINK_HZ_DEFAULT
    loop: bool = True
    # True：跳过 out_waiting 背压检查，尽量按 send_hz 往 OS 缓冲塞帧
    ignore_backpressure: bool = os.environ.get("HAND_IGNORE_TX_BP", "0").strip().lower() in (
        "1",
        "true",
        "yes",
    )

    @property
    def interval_s(self) -> float:
        return 1.0 / max(self.send_hz, 1.0)


class FingerSeqSweepWorker:
    """五指顺序往返；serial 可为 SerialController 或 UsbController。"""

    def __init__(self, serial_controller) -> None:
        self._serial = serial_controller
        self.params = FingerSweepParams()
        self.on_group_changed: Optional[Callable[[int, str], None]] = None
        self.on_stopped: Optional[Callable[[], None]] = None
        self.on_finished: Optional[Callable[[int], None]] = None
        self._stop_event = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._send_count = 0
        self._frame_checked = False
        self._local_rx_drain: Optional[_SerialRxDrain] = None

    @property
    def send_count(self) -> int:
        return self._send_count

    @property
    def is_running(self) -> bool:
        t = self._thread
        return t is not None and t.is_alive()

    def start(self) -> bool:
        with self._lock:
            if self.is_running:
                return False
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._run,
                name="FingerSeqSweep",
                daemon=True,
            )
            self._thread.start()
        return True

    def stop(self) -> None:
        self._stop_event.set()

    def _get_underlying_serial(self) -> Optional["serial.Serial"]:
        ser_iface = self._serial
        if ser_iface is None:
            return None
        return getattr(ser_iface, "_ser", None)

    def _send_downlink_frame(self, frame: bytes) -> bool:
        if not verify_hand_pos_frame(frame):
            return False
        ser_iface = self._serial
        if ser_iface is None or not getattr(ser_iface, "is_connected", False):
            return False
        if hasattr(ser_iface, "_write_downlink_bytes"):
            return bool(ser_iface._write_downlink_bytes(frame, stream=True))
        if hasattr(ser_iface, "_write_frame"):
            return bool(
                ser_iface._write_frame(
                    frame,
                    flush=False,
                    stream=True,
                    ignore_backpressure=self.params.ignore_backpressure,
                )
            )
        return False

    def _serial_has_rx_drain(self) -> bool:
        ser_iface = self._serial
        if ser_iface is None:
            return False
        rx_thread = getattr(ser_iface, "_rx_thread", None)
        if rx_thread is not None and rx_thread.is_alive():
            return True
        rx_drain = getattr(ser_iface, "_rx_drain", None)
        if rx_drain is not None:
            t = getattr(rx_drain, "_thread", None)
            if t is not None and t.is_alive():
                return True
        return False

    def join(self, timeout: float = 2.0) -> None:
        t = self._thread
        if t is not None:
            t.join(timeout=timeout)

    def _run(self) -> None:
        _set_thread_priority("below_normal")
        self._send_count = 0
        self._frame_checked = False
        if not self._serial_has_rx_drain():
            self._local_rx_drain = _SerialRxDrain(self._get_underlying_serial)
            self._local_rx_drain.start()
        params = self.params
        comm_diag_info(
            f"SWEEP 启动 send_hz={params.send_hz:g} "
            f"ignore_bp={params.ignore_backpressure} "
            f"线速参考上限≈{theoretical_line_max_hz():.0f}Hz "
            f"rx_drain={'UsbController' if self._serial_has_rx_drain() else 'local'}"
        )
        try:
            interval_s = params.interval_s
            n_groups = len(FINGER_SEQ_GROUPS)
            total_period = params.period_s * n_groups

            seq = 0
            t0 = time.perf_counter()
            next_tick = t0 + interval_s
            last_group_idx = -1

            builder = None
            last_linear: List[float] = []
            last_rotary: List[float] = []

            while not self._stop_event.is_set():
                loop_t0 = time.perf_counter()
                now = loop_t0
                slipped = False
                if now < next_tick:
                    wait_until(next_tick)
                else:
                    slip = now - next_tick
                    if slip >= interval_s:
                        slipped = True
                        next_tick = now

                elapsed = time.perf_counter() - t0

                if not params.loop and elapsed >= total_period:
                    break

                period_s = max(params.period_s, DL_SWEEP_PERIOD_S_MIN)
                group_idx = int(elapsed // period_s) % n_groups
                local_elapsed = elapsed - (int(elapsed // period_s) * period_s)

                if group_idx != last_group_idx:
                    last_group_idx = group_idx
                    cb = self.on_group_changed
                    if cb is not None:
                        try:
                            cb(group_idx, FINGER_SEQ_GROUPS[group_idx][0])
                        except Exception:
                            pass

                sweep = sine_sweep_pos(
                    local_elapsed, params.pos_start, params.pos_end, period_s
                )
                _, slots = FINGER_SEQ_GROUPS[group_idx]
                slot_set = set(slots)

                linear = [
                    sweep if i in slot_set else 0.0 for i in range(LINEAR_MOTOR_NUM)
                ]
                rotary = [0.0] * ROTARY_MOTOR_NUM

                if builder is None:
                    builder = DownlinkFrameBuilder(linear, rotary)
                    frame = builder.build(seq, linear, rotary)
                elif linear == last_linear and rotary == last_rotary:
                    frame = builder.build_seq_only(seq)
                else:
                    frame = builder.build(seq, linear, rotary)

                last_linear = linear
                last_rotary = rotary

                if not self._frame_checked:
                    if not verify_hand_pos_frame(frame):
                        print("[FingerSeqSweep] 帧格式校验失败，停止发送")
                        break
                    self._frame_checked = True

                sent = self._send_downlink_frame(frame)
                if sent:
                    seq = (seq + 1) & 0xFF
                    self._send_count += 1
                else:
                    time.sleep(0.001)
                loop_ms = (time.perf_counter() - loop_t0) * 1000.0
                record_sweep_tick(sent=sent, slip=slipped, loop_ms=loop_ms)

                next_tick += interval_s

        finally:
            comm_diag_info(f"SWEEP 结束 send_count={self._send_count}")
            if self._local_rx_drain is not None:
                self._local_rx_drain.stop()
                self._local_rx_drain = None
            cb = self.on_stopped
            if cb is not None:
                try:
                    cb()
                except Exception:
                    pass
            cb_finish = self.on_finished
            if cb_finish is not None:
                try:
                    cb_finish(self._send_count)
                except Exception:
                    pass


if __name__ == "__main__":
    import logging
    from datetime import datetime as _dt

    _log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")
    os.makedirs(_log_dir, exist_ok=True)
    _log_file = os.path.join(
        _log_dir, _dt.now().strftime("usb_offline_%Y%m%d_%H%M%S.log")
    )
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s.%(msecs)03d [%(threadName)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S",
        handlers=[
            logging.StreamHandler(sys.stdout),
            logging.FileHandler(_log_file, encoding="utf-8"),
        ],
    )
    logging.getLogger("comm_diag").setLevel(logging.INFO)

    print("五指顺序往返测试工具（usb.offline）")
    print("=" * 40)

    ports = SerialController.list_ports()
    if not ports:
        print("未找到串口设备")
        sys.exit(1)

    print("可用串口:")
    for i, p in enumerate(ports, 1):
        print(f"  {i}. {p}")

    choice = int(input("请选择串口编号: ")) - 1
    if choice < 0 or choice >= len(ports):
        print("无效选择")
        sys.exit(1)

    port = ports[choice].split(" — ")[0].strip()
    print(f"选择: {port}")

    usb = SerialController(port=port)
    if not usb.connect():
        print("连接失败")
        sys.exit(1)
    print("连接成功")

    worker = FingerSeqSweepWorker(usb)
    worker.params.pos_end = -16.0
    worker.params.period_s = 4.0
    worker.params.send_hz = DOWNLINK_HZ_DEFAULT
    worker.on_group_changed = lambda idx, name: print(f"[{idx + 1}/5] {name}")
    worker.on_stopped = lambda: print("发送停止")

    worker.start()
    print("开始发送（按 Ctrl+C 停止）")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("停止中...")
        worker.stop()
        worker.join()
        usb.disconnect()
        print("已停止")
