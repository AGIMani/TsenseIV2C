"""外部 Retarget / 历史动作文件 ↔ UI ↔ H7 角度协议。

thumb_swing：UI、URDF、Hardware 均为 [0,2] rad，三者同值；中位为 1。
2026-09 坐标重定义 q_new = 1 - q_old，物理姿态与旧版 H7 指令保持不变。
External / Protocol 仍保留历史 [-1,1]，转 UI 时 ui = 1 - protocol。
MotionFile 仍保留历史 [-1,1]：左手 ui = 1-file，右手 ui = 1+file；保存为逆变换。
右手 pinky_rota 下发取负；四指 swing 左右均正，左手硬件边界夹紧到 [0.5°,39.5°]。
thumb_rota 硬件范围 [0.5°,89.5°]。MCP/PIP/DIP 另由 finger_linkage 转直线行程。
显示转换见 hand_display_protocol。USB send_joint_angles 接收 Hardware 坐标。
"""
from __future__ import annotations

import math
from typing import Dict, Iterable, Mapping, MutableMapping, Optional, Tuple

Range = Tuple[float, float]
Cfg = Dict[str, float]

# ── 短名 ↔ URDF ───────────────────────────────────────────────────────────

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

# ── 角度常量（与 usb/offline.py、skeleton/tracker.JOINT_LIMITS 一致）──────

_DEG_SWING_MAX = 0.696         # 与 URDF / MJCF 精确一致，约 39.88°
_FINGER_SWING_HW_LEFT_MIN = math.radians(0.5)
_FINGER_SWING_HW_LEFT_MAX = math.radians(39.5)
_DEG_50 = math.radians(50.0)   # ≈ 0.8727
_FLEX_MAX = 1.57               # ≈ 90°，MCP/PIP/DIP
_THUMB_ROTA_HW_MIN = math.radians(0.5)
_THUMB_ROTA_HW_MAX = math.radians(89.5)
# thumb_swing：UI/URDF/H7 同为 [0,2]；旧坐标 q_old 对应 1-q_old。
_THUMB_SWING_MAX = 2.0
_THUMB_SWING_HW_MIN = 0.0
_THUMB_SWING_HW_MAX = 2.0
_FOUR_FINGER_SWING_SHORTS = (
    "index_swing", "mid_swing", "ring_swing", "pinky_swing"
)

# UI（本项目内部，单位 rad）
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

# Protocol（对外中间量纲；与顶部映射总表一致）
#   thumb_swing: External [-1,1] → Protocol [-1,1]（同号）
#   四指 swing:  External ±0.348 → [0, 0.696]
#   rota / MCP/PIP/DIP: 与 External 同为 [0, 1.57]
_PROTO_SWING = 0.696  # = 2 * 0.348
PROTOCOL_RANGES: Dict[str, Range] = {
    "thumb_rota": (0.0, 1.57),
    "thumb_swing": (-1.0, 1.0),
    "thumb_mcp": (0.0, 1.57),
    "thumb_pip": (0.0, 1.57),
    "index_swing": (0.0, _PROTO_SWING),
    "index_mcp": (0.0, 1.57),
    "index_pip": (0.0, 1.57),
    "index_dip": (0.0, 1.57),
    "mid_swing": (0.0, _PROTO_SWING),
    "mid_mcp": (0.0, 1.57),
    "mid_pip": (0.0, 1.57),
    "mid_dip": (0.0, 1.57),
    "ring_swing": (0.0, _PROTO_SWING),
    "ring_mcp": (0.0, 1.57),
    "ring_pip": (0.0, 1.57),
    "ring_dip": (0.0, 1.57),
    "pinky_rota": (0.0, 1.57),
    "pinky_swing": (0.0, _PROTO_SWING),
    "pinky_mcp": (0.0, 1.57),
    "pinky_pip": (0.0, 1.57),
    "pinky_dip": (0.0, 1.57),
}

# External 默认限位（来自 Left hand URDF model.urdf，可在 convert_* 时覆盖）
#   thumb_swing [-1,1]；四指 swing ±0.348；thumb/pinky rota 与 MCP/PIP/DIP [0,1.57]
_EXT_SWING = 0.348
EXTERNAL_RANGES_DEFAULT: Dict[str, Range] = {
    "thumb_rota": (0.0, 1.57),
    "thumb_swing": (-1.0, 1.0),
    "thumb_mcp": (0.0, 1.57),
    "thumb_pip": (0.0, 1.57),
    "index_swing": (-_EXT_SWING, _EXT_SWING),
    "index_mcp": (0.0, 1.57),
    "index_pip": (0.0, 1.57),
    "index_dip": (0.0, 1.57),
    "mid_swing": (-_EXT_SWING, _EXT_SWING),
    "mid_mcp": (0.0, 1.57),
    "mid_pip": (0.0, 1.57),
    "mid_dip": (0.0, 1.57),
    "ring_swing": (-_EXT_SWING, _EXT_SWING),
    "ring_mcp": (0.0, 1.57),
    "ring_pip": (0.0, 1.57),
    "ring_dip": (0.0, 1.57),
    "pinky_rota": (0.0, 1.57),
    "pinky_swing": (-_EXT_SWING, _EXT_SWING),
    "pinky_mcp": (0.0, 1.57),
    "pinky_pip": (0.0, 1.57),
    "pinky_dip": (0.0, 1.57),
}

# Hardware 覆盖范围（含符号）
HW_RANGES_LEFT: Dict[str, Range] = {
    "thumb_rota": (_THUMB_ROTA_HW_MIN, _THUMB_ROTA_HW_MAX),
    "thumb_swing": (_THUMB_SWING_HW_MIN, _THUMB_SWING_HW_MAX),  # UI 同值
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
    "thumb_rota": (_THUMB_ROTA_HW_MIN, _THUMB_ROTA_HW_MAX),  # 右手同号
    "thumb_swing": (_THUMB_SWING_HW_MIN, _THUMB_SWING_HW_MAX),  # UI 同值
    "thumb_mcp": (0.0, _FLEX_MAX),
    "thumb_pip": (0.0, _FLEX_MAX),
    "index_swing": (0.0, _DEG_SWING_MAX),   # 右手四指 swing 保持正
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

# UI→HW 符号规则说明（本文件实现，offline 不再做角度变号）
#   四指 swing : 左右手均同号（正值下发）
#   pinky_rota : 左手正；右手取负
#   thumb_rota : 左右手均同号（正），H7 边界直接钳到 [0.5°,89.5°]
#   thumb_swing: 左右手均与 UI 同值；H7 中位 1 rad、范围 [0,2]
#                MediaPipe 右手已在输入端镜像点云，retarget 输出同值进入 Form2 UI
#   MCP/PIP/DIP: UI 弧度 clamp；直线 mm 由 finger_linkage（stroke_sign=-1）另算


# ── 基础工具 ─────────────────────────────────────────────────────────────

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


def affine_map(
    x: float,
    src: Range,
    dst: Range,
    *,
    do_clamp: bool = True,
) -> float:
    """把 x 从 src 区间线性映射到 dst 区间。"""
    s0, s1 = float(src[0]), float(src[1])
    d0, d1 = float(dst[0]), float(dst[1])
    if do_clamp:
        x = clamp(float(x), min(s0, s1), max(s0, s1))
    else:
        x = float(x)
    span = s1 - s0
    if abs(span) < 1e-12:
        return d0
    t = (x - s0) / span
    return d0 + t * (d1 - d0)


def resolve_short_name(key: str) -> Optional[str]:
    """任意键名 → 短名；无法识别返回 None。"""
    k = str(key).strip()
    if k in SHORT_TO_URDF:
        return k
    if k in URDF_TO_SHORT:
        return URDF_TO_SHORT[k]
    # 容忍 right_hand_* → 按左手短名处理
    if k.startswith("right_hand_"):
        left = "left_hand_" + k[len("right_hand_") :]
        return URDF_TO_SHORT.get(left)
    return None


def to_urdf_key(key: str) -> str:
    short = resolve_short_name(key)
    if short is None:
        return str(key)
    return SHORT_TO_URDF[short]


def normalize_cfg_keys(cfg: Mapping[str, float], *, as_urdf: bool = True) -> Cfg:
    """统一键名；重复短名/长名时后者覆盖。"""
    out: Cfg = {}
    for k, v in cfg.items():
        short = resolve_short_name(k)
        if short is None:
            out[str(k)] = float(v)
            continue
        name = SHORT_TO_URDF[short] if as_urdf else short
        out[name] = float(v)
    return out


def _map_cfg_ranges(
    cfg: Mapping[str, float],
    src_ranges: Mapping[str, Range],
    dst_ranges: Mapping[str, Range],
    *,
    as_urdf: bool = True,
    pass_through_unknown: bool = True,
) -> Cfg:
    out: Cfg = {}
    for k, v in cfg.items():
        short = resolve_short_name(k)
        if short is None or short not in src_ranges or short not in dst_ranges:
            if pass_through_unknown:
                out[to_urdf_key(k) if as_urdf else str(k)] = float(v)
            continue
        mapped = affine_map(float(v), src_ranges[short], dst_ranges[short])
        out[SHORT_TO_URDF[short] if as_urdf else short] = mapped
    return out

# ── External ↔ Protocol ↔ UI ─────────────────────────────────────────────

def convert_external_to_protocol(
    cfg: Mapping[str, float],
    *,
    external_ranges: Optional[Mapping[str, Range]] = None,
    as_urdf: bool = True,
) -> Cfg:
    """External → Protocol。

    例：thumb_swing -1→-1，+1→+1（默认 EXTERNAL/PROTOCOL 均为 [-1,1]）。
    """
    src: Dict[str, Range] = dict(EXTERNAL_RANGES_DEFAULT)
    if external_ranges:
        for k, v in external_ranges.items():
            sk = resolve_short_name(k)
            if sk is not None:
                src[sk] = (float(v[0]), float(v[1]))
    return _map_cfg_ranges(cfg, src, PROTOCOL_RANGES, as_urdf=as_urdf)


def convert_protocol_to_ui(
    cfg: Mapping[str, float],
    *,
    as_urdf: bool = True,
) -> Cfg:
    """Protocol → UI(rad)。

    thumb_swing 保留 External/Protocol 旧坐标：-1→2，0→1，+1→0。
    """
    endpoints = dict(UI_RANGES)
    endpoints["thumb_swing"] = (2.0, 0.0)
    return _map_cfg_ranges(cfg, PROTOCOL_RANGES, endpoints, as_urdf=as_urdf)


def convert_ui_to_protocol(
    cfg: Mapping[str, float],
    *,
    as_urdf: bool = True,
) -> Cfg:
    """UI(rad) → Protocol。"""
    endpoints = dict(UI_RANGES)
    endpoints["thumb_swing"] = (2.0, 0.0)
    return _map_cfg_ranges(cfg, endpoints, PROTOCOL_RANGES, as_urdf=as_urdf)


def convert_protocol_to_external(
    cfg: Mapping[str, float],
    *,
    external_ranges: Optional[Mapping[str, Range]] = None,
    as_urdf: bool = True,
) -> Cfg:
    """Protocol → External。"""
    dst = dict(EXTERNAL_RANGES_DEFAULT)
    if external_ranges:
        for k, v in external_ranges.items():
            sk = resolve_short_name(k)
            if sk is not None:
                dst[sk] = (float(v[0]), float(v[1]))
    return _map_cfg_ranges(cfg, PROTOCOL_RANGES, dst, as_urdf=as_urdf)


def convert_external_to_ui(
    cfg: Mapping[str, float],
    *,
    external_ranges: Optional[Mapping[str, Range]] = None,
    as_urdf: bool = True,
) -> Cfg:
    """External → Protocol → UI(rad)。第三方接入本项目的推荐入口。"""
    proto = convert_external_to_protocol(
        cfg, external_ranges=external_ranges, as_urdf=as_urdf
    )
    return convert_protocol_to_ui(proto, as_urdf=as_urdf)


def convert_ui_to_external(
    cfg: Mapping[str, float],
    *,
    external_ranges: Optional[Mapping[str, Range]] = None,
    as_urdf: bool = True,
) -> Cfg:
    """UI(rad) → Protocol → External。"""
    proto = convert_ui_to_protocol(cfg, as_urdf=as_urdf)
    return convert_protocol_to_external(
        proto, external_ranges=external_ranges, as_urdf=as_urdf
    )


# ── UI ↔ Hardware（符号规则；权威实现在此，usb.offline 只下发）──────────
def convert_ui_to_hardware(
    cfg: Mapping[str, float],
    hand_side: Optional[str] = "left",
) -> Cfg:
    """UI(rad) → Hardware(rad)。左右手分支独立，勿混用。

    左手：四指 swing 同号并截断到[0.5°,39.5°]；thumb_swing 同值下发 [0,2] rad。
    右手：thumb_swing 同值下发 [0,2] rad；pinky_rota 取负。
    两手 thumb_rota 均同号，并在 H7 边界直接钳到 [0.5°,89.5°]。
    """
    side = normalize_hand_side(hand_side)
    ui = normalize_cfg_keys(cfg, as_urdf=True)
    out: Cfg = dict(ui)
    for urdf, value in ui.items():
        short = URDF_TO_SHORT.get(urdf)
        if short is None or short not in UI_RANGES:
            continue
        ulo, uhi = UI_RANGES[short]
        v = clamp(float(value), ulo, uhi)
        if side == "right":
            # 右手：thumb_rota 同号；pinky_rota 取负
            if short == "pinky_rota":
                v = -v
        # thumb_rota / flex / 四指 swing 左右手均同号
        hw = HW_RANGES_RIGHT[short] if side == "right" else HW_RANGES_LEFT[short]
        out[urdf] = clamp(v, hw[0], hw[1])
    return out


def convert_hardware_to_ui(
    cfg: Mapping[str, float],
    hand_side: Optional[str] = "left",
) -> Cfg:
    """Hardware(rad) → UI(rad)。与 convert_ui_to_hardware 互逆；左右手分开处理。"""
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


def convert_external_to_hardware(
    cfg: Mapping[str, float],
    hand_side: Optional[str] = "left",
    *,
    external_ranges: Optional[Mapping[str, Range]] = None,
) -> Cfg:
    """External → Protocol → UI → Hardware。下发前一条龙转换。"""
    ui = convert_external_to_ui(cfg, external_ranges=external_ranges, as_urdf=True)
    return convert_ui_to_hardware(ui, hand_side=hand_side)


# 兼容旧名（原 usb.offline / finger_linkage）
def remap_joint_angles_for_downlink(
    joint_angles: Mapping[str, float],
    hand_side: Optional[str] = "left",
) -> Cfg:
    """UI → Hardware（旧名，等同 convert_ui_to_hardware）。"""
    return convert_ui_to_hardware(joint_angles, hand_side=hand_side)


def remap_motion_cfg_file_to_ui(
    cfg: Mapping[str, float],
    hand_side: Optional[str] = "left",
) -> Cfg:
    """动作文件 → UI。

    thumb_rota 保持内部完整范围；thumb_swing 继续兼容历史动作文件的
    [-1,1] 约定：左手 UI=1-file，右手 UI=1+file；旧动作保持物理姿态。
    """
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
    # 动作文件保存 UI/模型值，不套用左手 H7 专用的 0.5°/39.5°截断。
    for short in _FOUR_FINGER_SWING_SHORTS:
        urdf = SHORT_TO_URDF[short]
        if urdf in normalized:
            lo, hi = UI_RANGES[short]
            ui[urdf] = clamp(normalized[urdf], lo, hi)
    return ui


def remap_motion_cfg_ui_to_file(
    cfg: Mapping[str, float],
    hand_side: Optional[str] = "left",
) -> Cfg:
    """UI → 动作文件；thumb_swing 仍写兼容的 [-1,1] 旧坐标。"""
    hw = convert_ui_to_hardware(cfg, hand_side=hand_side)
    ui = normalize_cfg_keys(cfg, as_urdf=True)
    thumb = SHORT_TO_URDF["thumb_rota"]
    if thumb in ui:
        lo, hi = UI_RANGES["thumb_rota"]
        hw[thumb] = clamp(ui[thumb], lo, hi)
    thumb_swing = SHORT_TO_URDF["thumb_swing"]
    if thumb_swing in ui:
        lo, hi = UI_RANGES["thumb_swing"]
        value = 1.0 - clamp(ui[thumb_swing], lo, hi)
        hw[thumb_swing] = -value if normalize_hand_side(hand_side) == "right" else value
    for short in _FOUR_FINGER_SWING_SHORTS:
        urdf = SHORT_TO_URDF[short]
        if urdf in ui:
            lo, hi = UI_RANGES[short]
            hw[urdf] = clamp(ui[urdf], lo, hi)
    return {k: round(float(v), 4) for k, v in hw.items()}


def swing_urdf_to_hw(rad_urdf: float, hand_side: Optional[str] = "left") -> float:
    """四指 swing：同号下发；左手在 H7 边界截断到 [0.5°,39.5°]。"""
    vlo, vhi = UI_RANGES["index_swing"]
    value = clamp(float(rad_urdf), vlo, vhi)
    side = normalize_hand_side(hand_side)
    hlo, hhi = (HW_RANGES_RIGHT if side == "right" else HW_RANGES_LEFT)["index_swing"]
    return clamp(value, hlo, hhi)


def swing_hw_to_urdf(rad_hw: float, hand_side: Optional[str] = "left") -> float:
    """四指 swing 硬件遥测 → UI；左手按 H7 [0.5°,39.5°] 截断。"""
    vlo, vhi = UI_RANGES["index_swing"]
    side = normalize_hand_side(hand_side)
    hlo, hhi = (HW_RANGES_RIGHT if side == "right" else HW_RANGES_LEFT)["index_swing"]
    value = clamp(float(rad_hw), hlo, hhi)
    return clamp(value, vlo, vhi)


def thumb_swing_ui_to_hw(rad_ui: float, hand_side: Optional[str] = "left") -> float:
    """拇指 swing：左右手 UI/H7 同为 [0,2] rad，同值传递。"""
    vlo, vhi = UI_RANGES["thumb_swing"]
    v = clamp(float(rad_ui), vlo, vhi)
    hlo, hhi = HW_RANGES_LEFT["thumb_swing"]
    return clamp(v, hlo, hhi)


def thumb_swing_hw_to_ui(rad_hw: float, hand_side: Optional[str] = "left") -> float:
    """拇指 swing：左右手 H7/UI 同为 [0,2] rad，同值还原。"""
    vlo, vhi = UI_RANGES["thumb_swing"]
    hlo, hhi = HW_RANGES_LEFT["thumb_swing"]
    h = clamp(float(rad_hw), hlo, hhi)
    return clamp(h, vlo, vhi)


def thumb_rota_ui_to_hw(rad_ui: float, hand_side: Optional[str] = "left") -> float:
    """拇指 rota：左右手同号，直接钳到 H7 [0.5°,89.5°]。"""
    side = normalize_hand_side(hand_side)
    vlo, vhi = UI_RANGES["thumb_rota"]
    v = clamp(float(rad_ui), vlo, vhi)
    hw = HW_RANGES_RIGHT["thumb_rota"] if side == "right" else HW_RANGES_LEFT["thumb_rota"]
    return clamp(v, hw[0], hw[1])


def thumb_rota_hw_to_ui(rad_hw: float, hand_side: Optional[str] = "left") -> float:
    """拇指 rota：左右手 H7 遥测同号还原到 UI [0,1.57rad]。"""
    side = normalize_hand_side(hand_side)
    vlo, vhi = UI_RANGES["thumb_rota"]
    hw = HW_RANGES_RIGHT["thumb_rota"] if side == "right" else HW_RANGES_LEFT["thumb_rota"]
    value = clamp(float(rad_hw), hw[0], hw[1])
    return clamp(value, vlo, vhi)


def pinky_rota_ui_to_hw(rad_ui: float, hand_side: Optional[str] = "left") -> float:
    vlo, vhi = UI_RANGES["pinky_rota"]
    v = clamp(float(rad_ui), vlo, vhi)
    if normalize_hand_side(hand_side) == "right":
        v = -v
    hw = HW_RANGES_RIGHT["pinky_rota"] if normalize_hand_side(hand_side) == "right" else HW_RANGES_LEFT["pinky_rota"]
    return clamp(v, hw[0], hw[1])


def pinky_rota_hw_to_ui(rad_hw: float, hand_side: Optional[str] = "left") -> float:
    v = float(rad_hw)
    if normalize_hand_side(hand_side) == "right":
        v = -v
    vlo, vhi = UI_RANGES["pinky_rota"]
    return clamp(v, vlo, vhi)


# ── 查询 / 文档辅助 ──────────────────────────────────────────────────────

def describe_joint(short_or_urdf: str) -> Dict[str, object]:
    """返回单关节各空间区间说明。"""
    short = resolve_short_name(short_or_urdf)
    if short is None:
        raise KeyError(f"unknown joint: {short_or_urdf!r}")
    return {
        "short": short,
        "urdf": SHORT_TO_URDF[short],
        "external_default": EXTERNAL_RANGES_DEFAULT[short],
        "protocol": PROTOCOL_RANGES[short],
        "ui_rad": UI_RANGES[short],
        "ui_deg": (math.degrees(UI_RANGES[short][0]), math.degrees(UI_RANGES[short][1])),
        "hw_left_rad": HW_RANGES_LEFT[short],
        "hw_right_rad": HW_RANGES_RIGHT[short],
        "hw_left_deg": (
            math.degrees(HW_RANGES_LEFT[short][0]),
            math.degrees(HW_RANGES_LEFT[short][1]),
        ),
        "hw_right_deg": (
            math.degrees(HW_RANGES_RIGHT[short][0]),
            math.degrees(HW_RANGES_RIGHT[short][1]),
        ),
    }


def iter_joint_table() -> Iterable[Dict[str, object]]:
    for short in SHORT_TO_URDF:
        yield describe_joint(short)


__all__ = [
    "SHORT_TO_URDF",
    "URDF_TO_SHORT",
    "UI_RANGES",
    "PROTOCOL_RANGES",
    "EXTERNAL_RANGES_DEFAULT",
    "HW_RANGES_LEFT",
    "HW_RANGES_RIGHT",
    "clamp",
    "normalize_hand_side",
    "affine_map",
    "resolve_short_name",
    "to_urdf_key",
    "normalize_cfg_keys",
    "convert_external_to_protocol",
    "convert_protocol_to_ui",
    "convert_ui_to_protocol",
    "convert_protocol_to_external",
    "convert_external_to_ui",
    "convert_ui_to_external",
    "convert_ui_to_hardware",
    "convert_hardware_to_ui",
    "convert_external_to_hardware",
    "remap_joint_angles_for_downlink",
    "remap_motion_cfg_file_to_ui",
    "remap_motion_cfg_ui_to_file",
    "swing_urdf_to_hw",
    "swing_hw_to_urdf",
    "thumb_swing_ui_to_hw",
    "thumb_swing_hw_to_ui",
    "thumb_rota_ui_to_hw",
    "thumb_rota_hw_to_ui",
    "pinky_rota_ui_to_hw",
    "pinky_rota_hw_to_ui",
    "describe_joint",
    "iter_joint_table",
]
