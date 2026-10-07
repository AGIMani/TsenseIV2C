"""UI → URDF 显示协议。

左右手显示保持 UI 数值，右手只更换关节名前缀；模型本身负责镜像。
thumb_swing 的 UI / URDF / H7 统一为 [0,2] rad，中位为 1。
新坐标 q_new = 1-q_old；左手轴 +Z、origin yaw=-1.29927，右手轴 -Z、yaw=+1.29927。
URDF 与碰撞 MJCF 同步重定义零位，旧 +1 / 0 / -1 姿态分别对应新 0 / 1 / 2。
四指 swing 显示范围 [0,0.696] rad；硬件夹紧与右手 pinky_rota 变号见 hand_angle_protocol。
"""
from __future__ import annotations

import math
from typing import Dict, Mapping, Optional, Tuple

from .hand_angle_protocol import (
    SHORT_TO_URDF,
    UI_RANGES,
    URDF_TO_SHORT,
    clamp,
    normalize_cfg_keys,
    normalize_hand_side,
    resolve_short_name,
)

Range = Tuple[float, float]
Cfg = Dict[str, float]

_DEG_SWING_MAX = 0.696
_DEG_50 = math.radians(50.0)
_FLEX_MAX = 1.57
_THUMB_SWING_MAX = 2.0

FINGER_SWING_DISPLAY_MIN = 0.0
FINGER_SWING_DISPLAY_MAX = _DEG_SWING_MAX

_FINGER_SWING_SHORT = frozenset(
    {"index_swing", "mid_swing", "ring_swing", "pinky_swing"}
)

# 滑块 / 内存 UI 显示限位（左右手滑块相同；与 UI_RANGES 一致）
DISPLAY_RANGES: Dict[str, Range] = dict(UI_RANGES)

# 左手 URDF 查看器数值空间（键 left_hand_*，与 UI 同号同区间）
DISPLAY_RANGES_LEFT: Dict[str, Range] = {
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

# 右手 URDF 查看器数值空间（键 right_hand_*）。
# 区间与左手一致，方向差由右手 URDF 的镜像几何 / 取反 axis 承担。
DISPLAY_RANGES_RIGHT: Dict[str, Range] = {
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


def is_four_finger_swing_short(short: str) -> bool:
    return short in _FINGER_SWING_SHORT


def is_four_finger_swing_joint(name: str) -> bool:
    """任意 left_/right_hand_*_swing_joint 键（排除拇指）。"""
    short = resolve_short_name(name)
    return short is not None and short in _FINGER_SWING_SHORT


def _side_urdf_key(short: str, hand_side: str) -> str:
    left = SHORT_TO_URDF[short]
    if hand_side == "right":
        return "right_hand_" + left[len("left_hand_") :]
    return left


def convert_ui_to_display(
    cfg: Mapping[str, float],
    hand_side: Optional[str] = "left",
    *,
    as_side_keys: bool = True,
) -> Cfg:
    """UI(rad) → URDF 显示姿态。

    - 左手：clamp 到 ``DISPLAY_RANGES_LEFT``，键 ``left_hand_*``
    - 右手：clamp 到 ``DISPLAY_RANGES_RIGHT``，键 ``right_hand_*``；数值不变
      （右手 URDF 已是镜像几何 + 取反 axis，见模块头部对照表）
    - 显示与 UI 全程同号，不做下发那套取负

    ``as_side_keys=False`` 时仍输出 ``left_hand_*``（仅改数值，供调试）。
    """
    side = normalize_hand_side(hand_side)
    ui = normalize_cfg_keys(cfg, as_urdf=True)
    ranges = DISPLAY_RANGES_RIGHT if side == "right" else DISPLAY_RANGES_LEFT
    out: Cfg = {}
    for urdf, value in ui.items():
        short = URDF_TO_SHORT.get(urdf)
        if short is None or short not in ranges:
            key = str(urdf)
            if as_side_keys and side == "right" and key.startswith("left_hand_"):
                key = "right_hand_" + key[len("left_hand_") :]
            out[key] = float(value)
            continue
        lo, hi = ranges[short]
        key = _side_urdf_key(short, side) if as_side_keys else SHORT_TO_URDF[short]
        out[key] = clamp(float(value), lo, hi)
    return out


def describe_display_joint(short_or_urdf: str) -> Dict[str, object]:
    """单关节显示区间说明（对照下发请看 hand_angle_protocol.describe_joint）。"""
    short = resolve_short_name(short_or_urdf)
    if short is None:
        raise KeyError(f"unknown joint: {short_or_urdf!r}")
    dlo, dhi = DISPLAY_RANGES[short]
    return {
        "short": short,
        "urdf_left": SHORT_TO_URDF[short],
        "urdf_right": "right_hand_" + SHORT_TO_URDF[short][len("left_hand_") :],
        "slider_ui_rad": DISPLAY_RANGES[short],
        "slider_ui_deg": (math.degrees(dlo), math.degrees(dhi)),
        "display_left_rad": DISPLAY_RANGES_LEFT[short],
        "display_right_rad": DISPLAY_RANGES_RIGHT[short],
        "note": "显示左右手同值同号；右手方向差由 URDF 镜像几何承担",
    }


__all__ = [
    "DISPLAY_RANGES",
    "DISPLAY_RANGES_LEFT",
    "DISPLAY_RANGES_RIGHT",
    "FINGER_SWING_DISPLAY_MIN",
    "FINGER_SWING_DISPLAY_MAX",
    "is_four_finger_swing_short",
    "is_four_finger_swing_joint",
    "convert_ui_to_display",
    "describe_display_joint",
]
