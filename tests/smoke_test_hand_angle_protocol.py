"""smoke_test：External 原始数据 → protocol → 下发 Hardware（及可选 mm/deg 组包）。

用法：
  python tests/smoke_test_hand_angle_protocol.py
  pytest tests/smoke_test_hand_angle_protocol.py -q
"""
from __future__ import annotations

import math
import sys
from pathlib import Path
from typing import Dict, Mapping

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from protocol import (  # noqa: E402
    HW_RANGES_LEFT,
    HW_RANGES_RIGHT,
    SHORT_TO_URDF,
    convert_external_to_hardware,
    convert_external_to_protocol,
    convert_external_to_ui,
    convert_hardware_to_ui,
    convert_protocol_to_ui,
    convert_ui_to_display,
    convert_ui_to_hardware,
    remap_motion_cfg_file_to_ui,
    remap_motion_cfg_ui_to_file,
)

# 一组覆盖端点/中位的 External 默认区间样例（短名）
SAMPLE_EXTERNAL: Dict[str, float] = {
    # thumb_swing Ext [-1,1]：中位 / 满行程
    "thumb_swing": 0.0,
    "thumb_rota": 1.57,
    "thumb_mcp": 0.785,
    "thumb_pip": 0.0,
    # 四指 swing Ext [-0.348, 0.348]：中位 / 两端
    "index_swing": 0.0,
    "index_mcp": 1.0,
    "index_pip": 0.5,
    "index_dip": 0.25,
    "mid_swing": -0.348,
    "mid_mcp": 0.0,
    "mid_pip": 0.0,
    "mid_dip": 0.0,
    "ring_swing": 0.348,
    "ring_mcp": 1.57,
    "ring_pip": 1.57,
    "ring_dip": 1.57,
    "pinky_rota": 1.57,
    "pinky_swing": 0.0,
    "pinky_mcp": 0.8,
    "pinky_pip": 0.4,
    "pinky_dip": 0.2,
}

# 文档第三节典型样例（单独断言）
DOC_CASES = [
    # (short, ext, expect_ui_deg, expect_hw_left_deg, expect_hw_right_deg)
    ("thumb_swing", -1.0, math.degrees(2.0), math.degrees(2.0), math.degrees(2.0)),
    ("thumb_swing", 0.0, math.degrees(1.0), math.degrees(1.0), math.degrees(1.0)),
    ("thumb_swing", 1.0, 0.0, 0.0, 0.0),
    ("thumb_rota", 0.0, 0.0, 0.5, 0.5),
    ("thumb_rota", 1.57, math.degrees(1.57), 89.5, 89.5),
    ("index_swing", -0.348, 0.0, 0.5, 0.0),
    ("index_swing", 0.0, math.degrees(0.348), math.degrees(0.348), math.degrees(0.348)),
    ("index_swing", 0.348, math.degrees(0.696), 39.5, math.degrees(0.696)),
]


def _short(urdf_or_short: str) -> str:
    if urdf_or_short in SHORT_TO_URDF:
        return urdf_or_short
    for short, urdf in SHORT_TO_URDF.items():
        if urdf == urdf_or_short:
            return short
    return urdf_or_short


def _fmt_row(
    short: str,
    ext: float,
    proto: float,
    ui: float,
    hw_l: float,
    hw_r: float,
) -> str:
    return (
        f"{short:14s}  "
        f"Ext={ext:7.3f}  "
        f"Proto={proto:7.3f}  "
        f"UI={ui:7.4f}rad({math.degrees(ui):6.2f}°)  "
        f"HW_L={hw_l:7.4f}rad({math.degrees(hw_l):7.2f}°)  "
        f"HW_R={hw_r:7.4f}rad({math.degrees(hw_r):7.2f}°)"
    )


def run_pipeline(external: Mapping[str, float]) -> Dict[str, Dict[str, float]]:
    """External → Protocol → UI → Hardware(左/右)。"""
    proto = convert_external_to_protocol(external, as_urdf=True)
    ui = convert_protocol_to_ui(proto, as_urdf=True)
    # 与 convert_external_to_ui 等价校验
    ui_direct = convert_external_to_ui(external, as_urdf=True)
    assert set(ui) == set(ui_direct)
    for k in ui:
        assert abs(ui[k] - ui_direct[k]) < 1e-9

    hw_left = convert_external_to_hardware(external, hand_side="left")
    hw_right = convert_external_to_hardware(external, hand_side="right")
    return {
        "external": {SHORT_TO_URDF.get(k, k): float(v) for k, v in external.items()},
        "protocol": proto,
        "ui": ui,
        "hw_left": hw_left,
        "hw_right": hw_right,
    }


def print_report(external: Mapping[str, float] = SAMPLE_EXTERNAL) -> None:
    stages = run_pipeline(external)
    print("=" * 100)
    print("hand_angle_protocol smoke：External → Protocol → UI → Hardware（下发角）")
    print("=" * 100)
    print()
    print("【1】输入 External（默认 URDF 区间原始数据）")
    for k, v in sorted(external.items()):
        print(f"  {k:14s} = {v: .6f}")

    print()
    print("【2】逐关节流水线（含左右手下发）")
    print("-" * 100)
    for short in sorted(external.keys()):
        urdf = SHORT_TO_URDF[short]
        print(
            _fmt_row(
                short,
                float(external[short]),
                float(stages["protocol"][urdf]),
                float(stages["ui"][urdf]),
                float(stages["hw_left"][urdf]),
                float(stages["hw_right"][urdf]),
            )
        )

    print()
    print("【3】左手 Hardware（可直接传 UsbController.send_joint_angles）")
    for urdf, v in sorted(stages["hw_left"].items()):
        print(f"  {urdf:40s}  {v: .6f} rad  ({math.degrees(v):7.2f}°)")

    print()
    print("【4】右手 Hardware")
    for urdf, v in sorted(stages["hw_right"].items()):
        print(f"  {urdf:40s}  {v: .6f} rad  ({math.degrees(v):7.2f}°)")

    # 可选：组包成 14×mm + 7×deg（真实串口 payload 数值）
    try:
        from usb.offline import joint_angles_to_hand_positions

        linear, rotary = joint_angles_to_hand_positions(stages["hw_left"])
        print()
        print("【5】左手组包（offline.joint_angles_to_hand_positions → 14 mm + 7 deg）")
        print(f"  linear_mm[{len(linear)}]:  " + ", ".join(f"{x:7.3f}" for x in linear))
        print(f"  rotary_deg[{len(rotary)}]: " + ", ".join(f"{x:7.3f}" for x in rotary))
    except Exception as exc:  # noqa: BLE001
        print()
        print(f"【5】组包跳过（usb.offline 不可用: {exc}）")

    print()
    print("=" * 100)


# ── pytest ───────────────────────────────────────────────────────────────

def test_sample_external_pipeline_shapes():
    stages = run_pipeline(SAMPLE_EXTERNAL)
    n = len(SAMPLE_EXTERNAL)
    assert len(stages["protocol"]) == n
    assert len(stages["ui"]) == n
    assert len(stages["hw_left"]) == n
    assert len(stages["hw_right"]) == n


@pytest.mark.parametrize(
    "short,ext,ui_deg,hw_l_deg,hw_r_deg",
    DOC_CASES,
    ids=[f"{s}@{e}" for s, e, *_ in DOC_CASES],
)
def test_doc_examples(short, ext, ui_deg, hw_l_deg, hw_r_deg):
    cfg = {short: ext}
    ui = convert_external_to_ui(cfg)
    hw_l = convert_external_to_hardware(cfg, hand_side="left")
    hw_r = convert_external_to_hardware(cfg, hand_side="right")
    urdf = SHORT_TO_URDF[short]
    assert abs(math.degrees(ui[urdf]) - ui_deg) < 0.05
    assert abs(math.degrees(hw_l[urdf]) - hw_l_deg) < 0.05
    assert abs(math.degrees(hw_r[urdf]) - hw_r_deg) < 0.05


def test_sample_key_expectations():
    """SAMPLE_EXTERNAL 关键关节期望。"""
    stages = run_pipeline(SAMPLE_EXTERNAL)
    ui, hw_l, hw_r = stages["ui"], stages["hw_left"], stages["hw_right"]

    # thumb_swing Ext 0 → UI 1 → 左右H7中位1rad
    assert ui[SHORT_TO_URDF["thumb_swing"]] == pytest.approx(1.0)
    assert hw_l[SHORT_TO_URDF["thumb_swing"]] == pytest.approx(1.0)
    assert hw_r[SHORT_TO_URDF["thumb_swing"]] == pytest.approx(1.0)

    # thumb_swing Ext -1 → UI 2；左右H7均为2
    stages_neg = run_pipeline({"thumb_swing": -1.0})
    assert stages_neg["ui"][SHORT_TO_URDF["thumb_swing"]] == pytest.approx(2.0)
    assert stages_neg["hw_left"][SHORT_TO_URDF["thumb_swing"]] == pytest.approx(2.0)

    # thumb_rota Ext 1.57 → UI 1.57rad；左右手 H7 均直接钳到 +89.5°
    assert ui[SHORT_TO_URDF["thumb_rota"]] == pytest.approx(1.57)
    assert math.degrees(hw_l[SHORT_TO_URDF["thumb_rota"]]) == pytest.approx(89.5)
    assert math.degrees(hw_r[SHORT_TO_URDF["thumb_rota"]]) == pytest.approx(89.5)

    # thumb_swing Ext -1 → 右H7 2rad
    assert stages_neg["hw_right"][SHORT_TO_URDF["thumb_swing"]] == pytest.approx(2.0)

    # index_swing Ext 0 → UI 1.348rad；左右手均保持正值
    swing_mid_deg = math.degrees(0.348)
    assert abs(math.degrees(ui[SHORT_TO_URDF["index_swing"]]) - swing_mid_deg) < 0.05
    assert abs(math.degrees(hw_l[SHORT_TO_URDF["index_swing"]]) - swing_mid_deg) < 0.05
    assert abs(math.degrees(hw_r[SHORT_TO_URDF["index_swing"]]) - swing_mid_deg) < 0.05

    # mid_swing Ext -0.348 → UI 0；左H7截到0.5°，右H7仍为0
    assert abs(ui[SHORT_TO_URDF["mid_swing"]]) < 1e-6
    assert math.degrees(hw_l[SHORT_TO_URDF["mid_swing"]]) == pytest.approx(0.5)

    # ring_swing Ext +0.348 → UI 0.696rad；左右手均保持正值
    swing_max_deg = math.degrees(0.696)
    assert abs(math.degrees(ui[SHORT_TO_URDF["ring_swing"]]) - swing_max_deg) < 0.05
    assert math.degrees(hw_l[SHORT_TO_URDF["ring_swing"]]) == pytest.approx(39.5)
    assert abs(math.degrees(hw_r[SHORT_TO_URDF["ring_swing"]]) - swing_max_deg) < 0.05


@pytest.mark.parametrize(
    "side,ui_value,expected_hw",
    (
        ("left", -0.2, 0.0),
        ("left", 0.0, 0.0),
        ("left", 1.0, 1.0),
        ("left", 2.0, 2.0),
        ("left", 2.2, 2.0),
        ("right", -0.2, 0.0),
        ("right", 0.0, 0.0),
        ("right", 1.0, 1.0),
        ("right", 2.0, 2.0),
        ("right", 2.2, 2.0),
    ),
)
def test_thumb_swing_h7_range_and_hand_mapping(
    side: str, ui_value: float, expected_hw: float,
) -> None:
    """两手H7范围均为[0,2]rad；左右手均与UI同值。"""
    urdf = SHORT_TO_URDF["thumb_swing"]
    hw = convert_ui_to_hardware({urdf: ui_value}, hand_side=side)
    assert hw[urdf] == pytest.approx(expected_hw)
    ranges = HW_RANGES_LEFT if side == "left" else HW_RANGES_RIGHT
    assert ranges["thumb_swing"] == pytest.approx((0.0, 2.0))


@pytest.mark.parametrize("ui_value", (0.0, 0.4, 1.0, 1.6, 2.0))
def test_left_thumb_swing_display_downlink_and_telemetry_contract(ui_value: float) -> None:
    """Form2/主窗显示保留UI姿态；左手H7同值下发并可逆还原。"""
    urdf = SHORT_TO_URDF["thumb_swing"]
    display = convert_ui_to_display({urdf: ui_value}, hand_side="left")
    hardware = convert_ui_to_hardware({urdf: ui_value}, hand_side="left")
    restored = convert_hardware_to_ui(hardware, hand_side="left")

    assert display[urdf] == pytest.approx(ui_value)
    assert hardware[urdf] == pytest.approx(ui_value)
    assert restored[urdf] == pytest.approx(ui_value)


@pytest.mark.parametrize("side", ("left", "right"))
@pytest.mark.parametrize("ui_value", (0.0, 0.4, 1.0, 1.6, 2.0))
def test_thumb_swing_h7_telemetry_and_motion_file_round_trip(
    side: str, ui_value: float,
) -> None:
    urdf = SHORT_TO_URDF["thumb_swing"]
    hw = convert_ui_to_hardware({urdf: ui_value}, hand_side=side)
    restored = convert_hardware_to_ui(hw, hand_side=side)
    assert restored[urdf] == pytest.approx(ui_value)
    saved = remap_motion_cfg_ui_to_file({urdf: ui_value}, hand_side=side)
    expected_file_value = 1.0 - ui_value if side == "left" else ui_value - 1.0
    assert saved[urdf] == pytest.approx(expected_file_value, abs=5e-5)
    loaded = remap_motion_cfg_file_to_ui(saved, hand_side=side)
    assert loaded[urdf] == pytest.approx(ui_value, abs=5e-5)


@pytest.mark.parametrize("short", ("index_swing", "mid_swing", "ring_swing", "pinky_swing"))
@pytest.mark.parametrize("side", ("left", "right"))
def test_four_finger_swing_wire_sign_is_positive_for_both_hands(short: str, side: str):
    """四指 swing：左右手均正值下发。"""
    from protocol import convert_ui_to_hardware
    from usb.offline import ROTARY_JOINT_SLOTS, joint_angles_to_hand_positions

    urdf = SHORT_TO_URDF[short]
    ui_value = math.radians(10.0)
    hw = convert_ui_to_hardware({urdf: ui_value}, hand_side=side)
    expected_rad = ui_value
    assert hw[urdf] == pytest.approx(expected_rad)
    _linear, rotary = joint_angles_to_hand_positions(hw)
    expected_deg = 10.0
    assert rotary[ROTARY_JOINT_SLOTS[urdf]] == pytest.approx(expected_deg)


@pytest.mark.parametrize("short", ("index_swing", "mid_swing", "ring_swing", "pinky_swing"))
@pytest.mark.parametrize(
    "ui_deg,expected_hw_deg",
    ((0.0, 0.5), (0.25, 0.5), (0.5, 0.5), (10.0, 10.0), (39.5, 39.5), (39.88, 39.5)),
)
def test_left_four_finger_swing_h7_clamp(short: str, ui_deg: float, expected_hw_deg: float):
    urdf = SHORT_TO_URDF[short]
    hw = convert_ui_to_hardware({urdf: math.radians(ui_deg)}, hand_side="left")
    assert math.degrees(hw[urdf]) == pytest.approx(expected_hw_deg)


@pytest.mark.parametrize("short", ("index_swing", "mid_swing", "ring_swing", "pinky_swing"))
@pytest.mark.parametrize("ui_value", (0.0, 0.348, 0.696))
def test_four_finger_motion_file_keeps_full_ui_range(short: str, ui_value: float):
    urdf = SHORT_TO_URDF[short]
    saved = remap_motion_cfg_ui_to_file({urdf: ui_value}, hand_side="left")
    loaded = remap_motion_cfg_file_to_ui(saved, hand_side="left")
    assert saved[urdf] == pytest.approx(ui_value, abs=5e-5)
    assert loaded[urdf] == pytest.approx(ui_value, abs=5e-5)


@pytest.mark.parametrize("side", ("left", "right"))
def test_four_finger_swing_telemetry_round_trip(side: str):
    """左右手正值硬件遥测均还原为正UI角。"""
    from protocol import swing_hw_to_urdf, swing_urdf_to_hw

    value = math.radians(17.0)
    hw = value
    assert swing_urdf_to_hw(value, side) == pytest.approx(hw)
    assert swing_hw_to_urdf(hw, side) == pytest.approx(value)


@pytest.mark.parametrize("side", ("left", "right"))
@pytest.mark.parametrize(
    "ui_deg,expected_hw_deg",
    ((0.0, 0.5), (0.25, 0.5), (0.5, 0.5), (40.0, 40.0), (89.5, 89.5), (90.0, 89.5)),
)
def test_thumb_rota_h7_direct_clamp(side: str, ui_deg: float, expected_hw_deg: float):
    """thumb_rota 不再放大，仅在 H7 边界钳到 [0.5°,89.5°]。"""
    urdf = SHORT_TO_URDF["thumb_rota"]
    hw = convert_ui_to_hardware({urdf: math.radians(ui_deg)}, hand_side=side)
    assert math.degrees(hw[urdf]) == pytest.approx(expected_hw_deg)


@pytest.mark.parametrize("side", ("left", "right"))
def test_thumb_rota_h7_telemetry_direct_range(side: str):
    """H7 遥测与 UI 同号同尺度，并按 H7 范围钳位。"""
    urdf = SHORT_TO_URDF["thumb_rota"]
    for hw_deg, expected_ui_deg in ((0.0, 0.5), (0.5, 0.5), (40.0, 40.0), (89.5, 89.5), (90.0, 89.5)):
        ui = convert_hardware_to_ui({urdf: math.radians(hw_deg)}, hand_side=side)
        assert math.degrees(ui[urdf]) == pytest.approx(expected_ui_deg)

    ranges = HW_RANGES_LEFT if side == "left" else HW_RANGES_RIGHT
    assert tuple(math.degrees(value) for value in ranges["thumb_rota"]) == pytest.approx((0.5, 89.5))


@pytest.mark.parametrize("side", ("left", "right"))
def test_thumb_rota_motion_file_keeps_full_ui_range(side: str):
    """动作文件保留内部完整范围，不套用 H7 的 0.5°/89.5°钳位。"""
    urdf = SHORT_TO_URDF["thumb_rota"]
    value = 1.57
    saved = remap_motion_cfg_ui_to_file({urdf: value}, hand_side=side)
    assert saved[urdf] == pytest.approx(1.57)
    loaded = remap_motion_cfg_file_to_ui(saved, hand_side=side)
    assert loaded[urdf] == pytest.approx(1.57)


def test_thumb_rota_downlink_guard_uses_unscaled_rotation_velocity():
    """去除角度增益后，安全网和 Ruckig 使用相同的旋转速度约束。"""
    from utils.downlink_guard import _max_vel_for
    from utils.ruckig_interpolator import _DEFAULT_CONSTRAINTS

    base = _DEFAULT_CONSTRAINTS["rotation"][0]
    assert _max_vel_for(SHORT_TO_URDF["thumb_rota"]) == pytest.approx(base)
    assert _max_vel_for(SHORT_TO_URDF["pinky_rota"]) == pytest.approx(base)


def test_hw_can_pack_for_downlink():
    stages = run_pipeline(SAMPLE_EXTERNAL)
    from usb.offline import joint_angles_to_hand_positions

    linear, rotary = joint_angles_to_hand_positions(stages["hw_left"])
    assert len(linear) == 14
    assert len(rotary) == 7
    assert all(math.isfinite(x) for x in linear + rotary)


if __name__ == "__main__":
    print_report(SAMPLE_EXTERNAL)
    # 再跑一遍断言，失败则非零退出
    test_sample_external_pipeline_shapes()
    for case in DOC_CASES:
        test_doc_examples(*case)
    test_sample_key_expectations()
    test_hw_can_pack_for_downlink()
    print("smoke assertions: PASS")
