#!/usr/bin/env python3
"""核对左右手旋转电机：UI→HW 符号与下发行程(deg)。"""
from __future__ import annotations

import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from protocol.hand_angle_protocol import (  # noqa: E402
    HW_RANGES_LEFT,
    HW_RANGES_RIGHT,
    SHORT_TO_URDF,
    UI_RANGES,
    convert_ui_to_hardware,
)
from usb.offline import JOINT_ORDER, ROTARY_JOINT_SLOTS, joint_angles_to_hand_positions  # noqa: E402

RAD2DEG = 180.0 / math.pi
ROTARY = [
    "thumb_rota",
    "thumb_swing",
    "index_swing",
    "mid_swing",
    "ring_swing",
    "pinky_swing",
    "pinky_rota",
]
EXPECTED_UI_TO_HW_SIGN = {
    "left": {
        "thumb_rota": 1,
        "thumb_swing": 1,
        "index_swing": 1,
        "mid_swing": 1,
        "ring_swing": 1,
        "pinky_swing": 1,
        "pinky_rota": 1,
    },
    "right": {
        "thumb_rota": 1,
        "thumb_swing": 1,
        "index_swing": 1,
        "mid_swing": 1,
        "ring_swing": 1,
        "pinky_swing": 1,
        "pinky_rota": -1,
    },
}


def main() -> None:
    print("=== UI ranges ===")
    for s in ROTARY:
        lo, hi = UI_RANGES[s]
        print(
            f"  {s:12s} UI [{lo*RAD2DEG:+7.1f}, {hi*RAD2DEG:+7.1f}] deg"
        )

    print("\n=== Declared HW ranges ===")
    for side, table in (("LEFT", HW_RANGES_LEFT), ("RIGHT", HW_RANGES_RIGHT)):
        print(f"-- {side} --")
        for s in ROTARY:
            lo, hi = table[s]
            print(
                f"  {s:12s} HW [{lo*RAD2DEG:+7.1f}, {hi*RAD2DEG:+7.1f}] deg"
            )

    print("\n=== UI endpoints -> HW -> wire deg ===")
    print(
        "  (wire = Hardware_rad * 180/pi, packed into rotary[slot] float32)"
    )
    for side in ("left", "right"):
        print(f"\n-- hand_side={side} --")
        print(
            f"  {'joint':12s} {'pt':3s} {'UI_deg':>8s} {'HW_deg':>8s} "
            f"{'wire_deg':>9s} {'slot':>5s} {'UI->HW':>6s}"
        )
        for s in ROTARY:
            ulo, uhi = UI_RANGES[s]
            mid = 0.5 * (ulo + uhi)
            name = SHORT_TO_URDF[s]
            slot = ROTARY_JOINT_SLOTS[name]
            for label, ui in (("min", ulo), ("mid", mid), ("max", uhi)):
                cfg = {n: 0.0 for n in JOINT_ORDER}
                cfg[name] = ui
                hw = convert_ui_to_hardware(cfg, hand_side=side)
                hw_v = float(hw[name])
                _lin, rot = joint_angles_to_hand_positions(hw)
                wire = float(rot[slot])
                expected = EXPECTED_UI_TO_HW_SIGN[side][s] * ui
                if s == "thumb_swing":
                    expected = ui
                elif s == "thumb_rota":
                    hw_range = HW_RANGES_LEFT[s] if side == "left" else HW_RANGES_RIGHT[s]
                    expected = min(max(ui, hw_range[0]), hw_range[1])
                elif side == "left" and s in (
                    "index_swing", "mid_swing", "ring_swing", "pinky_swing"
                ):
                    hw_range = HW_RANGES_LEFT[s]
                    expected = min(max(ui, hw_range[0]), hw_range[1])
                if abs(hw_v - expected) > 1e-9:
                    raise AssertionError(
                        f"{side} {s}: UI {ui:+.9f} -> HW {hw_v:+.9f}, "
                        f"expected {expected:+.9f}"
                    )
                if abs(wire - hw_v * RAD2DEG) > 1e-5:
                    raise AssertionError(
                        f"{side} {s}: wire {wire:+.6f} deg != HW {hw_v*RAD2DEG:+.6f} deg"
                    )
                if s == "thumb_swing":
                    rule = "same"
                elif abs(hw_v - ui) >= 1e-9 and (
                    s == "thumb_rota"
                    or (side == "left" and s in (
                        "index_swing", "mid_swing", "ring_swing", "pinky_swing"
                    ))
                ):
                    rule = "CLAMP"
                elif abs(ui) < 1e-12:
                    rule = "0"
                elif abs(hw_v - ui) < 1e-9:
                    rule = "same"
                elif abs(hw_v + ui) < 1e-9:
                    rule = "NEG"
                else:
                    rule = "other"
                print(
                    f"  {s:12s} {label:3s} {ui*RAD2DEG:+8.2f} {hw_v*RAD2DEG:+8.2f} "
                    f"{wire:+9.2f} R{slot:02d}   {rule}"
                )

    print("\n=== Slot map (USB rotary) ===")
    for name, slot in sorted(ROTARY_JOINT_SLOTS.items(), key=lambda x: x[1]):
        short = name.removeprefix("left_hand_").removesuffix("_joint")
        print(f"  R{slot:02d}  {short}")


if __name__ == "__main__":
    main()
