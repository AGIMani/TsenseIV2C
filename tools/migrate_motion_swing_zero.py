#!/usr/bin/env python3
"""迁移旧动作文件的四指 swing 数值到新 URDF 零点约定。

背景（2026-08）：TsenseI URDF 四指 swing 关节零点做了重定义——
旧 q=0.4538rad(26°) 的姿态现在对应新 q=0；方向也反了。
UI↔HW 的下发协议（hand_angle_protocol）本身没变，但「同一个 UI 数值」
现在对应的可视姿态和以前不同了。为了让旧动作文件回放时仍是原来的
视觉姿态，需要对文件里 index/mid/ring/pinky 的 swing 数值做变换：

    newUI = UPPER - oldUI          (UPPER = radians(26) ≈ 0.4538)

文件里存的是 Hardware 空间数值（remap_motion_cfg_ui_to_file 的产物），
所以先转 UI、做上面的变换、再转回 HW。

用法：
    python tools/migrate_motion_swing_zero.py motion.txt [more.json ...]
    python tools/migrate_motion_swing_zero.py --dry-run motion.txt

默认原地覆写，并在同目录生成 <name>.bak 备份（已存在则不覆盖备份）。
--dry-run 只打印会改动的数值，不写文件。
"""
from __future__ import annotations

import argparse
import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "src"))

from protocol import convert_hardware_to_ui, convert_ui_to_hardware  # noqa: E402
from utils.motion_file import (  # noqa: E402
    build_motion_file_payload,
    load_motion_json,
    parse_motion_file_data,
    save_motion_json,
)

UPPER = math.radians(26.0)
_SWING_KEYS_SHORT = ("index_swing", "mid_swing", "ring_swing", "pinky_swing")


def _swing_urdf_keys(prefix: str) -> list[str]:
    return [f"{prefix}hand_{f.split('_')[0]}_swing_joint" for f in _SWING_KEYS_SHORT]


def migrate_frame(frame: dict, hand_side: str) -> tuple[dict, list[str]]:
    """迁移单帧（一个动作）里的四指 swing 数值，返回 (新帧, 改动说明)。"""
    changes: list[str] = []
    hw_side_prefix = "right_" if hand_side == "right" else "left_"
    keys = _swing_urdf_keys(hw_side_prefix)
    out = dict(frame)
    for key in keys:
        if key not in frame:
            continue
        old_hw = float(frame[key])
        old_ui = convert_hardware_to_ui({key: old_hw}, hand_side=hand_side)[key]
        new_ui = UPPER - old_ui
        new_hw = convert_ui_to_hardware({key: new_ui}, hand_side=hand_side)[key]
        out[key] = round(float(new_hw), 4)
        if abs(new_hw - old_hw) > 1e-6:
            changes.append(f"    {key}: HW {old_hw:.4f} -> {new_hw:.4f} (UI {old_ui:.4f} -> {new_ui:.4f})")
    return out, changes


def migrate_file(path: str, *, dry_run: bool = False) -> None:
    data = load_motion_json(path)
    parsed = parse_motion_file_data(data)
    if parsed is None:
        print(f"[跳过] 无法解析: {path}")
        return
    motions, hand_side = parsed

    total_changes = 0
    new_motions: dict[str, dict] = {}
    for name, frame in motions.items():
        new_frame, changes = migrate_frame(frame, hand_side)
        new_motions[name] = new_frame
        if changes:
            print(f"[{os.path.basename(path)}] 动作 「{name}」(hand_side={hand_side}):")
            for c in changes:
                print(c)
            total_changes += len(changes)

    if total_changes == 0:
        print(f"[{path}] 无需改动（未发现四指 swing 数据，或已是新约定）")
        return

    if dry_run:
        print(f"[{path}] --dry-run，共 {total_changes} 处改动，未写入")
        return

    backup = path + ".bak"
    if not os.path.exists(backup):
        save_motion_json(backup, data if isinstance(data, dict) else build_motion_file_payload(motions, hand_side))
        print(f"  已备份原文件 -> {backup}")

    payload = build_motion_file_payload(new_motions, hand_side)
    save_motion_json(path, payload)
    print(f"  已写入迁移后的文件 -> {path}（共 {total_changes} 处改动）")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("files", nargs="+", help="动作文件路径（.txt / .json）")
    ap.add_argument("--dry-run", action="store_true", help="只打印改动，不写文件")
    args = ap.parse_args()

    for path in args.files:
        if not os.path.isfile(path):
            print(f"[跳过] 文件不存在: {path}")
            continue
        migrate_file(path, dry_run=args.dry_run)


if __name__ == "__main__":
    main()
