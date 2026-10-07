#!/usr/bin/env python3
"""诊断：哪些场景会产生负关节角；对比 生产钳位 / 严格逆解 / Ruckig 过冲。

用法:
  python tools/diagnose_negative_angles.py [logs/data_track_....log]
"""
from __future__ import annotations

import json
import math
import sys
from collections import Counter, defaultdict
from pathlib import Path
from typing import Any, Dict, List, Optional

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from kinematics.finger_linkage import (  # noqa: E402
    MOTOR_F1,
    MOTOR_F3,
    clamp_joint_rad_for_motor,
    create_default_hand_stroke_mapper,
    joint_rad_to_phi_deg,
)
from utils.ruckig_interpolator import RuckigJointInterpolator  # noqa: E402

_FLEX = ("index_MCP", "index_PIP", "index_DIP", "mid_MCP", "mid_PIP", "mid_DIP",
         "ring_MCP", "ring_PIP", "ring_DIP", "pinky_MCP", "pinky_PIP", "pinky_DIP",
         "thumb_MCP", "thumb_PIP")


def _deg(r: float) -> float:
    return math.degrees(r)


def analyze_log(path: Path) -> None:
    print("=" * 72)
    print(f"[1] 日志负角统计: {path.name}")
    print("=" * 72)
    by_joint: Dict[str, List[float]] = defaultdict(list)
    by_scenario = Counter()
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line or line.startswith("#"):
            continue
        rec = json.loads(line)
        kind = rec.get("kind")
        angles = rec.get("angles_by_name") or {}
        for short, val in angles.items():
            v = float(val)
            if v >= 0:
                continue
            by_joint[short].append(v)
            d = _deg(v)
            if short == "thumb_swing":
                scen = "A_拇指摆动(合法负角, URDF±57°)"
            elif short.endswith("_rota"):
                scen = "B_旋转关节近零噪声"
            elif short in _FLEX and d > -0.1:
                scen = "C_屈曲近零微小负角(<0.1°, 肉眼≈0)"
            elif short in _FLEX and d > -2:
                scen = "D_屈曲小负角(0.1~2°, retarget/Ruckig越界)"
            elif short in _FLEX:
                scen = "E_屈曲明显负角(>2°)"
            else:
                scen = "F_其他"
            by_scenario[(scen, kind, short)] += 1

    print("\n按场景计数 (场景, kind, 关节) → 次数:")
    for (scen, kind, short), n in sorted(by_scenario.items(), key=lambda x: -x[1]):
        print(f"  {n:4d}  {scen:40s}  kind={kind:6s}  {short}")

    print("\n各负角关节极值 (°):")
    for short, vals in sorted(by_joint.items()):
        degs = [_deg(v) for v in vals]
        print(f"  {short:14s}  n={len(vals):4d}  min={min(degs):8.4f}°  max={max(degs):8.4f}°")


def test_production_clamp() -> None:
    print("\n" + "=" * 72)
    print("[2] 生产路径 soft-clamp：负屈曲角 → 钳到 0 再逆解")
    print("=" * 72)
    mapper = create_default_hand_stroke_mapper()
    p = mapper.four_finger.params
    cases = [
        ("中指DIP 微小负角", MOTOR_F1, math.radians(-0.05)),
        ("中指DIP -0.7°", MOTOR_F1, math.radians(-0.7)),
        ("中指DIP -5°", MOTOR_F1, math.radians(-5.0)),
        ("小指MCP -0.7°", MOTOR_F3, math.radians(-0.7)),
        ("正角对照 +10°", MOTOR_F1, math.radians(10.0)),
    ]
    print(f"  {'场景':22s}  {'输入°':>10s}  {'钳位后°':>10s}  {'phi输入°':>10s}  {'说明'}")
    for name, motor, raw in cases:
        clamped = clamp_joint_rad_for_motor(raw, motor, p)
        print(
            f"  {name:22s}  {_deg(raw):10.4f}  {_deg(clamped):10.4f}  "
            f"{joint_rad_to_phi_deg(raw):10.4f}  "
            f"{'已钳到≥0' if clamped >= 0 and raw < 0 else '保持'}"
        )
    print("  → 下位机行程用钳位后角度；data_track 严格探测用原始角 → 日志报 IK 失败。")


def test_ruckig_overshoot() -> None:
    print("\n" + "=" * 72)
    print("[3] Ruckig 近零过冲测试（无位置限位，仅 vel/acc/jerk）")
    print("=" * 72)
    dip = "left_hand_mid_DIP_joint"
    swing = "left_hand_thumb_swing_joint"

    def _home() -> dict:
        return {j: 0.0 for j in (
            "right_hand_thumb_rota_joint", "right_hand_thumb_swing_joint",
            "right_hand_thumb_MCP_joint", "right_hand_thumb_PIP_joint",
            "right_hand_index_swing_joint", "right_hand_index_MCP_joint",
            "right_hand_index_PIP_joint", "right_hand_index_DIP_joint",
            "right_hand_mid_swing_joint", "right_hand_mid_MCP_joint",
            "right_hand_mid_PIP_joint", "right_hand_mid_DIP_joint",
            "right_hand_ring_swing_joint", "right_hand_ring_MCP_joint",
            "right_hand_ring_PIP_joint", "right_hand_ring_DIP_joint",
            "right_hand_pinky_rota_joint", "right_hand_pinky_swing_joint",
            "right_hand_pinky_MCP_joint", "right_hand_pinky_PIP_joint",
            "right_hand_pinky_DIP_joint",
        )}

    def _run(label: str, joint: str, start: float, targets: List[float]) -> None:
        interp = RuckigJointInterpolator()
        short = joint.replace("left_hand_", "").replace("_joint", "")
        rname = f"right_hand_{short}_joint"
        seed = _home()
        seed[rname] = start
        # 停在 start：start==end
        interp.seed_ramp(seed, seed, duration_s=0.0)
        for _ in range(5):
            interp.sample()
        neg_samples = []
        for tgt in targets:
            cfg = dict(seed)
            cfg[rname] = tgt
            interp.push(cfg)
            for i in range(500):  # 1.0s @ 500Hz
                s = interp.sample() or {}
                v = float(s.get(rname, 0.0))
                if v < -1e-6:
                    neg_samples.append((_deg(v), i * 0.002, _deg(tgt)))
        if neg_samples:
            worst = min(neg_samples, key=lambda x: x[0])
            print(
                f"  {label}: 出现负角 {len(neg_samples)} 次, "
                f"最负={worst[0]:.4f}° @t={worst[1]:.3f}s (目标={worst[2]:.4f}°)"
            )
        else:
            print(f"  {label}: 未出现负角")

    # 从弯曲回到接近 0：可能轻微过冲到负
    _run("DIP 30°→0°", dip, math.radians(30), [0.0])
    _run("DIP 5°→0°", dip, math.radians(5), [0.0])
    _run("DIP 1°→0°", dip, math.radians(1), [0.0])
    _run("DIP 0.05°→0°", dip, math.radians(0.05), [0.0])
    # retarget 目标本身已是微小负角
    _run("DIP 目标=-0.05°", dip, 0.0, [math.radians(-0.05)])
    _run("DIP 目标=-0.7°", dip, math.radians(2), [math.radians(-0.7)])
    # 拇指摆动合法负区间
    _run("thumb_swing 中位→旧-15°姿态", swing, 1.0, [1.0 + math.radians(15)])
    _run("thumb_swing +10→-30°", swing, math.radians(10), [math.radians(-30)])


def test_display_vs_log_gap(path: Optional[Path]) -> None:
    print("\n" + "=" * 72)
    print("[4] 为何模型看起来一直是正值？")
    print("=" * 72)
    print("""
  关键结论：
  ① URDF 显示与 USB 共用 Ruckig peek/sample，理论上同一帧数值相同；
     但视觉上 |θ|<1° 的负屈曲与 0° 几乎无差别（本日志 mid_DIP 最负仅约 -0.69°）。
  ② data_track 的 raw 记录的是 retarget 推入目标，不是当前显示态：
     目标已是微小负角时，Ruckig 当前态可能仍在正角 → 你看见正、日志写负。
  ③ 真正下发行程会 soft-clamp 负屈曲到 0，电机不会按负角解算；
     日志 ik_fail 用严格探测，故意暴露这些越界目标。
  ④ thumb_swing URDF/UI/H7 范围为 [0,2] rad，中位1；旧负角对应新坐标大于1。
""")
    if path is None or not path.is_file():
        return
    # raw vs 同 seq 附近：对比 raw 负 DIP 与同文件 ruckig
    raw_neg_dip = 0
    ruckig_still_pos_near = 0  # informational only within file
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line or line.startswith("#"):
            continue
        rec = json.loads(line)
        dip = float((rec.get("angles_by_name") or {}).get("mid_DIP", 0))
        if rec.get("kind") == "raw" and dip < 0:
            raw_neg_dip += 1
            if abs(_deg(dip)) < 0.1:
                ruckig_still_pos_near += 1  # reuse counter as tiny
    print(f"  本日志 raw 且 mid_DIP<0: {raw_neg_dip} 帧")
    print(f"  其中 |mid_DIP|<0.1°（视觉≈零位）: {ruckig_still_pos_near} 帧")


def test_synthetic_pipeline() -> None:
    print("\n" + "=" * 72)
    print("[5] 合成场景：负角如何进入 raw / ruckig / 行程")
    print("=" * 72)
    mapper = create_default_hand_stroke_mapper()
    p = mapper.four_finger.params
    scenarios = [
        ("张开手近零噪声", {"mid_DIP": math.radians(-0.05), "mid_PIP": 0.0, "mid_MCP": 0.0}),
        ("retarget 微越界", {"mid_DIP": math.radians(-0.7), "mid_PIP": 0.2, "mid_MCP": 0.3}),
        ("拇指摆向负侧", {"thumb_swing": 1.0 + math.radians(15), "thumb_MCP": 0.2, "thumb_PIP": 0.1}),
        ("小指MCP微负", {"pinky_MCP": math.radians(-0.5), "pinky_PIP": 0.1, "pinky_DIP": 0.1}),
    ]
    print(f"  {'场景':16s}  {'关键负角':18s}  {'生产钳位后°':>12s}  {'严格phi°':>10s}  {'严格IK'}")
    for name, angs in scenarios:
        for short, rad in angs.items():
            if rad >= 0 and "swing" not in short:
                continue
            if "DIP" in short:
                motor = MOTOR_F1
            elif "PIP" in short and "thumb" not in short:
                motor = 1  # F2
                from kinematics.finger_linkage import MOTOR_F2
                motor = MOTOR_F2
            elif "MCP" in short and "thumb" not in short:
                motor = MOTOR_F3
            else:
                # thumb_swing: not linkage flex
                print(
                    f"  {name:16s}  {short}={_deg(rad):+7.3f}°  "
                    f"{'—':>12s}  {'—':>10s}  合法(非连杆屈曲)"
                )
                continue
            clamped = clamp_joint_rad_for_motor(rad, motor, p)
            phi = joint_rad_to_phi_deg(rad)
            ok = "FAIL(负phi)" if phi < -1e-3 else "OK"
            print(
                f"  {name:16s}  {short}={_deg(rad):+7.3f}°  "
                f"{_deg(clamped):12.4f}  {phi:10.4f}  {ok}"
            )


def main() -> int:
    log = Path(sys.argv[1]) if len(sys.argv) > 1 else (
        ROOT / "logs" / "data_track_20260730_153204.log"
    )
    if log.is_file():
        analyze_log(log)
    else:
        print(f"(无日志) {log}")
    test_production_clamp()
    test_ruckig_overshoot()
    test_display_vs_log_gap(log if log.is_file() else None)
    test_synthetic_pipeline()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
