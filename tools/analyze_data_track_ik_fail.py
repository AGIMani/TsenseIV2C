#!/usr/bin/env python3
"""从 data_track 日志回放关节角，用 finger_linkage 找出会逆解失败/越界的角度。

输出：``<log_stem>_ik_fail.log``
  - 每条问题记录一行 JSON
  - 末尾 summary：去重失败角、按指统计

判定两类：
  1) ``ik_fail``：不做角度钳位的严格逆解（phi→CF 二分）失败
  2) ``out_of_range``：角度超出电机可达 phi 范围（生产路径会 soft-clamp）

用法:
  python tools/analyze_data_track_ik_fail.py logs/data_track_YYYYMMDD_HHMMSS.log
"""
from __future__ import annotations

import json
import math
import sys
from collections import Counter
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from kinematics.finger_linkage import (  # noqa: E402
    LinkageKinematicsError,
    MOTOR_F1,
    MOTOR_F2,
    MOTOR_F3,
    cf_to_phi_deg,
    clamp_joint_rad_for_motor,
    create_default_hand_stroke_mapper,
    joint_rad_to_phi_deg,
    motor_phi_deg_limits,
)
from protocol import convert_ui_to_hardware  # noqa: E402

_JOINT_SHORT = (
    "thumb_rota",
    "thumb_swing",
    "thumb_MCP",
    "thumb_PIP",
    "index_swing",
    "index_MCP",
    "index_PIP",
    "index_DIP",
    "mid_swing",
    "mid_MCP",
    "mid_PIP",
    "mid_DIP",
    "ring_swing",
    "ring_MCP",
    "ring_PIP",
    "ring_DIP",
    "pinky_rota",
    "pinky_swing",
    "pinky_MCP",
    "pinky_PIP",
    "pinky_DIP",
)


def _cfg_from_record(rec: Dict[str, Any]) -> Dict[str, float]:
    by_name = rec.get("angles_by_name") or {}
    out: Dict[str, float] = {}
    if isinstance(by_name, dict) and by_name:
        for short in _JOINT_SHORT:
            out[f"left_hand_{short}_joint"] = float(by_name.get(short, 0.0) or 0.0)
        return out
    angles = rec.get("angles_rad") or []
    for i, short in enumerate(_JOINT_SHORT):
        val = float(angles[i]) if i < len(angles) else 0.0
        out[f"left_hand_{short}_joint"] = val
    return out


def phi_deg_to_cf_mm_strict(
    phi_deg: float,
    motor: int,
    params,
    *,
    tol_deg: float = 1e-4,
    max_iter: int = 60,
) -> float:
    """不做 soft-clamp 的严格逆解：目标角超出端点可达范围即失败。"""
    cf_lo, cf_hi = params.motor_cf_limits(motor)
    if not math.isfinite(phi_deg):
        raise LinkageKinematicsError(
            f"F{motor + 1} 逆解输入非有限 phi={phi_deg}",
            context={"motor": motor + 1, "phi_deg": phi_deg},
        )
    phi_lim = motor_phi_deg_limits(motor, params)
    try:
        phi_lo = cf_to_phi_deg(cf_lo, motor, params)
        phi_hi = cf_to_phi_deg(cf_hi, motor, params)
    except LinkageKinematicsError as exc:
        raise LinkageKinematicsError(
            f"F{motor + 1} 端点 CF 不可解: {exc}",
            context={
                "motor": motor + 1,
                "phi_in_deg": float(phi_deg),
                "phi_lim_deg": phi_lim,
                "cf_lo": cf_lo,
                "cf_hi": cf_hi,
            },
        ) from exc

    lo_p, hi_p = (phi_lo, phi_hi) if phi_lo <= phi_hi else (phi_hi, phi_lo)
    # 允许 1e-3 deg 边界容差
    if phi_deg < lo_p - 1e-3 or phi_deg > hi_p + 1e-3:
        raise LinkageKinematicsError(
            f"F{motor + 1} 目标角超出可达范围 phi={phi_deg:.4f} "
            f"not in [{lo_p:.4f}, {hi_p:.4f}] deg",
            context={
                "motor": motor + 1,
                "phi_in_deg": float(phi_deg),
                "reachable_phi_deg": [lo_p, hi_p],
                "phi_lim_deg": phi_lim,
                "cf_lo": cf_lo,
                "cf_hi": cf_hi,
            },
        )

    target = float(phi_deg)
    lo, hi = cf_lo, cf_hi
    mid_fail = 0
    for _ in range(max_iter):
        mid = 0.5 * (lo + hi)
        try:
            phi_mid = cf_to_phi_deg(mid, motor, params)
        except LinkageKinematicsError:
            mid_fail += 1
            # 与生产类似：缩小到仍可解一侧
            hi = mid
            if abs(hi - lo) < 1e-9:
                raise LinkageKinematicsError(
                    f"F{motor + 1} 严格逆解区间塌缩 (mid_fail={mid_fail})",
                    context={
                        "motor": motor + 1,
                        "phi_in_deg": float(phi_deg),
                        "mid_fail_count": mid_fail,
                    },
                )
            continue
        if abs(phi_mid - target) <= tol_deg:
            return mid
        # 假设 CF↑ 时 |phi| 单调（与生产二分同向约定）
        if (phi_mid - target) * (phi_hi - phi_lo) > 0:
            hi = mid
        else:
            lo = mid
    raise LinkageKinematicsError(
        f"F{motor + 1} 严格逆解未收敛 phi={phi_deg:.4f}",
        context={
            "motor": motor + 1,
            "phi_in_deg": float(phi_deg),
            "cf_lo": lo,
            "cf_hi": hi,
            "mid_fail_count": mid_fail,
        },
    )


def strict_four_finger_ik(
    mapper,
    binding,
    hw_cfg: Dict[str, float],
) -> Tuple[Optional[Dict[str, float]], Optional[Dict[str, Any]]]:
    """返回 (strokes_or_None, fail_info_or_None)。不做生产 soft-clamp。"""
    p = mapper.four_finger.params
    raw_mcp = float(hw_cfg.get(binding.mcp_joint, 0.0))
    raw_pip = float(hw_cfg.get(binding.pip_joint, 0.0))
    raw_dip = float(hw_cfg.get(binding.dip_joint, 0.0))
    # 生产路径会 clamp；这里同时给出 clamp 对比
    c_mcp = clamp_joint_rad_for_motor(raw_mcp, binding.mcp_motor, p)
    c_pip = clamp_joint_rad_for_motor(raw_pip, binding.pip_motor, p)
    c_dip = clamp_joint_rad_for_motor(raw_dip, binding.dip_motor, p)
    clamped = any(
        abs(a - b) > 1e-6 for a, b in ((raw_mcp, c_mcp), (raw_pip, c_pip), (raw_dip, c_dip))
    )
    base = {
        "finger": binding.finger,
        "raw_rad": {
            "mcp": round(raw_mcp, 6),
            "pip": round(raw_pip, 6),
            "dip": round(raw_dip, 6),
        },
        "raw_deg": {
            "mcp": round(math.degrees(raw_mcp), 3),
            "pip": round(math.degrees(raw_pip), 3),
            "dip": round(math.degrees(raw_dip), 3),
        },
        "clamped_rad": {
            "mcp": round(c_mcp, 6),
            "pip": round(c_pip, 6),
            "dip": round(c_dip, 6),
        },
        "out_of_range": clamped,
    }
    try:
        phi_dip = joint_rad_to_phi_deg(raw_dip)  # F1
        phi_pip = joint_rad_to_phi_deg(raw_pip)  # F2
        phi_mcp = joint_rad_to_phi_deg(raw_mcp)  # F3
        cf1 = phi_deg_to_cf_mm_strict(phi_dip, MOTOR_F1, p)
        cf2 = phi_deg_to_cf_mm_strict(phi_pip, MOTOR_F2, p)
        cf3 = phi_deg_to_cf_mm_strict(phi_mcp, MOTOR_F3, p)
        cfs = (cf1, cf2, cf3)
        strokes = {
            binding.mcp_joint: binding.stroke_sign * cfs[binding.mcp_motor],
            binding.pip_joint: binding.stroke_sign * cfs[binding.pip_motor],
            binding.dip_joint: binding.stroke_sign * cfs[binding.dip_motor],
        }
        if clamped:
            return strokes, {**base, "status": "out_of_range_clamped_in_prod"}
        return strokes, None
    except LinkageKinematicsError as exc:
        return None, {
            **base,
            "status": "ik_fail",
            "reason": str(exc),
            "context": getattr(exc, "context", {}) or {},
        }


def strict_thumb_ik(
    mapper,
    hw_cfg: Dict[str, float],
) -> Tuple[Optional[Dict[str, float]], Optional[Dict[str, Any]]]:
    thumb = mapper.thumb
    p = thumb.params
    raw_mcp = float(hw_cfg.get(thumb.mcp_joint, 0.0))
    raw_pip = float(hw_cfg.get(thumb.pip_joint, 0.0))
    c_mcp = clamp_joint_rad_for_motor(raw_mcp, thumb.mcp_motor, p)
    c_pip = clamp_joint_rad_for_motor(raw_pip, thumb.pip_motor, p)
    clamped = abs(raw_mcp - c_mcp) > 1e-6 or abs(raw_pip - c_pip) > 1e-6
    base = {
        "finger": "thumb",
        "raw_rad": {"mcp": round(raw_mcp, 6), "pip": round(raw_pip, 6)},
        "raw_deg": {
            "mcp": round(math.degrees(raw_mcp), 3),
            "pip": round(math.degrees(raw_pip), 3),
        },
        "clamped_rad": {"mcp": round(c_mcp, 6), "pip": round(c_pip, 6)},
        "out_of_range": clamped,
    }
    try:
        cf3 = phi_deg_to_cf_mm_strict(
            joint_rad_to_phi_deg(raw_mcp), thumb.mcp_motor, p
        )
        cf1 = phi_deg_to_cf_mm_strict(
            joint_rad_to_phi_deg(raw_pip), thumb.pip_motor, p
        )
        strokes = {
            thumb.mcp_joint: thumb.stroke_sign * cf3,
            thumb.pip_joint: thumb.stroke_sign * cf1,
        }
        if clamped:
            return strokes, {**base, "status": "out_of_range_clamped_in_prod"}
        return strokes, None
    except LinkageKinematicsError as exc:
        return None, {
            **base,
            "status": "ik_fail",
            "reason": str(exc),
            "context": getattr(exc, "context", {}) or {},
        }


def analyze_record(mapper, rec: Dict[str, Any]) -> List[Dict[str, Any]]:
    """对日志中的 UI 角（angles_by_name）做严格逆解检测。

    说明：生产下发会先 ``convert_ui_to_hardware``（会把负弯曲等钳掉），
    再走 soft-clamp，因此生产路径很少抛失败。这里直接用日志里的 UI 角，
    找出「若未经转换/钳位」会逆解失败或越界的角度。
    """
    ui_cfg = _cfg_from_record(rec)
    side = str(rec.get("hand_side") or "left")
    hw_cfg = convert_ui_to_hardware(ui_cfg, hand_side=side)

    problems: List[Dict[str, Any]] = []
    # 1) UI 空间（日志原角）
    for binding in mapper.finger_bindings:
        _, info = strict_four_finger_ik(mapper, binding, ui_cfg)
        if info is not None:
            info = dict(info)
            info["space"] = "ui"
            problems.append(info)
    if mapper.thumb.is_configured():
        _, info = strict_thumb_ik(mapper, ui_cfg)
        if info is not None:
            info = dict(info)
            info["space"] = "ui"
            problems.append(info)

    # 2) HW 空间（与下发一致）——仅在与 UI 结果不同时附加
    hw_problems: List[Dict[str, Any]] = []
    for binding in mapper.finger_bindings:
        _, info = strict_four_finger_ik(mapper, binding, hw_cfg)
        if info is not None:
            info = dict(info)
            info["space"] = "hw"
            hw_problems.append(info)
    if mapper.thumb.is_configured():
        _, info = strict_thumb_ik(mapper, hw_cfg)
        if info is not None:
            info = dict(info)
            info["space"] = "hw"
            hw_problems.append(info)

    if hw_problems:
        for p in hw_problems:
            problems.append(p)
    elif problems:
        # UI 有问题但 HW 全过：标记已被协议层消掉
        for p in problems:
            if p.get("space") == "ui":
                p["note"] = "ui_fail_but_hw_ok_after_convert_ui_to_hardware"
    return problems


def _unique_key(p: Dict[str, Any]) -> Tuple:
    rr = p.get("raw_rad") or {}
    return (
        p.get("space"),
        p.get("finger"),
        p.get("status"),
        round(float(rr.get("mcp", 0.0)), 4),
        round(float(rr.get("pip", 0.0)), 4),
        round(float(rr.get("dip", 0.0)), 4) if "dip" in rr else None,
    )


def main() -> int:
    log_path = (
        Path(sys.argv[1])
        if len(sys.argv) >= 2
        else ROOT / "logs" / "data_track_20260730_150811.log"
    )
    if not log_path.is_file():
        print(f"找不到日志: {log_path}", file=sys.stderr)
        return 1

    out_path = log_path.with_name(log_path.stem + "_ik_fail.log")
    mapper = create_default_hand_stroke_mapper()

    n_rec = 0
    n_problem_rec = 0
    status_counts: Counter = Counter()
    finger_counts: Counter = Counter()
    kind_counts: Counter = Counter()
    unique: Dict[Tuple, Dict[str, Any]] = {}

    with log_path.open("r", encoding="utf-8") as fin, out_path.open(
        "w", encoding="utf-8"
    ) as fout:
        fout.write(
            f"# ik_fail / out_of_range analysis from {log_path.name}\n"
            f"# ik_fail = 严格逆解失败（无 soft-clamp）\n"
            f"# out_of_range_clamped_in_prod = 超出可达角，生产路径会钳位后仍可能下发\n"
        )
        for line in fin:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            try:
                rec = json.loads(line)
            except json.JSONDecodeError:
                continue
            if not isinstance(rec, dict):
                continue
            if "angles_by_name" not in rec and "angles_rad" not in rec:
                continue
            n_rec += 1
            problems = analyze_record(mapper, rec)
            if not problems:
                continue
            n_problem_rec += 1
            for p in problems:
                status_counts[str(p.get("status"))] += 1
                finger_counts[str(p.get("finger"))] += 1
                kind_counts[str(rec.get("kind"))] += 1
                unique[_unique_key(p)] = {
                    "space": p.get("space"),
                    "finger": p.get("finger"),
                    "status": p.get("status"),
                    "raw_rad": p.get("raw_rad"),
                    "raw_deg": p.get("raw_deg"),
                    "reason": p.get("reason"),
                    "note": p.get("note"),
                }
            row = {
                "seq": rec.get("seq"),
                "t": rec.get("t"),
                "kind": rec.get("kind"),
                "source": rec.get("source"),
                "hand_side": rec.get("hand_side"),
                "angles_by_name": rec.get("angles_by_name"),
                "problems": problems,
            }
            fout.write(
                json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n"
            )

        summary = {
            "summary": True,
            "input": str(log_path).replace("\\", "/"),
            "output": str(out_path).replace("\\", "/"),
            "records_scanned": n_rec,
            "records_with_problems": n_problem_rec,
            "status_counts": dict(status_counts),
            "finger_counts": dict(finger_counts),
            "kind_counts": dict(kind_counts),
            "unique_problem_angles": list(unique.values()),
            "unique_problem_angle_count": len(unique),
        }
        fout.write(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")

    print(f"扫描记录: {n_rec}")
    print(f"有问题记录: {n_problem_rec}")
    print(f"状态统计: {dict(status_counts)}")
    print(f"按指统计: {dict(finger_counts)}")
    print(f"去重问题角: {len(unique)}")
    print(f"输出: {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
