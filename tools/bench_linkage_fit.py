#!/usr/bin/env python3
"""对比连杆 phi→CF：精确二分 vs 拟合表（精度 + 耗时）。

用法::

    python tools/bench_linkage_fit.py
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from kinematics.finger_linkage import (  # noqa: E402
    MOTOR_F1,
    MOTOR_F2,
    MOTOR_F3,
    FingerLinkageParams,
    clear_phi_cf_lut_cache,
    create_default_hand_stroke_mapper,
    linkage_fit_max_error_mm,
    motor_phi_deg_limits,
    phi_deg_to_cf_mm,
)


def _bench(label: str, fn, n: int = 5000) -> float:
    t0 = time.perf_counter()
    for _ in range(n):
        fn()
    return (time.perf_counter() - t0) * 1000.0 / n


def main() -> None:
    clear_phi_cf_lut_cache()
    params = FingerLinkageParams.from_matlab_defaults()
    thumb = FingerLinkageParams.from_thumb_matlab_defaults()

    print("=== 拟合精度（相对精确二分）===")
    for name, p, motors in (
        ("四指", params, (MOTOR_F1, MOTOR_F2, MOTOR_F3)),
        ("拇指", thumb, (MOTOR_F1, MOTOR_F3)),
    ):
        for m in motors:
            mx, mean, _ = linkage_fit_max_error_mm(m, p, n_check=400)
            print(f"  {name} F{m+1}: max_err={mx*1000:.3f} um  mean={mean*1000:.3f} um")

    print("\n=== 单次 phi→CF 耗时 ===")
    m = MOTOR_F3
    lo, hi = motor_phi_deg_limits(m, params)
    phi = 0.5 * (lo + hi)
    # 预热表
    phi_deg_to_cf_mm(phi, m, params, exact=False)
    t_fit = _bench("fit", lambda: phi_deg_to_cf_mm(phi, m, params, exact=False))
    t_ex = _bench("exact", lambda: phi_deg_to_cf_mm(phi, m, params, exact=True))
    print(f"  fit   {t_fit*1000:.2f} us/call")
    print(f"  exact {t_ex*1000:.2f} us/call")
    print(f"  speedup x{t_ex / max(t_fit, 1e-12):.1f}")

    print("\n=== hand linear_strokes_mm ===")
    mapper = create_default_hand_stroke_mapper()
    cfg = {j: 0.4 for j in mapper.managed_linear_joints()}
    mapper.linear_strokes_mm(cfg)  # warm
    t_hand = _bench("hand", lambda: mapper.linear_strokes_mm(cfg), n=1000)
    print(f"  {t_hand*1000:.1f} us/call  (~{1.0/max(t_hand,1e-9):.2f} kHz)")


if __name__ == "__main__":
    main()
