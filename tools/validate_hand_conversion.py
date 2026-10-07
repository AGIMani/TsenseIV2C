"""Deterministic, hardware-free input/boundary stress audit of V2 and A3.2.

python tools/validate_hand_conversion.py --output docs/validation/hand_conversion.json
Any unexpected exception, fallback, nonfinite output, bound or accuracy failure
aborts with a nonzero exit. No serial ports/controllers are opened.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import platform
import random
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from kinematics import finger_linkage as fl
from usb import offline
from a32_kinematics import A32StrokeMapper
from a32_kinematics.a32_angles_to_inputs import load_configuration


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def v2_audit(count, rng):
    mapper = fl.create_default_hand_stroke_mapper()
    offline.configure_hand_stroke_mapper(mapper)
    channels = []
    for b in mapper.finger_bindings:
        channels.extend((n, mapper.four_finger.params, m) for n, m in (
            (b.mcp_joint, b.mcp_motor), (b.pip_joint, b.pip_motor), (b.dip_joint, b.dip_motor)))
    channels.extend(((mapper.thumb.mcp_joint, mapper.thumb.params, fl.MOTOR_F3),
                     (mapper.thumb.pip_joint, mapper.thumb.params, fl.MOTOR_F1)))
    bounds = {n: p.motor_cf_limits(m)[1] for n, p, m in channels}
    angles = {n: tuple(math.radians(x) for x in fl.motor_phi_deg_limits(m, p)) for n, p, m in channels}
    fallback = []
    original = fl._log_kin_failure
    fl._log_kin_failure = lambda *args, **kwargs: fallback.append((args, kwargs))
    max_angle_error = 0.0
    try:
        # Dense one-motor ramps plus adjacent floating-point values at every
        # mechanism endpoint and the soft-saturation transition.
        dense_samples = 0
        for name, p, m in channels:
            kernel = fl._get_prepared_motors(p)[m]
            lo, hi = angles[name]
            grid = [lo + (hi - lo) * i / 4096 for i in range(4097)]
            for boundary in (lo, hi, math.radians(kernel.hi - kernel.blend)):
                grid.extend((math.nextafter(boundary, -math.inf), boundary,
                             math.nextafter(boundary, math.inf)))
            previous = -math.inf
            for rad in sorted(grid):
                cf = kernel.joint_rad_to_cf_mm(rad)
                require(math.isfinite(cf) and -1e-12 <= cf <= bounds[name] + 1e-12, f"V2 invalid stroke: {name}")
                require(cf >= previous - 1e-12, f"V2 nonmonotonic stroke: {name}")
                target_phi = fl.smooth_limit_phi_deg_to_motor(math.degrees(rad), m, p)
                error = abs(fl.cf_to_phi_deg(cf, m, p) - target_phi)
                max_angle_error = max(max_angle_error, error)
                require(error < .01, f"V2 roundtrip error: {name} {error} deg")
                previous = cf
                dense_samples += 1
        for i in range(count):
            pose = {n: (rng.choice((lo, hi)) if i % 10 == 0 else rng.uniform(lo, hi))
                    for n, (lo, hi) in angles.items()}
            pose.update({n: rng.uniform(-.3, .3) for n in fl.ROTARY_SLOT_JOINTS})
            linear, rotary = offline.joint_angles_to_hand_positions(pose)
            require(len(linear) == 14 and len(rotary) == 7, "V2 output shape")
            for n, value in zip(fl.LINEAR_SLOT_JOINTS, linear):
                require(math.isfinite(value) and -bounds[n] - 1e-10 <= value <= 1e-10, f"V2 out of stroke range: {n}")
            require(all(math.isfinite(x) for x in rotary), "V2 nonfinite rotary")
            offline.pack_hand_pos_batch(i & 255, linear, rotary)
        require(not fallback, f"V2 silently held a valid input: {fallback[:1]}")
    finally:
        fl._log_kin_failure = original
    return {"random_poses": count, "dense_motor_samples": dense_samples,
            "max_roundtrip_error_deg": max_angle_error, "fallback_events": len(fallback),
            "unexpected_failures": 0, "nonfinite_outputs": 0, "out_of_range_outputs": 0}


def a32_audit(count, rng, side):
    mapper = A32StrokeMapper(side)
    exact = A32StrokeMapper(side, method="exact")
    config = load_configuration()
    stroke_bounds = {n: tuple(float(x) * 1000 for x in p["strokeRange"])
                     for n, p in zip(mapper.joint_names, config["motors"]) if p["type"] == "linear"}
    max_error_mm = 0.0
    # Prepared kernels are independent, so dense per-channel scans exercise
    # the whole domain without trying to enumerate a 20-dimensional grid.
    dense_samples = 0
    import numpy as np
    lo, hi = mapper._prepared.limits_deg.T
    degrees = np.linspace(lo, hi, 8193)
    for values in (degrees, np.nextafter(degrees, -np.inf), np.nextafter(degrees, np.inf)):
        fast_values = mapper._prepared.batch(values)
        exact_values = exact._prepared.batch(values)
        linear_indices = [i for i, flag in enumerate(mapper._linear) if flag]
        error = float(np.max(np.abs(fast_values[:, linear_indices] - exact_values[:, linear_indices]))) * 1000
        max_error_mm = max(max_error_mm, error)
        require(error <= 1e-6 + 1e-10, "A3.2 dense LUT error")
        require(np.all(np.isfinite(fast_values)), "A3.2 nonfinite dense output")
        require(np.all(np.diff(fast_values, axis=0) >= -1e-12), "A3.2 nonmonotonic output")
        dense_samples += values.size
    for i in range(count):
        pose = {n: (rng.choice((a, b)) if i % 10 == 0 else rng.uniform(a, b))
                for n, (a, b) in zip(mapper.joint_names, mapper._limits)}
        result = mapper.convert(pose)
        require(len(result.linear_strokes_mm) == 14 and len(result.rotary_positions_deg) == 6, "A3.2 output shape")
        require(not result.clamped_joints_rad, "A3.2 valid pose unexpectedly clamped")
        for n, value in result.linear_strokes_mm.items():
            a, b = stroke_bounds[n]
            require(math.isfinite(value) and a - 1e-10 <= value <= b + 1e-10, f"A3.2 out of stroke range: {n}")
        require(all(math.isfinite(x) for x in result.rotary_positions_deg.values()), "A3.2 nonfinite rotary")
        if i % 100 == 0:
            expected = exact.convert(pose)
            error = max(abs(v - expected.linear_strokes_mm[n]) for n, v in result.linear_strokes_mm.items())
            max_error_mm = max(max_error_mm, error)
            require(error <= 1e-6 + 1e-10, "A3.2 random LUT error")
    return {"method": mapper.method, "random_poses": count, "dense_motor_samples": dense_samples,
            "max_lut_vs_exact_error_mm": max_error_mm,
            "unexpected_failures": 0, "nonfinite_outputs": 0, "out_of_range_outputs": 0}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--poses", type=int, default=50000, help="per model/side; total 3 times this count")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.poses < 1:
        parser.error("--poses must be positive")
    start = time.perf_counter()
    rng = random.Random(32020918)
    report = {"python": platform.python_version(), "seed": 32020918,
              "scope": "Default bundled geometry; finite in-range poses; malformed inputs covered separately by pytest"}
    report["v2"] = v2_audit(args.poses, rng)
    print("V2 stress scan passed", flush=True)
    for side in ("left", "right"):
        report["a32_" + side] = a32_audit(args.poses, rng, side)
        print("A3.2 " + side + " stress scan passed", flush=True)
    sources = ["src/kinematics/finger_linkage.py", "src/usb/offline.py", "src/a32_kinematics/mapper.py",
               "src/a32_kinematics/a32_fast.py", "src/a32_kinematics/geometry_constants.py"]
    report["sha256"] = {p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest() for p in sources}
    report["elapsed_s"] = time.perf_counter() - start
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
