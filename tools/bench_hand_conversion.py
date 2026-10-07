"""Offline conversion benchmark; never opens USB or sends motor commands.

python tools/bench_hand_conversion.py --output results.json
Optional --v2-baseline points to a saved pre-change finger_linkage.py for
same-process timing and dense output equivalence checks.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import math
from pathlib import Path
import platform
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from kinematics import finger_linkage as linkage
from usb import offline
from a32_kinematics import A32StrokeMapper, A32Converter


def measure(fn, poses, repeats):
    for pose in poses[:100]:
        fn(pose)
    samples = []
    for _ in range(repeats):
        for pose in poses:
            start = time.perf_counter_ns()
            fn(pose)
            samples.append((time.perf_counter_ns() - start) / 1000)
    samples.sort()
    mean = statistics.mean(samples)
    return {"n": len(samples), "mean_us": mean,
            "median_us": statistics.median(samples),
            "p95_us": samples[int(.95 * len(samples))],
            "p99_us": samples[int(.99 * len(samples))],
            "throughput_hz": 1e6 / mean}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--v2-baseline", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--repeats", type=int, default=10)
    args = parser.parse_args()
    if args.repeats < 1:
        parser.error("--repeats must be positive")
    poses = [{name: .55 + .45 * math.sin(i * .037 + k * .19)
              for k, name in enumerate(linkage.LINEAR_SLOT_JOINTS)} |
             {name: .1 * math.sin(i * .03) for name in linkage.ROTARY_SLOT_JOINTS}
             for i in range(1000)]
    report = {"python": platform.python_version(), "platform": platform.platform(),
              "v2_fit_enabled": linkage._linkage_fit_enabled(),
              "scope": "warm conversion only; varying angles; no USB, retarget or motion interpolation",
              "benchmarks": {}}
    linkage.clear_phi_cf_lut_cache()
    start = time.perf_counter()
    linkage.create_default_hand_stroke_mapper()
    report["v2_initialization_ms"] = (time.perf_counter() - start) * 1000
    modules = {"v2": linkage}
    if args.v2_baseline:
        spec = importlib.util.spec_from_file_location("linkage_baseline", args.v2_baseline)
        baseline = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = baseline
        spec.loader.exec_module(baseline)
        modules = {"v2_before": baseline, **modules}
        old, new = baseline.create_default_hand_stroke_mapper(), linkage.create_default_hand_stroke_mapper()
        ramp = [{n: -1 + 5 * i / 1000 for n in linkage.LINEAR_SLOT_JOINTS} for i in range(1001)]
        probes = poses + ramp + [{n: bad for n in linkage.LINEAR_SLOT_JOINTS}
                                 for bad in (float("nan"), float("inf"), float("-inf"))]
        maximum = 0.0
        for pose in probes:
            a, b = old.linear_strokes_mm(pose), new.linear_strokes_mm(pose)
            maximum = max(maximum, max(abs(a[n] - b[n]) for n in a))
        report["v2_equivalence"] = {"poses": len(probes), "max_difference_mm": maximum,
                                    "allowed_difference_mm": linkage._UNIFORM_LUT_ERROR_MM}
        if maximum > linkage._UNIFORM_LUT_ERROR_MM + 1e-12:
            raise AssertionError(report["v2_equivalence"])
    for label, module in modules.items():
        mapper = module.create_default_hand_stroke_mapper()
        report["benchmarks"][label + "_14_linear"] = measure(mapper.linear_strokes_mm, poses, args.repeats)
        offline.configure_hand_stroke_mapper(mapper)
        report["benchmarks"][label + "_21_conversion"] = measure(offline.joint_angles_to_hand_positions, poses, args.repeats)
    report["benchmarks"]["v2_21_cached"] = measure(offline.joint_angles_to_hand_positions, [poses[0]] * 1000, args.repeats)
    converter = A32Converter()
    for method in ("exact", "lut"):
        mapper = A32StrokeMapper(method=method)
        a32_poses = [{name: lo + (hi - lo) * (.5 + .45 * math.sin(i * .037 + k * .19))
                      for k, (name, (lo, hi)) in enumerate(zip(mapper.joint_names, mapper._limits))}
                     for i in range(1000)]
        report["benchmarks"]["a32_20_" + method] = measure(mapper.convert, a32_poses, args.repeats)
        # Separate the degree-array -> SI prepared API from named project
        # inputs/outputs, so the upstream 19.94 us number has a matching scope.
        prepared = converter.prepare(method=method)
        import numpy as np
        degree_arrays = [np.array([math.degrees(p[n]) for n in mapper.joint_names]) for p in a32_poses]
        report["benchmarks"]["a32_20_core_" + method] = measure(prepared, degree_arrays, args.repeats)
    text = json.dumps(report, indent=2, ensure_ascii=False)
    print(text)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
