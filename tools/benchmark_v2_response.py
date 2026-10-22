"""Reproduce V2 fist-to-extension response without a camera or hardware.

python tools/benchmark_v2_response.py --ablation --output logs/v2_response/results.json
Frames are synthetic, geometrically reachable FK inputs at a declared rate;
timings are not measurements of a camera session or USB execution.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from TsenseI_retargeting import TopoRetargeter
from physics.mujoco_filter import MuJoCoPhysicsFilter
from utils.retarget_runtime import _build_topo_config
from utils.ruckig_interpolator import RuckigJointInterpolator, _RUCKIG_DT


def extension_case(config, *, finger="all", collision=False, frames=40, fps=28.):
    solver = TopoRetargeter(config)
    flex = [n for n in solver.joint_names
            if any(f"_{f}_" in n for f in ("index", "mid", "ring", "pinky"))
            and any(f"_{j}_" in n for j in ("MCP", "PIP", "DIP"))]
    fist = {n: 1.2 if n in flex else 0. for n in solver.joint_names}
    watched = [n for n in flex if finger == "all" or f"_{finger}_" in n]
    target = {n: 0. if n in watched else v for n, v in fist.items()}
    closed, opened = (solver.fk.compute(q) for q in (fist, target))
    physics = None
    if collision:
        physics = MuJoCoPhysicsFilter(ROOT / "assets/hands/TsenseI_hand/mjcf/mjmodel.xml",
                                      warmup_steps=0, default_n_steps=20, retarget_pull_steps=4)
        physics.sync_pose_from_dict(fist)
    previous = None
    durations = []

    def tick(points):
        nonlocal previous
        start = time.perf_counter()
        result = solver.retarget(points, previous_joint_values=previous, include_artifacts=False)
        durations.append((time.perf_counter()-start)*1000)
        if not result.success:
            raise RuntimeError(result.diagnostics)
        q = result.joint_values
        previous = physics.step_dict(q) if physics else q
        return q, previous

    for _ in range(60):
        tick(closed)
    interp = RuckigJointInterpolator(joint_names=solver.joint_names)
    interp.seed_ramp(previous, previous)
    traces = []
    elapsed = 0.

    def error(q):
        return float(np.degrees(max(abs(q[n]-target[n]) for n in watched)))

    try:
        for index in range(frames):
            q, output = tick(opened)
            interp.push(output)
            # A simulated 500 Hz trajectory between fixed-rate keyframes.
            while elapsed + _RUCKIG_DT <= (index+1)/fps + 1e-12:
                displayed = interp.sample()
                elapsed += _RUCKIG_DT
            traces.append({"frame": index+1, "elapsed_s": elapsed,
                           "solver_error_deg": error(q), "physics_error_deg": error(output),
                           "ruckig_error_deg": error(displayed),
                           "contacts": len(physics.contact_summary()) if physics else 0})
    finally:
        interp.shutdown()
    crossing = {}
    for stage in ("solver", "physics", "ruckig"):
        key = stage + "_error_deg"
        # Three consecutive values avoid treating a transient crossing as settling.
        crossing[stage] = next((r["frame"] for i, r in enumerate(traces[:-2])
                                if all(t[key] < 5 for t in traces[i:i+3])), None)
    return {"finger": finger, "collision": collision, "input_fps": fps,
            "frames_to_5deg": crossing, "second_frame": traces[1],
            "solve_ms_median": float(np.median(durations)), "trace": traces}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=ROOT / "logs/v2_response_20260918/results.json")
    parser.add_argument("--ablation", action="store_true")
    args = parser.parse_args()
    current = _build_topo_config()
    # Keep the historical baseline free of the newer fingertip position objective.
    old = replace(current, lambda_fingertip=0, lambda_smooth=2.5, lambda_reg=2.5,
                  init_max_iters=1, topo_max_iters=2)
    variants = {"before": old, "after": current}
    if args.ablation:
        variants.update(weights_only=replace(old, lambda_smooth=.1, lambda_reg=.1),
                        iterations_only=replace(old, init_max_iters=2, topo_max_iters=3))
    rows = []
    for finger in ("all", "mid"):
        for collision in (False, True):
            for label, config in variants.items():
                row = {"variant": label, **extension_case(config, finger=finger, collision=collision)}
                rows.append(row)
                print(label, finger, collision, row["frames_to_5deg"], flush=True)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(rows, indent=2), encoding="utf-8")
    print(args.output)


if __name__ == "__main__":
    main()
