"""Offline A3 collision benchmark: same geometry/reference and optional old assets.

No hardware or service is opened. Results include exact trajectory agreement and
50/95/99th percentile query times. Synthetic trajectories and one recorded pose
are explicit; this is not an end-to-end tracking/communication latency measure.
"""
from __future__ import annotations

import argparse
import collections
import dataclasses
import importlib.util
import json
from pathlib import Path
import platform
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
import mujoco
from hand_profiles import load_hand_profile
from online_control.online_motion_track.mink_guard import MinkCollisionGuard
from online_control.online_motion_track.pipeline_v3_0.collision import StopCollisionGuard
from online_control.online_motion_track.pipeline_v2.collision_guard import SelfCollisionGuard


class ExhaustiveMinkReference(MinkCollisionGuard):
    """Uncached full safety scans, with identical robust bounds and Mink QP.

    Raw MuJoCo 3.10 mesh distances can return zero for demonstrably separated
    convex shapes. The reference must use the same separation certificates to
    avoid comparing against a defective oracle. It still queries every pair,
    redoes FK/bounds on each call, and performs all original path samples.
    """

    def _update(self, q):
        self._last_q_key = None
        super()._update(q)

    def _distances(self, q):
        self._update(q)
        bounds = self.bounds.lower_bounds(self.configuration.data)
        distances = np.array([self._query(i) for i in range(len(self.pairs))])
        return np.where(bounds > 0, np.maximum(distances, bounds), distances)

    def _feasible(self, q):
        self._update(q)
        bounds = self.bounds.lower_bounds(self.configuration.data)
        # Preserve the legacy feasibility loop's early exit. Full scans here
        # would unfairly inflate the reference cost on colliding targets.
        for index, margin in enumerate(self.margins):
            distance = self._query(index)
            if bounds[index] > 0:
                distance = max(distance, bounds[index])
            if distance < margin - 1e-7:
                return False
        return True

    def _refine_clearances(self, q, indices):
        return self._distances(q)

    def _path_feasible(self, start, end):
        return self._check_path(start, end)


class FullForwardStopReference:
    def __init__(self, profile, side):
        self.current = profile.init_pose(side)
        self.guard = SelfCollisionGuard(
            physics_filter=profile.create_collision_filter(side), strategy="stop",
            allowed_penetration_mm=0, path_step_rad=.025, max_path_steps=128,
            bisection_iterations=14)
        self.diagnostics = {}

    def project(self, pose):
        self.current, self.diagnostics = self.guard.project(pose, current_values=self.current)
        return self.current


def targets(profile, side):
    initial = profile.init_pose(side)
    limits = profile.joint_limits(side)
    neutral = {n: .35 if "_swing_" in n else q for n, q in initial.items()}
    flex = {n: (.65 if n.endswith("_MCP_joint") else .8 if n.endswith("_PIP_joint")
                else .55 if n.endswith("_DIP_joint") else q) for n, q in neutral.items()}
    pinch = dict(neutral)
    for suffix, value in (("index_MCP", .65), ("index_PIP", .5), ("index_DIP", .3),
                          ("thumb_rota_block", 1.2), ("thumb_rotaback", .85),
                          ("thumb_MCP", .45), ("thumb_PIP", .65)):
        name = f"{side}_hand_{suffix}_joint"
        if name in pinch:
            pinch[name] = value
    crossed = dict(neutral)
    crossed[f"{side}_hand_ring_swing_joint"] = .695
    crossed[f"{side}_hand_pinky_swing_joint"] = .005
    recorded = json.loads((ROOT / "tests/fixtures/wuji_a32_mesh_boundary.json").read_text())
    recorded = {n: recorded.get(n.replace(f"{side}_hand_", "left_hand_"), q)
                for n, q in initial.items()}
    rows = []
    routes = [(initial, neutral, "open"), (neutral, flex, "flex"),
              (flex, pinch, "synthetic_pinch"), (pinch, crossed, "crossed"),
              (crossed, initial, "release"),
              (initial, recorded, "recorded_pose_joint_mapping"),
              (recorded, initial, "recorded_release")]
    for start, end, label in routes:
        for t in np.linspace(.125, 1, 8):
            rows.append((label, {n: float(np.clip(start[n] + t * (end[n] - start[n]), *limits[n]))
                                 for n in initial}))
    return rows


def summary(rows):
    times = [r["wall_ms"] for r in rows]
    return {"calls": len(rows), **{f"p{p}_ms": float(np.percentile(times, p)) for p in (50, 95, 99)},
            "max_ms": max(times), "mean_ms": float(np.mean(times)),
            "status_counts": dict(collections.Counter(r.get("status", r.get("resolution")) for r in rows)),
            "distance_queries": sum(r.get("distance_queries", 0) for r in rows),
            "cache_hits": sum(r.get("cache_hits", 0) for r in rows),
            "max_path_refinements": max(r.get("path_refinements", 0) for r in rows),
            "path_budget_exhausted_calls": sum(bool(r.get("path_budget_exhausted")) for r in rows),
            "witness_certificates": sum(r.get("witness_certificates", 0) for r in rows)}


def run(cls, profile, side):
    rows, output = [], []
    guard = cls(profile, side)
    for label, pose in targets(profile, side):
        started = time.perf_counter_ns()
        result = guard.project(pose)
        elapsed = (time.perf_counter_ns() - started) / 1e6
        rows.append({"category": label, "wall_ms": elapsed, **guard.diagnostics})
        output.append([result[n] for n in sorted(result)])
    return rows, np.asarray(output)


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--baseline-dir", type=Path)
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--models", nargs="+", default=["TsenseI_v3_0", "TsenseI_v3_0_withid",
                                                       "TsenseI_v3_2", "TsenseI_v3_2_withid"])
    parser.add_argument("--merge", action="store_true", help="Preserve other models in an existing output file")
    args = parser.parse_args()
    result = {"environment": {"python": platform.python_version(), "mujoco": mujoco.__version__},
              "notes": ["56 sequential targets per run; no hardware, retargeting or I/O timing",
                        "40 synthetic targets + 16 transitions to/from one recorded pose using matching joint names",
                        "Reference uses same final geometry, margins, path sampling, QP and certified separation",
                        "distance_queries counts safety/diagnostic narrowphase only, excluding Mink QP assembly",
                        "Final guards include palm outer meshes on ordinary A3 hands and adaptive swept checks in Mink",
                        "Mink limits extra distinct midpoint evaluations to 128 per project and rejects unproven intervals when exhausted",
                        "Old geometry changes motion behavior; its timing is context, not an output-equivalence claim"],
              "models": {}}
    if args.merge and args.output.exists():
        result["models"].update(json.loads(args.output.read_text())["models"])
    old_class = None
    if args.baseline_dir:
        spec = importlib.util.spec_from_file_location("old_mink", args.baseline_dir / "mink_guard.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        old_class = module.MinkCollisionGuard
    for model in args.models:
        for side in ("left", "right"):
            profile = load_hand_profile(model)
            key = f"{model}/{side}"
            entry = {}
            for label, fast, reference in (("mink", MinkCollisionGuard, ExhaustiveMinkReference),
                                           ("stop", StopCollisionGuard, FullForwardStopReference)):
                # Warm dependency/solver imports before timing.
                warm = fast(profile, side)
                warm.project(targets(profile, side)[0][1])
                fast_rows, reference_rows = [], []
                errors = []
                for repeat in range(args.repeats):
                    variants = [("fast", fast), ("reference", reference)]
                    if repeat % 2:
                        variants.reverse()
                    measured = {name: run(cls, profile, side) for name, cls in variants}
                    fast_rows += measured["fast"][0]
                    reference_rows += measured["reference"][0]
                    errors.append(float(np.max(np.abs(measured["fast"][1] - measured["reference"][1]))))
                entry[label] = {"optimized": summary(fast_rows), "reference": summary(reference_rows),
                                "max_output_difference_rad": max(errors)}
            if old_class:
                old_profile = dataclasses.replace(profile, root=args.baseline_dir / "hands" / profile.root.name)
                try:
                    rows, _ = run(old_class, old_profile, side)
                    entry["original_mink_old_geometry"] = summary(rows)
                except Exception as exc:
                    entry["original_mink_old_geometry"] = {"error": str(exc)}
            result["models"][key] = entry
            print(key, json.dumps(entry), flush=True)
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(result, indent=2) + "\n")


if __name__ == "__main__":
    main()
