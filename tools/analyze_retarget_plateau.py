"""Diagnose missing retarget observations and replay contact recovery without USB.

Run with the hand_dex interpreter, input JSONL files, and --output a directory.
Duplicate recordings are identified by SHA-256. Diagnostic logs are sampled and
may span multiple tracking sessions; their gaps are not treated as camera losses.
"""
import argparse
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]


def analyze(path):
    counts, raw, retarget, planner = Counter(), [], [], []
    digest = hashlib.sha256()
    schema = None
    with path.open("rb") as source:
        for line in source:
            digest.update(line)
            if not line.strip():
                continue
            row = json.loads(line)
            kind = row.get("type", row.get("event"))
            counts[kind] += 1
            if kind == "session":
                schema = row.get("schema")
            if kind == "mediapipe_raw":
                raw.append((row.get("frame_id"), row.get("handedness", [])))
            elif kind == "retarget":
                retarget.append(row)
            elif kind == "planner_sample":
                planner.append(row)
    sampled = schema == "hand-diagnostic/v1"
    gaps = []
    if not sampled:
        for a, b in zip(retarget, retarget[1:]):
            if b["t"] - a["t"] <= .3:
                continue
            labels = Counter()
            for frame_id, hands in raw:
                if frame_id is not None and a["frame_id"] < frame_id < b["frame_id"]:
                    labels["+".join(h[0]["label"] for h in hands if h) or "no_hand"] += 1
            gaps.append({"start_s": a["t"], "end_s": b["t"], "duration_s": b["t"]-a["t"],
                         "frame_ids": [a["frame_id"], b["frame_id"]], "raw_labels_between_frames": dict(labels)})
    flats = []
    start = 0
    joint = "left_hand_index_swing_joint"
    for i in range(1, len(retarget)+1):
        if i < len(retarget) and abs(retarget[i]["joints"].get(joint, 0)-retarget[start]["joints"].get(joint, 0)) <= 1e-6:
            continue
        if retarget[i-1]["t"]-retarget[start]["t"] > .3:
            flats.append({"start_s": retarget[start]["t"], "end_s": retarget[i-1]["t"],
                          "frames": i-start, "value_rad": retarget[start]["joints"].get(joint)})
        start = i
    return {"file": str(path), "sha256": digest.hexdigest(), "schema": schema,
            "sampled_diagnostic": sampled, "counts": dict(counts), "retarget_gaps": gaps,
            "index_swing_constant_runs": flats,
            "planner_collision_held_samples": sum(bool(r.get("held_fingers")) for r in planner)}


def replay(output):
    import numpy as np
    from hand_profiles import load_hand_profile
    from online_control.online_motion_track.a30.control import A30Control

    class PreviousHoldPolicy(A30Control):
        def _collision_hold_fingers(self, candidate, fingers):
            return fingers - set(self.finger_holds)

    case = json.loads((ROOT / "tests/fixtures/a30_plateau_145628.json").read_text())
    result = {"source": case["source"], "scope": case["note"],
              "timing": "500 Hz; initial velocity and acceleration assumed zero; not a reconstruction of firmware motion"}
    curves = {}
    joint = "left_hand_index_swing_joint"
    for label, cls in (("before", PreviousHoldPolicy), ("after", A30Control)):
        c = cls(load_hand_profile("TsenseI_v3_0"), "left", "mink", start=False)
        values, costs, holds, first_motion = [], [], 0, None
        target_index = 0
        try:
            c._seed(case["seed_pose"]); c.target = dict(c.pose)
            for tick in range(1500):
                stamp = case["targets"][0]["t"] + tick*.002
                while target_index < len(case["targets"]) and case["targets"][target_index]["t"] <= stamp:
                    row = case["targets"][target_index]
                    c.push(row["joints"], row["frame_id"]); target_index += 1
                started = time.perf_counter()
                c.step(.002)
                costs.append((time.perf_counter()-started)*1000)
                if not c.valid:
                    raise RuntimeError(c.error)
                c.guard.filter.sync_pose_from_dict(c.pose)
                if c.guard.filter.data.ncon:
                    raise RuntimeError("Published replay pose has contact")
                values.append(c.pose[joint]); holds += bool(c.finger_holds)
                if first_motion is None and abs(c.pose[joint]-case["seed_pose"][joint]) > .01:
                    first_motion = tick*.002
            result[label] = {"first_0_01_rad_motion_ms": first_motion*1000 if first_motion is not None else None,
                             "held_samples": holds, "published_contact_samples": 0,
                             "step_ms_median_p95_max": np.percentile(costs, [50,95,100]).tolist()}
            curves[label] = values
        finally:
            c.close()
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(11, 4.5), layout="constrained")
    times = case["targets"][0]["t"] + np.arange(1500)*.002
    ax.step([r["t"] for r in case["targets"]], [r["joints"][joint] for r in case["targets"]],
            where="post", color="#9aa3af", label="Recorded collision-safe target")
    ax.plot(times, curves["before"], label="Before: hold both contact fingers", color="#d36a43")
    ax.plot(times, curves["after"], label="After: allow checked separating motion", color="#149c85")
    ax.set(xlabel="Recording time (s)", ylabel="Index swing (rad)",
           title="Contact recovery replay — 500 Hz, no hardware, seed at rest")
    ax.grid(alpha=.2); ax.legend(); fig.savefig(output / "recovery.png", dpi=160); plt.close(fig)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", type=Path, nargs="+")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=True)
    reports, hashes = [], {}
    for path in args.inputs:
        report = analyze(path)
        if report["sha256"] in hashes:
            report = {"file": str(path), "sha256": report["sha256"], "duplicate_of": hashes[report["sha256"]]}
        else:
            hashes[report["sha256"]] = str(path)
        reports.append(report)
        print(path.name, "duplicate" if "duplicate_of" in report else "analyzed", flush=True)
    result = {"recordings": reports, "recovery_replay": replay(args.output)}
    (args.output / "metrics.json").write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(result["recovery_replay"], indent=2))


if __name__ == "__main__":
    main()
