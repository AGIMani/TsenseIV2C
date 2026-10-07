"""Compare old reset policy, fixed automatic recovery and uninterrupted tracking.

Only camera frames and recorded command poses are read. The real web session's
rebase API is exercised with offline all-missing feedback plus the recorded last
command. Control threads, USB, camera and network I/O are disabled.
"""
import argparse
from bisect import bisect_right
from functools import partial
import json
from pathlib import Path
import sys
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from online_control.online_motion_track.a30 import session as module


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("analysis", type=Path, help="metrics.json from analyze_stationary_jitter.py")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=True)
    info = json.loads(args.analysis.read_text(encoding="utf-8"))
    resets = {r["frame_id"] for r in info["raw_equals_filtered_frames"][1:]}
    raw, inputs, writes, recoveries = {}, [], [], []
    with args.source.open(encoding="utf-8-sig") as source:
        for line in source:
            r = json.loads(line)
            if r["type"] == "mediapipe_raw":
                raw[r["frame_id"]] = r
            elif r["type"] == "mediapipe":
                inputs.append(r)
            elif r["type"] == "h7" and r.get("sent") and r.get("source") == "shared_binary_stream":
                writes.append(r)
            elif r["type"] == "trajectory_recovery":
                recoveries.append(r["t"])
    write_times = [r["t"] for r in writes]
    sessions, values = {}, {k:[] for k in ("old_reset", "fixed", "uninterrupted")}
    recovered_indices = []
    with patch.object(module, "A30Control", partial(module.A30Control, start=False)):
        try:
            for label in values:
                sessions[label] = module.A30WebSession(recording=False, collision_mode="mink")
            for i, record in enumerate(inputs):
                row = raw[record["frame_id"]]
                ix = next(k for k,h in enumerate(row["handedness"]) if h[0]["label"] == "Left")
                packet = {"sequence":row["frame_id"], "timestamp_ms":row["timestamp_ms"], "capture_unix_ms":0,
                          "hand":{"label":"Left", "world":row["world_landmarks"][ix],
                                  "normalized":row["normalized_landmarks"][ix]}}
                if row["frame_id"] in resets:
                    recovered_indices.append(i)
                    recovery_at = recoveries[bisect_right(recoveries, record["t"])-1]
                    seed = writes[bisect_right(write_times, recovery_at)-1]["joints_ui"]
                    for label in ("old_reset", "fixed"):
                        s = sessions[label]
                        s.command("browser_rebase", {"telemetry":bytes(484).hex(), "pose":seed,
                                                      "resume_from_commanded_pose":True})
                        if label == "old_reset":
                            # Restore the previous estimator-side effects after
                            # the same validated command-side rebase.
                            s.pipeline.session.set_pose(seed)
                            s.pipeline.session.retargeter.reset()
                            guard = s.pipeline.mink_guard
                            q = guard._vector(seed)
                            if guard._feasible(q):
                                guard.q = q.copy()
                            elif not guard._feasible(guard.q):
                                raise RuntimeError("Legacy target-side seed is infeasible")
                            s.filter.reset(); s.normalized_filter.reset()
                for label, s in sessions.items():
                    result = s.process(packet)
                    if not result["tracking"]:
                        raise RuntimeError(result["message"])
                    values[label].append(result["retarget_joint_angles_rad"])
                if i % 200 == 0:
                    print(f"replayed {i}/{len(inputs)}", flush=True)
        finally:
            for s in sessions.values():
                s.close()
    names = list(values["fixed"][0])
    q = {label:np.array([[r[n] for n in names] for r in rows]) for label,rows in values.items()}
    ri = np.array(recovered_indices)
    result = {"source":str(args.source), "frames":len(inputs), "automatic_recoveries":len(ri),
              "scope":"Offline estimator replay; recorded successful frames and last commands; no physical writes; recovery timing inferred from reset frames",
              "max_fixed_vs_uninterrupted_deg":float(np.max(np.abs(q["fixed"]-q["uninterrupted"])))*180/np.pi}
    for label in values:
        steps = np.max(np.abs(q[label][ri]-q[label][ri-1]),axis=1)*180/np.pi
        result[label] = {"recovery_frame_max_joint_step_deg_p50_p95_max":np.percentile(steps,[50,95,100]).tolist()}
    (args.output/"metrics.json").write_text(json.dumps(result,indent=2)+"\n",encoding="utf-8")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    times = np.array([r["t"] for r in inputs]); mask = (times>=37)&(times<39)
    joint = names.index("left_hand_thumb_rota_block_joint")
    fig, ax = plt.subplots(figsize=(11,4),layout="constrained")
    for label,color in (("old_reset","#d36a43"),("fixed","#149c85"),("uninterrupted","#7186b5")):
        ax.plot(times[mask],q[label][mask,joint]*180/np.pi,label=label,color=color,
                linestyle=":" if label=="uninterrupted" else "-")
    for i in ri:
        if 37<=times[i]<39:
            ax.axvline(times[i],color="gray",ls=":",alpha=.4)
    ax.set(xlabel="Recording time (s)",ylabel="Thumb base retarget (deg)",
           title="Recorded landmark replay — automatic transport recovery")
    ax.grid(alpha=.2);ax.legend();fig.savefig(args.output/"comparison.png",dpi=150);plt.close(fig)
    print(json.dumps(result,indent=2))


if __name__ == "__main__":
    main()
