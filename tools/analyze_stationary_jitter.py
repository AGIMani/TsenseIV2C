"""Read-only A3.0 jitter diagnosis and constant-landmark reset experiment.

Use the hand_dex Python environment. No camera, USB or network is opened.
Low-motion windows are candidates, not externally verified stationary poses.
"""
import argparse
from collections import Counter, defaultdict
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from online_control.online_motion_track.pipeline_v3_2.pipeline import prepare_points
from online_control.online_motion_track.main import WebSession


def percentiles(values):
    return np.percentile(values, [50, 95, 100]).tolist() if len(values) else []


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(); args.output.mkdir(parents=True, exist_ok=True)
    rows, counts = defaultdict(list), Counter()
    issues = 0; issue_example = None
    keep = {"session", "session_end", "mediapipe_raw", "mediapipe", "retarget", "control_target",
            "planner_sample", "h7", "trajectory_recovery", "trajectory_recovered", "retarget_skipped"}
    with args.source.open(encoding="utf-8-sig") as source:
        for line in source:
            row = json.loads(line); kind = row["type"]; counts[kind] += 1
            if kind in keep:
                rows[kind].append(row)
            if kind == "telemetry" and row.get("decode_issues"):
                issues += 1; issue_example = issue_example or row["decode_issues"][:6]
    meta = rows["session"][0]["metadata"]
    if meta["model_id"] != "TsenseI_v3_0" or meta["hand_side"] != "left":
        raise ValueError("This analysis expects the left A3.0 recording")
    raw = {r["frame_id"]: r for r in rows["mediapipe_raw"]}
    target = {r["frame_id"]: r for r in rows["retarget"]}
    names = list(rows["retarget"][0]["joints"])
    aligned = []
    for r in rows["mediapipe"]:
        original = raw[r["frame_id"]]
        index = next((i for i, h in enumerate(original["handedness"]) if h and h[0]["label"] == "Left"), None)
        if index is None:
            continue
        aligned.append((r["t"], r["frame_id"], prepare_points(original["world_landmarks"][index], "left"),
                        np.asarray(r["landmarks"]), [target[r["frame_id"]]["joints"][n] for n in names]))
    times = np.array([x[0] for x in aligned]); ids = np.array([x[1] for x in aligned])
    original = np.array([x[2] for x in aligned]); filtered = np.array([x[3] for x in aligned])
    angles = np.array([x[4] for x in aligned])
    mismatch = np.max(np.linalg.norm(original-filtered, axis=2), axis=1)*1000
    resets = np.flatnonzero(mismatch < 1e-6)
    jumps = np.max(np.abs(np.diff(angles, axis=0)), axis=1)*180/np.pi
    reset_steps = resets[resets > 0]-1
    ordinary = np.setdiff1d(np.arange(len(jumps)), reset_steps)
    recoveries = rows["trajectory_recovery"]; completed = rows["trajectory_recovered"]
    sent = [r for r in rows["h7"] if r.get("sent") and r.get("source") == "shared_binary_stream"]
    rates, backlog = [], []
    for start, end in zip(completed, recoveries[1:]):
        selected = [r for r in sent if start["t"] <= r["t"] < end["t"]]
        if len(selected) < 3:
            continue
        elapsed = selected[-1]["t"]-selected[0]["t"]
        rates.append((len(selected)-1)/elapsed)
        backlog.append((elapsed-sum(r["control_dt_s"] for r in selected[1:]))*1000)
    # Rank complete 2-second windows by RMS coordinate variation after wrist
    # alignment. This finds stable articulation, not ground-truth immobility.
    candidates = []
    for lo in np.arange(np.ceil(times[0]+1), times[-1]-2, .5):
        mask = (times >= lo) & (times < lo+2)
        p = original[mask]
        if len(p) < 45 or np.max(np.diff(times[mask])) > .1:
            continue
        score = float(np.sqrt(np.mean((p-p.mean(0))**2))*1000)
        candidates.append((score, float(lo), float(lo+2)))
    selected_windows = []
    for score, lo, hi in sorted(candidates):
        if all(hi <= a or lo >= b for a,b in selected_windows):
            selected_windows.append((lo,hi))
        if len(selected_windows) == 5:
            break
    windows = []
    for lo, hi in sorted(selected_windows):
        mask = (times >= lo) & (times < hi)
        metrics = {"range_s": [lo,hi], "stationary_status": "unconfirmed low-motion candidate"}
        for label, data in (("raw", original), ("filtered", filtered)):
            p = data[mask]
            metrics[label+"_coordinate_std_rms_mm"] = float(np.sqrt(np.mean((p-p.mean(0))**2))*1000)
        for label, field in (("retarget", "joints"), ("control_target", "joints"), ("h7", "joints_ui")):
            stage = sent if label == "h7" else rows[label]
            q = np.array([[r[field][n] for n in names] for r in stage if lo <= r["t"] < hi])
            stdev = q.std(0)*180/np.pi; worst = int(stdev.argmax())
            metrics[label] = {"samples": len(q), "largest_std_joint": names[worst],
                              "std_deg": float(stdev[worst]),
                              "peak_to_peak_deg": float(np.ptp(q[:,worst])*180/np.pi)}
        windows.append(metrics)

    # A fully constant observation isolates resetting the estimator from camera
    # noise, hand tremor, network pacing, USB and actuator behavior.
    fixed = next(r for r in rows["mediapipe_raw"] if r["t"] >= 37.1 and r["handedness"]
                 and r["handedness"][0][0]["label"] == "Left")
    ablations = {}
    for mode in ("none", "filters_only", "solver_only", "both"):
        session = WebSession("left", False, collision_mode="mink", model_id="TsenseI_v3_0")
        values = []
        try:
            for i in range(120):
                if i == 80:
                    if mode in ("filters_only", "both"):
                        session.filter.reset(); session.normalized_filter.reset()
                    if mode in ("solver_only", "both"):
                        session.pipeline.session.retargeter.reset()
                response = session.process({"sequence": i, "timestamp_ms": 1000+i*1000/30, "capture_unix_ms": 0,
                                            "hand": {"label": "Left", "world": fixed["world_landmarks"][0],
                                                     "normalized": fixed["normalized_landmarks"][0]}})
                if not response["tracking"]:
                    raise RuntimeError(response["message"])
                values.append([response["retarget_joint_angles_rad"][n] for n in names])
        finally:
            session.close()
        values = np.asarray(values)
        ablations[mode] = float(np.max(np.abs(values[80]-values[79]))*180/np.pi)
        if mode == "both":
            poses = values
    delta = np.abs(poses[80]-poses[79])*180/np.pi; worst = int(delta.argmax())
    result = {"source": str(args.source), "counts": dict(counts), "metadata": meta,
              "recording_end": rows["session_end"], "recoveries": len(recoveries),
              "recovery_reasons": dict(Counter(r["reason"] for r in recoveries)),
              "recovery_interval_s_p50_p95_max": percentiles(np.diff([r["t"] for r in recoveries])),
              "recovery_duration_ms_p50_p95_max": percentiles([r["duration_ms"] for r in completed]),
              "write_rate_hz_p50_p95_max": percentiles(rates),
              "per_recovery_queue_growth_ms_p50_p95_max": percentiles(backlog),
              "raw_equals_filtered_frames": [{"t": float(times[i]), "frame_id": int(ids[i])} for i in resets],
              "retarget_max_joint_step_deg_reset_p50_p95_max": percentiles(jumps[reset_steps]),
              "retarget_max_joint_step_deg_other_p50_p95_max": percentiles(jumps[ordinary]),
              "window_selection": "Lowest raw wrist-aligned coordinate RMS variation; 2 s length, .5 s stride, five nonoverlapping windows; >=45 frames; gaps <=100 ms",
              "windows": windows, "planner_held_samples": sum(bool(r.get("held_fingers")) for r in rows["planner_sample"]),
              "telemetry_rows_with_decode_issues": issues, "telemetry_issue_example": issue_example,
              "constant_input_experiment": {"source_frame_id": fixed["frame_id"], "reset_frame": 80,
                  "scope": "identical input repeated at 30 Hz; reset only filters and retarget history; no hardware rebase",
                  "ablation_max_jump_deg": ablations,
                  "joint": names[worst], "reset_jump_deg": float(delta[worst]),
                  "previous_10_frames_max_step_deg": float(np.max(np.abs(np.diff(poses[70:80],axis=0)))*180/np.pi)}}
    (args.output/"metrics.json").write_text(json.dumps(result,ensure_ascii=False,indent=2)+"\n",encoding="utf-8")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(4, 1, figsize=(12,12), layout="constrained")
    mask = (times >= 37) & (times < 39)
    for label, data in (("Raw",original),("Filtered",filtered)):
        axes[0].plot(times[mask],data[mask,8,0]*1000,label=label)
    axes[0].set(ylabel="Index tip X (mm)",title="37–39 s: low-motion candidate, not independently confirmed stationary")
    thumb = names.index("left_hand_thumb_rota_block_joint")
    axes[1].plot(times[mask],angles[mask,thumb]*180/np.pi,color="#d56c41",label="Retarget thumb base rotation")
    axes[1].set(ylabel="Target (deg)")
    h = [r for r in sent if 37 <= r["t"] < 39]
    axes[2].plot([r["t"] for r in h],[r["joints_ui"]["left_hand_mid_swing_joint"]*180/np.pi for r in h],
                 color="#189b88",label="Successful USB command: middle swing")
    axes[2].set(ylabel="Sent angle (deg)",xlabel="Recording time (s)")
    for ax in axes[:3]:
        for r in recoveries:
            if 37 <= r["t"] < 39:
                ax.axvline(r["t"],color="gray",ls=":",label="Stream recovery" if r == next(x for x in recoveries if x["t"]>=37) else None)
    axes[3].plot(np.arange(65,100)/30,poses[65:100,worst]*180/np.pi,label="Constant input: thumb base rotation")
    axes[3].axvline(80/30,color="#d56c41",ls="--",label="Clear filter / solver history")
    axes[3].set(title="Independent reproduction: identical 21 landmarks on every frame",
                ylabel="Retarget (deg)",xlabel="Experiment time (s)")
    for ax in axes:
        ax.grid(alpha=.2); ax.legend(loc="best")
    fig.savefig(args.output/"diagnosis.png",dpi=150);plt.close(fig)
    print(json.dumps({k:result[k] for k in ("recoveries","recovery_reasons","retarget_max_joint_step_deg_reset_p50_p95_max",
                                         "retarget_max_joint_step_deg_other_p50_p95_max","constant_input_experiment")},indent=2))


if __name__ == "__main__":
    main()
