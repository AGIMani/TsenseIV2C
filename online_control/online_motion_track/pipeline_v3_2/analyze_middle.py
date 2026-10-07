"""Plot middle-finger logs and replay recorded 3-D inputs with current tuning.

Run from the repository: python online_control/online_motion_track/pipeline_v3_2/analyze_middle.py SESSION.jsonl
Angles are geometric human measurements versus URDF coordinates, not identical
anatomical quantities. Latency is measured on capture timestamps, not glass-to-glass.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from hand_profiles import load_hand_profile
from TsenseI_retargeting import TopoRetargetConfig, TopoRetargeter
from online_control.online_motion_track.pipeline_v3_2.recording import json_safe
from online_control.online_motion_track.pipeline_v3_2.skeleton_angles import degrees, skeleton_joint_angles


def extension_events(t, raw, threshold=45.0):
    """Require a clear fist before the crossing and sustained extension after it."""
    events = []
    for i in range(3, len(raw) - 2):
        before = (t >= t[i] - .5) & (t < t[i])
        if (raw[i - 1] >= threshold > raw[i]
                and before.sum() >= 3 and np.median(raw[before]) > 65
                and np.all(raw[i:i + 3] < threshold)
                and (not events or t[i] - t[events[-1]] > 2)):
            events.append(i)
    return events


def crossing_delay(t, angles, start, threshold=45.0):
    for j in range(start, len(angles) - 2):
        if t[j] - t[start] > 2:
            break
        if np.all(angles[j:j + 3] < threshold):
            return float(t[j] - t[start])
    return None


def replay(config, frames, names):
    solver = TopoRetargeter(config)
    poses, durations = [], []
    for frame in frames:
        start = time.perf_counter()
        result = solver.retarget(np.asarray(frame["topo_input_points_21_m"]), include_artifacts=False)
        durations.append((time.perf_counter() - start) * 1000)
        if not result.success:
            raise RuntimeError(f"Replay failed at frame {frame['frame_id']}: {result.diagnostics}")
        poses.append([result.joint_values[n] for n in names])
    return np.asarray(poses), np.asarray(durations)


def analyze(log, out):
    rows = [json.loads(line) for line in log.read_text(encoding="utf-8").splitlines() if line.strip()]
    header = rows[0]
    frames = [r for r in rows if r["type"] == "frame"]
    valid = [r for r in frames if r["retarget_success"] and r["topo_input_points_21_m"] is not None]
    if len(valid) < 6:
        raise ValueError("The session has too few successful retarget frames")
    profile = load_hand_profile(header["model_id"])
    side = header["hand_side"]
    # A changed model or calibration makes a before/after replay unfair.
    for key, digest in (("urdf", "urdf_sha256"), ("keypoints", "calibration_sha256")):
        if hashlib.sha256(profile.path(key, side).read_bytes()).hexdigest() != header[digest]:
            raise ValueError(f"Current {key} differs from this recording; restore the matching assets before replay")
    current = profile.create_retargeter(side).config
    assets = {k: getattr(current, k) for k in ("robot_root", "urdf_path", "keypoints_path", "joint_names")}
    recorded_options = {k: v for k, v in header.get("retarget_config", {}).items() if k not in assets}
    recorded_config = TopoRetargetConfig(**assets, **recorded_options)
    names = header["joint_names"]
    old = np.asarray([[r["retarget_joint_angles_rad"][n] for n in names] for r in valid])
    displayed = np.asarray([[r["output_joint_angles_rad"][n] for n in names] for r in valid])
    reproduced, old_ms = replay(recorded_config, valid, names)
    error = float(np.max(np.abs(reproduced - old)))
    if error > 1e-5:
        raise ValueError(f"Recorded solver state could not be reproduced (max error {error:.6g} rad)")
    fixed, new_ms = replay(current, valid, names)
    t0 = frames[0]["capture_timestamp_unix_ns"]
    t = (np.asarray([r["capture_timestamp_unix_ns"] for r in valid], dtype=np.int64) - t0) * 1e-9
    human_keys = ["middle_MCP_bend", "middle_PIP", "middle_DIP", "middle_swing"]
    model_keys = [f"{side}_hand_mid_{j}_joint" for j in ("MCP", "PIP", "DIP", "swing")]
    raw_dicts = [degrees(skeleton_joint_angles(r["raw_world_points_21_m"])) or {} for r in valid]
    raw = np.asarray([[r.get(k, np.nan) for k in human_keys] for r in raw_dicts], dtype=float)
    smoothed = np.asarray([[(r["skeleton_joint_angles_deg"] or {}).get(k, np.nan)
                            for k in human_keys] for r in valid], dtype=float)
    columns = [names.index(n) for n in model_keys]
    old_deg, fixed_deg, display_deg = [np.degrees(q[:, columns]) for q in (old, fixed, displayed)]
    events = extension_events(t, raw[:, 1])
    event_metrics = []
    for i in events:
        delays = {label: crossing_delay(t, series, i) for label, series in (
            ("landmark_smoothing_delay_s", smoothed[:, 1]),
            ("recorded_PIP_delay_s", old_deg[:, 1]), ("fixed_PIP_delay_s", fixed_deg[:, 1]))}
        event_metrics.append({"frame_id": valid[i]["frame_id"], "raw_crossing_time_s": float(t[i]), **delays,
                              "recorded_PIP_below_30_delay_s": crossing_delay(t, old_deg[:, 1], i, 30),
                              "fixed_PIP_below_30_delay_s": crossing_delay(t, fixed_deg[:, 1], i, 30)})
    metrics = {
        "source_log": str(log), "recorded_frames": len(frames), "replayed_frames": len(valid),
        "duration_s": (frames[-1]["capture_timestamp_unix_ns"] - t0) * 1e-9,
        "recorded_config_assumed_legacy_defaults": "retarget_config" not in header,
        "baseline_reproduction_max_error_rad": error,
        "output_vs_retarget_max_error_rad": float(np.max(np.abs(displayed - old))),
        "recorded_config": asdict(recorded_config), "fixed_config": asdict(current),
        "threshold_deg": 45, "sustained_samples": 3, "events": event_metrics,
        "replay_solver_median_ms": {"recorded": float(np.median(old_ms)), "fixed": float(np.median(new_ms))},
        "notes": ["Same stored filtered 3-D inputs; MediaPipe and GUI are not rerun.",
                  "45-degree crossing held for 3 successful frames; includes landmark smoothing delay.",
                  "Human geometric angles and URDF joint angles have different definitions.",
                  "Invalid/unselected-hand frames are omitted; this is not physical end-to-end latency."],
    }
    out.mkdir(parents=True, exist_ok=True)
    (out / "metrics.json").write_text(json.dumps(json_safe(metrics), indent=2, ensure_ascii=False), encoding="utf-8")
    with (out / "middle_angles.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.writer(stream)
        groups = ("human_raw", "human_smoothed", "recorded_retarget", "recorded_output", "fixed_replay")
        writer.writerow(["frame_id", "time_s"] + [f"{g}.{j}.deg" for g in groups for j in ("MCP", "PIP", "DIP", "swing")])
        for i, frame in enumerate(valid):
            writer.writerow([frame["frame_id"], t[i]] + [v for a in (raw, smoothed, old_deg, display_deg, fixed_deg) for v in a[i]])

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
    available = {f.name for f in font_manager.fontManager.ttflist}
    plt.rcParams["font.sans-serif"] = [n for n in ("Microsoft YaHei", "SimHei", "DejaVu Sans") if n in available]
    plt.rcParams["axes.unicode_minus"] = False
    colors = ("#8b929b", "#2277bd", "#d94c45", "#151b26", "#12966b")
    labels = ("骨架原始角", "骨架平滑角", "修复前 retarget", "记录的最终输出", "修复后回放")

    def curves(ax, column):
        for arr, color, label, style, width in zip(
                (raw, smoothed, old_deg, display_deg, fixed_deg), colors, labels,
                ("-", "-", "-", ":", "-"), (1, 1.5, 2.2, 1.1, 2.1)):
            # Break curves when rejected or missing observations interrupt tracking.
            x, y = [], []
            for i, frame in enumerate(valid):
                if i and frame["frame_id"] != valid[i - 1]["frame_id"] + 1:
                    x.append(np.nan); y.append(np.nan)
                x.append(t[i]); y.append(arr[i, column])
            ax.plot(x, y, color=color, label=label, ls=style, lw=width)
        ax.grid(alpha=.2)
        ax.set_ylabel("角度 / °")

    fig, axes = plt.subplots(4, 1, figsize=(13, 12), sharex=True, layout="constrained")
    for col, (ax, name) in enumerate(zip(axes, ("MCP / 掌指（骨架值含侧摆）", "PIP / 近端指间", "DIP / 远端指间", "swing / 侧摆（定义不同）"))):
        curves(ax, col)
        ax.set_title(name, loc="left", fontsize=11)
        for event in event_metrics:
            ax.axvline(event["raw_crossing_time_s"], color="#8993a0", ls="--", lw=.8)
    axes[0].legend(ncol=3, fontsize=9, loc="upper left")
    axes[-1].set_xlabel("距首个采集帧的时间 / 秒")
    fig.suptitle("V3.2 中指：原始检测 → 平滑 → retarget → 最终输出\n骨架几何角与 URDF 角并非一一等价；绿线为相同输入的离线回放", fontsize=14)
    for ext in ("png", "pdf"):
        fig.savefig(out / f"middle_curves.{ext}", dpi=170)
    plt.close(fig)
    if events:
        fig, axes = plt.subplots(len(events), 1, figsize=(12, 4 * len(events)), squeeze=False, layout="constrained")
        for ax, event in zip(axes[:, 0], event_metrics):
            curves(ax, 1)
            start = event["raw_crossing_time_s"]
            ax.set_xlim(start - .45, start + 1.65)
            ax.axhline(45, ls="--", color="#999999", lw=.9)
            ax.axvline(start, color="#777777", ls="--", lw=1)
            for key, color in (("recorded_PIP_delay_s", colors[2]), ("fixed_PIP_delay_s", colors[4])):
                delay = event[key]
                if delay is not None:
                    j = int(np.argmin(np.abs(t - start - delay)))
                    series = old_deg if key.startswith("recorded") else fixed_deg
                    ax.scatter([t[j]], [series[j, 1]], color=color, s=45, zorder=5)
                    ax.axvline(t[j], color=color, ls=":", lw=.8)
            old_delay, new_delay = event["recorded_PIP_delay_s"], event["fixed_PIP_delay_s"]
            delay_text = lambda value: "未达到" if value is None else f"{value:.3f}s"
            ax.set_title(f"动作起点 {start:.2f}s：PIP 降到 45° 的延迟 "
                         f"{delay_text(old_delay)} → {delay_text(new_delay)}", fontsize=11)
            ax.set_xlabel("距首个采集帧的时间 / 秒")
        axes[0, 0].legend(ncol=3, fontsize=9)
        fig.suptitle("握拳后伸中指：两次动作局部放大", fontsize=15)
        for ext in ("png", "pdf"):
            fig.savefig(out / f"middle_extension_zoom.{ext}", dpi=170)
        plt.close(fig)
    print(json.dumps(json_safe({"output": str(out), "events": event_metrics,
                               "baseline_error_rad": error, "solver_ms": metrics["replay_solver_median_ms"]}), indent=2, ensure_ascii=False))
    return metrics


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    log = args.log.resolve()
    analyze(log, (args.output_dir or log.parent / (log.stem + "_analysis_v3_2")).resolve())
