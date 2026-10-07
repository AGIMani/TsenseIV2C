"""Read-only latency analysis of hand-control-session/v2 JSONL recordings.

All times are server recording times. They are not camera-to-screen latency.
The optional Windows wait benchmark is confined to this analysis process.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import ctypes
import hashlib
import json
from pathlib import Path
import sys
import threading
import time

import numpy as np


def stats(values):
    values = np.asarray(values, dtype=float)
    return dict(zip(("min", "p50", "p95", "max"),
                    map(float, np.percentile(values, [0, 50, 95, 100])))) if values.size else {}


def wait_benchmark():
    event = threading.Event()
    result = {}
    for precise in (False, True) if sys.platform == "win32" else (False,):
        enabled = precise and ctypes.windll.winmm.timeBeginPeriod(1) == 0
        try:
            samples = []
            for _ in range(140):
                start = time.perf_counter()
                event.wait(.002)
                samples.append((time.perf_counter()-start)*1000)
            result["timer_1ms" if enabled else "default"] = stats(samples)
        finally:
            if enabled:
                ctypes.windll.winmm.timeEndPeriod(1)
    return result


def analyze(path, output, nominal_dt, benchmark):
    events = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines() if line.strip()]
    groups = defaultdict(list)
    for event in events:
        groups[event["type"]].append(event)
    if not all(groups[k] for k in ("retarget", "control_target", "h7")):
        raise ValueError("Recording requires retarget, control_target and h7 events")
    output.mkdir(parents=True, exist_ok=True)
    raw, targets, samples = (groups[k] for k in ("retarget", "control_target", "h7"))
    raw_by_id = {x["frame_id"]: x for x in raw}
    ht = np.array([x["t"] for x in samples])
    duration = float(ht[-1]-ht[0])
    metrics = {
        "source": str(path.resolve()), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
        "started_at": groups["session"][0].get("started_at"),
        "event_counts": {k: len(v) for k, v in groups.items()},
        "solve_ms": stats([x["solve_ms"] for x in raw]),
        "inference_ms": stats([x["infer_ms"] for x in groups["mediapipe_raw"]]),
        "inference_after_first_frame_ms": stats([x["infer_ms"] for x in groups["mediapipe_raw"][1:]]),
        "event_intervals_ms": {k: stats(np.diff([x["t"] for x in groups[k]])*1000)
                               for k in ("mediapipe_raw", "retarget", "h7")},
        "sample_wall_duration_s": duration,
        "sample_rate_hz": (len(samples)-1)/duration,
        "nominal_ruckig_dt_s_assumption": nominal_dt,
        "sample_implied_trajectory_duration_s": (len(samples)-1)*nominal_dt,
        "wall_to_trajectory_ratio": duration/((len(samples)-1)*nominal_dt),
        "sent_samples": sum(bool(x.get("sent")) for x in samples),
        "connected_samples": sum(bool(x.get("usb_connected")) for x in samples),
        "fault_samples": sum(bool(x.get("ruckig_faults")) for x in samples),
        "guard_limited_samples": sum(bool(x.get("guard_limited")) for x in samples),
        "collision_resolutions": dict(Counter(x["self_collision"]["resolution"] for x in targets)),
        "collision_contact_fingers": dict(Counter("/".join(x["self_collision"]["contact_fingers"])
                                for x in targets if x["self_collision"]["limited"])),
    }
    metrics["stage_ms"] = {}
    for before, after in (("mediapipe", "retarget"), ("retarget", "control_target")):
        index = {x["frame_id"]: x for x in groups[before]}
        metrics["stage_ms"][f"{before}_to_{after}"] = stats(
            [(x["t"]-index[x["frame_id"]]["t"])*1000 for x in groups[after] if x["frame_id"] in index])
    metrics["collision_max_joint_correction_deg"] = {
        joint: float(np.degrees(max(abs(x["joints"][joint]-raw_by_id[x["frame_id"]]["joints"][joint])
                                   for x in targets))) for joint in targets[0]["joints"]}
    # Matching recorded angle thresholds is interpretable, but is NOT the total
    # camera/display latency. Only analyze the supplied gesture's two transitions.
    metrics["middle_pip_crossings"] = []
    joint = "left_hand_mid_PIP_joint"
    if joint in targets[0]["joints"]:
        for start, end, threshold, rising in ((10, 13, 50, True), (13.5, 19, 40, False), (13.5, 19, 30, False)):
            crossing = {"window_s": [start, end], "threshold_deg": threshold, "rising": rising}
            for kind, key in (("retarget", "joints"), ("control_target", "joints"), ("h7", "joints_ruckig_ui")):
                selected = [x for x in groups[kind] if start <= x["t"] <= end and
                            (np.degrees(x[key][joint]) >= threshold if rising else np.degrees(x[key][joint]) <= threshold)]
                crossing[kind] = selected[0]["t"] if selected else None
            if crossing["h7"] is not None and crossing["control_target"] is not None:
                crossing["target_to_sample_s"] = crossing["h7"]-crossing["control_target"]
            metrics["middle_pip_crossings"].append(crossing)
    if benchmark:
        metrics["isolated_wait_2ms_benchmark_ms"] = wait_benchmark()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.sans-serif": ["Microsoft YaHei", "SimHei", "DejaVu Sans"],
                         "axes.unicode_minus": False, "font.size": 10})
    colors = ("#198caa", "#e89b29", "#ce4567")

    def trace(ax, finger):
        name = f"left_hand_{finger}_PIP_joint"
        if name not in targets[0]["joints"]:
            name = name.replace("left_", "right_")
        for records, key, label, color, linestyle in (
                (raw, "joints", "retarget 原始目标", colors[0], "-"),
                (targets, "joints", "碰撞规划后目标", colors[1], "--"),
                (samples, "joints_ruckig_ui", "Ruckig 输出（服务端采样）", colors[2], "-")):
            ax.plot([x["t"] for x in records], np.degrees([x[key][name] for x in records]),
                    label=label, color=color, linestyle=linestyle, linewidth=1.7)
        ax.set_ylabel("PIP 角度 / °")
        ax.grid(alpha=.18)

    fig, axes = plt.subplots(4, 1, figsize=(13, 10), sharex=True, layout="constrained")
    for ax, finger, title in zip(axes, ("index", "mid", "ring", "pinky"), ("食指", "中指", "无名指", "小指")):
        trace(ax, finger)
        ax.set_title(title, loc="left")
    axes[0].legend(loc="upper right", ncol=3)
    axes[-1].set_xlabel("记录开始后的时间 / s（服务器时钟）")
    fig.suptitle("V2 网页记录 · 2026-09-18 14:21:04\n目标角度、避碰修正与限速输出分开比较", fontsize=15)
    fig.savefig(output / "finger_angles.png", dpi=160)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(13, 10), layout="constrained")
    trace(axes[0], "mid")
    axes[0].set_xlim(9.5, 19)
    crossing = next((x for x in metrics["middle_pip_crossings"]
                     if x["threshold_deg"] == 40 and "target_to_sample_s" in x), None)
    title = "中指伸展：目标与限速输出对比"
    if crossing:
        start, end = crossing["control_target"], crossing["h7"]
        title = f"中指伸展：40° 阈值相差约 {end-start:.2f} s（目标 {start:.3f} s → 输出 {end:.3f} s）"
        axes[0].axhline(40, color="#555555", alpha=.4, linewidth=1)
        axes[0].annotate("", xy=(end, 40), xytext=(start, 40), arrowprops={"arrowstyle": "<->", "color": "#333333"})
    axes[0].set_title(title, loc="left")
    axes[0].legend(loc="lower left", ncol=3)
    axes[0].set_xlabel("记录开始后的时间 / s")
    for kind, label, color in (("mediapipe_raw", "识别结果到达间隔", colors[0]), ("h7", "控制采样间隔", colors[2])):
        times = np.array([x["t"] for x in groups[kind]])
        axes[1].plot(times[1:], np.diff(times)*1000, ".", color=color, markersize=2, alpha=.6, label=label)
    axes[1].axhline(nominal_dt*1000, color="#333333", linestyle="--", label=f"Ruckig 固定步长 {nominal_dt*1000:g} ms")
    axes[1].set_ylim(0, 105)
    axes[1].set_ylabel("间隔 / ms")
    axes[1].set_xlabel("记录开始后的时间 / s")
    axes[1].set_title("识别链路约 15 Hz；控制采样存在约 15 ms 等待与短间隔补采样", loc="left")
    axes[1].legend(loc="upper right", ncol=3)
    wall = ht-ht[0]
    axes[2].plot(wall, wall, "--", color=colors[0], label="真实经过时间")
    axes[2].plot(wall, np.arange(len(ht))*nominal_dt, color=colors[2], label=f"采样数 × {nominal_dt*1000:g} ms（推算推进量）")
    axes[2].set_xlabel("真实经过时间 / s")
    axes[2].set_ylabel("累计时间 / s")
    axes[2].set_title(f"{duration:.2f} s 内仅采样 {len(samples)} 次：固定步长累计约 {(len(samples)-1)*nominal_dt:.2f} s", loc="left")
    axes[2].legend()
    for ax in axes[1:]:
        ax.grid(alpha=.18)
    fig.suptitle("主要延迟发生在 retarget 后的轨迹跟随阶段", fontsize=15)
    fig.savefig(output / "middle_latency.png", dpi=160)
    plt.close(fig)
    (output / "metrics.json").write_text(json.dumps(metrics, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps({"output": str(output.resolve()), "sample_rate_hz": metrics["sample_rate_hz"],
                      "wall_to_trajectory_ratio": metrics["wall_to_trajectory_ratio"]}, ensure_ascii=False))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", type=Path)
    parser.add_argument("--output", type=Path, default=Path("logs/web_v2_latency_20260918_142104"))
    parser.add_argument("--nominal-dt", type=float, default=.002, help="Assumed runtime Ruckig delta time, in seconds")
    parser.add_argument("--benchmark-wait", action="store_true")
    args = parser.parse_args()
    if args.nominal_dt <= 0:
        parser.error("--nominal-dt must be positive")
    analyze(args.recording, args.output, args.nominal_dt, args.benchmark_wait)
