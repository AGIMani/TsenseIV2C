"""Offline A/B acceptance gate for the 5% landmark-filter latency budget.

Compare against the frozen PREVIOUS PRODUCTION filter, not the older legacy
filter. Synthetic ground truth measures signal delay independently of CPU time.
Recorded input exercises the real A3.0 session with control threads disabled;
camera, network, USB, firmware and actuator latency are outside this benchmark.
"""
import argparse
from functools import partial
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import time
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from online_control.online_motion_track.landmark_filter import ResponsiveLandmarkFilter

BASELINE = ROOT / "tests/fixtures/landmark_filter_before_20260923.py"
spec = importlib.util.spec_from_file_location("landmark_filter_baseline", BASELINE)
baseline_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(baseline_module)
BaselineFilter = baseline_module.ResponsiveLandmarkFilter
LIMIT = 1.05
WINDOWS = [(17, 19), (31.5, 33.5), (36, 38), (41, 43), (43.5, 45.5)]


def apply(factory, points, timestamps):
    f = factory()
    return np.array([f.filter(p, float(t)) for p, t in zip(points, timestamps)])


def ratio(after, before):
    if before <= 1e-12:
        return 1. if after <= 1e-12 else float("inf")
    return float(after / before)


def crossing_delay(t, y, target, fraction, onset):
    indices = np.flatnonzero((t >= onset) & (y >= target * fraction))
    if not len(indices):
        raise AssertionError("Step did not reach the requested fraction")
    i = indices[0]
    if t[i] == onset:
        return 0.  # Responded in the first available observation.
    at = t[i-1] + (t[i]-t[i-1]) * (target*fraction-y[i-1]) / (y[i]-y[i-1])
    return max(0., float(at-onset))


def signal_metrics(factory, rate):
    t = np.arange(0, 20, 1/rate)
    amplitudes = np.tile([.0002, .0005, .001, .002, .005, .01, .03], 3)
    frequencies = np.repeat([.25, .8, 2.], 7)
    points = np.zeros((len(t), 21, 3))
    points[:, :, 0] = amplitudes * np.sin(2*np.pi*frequencies*t[:, None])
    output = apply(factory, points, t)
    steady = t >= 8
    delays, gains = [], []
    for j, frequency in enumerate(frequencies):
        phase = 2*np.pi*frequency*t[steady]
        design = np.column_stack([np.sin(phase), np.cos(phase), np.ones(sum(steady))])
        a, b, _ = np.linalg.lstsq(design, output[steady, j, 0], rcond=None)[0]
        delays.append(max(0., float(-np.arctan2(b, a)/(2*np.pi*frequency))))
        gains.append(float(np.hypot(a, b)/amplitudes[j]))
    points[:] = 0
    points[t >= 2, :, 0] = amplitudes
    output = apply(factory, points, t)
    step = [crossing_delay(t, output[:, j, 0], amplitudes[j], fraction, 2.)
            for fraction in (.5, .9) for j in range(21)]
    overshoot = float(np.maximum(output[:, :, 0]-amplitudes, 0).max())
    points[:, :, 0] = amplitudes * np.maximum(0, t[:, None]-2)
    output = apply(factory, points, t)
    ramp = np.mean((points[steady, :, 0]-output[steady, :, 0])/amplitudes, axis=0)
    # Independent seed and colored noise cover more than the tuning sequence.
    rng = np.random.default_rng(872341 + rate)
    noise = rng.normal(0, .0005, points.shape)
    white = apply(factory, noise, t)
    colored_noise = noise.copy()
    for i in range(1, len(noise)):
        colored_noise[i] = .7*colored_noise[i-1] + np.sqrt(1-.7**2)*noise[i]
    colored = apply(factory, colored_noise, t)
    rest = t >= 2
    return {"sine_delay_s": delays, "sine_gain": gains, "step_delay_s": step,
            "ramp_delay_s": ramp.tolist(), "step_overshoot_m": overshoot,
            "white_noise_rmse_mm": float(np.sqrt(np.mean(white[rest]**2))*1000),
            "colored_noise_rmse_mm": float(np.sqrt(np.mean(colored[rest]**2))*1000)}


def signal_comparison():
    report = {}
    for rate in (15, 30, 60):
        before, after = signal_metrics(BaselineFilter, rate), signal_metrics(ResponsiveLandmarkFilter, rate)
        comparisons = {key: max(ratio(a, b) for a, b in zip(after[key], before[key]))
                       for key in ("sine_delay_s", "step_delay_s", "ramp_delay_s")}
        report[str(rate)] = {"before": before, "after": after, "worst_delay_ratios": comparisons,
                            "passes": max(comparisons.values()) <= LIMIT
                            and after["white_noise_rmse_mm"] < before["white_noise_rmse_mm"]
                            and after["colored_noise_rmse_mm"] < before["colored_noise_rmse_mm"]
                            and after["step_overshoot_m"] <= 1e-12}
    return report


def timing_summary(values):
    return {"p50_ms": float(np.median(values)), "p95_ms": float(np.percentile(values, 95))}


def cpu_benchmark():
    points = np.random.default_rng(937).normal(0, .003, (3000, 21, 3))
    samples = {"before": [], "after": []}
    factories = {"before": BaselineFilter, "after": ResponsiveLandmarkFilter}
    # Pair the filters on every frame so CPU frequency/load drift cannot favor
    # the first/last full run. Reverse order and discard warmup in every run.
    for run in range(10):
        filters = {name: factory() for name, factory in factories.items()}
        for i, p in enumerate(points):
            order = ("before", "after") if (i+run) % 2 else ("after", "before")
            for name in order:
                started = time.perf_counter_ns()
                filters[name].filter(p, i/30)
                duration = (time.perf_counter_ns()-started)/1e6
                if i >= 100:
                    samples[name].append(duration)
    return compare_timing(samples)


def compare_timing(samples):
    result = {name: timing_summary(values) for name, values in samples.items()}
    result["ratios"] = {key: ratio(result["after"][key], result["before"][key])
                        for key in ("p50_ms", "p95_ms")}
    result["passes"] = max(result["ratios"].values()) <= LIMIT
    return result


def load_packets(source):
    packets, elapsed = [], []
    with source.open(encoding="utf-8-sig") as handle:
        for line in handle:
            row = json.loads(line)
            if row["type"] == "session":
                metadata = row["metadata"]
                if metadata["model_id"] != "TsenseI_v3_0" or metadata["hand_side"] != "left":
                    raise ValueError("This replay expects a left A3.0 recording")
            if row["type"] != "mediapipe_raw":
                continue
            selected = next((i for i, h in enumerate(row["handedness"])
                             if h and h[0]["label"] == "Left"), None)
            hand = None if selected is None else {
                "label": "Left", "world": row["world_landmarks"][selected],
                "normalized": row["normalized_landmarks"][selected]}
            packets.append({"sequence": row["frame_id"], "timestamp_ms": row["timestamp_ms"],
                            "capture_unix_ms": 0, "hand": hand})
            elapsed.append(row["t"])
    if not packets:
        raise ValueError("Recording has no mediapipe_raw frames")
    return packets, np.asarray(elapsed)


def recorded_comparison(source, repeats):
    from online_control.online_motion_track.a30 import session as module
    packets, elapsed = load_packets(source)
    samples = {"before": [], "after": []}
    output = {"before": [], "after": []}
    rounds = []
    with patch.object(module, "A30Control", partial(module.A30Control, start=False)):
        for run in range(repeats):
            sessions = {name: module.A30WebSession(recording=False, collision_mode="mink")
                        for name in samples}
            sessions["before"].filter = BaselineFilter()
            current = {name: [] for name in samples}
            try:
                for i, packet in enumerate(packets):
                    order = ("before", "after") if (i+run) % 2 else ("after", "before")
                    for name in order:
                        started = time.perf_counter_ns()
                        result = sessions[name].process(packet)
                        duration = (time.perf_counter_ns()-started)/1e6
                        if i >= 30:
                            current[name].append(duration)
                        if packet["hand"] is not None and not result["tracking"]:
                            raise AssertionError(result["message"])
                        if run == 0:
                            output[name].append(result)
            finally:
                for s in sessions.values():
                    s.close()
            for name in samples:
                samples[name].extend(current[name])
            rounds.append(compare_timing(current))
            print(f"recorded round {run+1}/{repeats}: {rounds[-1]['ratios']}", flush=True)
    windows = []
    for low, high in WINDOWS:
        window = {"range_s": [low, high], "label": "unconfirmed low-motion interval"}
        for name in samples:
            rows = [r for t, r in zip(elapsed, output[name]) if low <= t < high and r["tracking"]]
            if len(rows) < 30:
                raise AssertionError("Insufficient samples in the predefined low-motion window")
            points = np.asarray([r["points"] for r in rows])
            names = list(rows[0]["retarget_joint_angles_rad"])
            joints = np.asarray([[r["retarget_joint_angles_rad"][k] for k in names] for r in rows])
            window[name] = {"samples": len(rows),
                            "point_std_rms_mm": float(np.sqrt(np.mean((points-points.mean(0))**2))*1000),
                            "joint_std_rms_deg": float(np.sqrt(np.mean(np.var(joints, axis=0)))*180/np.pi),
                            "joint_step_p95_deg": float(np.percentile(np.max(np.abs(np.diff(joints, axis=0)), axis=1), 95)*180/np.pi)}
        windows.append(window)
    timing = compare_timing(samples)
    return {"source": str(source), "frames": len(packets), "repeats": repeats,
            "server_process": timing, "rounds": rounds, "low_motion_windows": windows,
            "scope": "Offline A30WebSession.process including retarget, Mink, control.push and snapshot; no running control thread or physical I/O",
            "passes": timing["passes"] and all(r["passes"] for r in rounds)
            and all(w["after"]["point_std_rms_mm"] <= w["before"]["point_std_rms_mm"] for w in windows)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--repeats", type=int, default=4)
    args = parser.parse_args()
    if args.repeats < 2:
        parser.error("At least two alternating replay rounds are required")
    report = {"maximum_latency_ratio": LIMIT,
              "baseline_sha256": hashlib.sha256(BASELINE.read_bytes()).hexdigest(),
              "candidate_sha256": hashlib.sha256((ROOT / "online_control/online_motion_track/landmark_filter.py").read_bytes()).hexdigest(),
              "signal": signal_comparison(), "filter_cpu": cpu_benchmark(),
              "recorded": recorded_comparison(args.source, args.repeats),
              "physical_end_to_end_verified": False}
    report["passes_offline_gate"] = (all(r["passes"] for r in report["signal"].values())
                                     and report["filter_cpu"]["passes"] and report["recorded"]["passes"])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    print(json.dumps({"passes_offline_gate": report["passes_offline_gate"],
                      "filter_cpu": report["filter_cpu"],
                      "server_process": report["recorded"]["server_process"],
                      "signal_max_ratio": max(max(s["worst_delay_ratios"].values()) for s in report["signal"].values())}, indent=2))
    return 0 if report["passes_offline_gate"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
