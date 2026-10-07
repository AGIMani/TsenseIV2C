"""Replay recorded control targets without USB, using the production A3.0 path.

Usage: python tools/replay_direct_filter.py path/to/session.jsonl --output logs/direct_filter
The source is never modified. Generated filter events are explicitly offline,
sent=false; browser receipt times approximate target arrival, not device times.
"""
import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from hand_profiles import load_hand_profile
from online_control.online_motion_track.a30.control import A30Control
from online_control.online_motion_track.direct_filter import DirectJointFilter


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--output", type=Path, default=ROOT / "logs/direct_filter")
    args = parser.parse_args()
    rows = [json.loads(s) for s in args.source.read_text(encoding="utf-8-sig").splitlines() if s.strip()]
    meta = next(r for r in rows if r.get("type") == "session")
    if meta["metadata"]["model_id"] != "TsenseI_v3_0":
        raise ValueError("This replay currently validates A3.0 only")
    targets = sorted((r for r in rows if r.get("type") == "control_target"), key=lambda r: r["t"])
    side = meta["metadata"].get("hand_side", "left")
    control = A30Control(load_hand_profile("TsenseI_v3_0"), side, "stop", start=False)
    control.set_direct_mode(True)
    dt = .002
    grid = np.arange(targets[0]["t"], targets[-1]["t"] + .1, dt)
    names = control.names
    raw, candidate, checked, costs, filter_costs, events = [], [], [], [], [], []
    at = -1
    probe = DirectJointFilter(); probe.reset(control.pose)
    try:
        for tick, stamp in enumerate(grid):
            while at+1 < len(targets) and targets[at+1]["t"] <= stamp+1e-9:
                at += 1
                control.push(targets[at]["joints"], targets[at]["frame_id"])
            t0 = time.perf_counter_ns()
            probe.sample(control.target, dt)
            filter_costs.append((time.perf_counter_ns()-t0)/1e6)
            t0 = time.perf_counter_ns()
            control.step(dt, sample_at=float(stamp))
            costs.append((time.perf_counter_ns()-t0)/1e6)
            if not control.valid:
                raise RuntimeError(f"Replay tick {tick}: {control.error}")
            raw.append([control.target[n] for n in names])
            candidate.append([control.direct_filter.filtered[n] for n in names])
            checked.append([control.pose[n] for n in names])
            base = {"t": float(stamp), "frame_id": control.frame_id, "hand_side": side,
                    "sample_ticks": tick+1, "source": "offline_replay", "sent": False,
                    "direct_mode": True, "ruckig_enabled": False}
            events.append({**base, "type": "direct_filter", "stage": "checked", "joints": dict(control.pose)})
            if tick % 50 == 0:
                events.append({**base, "type": "direct_filter_detail", **control.direct_filter.snapshot()})
    finally:
        control.close()
    raw, candidate, checked = map(np.asarray, (raw, candidate, checked))
    # Ignore initial zero-to-first-pose acquisition when measuring motion lag.
    start = 250
    metrics = []
    for i, name in enumerate(names):
        x, y = raw[start:, i], candidate[start:, i]
        if np.ptp(x) < .01:
            continue
        lag = min(range(26), key=lambda k: np.mean((x[:len(x)-k or None]-y[k:])**2))
        vx, vy = np.diff(x)/dt, np.diff(y)/dt
        metrics.append({"joint": name, "best_fit_lag_ms": lag*dt*1000,
            "rms_error_deg": float(np.rad2deg(np.sqrt(np.mean((x-y)**2)))),
            "sample_velocity_rms_reduction_percent": float(100*(1-np.linalg.norm(vy)/np.linalg.norm(vx))),
            "peak_step_before_deg": float(np.rad2deg(np.max(np.abs(np.diff(x))))),
            "peak_step_after_deg": float(np.rad2deg(np.max(np.abs(np.diff(y)))) )})
    step_tests = []
    for amplitude in (.01, .1, .5, 1.):
        f = DirectJointFilter(); f.reset({"q": 0.})
        values = [f.sample({"q": amplitude}, dt)["q"] for _ in range(100)]
        step_tests.append({"amplitude_rad": amplitude,
            **{f"t{p}_ms": next(i+1 for i, q in enumerate(values) if q >= amplitude*p/100)*dt*1000 for p in (50,90,95)}})
    result = {"source": str(args.source), "scope": "offline replay; no USB or device latency measured",
        "target_frames": len(targets), "source_duration_s": targets[-1]["t"]-targets[0]["t"],
        "source_usb_writes": sum(r.get("type")=="downlink" and r.get("sent",False) for r in rows),
        "sample_hz": 500, "filter": DirectJointFilter.CONFIG,
        "timing_assumption": "target arrival uses browser control_target.t; quantized to 2 ms; initial pose is model zero",
        "lag_measurement": "min MSE delay on held input vs filtered candidate at 500 Hz, first 0.5 s excluded; 2 ms resolution",
        "smoothness_measurement": "RMS command velocity reduction on 500 Hz held targets; not ground-truth sensor noise rejection",
        "filter_compute_ms_median_p95": np.percentile(filter_costs, [50,95]).tolist(),
        "full_step_compute_ms_median_p95": np.percentile(costs, [50,95]).tolist(),
        "collision_modified_samples": int(np.sum(np.max(np.abs(candidate-checked), axis=1)>1e-8)),
        "max_collision_correction_deg": float(np.rad2deg(np.max(np.abs(candidate-checked)))),
        "step_response": step_tests, "joints": metrics}
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "metrics.json").write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding="utf-8")
    copied = [dict(r) for r in rows if r.get("type") != "session_end"]
    session = next(r for r in copied if r.get("type") == "session")
    session["metadata"] = {**session["metadata"], "offline_replay": True,
        "direct_filter": DirectJointFilter.CONFIG, "replay_source": str(args.source),
        "replay_note": "仅直接滤波事件为500Hz离线模拟；其余保留原日志，不代表真实串口下发"}
    combined = sorted(copied + events, key=lambda r:r["t"])
    with (args.output / "filtered_replay.jsonl").open("w",encoding="utf-8") as out:
        for seq, row in enumerate(combined, 1):
            out.write(json.dumps({**row,"source_seq":row.get("seq"),"seq":seq},ensure_ascii=False,separators=(",",":"))+"\n")
        out.write(json.dumps({"type":"session_end","t":float(grid[-1]),"seq":len(combined)+1,"reason":"offline_replay_complete"})+"\n")
    viewer = (ROOT / "tools/hand_control_log_viewer.html").read_text(encoding="utf-8")
    payload = json.dumps((args.output / "filtered_replay.jsonl").read_text(encoding="utf-8"),ensure_ascii=False).replace("<","\\u003c")
    bootstrap = ('<script id="replay-data" type="application/json">'+payload+'</script><script>'
        'resetData();for(const line of JSON.parse(document.getElementById("replay-data").textContent).trim().split("\\n"))consume(JSON.parse(line));'
        '$("finger").value="mid";$("stageRetarget").checked=false;$("stageH7").checked=false;$("stageControl").checked=true;'
        'selectLoadedData();$("status").textContent="已加载离线滤波回放 · 未写入真机";'
        '</script>')
    (args.output / "report.html").write_text(viewer.replace("</body>",bootstrap+"</body>"),encoding="utf-8")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(3, 1, figsize=(13,9), constrained_layout=True)
    for ax, suffix in zip(axes, ("mid_MCP_joint","mid_PIP_joint","mid_DIP_joint")):
        i = names.index(f"{side}_hand_{suffix}")
        ax.plot(grid-grid[0], np.rad2deg(raw[:,i]), color="#aab5be", lw=1,label="Input (held target)")
        ax.plot(grid-grid[0], np.rad2deg(candidate[:,i]), color="#07889b", lw=1.4,label="Filtered candidate")
        ax.plot(grid-grid[0], np.rad2deg(checked[:,i]), color="#e08d2e", ls="--",lw=.8,label="Collision-checked output")
        ax.set(title=suffix, xlabel="Replay time (s)", ylabel="Angle (deg)"); ax.grid(alpha=.2); ax.legend(loc="upper right")
    fig.suptitle("A3.0 direct-path filter replay (no physical downlink)")
    fig.savefig(args.output / "middle_finger.png",dpi=150)
    print(json.dumps(result,ensure_ascii=False,indent=2))


if __name__ == "__main__":
    main()
