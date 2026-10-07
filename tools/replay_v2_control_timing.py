"""Isolate Ruckig time advancement with recorded safe targets and sample times.

No camera, ports or robot. This does not rerun collision planning or tracking
recovery; both policies use identical recorded targets and unchanged limits.
"""
import argparse
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from utils.ruckig_interpolator import PhaseSynchronizedRuckigInterpolator


def replay(path, output):
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    targets = [x for x in rows if x["type"] == "control_target"]
    recorded = [x for x in rows if x["type"] == "h7"]
    first, last = targets[0]["t"], targets[-1]["t"]
    old, fixed = PhaseSynchronizedRuckigInterpolator(), PhaseSynchronizedRuckigInterpolator()
    initial = recorded[0]["joints_ruckig_ui"]
    old.seed_ramp(initial, initial)
    fixed.seed_ramp(initial, initial)
    previous = None
    target = None
    traces = []
    for row in rows:
        if not first <= row["t"] <= last:
            continue
        if row["type"] == "control_target":
            target = row["joints"]
            old.push(target)
            fixed.push(target)
        elif row["type"] == "h7" and target:
            dt = .002 if previous is None else row["t"] - previous
            if not 0 < dt <= .1:
                raise ValueError("This replay only supports uninterrupted sampling (0 < dt <= 100 ms)")
            previous = row["t"]
            traces.append({"t": previous, "target": target, "recorded": row["joints_ruckig_ui"],
                           "old": old.sample(), "fixed": fixed.sample(delta_time=dt)})
            assert not old.last_faults and not fixed.last_faults

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.sans-serif": ["Microsoft YaHei", "DejaVu Sans"], "axes.unicode_minus": False})
    fig, axes = plt.subplots(4, 1, figsize=(12, 9), sharex=True, layout="constrained")
    summary = {"source": str(path), "collision": rows[0]["metadata"]["self_collision_mode"],
               "method": "Timing-only replay; fixed recorded safe goals, full speed, no planner/recovery rerun", "fingers": {}}
    for ax, finger, title in zip(axes, ("index", "mid", "ring", "pinky"), ("食指", "中指", "无名指", "小指")):
        joint = f"left_hand_{finger}_PIP_joint"
        data = {key: np.degrees([x[key][joint] for x in traces]) for key in ("target", "old", "fixed")}
        for key, label, color, line in (("target", "记录中的安全目标", "#333333", "--"),
                                        ("old", "旧固定 2 ms 回放", "#ce4567", "-"),
                                        ("fixed", "修复：按实际间隔回放", "#178cab", "-")):
            ax.plot([x["t"] for x in traces], data[key], color=color, linestyle=line, label=label)
        summary["fingers"][finger] = {key + "_tracking_rmse_deg": float(np.sqrt(np.mean((data[key]-data["target"])**2)))
                                       for key in ("old", "fixed")}
        ax.set_title(title, loc="left")
        ax.set_ylabel("PIP / °")
        ax.grid(alpha=.2)
    axes[0].legend(ncol=3)
    axes[-1].set_xlabel("原记录时间 / s")
    fig.suptitle(path.stem + "\n隔离计时因素回放：相同目标、相同限速；不代表完整真人复测", fontsize=12)
    output.mkdir(parents=True, exist_ok=True)
    fig.savefig(output / (path.stem + ".png"), dpi=150)
    plt.close(fig)
    (output / (path.stem + ".json")).write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recordings", nargs="+", type=Path)
    parser.add_argument("--output", type=Path, default=Path("logs/web_v2_timing_fix_20260918"))
    args = parser.parse_args()
    for recording in args.recordings:
        replay(recording, args.output)
