"""Offline A30 compute benchmark. No server, browser, or physical serial I/O.

Run with the hand_dex Python; --output saves raw percentile measurements.
"""
from __future__ import annotations

import argparse
import cProfile
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import pstats
import random
import statistics
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src"), str(ROOT / "online_control/online_motion_play")]

import numpy as np
import ruckig
from ruckig import InputParameter, OutputParameter, Result, Ruckig
from a30_kinematics import A30StrokeMapper
from a30_device import A30Device
from online_control.shared.downlink import configure_ruckig, encode_control_targets


class Clock:
    value = 0.

    def __call__(self):
        return self.value


class MemoryPort:
    """Discard frames in memory; deliberately has no serial backend."""
    is_open = True
    in_waiting = 0

    def __init__(self, *args, **kwargs):
        self.count = 0

    def read(self, count):
        return b""

    def write(self, frame):
        self.count += 1
        return len(frame)

    def write_trajectory(self, frame, q):
        assert len(q) == 20
        return self.write(frame)

    def close(self):
        self.is_open = False


def summarize(samples):
    values = sorted(samples)
    percentile = lambda p: values[math.ceil(p * len(values)) - 1]
    return {"samples": len(values), "mean_us": statistics.mean(values),
            "p50_us": percentile(.5), "p95_us": percentile(.95),
            "p99_us": percentile(.99), "max_us": values[-1]}


def measure(fn, iterations, warmup):
    for i in range(warmup):
        fn(i)
    samples = []
    for i in range(iterations):
        start = time.perf_counter_ns()
        fn(i)
        samples.append((time.perf_counter_ns() - start) / 1000)
    return summarize(samples)


def run(iterations, warmup, profile=False):
    mapper = A30StrokeMapper()
    rng = random.Random(30)
    poses = [{n: rng.uniform(lo, hi) for n, (lo, hi) in zip(mapper.joint_names, mapper._limits)}
             for _ in range(256)]
    targets = [mapper.convert(pose) for pose in poses]
    result = {}
    result["mapper_convert_20"] = measure(lambda i: mapper.convert(poses[i % 256]), iterations, warmup)
    result["encode_108"] = measure(lambda i: encode_control_targets(targets[i % 256], seq=i), iterations, warmup)
    result["convert_and_encode"] = measure(
        lambda i: encode_control_targets(mapper.convert(poses[i % 256]), seq=i), iterations, warmup)

    for replan in (False, True):
        trajectory = Ruckig(20, .001)
        inp, out = InputParameter(20), OutputParameter(20)
        configure_ruckig(inp, mapper.joint_names)
        start_pose = [lo + .3 * (hi - lo) for lo, hi in mapper._limits]
        end_pose = [lo + .7 * (hi - lo) for lo, hi in mapper._limits]
        inp.current_position = start_pose
        inp.target_position = end_pose

        def update(i):
            # Fixed trajectory restarts every 250 samples, so steady-state
            # measurements stay in motion instead of timing only Finished.
            if i % 250 == 0:
                inp.current_position = start_pose
                inp.current_velocity = [0.] * 20
                inp.current_acceleration = [0.] * 20
                trajectory.reset()
            if replan:
                inp.target_position = [q + .001 * math.sin(i * .01) for q in end_pose]
            assert trajectory.update(inp, out) in (Result.Working, Result.Finished)
            out.pass_to_input(inp)

        result["ruckig_replan" if replan else "ruckig_step"] = measure(update, iterations, warmup)

    for interval_ms in (1, 2, 3, 4):
        clock = Clock()
        device = A30Device(mapper, MemoryPort, clock)
        device.connect("MEMORY-ONLY")
        device.payload = bytes(484)  # All axes offline: arm uses bounded fallback.
        device.feedback_at = clock()
        device.arm(poses[0])

        def tick(i):
            clock.value += interval_ms / 1000
            device.feedback_at = clock()  # Simulated fresh cached telemetry.
            if i % 100 == 0:
                device.set_target(poses[(i // 100) % 256])
            device.tick()
            assert device.armed, device.error

        result[f"tick_{interval_ms}ms"] = measure(tick, iterations, warmup)
        device.disconnect()

    if profile:
        profiler = cProfile.Profile()
        profiler.enable()
        for i in range(iterations):
            encode_control_targets(mapper.convert(poses[i % 256]), seq=i)
        profiler.disable()
        pstats.Stats(profiler).strip_dirs().sort_stats("cumtime").print_stats(25)

    sources = ["online_control/online_motion_play/a30_device.py", "src/a30_kinematics/mapper.py",
               "src/a30_kinematics/downlink.py", "src/a32_kinematics/a32_fast.py",
               "online_control/shared/downlink.py", "tools/benchmark_a30_python.py"]
    return {"environment": {"python": sys.version, "executable": sys.executable,
                            "platform": platform.platform(), "processor": platform.processor(),
                            "cpu_count": os.cpu_count(), "numpy": np.__version__,
                            "ruckig": ruckig.__version__, "device_dt_s": A30Device.CONTROL_DT},
            "method": {"iterations": iterations, "warmup": warmup, "seed": 30,
                       "gc": "enabled", "clock": "perf_counter_ns", "unit": "microseconds",
                       "serial": "memory only", "tick_clock": "synthetic; no sleep"},
            "sha256": {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in sources},
            "results": result}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--iterations", type=int, default=10000)
    parser.add_argument("--warmup", type=int, default=1000)
    parser.add_argument("--profile", action="store_true")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    if args.iterations < 1 or args.warmup < 0:
        parser.error("iterations must be positive; warmup must be nonnegative")
    data = run(args.iterations, args.warmup, args.profile)
    if args.output:
        args.output.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")
    for name, row in data["results"].items():
        print(f"{name:24} p50={row['p50_us']:8.2f} p95={row['p95_us']:8.2f} "
              f"p99={row['p99_us']:8.2f} max={row['max_us']:9.2f} us")
