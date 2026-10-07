"""Causal, per-joint low-lag smoothing for the non-Ruckig control path.

Small errors use a 16 ms time constant, large moves approach 4 ms. This
is a smoother, not a velocity/acceleration/jerk limiter. Call at control
rate, before path collision checking; feed the accepted pose back to avoid
accumulating motion behind a collision hold.
"""
import math


class DirectJointFilter:
    CONFIG = {"algorithm": "adaptive_exponential/v1", "tau_slow_ms": 16.0,
              "tau_fast_ms": 4.0, "error_scale_rad": 0.03}

    def __init__(self):
        self.pose = {}
        self.raw = {}
        self.filtered = {}

    def reset(self, pose):
        self.pose = dict(pose)
        self.raw = dict(pose)
        self.filtered = dict(pose)

    def sample(self, target, dt):
        if not math.isfinite(dt) or not 0 < dt <= .1:
            raise ValueError("直接滤波采样间隔无效")
        if any(not math.isfinite(q) for q in target.values()):
            raise ValueError("直接滤波目标含非有限值")
        if self.pose.keys() != target.keys():
            raise ValueError("直接滤波关节集合未初始化")
        fast = self.CONFIG["tau_fast_ms"] / 1000
        slow = self.CONFIG["tau_slow_ms"] / 1000
        scale = self.CONFIG["error_scale_rad"]
        result = {}
        for n, q in target.items():
            previous = self.pose[n]
            error = q - previous
            tau = fast + (slow-fast) / (1 + (error / scale) ** 2)
            alpha = -math.expm1(-dt / tau)
            # Convex update: no prediction, overshoot or look-ahead buffer.
            result[n] = min(max(q, previous), max(min(q, previous), previous + alpha * error))
        self.raw, self.filtered = dict(target), result
        self.pose = dict(result)
        return dict(result)

    def accept(self, checked_pose):
        self.pose = dict(checked_pose)

    def snapshot(self):
        return {**self.CONFIG, "input_joint_angles_rad": dict(self.raw),
                "filtered_joint_angles_rad": dict(self.filtered),
                "checked_joint_angles_rad": dict(self.pose)}
