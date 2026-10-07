"""
ruckig_interpolator.py — Ruckig-based jerk-limited online trajectory generation.

Replaces JointFrameInterpolator (smoothstep) for gesture downlink.
Each joint gets its own 1-DOF Ruckig instance; target updates from retarget
are fed asynchronously, and the 500 Hz downlink thread samples kinematically-
feasible positions with explicit velocity/acceleration/jerk limits.

Key advantage over smoothstep:
1. Explicit kinematic constraints (vel/acc/jerk) per joint type
2. Starts from actual current state — no jump when first target differs from 0
3. Mid-trajectory target changes are handled natively (Ruckig replans online)

Thread safety: push()/seed_ramp() called from Qt main thread;
               sample() called from GestureDownlink daemon thread.
               All public methods acquire a shared lock.
"""

from __future__ import annotations

import math
import os
import threading
from typing import Dict, Optional, Sequence

import ruckig
from joint_schema import default_joint_position
from utils.motion_limits import DEFAULT_MOTION_LIMITS

# ─────────────────────────────────────────────────────────
# Canonical joint list — must match JOINT_ORDER in usb/offline.py
# ─────────────────────────────────────────────────────────
CANONICAL_JOINTS: tuple[str, ...] = (
    "left_hand_thumb_rota_joint",
    "left_hand_thumb_swing_joint",
    "left_hand_thumb_MCP_joint",
    "left_hand_thumb_PIP_joint",
    "left_hand_index_swing_joint",
    "left_hand_index_MCP_joint",
    "left_hand_index_PIP_joint",
    "left_hand_index_DIP_joint",
    "left_hand_mid_swing_joint",
    "left_hand_mid_MCP_joint",
    "left_hand_mid_PIP_joint",
    "left_hand_mid_DIP_joint",
    "left_hand_ring_swing_joint",
    "left_hand_ring_MCP_joint",
    "left_hand_ring_PIP_joint",
    "left_hand_ring_DIP_joint",
    "left_hand_pinky_rota_joint",
    "left_hand_pinky_swing_joint",
    "left_hand_pinky_MCP_joint",
    "left_hand_pinky_PIP_joint",
    "left_hand_pinky_DIP_joint",
)

# ─────────────────────────────────────────────────────────
# Joint type classification → default kinematic constraints
# ─────────────────────────────────────────────────────────
# Joint types remain available for explicit legacy environment overrides.
# All types otherwise use the same defaults as online tracking and playback.

_JOINT_TYPES: dict[str, str] = {}
for name in CANONICAL_JOINTS:
    if "swing" in name and "thumb" in name:
        _JOINT_TYPES[name] = "thumb_swing"
    elif "swing" in name:
        _JOINT_TYPES[name] = "swing"
    elif "rota" in name:
        _JOINT_TYPES[name] = "rotation"
    else:
        _JOINT_TYPES[name] = "flexion"

# Default constraints per joint type (rad, rad/s, rad/s², rad/s³)
# Overridable via env vars: RUCKIG_FLEXION_VEL, RUCKIG_FLEXION_ACC, etc.
_DEFAULT_CONSTRAINTS = {
    "flexion":      DEFAULT_MOTION_LIMITS,
    "swing":        DEFAULT_MOTION_LIMITS,
    "rotation":     DEFAULT_MOTION_LIMITS,
    "thumb_swing":  DEFAULT_MOTION_LIMITS,
}


def _env_override(joint_type: str, idx: int) -> float:
    """Check for per-type environment variable override."""
    suffixes = {0: "VEL", 1: "ACC", 2: "JERK"}
    key = f"RUCKIG_{joint_type.upper()}_{suffixes.get(idx, '')}"
    val = os.environ.get(key)
    if val is not None:
        try:
            return float(val)
        except ValueError:
            pass
    return _DEFAULT_CONSTRAINTS[joint_type][idx]


def _constraints_for(name: str) -> tuple[float, float, float]:
    jtype = _JOINT_TYPES.get(name, "flexion")
    return (
        _env_override(jtype, 0),
        _env_override(jtype, 1),
        _env_override(jtype, 2),
    )


# Matches GestureDownlink thread rate (DOWNLINK_HZ_DEFAULT / HAND_SEND_HZ)
_RUCKIG_DT = float(
    os.environ.get(
        "RUCKIG_DELTA_TIME",
        str(1.0 / max(float(os.environ.get("HAND_SEND_HZ", "500")), 1.0)),
    )
)


def _to_canonical_name(name: str) -> str:
    """Map right-hand joint names to canonical left-hand names."""
    if name.startswith("right_hand_"):
        return "left_hand_" + name[len("right_hand_") :]
    return name


def _sample_delta_time(value: float | None) -> float:
    dt = _RUCKIG_DT if value is None else float(value)
    if not math.isfinite(dt) or dt <= 0:
        raise ValueError("Ruckig sample delta_time must be finite and positive")
    return dt


def _to_side_name(name: str, side: str) -> str:
    """Map canonical name to target side joint name."""
    if side == "right" and name.startswith("left_hand_"):
        return "right_hand_" + name[len("left_hand_") :]
    return name


class RuckigJointInterpolator:
    """Online jerk-limited trajectory generator for 21 dexterous-hand joints.

    Public API is a drop-in replacement for JointFrameInterpolator:

        interp = RuckigJointInterpolator()
        interp.push(cfg)               # update target from retarget
        angles = interp.sample()        # step trajectory, return {name: rad}
        interp.seed_ramp(start, end, d) # jump to 'start', then move to 'end'
        interp.reset()                  # reset all state
        interp.ready                    # True after first push/seed_ramp

    All public methods are thread-safe.
    """

    def __init__(self, joint_names: Optional[Sequence[str]] = None) -> None:
        names = list(joint_names) if joint_names else list(CANONICAL_JOINTS)

        self._otg: dict[str, ruckig.Ruckig] = {}
        self._inputs: dict[str, ruckig.InputParameter] = {}
        self._outputs: dict[str, ruckig.OutputParameter] = {}
        self._seeded: dict[str, bool] = {}    # True after first push for this joint
        self._joint_names = list(names)
        self._initialized = False
        self._lock = threading.Lock()
        self._output_hand_side = "left"
        # True right after reset()/seed_ramp(): the next sample() output is a
        # deliberate state snap, not a mid-trajectory jump. Consumers (e.g.
        # DownlinkGuard) should read+clear this via pop_reset_flag() to avoid
        # flagging an expected discontinuity as a fault.
        self._reset_pending = False
        # Joint names whose otg.update() raised on the most recent sample().
        self._last_fault_joints: frozenset[str] = frozenset()

        for name in self._joint_names:
            otg = ruckig.Ruckig(1, _RUCKIG_DT)
            inp = ruckig.InputParameter(1)
            out = ruckig.OutputParameter(1)

            vel, acc, jerk = _constraints_for(name)

            inp.current_position = [default_joint_position(name)]
            inp.current_velocity = [0.0]
            inp.current_acceleration = [0.0]
            inp.target_position = [default_joint_position(name)]
            inp.target_velocity = [0.0]
            inp.target_acceleration = [0.0]
            inp.max_velocity = [vel]
            inp.max_acceleration = [acc]
            inp.max_jerk = [jerk]

            self._otg[name] = otg
            self._inputs[name] = inp
            self._outputs[name] = out
            self._seeded[name] = False

    # ── public API ──────────────────────────────────────

    def pop_reset_flag(self) -> bool:
        """Read-and-clear the "next sample is a deliberate snap" flag."""
        with self._lock:
            flag = self._reset_pending
            self._reset_pending = False
        return flag

    @property
    def last_faults(self) -> frozenset[str]:
        """Joint (canonical) names whose Ruckig update raised on last sample()."""
        return self._last_fault_joints

    @property
    def ready(self) -> bool:
        return self._initialized

    def reset(self) -> None:
        """Rebuild all Ruckig instances. Call when hand tracking is lost."""
        with self._lock:
            for name in self._joint_names:
                otg = ruckig.Ruckig(1, _RUCKIG_DT)
                inp = ruckig.InputParameter(1)
                out = ruckig.OutputParameter(1)

                vel, acc, jerk = _constraints_for(name)

                inp.current_position = [default_joint_position(name)]
                inp.current_velocity = [0.0]
                inp.current_acceleration = [0.0]
                inp.target_position = [default_joint_position(name)]
                inp.target_velocity = [0.0]
                inp.target_acceleration = [0.0]
                inp.max_velocity = [vel]
                inp.max_acceleration = [acc]
                inp.max_jerk = [jerk]

                self._otg[name] = otg
                self._inputs[name] = inp
                self._outputs[name] = out
                self._seeded[name] = False

            self._initialized = False
            self._output_hand_side = "left"
            self._reset_pending = True
            self._last_fault_joints = frozenset()

    def shutdown(self) -> None:
        """Release all Ruckig C++ objects (avoid nanobind leak on process exit)."""
        with self._lock:
            self._otg.clear()
            self._inputs.clear()
            self._outputs.clear()
            self._seeded.clear()
            self._initialized = False

    def push(self, cfg: dict, t: float | None = None) -> None:
        """Update target positions from retarget output.

        On the very first call for each joint, both current and target
        are set to the incoming value — this prevents an initial jump
        from 0 to the first observed joint angle.

        Subsequent calls only update target_position; Ruckig handles
        mid-trajectory target changes by replanning online.
        """
        if not cfg:
            return
        with self._lock:
            mapped: dict[str, float] = {}
            side = self._output_hand_side
            for raw_name, raw_val in cfg.items():
                name = _to_canonical_name(str(raw_name))
                if name not in self._inputs:
                    continue
                val = float(raw_val)
                if math.isnan(val) or math.isinf(val):
                    continue
                mapped[name] = val
                if str(raw_name).startswith("right_hand_"):
                    side = "right"
                elif str(raw_name).startswith("left_hand_"):
                    side = "left"

            for name in self._joint_names:
                if name not in mapped:
                    continue
                val = mapped[name]
                inp = self._inputs[name]
                if not self._seeded[name]:
                    # First frame: current stays at the all-zero init pose,
                    # only set target. Ruckig generates a jerk-limited
                    # trajectory from init → val instead of jumping.
                    inp.target_position = [val]
                    inp.target_velocity = [0.0]
                    inp.target_acceleration = [0.0]
                    self._seeded[name] = True
                else:
                    # Subsequent: update target only.
                    # Ruckig replans online from the last sample() state.
                    inp.target_position = [val]

            self._output_hand_side = side
            self._initialized = True

    def peek(self) -> Optional[dict[str, float]]:
        """Return the current Ruckig state without advancing.

        Safe to call from any thread — read-only, no mutation.
        Use for display, collision seed, and other non-downlink reads.
        """
        if not self._initialized:
            return None
        with self._lock:
            result: dict[str, float] = {}
            for name in self._joint_names:
                # current_position holds the last step's output
                # (updated by pass_to_input in sample())
                out_name = _to_side_name(name, self._output_hand_side)
                result[out_name] = float(self._inputs[name].current_position[0])
        return result

    def sync_current_pose(self, cfg: dict, *, zero_derivatives: bool = True) -> None:
        """Synchronise OTG state to a pose actually accepted by a safety layer.

        A collision projection may hold or deflect a sampled Ruckig pose.  If
        the internal OTG state were allowed to continue through that rejected
        pose, a later sample could resume with stale velocity and effectively
        jump past the collision boundary.  This method makes the accepted pose
        the next replan state while preserving each joint's existing target.
        """
        if not cfg:
            return
        with self._lock:
            side = self._output_hand_side
            for raw_name, raw_value in cfg.items():
                name = _to_canonical_name(str(raw_name))
                if name not in self._inputs:
                    continue
                value = float(raw_value)
                if not math.isfinite(value):
                    continue
                inp = self._inputs[name]
                inp.current_position = [value]
                if zero_derivatives:
                    inp.current_velocity = [0.0]
                    inp.current_acceleration = [0.0]
                self._seeded[name] = True
                if str(raw_name).startswith("right_hand_"):
                    side = "right"
                elif str(raw_name).startswith("left_hand_"):
                    side = "left"
            self._output_hand_side = side
            self._initialized = True

    def sample(self, t: float | None = None, *, delta_time: float | None = None) -> Optional[dict[str, float]]:
        """Advance by delta_time, or one nominal control step if omitted.

        Only call from the downlink thread — this mutates internal state.
        For read-only access from other threads, use peek().

        Returns a dict {urdf_joint_name: position_rad} or None if
        the interpolator hasn't been seeded yet.
        """
        if not self._initialized:
            return None

        dt = _sample_delta_time(delta_time)
        with self._lock:
            result: dict[str, float] = {}
            faults: set[str] = set()
            for name in self._joint_names:
                otg = self._otg[name]
                inp = self._inputs[name]
                out = self._outputs[name]

                try:
                    otg.delta_time = dt
                    _ = otg.update(inp, out)
                except Exception:
                    # On Ruckig error, hold current position
                    faults.add(name)
                    out_name = _to_side_name(name, self._output_hand_side)
                    result[out_name] = float(inp.current_position[0])
                    continue
                out.pass_to_input(inp)

                out_name = _to_side_name(name, self._output_hand_side)
                result[out_name] = float(out.new_position[0])

            self._last_fault_joints = frozenset(faults)

        return result

    def seed_ramp(
        self, start: dict, end: dict, duration_s: float = 0.35
    ) -> None:
        """Reset all joints to `start` and set targets to `end`.

        Ruckig generates a smooth, kinematically-feasible transition.
        The `duration_s` parameter is advisory only — the actual
        transition time is governed by the per-joint velocity/acceleration/
        jerk limits. A shorter transition than the smoothstep equivalent
        is expected when the pose gap is small.

        Joints present in one dict but not the other are filled with 0.0.
        """
        if not start and not end:
            return

        with self._lock:
            all_keys = {_to_canonical_name(str(k)) for k in (set(start) | set(end))}
            side = self._output_hand_side
            for k in set(start) | set(end):
                ks = str(k)
                if ks.startswith("right_hand_"):
                    side = "right"
                elif ks.startswith("left_hand_"):
                    side = "left"
            for name in all_keys:
                # Ensure this joint is tracked (might be outside CANONICAL_JOINTS)
                if name not in self._otg:
                    continue

                s_val = float(
                    start.get(name, start.get(_to_side_name(name, "right"), default_joint_position(name)))
                )
                e_val = float(
                    end.get(name, end.get(_to_side_name(name, "right"), default_joint_position(name)))
                )

                inp = self._inputs[name]
                inp.current_position = [s_val]
                inp.current_velocity = [0.0]
                inp.current_acceleration = [0.0]
                inp.target_position = [e_val]
                inp.target_velocity = [0.0]
                inp.target_acceleration = [0.0]
                self._seeded[name] = True

            self._output_hand_side = side
            self._initialized = True
            self._reset_pending = True


class PhaseSynchronizedRuckigInterpolator:
    """Multi-DoF Ruckig path follower for collision-planned hand waypoints.

    The legacy interpolator intentionally runs one independent OTG per joint.
    That is convenient for raw retarget streaming, but an independently timed
    trajectory can leave a collision-free multi-joint edge.  This variant uses
    one Ruckig instance with phase synchronization, so all active joints follow
    the same normalized progress along each planned joint-space segment.
    """

    def __init__(self, joint_names: Optional[Sequence[str]] = None) -> None:
        raw_names = list(joint_names) if joint_names else list(CANONICAL_JOINTS)
        self._joint_names = [_to_canonical_name(str(name)) for name in raw_names]
        if len(set(self._joint_names)) != len(self._joint_names):
            raise ValueError("phase-synchronized Ruckig joint names must be unique")
        self._index = {name: index for index, name in enumerate(self._joint_names)}
        self._lock = threading.Lock()
        self._output_hand_side = "left"
        self._initialized = False
        self._reset_pending = False
        self._last_fault_joints: frozenset[str] = frozenset()
        self._build()

    def _build(self) -> None:
        dofs = len(self._joint_names)
        self._braking = False
        self._speed_scale = 1.0
        self._otg = ruckig.Ruckig(dofs, _RUCKIG_DT)
        self._input = ruckig.InputParameter(dofs)
        self._output = ruckig.OutputParameter(dofs)
        self._input.current_position = [default_joint_position(n) for n in self._joint_names]
        self._input.current_velocity = [0.0] * dofs
        self._input.current_acceleration = [0.0] * dofs
        self._input.target_position = list(self._input.current_position)
        self._input.target_velocity = [0.0] * dofs
        self._input.target_acceleration = [0.0] * dofs
        limits = [_constraints_for(name) for name in self._joint_names]
        self._input.max_velocity = [row[0] for row in limits]
        self._input.max_acceleration = [row[1] for row in limits]
        self._input.max_jerk = [row[2] for row in limits]
        self._input.synchronization = ruckig.Synchronization.Phase
        self._seeded = [False] * dofs

    @property
    def stationary(self) -> bool:
        with self._lock:
            return (
                max(map(abs, self._input.current_velocity), default=0.0) < 1.0e-5
                and max(map(abs, self._input.current_acceleration), default=0.0) < 1.0e-4
            )

    def brake(self) -> None:
        """Stop from the current position/velocity/acceleration with jerk limits.

        Velocity control discards the old tracking destination.  Sending the
        current position as a position target instead would overshoot and then
        reverse back to it.  The caller still collision-checks every brake step.
        """
        with self._lock:
            self._braking = True
            self._input.control_interface = ruckig.ControlInterface.Velocity
            self._input.synchronization = ruckig.Synchronization.Time
            self._input.target_position = list(self._input.current_position)
            self._input.target_velocity = [0.0] * len(self._joint_names)
            self._input.target_acceleration = [0.0] * len(self._joint_names)

    def set_speed_scale(self, scale: float) -> None:
        """Time-scale the joint limits; lower the scale only when stopped."""
        value = max(0.05, min(1.0, float(scale)))
        with self._lock:
            if abs(value - self._speed_scale) < 1.0e-6:
                return
            limits = [_constraints_for(name) for name in self._joint_names]
            self._input.max_velocity = [row[0] * value for row in limits]
            self._input.max_acceleration = [row[1] * value**2 for row in limits]
            self._input.max_jerk = [row[2] * value**3 for row in limits]
            self._speed_scale = value

    @property
    def ready(self) -> bool:
        return self._initialized

    @property
    def last_faults(self) -> frozenset[str]:
        return self._last_fault_joints

    def pop_reset_flag(self) -> bool:
        with self._lock:
            value = self._reset_pending
            self._reset_pending = False
            return value

    def reset(self) -> None:
        with self._lock:
            self._build()
            self._output_hand_side = "left"
            self._initialized = False
            self._reset_pending = True
            self._last_fault_joints = frozenset()

    def shutdown(self) -> None:
        with self._lock:
            self._initialized = False
            self._last_fault_joints = frozenset()

    @staticmethod
    def _mapped_values(cfg: dict) -> tuple[dict[str, float], str | None]:
        mapped: dict[str, float] = {}
        side: str | None = None
        for raw_name, raw_value in cfg.items():
            name = str(raw_name)
            value = float(raw_value)
            if not math.isfinite(value):
                continue
            mapped[_to_canonical_name(name)] = value
            if name.startswith("right_hand_"):
                side = "right"
            elif name.startswith("left_hand_"):
                side = "left"
        return mapped, side

    def push(self, cfg: dict, t: float | None = None) -> None:
        if not cfg:
            return
        mapped, side = self._mapped_values(cfg)
        with self._lock:
            self._braking = False
            self._input.control_interface = ruckig.ControlInterface.Position
            self._input.synchronization = ruckig.Synchronization.Phase
            targets = list(self._input.target_position)
            for name, value in mapped.items():
                index = self._index.get(name)
                if index is None:
                    continue
                targets[index] = value
                self._seeded[index] = True
            self._input.target_position = targets
            self._input.target_velocity = [0.0] * len(self._joint_names)
            self._input.target_acceleration = [0.0] * len(self._joint_names)
            if side is not None:
                self._output_hand_side = side
            self._initialized = any(self._seeded)

    def peek(self) -> Optional[dict[str, float]]:
        if not self._initialized:
            return None
        with self._lock:
            return {
                _to_side_name(name, self._output_hand_side): float(
                    self._input.current_position[index]
                )
                for index, name in enumerate(self._joint_names)
            }

    def sync_current_pose(self, cfg: dict, *, zero_derivatives: bool = True) -> None:
        if not cfg:
            return
        mapped, side = self._mapped_values(cfg)
        with self._lock:
            positions = list(self._input.current_position)
            for name, value in mapped.items():
                index = self._index.get(name)
                if index is None:
                    continue
                positions[index] = value
                self._seeded[index] = True
            self._input.current_position = positions
            if zero_derivatives:
                self._input.current_velocity = [0.0] * len(self._joint_names)
                self._input.current_acceleration = [0.0] * len(self._joint_names)
            if side is not None:
                self._output_hand_side = side
            self._initialized = any(self._seeded)

    def sample(self, t: float | None = None, *, delta_time: float | None = None) -> Optional[dict[str, float]]:
        """Advance by measured elapsed time, or one nominal step if omitted.

        The caller gates excessive gaps and collision-checks the output. A
        single update produces one current pose, never a burst of old samples.
        Fixed-step callers (including offline replay) keep their old behavior.
        """
        if not self._initialized:
            return None
        dt = _sample_delta_time(delta_time)
        with self._lock:
            faults: frozenset[str] = frozenset()
            try:
                self._otg.delta_time = dt
                result = self._otg.update(self._input, self._output)
                if result.value < 0:
                    # Phase synchronization may be temporarily infeasible after
                    # a sharp live-goal change.  Time synchronization is a safe
                    # one-tick fallback; the collision gate still validates it.
                    self._input.synchronization = ruckig.Synchronization.Time
                    result = self._otg.update(self._input, self._output)
                    self._input.synchronization = (
                        ruckig.Synchronization.Time if self._braking
                        else ruckig.Synchronization.Phase
                    )
                if result.value < 0:
                    raise RuntimeError(f"Ruckig returned {result}")
                self._output.pass_to_input(self._input)
                positions = list(self._output.new_position)
            except Exception:
                self._input.synchronization = (
                    ruckig.Synchronization.Time if self._braking
                    else ruckig.Synchronization.Phase
                )
                positions = list(self._input.current_position)
                faults = frozenset(self._joint_names)
            self._last_fault_joints = faults
            return {
                _to_side_name(name, self._output_hand_side): float(positions[index])
                for index, name in enumerate(self._joint_names)
            }

    def seed_ramp(
        self, start: dict, end: dict, duration_s: float = 0.35
    ) -> None:
        if not start and not end:
            return
        start_values, start_side = self._mapped_values(start)
        end_values, end_side = self._mapped_values(end)
        with self._lock:
            self._braking = False
            self._input.control_interface = ruckig.ControlInterface.Position
            current = [0.0] * len(self._joint_names)
            target = [0.0] * len(self._joint_names)
            for name, index in self._index.items():
                current[index] = float(start_values.get(name, default_joint_position(name)))
                target[index] = float(end_values.get(name, current[index]))
                self._seeded[index] = name in start_values or name in end_values
            self._input.current_position = current
            self._input.current_velocity = [0.0] * len(self._joint_names)
            self._input.current_acceleration = [0.0] * len(self._joint_names)
            self._input.target_position = target
            self._input.target_velocity = [0.0] * len(self._joint_names)
            self._input.target_acceleration = [0.0] * len(self._joint_names)
            self._input.synchronization = ruckig.Synchronization.Phase
            self._output_hand_side = end_side or start_side or self._output_hand_side
            self._initialized = any(self._seeded)
            self._reset_pending = True
