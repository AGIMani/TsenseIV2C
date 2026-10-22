"""Independent 20-DOF Ruckig controller; no USB opened on the server."""
import math
import threading
import time
import struct
from collections import deque
import ruckig
from a30_kinematics import A30StrokeMapper
from a30_kinematics.feedback import measured_pose
from a30_kinematics.mapper import JOINT_LIMIT_EPS_RAD
from online_control.online_motion_track.pipeline_v3_0.collision import StopCollisionGuard
from online_control.shared.downlink import CONTROL_DT, CONTROL_HZ, configure_ruckig, encode_control_targets
from utils.control_clock import precise_control_timer
from online_control.shared.stream import StreamLease
from .grouped_ruckig import GroupedRuckig, GroupedOutput
from ..safe_trajectory import SafeTrajectoryMixin


class A30Control(SafeTrajectoryMixin):
    def __init__(self, profile, side, collision_mode, *, start=True, preview_only=False):
        self.profile, self.side, self.mode = profile, side, collision_mode
        self.mapper = A30StrokeMapper(side, model_id=profile.model_id)
        self.names = self.mapper.joint_names
        self.limits = dict(zip(self.names, self.mapper._limits))
        ends = [self.mapper.convert({n: bounds[i] for n, bounds in self.limits.items()}) for i in (0, 1)]
        self.feedback_position_limits = {
            kind: {n: sorted((getattr(ends[0], field)[n], getattr(ends[1], field)[n]))
                   for n in getattr(ends[0], field)}
            for kind, field in (("linear", "linear_strokes_mm"), ("rotary", "rotary_positions_deg"))}
        self.guard = StopCollisionGuard(profile, side) if collision_mode in ("stop", "mink") else None
        self._normal_guard = self.guard
        self.lock = threading.RLock(); self.done = threading.Event()
        self.ready = threading.Condition(self.lock)
        self.stream_active = False
        self.preview_stream_active = False
        self.direct_mode = False
        # Only browser preview sessions may pause their simulated clock. Once
        # feedback is rebased or a downlink stream opens, timeout faults latch.
        self.preview_only = preview_only
        self.wire = b''
        self.produced_at = time.perf_counter()
        self.otg = GroupedRuckig(self.names, CONTROL_DT)
        self.input, self.output = ruckig.InputParameter(20), GroupedOutput(20)
        configure_ruckig(self.input, self.names)
        self.pose = profile.init_pose(side); self.target = dict(self.pose)
        self.frame_id = -1; self.epoch = 0; self.ticks = 0
        self.sample_frame_id = -1
        self.sample_intervals = deque(maxlen=250)
        self.target_at = 0.; self.valid = True
        self.phase = "holding"; self.error = ""; self.sample_collision = {}
        self.finger_holds = {}
        self.finger_hold_context = {}
        self.limit_recoveries = 0
        self.last_limit_event = None
        self.boundary_roundoffs = 0
        self.last_boundary_roundoff = None
        self.feedback_fallback_joints = []
        self.feedback_clipped_joints = []
        self.last_tick = self.started = time.perf_counter()
        self._seed(self.pose)
        self.thread = threading.Thread(target=self._run, daemon=True)
        if start: self.thread.start()

    def _seed(self, pose):
        self.ruckig_output_joint_angles_rad = None
        self.pose = dict(pose)
        self.input.current_position = [pose[n] for n in self.names]
        self.input.current_velocity = [0.]*20; self.input.current_acceleration = [0.]*20
        self.input.target_position = list(self.input.current_position)
        self.input.target_velocity = [0.]*20; self.input.target_acceleration = [0.]*20
        if self.guard:
            self.guard.current = dict(pose); self.guard.guard.rebase(pose)

    def invalidate(self, reason, *, phase="blocked"):
        with self.lock:
            self.valid = False; self.phase = phase; self.error = reason
            self.wire = b''
            self._seed(self.pose)
            self.ready.notify_all()

    def hold_target(self):
        """Finish the last safe target with Ruckig; missing hands are not faults.

        Keep velocity/acceleration continuous, and never clear a collision or
        computation fault merely because a later camera frame has no hand.
        """
        with self.lock:
            if self.valid and not self.finger_holds:
                self.phase = "holding"

    def set_direct_mode(self, enabled):
        if not isinstance(enabled, bool):
            raise ValueError("enabled 必须为布尔值")
        with self.lock:
            if enabled == self.direct_mode:
                return
            if enabled:
                self._normal_guard = self.guard
            self.guard = None if enabled else self._normal_guard
            self.direct_mode = enabled
            self.epoch += 1  # Invalidate every existing binary stream.
            self.wire = b''
            self._seed(self.pose)
            self.target = dict(self.pose)
            self.target_at = 0.
            self.frame_id = self.sample_frame_id = -1
            self.sample_collision = {}
            self.finger_holds.clear()
            self.finger_hold_context.clear()
            self.valid = True
            self.phase = "holding"
            self.error = ""
            self.last_tick = time.perf_counter()
            self.ready.notify_all()

    def rebase(self, payload, fallback_pose=None, *, resume_from_commanded_pose=False):
        with self.lock:
            target, target_at = dict(self.target), self.target_at
            if resume_from_commanded_pose and not self.valid:
                raise ValueError("控制器故障未解除，请手动重新开启")
            missing, clipped = [], []
            # The preview may already have advanced. Never use it as unmeasured
            # hardware feedback: use the client's last sent pose or initial pose.
            pose = measured_pose(payload, self.mapper,
                                 fallback_pose=self.profile.init_pose(self.side) if fallback_pose is None else fallback_pose,
                                 missing_joints=missing, clamp=True, clipped_joints=clipped)
            if resume_from_commanded_pose:
                # Automatic stream recovery must continue from the last USB
                # command, not a cached telemetry packet from before that write.
                if not isinstance(fallback_pose, dict) or any(
                    n not in fallback_pose or not isinstance(fallback_pose[n], (int, float))
                    or not math.isfinite(fallback_pose[n])
                    or not lo-1e-5 <= fallback_pose[n] <= hi+1e-5
                    for n, (lo, hi) in self.limits.items()
                ):
                    raise ValueError("恢复轨迹缺少有效的最后下发姿态")
                pose = self.mapper.convert(fallback_pose, clamp=True).joint_angles_rad
            if self.guard:
                self.guard.filter.sync_pose_from_dict(pose)
                if self.guard.filter.data.ncon:
                    raise ValueError("当前反馈姿态存在碰撞，请先调整实机")
            self.preview_only = False
            self._seed(pose); self.target = dict(pose); self.epoch += 1
            if resume_from_commanded_pose:
                # Keep the most recent accepted gesture while replacing only
                # the trajectory origin. A transport hiccup must not cancel
                # opening the hand if the next observation is briefly missing.
                self.target = target
                self.input.target_position = [target[n] for n in self.names]
            self.finger_holds.clear()
            self.finger_hold_context.clear()
            self.feedback_fallback_joints = missing
            self.feedback_clipped_joints = clipped
            self.valid = True; self.phase = "holding"; self.error = ""
            self.target_at = target_at if resume_from_commanded_pose else 0.
            self.wire = b''
            self.last_tick = time.perf_counter()
            return self.snapshot()

    def push(self, pose, frame_id):
        with self.lock:
            # Only collision pauses may recover from camera input. Hard faults
            # remain latched until an explicit feedback rebase.
            if not self.valid and self.phase != "collision_paused":
                return
            # Apply mechanism limits before Ruckig, never clamp its output afterwards.
            applied = self.mapper.convert(pose, clamp=True).joint_angles_rad
            # A hold depends on both the request and the surrounding fingers.
            # A neighbour may retreat while this finger's open-hand target is
            # unchanged. Retry from the accepted pose when either has changed;
            # _checked_step must still validate every replanned sample/path.
            # Compare at camera rate, not on every 500 Hz control tick.
            for finger, requested in list(self.finger_holds.items()):
                context = self.finger_hold_context.get(finger, {})
                neighbour_moved = any(abs(self.pose[n]-q) > 1e-7 for n, q in context.items())
                if neighbour_moved or max(abs(applied[n]-q) for n, q in requested.items()) > .005:
                    del self.finger_holds[finger]
                    self.finger_hold_context.pop(finger, None)
            self.target = dict(applied); self.input.target_position = [applied[n] for n in self.names]
            for finger in self.finger_holds:
                self._stop_joints(self._finger_joints(finger), hold=True)
            self.target_at = time.perf_counter(); self.frame_id = frame_id
            self.valid = True
            # Keep publishing pause heartbeats until a new sample passes every
            # check; the old wire was cleared when the collision occurred.
            if self.phase != "collision_paused":
                self.phase = "tracking"; self.error = ""

    def _checked_direct_step(self, dt):
        """Send the bounded latest target without trajectory or collision correction."""
        pose = dict(self.target)
        if any(not math.isfinite(pose[n]) or not lo <= pose[n] <= hi
               for n, (lo, hi) in self.limits.items()):
            raise ValueError("直接目标超出关节范围或含非有限值")
        return pose

    def step(self, dt, *, sample_at=None):
        with self.lock:
            if not self.valid: return
            if not math.isfinite(dt) or dt <= 0 or (dt > .1 and not self.preview_only):
                self.invalidate("控制采样超时"); return
            if dt > .1:
                # SDK startup / scheduler stalls may pause an offline preview.
                # Advance one bounded step, never catch up with a large jump.
                dt = CONTROL_DT
            if time.perf_counter()-self.target_at > .4:
                self.hold_target()
            try:
                sample_at = time.perf_counter() if sample_at is None else sample_at
                pose = self._checked_direct_step(dt) if self.direct_mode else self._checked_step(dt)
                targets = self.mapper.convert(pose)
                self.wire = encode_control_targets(targets, hand_side=self.side, seq=self.ticks & 255)
                # Timestamp the same clock boundary that determined dt, not
                # completion of a variably expensive collision calculation.
                self.produced_at = sample_at
                self.pose = pose
                self.sample_frame_id = self.frame_id
                if not self.direct_mode:
                    self.output.pass_to_input(self.input)
                self.ticks += 1
                if self.finger_holds:
                    self.phase = "collision_limited"
                elif self.phase in ("collision_paused", "collision_limited"):
                    self.phase = "tracking"
                self.error = ""
                self.sample_intervals.append(dt)
                self.ready.notify_all()
            except Exception as exc:
                self.invalidate(str(exc))

    def _run(self):
        with precise_control_timer():
            deadline = time.perf_counter()
            while not self.done.is_set():
                now = time.perf_counter(); dt = now-self.last_tick; self.last_tick = now
                self.step(dt, sample_at=now)
                deadline += CONTROL_DT
                if deadline <= now:
                    deadline += (math.floor((now-deadline)/CONTROL_DT)+1)*CONTROL_DT
                time.sleep(max(0., deadline-time.perf_counter()))

    def open_stream(self, epoch, *, preview=False):
        with self.lock:
            occupied = self.preview_stream_active if preview else self.stream_active
            if occupied or epoch != self.epoch or self.done.is_set():
                raise ValueError("下发流已占用或起始姿态已变化")
            if preview:
                self.preview_stream_active = True
            else:
                self.preview_only = False
                self.stream_active = True
        return StreamLease(self._records(epoch, preview=preview), lambda: self._release_stream(preview))

    def _release_stream(self, preview=False):
        with self.lock:
            if preview:
                self.preview_stream_active = False
            else:
                self.stream_active = False

    def _records(self, epoch, *, preview=False):
        last = -1
        try:
            while not self.done.is_set():
                with self.lock:
                    self.ready.wait_for(lambda: self.ticks != last or self.done.is_set() or self.epoch != epoch, timeout=.02)
                    if self.done.is_set() or self.epoch != epoch:
                        return
                    ready = self.valid and bool(self.wire)
                    # GTR1: 1=checked position, 2=latched fault, 4=collision
                    # pause heartbeat (no USB payload; retain browser arm).
                    flags = (1 if ready else 0) | (2 if self.phase == 'blocked' else 0) | (4 if self.phase == 'collision_paused' else 0)
                    packet = struct.pack('<4sIIidI20f', b'GTR1', self.epoch, self.ticks, self.sample_frame_id,
                                         self.produced_at if ready else time.perf_counter(), flags,
                                         *[self.pose[n] for n in self.names])
                    if ready and not preview:
                        packet += self.wire
                    last = self.ticks
                yield packet
        finally:
            pass  # StreamLease releases once, including early HTTP cancellation.

    def snapshot(self):
        with self.lock:
            actuation = self.mapper.preview(self.pose)
            return {"model_id": self.profile.model_id, "hand_side": self.side,
                    "control_stream": True, "direct_mode": self.direct_mode,
                    "server_time": time.perf_counter(), "control_joint_names": list(self.names),
                    "motion_limits": {n: {"velocity": self.input.max_velocity[i],
                        "acceleration": self.input.max_acceleration[i], "jerk": self.input.max_jerk[i]}
                        for i, n in enumerate(self.names)},
                    "motion_synchronization": {"mode": "finger_groups", "within_group": "time",
                        "groups": {key: [self.names[i] for i in indices] for key, indices in self.otg.groups.items()}},
                    "held_fingers": sorted(self.finger_holds),
                    "limit_recoveries": self.limit_recoveries, "last_limit_event": self.last_limit_event,
                    "boundary_roundoffs": self.boundary_roundoffs,
                    "last_boundary_roundoff": self.last_boundary_roundoff,
                    "ruckig_enabled": not self.direct_mode, "sample_collision_ready": self.guard is not None,
                    "direct_filter": None,
                    "motion_limits_enabled": not self.direct_mode,
                    "collision_protection_enabled": self.guard is not None,
                    "collision_mode": "off" if self.direct_mode else self.mode,
                    "sample_collision": self.sample_collision if self.guard is not None else {
                        "mode": "off" if self.direct_mode else self.mode, "checked": False, "protection_enabled": False,
                        "resolution": "disabled"},
                    "target_ready": self.valid and self.phase != "collision_paused" and actuation["valid"],
                    "rebase_required": not self.valid and self.phase == "blocked",
                    "tracking_transition": {"phase": self.phase}, "last_error": self.error,
                    "browser_epoch": self.epoch, "sample_ticks": self.ticks, "frame_id": self.frame_id,
                    "sample_frame_id": self.sample_frame_id,
                    "feedback_fallback_joints": list(self.feedback_fallback_joints),
                    "feedback_clipped_joints": list(self.feedback_clipped_joints),
                    "feedback_position_limits": self.feedback_position_limits,
                    "ruckig_joint_angles_rad": dict(self.pose), "target_joint_angles_rad": dict(self.target),
                    "ruckig_output_joint_angles_rad": (dict(self.ruckig_output_joint_angles_rad)
                        if not self.direct_mode and self.ruckig_output_joint_angles_rad is not None else None),
                    "command_joint_angles_rad": dict(self.pose),
                    "hardware_joint_angles_rad": dict(self.pose), "linear_strokes_mm": actuation["linear_strokes_mm"],
                    "rotary_positions_deg": actuation["rotary_positions_deg"], "actuation": actuation,
                    "control_timing": {"target_sample_hz": CONTROL_HZ, "actual_sample_hz": len(self.sample_intervals)/max(.001,sum(self.sample_intervals))},
                    "armed": False, "connected": False}

    def close(self):
        self.done.set()
        with self.lock:
            self.ready.notify_all()
        if self.thread.is_alive(): self.thread.join(timeout=2)
