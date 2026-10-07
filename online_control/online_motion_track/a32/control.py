"""A3.2 trajectories with optional feedback; only this worker writes targets."""
import math
import threading
import time
from collections import deque

import ruckig

from a32_driver import A32Device
from a32_driver.recording import A32SessionRecorder
from utils.control_clock import precise_control_timer, control_waiter
from utils.motion_limits import configure_ruckig
from ..a30.grouped_ruckig import GroupedRuckig, GroupedOutput
from ..safe_trajectory import SafeTrajectoryMixin


class A32Control(SafeTrajectoryMixin):
    PERIOD = .002
    MAX_PLANNER_STEP = .004
    INPUT_TIMEOUT = .4
    CLIENT_TIMEOUT = 2.

    def __init__(self, profile, side, collision_mode, *, device=None, start=True,
                 recording_dir=None, recorder=None):
        self.profile, self.side, self.mode = profile, side, collision_mode
        self.direct_mode = False
        self.device = device if device is not None else A32Device(side, model_id=profile.model_id)
        self.pose = profile.init_pose(side)
        self.names = tuple(self.pose)
        self.target = dict(self.pose)
        self.lock = threading.RLock()
        self.done = threading.Event()
        self.armed = False
        self.stop_pending = False
        self.tracking_started = False
        self.telemetry_seen = False
        self.phase = "holding"
        self.last_error = ""
        self.target_at = 0.
        self.client_at = time.monotonic()
        self.ticks = 0
        self.frame_id = -1
        self.sample_collision = {}
        self.control_timing = {}
        self._last_write_at = 0.
        self._native_spacing = False
        self.timing_violations = {'below_1ms': 0, 'above_4ms': 0, 'samples': 0}
        self.finger_holds = {}
        self.finger_hold_context = {}
        self.limit_recoveries = 0
        self.last_limit_event = None
        self.boundary_roundoffs = 0
        self.last_boundary_roundoff = None
        self.limits = self.device.joint_limits()
        self.last_sent = None
        self.samples = deque(maxlen=800)
        self.recorder = recorder if recorder is not None else A32SessionRecorder(
            recording_dir, model_id=profile.model_id, hand_side=side)
        self.recording_error = ""
        self._recording_state_at = 0.
        self._state_cache = None
        self._state_cache_error = None
        self._recording_cache = None
        self.guard = None
        if collision_mode in ("stop", "mink"):
            from online_control.online_motion_track.pipeline_v3_0.collision import StopCollisionGuard
            self.guard = StopCollisionGuard(profile, side)
        self._normal_guard = self.guard
        self.otg = GroupedRuckig(self.names, self.PERIOD)
        self.input = ruckig.InputParameter(len(self.names))
        self.output = GroupedOutput(len(self.names))
        configure_ruckig(self.input, self.names)
        self._seed(self.pose)
        self.thread = threading.Thread(target=self._run, name="A32Tracking", daemon=True)
        self.observer = threading.Thread(target=self._observe, name='A32TelemetryLog', daemon=True)
        if start:
            self.thread.start()
            self.observer.start()

    def start_recording(self, **metadata):
        with self.lock:
            self.recorder.stop(reason="reconnect")
            self.recorder.start(metadata={"platform": "motion_track",
                                          "camera_source": "browser-webcam", **metadata,
                                          "collision_mode": "off" if self.direct_mode else self.mode,
                                          "direct_mode": self.direct_mode,
                                          "ruckig_enabled": not self.direct_mode,
                                          "collision_protection_enabled": self.guard is not None})
            self.recording_error = ""
            self._recording_state_at = 0.

    def _record(self, method, *args, **details):
        # A disk failure is shown in status without interrupting motor control.
        try:
            return getattr(self.recorder, method)(*args, **details)
        except Exception as exc:
            self.recording_error = str(exc)

    def record_action(self, action, **details):
        with self.lock:
            try:
                state = self.device.get_state()
            except Exception as exc:
                self._record("record_error", exc, action="observe_action", request_action=action)
                state = None
            self._record("record_action", action, state, **details)

    def record_event(self, event, **details):
        # Raw skeleton deepcopy/recording must not own the motor control lock.
        self._record("record_event", event, **details)

    def _record_state(self, state=None):
        if not self.recorder.writer.active:
            return
        now = time.perf_counter()
        if now - self._recording_state_at < .02:
            return
        self._recording_state_at = now
        try:
            if state is None:
                state = self.device.get_state()
        except Exception as exc:
            self._record("record_error", exc, action="observe_state")
            return
        self._record("record_state", {**state, "armed": self.armed,
                                      "collision_mode": "off" if self.direct_mode else self.mode,
                                      "direct_mode": self.direct_mode,
                                      "ruckig_enabled": not self.direct_mode,
                                      "collision_protection_enabled": self.guard is not None,
                                      "sample_ticks": self.ticks, "frame_id": self.frame_id})

    def _preview_targets(self, pose):
        try:
            return self.device.prepare_targets(pose, clamp=False)
        except Exception:
            return {}

    def _observe(self):
        with control_waiter(self.done) as wait:
            while not wait(.02):
                try:
                    state = self._read_state()
                except Exception as exc:
                    self._state_cache_error = exc
                    self._record("record_error", exc, action="observe_state")
                    continue
                self._record_state(state)

    def _read_state(self):
        # This observer cache is for presentation only. step()/arm() always
        # read the SDK directly and retain all existing safety checks.
        observed = time.monotonic()
        state = self.device.get_state()
        self._state_cache = (observed, state)
        self._state_cache_error = None
        return state

    def _snapshot_state(self, refresh):
        cached = self._state_cache
        age = time.monotonic() - cached[0] if cached else float('inf')
        if refresh or age > .1:
            state = self._read_state()
            return {**state, 'sdk_snapshot_age_ms': 0.}
        if self._state_cache_error is not None:
            raise self._state_cache_error
        state = dict(cached[1])
        # A cached telemetry age must keep advancing while no new packet arrives.
        telemetry_age = state.get('telemetry_age_ms', -1)
        if isinstance(telemetry_age, (int, float)) and telemetry_age >= 0:
            state['telemetry_age_ms'] = telemetry_age + age * 1000
        state['sdk_snapshot_age_ms'] = age * 1000
        return state

    def _snapshot_recording(self, refresh):
        writer = self.recorder.writer
        key = (writer.path, writer.active)
        now = time.monotonic()
        cached = self._recording_cache
        if refresh or cached is None or cached[0] != key or now - cached[1] >= 1.:
            status = self.recorder.snapshot()
            self._recording_cache = (key, now, status)
        else:
            status = cached[2]
        # Only the rotated file list can lag by up to one second; live writer
        # errors and active state must never be hidden by this UI cache.
        return {**status, 'active': writer.active, 'error': writer.error}

    def _seed(self, pose):
        self.pose = dict(pose)
        self.input.current_position = [pose[n] for n in self.names]
        self.input.current_velocity = [0.] * len(self.names)
        self.input.current_acceleration = [0.] * len(self.names)
        self.input.target_position = list(self.input.current_position)
        self.input.target_velocity = [0.] * len(self.names)
        self.input.target_acceleration = [0.] * len(self.names)
        self.otg.reset()
        if self.guard:
            self.guard.current = dict(pose)
            self.guard.guard.rebase(pose)

    def touch(self):
        with self.lock:
            self.client_at = time.monotonic()

    def start_tracking(self):
        with self.lock:
            self.tracking_started = True
            self.hold()

    def reset_connection(self):
        with self.lock:
            self.telemetry_seen = False

    def set_direct_mode(self, enabled):
        """Change the command path only while hardware motion is disabled."""
        if not isinstance(enabled, bool):
            raise ValueError("enabled 必须为布尔值")
        with self.lock:
            if self.armed or self.stop_pending:
                raise ValueError("请先关闭下发，再切换直接控制模式")
            if enabled == self.direct_mode:
                return
            self.direct_mode = enabled
            self.guard = None if enabled else self._normal_guard
            self._seed(self.pose)
            self.target = dict(self.pose)
            self.target_at = 0.
            self.frame_id = -1
            self.finger_holds.clear()
            self.finger_hold_context.clear()
            self.sample_collision = {}
            self.phase = "holding"
            self.last_error = ""
            self.record_action("set_direct_mode", enabled=enabled)

    def telemetry_issue(self, state):
        # Native receivedAt is set only by the telemetry callback and reset on
        # reconnect. rx_ok also counts other traffic and is not a seed proof.
        if not state.get("connected"):
            self.telemetry_seen = False
            return "请先连接 SDK"
        if state.get("telemetry_age_ms", -1) >= 0:
            self.telemetry_seen = True
        if not self.telemetry_seen:
            return "首次开启请先接收一包有效的 A3 手部遥测"
        if state.get("hand_side") and state["hand_side"] != self.side:
            return "遥测左右手与所选模型不一致"
        for i, motor in enumerate(state.get("mit", [])):
            if motor.get("online") and (motor.get("fault") or motor.get("state") == 5):
                return f"外接拇指电机 M{i+1} 报告故障"
        return ""

    def arm_issue(self, state):
        if not self.tracking_started:
            return "请先开始识别"
        if not self.direct_mode and self.guard is None and self.mode not in ("off", "monitor"):
            return "请先选择碰撞停止或 Mink 保护模式"
        if state.get("homing_active"):
            return "自研电机找零中"
        return self.telemetry_issue(state)

    def push(self, pose, frame_id):
        with self.lock:
            target = self.device.constrain_pose(pose)
            for finger, requested in list(self.finger_holds.items()):
                context = self.finger_hold_context.get(finger, {})
                neighbour_moved = any(abs(self.pose[n] - q) > 1e-7 for n, q in context.items())
                if neighbour_moved or max(abs(target[n] - q) for n, q in requested.items()) > .005:
                    del self.finger_holds[finger]
                    self.finger_hold_context.pop(finger, None)
            self.target = dict(target)
            self.target_at = time.monotonic()
            self.frame_id = frame_id
            if self.armed:
                if not self.direct_mode:
                    self.input.target_position = [target[n] for n in self.names]
                    for finger in self.finger_holds:
                        self._stop_joints(self._finger_joints(finger), hold=True)
                self.phase = "tracking"

    def hold(self):
        with self.lock:
            # Match A3.0: finish the last checked target without resetting
            # derivatives or clearing a latched fault/collision hold.
            if not self.last_error and not self.finger_holds:
                self.phase = "holding"

    def arm(self, enabled, *, prepare_seed=None):
        if not isinstance(enabled, bool):
            raise ValueError("enabled 必须为布尔值")
        with self.lock:
            if not enabled:
                self.stop()
                return
            if self.armed:
                return
            issue = self.arm_issue(self.device.get_state())
            if issue:
                raise ValueError(issue)
            # Missing telemetry uses the last command or initial software pose.
            seed = self.device.starting_pose(self.pose)
            if self.guard is not None:
                self.guard.filter.sync_pose_from_dict(seed)
                if self.guard.filter.data.ncon:
                    raise ValueError("起步姿态存在碰撞，请先调整姿态")
            if prepare_seed is not None:
                prepare_seed(seed)
            self._seed(seed)
            try:
                self.device.enable_all(start_pose=seed)
            except Exception as exc:
                self._record("record_error", exc, action="enable", start_pose=seed)
                raise
            self._record("record_command", self._preview_targets(seed), pose=seed,
                         frame_id=self.frame_id, source="a32_enable_seed")
            self.target = dict(seed)
            self.target_at = 0.
            self.finger_holds.clear()
            self.finger_hold_context.clear()
            self.sample_collision = {}
            self.phase = "holding"
            self.last_error = ""
            self.client_at = time.monotonic()
            self.armed = True
            self._last_write_at = 0.
            self.timing_violations = {'below_1ms': 0, 'above_4ms': 0, 'samples': 0}
            self.record_action("enable", start_pose=seed, frame_id=self.frame_id)

    def reject_input(self, reason):
        """Latch a bad input; send a hardware stop only if control is active."""
        with self.lock:
            if self.armed or self.stop_pending:
                self.stop(reason)
            else:
                self.last_error = reason
                self.phase = "blocked"

    def stop(self, reason=""):
        with self.lock:
            self.armed = False
            self._seed(self.pose)
            self.target = dict(self.pose)
            self.target_at = 0.
            if reason:
                self.last_error = reason
                self.phase = "blocked"
            try:
                self.stop_pending = True
                if self.device.is_connected:
                    self.device.stop()
                self.stop_pending = False
            finally:
                self.record_action("stop", reason=reason, frame_id=self.frame_id)

    def step(self, dt):
        entered = time.perf_counter()
        with self.lock:
            acquired = time.perf_counter()
            # Observe the SDK cache independently of foreground browser polling.
            if not self.armed:
                if not self.observer.is_alive():
                    self._record_state()
                return
            try:
                state = getattr(self.device, 'get_control_state', self.device.get_state)()
                observed = time.perf_counter()
                issue = self.telemetry_issue(state)
                if issue:
                    raise ValueError(issue)
                if state.get("homing_active"):
                    raise ValueError("自研电机找零中，已停止下发")
                now = time.monotonic()
                if now - self.client_at > self.CLIENT_TIMEOUT:
                    raise ValueError("网页心跳超时，已停止下发")
                if not math.isfinite(dt) or dt <= 0:
                    raise ValueError("控制采样间隔无效，已停止下发")
                if now - self.target_at > self.INPUT_TIMEOUT:
                    self.hold()
                # Follow elapsed time within the accepted 1--4 ms envelope.
                # A larger stall never creates a large catch-up target.
                planner_dt = None if self.direct_mode else min(dt, self.MAX_PLANNER_STEP)
                if self.direct_mode:
                    if not self.target_at:
                        return  # A mode change/arm waits for a fresh accepted observation.
                    candidate = self.device.constrain_pose(self.target)
                else:
                    candidate = self._checked_step(planner_dt)
                    for name, velocity_limit in zip(self.names, self.input.max_velocity):
                        previous = self.pose[name]
                        limit = velocity_limit * planner_dt
                        if abs(candidate[name]-previous) > limit + 2e-6:
                            raise ValueError(f'相邻下发目标步进超限：{name}')
                planned = time.perf_counter()
                try:
                    targets = getattr(self.device, 'send_control_targets', self.device.send_joint_targets)(candidate, clamp=False)
                except Exception as exc:
                    self._record("record_command", self._preview_targets(candidate), pose=candidate,
                                 sent=False, error=exc, control_dt_s=dt, planner_dt_s=planner_dt, frame_id=self.frame_id,
                                 source="a32_tracking_direct" if self.direct_mode else "a32_tracking")
                    raise
                sent = time.perf_counter()
                interval_ms = (sent-self._last_write_at)*1000 if self._last_write_at else None
                self._last_write_at = sent
                native = targets.get('native_send_timing') or {}
                self._native_spacing = bool(native)
                measured = native.get('interval_ms', interval_ms)
                if measured is not None and measured >= 0:
                    self.timing_violations['samples'] += 1
                    self.timing_violations['below_1ms'] += int(measured < 1.)
                    self.timing_violations['above_4ms'] += int(measured > 4.)
                previous_record_ms = self.control_timing.get('command_record_ms')
                self.control_timing = dict(control_dt_ms=dt*1000, direct_mode=self.direct_mode,
                    planner_dt_ms=None if planner_dt is None else planner_dt*1000,
                    lock_wait_ms=(acquired-entered)*1000, sdk_state_ms=(observed-acquired)*1000,
                    plan_and_record_state_ms=(planned-observed)*1000, encode_write_ms=(sent-planned)*1000,
                    previous_command_record_ms=previous_record_ms,
                    host_write_interval_ms=interval_ms, native_write_interval_ms=native.get('interval_ms'),
                    native_write_duration_ms=native.get('duration_ms'), native_write_sequence=native.get('sequence'),
                    native_write_started_ns=native.get('started_ns'), native_write_finished_ns=native.get('finished_ns'),
                    planner_lost_ms=0. if planner_dt is None else max(0., dt-planner_dt)*1000)
                record_began = time.perf_counter()
                self._record("record_command", targets, pose=candidate, control_dt_s=dt, planner_dt_s=planner_dt,
                             frame_id=self.frame_id,
                             source="a32_tracking_direct" if self.direct_mode else "a32_tracking",
                             control_timing=self.control_timing)
                recorded = time.perf_counter()
                self.control_timing['command_record_ms'] = (recorded-record_began)*1000
                # Publish only the sample accepted by the SDK write.
                self.pose = candidate
                if not self.direct_mode:
                    self.output.pass_to_input(self.input)
                    self.input.current_position = [candidate[n] for n in self.names]
                self.last_sent = targets
                self.ticks += 1
                if self.finger_holds:
                    self.phase = "collision_limited"
                elif self.phase == "collision_limited":
                    self.phase = "tracking"
                self.samples.append({"tick": self.ticks, "frame_id": self.frame_id,
                                     "direct_mode": self.direct_mode,
                                     "sent_at_unix_ms": time.time() * 1000.,
                                     "joints": dict(candidate), "self": targets.get("self"),
                                     "mit_pos": targets.get("mit_pos"), "control_timing": dict(self.control_timing)})
                if not self.observer.is_alive():
                    self._record_state()
                finished = time.perf_counter()
                self.control_timing.update(post_send_ms=(finished-sent)*1000,
                                           step_total_ms=(finished-entered)*1000,
                                           lock_held_ms=(finished-acquired)*1000)
                self.samples[-1]['control_timing'] = dict(self.control_timing)
            except Exception as exc:
                self._record("record_error", exc, control_dt_s=dt, sample_ticks=self.ticks,
                             frame_id=self.frame_id)
                try:
                    self.stop(str(exc))
                except Exception as stop_error:
                    self.last_error = f"{exc}; stop: {stop_error}"

    def _run(self):
        with precise_control_timer(), control_waiter(self.done) as wait:
            # Python 3.11/Windows monotonic() uses GetTickCount64 (~15.6 ms).
            # A 5 ms loop can read the same tick twice and incorrectly fault
            # on dt == 0. QPC/perf_counter supplies the required timer precision.
            clock = time.perf_counter
            previous = clock()
            deadline = previous + self.PERIOD
            while not wait(max(0., max(deadline, getattr(self, '_last_write_at', 0.) +
                                       (0. if getattr(self, '_native_spacing', False) else .001)) - clock())):
                now = clock()
                if now <= previous:
                    continue  # No elapsed time means no sample, not a motor fault.
                self.step(now - previous)
                previous = now
                deadline += self.PERIOD
                if deadline <= clock():
                    # Work already consumed the wait budget; do not append
                    # another full period. The minimum write spacing above
                    # prevents burst catch-up after a missed deadline.
                    deadline = clock()

    def snapshot(self, after_tick=None, *, refresh_device=True):
        # SDK JSON and filesystem enumeration do not hold the planner lock.
        state = self._snapshot_state(refresh_device)
        recording = self._snapshot_recording(refresh_device)
        with self.lock:
            telemetry_issue = self.telemetry_issue(state)
            result = {**state, "model_id": self.profile.model_id, "hand_side": self.side,
                    "armed": self.armed, "last_error": self.last_error or state.get("last_error", ""),
                    "control_timing": dict(self.control_timing),
                    "packet_timing": {**self.timing_violations, 'target_period_ms': self.PERIOD*1000,
                                      'required_interval_ms': [1, 4], 'hard_realtime_guaranteed': False},
                    "target_ready": not bool(self.last_error),
                    "hand_recent": time.monotonic() - self.target_at <= self.INPUT_TIMEOUT,
                    "tracking_started": self.tracking_started, "arm_issue": self.arm_issue(state),
                    "telemetry_seen": self.telemetry_seen, "control_telemetry_issue": telemetry_issue,
                    "sample_collision_ready": self.guard is not None,
                    "collision_mode": "off" if self.direct_mode else self.mode,
                    "direct_mode": self.direct_mode,
                    "collision_protection_enabled": self.guard is not None,
                    "tracking_transition": {"phase": self.phase},
                    "held_fingers": sorted(self.finger_holds),
                    "limit_recoveries": self.limit_recoveries, "last_limit_event": self.last_limit_event,
                    "boundary_roundoffs": self.boundary_roundoffs,
                    "ruckig_enabled": not self.direct_mode, "sample_ticks": self.ticks, "frame_id": self.frame_id,
                    "command_joint_angles_rad": dict(self.pose),
                    "ruckig_joint_angles_rad": dict(self.pose), "target_joint_angles_rad": dict(self.target),
                    "sample_collision": self.sample_collision if self.guard is not None else {
                        "mode": "off" if self.direct_mode else self.mode,
                        "checked": False, "protection_enabled": False,
                        "resolution": "disabled"}, "last_sent": self.last_sent,
                    "transport": "native_sdk", "sample_period_s": self.PERIOD,
                    "recording": {**recording,
                                  "error": self.recording_error or self.recorder.writer.error}}
            if after_tick is not None:
                if isinstance(after_tick, bool) or not isinstance(after_tick, int) or after_tick < 0:
                    raise ValueError("after_tick 必须为非负整数")
                result["command_samples"] = [s for s in self.samples if s["tick"] > after_tick]
                oldest = self.samples[0]["tick"] if self.samples else self.ticks + 1
                result["dropped_command_samples"] = max(0, oldest - after_tick - 1)
            return result

    def close(self):
        self.done.set()
        with self.lock:
            try:
                self.stop()
            finally:
                try:
                    self.device.close()
                finally:
                    self._record("stop", reason="session_closed")
        if self.thread.is_alive():
            self.thread.join(timeout=2)
        if self.observer.is_alive():
            self.observer.join(timeout=2)
