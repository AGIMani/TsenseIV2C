"""Lightweight real-time control/downlink path shared by the test window.

The ordering is deliberately fixed and mirrors ``src/app/widget.py``::

    collision-safe retarget target
      -> Ruckig jerk-limited interpolation
      -> UI-angle to hardware-angle conversion
      -> DownlinkGuard last-line rate check
      -> angle-to-stroke linkage mapping
      -> UsbController H7 batch frame

Normal streaming is opt-in (``armed=False`` by default).  One-key homing owns
the link while active and follows Form2's linear-first, rotary-sequential state
machine, including telemetry evidence before a rotary motor is declared homed.
"""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Mapping

from protocol import convert_ui_to_hardware, convert_hardware_to_ui
from tracking_transition import TrackingTransition
from usb.offline import (
    DEFAULT_HOMING_IQ_SEEK,
    DOWNLINK_HZ_DEFAULT,
    LINEAR_MOTOR_NUM,
    LM_HOMING_RESET,
    LM_HOMING_START,
    ROTARY_HW_SPI_LABELS,
    UsbController,
    UsbDeviceInfo,
    joint_angles_to_hand_positions,
    rotary_bus_slot_index,
)
from utils.downlink_guard import DownlinkGuard
from utils.control_clock import precise_control_timer
from utils.hand_init import get_init_pose
from utils.ruckig_interpolator import (
    PhaseSynchronizedRuckigInterpolator,
    RuckigJointInterpolator,
)


# Kept identical to src/ui/form2_window.py and deprecated/main_app_v3.py.
ONEKEY_RM_ZERO_SEQUENCE = (
    (1, 1),
    (1, 0),
    (0, 1),
    (0, 0),
    (3, 0),
    (3, 1),
)
ONEKEY_LM_DELAY_S = 1.0
ONEKEY_TICK_S = 0.05
ONEKEY_RM_ENABLE_DELAY_S = 0.2
ONEKEY_RM_ZERO_TIMEOUT_S = 15.0
ONEKEY_RM_IQ_SEEK_ABS = 200.0
ONEKEY_RM_MIN_SEEK_S = 1.2
ONEKEY_RM_MOTION_DEG = 0.5
# Never integrate a suspended process into a large actuator position jump.
MAX_CONTROL_GAP_S = 0.1


def _normalise_side(value: str) -> str:
    return "right" if str(value).strip().lower().startswith("r") else "left"


def _finite_pose(values: Mapping[str, float]) -> dict[str, float]:
    pose: dict[str, float] = {}
    for name, value in values.items():
        numeric = float(value)
        if not math.isfinite(numeric):
            raise ValueError(f"joint {name!r} is NaN or infinity")
        joint_name = str(name)
        # pinky_rota is a fixed mechanical DOF in light_test.  Keep the safety
        # invariant even for replay files or callers produced before the
        # retarget solver removed it from its optimisation vector.
        pose[joint_name] = (
            0.0 if joint_name.endswith("_pinky_rota_joint") else numeric
        )
    return pose


class LightControlPipeline:
    """Threaded Ruckig + H7 downlink pipeline with a homing state machine."""

    def __init__(
        self,
        *,
        usb: UsbController | Any | None = None,
        interpolator: (
            PhaseSynchronizedRuckigInterpolator
            | RuckigJointInterpolator
            | Any
            | None
        ) = None,
        downlink_guard: DownlinkGuard | Any | None = None,
        hand_side: str = "right",
        downlink_hz: float = DOWNLINK_HZ_DEFAULT,
        ruckig_enabled: bool = True,
        allow_direct_downlink: bool = False,
        recorder: Any | None = None,
        source: str = "light_test/mediapipe",
    ) -> None:
        self.usb = usb or UsbController()
        self.interpolator = interpolator or PhaseSynchronizedRuckigInterpolator()
        self.downlink_guard = downlink_guard or DownlinkGuard()
        self.recorder = recorder
        self.source = str(source)
        self._target_frame_id = 0
        self._sample_frame_id = -1
        self.downlink_hz = max(1.0, float(downlink_hz))
        self._period_s = 1.0 / self.downlink_hz
        self._elapsed_sampling = isinstance(self.interpolator, (PhaseSynchronizedRuckigInterpolator, RuckigJointInterpolator))
        self._last_sample_time: float | None = None
        self._last_sample_dt = self._period_s
        self._sample_elapsed_s = 0.0
        self._timed_samples = 0
        self._timing_faults = 0
        self._timer_precision = False
        self._lock = threading.RLock()
        # A tracking event must not reseed Ruckig between sample() and accepting
        # that sample.  Always acquire motion before the metadata lock.
        self._motion_lock = threading.RLock()
        self._tracking_transition = TrackingTransition(self.interpolator)
        self._stop_event = threading.Event()
        self._wake_event = threading.Event()
        self._thread: threading.Thread | None = None

        self._hand_side = _normalise_side(hand_side)
        self._ruckig_enabled = bool(ruckig_enabled)
        self._allow_direct_downlink = bool(allow_direct_downlink)
        from online_control.online_motion_track.direct_filter import DirectJointFilter
        self._direct_filter = DirectJointFilter()
        self._direct_filter_at = None
        self._direct_filter_status = None
        self._armed = False
        self._tracking_active = False
        self._target_ready = False
        self._target_block_reason: str | None = None
        self._collision_mode = "off"
        self._sample_collision_guard: Any | None = None
        self._sample_collision_diagnostics: dict[str, Any] = {}
        self._target_ui: dict[str, float] = {}
        self._latest_ui: dict[str, float] = {}
        self._latest_hardware: dict[str, float] = {}
        self._latest_linear_mm: list[float] = []
        self._latest_rotary_deg: list[float] = []
        self._last_target_time: float | None = None
        self._last_send_ok: bool | None = None
        self._sample_ticks = 0
        self._send_ticks = 0
        self._send_drops = 0
        self._last_error: str | None = None
        self._last_reconcile = 0.0

        self._homing_active = False
        self._homing_phase = ""
        self._homing_status = "未找零"
        self._homing_result: str | None = None
        self._homing_t0 = 0.0
        self._homing_deadline = 0.0
        self._homing_next_tick = 0.0
        self._homing_queue: list[tuple[int, int]] = []
        self._homing_total = 0
        self._homing_index = 0
        self._homing_fail_caps: list[str] = []
        self._homing_step: tuple[int, int] | None = None
        self._homing_pos0 = 0.0
        self._homing_saw_clear = False
        self._homing_saw_seek_iq = False
        self._homing_saw_motion = False
        self._homing_iq_seek = float(DEFAULT_HOMING_IQ_SEEK)

    # ------------------------------------------------------------------
    # Lifecycle and connection
    # ------------------------------------------------------------------
    def start(self) -> None:
        with self._lock:
            if self._thread is not None and self._thread.is_alive():
                return
            self._stop_event.clear()
            self._thread = threading.Thread(
                target=self._run,
                name="LightControlDownlink",
                daemon=True,
            )
            self._thread.start()

    def close(self, *, disconnect: bool = True) -> None:
        self.set_armed(False)
        self.cancel_one_key_zero("窗口关闭")
        self._stop_event.set()
        self._wake_event.set()
        with self._lock:
            thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=2.0)
        with self._lock:
            self._thread = None
        if disconnect:
            try:
                self.usb.disconnect()
            except Exception:
                pass
        shutdown = getattr(self.interpolator, "shutdown", None)
        if callable(shutdown):
            shutdown()

    @property
    def is_connected(self) -> bool:
        try:
            return bool(self.usb.is_connected)
        except Exception:
            return False

    def scan_devices(self) -> list[UsbDeviceInfo]:
        return list(self.usb.scan_devices())

    def connect(self, device: UsbDeviceInfo) -> bool:
        self.disconnect()
        try:
            ok = bool(self.usb.connect(device))
            if ok:
                self.usb.send_hand_type(self.hand_side)
                with self._lock:
                    self._last_error = None
            return ok
        except Exception as exc:
            with self._lock:
                self._last_error = f"连接失败: {type(exc).__name__}: {exc}"
            return False

    def disconnect(self) -> None:
        self.set_armed(False)
        self.cancel_one_key_zero("串口断开")
        try:
            self.usb.disconnect()
        except Exception as exc:
            with self._lock:
                self._last_error = f"断开异常: {type(exc).__name__}: {exc}"

    # ------------------------------------------------------------------
    # Target / trajectory path
    # ------------------------------------------------------------------
    @property
    def hand_side(self) -> str:
        with self._lock:
            return self._hand_side

    def set_hand_side(self, hand_side: str) -> None:
        side = _normalise_side(hand_side)
        with self._motion_lock:
            with self._lock:
                if side == self._hand_side:
                    return
                self._hand_side = side
                self._last_sample_time = None
                self._armed = False
                self._target_ui = {}
                self._target_ready = False
                self._target_block_reason = None
                self._latest_ui = {}
                self._latest_hardware = {}
                self._latest_linear_mm = []
                self._latest_rotary_deg = []
            self.interpolator.reset()
            self._tracking_transition.reset()
            self.downlink_guard.reset()
            guard = self._sample_collision_guard
            if guard is not None:
                reset = getattr(guard, "reset", None)
                if callable(reset):
                    reset()
        if self.is_connected:
            self.usb.send_hand_type(side)
        self._wake_event.set()

    def observe_tracking_frame(
        self, valid: bool, *, reason: str = "hand_missing", now: float | None = None
    ) -> dict[str, Any]:
        with self._motion_lock:
            with self._lock:
                if not self._tracking_active or self._homing_active:
                    return {"allow_retarget": False, "reset_planner": False}
                current = dict(self._latest_ui) or None
            decision = self._tracking_transition.observe(
                valid,
                now=time.monotonic() if now is None else float(now),
                current=current,
                reason=reason,
            )
            if decision.get("reset_planner"):
                # Reacquisition reseeds the trajectory at the held pose. Time
                # spent confirming the new hand is not time on this new path.
                self._last_sample_time = None
            return decision

    def push_target(
        self, values: Mapping[str, float], *, tracking_generation: int | None = None,
        frame_id: int | None = None,
    ) -> bool:
        target = _finite_pose(values)
        if not target:
            return False
        with self._motion_lock:
            with self._lock:
                if tracking_generation is not None and (
                    tracking_generation != self._tracking_transition.generation
                    or not self._tracking_active
                    or self._homing_active
                    or self._tracking_transition.phase not in {"tracking", "recovering"}
                ):
                    return False
                self._target_ui = dict(target)
                if frame_id is not None:
                    self._target_frame_id = int(frame_id)
                self._last_target_time = time.monotonic()
                self._target_ready = True
                self._target_block_reason = None
                use_ruckig = self._ruckig_enabled
            if use_ruckig:
                self.interpolator.push(target)
        self._wake_event.set()
        return True

    def invalidate_target(self, reason: str) -> None:
        """Close the control/downlink gate without discarding the safe pose.

        Used when MediaPipe reports a hand other than the one selected by the
        operator.  Keeping the latest accepted pose avoids a visual jump, while
        ``target_ready=False`` prevents Ruckig, conversion and physical writes
        until a newly verified target is pushed.
        """
        with self._motion_lock:
            self._last_sample_time = None
            with self._lock:
                self._target_ready = False
                self._armed = False
                self._target_block_reason = str(reason)
                current = dict(self._latest_ui) or None
            self._tracking_transition.block(str(reason), current)
        self._wake_event.set()

    def set_tracking_active(self, active: bool) -> None:
        with self._motion_lock:
            self._last_sample_time = None
            with self._lock:
                self._tracking_active = bool(active)
                self._target_ready = False
                current = dict(self._latest_ui) or None
                if not self._tracking_active:
                    self._armed = False
                    self._target_block_reason = None
            self._tracking_transition.reset(current)

    def set_sample_collision_guard(
        self,
        guard: Any | None,
        *,
        mode: str,
    ) -> None:
        """Install the final 500 Hz collision gate used after Ruckig sampling."""
        collision_mode = str(mode).strip().lower()
        with self._motion_lock:
            if guard is not None:
                current = self.current_ui_pose()
                rebase = getattr(guard, "rebase", None)
                reset = getattr(guard, "reset", None)
                if current and callable(rebase):
                    rebase(current)
                elif callable(reset):
                    reset()
            with self._lock:
                self._sample_collision_guard = guard
                self._collision_mode = collision_mode
                self._sample_collision_diagnostics = {}
                if collision_mode != "off" and guard is None:
                    self._armed = False

    def set_ruckig_enabled(self, enabled: bool) -> None:
        enabled = bool(enabled)
        with self._motion_lock:
            with self._lock:
                if enabled == self._ruckig_enabled:
                    return
                start = dict(self._latest_ui)
                self._ruckig_enabled = enabled
                self._direct_filter_at = None
                self._direct_filter_status = None
                self._last_sample_time = None
                self._armed = False
                self._target_ready = False
                self._target_ui = dict(start)
            self.interpolator.reset()
            self._tracking_transition.reset(start or None)
            self.downlink_guard.reset()
        self._wake_event.set()

    def set_armed(self, armed: bool) -> bool:
        requested = bool(armed)
        with self._lock:
            if requested and (
                self._homing_active
                or not self.is_connected
                or (not self._ruckig_enabled and not self._allow_direct_downlink)
                or not self._tracking_active
                or not self._target_ready
                or (
                    self._collision_mode != "off"
                    and self._sample_collision_guard is None
                )
            ):
                self._armed = False
                return False
            self._armed = requested
            if requested and self._homing_result == "complete":
                self._homing_result = None
            return self._armed

    def current_ui_pose(self) -> dict[str, float] | None:
        with self._lock:
            return dict(self._latest_ui) if self._latest_ui else None

    def rebase_browser_preview(self, pose: Mapping[str, float]) -> None:
        """Restart computation at the visitor's last accepted pose; never arm USB.

        The web session validates the complete model-specific pose first. The
        motion lock prevents an in-flight 500 Hz sample from overwriting it.
        """
        current = _finite_pose(pose)
        with self._motion_lock:
            self._direct_filter_at = None
            self._direct_filter_status = None
            if self.is_connected:
                raise ValueError("浏览器通路不能同时使用服务器串口")
            self.interpolator.reset()
            self.downlink_guard.reset()
            self._tracking_transition.reset(current)
            if self._sample_collision_guard:
                self._sample_collision_guard.rebase(current)
            hw = convert_ui_to_hardware(current, hand_side=self.hand_side)
            linear, rotary = joint_angles_to_hand_positions(hw)
            with self._lock:
                self._armed = False
                self._target_ready = False
                self._target_block_reason = None
                self._target_ui = dict(current)
                self._latest_ui = dict(current)
                self._latest_hardware = hw
                self._latest_linear_mm = list(linear)
                self._latest_rotary_deg = list(rotary)
                self._last_sample_time = None
                self._target_frame_id = -1

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return {
                "connected": self.is_connected,
                "device": (
                    None
                    if getattr(self.usb, "current_device", None) is None
                    else str(self.usb.current_device.display_name)
                ),
                "hand_side": self._hand_side,
                "frame_id": self._target_frame_id,
                "sample_frame_id": self._sample_frame_id,
                "armed": self._armed,
                "tracking_active": self._tracking_active,
                "tracking_transition": self._tracking_transition.snapshot(),
                "target_ready": self._target_ready,
                "target_block_reason": self._target_block_reason,
                "ruckig_enabled": self._ruckig_enabled,
                "control_timing": {
                    "mode": "elapsed" if self._elapsed_sampling else "fixed",
                    "requested_hz": self.downlink_hz,
                    "actual_sample_hz": self._timed_samples / self._sample_elapsed_s if self._sample_elapsed_s else 0.0,
                    "last_step_ms": self._last_sample_dt * 1000,
                    "trajectory_elapsed_s": self._sample_elapsed_s,
                    "timeout_count": self._timing_faults,
                    "max_gap_s": MAX_CONTROL_GAP_S,
                    "windows_timer_1ms": self._timer_precision,
                },
                "collision_mode": self._collision_mode,
                "sample_collision_ready": (
                    self._collision_mode == "off"
                    or self._sample_collision_guard is not None
                ),
                "sample_collision": dict(self._sample_collision_diagnostics),
                "target_joint_angles_rad": dict(self._target_ui),
                "direct_filter": self._direct_filter_status if not self._ruckig_enabled and self._allow_direct_downlink else None,
                "ruckig_joint_angles_rad": dict(self._latest_ui),
                "hardware_joint_angles_rad": dict(self._latest_hardware),
                "linear_strokes_mm": list(self._latest_linear_mm),
                "rotary_positions_deg": list(self._latest_rotary_deg),
                "last_send_ok": self._last_send_ok,
                "sample_ticks": self._sample_ticks,
                "send_ticks": self._send_ticks,
                "send_drops": self._send_drops,
                "last_error": self._last_error,
                "homing_active": self._homing_active,
                "homing_phase": self._homing_phase,
                "homing_status": self._homing_status,
                "homing_result": self._homing_result,
                "homing_step": self._homing_index,
                "homing_total": self._homing_total,
                "downlink_guard": self.downlink_guard.stats(),
            }

    def _run(self) -> None:
        with precise_control_timer() as enabled:
            self._timer_precision = enabled
            try:
                next_tick = time.perf_counter()
                while not self._stop_event.is_set():
                    now = time.perf_counter()
                    if now >= next_tick:
                        # Read time after acquiring the motion lock, not before
                        # waiting on an input update. Skip expired deadlines:
                        # elapsed sampling already accounts for missed time.
                        self._control_tick()
                        next_tick += self._period_s
                        finished = time.perf_counter()
                        if next_tick <= finished:
                            next_tick = finished + self._period_s
                    timeout = max(0.0, min(self._period_s, next_tick - time.perf_counter()))
                    self._wake_event.wait(timeout)
                    self._wake_event.clear()
            finally:
                self._timer_precision = False

    def _control_tick(self, now: float | None = None) -> None:
        with self._motion_lock:
            self._control_tick_locked(now)

    def _control_tick_locked(self, now: float | None = None) -> None:
        now = time.perf_counter() if now is None else float(now)
        if now - self._last_reconcile >= 0.25:
            self._last_reconcile = now
            reconcile = getattr(self.usb, "reconcile_connection", None)
            if callable(reconcile):
                try:
                    reconcile()
                except Exception:
                    pass

        with self._lock:
            homing = self._homing_active
            use_ruckig = self._ruckig_enabled
            target = dict(self._target_ui)
            side = self._hand_side
            armed = self._armed
            target_ready = self._target_ready
            sample_collision_guard = self._sample_collision_guard
            frame_id = self._target_frame_id

        if homing:
            self._direct_filter_at = None
            self._last_sample_time = None
            if now >= self._homing_next_tick:
                self._homing_next_tick = now + ONEKEY_TICK_S
                self._tick_one_key_zero(time.monotonic())
            return

        self._tracking_transition.tick(
            now=time.monotonic(), current=self.current_ui_pose()
        )

        # A handedness/quality gate can invalidate the target while keeping
        # capture alive.  Do not continue processing a stale Ruckig target.
        if not target_ready or self._tracking_transition.phase == "blocked":
            self._direct_filter_at = None
            self._last_sample_time = None
            return

        if use_ruckig:
            self._direct_filter_at = None
            dt = self._period_s if self._last_sample_time is None else now - self._last_sample_time
            if dt <= 0:
                return
            if self._elapsed_sampling and dt > MAX_CONTROL_GAP_S:
                self._timing_faults += 1
                self.invalidate_target(f"控制线程超时 {dt * 1000:.0f} ms，请重新确认手势")
                return
            self._last_sample_time = now
            pose = (self.interpolator.sample(delta_time=dt) if self._elapsed_sampling
                    else self.interpolator.sample())
            self._last_sample_dt = dt
            expected_jump = self.interpolator.pop_reset_flag()
            faults = self.interpolator.last_faults
        else:
            self._last_sample_time = None
            pose = (
                self.current_ui_pose()
                if self._tracking_transition.phase in {"braking", "holding", "confirming"}
                else target or None
            )
            expected_jump = False
            faults = frozenset()
        if not pose:
            return

        try:
            ui_pose = _finite_pose(pose)
            if not use_ruckig and self._allow_direct_downlink:
                current = self.current_ui_pose() or ui_pose
                dt = self._period_s if self._direct_filter_at is None else now-self._direct_filter_at
                if self._direct_filter_at is None or self._direct_filter.pose != current:
                    self._direct_filter.reset(current)
                ui_pose = self._direct_filter.sample(ui_pose, dt)
                self._direct_filter_at = now
            if sample_collision_guard is not None:
                ui_pose, sample_collision_diag = sample_collision_guard.project(
                    ui_pose
                )
                with self._lock:
                    self._sample_collision_diagnostics = dict(
                        sample_collision_diag
                    )
                if sample_collision_diag.get("limited"):
                    sync_current = getattr(
                        self.interpolator,
                        "sync_current_pose",
                        None,
                    )
                    if callable(sync_current):
                        sync_current(ui_pose)
            hardware = convert_ui_to_hardware(ui_pose, hand_side=side)
            if not use_ruckig and self._allow_direct_downlink:
                self._direct_filter.accept(ui_pose)
            hardware_before_guard = dict(hardware)
            hardware = self.downlink_guard.filter(
                hardware,
                now=now,
                expected_jump=expected_jump or (not use_ruckig and self._allow_direct_downlink),
                ruckig_faults=faults,
            )
            linear, rotary = joint_angles_to_hand_positions(hardware)
        except Exception as exc:
            reason = f"采样安全/下发转换失败: {type(exc).__name__}: {exc}"
            with self._lock:
                self._last_error = reason
            self.invalidate_target(reason)
            return

        # Re-read the interlock immediately before the physical write.  A UI
        # stop/homing/side-switch may have happened while linkage conversion
        # was running, and a stale ``armed`` snapshot must never leak one frame.
        with self._lock:
            can_send = self._armed and not self._homing_active
        send_ok: bool | None = None
        if armed and can_send and self.is_connected:
            try:
                send_ok = bool(
                    self.usb.send_joint_angles(
                        hardware,
                        source=self.source,
                        stream=True,
                        hand_side=side,
                        frame_id=frame_id,
                    )
                )
            except Exception as exc:
                send_ok = False
                with self._lock:
                    self._last_error = f"下发异常: {type(exc).__name__}: {exc}"

        with self._lock:
            self._latest_ui = dict(ui_pose)
            self._sample_frame_id = frame_id
            self._direct_filter_status = self._direct_filter.snapshot() if not use_ruckig and self._allow_direct_downlink else None
            self._latest_hardware = dict(hardware)
            self._latest_linear_mm = [float(value) for value in linear]
            self._latest_rotary_deg = [float(value) for value in rotary]
            self._last_send_ok = send_ok
            self._sample_ticks += 1
            if use_ruckig and self._elapsed_sampling:
                self._sample_elapsed_s += self._last_sample_dt
                self._timed_samples += 1
            if send_ok is True:
                self._send_ticks += 1
            elif send_ok is False:
                self._send_drops += 1
        if self.recorder is not None and self.recorder.active:
            self.recorder.record_h7(
                joints_ui=convert_hardware_to_ui(hardware, hand_side=side),
                joints_hw=hardware, frame_id=frame_id,
                hand_side=side, sent=send_ok is True, usb_connected=self.is_connected,
                ruckig_faults=faults, guard_limited=hardware != hardware_before_guard,
                joints_before_guard=hardware_before_guard, joints_ruckig_ui=pose,
                control_dt_s=self._last_sample_dt if use_ruckig and self._elapsed_sampling else None,
            )

    # ------------------------------------------------------------------
    # One-key homing (Form2-compatible state machine)
    # ------------------------------------------------------------------
    @staticmethod
    def _homing_caption(bus: int, motor: int) -> str:
        return f"{ROTARY_HW_SPI_LABELS[bus]} M{motor}"

    def start_one_key_zero(
        self,
        iq_seek: float = DEFAULT_HOMING_IQ_SEEK,
    ) -> bool:
        if not self.is_connected:
            with self._lock:
                self._last_error = "一键找零失败：下位机未连接"
            return False
        with self._motion_lock:
            with self._lock:
                if self._homing_active:
                    return False
                self._armed = False
                self._target_ready = False
                self._homing_active = True
                self._homing_phase = "starting"
                self._homing_status = "正在写入左右手配置…"
                self._homing_result = None
                self._homing_fail_caps = []
                self._homing_iq_seek = max(0.0, float(iq_seek))
                side = self._hand_side
            self._tracking_transition.block("homing", self.current_ui_pose())

        try:
            if not self.usb.send_hand_type(side):
                self._fail_homing("手型配置下发失败")
                return False
            sent = int(
                self.usb.send_all_lm_homing(
                    LM_HOMING_START,
                    self._homing_iq_seek,
                )
            )
        except Exception as exc:
            self._fail_homing(f"直线找零异常: {type(exc).__name__}: {exc}")
            return False
        if sent != int(LINEAR_MOTOR_NUM):
            self._fail_homing(
                f"直线找零只发送 {sent}/{LINEAR_MOTOR_NUM} 帧"
            )
            return False

        now = time.monotonic()
        with self._lock:
            self._homing_phase = "lm_wait"
            self._homing_t0 = now
            self._homing_next_tick = 0.0
            self._homing_status = (
                f"14 路直线找零已下发，等待 {ONEKEY_LM_DELAY_S:.1f}s"
            )
        self._wake_event.set()
        return True

    def cancel_one_key_zero(self, reason: str = "已手动停止") -> bool:
        with self._lock:
            if not self._homing_active:
                return False
            phase = self._homing_phase
            step = self._homing_step
            iq_seek = self._homing_iq_seek
            self._homing_active = False
            self._homing_phase = ""
            self._homing_queue.clear()
            self._homing_step = None
            self._homing_status = reason
            self._homing_result = "cancelled"
        self._stop_homing_hardware(
            phase=phase,
            step=step,
            iq_seek=iq_seek,
        )
        return True

    def _fail_homing(self, reason: str) -> None:
        with self._lock:
            phase = self._homing_phase
            step = self._homing_step
            iq_seek = self._homing_iq_seek
            self._homing_active = False
            self._homing_phase = ""
            self._homing_queue.clear()
            self._homing_step = None
            self._homing_status = f"找零终止：{reason}"
            self._homing_result = "failed"
            self._last_error = self._homing_status
        self._stop_homing_hardware(
            phase=phase,
            step=step,
            iq_seek=iq_seek,
        )

    def _stop_homing_hardware(
        self,
        *,
        phase: str,
        step: tuple[int, int] | None,
        iq_seek: float,
    ) -> None:
        """Best-effort stop for commands already issued by the homing state machine."""
        if not self.is_connected:
            return
        if phase in {"starting", "lm_wait"}:
            try:
                self.usb.send_all_lm_homing(LM_HOMING_RESET, iq_seek)
            except Exception:
                pass
        if step is not None:
            try:
                self.usb.send_rm_drive_motor(step[0], step[1], False)
            except Exception:
                pass

    def _finish_homing(self) -> None:
        with self._lock:
            fails = list(self._homing_fail_caps)
        if fails:
            fail_txt = "、".join(fails)
            with self._lock:
                self._homing_active = False
                self._homing_phase = ""
                self._homing_step = None
                self._homing_status = f"整手一键找零结束（失败 {len(fails)} 轴）：{fail_txt}"
                self._homing_result = "failed"
                self._last_error = self._homing_status
            return
        side = self.hand_side
        init = get_init_pose(side)
        self.interpolator.reset()
        self._tracking_transition.reset(init)
        self.downlink_guard.reset()
        with self._lock:
            self._target_ui = dict(init)
            self._target_ready = False
            self._latest_ui = dict(init)
            self._latest_hardware = convert_ui_to_hardware(init, hand_side=side)
            linear, rotary = joint_angles_to_hand_positions(self._latest_hardware)
            self._latest_linear_mm = list(linear)
            self._latest_rotary_deg = list(rotary)
            self._homing_active = False
            self._homing_phase = ""
            self._homing_step = None
            self._homing_status = "整手一键找零完成；实时下发仍保持关闭"
            self._homing_result = "complete"

    def _latest_rotary_slot(self, bus: int, motor: int):
        try:
            frame = self.usb.read_latest_telemetry()
        except Exception:
            return None
        if not frame:
            return None
        index = rotary_bus_slot_index(bus, motor)
        rotary = frame.get("rotary")
        if index is None or not rotary or index >= len(rotary):
            return None
        return rotary[index]

    def _tick_one_key_zero(self, now: float) -> None:
        with self._lock:
            if not self._homing_active:
                return
            phase = self._homing_phase
        if not self.is_connected:
            self._fail_homing("串口已断开")
            return

        if phase == "lm_wait":
            elapsed = now - self._homing_t0
            remain = max(0.0, ONEKEY_LM_DELAY_S - elapsed)
            with self._lock:
                self._homing_status = f"直线电机找零等待 {remain:.1f}s"
            if elapsed < ONEKEY_LM_DELAY_S:
                return
            with self._lock:
                self._homing_queue = list(ONEKEY_RM_ZERO_SEQUENCE)
                self._homing_total = len(self._homing_queue)
                self._homing_index = 0
            self._kick_next_rotary()
            return

        if phase == "rm_enable_wait":
            if now >= self._homing_deadline:
                self._issue_rotary_zero(now)
            return

        if phase == "rm_seeking":
            self._tick_rotary_seeking(now)

    def _kick_next_rotary(self) -> None:
        with self._lock:
            if not self._homing_queue:
                step = None
            else:
                step = self._homing_queue.pop(0)
                self._homing_index += 1
                self._homing_step = step
                self._homing_saw_clear = False
                self._homing_saw_seek_iq = False
                self._homing_saw_motion = False
                self._homing_pos0 = 0.0
        if step is None:
            self._finish_homing()
            return
        bus, motor = step
        caption = self._homing_caption(bus, motor)
        try:
            ok = bool(self.usb.send_rm_drive_motor(bus, motor, True))
        except Exception:
            ok = False
        if not ok:
            self._fail_homing(f"{caption} 使能发送失败")
            return
        with self._lock:
            self._homing_phase = "rm_enable_wait"
            self._homing_deadline = time.monotonic() + ONEKEY_RM_ENABLE_DELAY_S
            self._homing_status = (
                f"旋转找零 {self._homing_index}/{self._homing_total}："
                f"{caption} 使能"
            )

    def _issue_rotary_zero(self, now: float) -> None:
        with self._lock:
            step = self._homing_step
        if step is None:
            self._fail_homing("内部状态缺少当前旋转电机")
            return
        bus, motor = step
        caption = self._homing_caption(bus, motor)
        slot = self._latest_rotary_slot(bus, motor)
        pos0 = float(slot.pos_actual_mm) if slot is not None else 0.0
        try:
            ok = bool(self.usb.send_rm_zero(bus, motor))
        except Exception:
            ok = False
        if not ok:
            self._fail_homing(f"{caption} 回零发送失败")
            return
        with self._lock:
            self._homing_phase = "rm_seeking"
            self._homing_t0 = now
            self._homing_pos0 = pos0
            self._homing_saw_clear = (
                slot is not None and int(slot.home_label) == 0
            )
            self._homing_status = (
                f"旋转找零 {self._homing_index}/{self._homing_total}："
                f"{caption} 寻零中"
            )

    def _tick_rotary_seeking(self, now: float) -> None:
        with self._lock:
            step = self._homing_step
            elapsed = now - self._homing_t0
        if step is None:
            self._fail_homing("内部状态缺少当前旋转电机")
            return
        bus, motor = step
        caption = self._homing_caption(bus, motor)
        if elapsed > ONEKEY_RM_ZERO_TIMEOUT_S:
            self._skip_timed_out_rotary(caption)
            return
        slot = self._latest_rotary_slot(bus, motor)
        if slot is None:
            return
        home = int(slot.home_label)
        iq_ref = float(slot.iq_ref)
        pos = float(slot.pos_actual_mm)
        with self._lock:
            if abs(iq_ref) >= ONEKEY_RM_IQ_SEEK_ABS:
                self._homing_saw_seek_iq = True
            if abs(pos - self._homing_pos0) >= ONEKEY_RM_MOTION_DEG:
                self._homing_saw_motion = True
            if home == 0:
                self._homing_saw_clear = True
                return
            edge_complete = home >= 1 and self._homing_saw_clear
            iq_complete = (
                home >= 1
                and elapsed >= ONEKEY_RM_MIN_SEEK_S
                and self._homing_saw_seek_iq
                and abs(iq_ref) < 50.0
            )
            motion_complete = (
                home >= 1
                and elapsed >= ONEKEY_RM_MIN_SEEK_S
                and self._homing_saw_motion
                and abs(pos) < 2.0
            )
        if edge_complete or iq_complete or motion_complete:
            with self._lock:
                self._homing_status = f"{caption} 找零成功"
                self._homing_step = None
            self._kick_next_rotary()

    def _skip_timed_out_rotary(self, caption: str) -> None:
        """与 main_app_v3 一致：单轴超时记失败，但继续剩余旋转轴。"""
        with self._lock:
            if caption not in self._homing_fail_caps:
                self._homing_fail_caps.append(caption)
            remain = len(self._homing_queue)
            self._homing_status = (
                f"{caption} 寻零超时，已跳过；剩余 {remain} 轴"
            )
            self._homing_step = None
            self._homing_phase = ""
            self._homing_saw_clear = False
            self._homing_saw_seek_iq = False
            self._homing_saw_motion = False
        self._kick_next_rotary()

    def emergency_stop(self) -> bool:
        self.cancel_one_key_zero("急停终止找零")
        self.set_armed(False)
        init = get_init_pose(self.hand_side)
        with self._lock:
            self._target_ui = dict(init)
            self._target_ready = False
        self.interpolator.reset()
        self.downlink_guard.reset()
        try:
            ok = bool(self.usb.emergency_stop()) if self.is_connected else False
        except Exception as exc:
            with self._lock:
                self._last_error = f"急停异常: {type(exc).__name__}: {exc}"
            return False
        with self._lock:
            self._homing_status = "急停已下发；实时下发已关闭"
            if not ok:
                self._last_error = "急停帧发送失败"
        return ok


__all__ = [
    "DEFAULT_HOMING_IQ_SEEK",
    "LightControlPipeline",
    "ONEKEY_LM_DELAY_S",
    "ONEKEY_RM_ZERO_SEQUENCE",
]
