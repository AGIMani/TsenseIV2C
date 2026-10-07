"""MediaPipe → 下发 H7 端到端耗时（控制台主看这个）。

环境变量：
  LATENCY_LOG=0          关闭（默认开）
  LATENCY_LOG_INTERVAL   控制台最小间隔秒（默认 0.5）
"""

from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass, field

from utils.log_control import emit, register_tag

TAG = "Latency"
register_tag(TAG, "MediaPipe→H7 端到端耗时（控制台默认开）", enabled=True)

_ENABLED = os.environ.get("LATENCY_LOG", "1").strip().lower() not in (
    "0",
    "false",
    "no",
    "off",
)
_INTERVAL_S = float(os.environ.get("LATENCY_LOG_INTERVAL", "0.5"))

_lock = threading.Lock()
_frame_seq = 0
_last_console = 0.0
_awaiting_usb: "FrameLatency | None" = None
_latest_pushed: "FrameLatency | None" = None
_latest_ready: "FrameLatency | None" = None


@dataclass
class FrameLatency:
    frame_id: int
    t_mp_start: float
    t_mp_end: float = 0.0
    t_retarget_end: float = 0.0
    t_physics_end: float = 0.0
    t_emit: float = 0.0
    t_push: float = 0.0
    t_usb: float = 0.0
    infer_ms: float = 0.0
    retarget_ms: float = 0.0
    physics_ms: float = 0.0
    hand_side: str = ""
    extras: dict = field(default_factory=dict)

    def mark_mp_done(self) -> None:
        self.t_mp_end = time.perf_counter()
        self.infer_ms = (self.t_mp_end - self.t_mp_start) * 1000.0

    def mark_retarget_done(self, *, solve_ms: float | None = None) -> None:
        now = time.perf_counter()
        self.t_retarget_end = now
        if solve_ms is not None:
            self.retarget_ms = float(solve_ms)
        elif self.t_mp_end > 0.0:
            self.retarget_ms = (now - self.t_mp_end) * 1000.0

    def mark_physics_done(self, *, elapsed_ms: float | None = None) -> None:
        now = time.perf_counter()
        self.t_physics_end = now
        if elapsed_ms is not None:
            self.physics_ms = float(elapsed_ms)
        elif self.t_retarget_end > 0.0:
            self.physics_ms = (now - self.t_retarget_end) * 1000.0

    def mark_emit(self) -> None:
        self.t_emit = time.perf_counter()

    def mark_push(self) -> None:
        self.t_push = time.perf_counter()

    def mark_usb(self) -> None:
        self.t_usb = time.perf_counter()


def enabled() -> bool:
    return _ENABLED


def log_interval_s() -> float:
    return float(_INTERVAL_S)


def begin_frame(*, hand_side: str = "") -> FrameLatency | None:
    """MediaPipe 推理开始时调用。关闭时返回 None。"""
    if not _ENABLED:
        return None
    global _frame_seq
    with _lock:
        _frame_seq += 1
        fid = _frame_seq
    return FrameLatency(
        frame_id=fid,
        t_mp_start=time.perf_counter(),
        hand_side=str(hand_side or ""),
    )


def publish_ready(lat: FrameLatency | None) -> None:
    """Tracker 发出 joint_angles 前调用：标记 emit 时刻供 Widget push 关联。"""
    if not _ENABLED or lat is None:
        return
    lat.mark_emit()
    with _lock:
        global _latest_ready
        _latest_ready = lat


def peek_ready() -> FrameLatency | None:
    if not _ENABLED:
        return None
    with _lock:
        return _latest_ready


def note_pushed(lat: FrameLatency | None) -> None:
    """Widget 把手势目标 push 进 Ruckig 后调用。"""
    if not _ENABLED or lat is None:
        return
    lat.mark_push()
    with _lock:
        global _awaiting_usb, _latest_pushed
        _awaiting_usb = lat
        _latest_pushed = lat


def note_usb_sent(*, ok: bool = True, usb: bool = True) -> None:
    """手势下行：sample 后调用。``usb=False`` 表示未连串口，仍报管线耗时。"""
    if not _ENABLED or not ok:
        return
    global _awaiting_usb, _last_console
    with _lock:
        lat = _awaiting_usb
        if lat is None:
            return
        _awaiting_usb = None
        lat.mark_usb()
        lat.extras["usb"] = bool(usb)
        now_mono = time.monotonic()
        if now_mono - _last_console < _INTERVAL_S:
            return
        _last_console = now_mono

    _emit_line(lat)


def _ms(a: float, b: float) -> float:
    if a <= 0.0 or b <= 0.0 or b < a:
        return 0.0
    return (b - a) * 1000.0


def _emit_line(lat: FrameLatency) -> None:
    post_ms = _ms(
        lat.t_retarget_end if lat.t_physics_end <= 0.0 else lat.t_physics_end,
        lat.t_emit,
    )
    handoff_ms = _ms(lat.t_emit, lat.t_push)
    usb_wait_ms = _ms(lat.t_push, lat.t_usb)
    keyframe_ms = _ms(lat.t_mp_start, lat.t_emit)
    e2e_ms = _ms(lat.t_mp_start, lat.t_usb)
    side = lat.hand_side or "?"
    usb_flag = "" if lat.extras.get("usb", True) else " usb=skip"
    emit(
        f"#{lat.frame_id} side={side} | "
        f"e2e={e2e_ms:.1f}ms keyframe={keyframe_ms:.1f}ms | "
        f"mp={lat.infer_ms:.1f} retarget={lat.retarget_ms:.1f} "
        f"phys={lat.physics_ms:.1f} post={post_ms:.1f} "
        f"handoff={handoff_ms:.1f} usb_wait={usb_wait_ms:.1f}{usb_flag}",
        TAG,
    )
