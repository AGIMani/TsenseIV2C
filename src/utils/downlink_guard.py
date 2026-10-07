"""
downlink_guard.py — 下发安全网 + 监控探针（"数据下发 expert"）

背景：GestureDownlink 线程每 tick 把 Ruckig 采样结果转 Hardware 空间后下发。
正常情况下 Ruckig 保证相邻两帧的位移不超过每类关节的 max_velocity*dt，
但下列场景会绕过这个约束，产生真实的"跳变"帧：

  · ``RuckigJointInterpolator.seed_ramp()`` / ``reset()``——业务上明确要求
    立刻落到某个姿态（回零完成、开同步、手再现），这类跳变是预期的。
  · Ruckig 内部 ``otg.update()`` 抛异常（数值病态/nan 输入等）——这类失效
    目前会让该关节"保持不动"，但如果只有部分关节失效、其余关节正常追踪
    大目标，也可能在下一帧追上时看起来像局部爆冲。
  · 未预见的软件 bug（比如某处直接写了绝对目标绕过 Ruckig）。

DownlinkGuard 是最后一道防线：在真正调用 ``usb.send_joint_angles`` 之前，
比较这一帧与上一帧「已下发」帧的差值。若某关节单 tick 位移超出其类型允许
最大速度的安全倍数，且不在"预期跳变"窗口内，就钳制为按最大速度前进一步，
而不是把整个冲量发给下位机；同时记录一条监控事件（节流打印 + 计数），
供日志/未来的监控面板读取，用来回答"下发到底有没有突然的爆冲"。

用法（GestureDownlink 线程每 tick）：

    cfg = gesture_joint_interp.sample()
    expected_jump = gesture_joint_interp.pop_reset_flag()
    faults = gesture_joint_interp.last_faults
    hw_cfg = convert_ui_to_hardware(cfg, hand_side=side)
    hw_cfg = downlink_guard.filter(
        hw_cfg, expected_jump=expected_jump, ruckig_faults=faults
    )
    usb.send_joint_angles(hw_cfg, ...)
"""
from __future__ import annotations

import math
import threading
import time
from collections import deque
from typing import Deque, Dict, FrozenSet, Optional

from utils.log_control import emit as log_emit
from utils.ruckig_interpolator import (
    _DEFAULT_CONSTRAINTS,
    _JOINT_TYPES,
    _to_canonical_name,
)


def _log(msg: str, tag: str = "DownlinkGuard") -> None:
    log_emit(msg, tag)


# 允许超过 Ruckig 标称最大速度的倍数，超过才判定为异常爆冲
# （留裕量：不同关节类型/单位换算误差、瞬时抖动不应误报）。
_SAFETY_FACTOR = 3.0
_MIN_DT = 1e-4
# 同一关节的告警日志最小间隔，避免持续爆冲刷屏
_LOG_THROTTLE_S = 1.0


def _max_vel_for(canonical_name: str) -> float:
    jtype = _JOINT_TYPES.get(canonical_name, "flexion")
    max_vel = _DEFAULT_CONSTRAINTS.get(jtype, _DEFAULT_CONSTRAINTS["flexion"])[0]
    return max_vel


class DownlinkGuard:
    """按帧钳制下发数据，监控并记录任何超出物理限速的跳变。"""

    def __init__(self, max_events: int = 200) -> None:
        self._lock = threading.Lock()
        self._last_cfg: Dict[str, float] = {}
        self._last_ts: Optional[float] = None
        self._events: Deque[dict] = deque(maxlen=max_events)
        self._last_log_ts: Dict[str, float] = {}
        self._total_ticks = 0
        self._total_clamped_ticks = 0
        self._total_fault_ticks = 0

    def reset(self) -> None:
        """手势消失 / 手侧切换等场景：清空基线，下一帧不做钳制判定。"""
        with self._lock:
            self._last_cfg = {}
            self._last_ts = None

    def filter(
        self,
        cfg: dict,
        *,
        now: Optional[float] = None,
        expected_jump: bool = False,
        ruckig_faults: FrozenSet[str] = frozenset(),
    ) -> dict:
        """检查并按需钳制 ``cfg``，返回可安全下发的新字典（不修改入参）。

        ``expected_jump=True``：本帧是 ``seed_ramp()``/``reset()`` 之后的
        第一帧，业务上明确要求跳变，直接接受为新基线，不做钳制。
        """
        if not cfg:
            return cfg
        now = now if now is not None else time.perf_counter()

        log_line: Optional[str] = None
        with self._lock:
            self._total_ticks += 1
            last_cfg = self._last_cfg
            last_ts = self._last_ts

            if last_ts is None or expected_jump:
                accepted = dict(cfg)
                self._last_cfg = accepted
                self._last_ts = now
                if expected_jump and last_ts is not None:
                    self._events.append({"kind": "expected_reset", "ts": now})
                if ruckig_faults:
                    self._total_fault_ticks += 1
                return accepted

            dt = max(now - last_ts, _MIN_DT)
            out = dict(cfg)
            clamped: list[tuple[str, float, float, bool]] = []
            for name, val in cfg.items():
                prev = last_cfg.get(name)
                if prev is None:
                    continue
                canonical = _to_canonical_name(str(name))
                max_step = _max_vel_for(canonical) * dt * _SAFETY_FACTOR
                delta = float(val) - float(prev)
                if abs(delta) > max_step:
                    out[name] = float(prev) + math.copysign(max_step, delta)
                    clamped.append((str(name), delta, max_step, canonical in ruckig_faults))

            self._last_cfg = dict(out)
            self._last_ts = now

            if ruckig_faults:
                self._total_fault_ticks += 1

            if clamped:
                self._total_clamped_ticks += 1
                self._events.append(
                    {"kind": "clamped", "ts": now, "dt": dt, "joints": list(clamped)}
                )
                key = clamped[0][0]
                last_log = self._last_log_ts.get(key, 0.0)
                if now - last_log >= _LOG_THROTTLE_S:
                    self._last_log_ts[key] = now
                    detail = "; ".join(
                        f"{n}: Δ{d:+.4f}rad(限{lim:.4f}){'[Ruckig失效]' if f else ''}"
                        for n, d, lim, f in clamped[:5]
                    )
                    log_line = f"检测到下发爆冲，已钳制({dt * 1000:.1f}ms): {detail}"

        if log_line:
            _log(log_line)
        return out

    def stats(self) -> dict:
        """供监控/诊断读取：累计计数与最近事件（只读快照）。"""
        with self._lock:
            return {
                "total_ticks": self._total_ticks,
                "total_clamped_ticks": self._total_clamped_ticks,
                "total_fault_ticks": self._total_fault_ticks,
                "recent_events": list(self._events)[-20:],
            }


__all__ = ["DownlinkGuard"]
