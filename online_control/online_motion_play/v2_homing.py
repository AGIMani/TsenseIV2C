"""V2 homing and rotary calibration state machines without GUI or device discovery.

The caller serializes calls and drives tick() from its regular service loop.
Stopping cancels future commands; it never invents a hardware emergency command.
"""
from __future__ import annotations

import math
import time

from usb.offline import (
    DEFAULT_HOMING_IQ_SEEK, LM_HOMING_START, ROTARY_HW_SPI_LABELS,
    rotary_bus_slot_index, rotary_calib_step_label, rotary_finger_part_label,
)

ONEKEY_ROTARY_STEPS = ((1, 1), (1, 0), (0, 1), (0, 0), (3, 0), (3, 1))
CALIBRATION_STEPS = ((0, 0), (0, 1), (1, 0), (1, 1), (2, 0), (3, 0), (3, 1))
LINEAR_WAIT_S = 1.0
ENABLE_WAIT_S = 0.2
ROTARY_TIMEOUT_S = 15.0
MIN_SEEK_S = 1.2
SEEK_CURRENT = 200.0
MOTION_DEGREES = 0.5


def _number(value, lower, upper, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{label}必须是有限数值")
    value = float(value)
    if not math.isfinite(value) or not lower <= value <= upper:
        raise ValueError(f"{label}必须在 {lower:g}..{upper:g} 范围内")
    return value


def _connected(usb):
    return usb is not None and bool(usb.is_connected)


class V2Homing:
    """Linear homing, fixed wait, then six rotary axes in the legacy order."""

    def __init__(self, usb, *, clock=None):
        self.usb = usb
        self.clock = clock or time.monotonic
        self.active = False
        self.status = "一键找零"
        self.failures = []
        self.phase = ""
        self.step = None
        self.result = None
        self._queue = []
        self._index = 0
        self._started = 0.0
        self._deadline = 0.0
        self._pos0 = 0.0
        self._saw_clear = self._saw_seek_current = self._saw_motion = False

    @staticmethod
    def caption(step):
        bus, motor = step
        return f"{ROTARY_HW_SPI_LABELS[bus]} M{motor}"

    def _reset_pending(self):
        self.active = False
        self.phase = ""
        self.step = None
        self._queue.clear()
        self._saw_clear = self._saw_seek_current = self._saw_motion = False

    def stop(self, reason="已手动停止"):
        was_active = self.active
        self._reset_pending()
        if was_active:
            self.status = str(reason)
        self.result = None

    def _fail(self, reason):
        self._reset_pending()
        self.status = str(reason)
        self.result = {"title": "一键找零", "text": f"已终止：{reason}"}

    def start(self, side="left", iq_seek=DEFAULT_HOMING_IQ_SEEK):
        if self.active:
            raise ValueError("一键找零正在进行，请先停止")
        if side not in ("left", "right"):
            raise ValueError("手别必须是 left 或 right")
        iq_seek = _number(iq_seek, 0.0, 5000.0, "找零电流")
        if not _connected(self.usb):
            raise ValueError("请先连接下位机串口")
        self.result = None
        self.failures = []
        self._index = 0
        self._queue = []
        self.step = None
        try:
            if not self.usb.send_hand_type(side):
                raise ValueError("H7 左右手配置未能写入，已取消找零")
            if not self.usb.send_all_lm_homing(LM_HOMING_START, iq_seek):
                raise ValueError("直线回零命令未能发出，请检查串口连接")
        except Exception as exc:
            self._fail(str(exc))
            raise ValueError(str(exc)) from exc
        self.active = True
        self.phase = "lm_wait"
        self._started = self.clock()
        self.status = "直线等待 1.0s"

    def _slot(self):
        if self.step is None:
            return None
        try:
            frame = self.usb.read_latest_telemetry()
            index = rotary_bus_slot_index(*self.step)
            slots = frame.get("rotary") if frame else None
            if index is None or not slots or index >= len(slots):
                return None
            slot = slots[index]
            values = (float(slot.home_label), float(slot.iq_ref), float(slot.pos_actual_mm))
            if not all(math.isfinite(value) for value in values):
                return None
            return int(values[0]), values[1], values[2]
        except (AttributeError, IndexError, KeyError, TypeError, ValueError, OverflowError):
            return None
        except Exception:
            # The original read path treats unavailable telemetry as no evidence.
            return None

    def _command(self, name, *args, failure):
        try:
            if getattr(self.usb, name)(*args):
                return True
        except Exception as exc:
            failure = f"{failure}：{exc}"
        self._fail(failure)
        return False

    def _finish(self):
        self._reset_pending()
        if self.failures:
            self.status = f"找零结束（{len(self.failures)} 轴超时）"
            self.result = {
                "title": "一键找零",
                "text": f"找零队列已执行完毕，其中 {len(self.failures)} 轴超时："
                        + "、".join(self.failures) + "；其余旋转轴已继续完成，请检查超时轴。",
            }
        else:
            self.status = "找零完成"
            self.result = {"title": "一键找零", "text": "直线找零与旋转电机依次找零已全部完成。"}

    def _next(self, now):
        if not self._queue:
            self._finish()
            return
        self.step = self._queue.pop(0)
        self._index += 1
        self._saw_clear = self._saw_seek_current = self._saw_motion = False
        self._pos0 = 0.0
        self.status = f"找零 {self._index}/{len(ONEKEY_ROTARY_STEPS)}"
        if not self._command("send_rm_drive_motor", *self.step, True,
                             failure=f"{self.caption(self.step)} 使能发送失败"):
            return
        self.phase = "rm_enable_wait"
        self._deadline = now + ENABLE_WAIT_S

    def tick(self):
        if not self.active:
            return
        if not _connected(self.usb):
            self._fail("串口已断开")
            return
        now = self.clock()
        if self.phase == "lm_wait":
            remaining = max(0.0, LINEAR_WAIT_S - (now - self._started))
            self.status = f"直线等待 {remaining:.1f}s"
            if now - self._started < LINEAR_WAIT_S:
                return
            self._queue = list(ONEKEY_ROTARY_STEPS)
            self._next(now)
            return
        if self.phase == "rm_enable_wait":
            if now < self._deadline:
                return
            slot = self._slot()
            self._pos0 = slot[2] if slot else 0.0
            if not self._command("send_rm_zero", *self.step,
                                 failure=f"{self.caption(self.step)} 回零发送失败"):
                return
            self.phase = "rm_seeking"
            self._started = now
            self._saw_clear = slot is not None and slot[0] == 0
            self._saw_seek_current = self._saw_motion = False
            return
        if self.phase != "rm_seeking":
            return
        elapsed = now - self._started
        if elapsed > ROTARY_TIMEOUT_S:
            label = self.caption(self.step)
            if label not in self.failures:
                self.failures.append(label)
            self._next(now)
            return
        slot = self._slot()
        if slot is None:
            return
        home, current, position = slot
        if abs(current) >= SEEK_CURRENT:
            self._saw_seek_current = True
        if abs(position - self._pos0) >= MOTION_DEGREES:
            self._saw_motion = True
        if home == 0:
            self._saw_clear = True
            return
        if home >= 1 and self._saw_clear:
            self._next(now)
            return
        if home >= 1 and elapsed >= MIN_SEEK_S:
            if self._saw_seek_current and abs(current) < 50.0:
                self._next(now)
            elif self._saw_motion and abs(position) < 2.0:
                self._next(now)


class V2Calibration:
    """Seven explicit rotary calibration steps; changing step sends nothing."""

    def __init__(self, usb):
        self.usb = usb
        self.index = 0
        self.position = 0.0
        self.completed = False

    @property
    def step(self):
        return CALIBRATION_STEPS[self.index]

    def previous(self):
        if self.index > 0:
            self.index -= 1
            self.completed = False

    def next(self):
        if self.index < len(CALIBRATION_STEPS) - 1:
            self.index += 1
            self.completed = False

    def _send(self, name, *args, label):
        if not _connected(self.usb):
            raise ValueError("请先连接下位机串口")
        try:
            ok = getattr(self.usb, name)(*self.step, *args)
        except Exception as exc:
            raise ValueError(f"{label}指令未能发送：{exc}") from exc
        if not ok:
            raise ValueError(f"{label}指令未能发送，请检查串口连接")

    def enable(self):
        self._send("send_rm_drive_motor", True, label="使能")

    def send_position(self, value):
        position = _number(value, -3600.0, 3600.0, "位置")
        self._send("send_rm_pos_motor", position, label="位置")
        self.position = position

    def confirm_zero(self):
        if self.completed:
            raise ValueError("当前最后一步已完成")
        self._send("send_rm_zero", label="回零")
        if self.index == len(CALIBRATION_STEPS) - 1:
            self.completed = True

    def info(self):
        bus, motor = self.step
        finger, part = rotary_finger_part_label(bus, motor)
        return {
            "progress": f"步骤 {self.index + 1} / {len(CALIBRATION_STEPS)}",
            "label": rotary_calib_step_label(bus, motor),
            "hardware": f"协议 spi_bus={bus} → 硬件 {ROTARY_HW_SPI_LABELS[bus]} M{motor}"
                        " (0x22 使能 / 0x21 位置 / 0x25 回零)",
            "finger_part": f"{finger} — {part}",
            "hint": "先使能、发送位置、确认回零；然后手动选择下一个，完成后可关闭校准。",
        }
