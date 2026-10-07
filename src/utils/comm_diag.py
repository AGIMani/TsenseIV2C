"""
comm_diag.py — USB / 相机 / 下行通信诊断日志

用法：各模块在热路径调用 record_*()，每 ~2s 自动输出汇总到日志。
日志文件与 widget 主程序共用 logs/ 目录（由 widget 启动时 basicConfig 配置）。

环境变量 COMM_DIAG=1 可开启（默认关闭，避免终端刷屏）。
"""

from __future__ import annotations

import os
import threading
import time
from typing import Optional
try:
    from utils.log_control import emit as log_emit, is_enabled as tag_enabled, register_tag
except ModuleNotFoundError:
    from .log_control import emit as log_emit, is_enabled as tag_enabled, register_tag

_ENABLED = os.environ.get("COMM_DIAG", "0").strip().lower() in ("1", "true", "yes", "on")
_REPORT_INTERVAL_S = float(os.environ.get("COMM_DIAG_INTERVAL", "2.0"))
_SLOW_WRITE_MS = float(os.environ.get("COMM_DIAG_SLOW_WRITE_MS", "5.0"))
_SLOW_FRAME_MS = float(os.environ.get("COMM_DIAG_SLOW_FRAME_MS", "80.0"))

_WINDOW_TAG_MAP = {
    "USB_TX": "COMM_DIAG_USB_TX",
    "USB_RX": "COMM_DIAG_USB_RX",
    "USB_SERIAL": "COMM_DIAG_USB_SERIAL",
    "CAMERA": "COMM_DIAG_CAMERA",
    "TRACKER": "COMM_DIAG_TRACKER",
    "SWEEP": "COMM_DIAG_SWEEP",
    "FORM2_STREAM": "COMM_DIAG_FORM2_STREAM",
    "WIDGET_SYNC": "COMM_DIAG_WIDGET_SYNC",
}
for _tag, _desc in (
    ("COMM_DIAG_USB_TX", "通信诊断-USB下发统计"),
    ("COMM_DIAG_USB_RX", "通信诊断-USB上行统计"),
    ("COMM_DIAG_USB_SERIAL", "通信诊断-串口缓冲统计"),
    ("COMM_DIAG_CAMERA", "通信诊断-相机统计"),
    ("COMM_DIAG_TRACKER", "通信诊断-追踪统计"),
    ("COMM_DIAG_SWEEP", "通信诊断-扫动统计"),
    ("COMM_DIAG_FORM2_STREAM", "通信诊断-Form2动作流统计"),
    ("COMM_DIAG_WIDGET_SYNC", "通信诊断-Widget同步统计"),
):
    register_tag(_tag, _desc)


class _Window:
    """滑动窗口计数器（线程安全）。"""

    __slots__ = (
        "name", "_lock", "_t0", "_count", "_bytes", "_skip_bp", "_skip_lock",
        "_fail", "_slow", "_sum_ms", "_max_ms", "_max_gap_ms", "_last_event",
        "_extras",
    )

    def __init__(self, name: str) -> None:
        self.name = name
        self._lock = threading.Lock()
        self._t0 = time.monotonic()
        self._count = 0
        self._bytes = 0
        self._skip_bp = 0
        self._skip_lock = 0
        self._fail = 0
        self._slow = 0
        self._sum_ms = 0.0
        self._max_ms = 0.0
        self._max_gap_ms = 0.0
        self._last_event = 0.0
        self._extras: dict[str, int] = {}

    def _maybe_reset(self, now: float) -> Optional[str]:
        if now - self._t0 < _REPORT_INTERVAL_S:
            return None
        dt = max(now - self._t0, 1e-6)
        parts = [f"{self.name} {dt:.1f}s:"]
        if self._count:
            parts.append(f"n={self._count} ({self._count / dt:.1f}/s)")
        if self._bytes:
            parts.append(f"bytes={self._bytes} ({self._bytes / dt / 1024:.1f} KiB/s)")
        if self._skip_bp:
            parts.append(f"skip_bp={self._skip_bp}")
        if self._skip_lock:
            parts.append(f"skip_lock={self._skip_lock}")
        if self._fail:
            parts.append(f"fail={self._fail}")
        if self._slow:
            parts.append(f"slow={self._slow}")
        if self._count and self._sum_ms:
            parts.append(f"avg_ms={self._sum_ms / self._count:.2f}")
        if self._max_ms:
            parts.append(f"max_ms={self._max_ms:.2f}")
        if self._max_gap_ms:
            parts.append(f"max_gap_ms={self._max_gap_ms:.1f}")
        for k, v in sorted(self._extras.items()):
            if v:
                parts.append(f"{k}={v}")
        line = " ".join(parts)
        self._t0 = now
        self._count = 0
        self._bytes = 0
        self._skip_bp = 0
        self._skip_lock = 0
        self._fail = 0
        self._slow = 0
        self._sum_ms = 0.0
        self._max_ms = 0.0
        self._max_gap_ms = 0.0
        self._last_event = 0.0
        self._extras.clear()
        return line

    def bump(self, key: str, n: int = 1) -> None:
        self._extras[key] = self._extras.get(key, 0) + n

    def tick(self) -> None:
        if not _ENABLED:
            return
        now = time.monotonic()
        with self._lock:
            line = self._maybe_reset(now)
        if line:
            tag = _WINDOW_TAG_MAP.get(self.name, "comm_diag")
            if tag_enabled("comm_diag") and tag_enabled(tag):
                log_emit(line, tag)


# ── 全局窗口 ──
usb_tx = _Window("USB_TX")
usb_rx = _Window("USB_RX")
usb_serial = _Window("USB_SERIAL")
camera = _Window("CAMERA")
tracker = _Window("TRACKER")
sweep = _Window("SWEEP")
form2_stream = _Window("FORM2_STREAM")
widget_sync = _Window("WIDGET_SYNC")


def enabled() -> bool:
    return _ENABLED


def warn(msg: str) -> None:
    if _ENABLED and tag_enabled("comm_diag"):
        log_emit(msg, "comm_diag")


def info(msg: str) -> None:
    if _ENABLED and tag_enabled("comm_diag"):
        log_emit(msg, "comm_diag")


def record_usb_tx(
    *,
    ok: bool,
    nbytes: int = 96,
    write_ms: float = 0.0,
    out_waiting: int = 0,
    skip_bp: bool = False,
    skip_lock: bool = False,
    source: str = "",
) -> None:
    if not _ENABLED:
        return
    now = time.monotonic()
    with usb_tx._lock:
        if skip_bp:
            usb_tx._skip_bp += 1
        elif skip_lock:
            usb_tx._skip_lock += 1
        elif ok:
            usb_tx._count += 1
            usb_tx._bytes += nbytes
            usb_tx._sum_ms += write_ms
            if write_ms > usb_tx._max_ms:
                usb_tx._max_ms = write_ms
            if write_ms >= _SLOW_WRITE_MS:
                usb_tx._slow += 1
        else:
            usb_tx._fail += 1
        if out_waiting:
            cur = usb_tx._extras.get("peak_out_wait", 0)
            if out_waiting > cur:
                usb_tx._extras["peak_out_wait"] = out_waiting
    if write_ms >= _SLOW_WRITE_MS:
        warn(
            f"USB_TX 慢写 {write_ms:.2f}ms out_waiting={out_waiting} "
            f"ok={ok} src={source or '-'}"
        )
    usb_tx.tick()


def record_usb_rx(*, nbytes: int, in_waiting_after: int = 0, drain_ms: float = 0.0) -> None:
    if not _ENABLED:
        return
    with usb_rx._lock:
        usb_rx._count += 1
        usb_rx._bytes += nbytes
        if drain_ms > usb_rx._max_ms:
            usb_rx._max_ms = drain_ms
        if in_waiting_after:
            cur = usb_rx._extras.get("peak_in_wait", 0)
            if in_waiting_after > cur:
                usb_rx._extras["peak_in_wait"] = in_waiting_after
    if in_waiting_after > 16384:
        warn(f"USB_RX 积压 in_waiting={in_waiting_after} 本次读={nbytes}B")
    usb_rx.tick()


def record_usb_serial_state(*, in_waiting: int = 0, out_waiting: int = 0, label: str = "") -> None:
    if not _ENABLED:
        return
    with usb_serial._lock:
        usb_serial._count += 1
        if in_waiting:
            cur = usb_serial._extras.get("peak_in", 0)
            if in_waiting > cur:
                usb_serial._extras["peak_in"] = in_waiting
        if out_waiting:
            cur = usb_serial._extras.get("peak_out", 0)
            if out_waiting > cur:
                usb_serial._extras["peak_out"] = out_waiting
        if label:
            usb_serial.bump(label, 1)
    if in_waiting > 32768 or out_waiting > 16384:
        warn(
            f"USB_SERIAL 高水位 in={in_waiting} out={out_waiting} {label}"
        )
    usb_serial.tick()


def record_camera_frame(*, read_ms: float, gap_ms: float) -> None:
    if not _ENABLED:
        return
    with camera._lock:
        camera._count += 1
        camera._sum_ms += read_ms
        if read_ms > camera._max_ms:
            camera._max_ms = read_ms
        if gap_ms > camera._max_gap_ms:
            camera._max_gap_ms = gap_ms
    if gap_ms >= _SLOW_FRAME_MS:
        warn(f"CAMERA 帧间隔过大 gap={gap_ms:.1f}ms read={read_ms:.1f}ms")
    camera.tick()


def record_tracker_infer(*, infer_ms: float, queue_drop: bool = False) -> None:
    if not _ENABLED:
        return
    with tracker._lock:
        tracker._count += 1
        tracker._sum_ms += infer_ms
        if infer_ms > tracker._max_ms:
            tracker._max_ms = infer_ms
        if queue_drop:
            tracker.bump("queue_drop", 1)
    if infer_ms >= 80.0 and tag_enabled("comm_diag") and tag_enabled("COMM_DIAG_TRACKER"):
        log_emit(
            f"TRACKER 推理慢 infer={infer_ms:.1f}ms drop={queue_drop}",
            "COMM_DIAG_TRACKER",
        )
    tracker.tick()


def record_sweep_tick(
    *,
    sent: bool,
    slip: bool = False,
    loop_ms: float = 0.0,
) -> None:
    if not _ENABLED:
        return
    with sweep._lock:
        if sent:
            sweep._count += 1
        else:
            sweep.bump("skip", 1)
        if slip:
            sweep.bump("slip", 1)
        if loop_ms > sweep._max_ms:
            sweep._max_ms = loop_ms
    sweep.tick()


def record_form2_stream_tick(*, sent: bool) -> None:
    if not _ENABLED:
        return
    with form2_stream._lock:
        if sent:
            form2_stream._count += 1
        else:
            form2_stream.bump("skip", 1)
    form2_stream.tick()


def record_widget_usb_send(*, ok: bool, source: str = "") -> None:
    if not _ENABLED:
        return
    with widget_sync._lock:
        if ok:
            widget_sync._count += 1
        else:
            widget_sync.bump("fail", 1)
        if source:
            widget_sync.bump(f"src_{source}", 1)
    widget_sync.tick()


def snapshot_usb_port(ser) -> None:
    """读取串口 in/out_waiting 快照（调用方传入 pyserial.Serial）。"""
    if not _ENABLED or ser is None:
        return
    try:
        record_usb_serial_state(
            in_waiting=ser.in_waiting,
            out_waiting=ser.out_waiting,
            label="snapshot",
        )
    except Exception as exc:
        warn(f"USB_SERIAL 快照失败: {exc}")
