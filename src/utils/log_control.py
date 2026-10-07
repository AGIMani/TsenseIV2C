"""应用日志标签注册与动态开关（供 ui_form4 日志面板使用）。

emit() 默认：写文件；是否打控制台由 tag 的 file_only 决定。
stdlib logging：StreamHandler 跳过 file_only；FileHandler 仍写入（tag 启用时）。
"""

from __future__ import annotations

import atexit
import logging
import os
import threading
from datetime import datetime
from typing import Dict, Iterable, Optional, TextIO

# tag -> 界面描述
LOG_TAGS: Dict[str, str] = {
    "Latency": "MediaPipe→H7 端到端耗时（控制台主看）",
    "Boot": "启动 / 模块加载 / 预加载（默认仅文件）",
    "SkeletonTracker": "骨架追踪与检测后端",
    "URDFViewer": "URDF 3D 渲染 (OpenGL/VBO)",
    "URDFDisplay": "URDF 显示封装层",
    "URDFLoader": "URDF 文件加载",
    "Collision": "MuJoCo 碰撞投影（Form2 滑块阻挡/轨迹）",
    "MuJoCoPhysics": "MuJoCo 物理滤波与接触日志",
    "Widget": "主窗口 widget",
    "Form2Window": "Form2 关节控制",
    "MotionStream": "Form2 动作序列播放 (wave 等)",
    "Form3Window": "Form3 调试面板",
    "Form5Window": "Form5 遥测回传",
    "TsenseIRetargeting": "TsenseI 手势重定向",
    "PipelineLog": "手势→重定向→碰撞 全链路调试",
    "RetargetDiag": "Retarget 精简诊断",
    "RetargetAngleCmp": "MediaPipe角度 vs 模型角度",
    "FingerLinkage": "闭环连杆角度↔行程逆解",
    "comm_diag": "通信诊断 (USB/下发)",
    "COMM_DIAG_CAMERA": "通信诊断-相机统计",
    "COMM_DIAG_TRACKER": "通信诊断-追踪统计",
    "COMM_DIAG_USB_TX": "通信诊断-USB下发统计",
    "COMM_DIAG_USB_RX": "通信诊断-USB上行统计",
    "COMM_DIAG_USB_SERIAL": "通信诊断-串口缓冲统计",
    "COMM_DIAG_SWEEP": "通信诊断-扫动统计",
    "COMM_DIAG_FORM2_STREAM": "通信诊断-Form2动作流统计",
    "COMM_DIAG_WIDGET_SYNC": "通信诊断-Widget同步统计",
    "UsbController": "USB 下位机",
    "Camera": "相机采集",
    "HandSkeleton": "手部骨架显示",
    "HandOccNet": "HandOccNet 推理后端",
    "WiLoR": "WiLoR 推理后端",
    "URDFGLWidget": "URDF OpenGL(moderngl) 渲染",
    "RetargetRuntime": "重定向运行时桥接",
    "EgoHandICL": "EgoHandICL 后端",
    "form2_window": "Form2 (logger 名)",
    "widget": "主窗口 (logger 名)",
}

# 默认仅写文件、不刷控制台的标签（仍可用 Form4 关掉写入；file_only 可改）
_FILE_ONLY_DEFAULT: Dict[str, bool] = {
    "Boot": True,
    "URDFViewer": True,
    "URDFDisplay": True,
    "URDFLoader": True,
    "URDFGLWidget": True,
    "HandSkeleton": True,
    "Collision": True,
    "FingerLinkage": True,
    "RetargetDiag": True,
    "RetargetAngleCmp": True,
    "TsenseIRetargeting": True,
    "RetargetRuntime": True,
    "PipelineLog": True,
    "Camera": True,
    "MuJoCoPhysics": True,
    "SkeletonTracker": True,
    "HandOccNet": True,
    "WiLoR": True,
    "EgoHandICL": True,
    "COMM_DIAG_CAMERA": True,
    "COMM_DIAG_TRACKER": True,
    "COMM_DIAG_USB_TX": True,
    "COMM_DIAG_USB_RX": True,
    "COMM_DIAG_USB_SERIAL": True,
    "COMM_DIAG_SWEEP": True,
    "COMM_DIAG_FORM2_STREAM": True,
    "COMM_DIAG_WIDGET_SYNC": True,
}

# stdlib logging.Logger.name -> tag
_LOGGER_TO_TAG: Dict[str, str] = {
    "widget": "Widget",
    "form2_window": "Form2Window",
    "form3_window": "Form3Window",
    "form5_window": "Form5Window",
    "comm_diag": "comm_diag",
    "usb_offline": "UsbController",
    "camera": "Camera",
}

_lock = threading.Lock()
_enabled: Dict[str, bool] = {tag: True for tag in LOG_TAGS}
_file_only: Dict[str, bool] = {
    tag: bool(_FILE_ONLY_DEFAULT.get(tag, False)) for tag in LOG_TAGS
}
# Latency 必须打到控制台
_file_only["Latency"] = False

# 默认关闭全链路调试日志，避免运行时刷屏；可在 Form4 手动开启。
# 环境变量 PIPELINE_LOG=1 时自动打开 PipelineLog（便于终端调试 retarget）。
_enabled["PipelineLog"] = os.environ.get("PIPELINE_LOG", "0").strip().lower() in (
    "1",
    "true",
    "yes",
    "on",
)
_enabled["COMM_DIAG_TRACKER"] = False
_enabled["COMM_DIAG_CAMERA"] = False
# 通信诊断总开关：默认关；COMM_DIAG=1 时开启。
_enabled["comm_diag"] = os.environ.get("COMM_DIAG", "0").strip().lower() in (
    "1",
    "true",
    "yes",
    "on",
)
_enabled["COMM_DIAG_USB_TX"] = False
_enabled["COMM_DIAG_USB_RX"] = False
_enabled["COMM_DIAG_USB_SERIAL"] = False
_enabled["COMM_DIAG_SWEEP"] = False
_enabled["COMM_DIAG_FORM2_STREAM"] = False
_enabled["COMM_DIAG_WIDGET_SYNC"] = False
# MuJoCo 接触/修正日志默认关（刷屏）；MUJOCO_CONTACT_LOG=1 或 Form4 可开。
_enabled["MuJoCoPhysics"] = os.environ.get("MUJOCO_CONTACT_LOG", "0").strip().lower() in (
    "1",
    "true",
    "yes",
    "on",
)
_filter_installed = False

# emit() 落盘：与主程序 FileHandler 共用同一路径（追加写入）
_log_file_path: Optional[str] = None
_log_file_fp: Optional[TextIO] = None
_atexit_registered = False


def _close_log_file() -> None:
    global _log_file_fp
    with _lock:
        fp = _log_file_fp
        _log_file_fp = None
        if fp is not None:
            try:
                fp.flush()
                fp.close()
            except Exception:
                pass


def configure_log_file(path: str) -> str:
    """设置 emit() 落盘路径；返回实际路径。可重复调用以切换文件。"""
    global _log_file_path, _log_file_fp, _atexit_registered
    path = str(path)
    with _lock:
        if _log_file_fp is not None:
            try:
                _log_file_fp.flush()
                _log_file_fp.close()
            except Exception:
                pass
            _log_file_fp = None
        _log_file_path = path
        _log_file_fp = open(path, "a", encoding="utf-8", buffering=1)
        if not _atexit_registered:
            atexit.register(_close_log_file)
            _atexit_registered = True
    return path


def get_log_file_path() -> Optional[str]:
    with _lock:
        return _log_file_path


def _write_log_file(line: str) -> None:
    """将一行日志追加写入文件（调用方已持有或不依赖 _lock 外的一致性）。"""
    fp = _log_file_fp
    if fp is None:
        return
    try:
        fp.write(line if line.endswith("\n") else line + "\n")
        fp.flush()
    except Exception:
        pass


def register_tag(tag: str, description: str, *, enabled: bool = True, file_only: bool = False) -> None:
    with _lock:
        LOG_TAGS[tag] = description
        if tag not in _enabled:
            _enabled[tag] = enabled
        if tag not in _file_only:
            _file_only[tag] = bool(file_only)


def is_enabled(tag: str) -> bool:
    with _lock:
        return _enabled.get(tag, True)


def is_file_only(tag: str) -> bool:
    with _lock:
        return bool(_file_only.get(tag, False))


def set_enabled(tag: str, enabled: bool) -> None:
    with _lock:
        if tag not in LOG_TAGS:
            LOG_TAGS[tag] = tag
        _enabled[tag] = bool(enabled)


def set_file_only(tag: str, file_only: bool) -> None:
    with _lock:
        if tag not in LOG_TAGS:
            LOG_TAGS[tag] = tag
        _file_only[tag] = bool(file_only)


def all_tags() -> Dict[str, str]:
    with _lock:
        return dict(LOG_TAGS)


def enabled_tags() -> Dict[str, bool]:
    with _lock:
        return dict(_enabled)


def file_only_tags() -> Dict[str, bool]:
    with _lock:
        return dict(_file_only)


def set_all_enabled(enabled: bool) -> None:
    with _lock:
        for tag in LOG_TAGS:
            _enabled[tag] = bool(enabled)


def emit(msg: str, tag: str = "App", *, to_console: bool | None = None) -> None:
    """统一日志：始终写文件（tag 启用时）；控制台受 file_only / to_console 控制。"""
    if not is_enabled(tag):
        return
    now = datetime.now().strftime("%H:%M:%S.%f")[:-3]
    tname = threading.current_thread().name
    line = f"{now} [{tname}] [{tag}] {msg}"
    console = (not is_file_only(tag)) if to_console is None else bool(to_console)
    if console:
        print(line, flush=True)
    with _lock:
        _write_log_file(line)


def _resolve_tag(record: logging.LogRecord) -> str:
    tag = _LOGGER_TO_TAG.get(record.name, record.name)
    if tag in LOG_TAGS:
        return tag
    if record.name in LOG_TAGS:
        return record.name
    return tag


class TagLogFilter(logging.Filter):
    """过滤 stdlib logging：按 logger 名映射到 tag（启用与否）。"""

    def filter(self, record: logging.LogRecord) -> bool:
        tag = _resolve_tag(record)
        if tag in LOG_TAGS or record.name in LOG_TAGS:
            return is_enabled(tag if tag in LOG_TAGS else record.name)
        return True


class ConsoleTagFilter(logging.Filter):
    """控制台：tag 关闭或 file_only 时丢弃。"""

    def filter(self, record: logging.LogRecord) -> bool:
        tag = _resolve_tag(record)
        if tag not in LOG_TAGS and record.name not in LOG_TAGS:
            return True
        key = tag if tag in LOG_TAGS else record.name
        if not is_enabled(key):
            return False
        if is_file_only(key):
            return False
        return True


class FileTagFilter(logging.Filter):
    """文件：仅按 enabled 过滤。"""

    def filter(self, record: logging.LogRecord) -> bool:
        tag = _resolve_tag(record)
        if tag in LOG_TAGS or record.name in LOG_TAGS:
            return is_enabled(tag if tag in LOG_TAGS else record.name)
        return True


def install_logging_filter(root: Optional[logging.Logger] = None) -> None:
    global _filter_installed
    if _filter_installed:
        return
    logger = root or logging.getLogger()
    console_flt = ConsoleTagFilter()
    file_flt = FileTagFilter()
    for h in logger.handlers:
        if isinstance(h, logging.StreamHandler) and not isinstance(h, logging.FileHandler):
            h.addFilter(console_flt)
        else:
            h.addFilter(file_flt)
    _filter_installed = True


def attach_handler_filters(handlers: Iterable[logging.Handler]) -> None:
    console_flt = ConsoleTagFilter()
    file_flt = FileTagFilter()
    for h in handlers:
        if isinstance(h, logging.StreamHandler) and not isinstance(h, logging.FileHandler):
            h.addFilter(console_flt)
        else:
            h.addFilter(file_flt)
