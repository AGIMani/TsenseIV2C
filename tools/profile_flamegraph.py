#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""为 GUI 卡顿生成 CPU 火焰图（py-spy）或时间线（VizTracer）。

典型用法（conda 环境 hand_dex）::

    # 1) 启动主程序并采样 40 秒 → 输出 SVG 火焰图
    python tools/profile_flamegraph.py

    # 2) 程序已在跑：先查 PID，再附加采样
    python tools/profile_flamegraph.py --attach <PID> --duration 30

    # 3) 卡顿时更关心主线程（UI 冻结）
    python tools/profile_flamegraph.py --idle --duration 40

    # 4) 需要更细的调用时间线（开销更大）
    python tools/profile_flamegraph.py --backend viztracer --duration 20

输出默认写到 ``profiles/``，用浏览器打开 ``*.svg`` 即可看火焰图：
  - X 轴：样本占比（越宽越耗时）
  - Y 轴：调用栈（越往上越深）
  - 点选函数可放大；搜索框可搜 ``retarget`` / ``paintGL`` / ``WiLoR`` 等
"""

from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_APP = ROOT / "example" / "main.py"
OUT_DIR = ROOT / "profiles"


def _stamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def _ensure_out_dir() -> Path:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    return OUT_DIR


def _find_py_spy() -> str:
    exe = shutil.which("py-spy")
    if exe:
        return exe
    # pip 安装到当前解释器 Scripts
    scripts = Path(sys.executable).resolve().parent
    candidates = [
        scripts / "py-spy.exe",
        scripts / "Scripts" / "py-spy.exe",
        scripts / "py-spy",
    ]
    for c in candidates:
        if c.is_file():
            return str(c)
    raise SystemExit(
        "未找到 py-spy。请先安装:\n"
        f"  {sys.executable} -m pip install py-spy"
    )


def _find_viztracer() -> list[str]:
    try:
        import viztracer  # noqa: F401
    except ImportError as exc:
        raise SystemExit(
            "未找到 viztracer。请先安装:\n"
            f"  {sys.executable} -m pip install viztracer"
        ) from exc
    return [sys.executable, "-m", "viztracer"]


def _print_how_to_read(svg_or_html: Path) -> None:
    print()
    print("=" * 60)
    print(f"输出文件: {svg_or_html}")
    print("如何读火焰图:")
    print("  1. 用 Chrome / Edge 打开 SVG")
    print("  2. 越宽的横条 = 占 CPU 时间越多（卡顿嫌疑最大）")
    print("  3. 从上往下是调用栈：顶层是入口，往下是被调用的函数")
    print("  4. 优先搜索: paintGL / retarget / toporetarget / wilor /")
    print("     mediapipe / processEvents / QTimer / update")
    print("  5. 若最宽条在 Qt 事件循环 / paintGL：多半是主线程渲染过重")
    print("  6. 若最宽条在推理/优化(torch/pinocchio)：考虑降频或挪到后台线程")
    print("=" * 60)


def run_pyspy_launch(args: argparse.Namespace) -> Path:
    py_spy = _find_py_spy()
    out = _ensure_out_dir() / f"flame_{_stamp()}.svg"
    app = Path(args.app).resolve()
    if not app.is_file():
        raise SystemExit(f"找不到入口脚本: {app}")

    cmd = [
        py_spy,
        "record",
        "-o",
        str(out),
        "-d",
        str(args.duration),
        "-r",
        str(args.rate),
        "--format",
        "flamegraph",
        "--subprocesses",
    ]
    if args.idle:
        cmd.append("--idle")
    if args.native:
        cmd.append("--native")
    # Windows 上非管理员附加常需 --nonblocking；launch 模式一般不需要
    cmd += ["--", sys.executable, str(app), *args.app_args]

    print("[profile] 即将启动 GUI 并采样:")
    print(" ", " ".join(cmd))
    print(f"[profile] 采样时长 {args.duration}s — 请在窗口里复现卡顿操作")
    print("[profile] 结束后会写出火焰图 SVG")
    env = os.environ.copy()
    env.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")
    env.setdefault("OMP_NUM_THREADS", "1")
    # py-spy 在 Windows 上有时对退出码敏感；仍尽量跑完
    try:
        subprocess.run(cmd, cwd=str(ROOT), env=env, check=False)
    except KeyboardInterrupt:
        print("\n[profile] 用户中断，若已生成部分结果请检查 profiles/")
    if not out.is_file():
        raise SystemExit(
            f"未生成火焰图: {out}\n"
            "常见原因: 权限不足（管理员运行）、杀毒拦截、或程序秒退。\n"
            "可改用: python tools/profile_flamegraph.py --attach <PID>"
        )
    return out


def run_pyspy_attach(args: argparse.Namespace) -> Path:
    py_spy = _find_py_spy()
    out = _ensure_out_dir() / f"flame_attach_{_stamp()}.svg"
    cmd = [
        py_spy,
        "record",
        "-o",
        str(out),
        "-p",
        str(args.attach),
        "-d",
        str(args.duration),
        "-r",
        str(args.rate),
        "--format",
        "flamegraph",
        "--subprocesses",
    ]
    if args.idle:
        cmd.append("--idle")
    if args.native:
        cmd.append("--native")
    # Windows 附加采样经常需要
    if sys.platform == "win32":
        cmd.append("--nonblocking")

    print("[profile] 附加到已运行进程:")
    print(" ", " ".join(cmd))
    print(f"[profile] 请在接下来 {args.duration}s 内复现卡顿")
    try:
        subprocess.run(cmd, cwd=str(ROOT), check=False)
    except KeyboardInterrupt:
        print("\n[profile] 用户中断")
    if not out.is_file():
        raise SystemExit(
            f"未生成火焰图: {out}\n"
            "Windows 上请尝试「以管理员身份」打开终端再 attach；\n"
            "或确认 PID 是 python.exe（任务管理器 → 详细信息）。"
        )
    return out


def run_viztracer(args: argparse.Namespace) -> Path:
    vt = _find_viztracer()
    out = _ensure_out_dir() / f"viz_{_stamp()}.json"
    app = Path(args.app).resolve()
    if not app.is_file():
        raise SystemExit(f"找不到入口脚本: {app}")

    # VizTracer 本身不会按时长自动停 GUI；用超时杀掉进程后仍保留 trace
    cmd = [
        *vt,
        "-o",
        str(out),
        "--tracer_entries",
        str(args.viz_entries),
        "--max_stack_depth",
        "20",
        "--ignore_c_function",
        str(app),
        *args.app_args,
    ]
    print("[profile] VizTracer 启动 GUI（开销较大，适合短时复现）:")
    print(" ", " ".join(cmd))
    print(f"[profile] {args.duration}s 后自动结束并写 trace")
    env = os.environ.copy()
    env.setdefault("KMP_DUPLICATE_LIB_OK", "TRUE")
    env.setdefault("OMP_NUM_THREADS", "1")
    proc = subprocess.Popen(cmd, cwd=str(ROOT), env=env)
    try:
        proc.wait(timeout=float(args.duration))
    except subprocess.TimeoutExpired:
        print("[profile] 到时，结束进程以落盘 trace…")
        proc.terminate()
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=5)
        # 给 viztracer atexit 一点时间
        time.sleep(1.5)
    if not out.is_file():
        raise SystemExit(f"未生成 VizTracer 文件: {out}")

    # 生成可离线打开的 HTML（若可用）
    html = out.with_suffix(".html")
    try:
        subprocess.run(
            [sys.executable, "-m", "viztracer", "--combine", str(out), "-o", str(html)],
            cwd=str(ROOT),
            check=False,
        )
    except Exception:
        pass
    if html.is_file():
        print(f"[profile] 也可用: python -m vizviewer {out}")
        return html
    print(f"[profile] 查看: python -m vizviewer {out}")
    return out


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="为达显身手 GUI 生成火焰图 / 性能时间线",
    )
    p.add_argument(
        "--backend",
        choices=("pyspy", "viztracer"),
        default="pyspy",
        help="pyspy=火焰图SVG（推荐查卡顿）；viztracer=时间线（更细、开销更大）",
    )
    p.add_argument(
        "--app",
        default=str(DEFAULT_APP),
        help=f"入口脚本（默认 {DEFAULT_APP}）",
    )
    p.add_argument(
        "--attach",
        type=int,
        default=None,
        help="附加到已运行的 PID（仅 pyspy）",
    )
    p.add_argument(
        "--duration",
        type=int,
        default=40,
        help="采样秒数（默认 40）",
    )
    p.add_argument(
        "--rate",
        type=int,
        default=100,
        help="py-spy 采样频率 Hz（默认 100）",
    )
    p.add_argument(
        "--idle",
        action="store_true",
        help="py-spy 包含空闲样本（看主线程堵在哪时有用）",
    )
    p.add_argument(
        "--native",
        action="store_true",
        help="py-spy 采样原生栈（C++/torch 等，Windows 上可能不稳定）",
    )
    p.add_argument(
        "--viz-entries",
        type=int,
        default=3_000_000,
        help="VizTracer 环形缓冲条目数",
    )
    p.add_argument(
        "--open",
        action="store_true",
        help="结束后尝试用系统默认程序打开结果",
    )
    p.add_argument(
        "app_args",
        nargs="*",
        help="传给入口脚本的额外参数",
    )
    return p


def main() -> None:
    args = build_parser().parse_args()
    os.chdir(ROOT)

    if args.backend == "viztracer":
        if args.attach is not None:
            raise SystemExit("viztracer 模式不支持 --attach，请用 pyspy")
        result = run_viztracer(args)
    elif args.attach is not None:
        result = run_pyspy_attach(args)
    else:
        result = run_pyspy_launch(args)

    _print_how_to_read(result)

    if args.open:
        try:
            if sys.platform == "win32":
                os.startfile(str(result))  # type: ignore[attr-defined]
            elif sys.platform == "darwin":
                subprocess.run(["open", str(result)], check=False)
            else:
                subprocess.run(["xdg-open", str(result)], check=False)
        except Exception as exc:  # noqa: BLE001
            print(f"[profile] 自动打开失败: {exc}")


if __name__ == "__main__":
    main()
