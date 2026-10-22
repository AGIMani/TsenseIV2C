#!/usr/bin/env python3
"""Prepare one USB Quest reverse port without changing an unrelated mapping."""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys


def adb(*args: str) -> str:
    try:
        result = subprocess.run(["adb", *args], capture_output=True, text=True, timeout=15)
    except subprocess.TimeoutExpired as exc:
        raise ValueError("ADB 超时，请检查 USB 数据线和头显调试授权") from exc
    if result.returncode:
        raise ValueError(result.stderr.strip() or result.stdout.strip() or "ADB 命令失败")
    return result.stdout


def prepare(port: int, serial: str = "") -> str:
    if not 1 <= port <= 65535:
        raise ValueError("端口必须在 1..65535")
    if not shutil.which("adb"):
        raise ValueError("未安装 adb；请先运行 sudo apt install adb android-sdk-platform-tools-common")
    devices = []
    for line in adb("devices", "-l").splitlines():
        fields = line.split()
        if len(fields) >= 2 and any(field.startswith("usb:") for field in fields[2:]):
            devices.append((fields[0], fields[1]))
    if serial:
        devices = [device for device in devices if device[0] == serial]
    if not devices:
        raise ValueError("未找到指定的 USB ADB 设备。请开启 Quest 开发者模式、连接 USB 数据线，"
                         "在头显内允许 USB 调试，再用 adb devices -l 检查（需显示 usb:）。")
    if len(devices) > 1:
        raise ValueError("发现多个 USB ADB 设备；请用 HAND_QUEST_ADB_SERIAL=<Quest序列号> 指定头显")
    selected, state = devices[0]
    if state != "device":
        raise ValueError(f"USB 设备 {selected} 状态为 {state}；请在头显内允许 USB 调试后重试")
    endpoint = f"tcp:{port}"
    existing = None
    for line in adb("-s", selected, "reverse", "--list").splitlines():
        fields = line.split()
        if len(fields) == 3 and fields[1] == endpoint:
            existing = fields[2]
            break
    if existing is not None and existing != endpoint:
        raise ValueError(f"头显 {endpoint} 已转发到 {existing}；请使用 HAND_QUEST_ADB_PORT 选择其他端口")
    if existing is None:
        adb("-s", selected, "reverse", "--no-rebind", endpoint, endpoint)
    return selected


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--serial", default=os.environ.get("HAND_QUEST_ADB_SERIAL", ""))
    args = parser.parse_args(argv)
    try:
        serial = prepare(args.port, args.serial)
    except (ValueError, OSError) as exc:
        print(f"[Quest USB ADB] {exc}", file=sys.stderr)
        return 2
    print(f"[Quest USB ADB] {serial}: tcp:{args.port} → Ubuntu tcp:{args.port}", flush=True)
    print(f"电脑工作台：http://localhost:{args.port}/?source=quest_adb", flush=True)
    print(f"Quest Browser：http://localhost:{args.port}/q/adb", flush=True)
    print("先在电脑开始识别，再在头显进入 XR；每只手通过 USB 发送 21 个米制关节点。", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
