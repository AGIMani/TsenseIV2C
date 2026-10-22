#!/usr/bin/env python3
"""启动 a3_web_bridge 并打开浏览器。未编译则提示先 cmake 构建。"""
from __future__ import annotations

import subprocess
import sys
import time
import webbrowser
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
NAMES = (
    "a3_web_bridge.exe",
    "a3_web_bridge",
)


def _skip(p: Path) -> bool:
    parts = {x.lower() for x in p.parts}
    return "cmakefiles" in parts or ".vs" in parts


def find_bridge() -> Path | None:
    preferred = (
        ROOT / "build" / "Release" / "a3_web_bridge.exe",
        ROOT / "build" / "Debug" / "a3_web_bridge.exe",
        ROOT / "build" / "RelWithDebInfo" / "a3_web_bridge.exe",
        ROOT / "build" / "a3_web_bridge.exe",
        ROOT / "build" / "a3_web_bridge",
        ROOT / "build" / "bin" / "a3_web_bridge.exe",
        ROOT / "build" / "bin" / "a3_web_bridge",
        ROOT / "out" / "build" / "x64-Release" / "a3_web_bridge.exe",
        ROOT / "out" / "build" / "x64-Debug" / "a3_web_bridge.exe",
        ROOT / "out" / "build" / "x64-Release" / "a3_web_bridge",
        ROOT / "out" / "build" / "x64-Debug" / "a3_web_bridge",
    )
    for p in preferred:
        if p.is_file():
            return p
    found: list[Path] = []
    for name in NAMES:
        found.extend(p for p in ROOT.rglob(name) if p.is_file() and not _skip(p))
    if not found:
        return None
    found.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    return found[0]


def main() -> int:
    exe = find_bridge()
    if exe is None:
        print("未找到 a3_web_bridge。请先构建：")
        print("  cmake --preset vs2026-x64")
        print("  cmake --build --preset vs2026-x64-release")
        print("或 Visual Studio 打开本文件夹后生成。")
        return 1
    port = "8080"
    args = [str(exe), "--port", port, "--webroot", str(ROOT / "webui" / "static")]
    args += sys.argv[1:]
    print("启动", " ".join(args))
    proc = subprocess.Popen(args, cwd=str(exe.parent))
    time.sleep(0.4)
    webbrowser.open(f"http://127.0.0.1:{port}/")
    return proc.wait()


if __name__ == "__main__":
    raise SystemExit(main())
