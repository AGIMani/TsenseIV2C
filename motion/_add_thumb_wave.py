"""Add thumb into the wave section of motion_show.txt (from 01_W1_t0.03).

拇指相位领先食指 0.25s；起始用缓入包络把振幅从 0 慢慢拉满，避免首帧突跳。
"""
from __future__ import annotations

import json
import math
import re
from pathlib import Path

PATH = Path(__file__).with_name("motion_show.txt")

AMP = 0.65
PIP_RATIO = 0.738  # same ratio as four-finger PIP/MCP
# Lead index by 0.25s: thumb -> index -> mid -> ring -> pinky
THUMB_PHASE = -0.25
# 前 AMP_RAMP_S 秒余弦缓入：起始接近 0，再逐渐加大到满幅
AMP_RAMP_S = 1.25
START_KEY = "01_W1_t0.03"


def wave_mcp(t: float, phase: float) -> float:
    tau = t - phase
    if tau < 0:
        return 0.0
    return AMP * (math.sin(math.pi * (tau % 1.0)) ** 2)


def amp_envelope(t: float) -> float:
    """0→1 余弦缓入，避免首帧拇指突然抽动。"""
    if AMP_RAMP_S <= 1e-9:
        return 1.0
    if t <= 0.0:
        return 0.0
    if t >= AMP_RAMP_S:
        return 1.0
    # 0.5 - 0.5*cos(pi * u)：起点斜率 0，终点斜率 0
    u = t / AMP_RAMP_S
    return 0.5 - 0.5 * math.cos(math.pi * u)


def main() -> None:
    text = PATH.read_text(encoding="utf-8")
    data = json.loads(text)
    keys = list(data.keys())
    if START_KEY not in keys:
        raise SystemExit(f"missing key {START_KEY}")

    updated = 0
    samples: list[tuple[str, float, float, float]] = []
    for k in keys[keys.index(START_KEY) :]:
        m = re.search(r"_t([\d.]+)$", k)
        if not m:
            continue
        t = float(m.group(1))
        env = amp_envelope(t)
        mcp = round(wave_mcp(t, THUMB_PHASE) * env, 4)
        pip = round(mcp * PIP_RATIO, 4)
        fr = data[k]
        fr["left_hand_thumb_MCP_joint"] = mcp
        fr["left_hand_thumb_PIP_joint"] = pip
        updated += 1
        if updated <= 8 or abs(t - 0.5) < 0.02 or abs(t - 1.25) < 0.02 or abs(t - 1.5) < 0.02:
            samples.append((k, t, env, mcp))

    out = json.dumps(data, indent=2, ensure_ascii=False) + "\n"
    PATH.write_text(out, encoding="utf-8")

    print(f"wrote {PATH}")
    print(f"updated={updated}  AMP_RAMP_S={AMP_RAMP_S}s  phase={THUMB_PHASE}")
    for k, t, env, mcp in samples:
        print(f"  {k}: t={t:.2f} env={env:.3f} thumb_MCP={mcp:.4f}")


if __name__ == "__main__":
    main()
