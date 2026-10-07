"""Extend motion_show2.txt MCP finger-wave: amp in [0, 0.7], denser smooth frames."""
from __future__ import annotations

import json
import math
from pathlib import Path

PATH = Path(__file__).resolve().parents[1] / "motion" / "motion_show2.txt"

# 可调：根关节（MCP）波浪最大幅度；最小为 0
AMP_MAX = 0.7
FRAMES_PER_CYCLE = 16  # 越大相邻帧越平滑
N_CYCLES = 2
N_RAMP = 4  # 幅度渐入/渐出帧数
TRANSITION_S = 0.3
INTERVAL_S = 0.0
SWING_MID = math.radians(13.0)  # 硬件中位（26° 行程的一半）

FINGERS = ("index", "mid", "ring", "pinky")


def _mcp_wave(amp: float, phi: float) -> dict[str, float]:
    """四指 MCP：amp*[0,1] 包络下的行波，减小相位偏移 → 食→中→无名→小。"""
    out = {}
    for j, name in enumerate(FINGERS):
        # cos∈[-1,1] → MCP∈[0, amp]
        v = amp * 0.5 * (1.0 + math.cos(phi - j * (math.pi / 2.0)))
        out[f"left_hand_{name}_MCP_joint"] = round(max(0.0, min(amp, v)), 4)
        out[f"left_hand_{name}_PIP_joint"] = 0.0
        out[f"left_hand_{name}_DIP_joint"] = 0.0
        out[f"left_hand_{name}_swing_joint"] = SWING_MID
    return out


def _base_pose() -> dict:
    cfg = {
        "_transition_s": TRANSITION_S,
        "_interval_s": INTERVAL_S,
        "left_hand_thumb_rota_joint": 0.0,
        "left_hand_thumb_swing_joint": 0.0,
        "left_hand_thumb_MCP_joint": 0.0,
        "left_hand_thumb_PIP_joint": 0.0,
        "left_hand_pinky_rota_joint": 0.0,
    }
    cfg.update(_mcp_wave(0.0, 0.0))
    # 全 0 MCP 时仍保持四指 swing 中位
    for name in FINGERS:
        cfg[f"left_hand_{name}_swing_joint"] = SWING_MID
        cfg[f"left_hand_{name}_MCP_joint"] = 0.0
        cfg[f"left_hand_{name}_PIP_joint"] = 0.0
        cfg[f"left_hand_{name}_DIP_joint"] = 0.0
    return cfg


def _wave_pose(amp: float, phi: float, *, transition: float | None = None) -> dict:
    cfg = _base_pose()
    if transition is not None:
        cfg["_transition_s"] = transition
    cfg.update(_mcp_wave(amp, phi))
    return cfg


def main() -> None:
    data = json.loads(PATH.read_text(encoding="utf-8"))
    # 保留到 m18_end（含），删除其后旧波浪
    keys = list(data.keys())
    if "m18_end" not in keys:
        raise SystemExit("m18_end not found")
    cut = keys.index("m18_end") + 1
    for k in keys[cut:]:
        data.pop(k, None)

    # 统一 m18_end 的 swing/MCP 作为波浪起点
    data["m18_end"] = _wave_pose(0.0, 0.0, transition=0.5)
    data["m18_end"]["_interval_s"] = 0.1

    poses: list[dict] = []
    # 渐入
    for i in range(N_RAMP):
        amp = AMP_MAX * (i + 1) / N_RAMP
        phi = 2.0 * math.pi * i / FRAMES_PER_CYCLE
        poses.append(_wave_pose(amp, phi))
    # 主循环
    for i in range(N_CYCLES * FRAMES_PER_CYCLE):
        phi = 2.0 * math.pi * (N_RAMP + i) / FRAMES_PER_CYCLE
        poses.append(_wave_pose(AMP_MAX, phi))
    # 渐出
    base = N_RAMP + N_CYCLES * FRAMES_PER_CYCLE
    for i in range(N_RAMP):
        amp = AMP_MAX * (N_RAMP - i) / N_RAMP
        phi = 2.0 * math.pi * (base + i) / FRAMES_PER_CYCLE
        poses.append(_wave_pose(amp, phi))
    # 落回张开
    poses.append(_wave_pose(0.0, 0.0, transition=0.5))

    for i, pose in enumerate(poses, start=19):
        data[f"m{i}"] = pose

    PATH.write_text(
        json.dumps(data, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(
        f"wrote {PATH.name}: wave AMP_MAX={AMP_MAX}, "
        f"frames={len(poses)} (m19..m{18+len(poses)}), "
        f"{FRAMES_PER_CYCLE}/cycle x {N_CYCLES} + ramp {N_RAMP}+{N_RAMP}"
    )
    # sample peak check
    peaks = []
    for name, cfg in list(data.items())[-5:]:
        mcps = [cfg[f"left_hand_{f}_MCP_joint"] for f in FINGERS]
        peaks.append((name, mcps, max(mcps)))
    for row in peaks:
        print(row)


if __name__ == "__main__":
    main()
