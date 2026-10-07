"""Convert motion_show.txt to right hand: set _hand_side=right.

右手 HW 约定（与 protocol.convert_ui_to_hardware 一致）：
  · thumb_rota：与左手同号（不再取负）
  · thumb_swing：相对左手动作文件值取负（动作文件保留旧[-1,1]约定）
  · pinky_rota：相对左手 HW 取负
"""
from __future__ import annotations

import json
from pathlib import Path

PATH = Path(__file__).with_name("motion_show.txt")
# 左手 HW → 右手 HW 需变号的关节
_NEGATE_ON_RIGHT = (
    "left_hand_thumb_swing_joint",
    "left_hand_pinky_rota_joint",
)


def main() -> None:
    data = json.loads(PATH.read_text(encoding="utf-8"))
    motions = {
        k: v
        for k, v in data.items()
        if k not in ("_hand_side", "hand_side", "_meta") and isinstance(v, dict)
    }
    changed = 0
    samples = []
    for name, fr in motions.items():
        for key in _NEGATE_ON_RIGHT:
            if key not in fr:
                continue
            old = float(fr[key])
            new = round(-old, 4)
            if abs(old - new) > 1e-12:
                fr[key] = new
                changed += 1
                if len(samples) < 8:
                    samples.append((name, key, old, new))

    out = {"_hand_side": "right"}
    out.update(motions)
    PATH.write_text(json.dumps(out, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"wrote {PATH}")
    print(f"_hand_side=right, negated fields={changed} (thumb_swing/pinky_rota; thumb_rota unchanged)")
    for name, key, old, new in samples:
        print(f"  {name} {key}: {old} -> {new}")


if __name__ == "__main__":
    main()
