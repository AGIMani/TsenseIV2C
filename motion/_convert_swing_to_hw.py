"""Convert motion_show.txt four-finger swing from old URDF [-0.348,0.348]
to direct hardware range [0, 26°] (same formula as usb.offline remap).
"""
from __future__ import annotations

import json
import math
from pathlib import Path

PATH = Path(__file__).with_name("motion_show.txt")

_SWING_URDF_MIN = -0.348
_SWING_URDF_MAX = 0.348
_SWING_HW_MIN = 0.0
_SWING_HW_MAX = math.radians(26.0)
_SWING_MID_MAX = _SWING_HW_MAX * 0.5
_SWING_MID_MIN = -_SWING_MID_MAX
_SWING_MID_SPAN = _SWING_MID_MAX - _SWING_MID_MIN
_SWING_HW_SPAN = _SWING_HW_MAX - _SWING_HW_MIN
_SWING_URDF_TO_MID_SCALE = _SWING_MID_MAX / _SWING_URDF_MAX

_SWING_KEYS = (
    "left_hand_index_swing_joint",
    "left_hand_mid_swing_joint",
    "left_hand_ring_swing_joint",
    "left_hand_pinky_swing_joint",
)


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def urdf_swing_to_hw(rad: float) -> float:
    x = _clamp(float(rad), _SWING_URDF_MIN, _SWING_URDF_MAX)
    x_mid = x * _SWING_URDF_TO_MID_SCALE
    t = (x_mid - _SWING_MID_MIN) / _SWING_MID_SPAN
    return round(_SWING_HW_MIN + t * _SWING_HW_SPAN, 4)


def main() -> None:
    data = json.loads(PATH.read_text(encoding="utf-8"))
    changed = 0
    samples = []
    for name, fr in data.items():
        if not isinstance(fr, dict):
            continue
        for key in _SWING_KEYS:
            if key not in fr:
                continue
            old = float(fr[key])
            new = urdf_swing_to_hw(old)
            if abs(old - new) > 1e-9:
                fr[key] = new
                changed += 1
                if len(samples) < 12:
                    samples.append((name, key.split("_")[2], old, new))

    PATH.write_text(json.dumps(data, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(f"wrote {PATH}")
    print(f"changed swing fields: {changed}")
    for name, finger, old, new in samples:
        print(f"  {name} {finger}: {old:.4f} -> {new:.4f}")
    # sanity
    init = data.get("init", {})
    print(
        "init swings:",
        {k.split("_")[2]: init.get(k) for k in _SWING_KEYS},
    )


if __name__ == "__main__":
    main()
