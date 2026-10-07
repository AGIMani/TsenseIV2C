"""Batch-convert four-finger swing: old HW [0, 0.523] (~30°) → [0, 26°].

Preserves relative pose within the old range:
  new = old / 0.523 * radians(26)
  e.g. mid 0.2615 → ~0.2269 (13°)
"""
from __future__ import annotations

import json
import math
from pathlib import Path

MOTION_DIR = Path(__file__).resolve().parent
_SWING_KEYS = (
    "left_hand_index_swing_joint",
    "left_hand_mid_swing_joint",
    "left_hand_ring_swing_joint",
    "left_hand_pinky_swing_joint",
)
_OLD_HW_MAX = 0.523
_NEW_HW_MAX = math.radians(26.0)  # ≈ 0.4538
_SCALE = _NEW_HW_MAX / _OLD_HW_MAX


def _clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def old_hw_to_new(rad: float) -> float:
    return round(_clamp(float(rad) * _SCALE, 0.0, _NEW_HW_MAX), 4)


def convert_file(path: Path) -> tuple[int, list[tuple]]:
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError(f"root is not object: {path}")

    changed = 0
    samples: list[tuple] = []
    for name, fr in data.items():
        if not isinstance(fr, dict):
            continue
        for key in _SWING_KEYS:
            if key not in fr:
                continue
            try:
                old = float(fr[key])
            except (TypeError, ValueError):
                continue
            new = old_hw_to_new(old)
            if abs(old - new) > 1e-9:
                fr[key] = new
                changed += 1
                if len(samples) < 6:
                    samples.append((name, key.split("_")[2], old, new))

    if changed:
        path.write_text(
            json.dumps(data, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )
    return changed, samples


def main() -> None:
    paths = sorted(
        p
        for p in MOTION_DIR.iterdir()
        if p.is_file() and p.suffix.lower() in (".txt", ".json")
        and not p.name.startswith("_")
    )
    print(
        f"scale={_SCALE:.6f}  old_max={_OLD_HW_MAX}  "
        f"new_max={_NEW_HW_MAX:.6f} rad ({math.degrees(_NEW_HW_MAX):.1f}°)"
    )
    total = 0
    for path in paths:
        changed, samples = convert_file(path)
        total += changed
        print(f"{path.name}: changed {changed}")
        for name, finger, old, new in samples:
            print(
                f"  {name} {finger}: {old:.4f} -> {new:.4f} "
                f"({math.degrees(old):.2f}° -> {math.degrees(new):.2f}°)"
            )
    print(f"total swing fields updated: {total}")


if __name__ == "__main__":
    main()
