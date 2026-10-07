"""Shift four-finger swing URDF origin so joint=13° matches old geometric 0,
and set limits to [0, 26°].
"""
from __future__ import annotations

import math
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
URDF_DIR = ROOT / "assets" / "hands" / "TsenseI_hand" / "urdf"
MID = math.radians(13.0)
LO = 0.0
HI = math.radians(26.0)

JOINT_RE = re.compile(
    r'(<joint\s+name="(?P<name>(?:left|right)_hand_(?:index|mid|ring|pinky)_swing_joint)"'
    r'[\s\S]*?</joint>)'
)


def _patch_block(block: str, name: str) -> str:
    axis_m = re.search(r'<axis\s+xyz="([^"]+)"', block)
    if not axis_m:
        raise RuntimeError(f"no axis in {name}")
    ax = [float(x) for x in axis_m.group(1).split()]
    # +X → roll_new = roll_old - mid; -X → roll_new = roll_old + mid
    if abs(ax[0] - 1.0) < 1e-6 and abs(ax[1]) < 1e-6 and abs(ax[2]) < 1e-6:
        roll_delta = -MID
    elif abs(ax[0] + 1.0) < 1e-6 and abs(ax[1]) < 1e-6 and abs(ax[2]) < 1e-6:
        roll_delta = +MID
    else:
        raise RuntimeError(f"unexpected axis {ax} for {name}")

    def repl_origin(m: re.Match) -> str:
        xyz = m.group(1)
        r, p, y = (float(v) for v in m.group(2).split())
        r2 = r + roll_delta
        return (
            f'<origin\n'
            f'      xyz="{xyz}"\n'
            f'      rpy="{r2} {p} {y}" />'
        )

    block2, n1 = re.subn(
        r'<origin\s*\n\s*xyz="([^"]+)"\s*\n\s*rpy="([^"]+)"\s*/>',
        repl_origin,
        block,
        count=1,
    )
    if n1 != 1:
        raise RuntimeError(f"origin replace failed for {name}")

    def repl_limit(m: re.Match) -> str:
        return (
            f'<limit\n'
            f'      lower="{LO}"\n'
            f'      upper="{HI}"\n'
            f'      effort="{m.group(1)}"\n'
            f'      velocity="{m.group(2)}" />'
        )

    block3, n2 = re.subn(
        r'<limit\s*\n\s*lower="[^"]+"\s*\n\s*upper="[^"]+"\s*\n'
        r'\s*effort="([^"]+)"\s*\n\s*velocity="([^"]+)"\s*/>',
        repl_limit,
        block2,
        count=1,
    )
    if n2 != 1:
        raise RuntimeError(f"limit replace failed for {name}")
    return block3


def patch_file(path: Path) -> int:
    text = path.read_text(encoding="utf-8")
    count = 0

    def repl(m: re.Match) -> str:
        nonlocal count
        count += 1
        return _patch_block(m.group(1), m.group("name"))

    new_text, n = JOINT_RE.subn(repl, text)
    if n != 4:
        raise RuntimeError(f"{path.name}: expected 4 swing joints, got {n}")
    path.write_text(new_text, encoding="utf-8")
    return count


def main() -> None:
    for name in ("TsenseI__hand_left_v1.urdf", "TsenseI__hand_right_v1.urdf"):
        path = URDF_DIR / name
        n = patch_file(path)
        print(f"{path.name}: patched {n} swing joints "
              f"(mid={MID:.6f}, limits=[{LO}, {HI:.6f}])")


if __name__ == "__main__":
    main()
