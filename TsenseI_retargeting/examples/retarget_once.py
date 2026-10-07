"""Smoke / demo: MediaPipe (21,3) landmarks -> robot joint angles."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE_ROOT.parent))

from TsenseI_retargeting import TopoRetargetConfig, TopoRetargeter  # noqa: E402


def synthetic_open_hand_m(*, palm_width_m: float = 0.08) -> np.ndarray:
    """Build a coarse open-hand MediaPipe landmark cloud in meters."""
    points = np.zeros((21, 3), dtype=float)
    # Wrist at origin; fingers extend along +z, spread along +y.
    points[0] = (0.0, 0.0, 0.0)
    mcp_y = np.linspace(-0.5, 0.5, 4) * palm_width_m
    finger_starts = {
        "thumb": (0.02, -0.55 * palm_width_m, 0.01),
        "index": (0.0, mcp_y[0], 0.02),
        "middle": (0.0, mcp_y[1], 0.022),
        "ring": (0.0, mcp_y[2], 0.02),
        "little": (0.0, mcp_y[3], 0.018),
    }
    chains = {
        "thumb": (1, 2, 3, 4),
        "index": (5, 6, 7, 8),
        "middle": (9, 10, 11, 12),
        "ring": (13, 14, 15, 16),
        "little": (17, 18, 19, 20),
    }
    lengths = {
        "thumb": (0.03, 0.025, 0.02, 0.018),
        "index": (0.0, 0.035, 0.025, 0.02),
        "middle": (0.0, 0.038, 0.027, 0.022),
        "ring": (0.0, 0.035, 0.025, 0.02),
        "little": (0.0, 0.028, 0.02, 0.016),
    }
    for finger, ids in chains.items():
        origin = np.asarray(finger_starts[finger], dtype=float)
        cursor = origin.copy()
        for idx, length in zip(ids, lengths[finger], strict=True):
            if length <= 0.0 and finger != "thumb":
                # MCP sits at the start for non-thumb fingers.
                points[idx] = cursor
                continue
            direction = np.array([0.15 if finger == "thumb" else 0.0, 0.0, 1.0], dtype=float)
            if finger == "thumb":
                direction = np.array([0.35, -0.55, 0.75], dtype=float)
            direction = direction / np.linalg.norm(direction)
            if length <= 0.0:
                points[idx] = cursor
            else:
                cursor = cursor + length * direction
                points[idx] = cursor
    # Ensure palm width IndexMCP-PinkyMCP matches request.
    current = float(np.linalg.norm(points[5] - points[17]))
    if current > 1.0e-9:
        scale = palm_width_m / current
        points *= scale
    return points


def main() -> int:
    landmarks = synthetic_open_hand_m()
    retargeter = TopoRetargeter(
        TopoRetargetConfig(
            use_topo_optimization=True,
            init_max_iters=3,
            topo_max_iters=5,
            use_analytic_jacobian=False,
        )
    )
    result = retargeter.retarget(landmarks)
    print(f"success={result.success}")
    print(f"num_joints={len(result.joint_values)}")
    print(f"loss_total={result.loss.get('total')}")
    for name, value in list(result.joint_values.items())[:5]:
        print(f"  {name}: {value:.4f}")
    if len(result.joint_values) > 5:
        print("  ...")
    assert len(result.joint_values) == len(retargeter.joint_names)
    assert all(np.isfinite(v) for v in result.joint_values.values())
    print("smoke ok")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
