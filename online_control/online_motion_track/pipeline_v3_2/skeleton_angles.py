"""Observable geometric hand angles; these are NOT URDF joint coordinates."""
from __future__ import annotations

import numpy as np

FINGER_STARTS = {"index": 5, "middle": 9, "ring": 13, "pinky": 17}
ANGLE_KEYS = tuple(f"{finger}_{part}" for finger in FINGER_STARTS
                   for part in ("MCP_bend", "PIP", "DIP", "swing")) + (
    "thumb_CMC_bend_proxy", "thumb_MCP", "thumb_IP", "thumb_CMC_spread_proxy")
ANGLE_DEFINITIONS = {
    "bend": "Angle between consecutive directed bones; straight=0, range [0, pi].",
    "four_finger_MCP_bend": "wrist->MCP versus MCP->PIP; a geometric proxy combining flexion and spread.",
    "PIP_DIP_thumb_MCP_IP": "Three-point bone bend at the named landmark.",
    "swing": "Projected MCP->PIP azimuth minus wrist->MCP azimuth in palm plane; wrapped [-pi, pi].",
    "palm_basis": "Y=wrist->middle MCP; X=pinky MCP->index MCP orthogonalized against Y; positive azimuth towards X.",
    "thumb_CMC_bend_proxy": "wrist->CMC versus CMC->MCP; not independently observable CMC axial rotation.",
    "thumb_CMC_spread_proxy": "CMC->MCP azimuth in palm plane relative to Y; not robot thumb rota/swing.",
    "invalid": "Unobservable/degenerate angles are null, never an invented zero.",
}


def _unit(vector):
    length = float(np.linalg.norm(vector))
    return None if length < 1e-8 else vector / length


def _bend(points, a, b, c):
    first, second = _unit(points[b] - points[a]), _unit(points[c] - points[b])
    if first is None or second is None:
        return None
    return float(np.arctan2(np.linalg.norm(np.cross(first, second)), np.dot(first, second)))


def skeleton_joint_angles(points):
    """Return 20 named angles in radians, invariant to proper rotations/scale.

    Left/right use the same anatomical landmark basis, so reflected copies have
    equal signed spread. A scalar bend is unsigned and is not a flexion-axis IK.
    """
    if points is None:
        return None
    arr = np.asarray(points, dtype=float)
    if arr.shape != (21, 3) or not np.isfinite(arr).all():
        return None
    y = _unit(arr[9] - arr[0])
    across = arr[5] - arr[17]
    x = None if y is None else _unit(across - np.dot(across, y) * y)

    def azimuth(vector):
        if x is None or y is None:
            return None
        vx, vy = float(np.dot(vector, x)), float(np.dot(vector, y))
        return None if np.hypot(vx, vy) < 1e-8 else float(np.arctan2(vx, vy))

    result = {}
    for name, start in FINGER_STARTS.items():
        result[f"{name}_MCP_bend"] = _bend(arr, 0, start, start + 1)
        result[f"{name}_PIP"] = _bend(arr, start, start + 1, start + 2)
        result[f"{name}_DIP"] = _bend(arr, start + 1, start + 2, start + 3)
        base, bone = azimuth(arr[start] - arr[0]), azimuth(arr[start + 1] - arr[start])
        delta = None if base is None or bone is None else bone - base
        result[f"{name}_swing"] = None if delta is None else float(np.arctan2(np.sin(delta), np.cos(delta)))
    result["thumb_CMC_bend_proxy"] = _bend(arr, 0, 1, 2)
    result["thumb_MCP"] = _bend(arr, 1, 2, 3)
    result["thumb_IP"] = _bend(arr, 2, 3, 4)
    result["thumb_CMC_spread_proxy"] = azimuth(arr[2] - arr[1])
    return result


def degrees(angles):
    return None if angles is None else {
        key: None if value is None else float(np.degrees(value)) for key, value in angles.items()
    }
