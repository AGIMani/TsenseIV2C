"""A3.2 application coordinates; the source mechanism keeps its CAD zero."""
from __future__ import annotations

COORDINATE_VERSION = "a32-positive-swing-v1"
SWING_AXIS_ALIGNMENT = "id-direction-v1"
THUMB_ZERO_ALIGNMENT = "mechanical-stop-v1"
# Bare CAD first contact between the rota-block stop and rotaback bracket.
# This corrects the URDF origin only: hardware/saved angles already use the
# mechanical zero, so the offset must never be added to motor commands.
THUMB_ROTABACK_ORIGIN_OFFSET = -0.25042009379455915
SWING_CENTER = .350
# Preserve the translated model endpoints and zero default. The hardware
# command is clipped separately, after the coordinate conversion.
SWING_LIMITS = (0., .7)
SWING_COMMAND_LIMITS = (.005, .695)
FLEXION_LIMITS = (.005, 1.565)
MCP_LIMITS = (.005, 1.3)


def thumb_rotaback_origin_offset(model_id: str, name: str) -> float:
    if model_id == "TsenseI_v3_2" and name.endswith("_thumb_rotaback_joint2"):
        return THUMB_ROTABACK_ORIGIN_OFFSET
    return 0.


def is_finger_swing(name: str) -> bool:
    return any(name.lower().endswith(f"_{finger}_swing_joint")
               for finger in ("index", "mid", "ring", "pinky"))


def requested_limits(name: str):
    if is_finger_swing(name):
        return SWING_LIMITS
    if name.endswith(("_PIP_joint", "_DIP_joint")):
        return FLEXION_LIMITS
    if name.endswith("_MCP_joint"):
        return MCP_LIMITS
    return None


def reverse_swing(name: str, angle: float) -> float:
    """Reflect swing around the positive-coordinate offset before source signs."""
    return SWING_CENTER - angle if is_finger_swing(name) else angle


def legacy_to_model(model_id: str, name: str, angle: float) -> float:
    """Saved signed swing -> current coordinate; retain mechanical motor zero."""
    if not is_finger_swing(name):
        return angle
    return SWING_CENTER + angle if model_id == "TsenseI_v3_2" else SWING_CENTER - angle


def model_to_legacy(model_id: str, name: str, angle: float) -> float:
    """Undo saved-pose swing conversion, excluding fixed CAD origin correction."""
    if not is_finger_swing(name):
        return angle
    return angle - SWING_CENTER if model_id == "TsenseI_v3_2" else SWING_CENTER - angle


def initial_pose(limits):
    return {name: min(hi, max(lo, 0.))
            for name, (lo, hi) in limits.items()}


def migrate_legacy_pose(pose, limits, *, model_id):
    """Convert unversioned saved poses once, checking the old coordinate domain."""
    import math
    from numbers import Real
    if not isinstance(pose, dict) or pose.keys() != limits.keys():
        raise ValueError("A3.2 旧动作必须包含完整 20 个关节")
    result = {}
    for name, value in pose.items():
        old_bounds = (-.35, .35) if is_finger_swing(name) else (0., 1.57) if requested_limits(name) else limits[name]
        if (not isinstance(value, Real) or isinstance(value, bool) or not math.isfinite(value)
                or not old_bounds[0] - 1e-8 <= value <= old_bounds[1] + 1e-8):
            raise ValueError(f"A3.2 旧动作关节超出原坐标范围：{name}")
        lo, hi = limits[name]
        result[name] = min(hi, max(lo, legacy_to_model(model_id, name, value)))
    return result
