"""A3.2 application coordinates; the source mechanism keeps its CAD zero."""
from __future__ import annotations

COORDINATE_VERSION = "a32-signed-swing-v2"
POSITIVE_COORDINATE_VERSION = "a32-positive-swing-v1"
POSITIVE_SWING_AXIS_ALIGNMENT = "id-direction-v1"
SWING_AXIS_ALIGNMENT = "id-cad-direction-v2"
THUMB_ZERO_ALIGNMENT = "reference-cad-v2"
LEGACY_THUMB_ZERO_ALIGNMENT = "mechanical-stop-v1"
LEGACY_THUMB_ROTABACK_ORIGIN_OFFSET = -0.25042009379455915
THUMB_ZERO_REFERENCE = "A3.2 URDF/urdf/A3.1V6 26.8.4-2.urdf"
THUMB_ZERO_REFERENCE_SHA256 = "84fc48faf5d89b97a8766be439fca469f9ab47c3e4bb5faf4cdde4583d3d23f3"
# Measured with tools/measure_a32_thumb_zero.py. These are fixed URDF-origin
# corrections relative to each original CAD, never motor-command offsets.
# The supplied bare CAD is the user's confirmed mechanical zero. ID uses
# different frames; align its palm-relative thumb MCP orientation to that CAD.
THUMB_ORIGIN_OFFSETS = {
    "TsenseI_v3_2": {"thumb_rota_block_joint": 0., "thumb_rotaback_joint2": 0.},
    "TsenseI_v3_2_withid": {
        "thumb_rota_block_joint": -0.10409343842285081,
        "thumb_rotaback_joint2": 0.36582211138762671,
    },
}
POSITIVE_SWING_CENTER = .350
SWING_CENTER = 0.
# Reference ID CAD range; the hardware command reserves 0.01 rad at each end.
SWING_LIMITS = (-.35, .35)
SWING_COMMAND_LIMITS = (-.34, .34)
# Match the active A3.2 URDF and SDK domain.  Keeping a narrower application
# range here silently rejects valid motion files and clips their downlink.
FLEXION_LIMITS = (0.0, 1.57)
MCP_LIMITS = (0.0, 1.57)


def thumb_origin_offset(model_id: str, name: str) -> float:
    suffix = name.removeprefix("left_hand_").removeprefix("right_hand_")
    return THUMB_ORIGIN_OFFSETS.get(model_id, {}).get(suffix, 0.)


def thumb_zero_manifest(model_id: str) -> dict:
    return {"thumb_zero_alignment": THUMB_ZERO_ALIGNMENT,
            "thumb_zero_reference": THUMB_ZERO_REFERENCE,
            "thumb_zero_reference_sha256": THUMB_ZERO_REFERENCE_SHA256,
            "thumb_origin_offsets_rad": dict(THUMB_ORIGIN_OFFSETS[model_id])}


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
    """Map the reference ID swing direction to the original SDK mechanism."""
    return SWING_CENTER - angle if is_finger_swing(name) else angle


def legacy_to_model(model_id: str, name: str, angle: float) -> float:
    """Saved signed swing -> current coordinate; retain mechanical motor zero."""
    if not is_finger_swing(name):
        return angle
    return -angle if model_id == "TsenseI_v3_2" else angle


def model_to_legacy(model_id: str, name: str, angle: float) -> float:
    """Undo saved-pose swing conversion, excluding fixed CAD origin correction."""
    if not is_finger_swing(name):
        return angle
    return -angle if model_id == "TsenseI_v3_2" else angle


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


def migrate_positive_pose(pose, limits):
    """Preserve the old positive-coordinate model pose, without rewriting files."""
    import math
    from numbers import Real
    if not isinstance(pose, dict) or pose.keys() != limits.keys():
        raise ValueError("A3.2 旧动作必须包含完整 20 个关节")
    result = {}
    for name, value in pose.items():
        bounds = (0., .7) if is_finger_swing(name) else limits[name]
        if (not isinstance(value, Real) or isinstance(value, bool) or not math.isfinite(value)
                or not bounds[0] - 1e-8 <= value <= bounds[1] + 1e-8):
            raise ValueError(f"A3.2 旧动作关节超出原坐标范围：{name}")
        value = POSITIVE_SWING_CENTER - value if is_finger_swing(name) else value
        lo, hi = limits[name]
        result[name] = min(hi, max(lo, value))
    return result
