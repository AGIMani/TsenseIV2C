"""Map A3.2 project URDF radians to named mechanical inputs (mm / deg).

These are geometry outputs, not firmware commands. Motor indices retain the
source model's 1..20 numbering and are NOT V2 USB slots. Mirrored right-hand
URDFs use equal joint coordinates, so geometry needs no additional sign flip.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real
import time
from typing import Mapping

from .a32_angles_to_inputs import A32Error, load_configuration, load_kinematic_model
from .a32_fast import default_converter
from .joint_coordinates import COORDINATE_VERSION, SWING_AXIS_ALIGNMENT, is_finger_swing, requested_limits, reverse_swing


# Current project URDF uses functional names: MCP is Y-axis linear flexion,
# swing is X-axis rotary abduction. Only the source SWING spelling differs.
SOURCE_TO_PROJECT = {
    "left_hand_pinky_SWING_joint": "left_hand_pinky_swing_joint",
}

A32_MODEL_IDS = ("TsenseI_v3_2", "TsenseI_v3_2_withid")


def model_joint_to_source_sign(model_id: str, joint_name: str) -> float:
    """Centered application swing -> original SDK mechanism angle.

    Both installed variants now use the ID swing direction. After undoing
    the positive offset with 0.350-q, negate all four swings to recover the
    original bare SDK mechanism angle. Other axes retain their direction.
    """
    if model_id not in A32_MODEL_IDS:
        raise ValueError(f"Unsupported A3.2 model: {model_id}")
    return -1.0 if is_finger_swing(joint_name) else 1.0


@dataclass(frozen=True)
class A32MotorTargets:
    hand_side: str
    method: str
    joint_angles_rad: dict[str, float]
    linear_strokes_mm: dict[str, float]
    rotary_positions_deg: dict[str, float]
    clamped_joints_rad: dict[str, float]
    model_id: str = "TsenseI_v3_2"

    def as_dict(self) -> dict:
        return {
            "model_id": self.model_id, "hand_side": self.hand_side,
            "joint_coordinate_version": COORDINATE_VERSION,
            "swing_axis_alignment": SWING_AXIS_ALIGNMENT,
            "method": self.method, "valid": True,
            "joint_angles_rad": dict(self.joint_angles_rad),
            "linear_strokes_mm": dict(self.linear_strokes_mm),
            "rotary_positions_deg": dict(self.rotary_positions_deg),
            "clamped_joints_rad": dict(self.clamped_joints_rad),
        }


class A32StrokeMapper:
    """Create once per A3.2 session; convert complete 20-joint URDF poses.

    lut is the project default (interpolation error bound <= 1e-9 m).
    exact remains available with full branch/roundtrip checks. Strict
    conversion rejects unreachable targets; preview explicitly clamps to
    mechanism limits and reports the affected joints.
    """
    model_id = "TsenseI_v3_2"

    def __init__(self, hand_side: str = "left", *, method: str = "lut",
                 model_id: str = "TsenseI_v3_2"):
        if hand_side not in ("left", "right"):
            raise ValueError("A3.2 hand_side must be left or right")
        self.hand_side = hand_side
        if model_id not in A32_MODEL_IDS:
            raise ValueError(f"Unsupported A3.2 model: {model_id}")
        self.model_id = model_id
        self.method = method
        model = load_kinematic_model()
        config = load_configuration()
        self._prepared = default_converter().prepare(method=method)
        self.source_joint_names = model.q_names
        self.joint_names = tuple(
            SOURCE_TO_PROJECT.get(name, name).replace("left_hand_", f"{hand_side}_hand_")
            for name in self.source_joint_names)
        self._name_set = frozenset(self.joint_names)
        self._linear = tuple(p["type"] == "linear" for p in config["motors"])
        self._source_signs = tuple(model_joint_to_source_sign(model_id, name) for name in self.joint_names)
        # Source geometry retains its signed CAD zero for linear inversion.
        # Application limits/coordinates are shared with the installed URDFs.
        limits = []
        for name, (lo, hi), sign in zip(self.joint_names, self._prepared.limits_deg, self._source_signs):
            bounds = sorted(reverse_swing(name, sign * math.radians(q)) for q in (lo, hi))
            requested = requested_limits(name)
            if requested is not None:
                bounds = [max(bounds[0], requested[0]), min(bounds[1], requested[1])]
            limits.append(tuple(bounds))
        self._limits = tuple(limits)
        if len(self.joint_names) != 20 or len(self._name_set) != 20 or sum(self._linear) != 14:
            raise ValueError("Unexpected A3.2 model: expected 20 joints (14 linear / 6 rotary)")

    def description(self) -> dict:
        return {
            "model_id": self.model_id, "hand_side": self.hand_side,
            "joint_coordinate_version": COORDINATE_VERSION,
            "swing_axis_alignment": SWING_AXIS_ALIGNMENT,
            "method": self.method, "coordinate_space": "urdf_radians",
            "output_space": "mechanical_geometry", "hardware_enabled": False,
            "units": {"linear": "mm", "rotary": "deg"},
            "channels": [
                {"motor_index": i + 1, "source_joint": source, "joint": name,
                 "type": "linear" if linear else "rotary", "limits_rad": list(limits),
                 "source_sign": sign}
                for i, (source, name, linear, limits, sign) in enumerate(zip(
                    self.source_joint_names, self.joint_names, self._linear, self._limits, self._source_signs))],
        }

    def convert(self, joint_angles: Mapping[str, float], *, clamp: bool = False) -> A32MotorTargets:
        if not isinstance(joint_angles, Mapping):
            raise ValueError("A3.2 pose must be a joint-name mapping")
        if joint_angles.keys() != self._name_set:
            missing = self._name_set - joint_angles.keys()
            unknown = joint_angles.keys() - self._name_set
            raise ValueError(f"A3.2 requires its own complete 20-joint pose; "
                             f"missing={sorted(missing)}, unknown={sorted(unknown, key=repr)}")
        angles, applied, clamped = [], {}, {}
        for name, (lo, hi), sign in zip(self.joint_names, self._limits, self._source_signs):
            value = joint_angles[name]
            if isinstance(value, bool) or not isinstance(value, Real):
                raise ValueError(f"A3.2 joint must be a finite real angle: {name}")
            try:
                value = float(value)
            except (ValueError, TypeError, OverflowError) as exc:
                raise ValueError(f"A3.2 joint must be a finite real angle: {name}") from exc
            if not math.isfinite(value):
                raise ValueError(f"A3.2 joint must be a finite real angle: {name}")
            if value < lo - 1e-10 or value > hi + 1e-10:
                if not clamp:
                    raise A32Error("AngleLimit", f"{name}: {value} rad outside [{lo}, {hi}]")
                value = min(hi, max(lo, value))
                clamped[name] = value
            # Match the upstream epsilon handling at zero/limit endpoints.
            value = min(hi, max(lo, value))
            angles.append(math.degrees(sign * reverse_swing(name, value)))
            applied[name] = value
        inputs = self._prepared(angles)
        linear, rotary = {}, {}
        for name, is_linear, value in zip(self.joint_names, self._linear, inputs):
            if is_linear:
                linear[name] = float(value) * 1000.0
            elif is_finger_swing(name):
                # Both variants command the new positive coordinate directly.
                # The ID CAD axis sign is only used by the source mechanism;
                # applying it again here would swap the requested endpoints.
                rotary[name] = math.degrees(applied[name])
            else:
                rotary[name] = math.degrees(float(value))
        return A32MotorTargets(self.hand_side, self.method, applied, linear, rotary, clamped, self.model_id)

    def preview(self, joint_angles: Mapping[str, float]) -> dict:
        """Per-frame preview, including explicit failure instead of stale inputs."""
        start = time.perf_counter()
        try:
            result = self.convert(joint_angles, clamp=True).as_dict()
        except (ValueError, TypeError, ArithmeticError) as exc:
            result = {"model_id": self.model_id, "hand_side": self.hand_side,
                      "method": self.method, "valid": False, "error": str(exc),
                      "linear_strokes_mm": {}, "rotary_positions_deg": {}}
        result["conversion_ms"] = (time.perf_counter() - start) * 1000.0
        return result
