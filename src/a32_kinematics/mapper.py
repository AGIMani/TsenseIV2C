"""Map A3.2 URDF radians to SDK 1.1.15 actuator inputs (mm / deg).

Motor indices retain the source model's 1..20 numbering and are NOT V2 USB
slots. Mirrored right-hand URDFs use equal joint coordinates. Conversion and
preview use pure Python SDK math and never load a device or native library.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real
import time
from typing import Mapping

from .a32_angles_to_inputs import A32Error
from .joint_coordinates import COORDINATE_VERSION, SWING_AXIS_ALIGNMENT, requested_limits
from . import sdk_stroke


# Current project URDF uses functional names: MCP is Y-axis linear flexion,
# swing is X-axis rotary abduction. Only the source SWING spelling differs.
SOURCE_TO_PROJECT = {
    "left_hand_pinky_SWING_joint": "left_hand_pinky_swing_joint",
}

A32_MODEL_IDS = ("TsenseI_v3_2", "TsenseI_v3_2_withid")


def model_joint_to_source_sign(model_id: str, joint_name: str) -> float:
    """Both application models use the SDK's direct signed swing commands."""
    if model_id not in A32_MODEL_IDS:
        raise ValueError(f"Unsupported A3.2 model: {model_id}")
    return 1.0


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
            "method": self.method, "conversion": sdk_stroke.METHOD, "valid": True,
            "joint_angles_rad": dict(self.joint_angles_rad),
            "linear_strokes_mm": dict(self.linear_strokes_mm),
            "rotary_positions_deg": dict(self.rotary_positions_deg),
            "clamped_joints_rad": dict(self.clamped_joints_rad),
        }


class A32StrokeMapper:
    """Create once per A3.2 session; convert complete 20-joint URDF poses.

    The historical lut/exact method selectors remain accepted; both now use
    the SDK calibrated closed-loop solver. Strict conversion rejects angles
    outside application limits; preview clamps and reports affected joints.
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
        if method not in ("lut", "exact"):
            raise ValueError("A3.2 method must be lut or exact")
        self.method = method
        self.source_joint_names = sdk_stroke.SOURCE_JOINT_NAMES
        self.joint_names = tuple(
            SOURCE_TO_PROJECT.get(name, name).replace("left_hand_", f"{hand_side}_hand_")
            for name in self.source_joint_names)
        self._name_set = frozenset(self.joint_names)
        self._linear = tuple(joint.linear for joint in sdk_stroke.JOINTS)
        self._joint_indices = {name: i for i, name in enumerate(self.joint_names)}
        self._source_signs = tuple(model_joint_to_source_sign(model_id, name) for name in self.joint_names)
        # The SDK accepts the full linear domain up to 90 degrees. Match the
        # active URDF/application range of 0..1.57 radians for flexion.
        limits = []
        for name, joint in zip(self.joint_names, sdk_stroke.JOINTS):
            bounds = [joint.lo, joint.hi]
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
            "method": self.method, "conversion": sdk_stroke.METHOD,
            "sdk_stroke_source_sha256": sdk_stroke.SOURCE_SHA256,
            "coordinate_space": "urdf_radians",
            "output_space": "sdk_actuator_inputs", "hardware_enabled": False,
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
            angles.append(math.degrees(sign * value))
            applied[name] = value
        inputs = sdk_stroke.angles_deg_to_actuators(angles)
        linear, rotary = {}, {}
        for name, is_linear, value in zip(self.joint_names, self._linear, inputs):
            if is_linear:
                linear[name] = value
            else:
                rotary[name] = value
        return A32MotorTargets(self.hand_side, self.method, applied, linear, rotary, clamped, self.model_id)

    def inverse_actuator(self, name: str, value: float) -> float:
        """Convert one SDK actuator value (mm/deg) back to joint radians.

        This uses the same calibrated SDK equation as forward conversion. Its
        mathematical endpoints include zero for feedback and hardware limits;
        callers apply the narrower application limits to the returned angle.
        """
        try:
            index = self._joint_indices[name]
        except (KeyError, TypeError) as exc:
            raise ValueError(f"Unknown A3.2 joint: {name}") from exc
        return sdk_stroke.inverse_actuator(index, value) * self._source_signs[index]

    def preview(self, joint_angles: Mapping[str, float]) -> dict:
        """Per-frame preview, including explicit failure instead of stale inputs."""
        start = time.perf_counter()
        try:
            result = self.convert(joint_angles, clamp=True).as_dict()
        except (ValueError, TypeError, ArithmeticError) as exc:
            result = {"model_id": self.model_id, "hand_side": self.hand_side,
                      "method": self.method, "conversion": sdk_stroke.METHOD,
                      "valid": False, "error": str(exc),
                      "linear_strokes_mm": {}, "rotary_positions_deg": {}}
        result["conversion_ms"] = (time.perf_counter() - start) * 1000.0
        return result
