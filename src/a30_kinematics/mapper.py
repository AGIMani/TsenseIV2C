"""Adapt A3.2 retained MCP/PIP linkages for A3.0; four-finger DIP and thumb PIP are direct rotary.

These are geometry outputs, not firmware commands. Motor indices retain the
A3.2 source model's 1..20 numbering and are NOT V2 USB slots. Mirrored right-hand
URDFs use equal joint coordinates, so geometry needs no additional sign flip.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from numbers import Real
import time
from typing import Mapping

from a32_kinematics.a32_angles_to_inputs import A32Error, load_configuration, load_kinematic_model
from hand_profiles import A30_MODEL_IDS, load_hand_profile
from a32_kinematics.a32_fast import default_converter
from .downlink import channel_mapping


# Shared with A3.0 trajectory validation. Only floating-point boundary noise
# within this tolerance may be normalized; physical joint ranges do not change.
JOINT_LIMIT_EPS_RAD = 1e-7


# A3.0 assets use functional pinky naming; normalize the source SWING spelling.
SOURCE_TO_PROJECT = {"left_hand_pinky_SWING_joint": "left_hand_pinky_swing_joint"}
DIRECT_ROTARY_SOURCE = frozenset(f"left_hand_{finger}_DIP_joint"
                              for finger in ("index", "mid", "ring", "pinky")) | {"left_hand_thumb_PIP_joint"}



@dataclass(frozen=True)
class A30MotorTargets:
    hand_side: str
    method: str
    joint_angles_rad: dict[str, float]
    linear_strokes_mm: dict[str, float]
    rotary_positions_deg: dict[str, float]
    clamped_joints_rad: dict[str, float]
    model_id: str = "TsenseI_v3_0"

    def as_dict(self) -> dict:
        return {
            "model_id": self.model_id, "hand_side": self.hand_side,
            "method": self.method, "valid": True,
            "joint_angles_rad": dict(self.joint_angles_rad),
            "linear_strokes_mm": dict(self.linear_strokes_mm),
            "rotary_positions_deg": dict(self.rotary_positions_deg),
            "clamped_joints_rad": dict(self.clamped_joints_rad),
        }


class A30StrokeMapper:
    """Create once per A3.0 session; convert complete 20-joint URDF poses.

    lut is the project default (interpolation error bound <= 1e-9 m).
    exact remains available with full branch/roundtrip checks. Strict
    conversion rejects unreachable targets; preview explicitly clamps to
    mechanism limits and reports the affected joints.
    """
    model_id = "TsenseI_v3_0"

    def __init__(self, hand_side: str = "left", *, method: str = "lut", model_id: str = "TsenseI_v3_0"):
        if model_id not in A30_MODEL_IDS:
            raise ValueError("Unsupported A3.0 model")
        self.model_id = model_id
        if hand_side not in ("left", "right"):
            raise ValueError("A3.0 hand_side must be left or right")
        self.hand_side = hand_side
        self.method = method
        model = load_kinematic_model()
        config = load_configuration()
        self.source_joint_names = model.q_names
        self.joint_names = tuple(
            SOURCE_TO_PROJECT.get(name, name).replace("left_hand_", f"{hand_side}_hand_")
            for name in self.source_joint_names)
        self._name_set = frozenset(self.joint_names)
        self._linear = tuple(p["type"] == "linear" and name not in DIRECT_ROTARY_SOURCE
                             for p, name in zip(config["motors"], self.source_joint_names))
        # Only the nine retained linear mechanisms enter the per-frame converter.
        # Fingertip targets never pass through an A3.2 slider limit or lookup table.
        self._linear_indices = tuple(i for i, linear in enumerate(self._linear) if linear)
        self._prepared = default_converter().prepare(
            [i + 1 for i in self._linear_indices], method=method)
        urdf_limits = load_hand_profile(self.model_id).joint_limits(hand_side)
        if set(urdf_limits) != self._name_set:
            raise ValueError("A3.0 conversion joint names disagree with the model profile")
        limits = [urdf_limits[name] for name in self.joint_names]
        for i, (lo, hi) in zip(self._linear_indices, self._prepared.limits_deg):
            ulo, uhi = limits[i]
            lower, upper = max(ulo, math.radians(lo)), min(uhi, math.radians(hi))
            # Geometry conversion can turn the same zero endpoint into 4e-16.
            # Keep the URDF endpoint exact when the discrepancy is roundoff.
            limits[i] = (ulo if abs(lower-ulo) <= JOINT_LIMIT_EPS_RAD else lower,
                         uhi if abs(upper-uhi) <= JOINT_LIMIT_EPS_RAD else upper)
        self._limits = tuple(limits)
        self._firmware_channels = {r["joint"]: r for r in channel_mapping(hand_side)}
        if (len(self.joint_names) != 20 or len(self._name_set) != 20
                or sum(self._linear) != 9 or any(lo > hi for lo, hi in limits)):
            raise ValueError("Unexpected A3.0 model: expected 20 joints (9 linear / 11 rotary)")

    def description(self) -> dict:
        from .policy import downlink_ranges
        ranges = downlink_ranges(self)
        return {
            "model_id": self.model_id, "hand_side": self.hand_side,
            "method": self.method, "coordinate_space": "urdf_radians",
            "output_space": "mechanical_geometry", "hardware_enabled": False,
            "conversion_ready": True, "encoder_ready": True, "downlink_ready": False,
            "status": "conversion_ready", "transport": "browser_web_serial",
            "linear_geometry_source": "A3.2 retained MCP/PIP linkage parameters",
            "direct_rotary_joints": [n for n, linear in zip(self.joint_names, self._linear) if not linear],
            "units": {"linear": "mm", "rotary": "deg"},
            "channels": [
                {"source_motor_index": i + 1, "firmware_channel": dict(self._firmware_channels[name]), "source_joint": source, "joint": name,
                 "type": "linear" if linear else "rotary", "limits_rad": list(limits),
                 "downlink": ranges[name]}
                for i, (source, name, linear, limits) in enumerate(zip(
                    self.source_joint_names, self.joint_names, self._linear, self._limits))],
        }

    def convert(self, joint_angles: Mapping[str, float], *, clamp: bool = False) -> A30MotorTargets:
        if not isinstance(joint_angles, Mapping):
            raise ValueError("A3.0 pose must be a joint-name mapping")
        if joint_angles.keys() != self._name_set:
            missing = self._name_set - joint_angles.keys()
            unknown = joint_angles.keys() - self._name_set
            raise ValueError(f"A3.0 requires its own complete 20-joint pose; "
                             f"missing={sorted(missing)}, unknown={sorted(unknown, key=repr)}")
        angles, applied, clamped = [], {}, {}
        for name, (lo, hi) in zip(self.joint_names, self._limits):
            value = joint_angles[name]
            # Ruckig produces built-in floats. Avoid the costly numeric ABC
            # path for those while retaining support for other Real types.
            if type(value) not in (float, int) and (isinstance(value, bool) or not isinstance(value, Real)):
                raise ValueError(f"A3.0 joint must be a finite real angle: {name}")
            try:
                value = float(value)
            except (ValueError, TypeError, OverflowError) as exc:
                raise ValueError(f"A3.0 joint must be a finite real angle: {name}") from exc
            if not math.isfinite(value):
                raise ValueError(f"A3.0 joint must be a finite real angle: {name}")
            if value < lo - JOINT_LIMIT_EPS_RAD or value > hi + JOINT_LIMIT_EPS_RAD:
                if not clamp:
                    raise A32Error("AngleLimit", f"{name}: {value} rad outside [{lo}, {hi}]")
                value = min(hi, max(lo, value))
                clamped[name] = value
            # Match the upstream epsilon handling at zero/limit endpoints.
            value = min(hi, max(lo, value))
            angles.append(math.degrees(value))
            applied[name] = value
        inputs = self._prepared([angles[i] for i in self._linear_indices])
        linear = {self.joint_names[i]: float(value) * 1000.0
                  for i, value in zip(self._linear_indices, inputs)}
        rotary = {name: angles[i] for i, name in enumerate(self.joint_names) if not self._linear[i]}
        return A30MotorTargets(self.hand_side, self.method, applied, linear, rotary, clamped, self.model_id)

    def preview(self, joint_angles: Mapping[str, float]) -> dict:
        """Per-frame preview, including explicit failure instead of stale inputs."""
        start = time.perf_counter()
        try:
            result = self.convert(joint_angles, clamp=True).as_dict()
        except (ValueError, TypeError, ArithmeticError) as exc:
            result = {"model_id": self.model_id, "hand_side": self.hand_side,
                      "method": self.method, "valid": False, "error": str(exc),
                      "linear_strokes_mm": {}, "rotary_positions_deg": {}}
        result.update(conversion_ready=True, encoder_ready=True, downlink_ready=False, hardware_enabled=False,
                      status="conversion_ready", transport="browser_web_serial",
                      linear_geometry_source="A3.2 retained MCP/PIP linkage parameters")
        result["conversion_ms"] = (time.perf_counter() - start) * 1000.0
        return result
