"""A3.0 conversion and pure protocol encoder for future browser-local transport.

Angles are full, named URDF joint poses in radians; stroke commands use mm and
rotary motor commands use output-shaft degrees. Confirmed channels use CAN1/2/3.
The browser (not the server) will own the eventual serial transport.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Protocol
from a30_kinematics import A30StrokeMapper, A30MotorTargets
from a30_kinematics.policy import encode_control_targets as encode_targets


@dataclass(frozen=True)
class ActuatorTargets:
    linear_strokes_mm: Mapping[str, float]
    rotary_positions_deg: Mapping[str, float]


class AngleToStroke(Protocol):
    def convert(self, joint_angles_rad: Mapping[str, float], *, hand_side: str) -> A30MotorTargets: ...


class DownlinkEncoder(Protocol):
    """Return protocol bytes for the client-side transport; never open server USB."""
    def encode(self, targets: ActuatorTargets, *, hand_side: str) -> bytes: ...


class A30Actuation(A30StrokeMapper):
    def convert(self, joint_angles_rad: Mapping[str, float], *, hand_side=None, clamp=False) -> A30MotorTargets:
        if hand_side is not None and hand_side != self.hand_side:
            raise ValueError("A3.0 conversion hand_side does not match this session")
        return super().convert(joint_angles_rad, clamp=clamp)

    def encode(self, targets: ActuatorTargets | A30MotorTargets, *, hand_side=None,
               seq=0, mit_vel_rpm=(0.0, 0.0), mit_iq_ff_a=(0.0, 0.0)) -> bytes:
        side = self.hand_side if hand_side is None else hand_side
        if side != self.hand_side:
            raise ValueError("A3.0 encoder hand_side does not match this session")
        return encode_targets(targets, hand_side=side, seq=seq,
                              mit_vel_rpm=mit_vel_rpm, mit_iq_ff_a=mit_iq_ff_a)
