"""Convert AGIMani/Teleop Quest hand snapshots to the Form UDP protocol.

This module deliberately depends only on the Python standard library.  The
runtime bridge normally runs in the IsaacTeleop/CloudXR environment, which
does not need to install the Qt application's vision dependencies.
"""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import time
from typing import Any, Mapping, Sequence


QUEST3_BRIDGE_SCHEMA = "hand.quest3.openxr26.v1"
MAX_UDP_PAYLOAD_BYTES = 65_507

# OpenXR XR_EXT_hand_tracking / IsaacTeleop HandJointIndex order.
OPENXR_HAND_JOINT_NAMES: tuple[str, ...] = (
    "palm",
    "wrist",
    "thumb_metacarpal",
    "thumb_proximal",
    "thumb_distal",
    "thumb_tip",
    "index_metacarpal",
    "index_proximal",
    "index_intermediate",
    "index_distal",
    "index_tip",
    "middle_metacarpal",
    "middle_proximal",
    "middle_intermediate",
    "middle_distal",
    "middle_tip",
    "ring_metacarpal",
    "ring_proximal",
    "ring_intermediate",
    "ring_distal",
    "ring_tip",
    "little_metacarpal",
    "little_proximal",
    "little_intermediate",
    "little_distal",
    "little_tip",
)

# The Form pipeline consumes the MediaPipe-compatible subset of the 26 joints.
# Keep this identical to tracking.cloudxr.OPENXR_26_TO_MEDIAPIPE_21 while
# retaining a dependency-free module for the remote CloudXR host.
OPENXR_26_TO_FORM_21: tuple[int, ...] = (
    1,
    2, 3, 4, 5,
    7, 8, 9, 10,
    12, 13, 14, 15,
    17, 18, 19, 20,
    22, 23, 24, 25,
)


class TeleopHandSnapshotError(ValueError):
    """Raised when a Teleop ``hand_debug`` snapshot is structurally invalid."""


@dataclass(frozen=True)
class PreparedOpenXRHand:
    """Canonical, finite OpenXR-26 positions plus per-joint validity."""

    positions: tuple[tuple[float, float, float], ...]
    valid: tuple[bool, ...]

    @property
    def form_joints_valid(self) -> bool:
        return all(self.valid[index] for index in OPENXR_26_TO_FORM_21)

    @property
    def valid_count(self) -> int:
        return sum(self.valid)


def _is_array(value: Any) -> bool:
    return isinstance(value, Sequence) and not isinstance(
        value, (str, bytes, bytearray)
    )


def _normalize_joint_name(value: Any) -> str:
    name = str(value).strip().lower().replace("-", "_").replace(" ", "_")
    if "." in name:
        name = name.rsplit(".", 1)[-1]
    for prefix in ("xr_hand_joint_", "hand_joint_"):
        if name.startswith(prefix):
            name = name[len(prefix):]
    if name.endswith("_ext"):
        name = name[:-4]
    if name.startswith("pinky_"):
        name = "little_" + name[len("pinky_"):]
    return name


def _canonical_source_indices(raw_names: Any) -> tuple[int, ...]:
    if raw_names is None:
        return tuple(range(len(OPENXR_HAND_JOINT_NAMES)))
    if not _is_array(raw_names) or len(raw_names) != len(OPENXR_HAND_JOINT_NAMES):
        raise TeleopHandSnapshotError("joint_names must contain 26 entries")

    normalized = tuple(_normalize_joint_name(name) for name in raw_names)
    if len(set(normalized)) != len(normalized):
        raise TeleopHandSnapshotError("joint_names contains duplicate entries")
    index_by_name = {name: index for index, name in enumerate(normalized)}
    missing = [name for name in OPENXR_HAND_JOINT_NAMES if name not in index_by_name]
    if missing:
        raise TeleopHandSnapshotError(
            "joint_names is missing OpenXR entries: " + ", ".join(missing)
        )
    return tuple(index_by_name[name] for name in OPENXR_HAND_JOINT_NAMES)


def _finite_xyz(value: Any) -> tuple[float, float, float] | None:
    if not _is_array(value) or len(value) < 3:
        return None
    try:
        point = (float(value[0]), float(value[1]), float(value[2]))
    except (TypeError, ValueError, OverflowError):
        return None
    return point if all(math.isfinite(axis) for axis in point) else None


def prepare_teleop_hand_snapshot(snapshot: Mapping[str, Any]) -> PreparedOpenXRHand:
    """Validate and canonicalize Teleop's ``optional_hand_debug_snapshot`` output.

    ``joint_names`` is used when available, so a future producer may change its
    array order without silently corrupting finger identity.  Invalid points are
    replaced with finite placeholders but retain ``valid=False``; a tracked
    packet is emitted only when every joint needed by the Form's 21-point chain
    is valid.
    """

    if not isinstance(snapshot, Mapping):
        raise TeleopHandSnapshotError("hand_debug snapshot must be an object")

    raw_positions = snapshot.get("joint_positions_xyz")
    if not _is_array(raw_positions) or len(raw_positions) != 26:
        raise TeleopHandSnapshotError("joint_positions_xyz must have shape (26, 3)")

    raw_valid = snapshot.get("joint_valid")
    if raw_valid is None:
        source_valid = [True] * 26
    elif _is_array(raw_valid) and len(raw_valid) == 26:
        source_valid = [bool(value) for value in raw_valid]
    else:
        raise TeleopHandSnapshotError("joint_valid must contain 26 entries")

    source_indices = _canonical_source_indices(snapshot.get("joint_names"))
    positions: list[tuple[float, float, float]] = []
    valid: list[bool] = []
    for source_index in source_indices:
        point = _finite_xyz(raw_positions[source_index])
        is_valid = bool(source_valid[source_index]) and point is not None
        positions.append(point if point is not None else (0.0, 0.0, 0.0))
        valid.append(is_valid)

    return PreparedOpenXRHand(tuple(positions), tuple(valid))


class TeleopHandPacketEncoder:
    """Stateful Teleop snapshot to compact JSON/UDP packet encoder.

    Briefly invalid hand frames are dropped instead of injecting partial joint
    data.  Once ``loss_timeout_s`` elapses, one explicit ``tracked=false``
    packet is emitted.  The Form receiver has the same stale-frame safety net.
    """

    def __init__(
        self,
        *,
        hand_side: str = "right",
        loss_timeout_s: float = 0.35,
        coordinate_space: str = "openxr",
    ) -> None:
        normalized_side = str(hand_side).strip().lower()
        if normalized_side not in {"left", "right"}:
            raise ValueError("hand_side must be 'left' or 'right'")
        if float(loss_timeout_s) < 0.0:
            raise ValueError("loss_timeout_s must be non-negative")

        self.hand_side = normalized_side
        self.loss_timeout_s = float(loss_timeout_s)
        self.coordinate_space = str(coordinate_space or "openxr").strip().lower()
        self._sequence = -1
        self._tracking_state = "unknown"
        self._last_valid_monotonic: float | None = None
        self.last_error = ""

    @property
    def sequence(self) -> int:
        return self._sequence

    @property
    def tracking_state(self) -> str:
        return self._tracking_state

    def packet_for_snapshot(
        self,
        snapshot: Mapping[str, Any] | None,
        *,
        timestamp_ns: int | None = None,
        monotonic_s: float | None = None,
    ) -> dict[str, Any] | None:
        """Return a tracked/lost packet, or ``None`` for a tolerated bad frame."""

        self._sequence += 1
        now = time.monotonic() if monotonic_s is None else float(monotonic_s)
        timestamp = time.monotonic_ns() if timestamp_ns is None else int(timestamp_ns)

        prepared: PreparedOpenXRHand | None = None
        if snapshot is not None:
            try:
                prepared = prepare_teleop_hand_snapshot(snapshot)
                self.last_error = ""
            except TeleopHandSnapshotError as exc:
                self.last_error = str(exc)

        if prepared is not None and prepared.form_joints_valid:
            self._last_valid_monotonic = now
            self._tracking_state = "tracked"
            return self._base_packet(timestamp_ns=timestamp, tracked=True) | {
                "joints": [list(point) for point in prepared.positions],
                "joint_valid": list(prepared.valid),
                "joint_names": list(OPENXR_HAND_JOINT_NAMES),
            }

        if prepared is not None and not prepared.form_joints_valid:
            self.last_error = (
                "required Form joints are invalid "
                f"({prepared.valid_count}/26 OpenXR joints valid)"
            )

        if self._tracking_state == "unknown":
            self._tracking_state = "lost"
            return self._base_packet(timestamp_ns=timestamp, tracked=False)
        if self._tracking_state == "tracked":
            last_valid = self._last_valid_monotonic
            if last_valid is None or now - last_valid >= self.loss_timeout_s:
                self._tracking_state = "lost"
                return self._base_packet(timestamp_ns=timestamp, tracked=False)
        return None

    def force_tracking_lost(
        self,
        *,
        timestamp_ns: int | None = None,
    ) -> dict[str, Any] | None:
        """Emit one final loss packet when a previously tracked stream stops."""

        if self._tracking_state != "tracked":
            return None
        self._sequence += 1
        self._tracking_state = "lost"
        timestamp = time.monotonic_ns() if timestamp_ns is None else int(timestamp_ns)
        return self._base_packet(timestamp_ns=timestamp, tracked=False)

    def _base_packet(self, *, timestamp_ns: int, tracked: bool) -> dict[str, Any]:
        return {
            "type": "hand_landmarks",
            "schema": QUEST3_BRIDGE_SCHEMA,
            "source": "agi_teleop_isaacteleop_hand_tracker",
            "sequence": self._sequence,
            "timestamp_ns": int(timestamp_ns),
            "handedness": self.hand_side,
            "tracked": bool(tracked),
            "coordinate_space": self.coordinate_space,
            "unit": "m",
        }


def encode_udp_packet(packet: Mapping[str, Any]) -> bytes:
    """Encode one protocol packet and enforce the IPv4 UDP payload limit."""

    try:
        payload = json.dumps(
            packet,
            ensure_ascii=True,
            allow_nan=False,
            separators=(",", ":"),
        ).encode("utf-8")
    except (TypeError, ValueError, OverflowError) as exc:
        raise TeleopHandSnapshotError("packet is not finite JSON data") from exc
    if len(payload) > MAX_UDP_PAYLOAD_BYTES:
        raise TeleopHandSnapshotError(
            f"encoded packet is too large for UDP: {len(payload)} bytes"
        )
    return payload


__all__ = [
    "MAX_UDP_PAYLOAD_BYTES",
    "OPENXR_26_TO_FORM_21",
    "OPENXR_HAND_JOINT_NAMES",
    "PreparedOpenXRHand",
    "QUEST3_BRIDGE_SCHEMA",
    "TeleopHandPacketEncoder",
    "TeleopHandSnapshotError",
    "encode_udp_packet",
    "prepare_teleop_hand_snapshot",
]
