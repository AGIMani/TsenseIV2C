"""Strict, native 21-point Quest input carried over an ADB reverse tunnel.

This protocol is separate from WebXR's native 25 joints.  Coordinates use the
same right-handed metric axes as WebXR, but no missing metacarpals are invented.
The relay owns pairing, stream ordering, freshness and wrist normalization.
"""
from __future__ import annotations

from dataclasses import dataclass
import math


QUEST_ADB_PROTOCOL_SCHEMA = "teleop.quest.adb.hand_skeleton.v1"
QUEST_ADB_JOINT_SET = "mediapipe-21"
_EVENT_TYPES = frozenset({
    "client_hello", "session_started", "hand_skeleton", "tracking_lost", "tracking_restored",
    "session_ended", "client_error",
})
_COORDINATE_SPACES = frozenset({"webxr_local", "webxr_local_floor"})


@dataclass(frozen=True)
class QuestADBHandFrame:
    hand_side: str
    tracked: bool
    landmarks: tuple[tuple[float, float, float], ...]

    def mediapipe_landmarks(self) -> tuple[tuple[float, float, float], ...]:
        return self.landmarks if self.tracked else ()


@dataclass(frozen=True)
class ParsedQuestADBMessage:
    type: str
    handedness: str | None
    frames: tuple[QuestADBHandFrame, ...]


def _number(value, label: str, *, nonnegative: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"Quest ADB {label} 必须为有限数值")
    try:
        number = float(value)
    except OverflowError as exc:
        raise ValueError(f"Quest ADB {label} 必须为有限数值") from exc
    if not math.isfinite(number) or (nonnegative and number < 0):
        raise ValueError(f"Quest ADB {label} 必须为有限{'非负' if nonnegative else ''}数值")
    return number


def _side(value) -> str:
    if not isinstance(value, str) or value not in ("left", "right"):
        raise ValueError("Quest ADB handedness 必须为 left 或 right")
    return value


def parse_quest_adb_message(message: dict) -> ParsedQuestADBMessage:
    """Validate the 21-point payload before the shared relay changes state."""
    if not isinstance(message, dict) or message.get("schema") != QUEST_ADB_PROTOCOL_SCHEMA:
        raise ValueError(f"Quest ADB schema 必须为 {QUEST_ADB_PROTOCOL_SCHEMA}")
    event = message.get("type")
    if not isinstance(event, str) or event not in _EVENT_TYPES:
        raise ValueError("Quest ADB type 不受支持")
    skeleton = event == "hand_skeleton"
    for key, expected in (("joint_set", QUEST_ADB_JOINT_SET), ("units", "m")):
        if (skeleton or key in message) and message.get(key) != expected:
            raise ValueError(f"Quest ADB {key} 必须为 {expected}")
    if skeleton or "coordinate_space" in message:
        space = message.get("coordinate_space")
        if not isinstance(space, str) or space not in _COORDINATE_SPACES:
            raise ValueError("Quest ADB coordinate_space 必须为 webxr_local 或 webxr_local_floor")
    for key in ("timestamp_ms", "client_perf_ms", "xr_predicted_display_time_ms"):
        if key in message:
            _number(message[key], key, nonnegative=True)
    if skeleton and "client_perf_ms" not in message:
        raise ValueError("Quest ADB 骨架必须带有 client_perf_ms")
    side = _side(message["handedness"]) if message.get("handedness") is not None else None
    frames = []
    if skeleton:
        hands = message.get("hands")
        if not isinstance(hands, list) or len(hands) > 2:
            raise ValueError("Quest ADB hands 必须为最多两只手的数组")
        seen_sides = set()
        for record in hands:
            if not isinstance(record, dict):
                raise ValueError("Quest ADB 每只手必须为对象")
            hand_side = _side(record.get("handedness"))
            if hand_side in seen_sides:
                raise ValueError("Quest ADB hands 不能包含重复 handedness")
            seen_sides.add(hand_side)
            tracked = record.get("tracked")
            if not isinstance(tracked, bool):
                raise ValueError("Quest ADB tracked 必须为布尔值")
            landmarks = record.get("landmarks")
            if not isinstance(landmarks, list) or len(landmarks) not in ((21,) if tracked else (0, 21)):
                raise ValueError("Quest ADB landmarks 必须恰好为 21 个关节点；未追踪时可为空数组")
            points = []
            for point in landmarks:
                if not isinstance(point, list) or len(point) != 3:
                    raise ValueError("Quest ADB 每个关节点必须为 [x, y, z]")
                points.append(tuple(_number(value, "landmarks") for value in point))
            frames.append(QuestADBHandFrame(hand_side, tracked, tuple(points)))
    return ParsedQuestADBMessage(event, side, tuple(frames))
