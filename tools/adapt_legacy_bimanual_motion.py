#!/usr/bin/env python3
"""Adapt a legacy bimanual TsenseI motion to the current Form2 format.

The legacy URDF used the opposite four-finger swing axes and placed the
business init at q=26 degrees.  The current URDF represents the same pose as
``q_current = 26deg - q_legacy`` and limits swing to [0, 0.696] rad.

Form2 motion files are single-hand files.  This tool therefore rewrites the
input as the left-hand file and creates a ``*_right`` companion.  A byte-for-
byte backup is made before the input is replaced.
"""
from __future__ import annotations

import argparse
import json
import math
import shutil
from pathlib import Path
from typing import Any


LEGACY_SWING_ZERO_RAD = math.radians(26.0)
CURRENT_SWING_MIN_RAD = 0.0
CURRENT_SWING_MAX_RAD = 0.696

JOINT_SUFFIXES = (
    "thumb_rota_joint",
    "thumb_swing_joint",
    "thumb_MCP_joint",
    "thumb_PIP_joint",
    "index_swing_joint",
    "index_MCP_joint",
    "index_PIP_joint",
    "index_DIP_joint",
    "mid_swing_joint",
    "mid_MCP_joint",
    "mid_PIP_joint",
    "mid_DIP_joint",
    "ring_swing_joint",
    "ring_MCP_joint",
    "ring_PIP_joint",
    "ring_DIP_joint",
    "pinky_rota_joint",
    "pinky_swing_joint",
    "pinky_MCP_joint",
    "pinky_PIP_joint",
    "pinky_DIP_joint",
)
FOUR_FINGER_SWING_SUFFIXES = frozenset(
    {
        "index_swing_joint",
        "mid_swing_joint",
        "ring_swing_joint",
        "pinky_swing_joint",
    }
)
FOREARM_SUFFIXES = ("tx", "ty", "tz", "pitch", "yaw")


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _motion_file_value(current_ui_value: float, side: str, suffix: str) -> float:
    """Encode current UI/model q using the project's motion-file convention."""
    if side == "right" and suffix in {"thumb_swing_joint", "pinky_rota_joint"}:
        return -current_ui_value
    return current_ui_value


def _adapt_frame(frame: dict[str, Any], side: str) -> tuple[dict[str, Any], int]:
    result: dict[str, Any] = {}
    for timing_key in ("_transition_s", "_interval_s"):
        if timing_key in frame:
            result[timing_key] = frame[timing_key]

    clamp_count = 0
    source_prefix = f"{side}_hand_"
    for suffix in JOINT_SUFFIXES:
        source_key = source_prefix + suffix
        if source_key not in frame:
            raise KeyError(f"missing {source_key}")
        value = float(frame[source_key])
        if suffix in FOUR_FINGER_SWING_SUFFIXES:
            converted = LEGACY_SWING_ZERO_RAD - value
            value = _clamp(converted, CURRENT_SWING_MIN_RAD, CURRENT_SWING_MAX_RAD)
            clamp_count += int(value != converted)
        result["left_hand_" + suffix] = _motion_file_value(value, side, suffix)

    # Form2/H7 currently ignore forearm DOFs, but retaining them prevents data
    # loss and keeps the adapted motion useful to external tooling.
    for suffix in FOREARM_SUFFIXES:
        source_key = f"{side}_hand_forearm_{suffix}"
        if source_key in frame:
            result[source_key] = frame[source_key]
    return result, clamp_count


def _adapt_payload(
    source: dict[str, Any], side: str, backup_name: str
) -> tuple[dict[str, Any], int]:
    output: dict[str, Any] = {
        "_hand_side": side,
        "_meta": {
            "adapted_from": backup_name,
            "legacy_swing_zero_rad": LEGACY_SWING_ZERO_RAD,
            "current_swing_range_rad": [
                CURRENT_SWING_MIN_RAD,
                CURRENT_SWING_MAX_RAD,
            ],
            "forearm_note": "preserved for external tools; current Form2/H7 ignores forearm DOFs",
        },
    }
    clamp_count = 0
    for name, frame in source.items():
        if name.startswith("_") or not isinstance(frame, dict):
            continue
        output[name], frame_clamps = _adapt_frame(frame, side)
        clamp_count += frame_clamps
    output["_meta"]["clamped_swing_samples"] = clamp_count
    return output, clamp_count


def _write_json(path: Path, payload: dict[str, Any]) -> None:
    temporary = path.with_name(path.name + ".tmp")
    with temporary.open("w", encoding="utf-8", newline="\n") as stream:
        json.dump(payload, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    temporary.replace(path)


def adapt(path: Path) -> tuple[Path, Path, dict[str, int]]:
    path = path.resolve()
    right_path = path.with_name(f"{path.stem}_right{path.suffix}")
    backup_path = path.with_name(f"{path.stem}_legacy_urdf_backup{path.suffix}")
    if backup_path.exists():
        raise FileExistsError(f"backup already exists: {backup_path}")
    if right_path.exists():
        raise FileExistsError(f"right-hand output already exists: {right_path}")

    with path.open("r", encoding="utf-8-sig") as stream:
        source = json.load(stream)
    if not isinstance(source, dict):
        raise TypeError("legacy motion root must be a JSON object")
    if "_hand_side" in source or "hand_side" in source:
        raise ValueError("input is already marked as a current single-hand motion")

    shutil.copy2(path, backup_path)
    left, left_clamps = _adapt_payload(source, "left", backup_path.name)
    right, right_clamps = _adapt_payload(source, "right", backup_path.name)
    _write_json(path, left)
    _write_json(right_path, right)
    return path, right_path, {"left": left_clamps, "right": right_clamps}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("motion_file", type=Path)
    args = parser.parse_args()
    left_path, right_path, clamps = adapt(args.motion_file)
    print(f"left : {left_path}")
    print(f"right: {right_path}")
    print(f"clamped swing samples: left={clamps['left']}, right={clamps['right']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
