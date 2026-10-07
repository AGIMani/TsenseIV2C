#!/usr/bin/env python3
"""Offline replay for light_test JSONL recordings (never opens a serial port).

The replay boundary is ``wuji_points_21_m``.  From there this module executes
the same TopoRetarget -> collision policy -> Ruckig -> hardware-angle ->
linkage stroke -> H7 frame packing path used by the live window.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Iterable

import numpy as np


HERE = Path(__file__).resolve().parent
PROJECT_ROOT = HERE.parents[2]
PROJECT_SRC = PROJECT_ROOT / "src"
for path in (HERE, PROJECT_SRC):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from collision_guard import (  # noqa: E402
    COLLISION_STRATEGY_AVOID,
    COLLISION_STRATEGY_STOP,
    SelfCollisionGuard,
)
from hand_gate import (  # noqa: E402
    expected_label_for_selected_hand,
    hand_selection_matches,
)
from tracking_transition import TrackingTransition  # noqa: E402
from pipeline_geometry import (  # noqa: E402
    landmarks_ok_for_retarget,
    prepare_topo_points,
)
from protocol import convert_ui_to_hardware  # noqa: E402
from usb.offline import (  # noqa: E402
    DEV_TELEM,
    FUN_HAND_POS_BATCH,
    build_downlink,
    joint_angles_to_hand_positions,
    pack_hand_pos_batch,
    verify_hand_pos_frame,
)
from utils.downlink_guard import DownlinkGuard  # noqa: E402
from utils.retarget_runtime import (  # noqa: E402
    reset_retarget_state,
    retarget_hand,
    set_retarget_hand_side,
)
from utils.ruckig_interpolator import (  # noqa: E402
    PhaseSynchronizedRuckigInterpolator,
)


def load_recording(path: str | Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    """Read a recoverable JSONL session, ignoring an incomplete final line."""
    source = Path(path).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(source)
    metadata: dict[str, Any] = {}
    frames: list[dict[str, Any]] = []
    with source.open("r", encoding="utf-8") as stream:
        for line_number, line in enumerate(stream, start=1):
            text = line.strip()
            if not text:
                continue
            try:
                row = json.loads(text)
            except json.JSONDecodeError:
                # JSONL is deliberately crash-recoverable.  Only a truncated
                # last record is tolerated; corruption in the middle is not.
                if not any(rest.strip() for rest in stream):
                    break
                raise ValueError(f"invalid JSON at line {line_number}: {source}")
            if row.get("type") == "session_start" and not metadata:
                metadata = dict(row)
            elif row.get("type") == "frame":
                frames.append(dict(row))
    if not metadata:
        raise ValueError(f"recording has no session_start record: {source}")
    if not frames:
        raise ValueError(f"recording has no frame records: {source}")
    return metadata, frames


def replay_points(frame: dict[str, Any]) -> np.ndarray | None:
    """Extract and validate the stable MediaPipe/Wuji replay boundary."""
    value = frame.get("wuji_points_21_m")
    if value is None:
        return None
    points = np.asarray(value, dtype=np.float64)
    if points.shape != (21, 3) or not np.isfinite(points).all():
        return None
    return points


def build_verified_h7_frame(
    sequence: int,
    linear_mm: Iterable[float],
    rotary_deg: Iterable[float],
) -> bytes:
    """Pack the exact H7 0x20 frame and fail if structure/CRC is invalid."""
    payload = pack_hand_pos_batch(
        int(sequence),
        list(linear_mm),
        list(rotary_deg),
    )
    frame = build_downlink(DEV_TELEM, FUN_HAND_POS_BATCH, payload)
    if not verify_hand_pos_frame(frame):
        raise RuntimeError("generated H7 hand-position frame failed CRC/shape check")
    return frame


def _pinky_value(pose: dict[str, float]) -> float:
    for name, value in pose.items():
        if str(name).endswith("_pinky_rota_joint"):
            return float(value)
    return 0.0


def _frame_ticks(
    frame: dict[str, Any],
    following: dict[str, Any] | None,
    *,
    tick_hz: float,
    fallback_fps: float,
    max_gap_s: float,
) -> int:
    current_s = frame.get("session_elapsed_s")
    following_s = None if following is None else following.get("session_elapsed_s")
    if isinstance(current_s, (int, float)) and isinstance(following_s, (int, float)):
        delta_s = max(0.0, min(float(following_s) - float(current_s), max_gap_s))
    else:
        delta_s = 1.0 / max(float(fallback_fps), 1.0)
    return max(1, int(round(delta_s * tick_hz)))


def replay_recording(
    path: str | Path,
    *,
    collision_mode: str | None = None,
    max_frames: int | None = None,
    tick_hz: float = 500.0,
    output_path: str | Path | None = None,
) -> dict[str, Any]:
    """Run a recorded MediaPipe session through the complete offline chain."""
    source = Path(path).expanduser().resolve()
    metadata, all_frames = load_recording(source)
    frames = all_frames[: max(0, int(max_frames))] if max_frames is not None else all_frames
    if not frames:
        raise ValueError("max_frames selected zero frames")

    side = "right" if str(metadata.get("hand_side", "right")).lower().startswith("r") else "left"
    selected_collision = str(
        collision_mode
        if collision_mode is not None
        else metadata.get("self_collision_mode", COLLISION_STRATEGY_AVOID)
    ).lower()
    if selected_collision not in {
        "off",
        COLLISION_STRATEGY_AVOID,
        COLLISION_STRATEGY_STOP,
    }:
        raise ValueError(f"unsupported collision mode: {selected_collision}")

    fallback_fps = float(metadata.get("requested_camera_fps", 30.0) or 30.0)
    target_guard = (
        None
        if selected_collision == "off"
        else SelfCollisionGuard(strategy=selected_collision)
    )
    sample_guard = (
        None
        if selected_collision == "off"
        else SelfCollisionGuard(
            strategy=COLLISION_STRATEGY_STOP,
            path_step_rad=0.04,
            bisection_iterations=8,
        )
    )
    interpolator = PhaseSynchronizedRuckigInterpolator()
    tracking = TrackingTransition(interpolator)
    downlink_guard = DownlinkGuard()
    previous_retarget: dict[str, float] | None = None
    current_pose: dict[str, float] | None = None
    control_gate_open = False
    simulated_time = 0.0
    output_stream = None

    summary: dict[str, Any] = {
        "input": str(source),
        "schema_version": metadata.get("schema_version", 1),
        "recording_mode": metadata.get("recording_mode", "legacy_full"),
        "hand_side": side,
        "handedness_mapping": "direct_physical_label",
        "collision_mode": selected_collision,
        "available_frames": len(all_frames),
        "selected_frames": len(frames),
        "frames_with_replay_points": 0,
        "hand_match_frames": 0,
        "hand_mismatch_blocked_frames": 0,
        "hand_unverified_blocked_frames": 0,
        "retarget_frames": 0,
        "quality_rejected_frames": 0,
        "missing_frames": 0,
        "held_frames": 0,
        "recovery_wait_frames": 0,
        "tracking_phase_counts": {},
        "max_sample_joint_step_rad": 0.0,
        "max_sample_penetration_mm": 0.0,
        "target_collision_frames": 0,
        "target_avoidance_frames": 0,
        "target_stop_fallback_frames": 0,
        "avoidance_phase_counts": {},
        "planning_result_counts": {},
        "avoidance_backend_counts": {},
        "sample_collision_ticks": 0,
        "ruckig_ticks": 0,
        "h7_frames_verified": 0,
        "h7_frame_bytes": 0,
        "pinky_rota_nonzero_frames": 0,
        "retarget_errors": [],
    }

    try:
        if output_path is not None:
            destination = Path(output_path).expanduser().resolve()
            if destination == source:
                raise ValueError("replay output must not overwrite the source recording")
            destination.parent.mkdir(parents=True, exist_ok=True)
            output_stream = destination.open("x", encoding="utf-8", newline="\n")
            summary["output"] = str(destination)

        set_retarget_hand_side(side)
        reset_retarget_state()
        sequence = 0
        for index, row in enumerate(frames):
            points = replay_points(row)
            if points is None:
                summary["missing_frames"] += 1
            else:
                summary["frames_with_replay_points"] += 1

            detected_label = row.get("detected_label")
            # ``expected_label`` and ``used_fallback_hand`` were derived by the
            # live selection policy.  Recompute them so recordings made before
            # the handedness fix exercise the current pipeline rather than
            # permanently reproducing the old inverted gate.
            expected_label = expected_label_for_selected_hand(side)
            hand_match = hand_selection_matches(
                detected_label,
                expected_label,
                used_fallback_hand=False,
            )
            is_new_detection = bool(
                row.get("detected", True)
                and not row.get("held_last_valid", False)
            )
            if row.get("held_last_valid", False):
                summary["held_frames"] += 1
            if is_new_detection:
                if hand_match:
                    summary["hand_match_frames"] += 1
                else:
                    control_gate_open = False
                    tracking.block("hand_mismatch", current_pose)
                    if detected_label and expected_label:
                        summary["hand_mismatch_blocked_frames"] += 1
                    else:
                        summary["hand_unverified_blocked_frames"] += 1

            quality_ok = bool(points is not None and landmarks_ok_for_retarget(points))
            valid_frame = bool(
                points is not None
                and is_new_detection
                and hand_match
                and quality_ok
            )
            if is_new_detection and hand_match and points is not None and not quality_ok:
                summary["quality_rejected_frames"] += 1
            tracking_decision = {"allow_retarget": False}
            if not (is_new_detection and not hand_match):
                tracking_decision = tracking.observe(
                    valid_frame,
                    now=simulated_time,
                    current=current_pose,
                    reason="hand_missing" if not is_new_detection else "invalid_landmarks",
                )
                if valid_frame and not tracking_decision["allow_retarget"]:
                    summary["recovery_wait_frames"] += 1
            should_solve = valid_frame and tracking_decision["allow_retarget"]
            collision_diag: dict[str, Any] = {}
            if should_solve and points is not None:
                if not landmarks_ok_for_retarget(points):
                    summary["quality_rejected_frames"] += 1
                else:
                    try:
                        topo_points = prepare_topo_points(points, side)
                        solved = retarget_hand(
                            topo_points,
                            frame_id=int(row.get("frame_id", index + 1)),
                            previous_joint_values=previous_retarget,
                            hand_side=side,
                        )
                        if not solved:
                            raise RuntimeError("TopoRetarget returned no valid target")
                        if solved:
                            previous_retarget = dict(solved)
                            safe_target = dict(solved)
                            if target_guard is not None:
                                if tracking_decision.get("reset_planner"):
                                    target_guard.rebase(current_pose)
                                safe_target, collision_diag = target_guard.project(
                                    solved,
                                    current_values=current_pose,
                                )
                                if collision_diag.get("limited"):
                                    summary["target_collision_frames"] += 1
                                if collision_diag.get("avoided"):
                                    summary["target_avoidance_frames"] += 1
                                elif collision_diag.get("limited"):
                                    summary["target_stop_fallback_frames"] += 1
                                phase = collision_diag.get("avoidance_phase")
                                if phase:
                                    phase_counts = summary["avoidance_phase_counts"]
                                    phase_counts[str(phase)] = (
                                        int(phase_counts.get(str(phase), 0)) + 1
                                    )
                                planning_result = collision_diag.get(
                                    "planning_result"
                                )
                                if planning_result:
                                    result_counts = summary[
                                        "planning_result_counts"
                                    ]
                                    result_counts[str(planning_result)] = (
                                        int(
                                            result_counts.get(
                                                str(planning_result), 0
                                            )
                                        )
                                        + 1
                                    )
                                backend = collision_diag.get("avoidance_backend")
                                if backend:
                                    backend_counts = summary[
                                        "avoidance_backend_counts"
                                    ]
                                    backend_counts[str(backend)] = (
                                        int(backend_counts.get(str(backend), 0))
                                        + 1
                                    )
                            interpolator.push(safe_target)
                            control_gate_open = True
                            summary["retarget_frames"] += 1
                    except Exception as exc:  # noqa: BLE001
                        tracking.missing("retarget_or_planning_error", current_pose)
                        errors = summary["retarget_errors"]
                        if len(errors) < 20:
                            errors.append(
                                {
                                    "frame_id": row.get("frame_id", index + 1),
                                    "error": f"{type(exc).__name__}: {exc}",
                                }
                            )

            following = frames[index + 1] if index + 1 < len(frames) else None
            ticks = _frame_ticks(
                row,
                following,
                tick_hz=tick_hz,
                fallback_fps=fallback_fps,
                # Preserve camera stalls long enough for watchdog + braking.
                # Capping below the watchdog hid stale-target chasing in replay.
                max_gap_s=2.0,
            )
            last_hardware: dict[str, float] | None = None
            last_linear: list[float] = []
            last_rotary: list[float] = []
            for _ in range(ticks):
                tracking.tick(now=simulated_time, current=current_pose)
                # Match the live hard interlock: a mismatching or unverified
                # new detection revokes the target, so no stale Ruckig pose,
                # angle conversion, stroke conversion, or H7 packing runs.
                if not control_gate_open:
                    simulated_time += 1.0 / tick_hz
                    continue
                pose = interpolator.sample()
                if not pose:
                    simulated_time += 1.0 / tick_hz
                    continue
                expected_jump = interpolator.pop_reset_flag()
                faults = interpolator.last_faults
                if sample_guard is not None:
                    pose, sample_diag = sample_guard.project(pose)
                    if sample_diag.get("limited"):
                        summary["sample_collision_ticks"] += 1
                        interpolator.sync_current_pose(pose)
                    summary["max_sample_penetration_mm"] = max(
                        summary["max_sample_penetration_mm"],
                        float(sample_diag.get("output_max_penetration_mm", 0.0)),
                    )
                if current_pose:
                    summary["max_sample_joint_step_rad"] = max(
                        summary["max_sample_joint_step_rad"],
                        max(abs(pose[name] - current_pose.get(name, pose[name])) for name in pose),
                    )
                current_pose = dict(pose)
                hardware = convert_ui_to_hardware(current_pose, hand_side=side)
                hardware = downlink_guard.filter(
                    hardware,
                    now=simulated_time,
                    expected_jump=expected_jump,
                    ruckig_faults=faults,
                )
                linear, rotary = joint_angles_to_hand_positions(hardware)
                h7 = build_verified_h7_frame(sequence, linear, rotary)
                sequence = (sequence + 1) & 0xFF
                simulated_time += 1.0 / tick_hz
                summary["ruckig_ticks"] += 1
                summary["h7_frames_verified"] += 1
                summary["h7_frame_bytes"] = len(h7)
                if abs(_pinky_value(current_pose)) > 1.0e-12 or abs(
                    _pinky_value(hardware)
                ) > 1.0e-12:
                    summary["pinky_rota_nonzero_frames"] += 1
                last_hardware = dict(hardware)
                last_linear = [float(value) for value in linear]
                last_rotary = [float(value) for value in rotary]

            phase = tracking.phase
            phase_counts = summary["tracking_phase_counts"]
            phase_counts[phase] = phase_counts.get(phase, 0) + 1

            if output_stream is not None:
                output_stream.write(
                    json.dumps(
                        {
                            "type": "replay_frame",
                            "source_frame_id": row.get("frame_id", index + 1),
                            "retarget_joint_angles_rad": previous_retarget,
                            "collision": collision_diag,
                            "tracking_transition": tracking.snapshot(),
                            "ruckig_joint_angles_rad": current_pose,
                            "hardware_joint_angles_rad": last_hardware,
                            "linear_strokes_mm": last_linear,
                            "rotary_positions_deg": last_rotary,
                            "h7_sequence_next": sequence,
                        },
                        ensure_ascii=False,
                        separators=(",", ":"),
                    )
                    + "\n"
                )
    finally:
        if output_stream is not None:
            output_stream.close()
        interpolator.shutdown()
        reset_retarget_state()

    summary["downlink_guard"] = downlink_guard.stats()
    summary["tracking_transition"] = tracking.snapshot()
    summary["ok"] = bool(
        summary["retarget_frames"] > 0
        and summary["h7_frames_verified"] > 0
        and not summary["retarget_errors"]
        and summary["pinky_rota_nonzero_frames"] == 0
        and (
            sample_guard is None
            or summary["max_sample_penetration_mm"]
            <= sample_guard.allowed_penetration_mm + 1.0e-6
        )
    )
    return summary


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recording", help="light_test session JSONL path")
    parser.add_argument(
        "--collision-mode",
        choices=("recorded", "off", "avoid", "stop"),
        default="recorded",
    )
    parser.add_argument("--max-frames", type=int)
    parser.add_argument("--tick-hz", type=float, default=500.0)
    parser.add_argument("--output", help="optional per-frame replay JSONL")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    summary = replay_recording(
        args.recording,
        collision_mode=(
            None if args.collision_mode == "recorded" else args.collision_mode
        ),
        max_frames=args.max_frames,
        tick_hz=max(1.0, float(args.tick_hz)),
        output_path=args.output,
    )
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if summary["ok"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
