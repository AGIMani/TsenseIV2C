"""A3.2 SDK events in the existing hand-control-session/v2 viewer format.

The SDK exposes decoded latest telemetry, not every received wire packet.
Never invent wire bytes, acknowledgements, or measurements for offline motors.
JSON and disk writes use the project's bounded background recorder.
"""
from __future__ import annotations

import copy
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import time
from collections.abc import Mapping

from utils.control_session_recorder import ControlSessionRecorder
from a32_kinematics.joint_coordinates import COORDINATE_VERSION, SWING_AXIS_ALIGNMENT

TELEMETRY_FORMAT = "a32-native-sdk-state/v2"


def _current_unit(state, channel, row=None):
    """Keep old captures in their original units; native SDK 1.1.15 supplies A."""
    return ((row or {}).get("current_unit") or
            state.get("telemetry_units", {}).get("external_current" if channel == "mit" else "self_current") or
            ("A" if channel == "mit" or state.get("telemetry_format") == TELEMETRY_FORMAT else "SDK native"))


def default_channels(hand_side="left"):
    import a32_driver
    path = Path(a32_driver.__file__).resolve().parent / "a32_hardware.json"
    channels = json.loads(path.read_text(encoding="utf-8"))["channels"]
    return [{**c, "joint": c["joint"].replace("left_hand_", f"{hand_side}_hand_")} for c in channels]


def _ordered(channels):
    return sorted(channels, key=lambda c: (c["channel"] != "self", c["index"]))


def recording_metadata(model_id, hand_side, channels, **extra):
    ordered = _ordered(channels)
    return {"platform": "motion_play", "transport": "a32_native_sdk", **extra,
            "model_id": model_id, "hand_side": hand_side,
            "joint_coordinate_version": COORDINATE_VERSION,
            "swing_axis_alignment": SWING_AXIS_ALIGNMENT,
            "hardware_location": "local Python service / A3 SDK DLL",
            "recording_location": "local service filesystem",
            "sampling": "every submitted host command; distinct SDK latest-state snapshots, not every wire packet",
            "telemetry_format": TELEMETRY_FORMAT,
            "telemetry_units": {"self_position": "mm or deg by slot", "self_current": "A",
                                "external_current": "A", "external_velocity": "output rpm"},
            "current_unit_source": "per-motor current_unit; legacy captures retain SDK native values",
            "homing_current_unit": "counts",
            "feedback_position_convention": "linear feedback = -SDK pos_act / pos_ref; rotary feedback unchanged",
            "linear_feedback_sign": -1,
            "slot_maps": {kind: {c["joint"]: i for i, c in enumerate(
                c for c in ordered if (c["unit"] == "mm") == (kind == "linear"))}
                for kind in ("linear", "rotary")},
            "hardware_channels": copy.deepcopy(ordered)}


def _finite(value):
    return value if isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) else None


def _safe(value):
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, Mapping):
        return {key: _safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_safe(item) for item in value]
    return value


def telemetry_event(state, channels):
    """Normalize an observed SDK cache; retain raw values for investigation."""
    linear, rotary, issues = [], [], []
    age = _finite(state.get("telemetry_age_ms"))
    fresh = bool(state.get("connected") and age is not None and 0 <= age <= 500)
    for channel in _ordered(channels):
        group, index = channel["channel"], channel["index"]
        rows = state.get(group) or []
        raw = rows[index] if index < len(rows) else {}
        joint = channel["joint"]
        suffix = joint.removeprefix("left_hand_").removeprefix("right_hand_")
        usable = bool(fresh and raw.get("online") and (group != "mit" or raw.get("valid")))
        position = _finite(raw.get("position")) if usable else None
        target = _finite(raw.get("target")) if usable else None
        current = _finite(raw.get("iq")) if usable else None
        centi = _finite(raw.get("temperature_centi"))
        temperature = centi / 100 if usable and centi is not None and centi != 32767 else None
        row = {**copy.deepcopy(raw), "slot": index, "joint": joint, "joint_suffix": suffix,
               "channel": group, "position_unit": channel["unit"], "current_unit": _current_unit(state, group, raw),
               "online": bool(raw.get("online")),
               "measurement_valid": usable and position is not None,
               "pos_ref_mm": target, "iq": current, "iq_ref": _finite(raw.get("iq_ref")) if usable else None,
               "mcu_temp_c": temperature, "home_state": raw.get("homing")}
        if channel["unit"] == "mm":
            row.update(pos_act_mm=-position if position is not None else None,
                       pos_ref_mm=-target if target is not None else None,
                       iq_meas=current, mech_home_state=raw.get("homing"))
            linear.append(row)
        else:
            row.update(pos_actual_mm=position, home_label=raw.get("homing"),
                       velocity=_finite(raw.get("velocity_rpm")) if usable else None,
                       vbus_v=_finite(raw.get("voltage")) if usable else None)
            rotary.append(row)
        if usable and position is None:
            issues.append({"slot": index, "joint_suffix": suffix, "field": "position", "reason": "nonfinite_sdk_value"})
    return {"telemetry_format": TELEMETRY_FORMAT, "linear_feedback_sign": -1,
            "telemetry_units": {"self_current": _current_unit(state, "self"), "external_current": _current_unit(state, "mit")},
            "telemetry_seq": state.get("sequence"),
            "tick_ms": state.get("tick_ms"), "rx_ok": state.get("rx_ok"), "rx_mask": state.get("rx_mask"),
            "telemetry_age_ms": state.get("telemetry_age_ms"), "accepted": True,
            "source": "a32_sdk_latest_snapshot", "sampled": True,
            "linear": linear, "rotary": rotary, "decode_issues": issues,
            "sdk_state": copy.deepcopy(state)}


def _has_telemetry(state):
    age = _finite(state.get("telemetry_age_ms"))
    return bool((state.get("self") or state.get("mit")) and age is not None and age >= 0)


def _telemetry_key(state):
    return (state.get("sequence"), state.get("tick_ms"), state.get("rx_ok"))


class A32SessionRecorder:
    def __init__(self, output_dir=None, *, model_id="TsenseI_v3_2", hand_side="left", channels=None):
        self.output_dir = Path(output_dir) if output_dir is not None else None
        self.model_id, self.hand_side = model_id, hand_side
        self.channels = copy.deepcopy(channels if channels is not None else default_channels(hand_side))
        self.writer = ControlSessionRecorder()
        self._last_telemetry = None
        self._last_host_command = None
        self._status_key = None
        self._status_at = 0.

    def start(self, metadata=None):
        if self.writer.active:
            return self.writer.path
        self._last_telemetry = self._last_host_command = self._status_key = None
        self._status_at = 0.
        return self.writer.start(output_dir=self.output_dir,
            metadata=recording_metadata(self.model_id, self.hand_side, self.channels, **(metadata or {})))

    def _record(self, event, **details):
        return self.writer._record(event, model_id=self.model_id, hand_side=self.hand_side, **details)

    def record_event(self, event, **details):
        """Record platform stages with the same source timestamps and schema."""
        if not self.writer.active:
            return
        if event in {"retarget", "direct", "ruckig", "mink", "sdk"}:
            context = {k: copy.deepcopy(v) for k, v in details.items() if k != "joints"}
            if event == "retarget":
                self._record(event, **context)
            return self.writer.record_joint_angles(event, details.get("joints"),
                frame_id=details.get("frame_id"), hand_side=self.hand_side, model_id=self.model_id,
                sent=details.get("sent") if event == "sdk" else None)
        if event in {"control_target", "filter_candidate", "direct_filter", "direct_command"}:
            return self._record(event, **{k: copy.deepcopy(v) for k, v in details.items() if k != "joints"})
        return self._record(event, **copy.deepcopy(details))

    def record_action(self, action, state=None, **details):
        self._record("sdk_action", action=action, **copy.deepcopy(details))
        if state is not None:
            self.record_state(state, force=True)

    def record_error(self, error, **details):
        self._record("sdk_error", error=str(error), **copy.deepcopy(details))

    def record_state(self, state, *, force=False):
        if not self.writer.active:
            return
        now = time.monotonic()
        status_key = tuple(state.get(k) for k in ("connected", "enabled", "homing", "last_error", "command_ready"))
        age = _finite(state.get("telemetry_age_ms"))
        status_key += (age is not None and 0 <= age <= 500,)
        # Keep idle/disconnected evidence and stale ages without labelling it a
        # newly received telemetry frame. Polling a cache is explicitly sampled.
        if force or status_key != self._status_key or now - self._status_at >= 1:
            self._record("sdk_status", sdk_state=copy.deepcopy(state))
            self._status_key, self._status_at = status_key, now
        key = _telemetry_key(state)
        if _has_telemetry(state) and key != self._last_telemetry:
            self._record("telemetry", **telemetry_event(state, self.channels),
                         correlation="latest_successful_host_write_before_observation",
                         latest_host_command=copy.deepcopy(self._last_host_command))
            self._last_telemetry = key
            board = state.get("board") or {}
            if board.get("valid"):
                def scaled(key, divisor, sentinel):
                    value = _finite(board.get(key))
                    return value / divisor if value is not None and value != sentinel else None
                self._record("board_telemetry", telemetry_seq=state.get("sequence"),
                             ntc_c=scaled("ntc_c_centi", 100, 32767), ntc_b=scaled("ntc_b_centi", 100, 32767),
                             vin_v=scaled("vin24_mv", 1000, 65535), sdk_board=copy.deepcopy(board))

    def record_command(self, targets, *, pose=None, sent=True, error=None, control_dt_s=None,
                       planner_dt_s=None, source="a32_sdk", frame_id=None, control_timing=None):
        if not self.writer.active:
            return
        targets = targets or {}
        pose = copy.deepcopy(pose if pose is not None else targets.get("applied_pose", targets.get("joint_angles_rad", {})))
        applied_pose = copy.deepcopy(targets.get("applied_pose", pose))
        linear, rotary = [], []
        for channel in _ordered(self.channels):
            values = targets.get("self" if channel["channel"] == "self" else "mit_pos") or []
            value = _finite(values[channel["index"]]) if channel["index"] < len(values) else None
            (linear if channel["unit"] == "mm" else rotary).append(value)
        command_id = self._record("conversion", joints_hw=applied_pose, linear_mm=linear, rotary_deg=rotary, source=source, frame_id=frame_id)
        seq = self._record("downlink", command_id=command_id, linear_mm=linear, rotary_deg=rotary,
                           sent=bool(sent), error=str(error) if error is not None else None,
                           source=source, sdk_targets=copy.deepcopy(targets), frame_id=frame_id,
                           write_semantics="SDK sendOnce returned successfully" if sent else "SDK write failed or not completed")
        self._record("h7", sent=bool(sent), sdk_connected=bool(sent),
                     control_dt_s=control_dt_s, control_period_ms=control_dt_s * 1000 if control_dt_s is not None else None,
                     planner_dt_s=planner_dt_s, control_timing=copy.deepcopy(control_timing),
                     command_id=command_id, source=source, frame_id=frame_id)
        if sent:
            self.writer.record_joint_angles("sdk", applied_pose, frame_id=frame_id,
                model_id=self.model_id, hand_side=self.hand_side, sent=True)
        if sent and seq is not None:
            self._last_host_command = {"downlink_seq": seq, "command_id": command_id}

    def snapshot(self):
        path = self.writer.path
        files = ([path] + sorted(path.parent.glob(path.stem + "_part*.jsonl"))) if path is not None else []
        return {"active": self.writer.active, "path": str(path.resolve()) if path else None,
                "filename": path.name if path else None, "error": self.writer.error,
                "files": [str(item.resolve()) for item in files if item.is_file()]}

    def stop(self, reason="disconnected"):
        return self.writer.stop(reason=reason)

    def read_download(self, name=None):
        """Read only this recorder's files; an active export is a flushed prefix.

        Call on a request thread, never the control thread. The writer flushes
        at most every 0.5 seconds; export does not stop or move the hand.
        Returns ``(filename, bytes)`` containing complete JSONL rows only.
        """
        status = self.snapshot()
        name = name or status["filename"]
        allowed = {Path(path).name: Path(path) for path in status["files"]}
        if not isinstance(name, str) or name not in allowed:
            raise ValueError("A3.2 记录文件不存在或不属于当前会话")
        with allowed[name].open("rb") as stream:
            size = stream.seek(0, 2)
            stream.seek(0)
            data = stream.read(size)
        return name, data[:data.rfind(b"\n") + 1]


def export_state_capture(samples, output_dir, *, model_id="TsenseI_v3_2", hand_side="left", channels=None, metadata=None):
    """Convert read-only HTTP samples to a viewer log, without replaying them.

    Items accept ``{t, state}`` (full platform state or device state), ``{t,
    device}``, or a plain device dictionary. No command events are fabricated.
    """
    channels = channels if channels is not None else default_channels(hand_side)
    directory = Path(output_dir)
    directory.mkdir(parents=True, exist_ok=True)
    now = datetime.now(timezone.utc).astimezone()
    path = directory / f"hand_control_{now.strftime('%Y%m%d_%H%M%S_%f')}_sdk_capture.jsonl"
    meta = recording_metadata(model_id, hand_side, channels, **(metadata or {}))
    meta.update(read_only_capture=True, sampling="read-only HTTP state samples; no motor commands issued")
    started_at = meta.pop("started_at", now.isoformat())
    started_ms = datetime.fromisoformat(started_at.replace("Z", "+00:00")).timestamp() * 1000
    meta["time_fields"] = {"t": "original capture elapsed seconds", "wall_time_unix_ms": "original sample at; otherwise started_at + t",
                           "wall_time": "ISO 8601 with time zone", "tick_ms": "device clock, when present"}
    last_telemetry, seq, last_t, ended_at = None, 0, 0., None
    with path.open("x", encoding="utf-8", newline="\n") as stream:
        def emit(event, t, at=None, **details):
            nonlocal seq
            seq += 1
            wall_ms = datetime.fromisoformat(at.replace("Z", "+00:00")).timestamp() * 1000 if isinstance(at, str) else started_ms + t * 1000
            wall_iso = datetime.fromtimestamp(wall_ms / 1000, timezone.utc).astimezone().isoformat(timespec="milliseconds")
            stream.write(json.dumps(_safe({**details, "type": event, "seq": seq, "t": t,
                                          "wall_time_unix_ms": wall_ms, "wall_time": wall_iso,
                                          "wall_time_source": "captured" if at is not None else "derived_from_session_start"}),
                                    ensure_ascii=False, allow_nan=False, separators=(",", ":")) + "\n")
        emit("session", 0., schema="hand-control-session/v2", session_id=path.stem,
             started_at=started_at, at=started_at, metadata=meta,
             joint_unit="rad", units={"linear": "mm", "rotary": "deg", "temperature": "degC"})
        for sample in samples:
            if sample.get("type") in ("diagnostic_session", "diagnostic_session_end", "diagnostic_end", "session", "session_end"):
                continue
            t = _finite(sample.get("t", sample.get("elapsed_s")))
            if t is None:
                elapsed = _finite(sample.get("elapsed_ms"))
                t = elapsed / 1000 if elapsed is not None else last_t
            last_t = max(last_t, t)
            if isinstance(sample.get("at"), str):
                ended_at = sample["at"]
            state = sample.get("state", sample)
            state = state.get("device", state) if isinstance(state, Mapping) else {}
            emit("sdk_status", t, at=sample.get("at"), model_id=model_id, hand_side=hand_side, sdk_state=state)
            key = _telemetry_key(state)
            if _has_telemetry(state) and key != last_telemetry:
                emit("telemetry", t, at=sample.get("at"), model_id=model_id, hand_side=hand_side, **telemetry_event(state, channels))
                last_telemetry = key
        ended_at = ended_at or datetime.fromtimestamp((started_ms + last_t * 1000) / 1000, timezone.utc).astimezone().isoformat(timespec="milliseconds")
        emit("session_end", last_t, at=ended_at, reason="read_only_capture_complete", duration_s=last_t,
             ended_at=ended_at, dropped_events=0)
    return path.resolve()
