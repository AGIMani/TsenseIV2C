"""A3.2 calibrated device API shared by tracking and motion playback.

All public poses are complete 20-joint URDF radians for ``model_id``. The
hardware table is the shared A3.2 slot table, independent of the selected
URDF side and of the legacy A30/V2 protocol.
"""
from __future__ import annotations

import copy
import json
import math
from numbers import Real
from pathlib import Path
import threading
import time
from typing import Mapping

from a32_kinematics import A32StrokeMapper
from .native import A32DriverError, NativeHand
from .limits import sdk_channel_limits


class A32Device:
    """Lazy native transport; no SDK process, automatic connection or enable.

    ``backend`` is injectable for offline tests. Production uses NativeHand.
    Do trajectory planning in joint space before ``send_joint_targets``.
    ``enabled`` is the software permission to send targets, not measured motor
    power. All stop/error/close paths revoke that permission without sending
    motor-disable commands; an already accepted motion or home may continue.
    """

    def __init__(self, hand_side="left", model_id="TsenseI_v3_2", *, backend=None,
                 hardware_config=None, telemetry_timeout_ms=500, home_timeout_s=30.):
        self.hand_side, self.model_id = hand_side, model_id
        self.mapper = A32StrokeMapper(hand_side, model_id=model_id)
        config_path = Path(hardware_config or Path(__file__).with_name("a32_hardware.json"))
        self._config = json.loads(config_path.read_text(encoding="utf-8"))
        self.channels = copy.deepcopy(self._config["channels"])
        for row in self.channels:
            row["joint"] = row["joint"].replace("left_hand_", f"{hand_side}_hand_")
        self._validate_config()
        # Clamp before trajectory planning so the reported applied pose agrees
        # with the SDK, including for older/custom hardware-map files.
        for row in self.channels:
            lo, hi = sdk_channel_limits(row["channel"], row["index"])
            row["limits"] = [max(lo, row["limits"][0]), min(hi, row["limits"][1])]
            if row["limits"][0] > row["limits"][1]:
                raise A32DriverError(f"No SDK/calibration limit intersection: {row['joint']}")
        self._by_joint = {row["joint"]: row for row in self.channels}
        self._joint_limits = self._derive_joint_limits()
        self._backend = backend
        self._lock = threading.RLock()
        self._enabled = False
        self._homing_slots = set()
        self._home_started = 0.0
        self._home_sequence = None
        self._home_reset_sequence = None
        self._home_reset_pending = False
        self._home_watch_stop = threading.Event()
        self._home_thread = None
        self._last_targets = None
        self._seed_missing = []
        self._last_error = ""
        self.telemetry_timeout_ms = int(telemetry_timeout_ms)
        if not 1 <= self.telemetry_timeout_ms <= 5000:
            raise ValueError("telemetry_timeout_ms must be between 1 and 5000")
        self.home_timeout_s = float(home_timeout_s)
        if not math.isfinite(self.home_timeout_s) or not 1 <= self.home_timeout_s <= 120:
            raise ValueError("home_timeout_s must be between 1 and 120")

    def _validate_config(self):
        if self._config.get("schema_version") != 1 or self._config.get("hand_side") not in ("left", "right"):
            raise A32DriverError("A3.2 requires a version 1 hardware map")
        if {c["joint"] for c in self.channels} != set(self.mapper.joint_names) or len(self.channels) != 20:
            raise A32DriverError("A3.2 hardware map must name exactly the model's 20 joints")
        if {(c["channel"], c["index"]) for c in self.channels} != {
            *(("self", i) for i in range(18)), *(("mit", i) for i in range(2))
        }:
            raise A32DriverError("A3.2 hardware map requires distinct 18 self + 2 MIT channels")
        for c in self.channels:
            expected = "deg" if c["channel"] == "mit" or c["index"] in (6, 7, 14, 15) else "mm"
            if c["unit"] != expected or c["direction"] not in (-1, 1):
                raise A32DriverError(f"Invalid channel units/direction: {c['joint']}")
            values = [*c["limits"], c["offset"]]
            if len(c["limits"]) != 2 or not all(math.isfinite(float(v)) for v in values) or c["limits"][0] > c["limits"][1]:
                raise A32DriverError(f"Invalid channel limits: {c['joint']}")

    def _inverse(self, channel, value):
        name = channel["joint"]
        source = (float(value) - channel["offset"]) / channel["direction"]
        return self.mapper.inverse_actuator(name, source)

    def _derive_joint_limits(self):
        result = {}
        # Intersect source geometry outputs with physical limits, then invert.
        low_pose = {c["joint"]: c["limits_rad"][0] for c in self.mapper.description()["channels"]}
        high_pose = {c["joint"]: c["limits_rad"][1] for c in self.mapper.description()["channels"]}
        endpoints = [self.mapper.convert(p) for p in (low_pose, high_pose)]
        for c in self.channels:
            name = c["joint"]
            source_values = [(p.linear_strokes_mm if c["unit"] == "mm" else p.rotary_positions_deg)[name]
                             * c["direction"] + c["offset"] for p in endpoints]
            lo = max(min(source_values), c["limits"][0])
            hi = min(max(source_values), c["limits"][1])
            if lo > hi + 1e-6:
                raise A32DriverError(f"No physical/geometry limit intersection: {name}")
            bounds = sorted((self._inverse(c, lo), self._inverse(c, hi)))
            # LUT interpolation and loop inversion have small endpoint errors.
            # Keep the authored boundary usable by strict initial-pose checks.
            if abs(bounds[0] - low_pose[name]) < 1e-7:
                bounds[0] = low_pose[name]
            if abs(bounds[1] - high_pose[name]) < 1e-7:
                bounds[1] = high_pose[name]
            result[name] = (max(bounds[0], low_pose[name]), min(bounds[1], high_pose[name]))
        return result

    def joint_limits(self):
        """Effective hardware + geometry limits in the selected URDF's radians."""
        return dict(self._joint_limits)

    def constrain_pose(self, pose):
        """Validate complete finite input and clamp in joint space before planning."""
        if not isinstance(pose, Mapping) or pose.keys() != self._joint_limits.keys():
            raise ValueError("A3.2 requires its own complete 20-joint pose")
        result = {}
        for name, (lo, hi) in self._joint_limits.items():
            value = pose[name]
            if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(float(value)):
                raise ValueError(f"A3.2 requires finite joint radians: {name}")
            result[name] = min(hi, max(lo, float(value)))
        return result

    def _transport(self):
        if self._backend is None:
            self._backend = NativeHand()
        return self._backend

    def _permit_hardware(self):
        if not self._config.get("calibration_confirmed"):
            raise A32DriverError("A3.2 hardware calibration is not confirmed")

    def connect_usb(self, port="auto"):
        with self._lock:
            self._permit_hardware()
            if not isinstance(port, str) or not port.strip():
                raise ValueError("A3.2 USB port must be a port name or auto")
            if self.is_connected:
                raise A32DriverError("A3.2 is already connected; disconnect before changing transport")
            self._transport().connect_usb(port)
            self._enabled, self._last_error = False, ""
            self._homing_slots.clear()
            self._last_targets = None
            self._seed_missing = []
            return self.get_state()

    def connect_eth(self, host, port=5001):
        with self._lock:
            self._permit_hardware()
            if not isinstance(host, str) or not host.strip() or isinstance(port, bool) or not 1 <= int(port) <= 65535:
                raise ValueError("A3.2 ETH requires a host and TCP port 1..65535")
            if self.is_connected:
                raise A32DriverError("A3.2 is already connected; disconnect before changing transport")
            self._transport().connect_eth(host.strip(), int(port))
            self._enabled, self._last_error = False, ""
            self._homing_slots.clear()
            self._last_targets = None
            self._seed_missing = []
            return self.get_state()

    @property
    def is_connected(self):
        with self._lock:
            return bool(self._backend is not None and self._backend.get_state().get("connected"))

    def get_state(self):
        with self._lock:
            if self._backend is None:
                raw = {"connected": False, "streaming": False, "self": [], "mit": [],
                       "telemetry_age_ms": -1, "rx_hz": 0., "tx_hz": 0., "rx_mask": 0,
                       "firmware": {}, "last_error": ""}
            else:
                raw = copy.deepcopy(self._backend.get_state())
            if not raw.get("connected"):
                self._enabled = False
                self._homing_slots.clear()
                self._home_reset_pending = False
            age = raw.get("telemetry_age_ms", -1)
            fresh = isinstance(age, Real) and 0 <= age <= self.telemetry_timeout_ms
            online = (len(raw.get("self", [])) == 18 and len(raw.get("mit", [])) == 2 and
                      all(c.get("online") and c.get("position") is not None and
                          math.isfinite(c["position"]) for c in raw["self"]) and
                      all(c.get("online") and c.get("valid") and not c.get("fault") and
                          c.get("position") is not None and math.isfinite(c["position"]) for c in raw["mit"]))
            homed = online and all(c.get("homing") in (2, 254) for c in raw["self"])
            if self._home_reset_pending and fresh and online:
                sequence = raw.get("sequence", raw.get("rx_ok"))
                if sequence is not None and sequence != self._home_reset_sequence:
                    self._home_reset_pending = False
            home_error = ""
            if self._homing_slots:
                elapsed = time.monotonic() - self._home_started
                sequence = raw.get("sequence", raw.get("rx_ok"))
                new_frame = sequence is not None and sequence != self._home_sequence
                if elapsed > self.home_timeout_s:
                    home_error = "A3.2 homing timed out; command sending stopped (motor power unchanged)"
                elif elapsed > self.telemetry_timeout_ms / 1000. and (not fresh or not online):
                    home_error = "A3.2 homing feedback became stale/offline; command sending stopped (motor power unchanged)"
                elif len(raw.get("self", [])) == 18 and any(
                        raw["self"][i].get("homing") == 3 for i in self._homing_slots):
                    home_error = "A3.2 motor reported homing failure; command sending stopped (motor power unchanged)"
                elif elapsed >= .3 and fresh and online and new_frame and all(
                        raw["self"][i].get("homing") in (2, 254) for i in self._homing_slots):
                    self._homing_slots.clear()
                if home_error:
                    self._stop_commands()
                    self._last_error = home_error
            homing = not home_error and (bool(self._homing_slots) or
                (fresh and not self._home_reset_pending and any(
                    c.get("online") and c.get("homing") in (1, 4) for c in raw.get("self", []))))
            raw.update(model_id=self.model_id, hand_side=self.hand_side, enabled=self._enabled,
                       ready=bool(raw.get("connected") and fresh and homed and not homing and not home_error),
                       command_ready=bool(raw.get("connected") and not homing and not home_error),
                       seed_missing=list(self._seed_missing),
                       feedback_valid=bool(raw.get("connected") and fresh and online), homing=homing,
                       homing_active=homing,
                       home_reset_pending=self._home_reset_pending,
                       last_error=self._last_error or raw.get("last_error", ""))
            for c in self.channels:
                if len(raw.get(c["channel"], [])) > c["index"]:
                    row = raw[c["channel"]][c["index"]]
                    row.update(joint=c["joint"], unit=c["unit"], limits=list(c["limits"]))
                    if c["channel"] == "self":
                        slot = c["index"]
                        row.update(bus=slot // 8 + 1, node=slot % 8 + 1,
                                   can_node=5 if slot == 17 else slot % 8 + 1,
                                   homing_label={0: "WAIT", 1: "SEEK", 2: "DONE", 3: "ERR", 4: "RELEASE", 254: "THETA（电角度模式）"}.get(row.get("homing"), "未知"))
            mask = int(raw.get("rx_mask", 0)) & ((1 << 18) - 1)
            raw["telemetry_diagnostics"] = {
                "self_online_mask": mask, "self_online_mask_hex": f"0x{mask:05X}",
                "self_offline_slots": [i for i in range(18) if not mask & (1 << i)],
                "fresh": bool(fresh), "protocol": "A3 SDK FUN 0x02 (18 self + 2 MIT)",
                "online_source": "firmware self mask / MIT status bits; not inferred from position",
            }
            return raw

    status = get_state

    def get_control_state(self):
        """Fresh minimal native status for tracking; homing keeps full checks."""
        with self._lock:
            if (self._backend is None or self._homing_slots or self._home_reset_pending
                    or not hasattr(self._backend, 'get_control_state')
                    or not getattr(self._backend, 'fast_control', True)):
                return self.get_state()
            state = self._backend.get_control_state()
            if not state.get('connected'):
                self._enabled = False
            return {**state, 'hand_side': self.hand_side, 'enabled': self._enabled,
                    'homing': bool(state.get('homing_active'))}

    def _feedback(self, require_ready=True):
        state = self.get_state()
        if not state.get("ready" if require_ready else "feedback_valid"):
            raise A32DriverError("A3.2 needs fresh online telemetry for all 18 self and 2 MIT channels" +
                                 (" and completed self homing (DONE/THETA)" if require_ready else ""))
        return state

    @staticmethod
    def feedback_position(channel, value):
        """Raw SDK position -> command coordinate; linear feedback is reversed.

        Keep native telemetry unchanged and apply this only before feedback
        inversion. Rotary channels already use the command coordinate.
        """
        return -value if channel["unit"] == "mm" else value

    def feedback_pose(self):
        with self._lock:
            state = self._feedback()
            pose = {}
            for c in self.channels:
                try:
                    value = state[c["channel"]][c["index"]]["position"]
                    q = self._inverse(c, self.feedback_position(c, value))
                except (ValueError, ArithmeticError) as exc:
                    raise A32DriverError(f"Cannot invert A3.2 feedback for {c['joint']}: {exc}") from exc
                lo, hi = self._joint_limits[c["joint"]]
                if q < lo - .002 or q > hi + .002:
                    raise A32DriverError(f"A3.2 feedback outside calibrated limits: {c['joint']}")
                pose[c["joint"]] = min(hi, max(lo, q))
            return pose

    def starting_pose(self, fallback_pose=None):
        """Use valid telemetry where available, otherwise a bounded fallback.

        Missing/invalid feedback is diagnostic only, not an arming gate.
        Callers should pass their current software pose as the fallback.
        """
        with self._lock:
            pose = {}
            fallback = self.constrain_pose(fallback_pose if fallback_pose is not None else
                                           {n: 0. for n in self._joint_limits})
            state = self.get_state()
            age = state.get("telemetry_age_ms", -1)
            fresh = (state.get("connected") and isinstance(age, Real)
                     and not isinstance(age, bool) and 0 <= age <= self.telemetry_timeout_ms)
            missing = []
            for c in self.channels:
                rows = state.get(c["channel"], [])
                row = rows[c["index"]] if len(rows) > c["index"] else {}
                value = row.get("position")
                usable = (fresh and row.get("online") and isinstance(value, Real)
                          and not isinstance(value, bool) and math.isfinite(value)
                          and (c["channel"] != "mit" or (row.get("valid") and not row.get("fault"))))
                if usable:
                    try:
                        value = self.feedback_position(c, value)
                        tolerance = .1 if c["unit"] == "mm" else math.degrees(.002)
                        lower, upper = c["limits"]
                        if not lower - tolerance <= value <= upper + tolerance:
                            raise ValueError("feedback outside hardware limits")
                        q = self._inverse(c, min(upper, max(lower, value)))
                        lo, hi = self._joint_limits[c["joint"]]
                        if math.isfinite(q):
                            pose[c["joint"]] = min(hi, max(lo, q))
                            continue
                    except (ValueError, ArithmeticError):
                        pass
                missing.append(c["joint"])
            self._seed_missing = missing
            for name in missing:
                pose[name] = fallback[name]
            return pose

    def enable_all(self, *, start_pose=None, fallback_pose=None, send_initial_pose=True):
        with self._lock:
            self._permit_hardware()
            state = self.get_state()
            if not state.get("connected"):
                raise A32DriverError("Connect A3.2 before enabling motion")
            if state["homing"]:
                raise A32DriverError("Wait for A3.2 homing to complete before enabling motion")
            # An explicit start_pose lets the planner and SDK use the same
            # collision-checked seed, even if telemetry changes between calls.
            targets = None
            if send_initial_pose:
                seed = start_pose if start_pose is not None else self.starting_pose(fallback_pose)
                targets = self.prepare_targets(seed, clamp=False)
            self._enabled = False
            try:
                if send_initial_pose:
                    self._transport().set_targets(targets["self"], targets["mit_pos"])
                    self._transport().send_once()
                self._transport().enable_all(True)
            except Exception as exc:
                self._last_error = str(exc)
                self._stop_commands()
                raise
            if send_initial_pose:
                self._last_targets = targets
            self._enabled = True
            self._last_error = ""
            return self.get_state()

    def disable_all(self):
        """Compatibility name: stop host commands without disabling motors."""
        return self.stop()

    def home(self, slot=None, iq_counts=80.):
        with self._lock:
            if isinstance(iq_counts, bool) or not isinstance(iq_counts, Real) or not math.isfinite(iq_counts) or not 0 < iq_counts <= 1000:
                raise ValueError("A3.2 homing iq_counts must be finite in (0, 1000]")
            if slot is not None and (isinstance(slot, bool) or not isinstance(slot, int) or not 0 <= slot < 18):
                raise ValueError("A3.2 home slot must be 0..17; MIT homing is not provided by the SDK")
            if not self._enabled:
                # Explicit homing establishes position; it must not prepend a
                # guessed or cached position command when feedback is absent.
                self.enable_all(send_initial_pose=False)
            slots = range(18) if slot is None else [slot]
            before = self.get_state()
            self._home_sequence = before.get("sequence", before.get("rx_ok"))
            self._enabled = False
            self._home_started = time.monotonic()
            try:
                for index in slots:
                    self._transport().home_slot(index, iq_counts)
                    self._homing_slots.add(index)
            except Exception:
                self.emergency_stop()
                raise
            if self._home_thread is None or not self._home_thread.is_alive():
                self._home_watch_stop.clear()
                self._home_thread = threading.Thread(target=self._watch_home, name="A32HomeMonitor", daemon=True)
                self._home_thread.start()
            return self.get_state()

    def _watch_home(self):
        """Enforce timeout/error stop even if the browser stops polling."""
        while not self._home_watch_stop.wait(.05):
            with self._lock:
                if not self._homing_slots:
                    return
                try:
                    self.get_state()
                except Exception as exc:
                    self._last_error = f"A3.2 homing monitor failed: {exc}"
                    self._stop_commands()
                    return

    def reset_home(self, slot=None):
        """Stop host targets and reset self homing state without disabling motors.

        SDK homeSlot does not promise to reset SEEK/RELEASE after cancellation.
        This recovery operation is intentionally separate from stop/disconnect.
        """
        with self._lock:
            self._permit_hardware()
            if slot is not None and (isinstance(slot, bool) or not isinstance(slot, int) or not 0 <= slot < 18):
                raise ValueError("A3.2 reset home slot must be 0..17; MIT has no self homing state")
            if not self.is_connected:
                raise A32DriverError("Connect A3.2 before resetting homing state")
            before = self._backend.get_state()
            self._home_reset_sequence = before.get("sequence", before.get("rx_ok"))
            self._stop_commands()
            self._home_reset_pending = True
            for index in range(18) if slot is None else [slot]:
                self._transport().home_reset_slot(index)
            self._last_error = ""
            # Only a new SDK telemetry frame can confirm the resulting state.
            return self.get_state()

    def prepare_targets(self, pose, *, clamp=True):
        """Pure conversion useful for previews/tests; never loads or calls a DLL."""
        constrained = self.constrain_pose(pose)
        changed = {name: value for name, value in constrained.items() if abs(value - pose[name]) > 1e-9}
        if changed and not clamp:
            raise A32DriverError(f"A3.2 target outside calibrated joint limits: {sorted(changed)}")
        converted = self.mapper.convert(constrained)
        own, mit = [0.] * 18, [0.] * 2
        for c in self.channels:
            values = converted.linear_strokes_mm if c["unit"] == "mm" else converted.rotary_positions_deg
            value = values[c["joint"]] * c["direction"] + c["offset"]
            lo, hi = c["limits"]
            if value < lo - 2e-6 or value > hi + 2e-6:
                raise A32DriverError(f"A3.2 conversion exceeds physical limits: {c['joint']}")
            (own if c["channel"] == "self" else mit)[c["index"]] = min(hi, max(lo, value))
        return {**converted.as_dict(), "model_id": self.model_id, "self": own, "mit_pos": mit,
                "applied_pose": dict(constrained), "clamped_joints_rad": changed}

    def send_joint_targets(self, pose, *, clamp=True, control=False):
        with self._lock:
            self._permit_hardware()
            state = self.get_control_state() if control else self.get_state()
            if not state.get("connected"):
                raise A32DriverError("Connect A3.2 before sending targets")
            if not self._enabled:
                raise A32DriverError("A3.2 is not enabled; enable before sending")
            if state["homing"]:
                raise A32DriverError("Wait for A3.2 homing to complete before sending")
            targets = self.prepare_targets(pose, clamp=clamp)
            try:
                backend = self._transport()
                if control and hasattr(backend, 'send_targets'):
                    timing = backend.send_targets(targets['self'], targets['mit_pos'])
                    if timing is not None:
                        targets['native_send_timing'] = timing
                else:
                    backend.set_targets(targets["self"], targets["mit_pos"])
                    backend.send_once()
            except Exception as exc:
                self._stop_commands()
                self._last_error = str(exc)
                raise
            self._last_targets = targets
            return targets

    def send_control_targets(self, pose, *, clamp=False):
        return self.send_joint_targets(pose, clamp=clamp, control=True)

    send_pose = send_joint_targets

    def _stop_commands(self):
        """Revoke host sending only; caller holds the device lock."""
        self._enabled = False
        self._homing_slots.clear()

    def emergency_stop(self):
        """Legacy API: stop further targets, never send a motor-disable frame."""
        with self._lock:
            self._stop_commands()
            return self.get_state()

    stop = emergency_stop

    def disconnect(self):
        with self._lock:
            self._stop_commands()
            if self._backend is not None:
                self._backend.disconnect()
            return self.get_state()

    def close(self):
        self._home_watch_stop.set()
        with self._lock:
            try:
                self.disconnect()
            finally:
                if self._backend is not None:
                    self._backend.close()
                    self._backend = None
        if self._home_thread is not None and self._home_thread is not threading.current_thread():
            self._home_thread.join(timeout=1.)

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
