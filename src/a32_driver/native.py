"""ctypes C ABI loader. SDK examples/Bridge executables are never used."""
from __future__ import annotations

import ctypes
import json
import os
import sys
from pathlib import Path
import threading

# ABI v2 exposes SDK 1.1's fixed 50:1 output-axis units on both MIT channels.
# Physical channels: both restored to 50:1 at the user's request, 2026-09-29.
SDK_MIT_GEAR_RATIO = 50.0
MIT_GEAR_RATIOS = (50.0, 50.0)


class A32DriverError(RuntimeError):
    """Native transport, firmware, calibration or readiness failure."""


def resolve_native_library(library_path=None):
    """Locate the platform library without loading it or touching a device."""
    binary_dir = Path(__file__).with_name("bin")
    if sys.platform.startswith("linux"):
        default_library = binary_dir / "liba32_native.so"
    elif sys.platform == "win32":
        default_library = binary_dir / "a32_native_timed.dll"
        if not default_library.is_file():
            default_library = binary_dir / "a32_native.dll"
    else:
        raise A32DriverError(f"Unsupported A3.2 driver platform: {sys.platform}")
    return Path(library_path or os.environ.get("A32_NATIVE_LIBRARY", "") or
                os.environ.get("A32_NATIVE_DLL", "") or default_library).resolve()


class _ControlState(ctypes.Structure):
    _fields_ = [(n, ctypes.c_int32) for n in ('connected', 'telemetry_age_ms', 'homing_mask', 'mit_fault_mask')]


class _SendTiming(ctypes.Structure):
    _fields_ = [(n, ctypes.c_int64) for n in ('sequence', 'started_ns', 'finished_ns')] + [
        ('interval_ms', ctypes.c_double), ('duration_ms', ctypes.c_double)]


class NativeHand:
    """One SDK object. Each call is serialized; CDLL releases Python's GIL."""

    def __init__(self, library_path=None):
        # Separate filename permits deployment while a replay service still
        # holds the previous DLL. Explicit paths/environment always win.
        path = resolve_native_library(library_path)
        build_command = ("bash src/a32_driver/build_native_linux.sh" if sys.platform.startswith("linux")
                         else "src/a32_driver/build_native.ps1")
        if not path.is_file():
            raise A32DriverError(
                f"A3.2 native library missing: {path}. Run {build_command}.")
        self._lock = threading.RLock()
        try:
            self._dll = ctypes.CDLL(str(path))
        except OSError as exc:
            raise A32DriverError(f"Cannot load A3.2 native library {path}: {exc}") from exc
        ptr, integer, string = ctypes.c_void_p, ctypes.c_int, ctypes.c_char_p
        try:
            self._dll.a32_abi_version.argtypes = []
            self._dll.a32_abi_version.restype = integer
            if self._dll.a32_abi_version() != 2:
                raise A32DriverError("Unsupported A3.2 native ABI; rebuild the native library")
        except AttributeError as exc:
            raise A32DriverError("Invalid A3.2 native library; rebuild the native library") from exc
        signatures = {
            "a32_abi_version": ([], integer), "a32_create": ([], ptr),
            "a32_destroy": ([ptr], None),
            "a32_last_error": ([ptr, ptr, integer], integer),
            "a32_connect_usb": ([ptr, string], integer),
            "a32_connect_eth": ([ptr, string, ctypes.c_ushort], integer),
            "a32_disconnect": ([ptr], integer),
            "a32_set_targets": ([ptr, ctypes.POINTER(ctypes.c_float), ctypes.POINTER(ctypes.c_float)], integer),
            "a32_send_once": ([ptr], integer), "a32_enable_all": ([ptr, integer], integer),
            "a32_home_slot": ([ptr, integer, ctypes.c_float], integer),
            "a32_home_reset_slot": ([ptr, integer], integer),
            "a32_estop": ([ptr], integer), "a32_state": ([ptr, ptr, integer], integer),
        }
        for name, (args, result) in signatures.items():
            method = getattr(self._dll, name)
            method.argtypes, method.restype = args, result
        # This additive entry point only clears queued host targets. Never use
        # legacy estop/enable(false), which older libraries implement as power-off.
        self._can_stop_sending = hasattr(self._dll, "a32_stop_sending")
        if self._can_stop_sending:
            self._dll.a32_stop_sending.argtypes = [ptr]
            self._dll.a32_stop_sending.restype = integer
        # Additive ABI-v2 capabilities. Older DLLs keep their functional path;
        # their status explicitly lacks native per-packet measurements.
        self.fast_control = all(hasattr(self._dll, n) for n in ('a32_control_state', 'a32_send_targets'))
        if self.fast_control:
            self._dll.a32_control_state.argtypes = [ptr, ctypes.POINTER(_ControlState)]
            self._dll.a32_control_state.restype = integer
            self._dll.a32_send_targets.argtypes = [ptr, ctypes.POINTER(ctypes.c_float),
                ctypes.POINTER(ctypes.c_float), ctypes.POINTER(_SendTiming)]
            self._dll.a32_send_targets.restype = integer
        self._handle = self._dll.a32_create()
        if not self._handle:
            raise A32DriverError("Failed to create native A3 SDK object")

    def _text(self, method):
        # State may grow between length query and copy as telemetry changes.
        # The state fits comfortably in 64 KiB. Avoid serializing and reading
        # all SDK channels twice per poll merely to query the required length.
        capacity = 65536
        for _ in range(3):
            output = ctypes.create_string_buffer(capacity)
            needed = method(self._handle, output, capacity)
            if needed < 1:
                raise A32DriverError("Native A3 SDK snapshot failed")
            if needed <= capacity:
                return output.value.decode("utf-8", errors="replace")
            capacity = needed + 4096
        raise A32DriverError("Native A3 SDK snapshot changed unexpectedly")

    def _call(self, name, *args):
        with self._lock:
            if not self._handle:
                raise A32DriverError("Native A3 SDK object is closed")
            if not getattr(self._dll, name)(self._handle, *args):
                raise A32DriverError(self._text(self._dll.a32_last_error) or f"{name} failed")

    def connect_usb(self, port="auto"):
        self._call("a32_connect_usb", str(port).encode("utf-8"))

    def connect_eth(self, host, port=5001):
        self._call("a32_connect_eth", str(host).encode("utf-8"), int(port))

    def disconnect(self):
        self._call("a32_disconnect")

    def set_targets(self, self_positions, mit_positions):
        if len(self_positions) != 18 or len(mit_positions) != 2:
            raise ValueError("A3.2 requires 18 self slots and 2 MIT channels")
        # The DLL/SDK multiplies by 50 on the wire. Adapt once at this boundary
        # so callers continue to supply physical output-axis degrees.
        sdk_positions = [position * ratio / SDK_MIT_GEAR_RATIO
                         for position, ratio in zip(mit_positions, MIT_GEAR_RATIOS)]
        self._call("a32_set_targets", (ctypes.c_float * 18)(*self_positions),
                   (ctypes.c_float * 2)(*sdk_positions))

    def send_once(self):
        self._call("a32_send_once")

    def get_control_state(self):
        if not self.fast_control:
            return self.get_state()
        result = _ControlState()
        self._call('a32_control_state', ctypes.byref(result))
        return dict(connected=bool(result.connected), telemetry_age_ms=result.telemetry_age_ms,
            homing_active=bool(result.homing_mask), mit=[
                dict(online=True, fault=True) if result.mit_fault_mask & (1 << i)
                else dict(online=False, fault=False) for i in range(2)])

    def send_targets(self, self_positions, mit_positions):
        if not self.fast_control:
            self.set_targets(self_positions, mit_positions)
            self.send_once()
            return None
        if len(self_positions) != 18 or len(mit_positions) != 2:
            raise ValueError('A3.2 requires 18 self slots and 2 MIT channels')
        sdk_positions = [p * ratio / SDK_MIT_GEAR_RATIO for p, ratio in zip(mit_positions, MIT_GEAR_RATIOS)]
        timing = _SendTiming()
        self._call('a32_send_targets', (ctypes.c_float*18)(*self_positions),
                   (ctypes.c_float*2)(*sdk_positions), ctypes.byref(timing))
        return {name: getattr(timing, name) for name, _ in timing._fields_}

    def enable_all(self, on=True):
        if on:
            self._call("a32_enable_all", 1)
        else:
            self.estop()

    def home_slot(self, slot, iq_counts):
        self._call("a32_home_slot", int(slot), float(iq_counts))

    def home_reset_slot(self, slot):
        self._call("a32_home_reset_slot", int(slot))

    def estop(self):
        """Compatibility stop: discard pending targets without a disable frame."""
        if self._can_stop_sending:
            self._call("a32_stop_sending")

    def get_state(self):
        with self._lock:
            if not self._handle:
                raise A32DriverError("Native A3 SDK object is closed")
            state = json.loads(self._text(self._dll.a32_state))
            for row in state.get("mit", []):
                ratio = MIT_GEAR_RATIOS[row["index"]]
                for key in ("position", "velocity_rpm"):
                    if row.get(key) is not None:
                        row[key] *= SDK_MIT_GEAR_RATIO / ratio
                row["gear_ratio"] = ratio
            positions = state.get("targets", {}).get("mit_pos")
            if positions is not None:
                state["targets"]["mit_pos"] = [
                    value * SDK_MIT_GEAR_RATIO / ratio if value is not None else None
                    for value, ratio in zip(positions, MIT_GEAR_RATIOS)]
            state["mit_gear_ratios"] = list(MIT_GEAR_RATIOS)
            return state

    def close(self):
        with self._lock:
            if self._handle:
                self._dll.a32_destroy(self._handle)
                self._handle = None

    def __del__(self):
        if getattr(self, "_handle", None):
            self.close()
