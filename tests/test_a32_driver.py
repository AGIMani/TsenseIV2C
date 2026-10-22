"""A3.2 SDK adapter contract, tested without opening USB/TCP or moving motors."""
import copy
import math
from pathlib import Path
import sys
import time

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "src"), str(ROOT)]
from a32_driver import A32Device, A32DriverError, NativeHand


class SimulatedNativeHand:
    def __init__(self):
        self.calls = []
        self.state = {
            "connected": False, "streaming": False, "telemetry_age_ms": 0,
            "rx_ok": 1, "rx_mask": (1 << 18) - 1, "rx_hz": 100., "tx_hz": 0.,
            "firmware": {"valid": True, "proto_major": 1}, "last_error": "",
            "self": [{"slot": i, "online": True, "homing": 2, "position": 0.,
                      "target": 0., "iq": 0.} for i in range(18)],
            "mit": [{"index": i, "valid": True, "online": True, "fault": 0,
                     "position": 0., "velocity_rpm": 0., "iq": 0.} for i in range(2)],
        }
        self.fail_send = False

    def get_state(self):
        return copy.deepcopy(self.state)

    def connect_usb(self, port):
        self.calls.append(("usb", port))
        self.state["connected"] = True

    def connect_eth(self, host, port):
        self.calls.append(("eth", host, port))
        self.state["connected"] = True

    def set_targets(self, own, mit):
        self.calls.append(("targets", list(own), list(mit)))

    def send_once(self):
        self.calls.append(("send",))
        if self.fail_send:
            raise A32DriverError("simulated link loss")

    def enable_all(self, on):
        self.calls.append(("enable", on))

    def home_slot(self, slot, iq):
        self.calls.append(("home", slot, iq))
        if slot < len(self.state["self"]):
            self.state["self"][slot]["homing"] = 1

    def home_reset_slot(self, slot):
        self.calls.append(("home_reset", slot))
        if slot < len(self.state["self"]):
            self.state["self"][slot]["homing"] = 0

    def estop(self):
        self.calls.append(("estop",))

    def disconnect(self):
        self.calls.append(("disconnect",))
        self.state["connected"] = False

    def close(self):
        self.calls.append(("close",))


def connected_device(model_id="TsenseI_v3_2"):
    backend = SimulatedNativeHand()
    device = A32Device(model_id=model_id, backend=backend)
    device.connect_usb("SIM_ONLY")
    return device, backend


def test_import_constructor_and_preview_do_not_load_native_or_connect():
    device = A32Device()
    assert device._backend is None
    assert not device.get_state()["connected"]
    pose = {n: 0. for n in device.joint_limits()}
    device.prepare_targets(pose)
    assert device._backend is None


@pytest.mark.parametrize("model_id", ["TsenseI_v3_2", "TsenseI_v3_2_withid"])
def test_channels_limits_and_feedback_roundtrip(model_id):
    device, backend = connected_device(model_id)
    limits = device.joint_limits()
    pose = {n: (lo + hi) * .43 for n, (lo, hi) in limits.items()}
    pose["left_hand_index_swing_joint"] = .12
    command = device.prepare_targets(pose, clamp=False)
    assert len(command["self"]) == 18 and len(command["mit_pos"]) == 2
    assert command["self"][15] == pytest.approx(math.degrees(.12))
    assert command["mit_pos"][0] == pytest.approx(math.degrees(pose["left_hand_thumb_rotaback_joint2"]))
    assert command["mit_pos"][1] == pytest.approx(math.degrees(pose["left_hand_thumb_rota_block_joint"]))
    for index, (row, value) in enumerate(zip(backend.state["self"], command["self"])):
        row["position"] = value if index in (6, 7, 14, 15) else -value
        row["target"] = row["position"] * .5
    for row, value in zip(backend.state["mit"], command["mit_pos"]):
        row["position"] = value
    raw_before = copy.deepcopy(backend.state)
    calls_before = list(backend.calls)
    feedback = device.feedback_pose()
    assert feedback == pytest.approx(pose, abs=2e-7)
    assert device.starting_pose({n: lo for n, (lo, _) in limits.items()}) == pytest.approx(pose, abs=2e-7)
    state = device.get_state()
    assert state["seed_missing"] == []
    for raw, actual in zip(raw_before["self"], state["self"]):
        assert actual["position"] == raw["position"]
        assert actual["target"] == raw["target"]
    assert backend.state == raw_before
    assert backend.calls == calls_before
    for c in device.channels:
        value = command["self" if c["channel"] == "self" else "mit_pos"][c["index"]]
        assert c["limits"][0] <= value <= c["limits"][1]


@pytest.mark.parametrize("model_id", ["TsenseI_v3_2", "TsenseI_v3_2_withid"])
def test_physical_limit_clamp_changes_joint_pose_before_conversion(model_id):
    device = A32Device(model_id=model_id)
    pose = {n: 999. for n in device.joint_limits()}
    constrained = device.constrain_pose(pose)
    assert pose != constrained
    command = device.prepare_targets(pose)
    assert command["applied_pose"] == constrained
    assert command["joint_angles_rad"] == constrained
    assert set(command["clamped_joints_rad"]) == set(pose)
    assert math.degrees(-.34) - 1e-9 <= command["self"][6] <= math.degrees(.34) + 1e-9
    for c in device.channels:
        value = command["self" if c["channel"] == "self" else "mit_pos"][c["index"]]
        assert c["limits"][0] <= value <= c["limits"][1]
    assert command["mit_pos"][0] <= 99 and command["mit_pos"][1] <= 119
    with pytest.raises(A32DriverError, match="calibrated joint limits"):
        device.prepare_targets(pose, clamp=False)


def test_named_map_not_source_index_order():
    device = A32Device()
    pose = {n: 0. for n in device.joint_limits()}
    baseline = device.prepare_targets(pose)
    pose["left_hand_pinky_DIP_joint"] = .2
    command = device.prepare_targets(pose)
    assert command["self"][0] > 0
    assert sum(abs(x - y) > 1e-9 for x, y in zip(command["self"], baseline["self"])) == 1
    assert command["mit_pos"] == [0., 0.]


def test_telemetry_mask_and_can_nodes_match_sdk_mapping():
    device, backend = connected_device()
    backend.state["rx_mask"] = 0x3C0FF
    for i, row in enumerate(backend.state["self"]):
        row["online"] = not 8 <= i <= 13
        row["homing"] = 254
    state = device.get_state()
    assert state["telemetry_diagnostics"]["self_offline_slots"] == list(range(8, 14))
    assert state["telemetry_diagnostics"]["self_online_mask_hex"] == "0x3C0FF"
    assert [(r["bus"], r["can_node"]) for r in state["self"][8:14]] == [(2, i) for i in range(1, 7)]
    assert state["self"][14]["online"] and state["self"][15]["online"]
    assert state["self"][17]["bus"] == 3 and state["self"][17]["node"] == 2 and state["self"][17]["can_node"] == 5
    assert state["self"][0]["homing_label"].startswith("THETA")


def test_usb_and_eth_connect_never_enable_or_send():
    device, backend = connected_device()
    assert backend.calls == [("usb", "SIM_ONLY")]
    device.disconnect()
    device.connect_eth("127.0.0.1", 5001)
    assert backend.calls[-1] == ("eth", "127.0.0.1", 5001)
    assert not device.get_state()["enabled"]
    with pytest.raises(A32DriverError, match="already connected"):
        device.connect_usb("COM1")


def test_enable_aligns_all_twenty_feedback_positions_before_power():
    device, backend = connected_device()
    backend.state["self"][0]["position"] = -1.25
    backend.state["mit"][1]["position"] = 31.25
    device.enable_all()
    assert [c[0] for c in backend.calls[-3:]] == ["targets", "send", "enable"]
    assert backend.calls[-3][1][0] == pytest.approx(1.25)
    assert backend.calls[-3][2][1] == pytest.approx(31.25)
    assert backend.calls[-1] == ("enable", True)
    assert device.get_state()["enabled"]


@pytest.mark.parametrize("model_id", ["TsenseI_v3_2", "TsenseI_v3_2_withid"])
def test_enable_without_initial_pose_does_not_set_or_send_targets(model_id):
    device, backend = connected_device(model_id)
    try:
        seed = device.starting_pose()
        device.enable_all(start_pose=seed, send_initial_pose=False)
        assert backend.calls == [("usb", "SIM_ONLY"), ("enable", True)]
        assert device._last_targets is None
        assert device.get_state()["enabled"]
        device.send_joint_targets(seed, clamp=False)
        assert [call[0] for call in backend.calls[-2:]] == ["targets", "send"]
        assert device._last_targets["applied_pose"] == seed
    finally:
        device.close()


def test_initial_wait_can_home_without_previous_enable_and_cannot_send():
    device, backend = connected_device()
    for row in backend.state["self"]:
        row["homing"] = 0
    assert not device.get_state()["ready"]
    device.home()
    assert [c[1] for c in backend.calls if c[0] == "home"] == list(range(18))
    assert device.get_state()["homing"] and device.get_state()["homing_active"]
    assert not device.get_state()["enabled"]
    with pytest.raises(A32DriverError, match="not enabled"):
        device.send_joint_targets({n: 0. for n in device.joint_limits()})
    for row in backend.state["self"]:
        row["homing"] = 2
    device._home_started -= 1
    assert device.get_state()["homing"]  # Old DONE snapshot cannot finish this home.
    backend.state["rx_ok"] += 1
    assert not device.get_state()["homing"]
    assert device.get_state()["ready"]
    device.enable_all()
    assert device.get_state()["enabled"]


@pytest.mark.parametrize("reason", ["timeout", "motor_error", "stale", "offline"])
def test_homing_error_stops_host_commands_without_disabling_motors(reason):
    device, backend = connected_device()
    device.home(slot=3)
    before = list(backend.calls)
    if reason == "timeout":
        device._home_started -= device.home_timeout_s + 1
    elif reason == "motor_error":
        backend.state["self"][3]["homing"] = 3
    else:
        device._home_started -= 1
        if reason == "stale":
            backend.state["telemetry_age_ms"] = 501
        else:
            backend.state["self"][0]["online"] = False
    state = device.get_state()
    assert not state["enabled"] and not state["ready"]
    assert "command sending stopped" in state["last_error"]
    assert backend.calls == before
    device.close()


def test_home_watchdog_blocks_commands_without_browser_polling_or_disable():
    device, backend = connected_device()
    device.home(slot=3)
    with device._lock:
        device._home_started -= device.home_timeout_s + 1
    before = list(backend.calls)
    deadline = time.monotonic() + 1
    while time.monotonic() < deadline and device._homing_slots:
        time.sleep(.01)
    assert not device._homing_slots
    assert "homing timed out" in device._last_error
    assert backend.calls == before
    device.close()


def test_explicit_reset_recovers_cancelled_home_without_claiming_done():
    device, backend = connected_device()
    device.home(slot=3)
    device.stop()
    assert device.get_state()["homing"]  # Host stop does not cancel firmware homing.
    with pytest.raises(A32DriverError, match="homing to complete"):
        device.enable_all()
    before = len(backend.calls)
    state = device.reset_home(slot=3)
    assert backend.calls[before:] == [("home_reset", 3)]
    assert state["home_reset_pending"] and not state["enabled"] and not state["ready"]
    device.enable_all()  # Reset ACK is diagnostic, not a telemetry prerequisite.
    assert device.get_state()["enabled"]
    backend.state["rx_ok"] += 1
    assert not device.get_state()["home_reset_pending"]
    device.home(slot=3)
    assert backend.calls[-1] == ("home", 3, 80.)
    device.close()


def test_reset_all_does_not_disable_or_reset_mit():
    device, backend = connected_device()
    before = len(backend.calls)
    device.reset_home()
    assert backend.calls[before:] == [("home_reset", i) for i in range(18)]
    assert not device.get_state()["enabled"]


@pytest.mark.parametrize("slot", [-1, 18, True, 1.5])
def test_reset_rejects_invalid_slots_without_writing(slot):
    device, backend = connected_device()
    before = list(backend.calls)
    with pytest.raises(ValueError, match="slot must be 0..17"):
        device.reset_home(slot=slot)
    assert backend.calls == before


def test_reset_requires_connection_and_does_not_load_native_when_disconnected():
    device = A32Device()
    with pytest.raises(A32DriverError, match="Connect A3.2"):
        device.reset_home()
    assert device._backend is None


@pytest.mark.parametrize("fault", ["stale", "self_offline", "mit_offline", "mit_fault", "self_home_wait"])
def test_send_continues_with_missing_or_unready_feedback(fault):
    device, backend = connected_device()
    device.enable_all()
    if fault == "stale":
        backend.state["telemetry_age_ms"] = 501
    elif fault == "self_offline":
        backend.state["self"][3]["online"] = False
    elif fault == "mit_offline":
        backend.state["mit"][0]["online"] = False
    elif fault == "mit_fault":
        backend.state["mit"][1]["fault"] = 4
    else:
        backend.state["self"][6]["homing"] = 0
    device.send_joint_targets({n: 0. for n in device.joint_limits()})
    assert backend.calls[-1] == ("send",)
    assert device.get_state()["enabled"]


@pytest.mark.parametrize("model_id", ["TsenseI_v3_2", "TsenseI_v3_2_withid"])
@pytest.mark.parametrize("telemetry", ["empty", "stale", "offline", "partial", "nan", "outside", "mit_fault"])
def test_enable_requires_current_complete_feedback_without_fallback(model_id, telemetry):
    device, backend = connected_device(model_id)
    fallback = {n: (lo + hi) * .3 for n, (lo, hi) in device.joint_limits().items()}
    device.enable_all()
    device.send_joint_targets(fallback, clamp=False)
    device.stop()
    if telemetry == "empty":
        backend.state.update(self=[], mit=[], telemetry_age_ms=-1)
    elif telemetry == "stale":
        backend.state["telemetry_age_ms"] = 999
    elif telemetry == "offline":
        backend.state["self"][2]["online"] = False
    elif telemetry == "partial":
        backend.state["mit"] = backend.state["mit"][:1]
    elif telemetry == "nan":
        backend.state["self"][2]["position"] = float("nan")
    elif telemetry == "outside":
        backend.state["self"][2]["position"] = 1.  # 1 mm beyond zero, not noise.
    else:
        backend.state["mit"][0]["fault"] = 1
    before = list(backend.calls)
    with pytest.raises(A32DriverError, match="反馈"):
        device.starting_pose(fallback)
    for kwargs in ({}, {"fallback_pose": fallback}, {"start_pose": fallback}):
        with pytest.raises(A32DriverError, match="反馈"):
            device.enable_all(**kwargs)
    assert device.get_state()["seed_missing"]
    assert not device.get_state()["enabled"]
    assert backend.calls == before


def test_explicit_old_seed_cannot_override_new_feedback():
    device, backend = connected_device("TsenseI_v3_2_withid")
    old = {n: (lo + hi) * .3 for n, (lo, hi) in device.joint_limits().items()}
    before = list(backend.calls)
    with pytest.raises(A32DriverError, match="起步姿态与当前反馈不一致"):
        device.enable_all(start_pose=old)
    assert backend.calls == before and not device.get_state()["enabled"]


def test_fresh_wait_feedback_can_seed_without_requiring_homing():
    device, backend = connected_device()
    for row in backend.state["self"]:
        row["homing"] = 0
    device.enable_all()
    assert device.get_state()["enabled"] and not device.get_state()["ready"]


def test_explicit_home_without_feedback_does_not_send_a_position_seed():
    device, backend = connected_device()
    backend.state.update(self=[], mit=[], telemetry_age_ms=-1)
    try:
        device.home(slot=3)
        assert backend.calls == [("usb", "SIM_ONLY"), ("enable", True), ("home", 3, 80.)]
        assert not device.get_state()["enabled"]
    finally:
        device.close()


def test_disconnected_device_cannot_enable_or_send_without_telemetry():
    device = A32Device()
    with pytest.raises(A32DriverError, match="Connect A3.2"):
        device.enable_all()
    with pytest.raises(A32DriverError, match="Connect A3.2"):
        device.send_joint_targets({n: 0. for n in device.joint_limits()})
    assert device._backend is None


def test_failed_send_revokes_host_permission_without_disable():
    device, backend = connected_device()
    device.enable_all()
    backend.fail_send = True
    with pytest.raises(A32DriverError, match="simulated link loss"):
        device.send_joint_targets({n: 0. for n in device.joint_limits()})
    assert not device.get_state()["enabled"]
    assert backend.calls[-1] == ("send",)
    assert ("estop",) not in backend.calls and ("enable", False) not in backend.calls
    before = list(backend.calls)
    with pytest.raises(A32DriverError, match="not enabled"):
        device.send_joint_targets({n: 0. for n in device.joint_limits()})
    assert backend.calls == before


def test_unexpected_disconnect_and_close_never_disable_motors():
    device, backend = connected_device()
    device.enable_all()
    backend.state["connected"] = False
    assert not device.get_state()["enabled"]
    backend.state["connected"] = True
    device.close()
    assert [c[0] for c in backend.calls[-2:]] == ["disconnect", "close"]
    assert ("estop",) not in backend.calls and ("enable", False) not in backend.calls


@pytest.mark.parametrize("model_id", ["TsenseI_v3_2", "TsenseI_v3_2_withid"])
@pytest.mark.parametrize("action", ["stop", "emergency_stop", "disable_all", "disconnect", "close"])
def test_every_stop_api_blocks_targets_without_sending_disable(model_id, action):
    device, backend = connected_device(model_id)
    device.enable_all()
    pose = device.starting_pose()
    device.send_joint_targets(pose)
    before = list(backend.calls)
    getattr(device, action)()
    assert not device.get_state()["enabled"]
    expected = [("disconnect",), ("close",)] if action == "close" else [("disconnect",)] if action == "disconnect" else []
    assert backend.calls == before + expected
    with pytest.raises(A32DriverError):
        device.send_joint_targets(pose)
    assert backend.calls == before + expected
    if action not in ("disconnect", "close"):
        device.enable_all(start_pose=pose)
        device.send_joint_targets(pose)
        assert backend.calls[-1] == ("send",)
    device.close()
    assert ("estop",) not in backend.calls and ("enable", False) not in backend.calls


@pytest.mark.parametrize("operation", ["set_targets", "send_once", "enable_all", "home_slot"])
def test_enable_and_home_errors_never_send_disable(monkeypatch, operation):
    device, backend = connected_device()

    def fail(*args):
        raise A32DriverError("simulated command failure")

    monkeypatch.setattr(backend, operation, fail)
    with pytest.raises(A32DriverError, match="simulated command failure"):
        if operation == "home_slot":
            device.home(slot=3)
        else:
            device.enable_all()
    assert not device.get_state()["enabled"]
    device.close()
    assert ("estop",) not in backend.calls and ("enable", False) not in backend.calls


def test_home_monitor_read_error_never_sends_disable(monkeypatch):
    device, backend = connected_device()
    device.home(slot=3)
    before = list(backend.calls)

    def fail():
        raise A32DriverError("simulated monitor failure")

    with monkeypatch.context() as patch:
        patch.setattr(backend, "get_state", fail)
        device._home_thread.join(timeout=1)
        assert not device._home_thread.is_alive()
        assert not device._homing_slots and not device._enabled
        assert "homing monitor failed" in device._last_error
        assert backend.calls == before
    device.close()


@pytest.mark.parametrize("has_stop_sending", [False, True])
def test_native_python_stop_never_calls_legacy_disable_abi(has_stop_sending):
    # Even a legacy library selected through A32_NATIVE_LIBRARY must never
    # receive a32_estop or a32_enable_all(false) through the Python adapter.
    hand = NativeHand.__new__(NativeHand)
    hand._can_stop_sending = has_stop_sending
    calls = []
    hand._call = lambda *args: calls.append(args)
    hand.estop()
    hand.enable_all(False)
    assert calls == ([("a32_stop_sending",)] * 2 if has_stop_sending else [])
    hand.enable_all(True)
    assert calls[-1] == ("a32_enable_all", 1)


def test_right_hand_connection_uses_shared_calibrated_slot_map():
    backend = SimulatedNativeHand()
    device = A32Device(hand_side="right", model_id="TsenseI_v3_2_withid", backend=backend)
    device.connect_usb("SIM_ONLY")
    assert backend.calls == [("usb", "SIM_ONLY")]
    assert device.get_state()["hand_side"] == "right"
    pose = {n: 0. for n in device.joint_limits()}
    command = device.prepare_targets(pose)
    assert len(command["self"]) == 18 and len(command["mit_pos"]) == 2
    device.close()


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), True, "0"])
def test_nonfinite_and_nonreal_targets_rejected_before_sdk(bad):
    device = A32Device()
    pose = {n: 0. for n in device.joint_limits()}
    pose[next(iter(pose))] = bad
    with pytest.raises(ValueError, match="finite joint radians"):
        device.prepare_targets(pose)


@pytest.mark.skipif(sys.platform != "linux" or not (ROOT / "src/a32_driver/bin/liba32_native.so").is_file(),
                    reason="Built Linux SDK wrapper is not available")
def test_real_native_dll_load_and_snapshot_without_hardware():
    native = NativeHand()
    try:
        state = native.get_state()
        assert state["sdk_version"] == "1.1.15"
        assert not state["connected"] and not state["streaming"]
        assert len(state["self"]) == 18 and len(state["mit"]) == 2
        assert state["telemetry_age_ms"] == -1
        assert state["diagnostics"]["disconnect_count"] == 0
        assert state["diagnostics"]["send_attempts"] == 0
    finally:
        native.close()
