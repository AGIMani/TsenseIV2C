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
    assert math.degrees(.005) - 1e-9 <= command["self"][6] <= math.degrees(.695) + 1e-9
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


@pytest.mark.parametrize("reason", ["timeout", "motor_error"])
def test_homing_timeout_or_motor_error_requests_whole_hand_stop(reason):
    device, backend = connected_device()
    device.home(slot=3)
    if reason == "timeout":
        device._home_started -= device.home_timeout_s + 1
    else:
        backend.state["self"][3]["homing"] = 3
    state = device.get_state()
    assert not state["enabled"] and not state["ready"]
    assert "stop requested" in state["last_error"]
    assert backend.calls[-1] == ("estop",)


def test_home_watchdog_stops_without_browser_polling():
    device, backend = connected_device()
    device.home(slot=3)
    with device._lock:
        device._home_started -= device.home_timeout_s + 1
    deadline = time.monotonic() + 1
    while time.monotonic() < deadline and backend.calls[-1][0] != "estop":
        time.sleep(.01)
    assert backend.calls[-1] == ("estop",)
    device.close()


def test_explicit_reset_recovers_cancelled_home_without_claiming_done():
    device, backend = connected_device()
    device.home(slot=3)
    device.stop()
    assert device.get_state()["homing"]  # estop does not rewrite firmware state.
    with pytest.raises(A32DriverError, match="homing to complete"):
        device.enable_all()
    before = len(backend.calls)
    state = device.reset_home(slot=3)
    assert backend.calls[before:] == [("estop",), ("home_reset", 3)]
    assert state["home_reset_pending"] and not state["enabled"] and not state["ready"]
    device.enable_all()  # Reset ACK is diagnostic, not a telemetry prerequisite.
    assert device.get_state()["enabled"]
    backend.state["rx_ok"] += 1
    assert not device.get_state()["home_reset_pending"]
    device.home(slot=3)
    assert backend.calls[-1] == ("home", 3, 80.)
    device.close()


def test_reset_all_stops_first_and_never_resets_mit():
    device, backend = connected_device()
    before = len(backend.calls)
    device.reset_home()
    assert backend.calls[before:] == [("estop",)] + [("home_reset", i) for i in range(18)]
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
@pytest.mark.parametrize("telemetry", ["empty", "stale", "offline", "wait"])
def test_enable_and_send_without_telemetry_or_homing(model_id, telemetry):
    device, backend = connected_device(model_id)
    if telemetry == "empty":
        backend.state.update(self=[], mit=[], telemetry_age_ms=-1)
    elif telemetry == "stale":
        backend.state["telemetry_age_ms"] = 999
    elif telemetry == "offline":
        for row in backend.state["self"] + backend.state["mit"]:
            row["online"] = False
    else:
        for row in backend.state["self"]:
            row["homing"] = 0
    fallback = {n: (lo + hi) * .3 for n, (lo, hi) in device.joint_limits().items()}
    device.enable_all(fallback_pose=fallback)
    assert device.get_state()["enabled"] and device.get_state()["command_ready"]
    if telemetry != "wait":
        assert len(device.get_state()["seed_missing"]) == 20
        expected = device.prepare_targets(fallback)
        assert backend.calls[-3][1] == pytest.approx(expected["self"])
        assert backend.calls[-3][2] == pytest.approx(expected["mit_pos"])
    sent = device.send_joint_targets(fallback, clamp=False)
    assert sent["applied_pose"] == fallback
    # Losing all telemetry must neither stop the stream nor reset its origin.
    backend.state.update(self=[], mit=[], telemetry_age_ms=-1)
    device.stop()
    assert device.starting_pose({n: 0. for n in fallback}) == fallback
    device.enable_all()
    device.send_joint_targets(fallback, clamp=False)
    device.reset_home()
    assert device.get_state()["home_reset_pending"]
    device.enable_all()  # No reset telemetry ACK is required either.


def test_partial_feedback_only_overrides_available_starting_joints():
    device, backend = connected_device("TsenseI_v3_2_withid")
    fallback = {n: (lo + hi) * .3 for n, (lo, hi) in device.joint_limits().items()}
    backend.state["self"] = backend.state["self"][:1]
    backend.state["self"][0]["position"] = -1.25
    backend.state["mit"] = []
    pose = device.starting_pose(fallback)
    measured_name = next(c["joint"] for c in device.channels if c["channel"] == "self" and c["index"] == 0)
    assert pose[measured_name] != fallback[measured_name]
    assert {n: v for n, v in pose.items() if n != measured_name} == {n: v for n, v in fallback.items() if n != measured_name}
    assert len(device.get_state()["seed_missing"]) == 19
    device.enable_all(start_pose=pose)
    assert backend.calls[-3][1][0] == pytest.approx(1.25)


def test_disconnected_device_cannot_enable_or_send_without_telemetry():
    device = A32Device()
    with pytest.raises(A32DriverError, match="Connect A3.2"):
        device.enable_all()
    with pytest.raises(A32DriverError, match="Connect A3.2"):
        device.send_joint_targets({n: 0. for n in device.joint_limits()})
    assert device._backend is None


def test_failed_send_disarms_and_best_effort_estops():
    device, backend = connected_device()
    device.enable_all()
    backend.fail_send = True
    with pytest.raises(A32DriverError, match="simulated link loss"):
        device.send_joint_targets({n: 0. for n in device.joint_limits()})
    assert not device.get_state()["enabled"]
    assert backend.calls[-1] == ("estop",)


def test_unexpected_disconnect_disarms_and_close_stops_before_disconnect():
    device, backend = connected_device()
    device.enable_all()
    backend.state["connected"] = False
    assert not device.get_state()["enabled"]
    backend.state["connected"] = True
    device.close()
    assert [c[0] for c in backend.calls[-3:]] == ["estop", "disconnect", "close"]


def test_right_hand_connection_blocked_without_loading_dll():
    device = A32Device(hand_side="right")
    with pytest.raises(A32DriverError, match="not calibrated"):
        device.connect_usb()
    with pytest.raises(A32DriverError, match="not calibrated"):
        device.connect_eth("127.0.0.1")
    assert device._backend is None
    device.prepare_targets({n: 0. for n in device.joint_limits()})


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), True, "0"])
def test_nonfinite_and_nonreal_targets_rejected_before_sdk(bad):
    device = A32Device()
    pose = {n: 0. for n in device.joint_limits()}
    pose[next(iter(pose))] = bad
    with pytest.raises(ValueError, match="finite joint radians"):
        device.prepare_targets(pose)


@pytest.mark.skipif(sys.platform != "win32" or not (ROOT / "src/a32_driver/bin/a32_native.dll").is_file(),
                    reason="Built Windows x64 SDK wrapper is not available")
def test_real_native_dll_load_and_snapshot_without_hardware():
    native = NativeHand()
    try:
        state = native.get_state()
        assert state["sdk_version"] == "1.1.0"
        assert not state["connected"] and not state["streaming"]
        assert len(state["self"]) == 18 and len(state["mit"]) == 2
        assert state["telemetry_age_ms"] == -1
        assert state["diagnostics"]["disconnect_count"] == 0
        assert state["diagnostics"]["send_attempts"] == 0
    finally:
        native.close()
