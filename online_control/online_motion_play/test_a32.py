"""A3.2 editor, ownership and playback regressions; no physical device I/O."""
import copy
import io
import json
from pathlib import Path
import time
from types import SimpleNamespace

import pytest

import server
from a32_session import A32Session, MODEL_IDS, MOTOR_FINGER_SCALES
from hand_profiles import load_hand_profile
from a32_kinematics.joint_coordinates import COORDINATE_VERSION, SWING_AXIS_ALIGNMENT, is_finger_swing
from test_server import bridge


@pytest.fixture(autouse=True)
def isolated_recording_root(tmp_path, monkeypatch):
    monkeypatch.setattr("utils.control_session_recorder.control_log_parent", lambda: tmp_path / "logs")


class FakeDevice:
    def __init__(self, side="left", model_id=MODEL_IDS[0]):
        self.profile = load_hand_profile(model_id)
        self.side = side
        self.is_connected = self.enabled = False
        self.ready = True
        self.homing = False
        self.calls = []
        self.pose = self.profile.init_pose(side)

    def joint_limits(self):
        return self.profile.joint_limits(self.side)

    def constrain_pose(self, pose):
        return dict(pose)

    def get_state(self):
        return {"connected": self.is_connected, "enabled": self.enabled, "ready": self.ready,
                "homing": self.homing, "self": [], "mit": [], "last_error": ""}

    def connect_usb(self, port="auto"):
        self.calls.append(("usb", port))
        self.is_connected = True

    def connect_eth(self, host, port=5001):
        self.calls.append(("eth", host, port))
        self.is_connected = True

    def stop(self):
        self.calls.append(("stop",))
        self.enabled = False
        self.homing = False

    def disconnect(self):
        self.calls.append(("disconnect",))
        self.is_connected = self.enabled = False

    close = disconnect

    def enable_all(self, *, fallback_pose=None, start_pose=None, send_initial_pose=True):
        if send_initial_pose:
            self.calls.append(("send", dict(start_pose or self.pose)))
        self.calls.append(("enable",))
        self.enabled = True

    def starting_pose(self, fallback_pose=None):
        if not self.ready:
            raise ValueError("A3.2 起步需要完整有效的反馈")
        return dict(self.pose)

    def send_joint_targets(self, pose, *, clamp, motor_scales=None):
        assert clamp is True
        assert self.is_connected and self.enabled
        self.calls.append(("send", dict(pose)))

    def home(self, slot=None, iq_counts=80):
        self.calls.append(("home", slot, iq_counts))
        self.enabled = False
        self.homing = True

    def reset_home(self, slot=None):
        self.calls.append(("reset_home", slot))
        self.enabled = self.homing = False


class Clock:
    value = 100.

    def __call__(self):
        return self.value


@pytest.mark.parametrize("model_id", MODEL_IDS)
@pytest.mark.parametrize("side", ["left", "right"])
def test_playback_uses_shared_motion_limits(tmp_path, model_id, side):
    value = A32Session(tmp_path, side, model_id=model_id,
                       device=FakeDevice(side, model_id), clock=Clock(), start=False)
    try:
        assert value.input.max_velocity == [20.] * 20
        assert value.input.max_acceleration == [400.] * 20
        assert value.input.max_jerk == [2400.] * 20
        assert value.planner.delta_time == .01
    finally:
        value.close()


def tick(session, elapsed=.01):
    session._clock.value += elapsed
    session.tick()


@pytest.fixture
def session(tmp_path):
    device = FakeDevice()
    value = A32Session(tmp_path, device=device, clock=Clock(), start=False)
    yield value
    value.close()


def widget(session, name, value=None):
    return session.dispatch("widget", {"id": session.generation + ":" + name, "value": value})


def test_connect_does_not_enable_and_real_control_starts_from_feedback(session):
    session.dispatch("connect", {"transport": "usb", "port": "COM_TEST"})
    assert session.device.calls == [("usb", "COM_TEST")]
    assert session.snapshot()["controls"]["realtimeJointControl"]["enabled"]
    session.device.pose[session.names[0]] = .2
    widget(session, "a32Enable")
    assert session.device.calls[-1] == ("enable",)
    widget(session, "realtimeJointControl", True)
    assert session.pose == session.device.pose
    name = session.names[0]
    widget(session, name, .4)
    tick(session)
    assert .2 < session.device.calls[-1][1][name] < .21
    assert session.device.calls[-1][0] == "send"
    session.device.ready = False
    tick(session)
    assert session.armed and session.device.calls[-1][0] == "send"
    session.device.is_connected = False
    tick(session)
    assert not session.armed and "断开" in session.message


@pytest.mark.parametrize("model_id", MODEL_IDS)
@pytest.mark.parametrize("pre_enabled", [False, True])
@pytest.mark.parametrize("control", ["realtimeJointControl", "isConnectOffline"])
def test_control_checkbox_waits_for_explicit_target_without_sending(tmp_path, model_id, pre_enabled, control):
    device = FakeDevice(model_id=model_id)
    value = A32Session(tmp_path, model_id=model_id, device=device, clock=Clock(), start=False)
    try:
        value.dispatch("connect", {})
        if pre_enabled:
            widget(value, "a32Enable")
        widget(value, control, True)
        for _ in range(100):
            tick(value)
        assert value.armed and value.sample_ticks == 0
        assert not any(call[0] == "send" for call in device.calls)
        assert not value.snapshot()["device"]["command_started"]
        if control == "isConnectOffline":
            value.persist({"one": {"pose": dict(value.pose), "transition": .1, "interval": 0}})
            widget(value, "motionstreamplay")
        else:
            widget(value, value.names[0], .2)
        tick(value)
        assert value.sample_ticks == 1 and device.calls[-1][0] == "send"
        assert value.snapshot()["device"]["command_started"]
        value.recorder.stop()
        log = [json.loads(line) for line in value.recorder.writer.path.read_text(encoding="utf-8").splitlines()]
        for kind in ("conversion", "downlink", "h7"):
            assert sum(row["type"] == kind for row in log) == 1
        assert not any(row.get("source") == "a32_enable_seed" for row in log)
    finally:
        value.close()


@pytest.mark.parametrize("control", ["a32Enable", "realtimeJointControl", "isConnectOffline"])
def test_no_feedback_blocks_enable_realtime_and_synchronized_playback(session, control):
    session.device.ready = False
    session.dispatch("connect", {})
    widget(session, session.names[0], .2)
    before = list(session.device.calls)
    with pytest.raises(ValueError, match="反馈"):
        widget(session, control, True)
    assert not session.armed and session.device.calls == before
    session.device.ready = True
    widget(session, control, True)
    assert session.device.enabled


def test_page_hidden_pauses_without_disconnect_and_checkbox_rearms(session):
    session.dispatch("connect", {})
    widget(session, "realtimeJointControl", True)
    tick(session)
    session.dispatch("stop", {"reason": "page_hidden"})
    assert session.connected and not session.armed and not session.device.enabled
    assert session.stop_reason == "page_hidden"
    assert not any(call[0] == "disconnect" for call in session.device.calls)
    assert session.snapshot()["controls"]["realtimeJointControl"]["enabled"]
    widget(session, "realtimeJointControl", True)
    tick(session)
    assert session.armed and session.device.calls[-1][0] == "enable"
    widget(session, session.names[0], .2)
    tick(session)
    assert session.device.calls[-1][0] == "send"


def test_control_worker_keeps_sending_while_qt_event_loop_is_not_pumped(tmp_path):
    device = FakeDevice()
    value = A32Session(tmp_path, device=device)
    try:
        value.dispatch("connect", {})
        widget(value, "realtimeJointControl", True)
        widget(value, value.names[0], .2)
        # Deliberately do no Qt event processing. The old QTimer would make
        # zero progress here; native motion must have its own send clock.
        time.sleep(.25)
        assert value.armed and value.sample_ticks >= 5
        assert value._io_thread.is_alive()
    finally:
        value.close()
    assert not value._io_thread.is_alive()


def test_offline_playback_keeps_keyframes_and_never_sends(session):
    first = dict(session.pose)
    last = dict(first)
    last[session.names[0]] = .25
    session.persist({"one": {"pose": first, "transition": 0, "interval": 0},
                     "two": {"pose": last, "transition": .1, "interval": 0}})
    widget(session, "motionstreamplay")
    for _ in range(500):
        tick(session)
        if not session.playback:
            break
    assert not session.playback
    assert session.pose == pytest.approx(last)
    assert not session.device.calls


def test_live_sequence_uses_sync_and_stops_without_more_target_frames(session):
    session.dispatch("connect", {"transport": "eth", "host": "192.0.2.1", "tcp_port": 5001})
    widget(session, "a32Enable")
    widget(session, "isConnectOffline", True)
    pose = dict(session.pose)
    pose[session.names[0]] = .3
    session.persist({"one": {"pose": pose, "transition": .4, "interval": 0}})
    widget(session, "motionstreamplay")
    for _ in range(500):
        tick(session)
        if not session.playback:
            break
    assert session.device.calls[-1][1] == pytest.approx(pose)
    session.dispatch("stop", {})
    before = len(session.device.calls)
    tick(session)
    assert len(session.device.calls) == before and not session.armed and not session.sync


def test_actual_sample_interval_and_stalled_loop_stop(session):
    session.dispatch("connect", {})
    widget(session, "a32Enable")
    widget(session, "realtimeJointControl", True)
    widget(session, session.names[0], .3)
    tick(session, .025)
    assert session.planner.delta_time == pytest.approx(.025)
    previous = dict(session.pose)
    tick(session, .101)
    assert session.pose == previous and not session.armed
    assert "100 ms" in session.message
    assert session.device.calls[-1] == ("stop",)
    assert session.connected and session.snapshot()["controls"]["realtimeJointControl"]["enabled"]
    widget(session, "realtimeJointControl", True)
    tick(session)
    assert session.armed and session.device.calls[-1][0] == "enable"
    widget(session, session.names[0], .3)
    tick(session)
    assert session.device.calls[-1][0] == "send"
    assert session.stop_reason == ""


def test_sdk_send_failure_keeps_last_successful_pose(session, monkeypatch):
    session.dispatch("connect", {})
    widget(session, "a32Enable")
    widget(session, "realtimeJointControl", True)
    widget(session, session.names[0], .3)
    tick(session)
    previous = dict(session.pose)
    def fail(*args, **kwargs):
        raise RuntimeError("simulated SDK write failure")
    monkeypatch.setattr(session.device, "send_joint_targets", fail)
    tick(session)
    assert session.pose == previous and not session.armed
    assert "write failure" in session.message


def test_future_trajectory_extremum_rejects_before_hardware_write(session):
    session.dispatch("connect", {})
    widget(session, "a32Enable")
    widget(session, "realtimeJointControl", True)
    name = session.names[0]
    lo, hi = session.limits[name]
    session.pose[name] = hi - .001
    session._reset_planner()
    velocity = [0.] * len(session.names)
    velocity[0] = 1.
    session.input.current_velocity = velocity
    widget(session, name, hi - .2)
    previous = dict(session.pose)
    tick(session, .0001)
    assert session.pose == previous and not session.armed
    assert "轨迹将超出" in session.message
    assert not any(call[0] == "send" for call in session.device.calls)


def test_homing_is_explicit_self_only_and_blocks_pose_changes(session):
    session.dispatch("connect", {})
    with pytest.raises(ValueError, match="Iq"):
        widget(session, "a32Home")
    widget(session, "a32HomeIq", 90)
    widget(session, "a32HomeSlot", 4)
    widget(session, "a32Home")
    assert session.device.calls == [("usb", "auto")]
    widget(session, "confirm")
    assert session.device.calls[-1] == ("home", 3, 90)
    assert not session.snapshot()["joints"][0]["enabled"]
    with pytest.raises(ValueError):
        widget(session, session.names[0], .1)
    widget(session, "a32Reset")
    assert session.device.calls[-1] == ("reset_home", 3)
    assert not session.device.homing and not session.device.enabled


@pytest.mark.parametrize("model_id", MODEL_IDS)
@pytest.mark.parametrize("side", ["left", "right"])
def test_model_files_urdf_and_import_are_isolated(tmp_path, model_id, side):
    s = A32Session(tmp_path, side, model_id=model_id, device=FakeDevice(side, model_id))
    try:
        state = s.snapshot()
        assert len(state["joints"]) == 20
        assert (server.ROOT / state["urdf"].lstrip("/")).is_file()
        s.persist({"one": {"pose": dict(s.pose), "transition": 1, "interval": .2}})
        before = s.path.read_bytes()
        valid = s.payload()
        invalids = []
        for key, value in (("_model_id", "TsenseI_v3_0"), ("_model_id", next(m for m in MODEL_IDS if m != model_id)), ("_hand_side", "right" if side == "left" else "left")):
            invalids.append({**valid, key: value})
        bad = copy.deepcopy(valid)
        bad["motions"]["one"]["pose"][s.names[0]] = float("nan")
        invalids.append(bad)
        bad = copy.deepcopy(valid)
        bad["motions"]["one"]["pose"].pop(s.names[0])
        invalids.append(bad)
        for value in invalids:
            with pytest.raises(ValueError):
                s.dispatch("import", {"text": json.dumps(value)})
            assert s.path.read_bytes() == before
        assert json.loads(before)["_model_id"] == model_id
        assert s.path.name == f"motions_a32{'_withid' if model_id.endswith('_withid') else ''}_{side}.json"
    finally:
        s.close()


def test_bridge_switch_lease_and_native_transport_do_not_use_browser_serial(bridge, monkeypatch):
    import a32_session
    monkeypatch.setattr(a32_session, "A32Device", FakeDevice)
    bridge.dispatch("model", {"model_id": MODEL_IDS[0]})
    try:
        assert bridge.dispatch("state", {})["serial_mode"] == "native-sdk"
        bridge.dispatch("connect", {"transport": "usb", "port": "COM_TEST"})
        assert bridge.a32.device.calls == [("usb", "COM_TEST")]
        with pytest.raises(ValueError):
            bridge.dispatch("model", {"model_id": "TsenseI_v2"})
        with pytest.raises(ValueError, match="浏览器"):
            bridge.call("serial_binary", b"invalid", client="viewer")
        bridge.last_seen = time.monotonic() - 6
        bridge.controller = "expired-owner"
        bridge.check_lease()
        assert bridge.a32.connected and not bridge.a32.armed
        assert bridge.controller is None
        assert bridge.a32.stop_reason == "controller_heartbeat_timeout"
        count = len(bridge.a32.device.calls)
        bridge.check_lease()
        assert len(bridge.a32.device.calls) == count  # No repeated estop every 50 ms.
        bridge.dispatch("disconnect", {})
        bridge.dispatch("widget", {"id": bridge.a32.generation + ":handSide", "value": 1})
        assert bridge.a32.side == "right"
        old_id = bridge.a32.generation + ":savemotion"
        bridge.dispatch("model", {"model_id": MODEL_IDS[1]})
        with pytest.raises(ValueError, match="模型"):
            bridge.dispatch("widget", {"id": old_id})
    finally:
        bridge.dispatch("model", {"model_id": "TsenseI_v2"})


def test_bundle_contains_both_a32_models_native_config_and_no_exe():
    import package
    manifest = {name: path for path, name in package.files()}
    assert "a32_session.py" in manifest
    assert "app/src/a32_driver/a32_hardware.json" in manifest
    assert "app/src/a32_driver/SDK_LICENSE.txt" in manifest
    assert "app/src/a32_driver/native_linux/CMakeLists.txt" in manifest
    assert "app/src/a32_driver/bin/liba32_native.so" in manifest
    assert "docs/a32_motion_play.md" in manifest and "docs/A32_NATIVE_SDK.md" in manifest
    for model in MODEL_IDS:
        folder = "TsenseI_hand_" + model.removeprefix("TsenseI_")
        assert f"app/assets/hands/{folder}/hand_profile.yaml" in manifest
        for side in ("left", "right"):
            assert f"app/assets/hands/{folder}/urdf/TsenseI_hand_{side}_{model.removeprefix('TsenseI_')}.urdf" in manifest
    assert all(path.is_file() for path in manifest.values())
    assert not any(path.suffix.lower() in {".exe", ".dll", ".ps1"} for path in manifest.values())
    assert not any("/a32_driver/build/" in name for name in manifest)


@pytest.mark.parametrize("model_id", MODEL_IDS)
def test_legacy_motion_coordinates_convert_once_without_rewriting_source(tmp_path, model_id):
    device = FakeDevice(model_id=model_id)
    session = A32Session(tmp_path, model_id=model_id, device=device, start=False)
    old_pose = {name: .2 if is_finger_swing(name) else 0. for name in session.limits}
    old_pose["left_hand_index_MCP_joint"] = 1.4
    old_pose["left_hand_index_DIP_joint"] = 1.57
    payload = {"_model_id": model_id, "_hand_side": "left",
               "motions": {"old": {"pose": old_pose, "transition": 1., "interval": .2}}}
    original = copy.deepcopy(payload)
    try:
        migrated = session.validate_file(payload)
        assert payload == original
        pose = migrated["old"]["pose"]
        assert pose["left_hand_index_swing_joint"] == pytest.approx(-.2 if model_id == "TsenseI_v3_2" else .2)
        assert pose["left_hand_index_MCP_joint"] == 1.4
        assert pose["left_hand_index_DIP_joint"] == 1.565
        assert pose["left_hand_index_PIP_joint"] == .005
        current = session.payload(migrated)
        assert current["_joint_coordinate_version"] == COORDINATE_VERSION
        assert current["_swing_axis_alignment"] == SWING_AXIS_ALIGNMENT
        assert session.validate_file(current) == migrated
        current["motions"]["old"]["pose"]["left_hand_index_swing_joint"] = -.4
        with pytest.raises(ValueError):
            session.validate_file(current)
        payload["motions"]["old"]["pose"]["left_hand_index_MCP_joint"] = 999
        with pytest.raises(ValueError):
            session.validate_file(payload)
    finally:
        session.close()


@pytest.mark.parametrize("model_id", MODEL_IDS)
@pytest.mark.parametrize("side", ["left", "right"])
def test_mcp_editor_keeps_requested_angles_with_sdk_scaled_downlink(tmp_path, model_id, side):
    from a32_driver import A32Device

    device = A32Device(side, model_id=model_id)
    session = A32Session(tmp_path, side, model_id=model_id, device=device, start=False)
    try:
        controls = {row["name"]: row for row in session.snapshot()["joints"]}
        for finger in ("thumb", "index", "mid", "ring", "pinky"):
            name = f"{side}_hand_{finger}_MCP_joint"
            assert session.limits[name] == (.005, 1.565)
            assert (controls[name]["min"], controls[name]["max"]) == (.005, 1.565)
            widget(session, name, 1.565)
        assert all(session.pose[f"{side}_hand_{finger}_MCP_joint"] == 1.565
                   for finger in ("thumb", "index", "mid", "ring", "pinky"))
        pose = dict(session.pose)
        targets = session._preview_targets(pose)
        for channel in device.channels:
            name = channel["joint"]
            if not name.endswith("_MCP_joint"):
                continue
            upper = 1.565
            assert targets["applied_pose"][name] == pytest.approx(upper, abs=1e-10, rel=0)
            assert targets["self"][channel["index"]] == pytest.approx(controls[name]["downlink"]["at_ui_max"])
        assert session.pose == pose
        assert device._backend is None
    finally:
        session.close()


@pytest.mark.parametrize("model_id", MODEL_IDS)
def test_positive_motion_preserves_pose_and_migrates_only_once(tmp_path, model_id):
    from a32_kinematics.joint_coordinates import POSITIVE_COORDINATE_VERSION, POSITIVE_SWING_AXIS_ALIGNMENT
    session = A32Session(tmp_path, model_id=model_id, device=FakeDevice(model_id=model_id), start=False)
    try:
        for old_angle in (0., .35, .7):
            pose = {name: old_angle if is_finger_swing(name) else value for name, value in session.pose.items()}
            data = {"_model_id": model_id, "_hand_side": "left",
                    "_joint_coordinate_version": POSITIVE_COORDINATE_VERSION,
                    "_swing_axis_alignment": POSITIVE_SWING_AXIS_ALIGNMENT,
                    "motions": {"old": {"pose": pose, "transition": .1, "interval": .2}}}
            original = copy.deepcopy(data)
            migrated = session.validate_file(data)
            assert data == original
            for name, value in migrated["old"]["pose"].items():
                if is_finger_swing(name):
                    assert value == pytest.approx(.35 - old_angle)
            assert session.validate_file(session.payload(migrated)) == migrated
        for invalid in (-.1, .71, float("nan"), True):
            data["motions"]["old"]["pose"]["left_hand_index_swing_joint"] = invalid
            with pytest.raises(ValueError):
                session.validate_file(data)
    finally:
        session.close()


def test_existing_signed_motion_keeps_numeric_command(tmp_path):
    device = FakeDevice()
    session = A32Session(tmp_path, device=device, start=False)
    try:
        pose = {**session.pose, "left_hand_index_swing_joint": .2}
        data = {"_model_id": session.model_id, "_hand_side": "left",
                "_joint_coordinate_version": COORDINATE_VERSION,
                "motions": {"saved": {"pose": pose, "transition": .1, "interval": .2}}}
        assert session.validate_file(data)["saved"]["pose"] == pose
        from a32_driver import A32Device
        targets = A32Device().prepare_targets(pose)
        import math
        assert targets["self"][15] == pytest.approx(math.degrees(.2))
        data["_swing_axis_alignment"] = "unknown"
        with pytest.raises(ValueError, match="轴向版本"):
            session.validate_file(data)
    finally:
        session.close()


@pytest.mark.parametrize("peer,origin,host,allowed", [
    ("127.0.0.1", "http://127.0.0.1:8765", "127.0.0.1:8765", True),
    ("192.0.2.10", "http://127.0.0.1:8765", "127.0.0.1:8765", False),
    ("127.0.0.1", "http://other.example:8765", "127.0.0.1:8765", False),
    ("127.0.0.1", "http://lan-proxy.example:8765", "lan-proxy.example:8765", False),
])
def test_http_native_connection_requires_loopback_same_origin(bridge, monkeypatch, peer, origin, host, allowed):
    import a32_session
    monkeypatch.setattr(a32_session, "A32Device", FakeDevice)
    bridge.dispatch("model", {"model_id": MODEL_IDS[0]})
    handler = object.__new__(server.Handler)
    handler.path = "/api/connect"
    body = json.dumps({"transport": "usb", "port": "COM_TEST"}).encode()
    handler.headers = {"Host": host, "Origin": origin,
                       "Content-Type": "application/json", "Content-Length": str(len(body)), "X-Motion-Client": "a" * 32}
    handler.client_address = (peer, 12345)
    handler.rfile = io.BytesIO(body)
    calls, replies = [], []
    def call(action, body=None, client=None, **kwargs):
        calls.append(kwargs)
        return bridge.dispatch(action, body or {}, client=client, **kwargs)
    handler.server = SimpleNamespace(bridge=SimpleNamespace(call=call), access=SimpleNamespace(token=lambda _headers: ""))
    handler.allowed = lambda: True  # The request already passed the LAN allowlist.
    handler.reply = lambda status, payload: replies.append((status, payload))
    bridge.dispatch("control", {}, client=handler.client_key())
    try:
        handler.do_POST()
        assert calls[-1]["trusted_local"] is allowed
        assert bridge.a32.connected is allowed
        assert replies[-1][0] == (200 if allowed else 400)
        if not allowed:
            assert not bridge.a32.device.calls
            assert "本机" in replies[-1][1]["error"]
    finally:
        bridge.dispatch("disconnect", {})
        bridge.dispatch("model", {"model_id": "TsenseI_v2"})


def test_remote_offline_edit_allowed_but_connected_widget_and_import_rejected(bridge, monkeypatch):
    import a32_session
    monkeypatch.setattr(a32_session, "A32Device", FakeDevice)
    bridge.dispatch("model", {"model_id": MODEL_IDS[0]})
    try:
        ident = bridge.a32.generation + ":" + bridge.a32.names[0]
        bridge.dispatch("widget", {"id": ident, "value": .2}, trusted_local=False)
        assert not bridge.dispatch("state", {}, trusted_local=False)["sdk_local_access"]
        bridge.dispatch("connect", {})
        count = len(bridge.a32.device.calls)
        for action, body in (("widget", {"id": ident, "value": .3}), ("import", {"text": json.dumps(bridge.a32.payload())}),
                             ("stop", {}), ("disconnect", {}), ("release", {})):
            with pytest.raises(ValueError, match="本机"):
                bridge.dispatch(action, body, trusted_local=False)
        assert len(bridge.a32.device.calls) == count
    finally:
        bridge.dispatch("disconnect", {})
        bridge.dispatch("model", {"model_id": "TsenseI_v2"})


@pytest.mark.parametrize("model_id", MODEL_IDS)
@pytest.mark.parametrize("control", ["realtimeJointControl", "isConnectOffline"])
def test_real_driver_reconnect_rebases_playback_from_new_feedback(tmp_path, model_id, control):
    import sys
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root / "tests"))
    from test_a32_driver import SimulatedNativeHand
    from a32_driver import A32Device
    case = json.loads((root / "tests/fixtures/a32_reconnect_feedback.json").read_text())
    backend = SimulatedNativeHand()
    device = A32Device(model_id=model_id, backend=backend)
    value = A32Session(tmp_path, model_id=model_id, device=device, clock=Clock(), start=False,
                       recording_dir=tmp_path / "logs")
    try:
        value.dispatch("connect", {})
        widget(value, control, True)
        device.send_joint_targets(case["previous_pose"], clamp=False)
        value.pose = dict(case["previous_pose"])
        value._reset_planner()
        value.dispatch("disconnect", {})
        value.dispatch("connect", {})
        backend.state = copy.deepcopy(case["feedback"])
        before = len(backend.calls)
        widget(value, control, True)
        seed = dict(value.pose)
        assert seed == pytest.approx(device.starting_pose())
        assert value.target_pose == seed
        for _ in range(3):
            tick(value)
        assert backend.calls[before:] == [("enable", True)]
        target = {**seed, "left_hand_index_PIP_joint": .2}
        if control == "isConnectOffline":
            value.persist({"new": {"pose": target, "transition": .1, "interval": 0}})
            widget(value, "motionstreamplay")
        else:
            widget(value, "left_hand_index_PIP_joint", .2)
        tick(value)
        own = next(call[1] for call in backend.calls[before:] if call[0] == "targets")
        for channel in device.channels:
            if channel["unit"] == "mm":
                assert own[channel["index"]] < .1
    finally:
        value.close()
    assert ("estop",) not in backend.calls and ("enable", False) not in backend.calls


@pytest.mark.parametrize("model_id", MODEL_IDS)
@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("compensation", [True, False])
def test_all_motion_play_motor_commands_are_compensated(tmp_path, model_id, side, compensation):
    import sys
    sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tests"))
    from test_a32_driver import SimulatedNativeHand
    from a32_driver import A32Device

    backend = SimulatedNativeHand()
    device = A32Device(side, model_id=model_id, backend=backend)
    s = A32Session(tmp_path, side, model_id=model_id, device=device, clock=Clock(), start=False)
    try:
        assert s.snapshot()["controls"]["a32BypassMotorCompensation"]["value"] is False
        widget(s, "a32BypassMotorCompensation", not compensation)
        assert s.snapshot()["controls"]["a32BypassMotorCompensation"]["value"] is (not compensation)
        s.dispatch("connect", {})
        widget(s, "realtimeJointControl", True)
        pose = {name: (lo + hi) / 3 for name, (lo, hi) in s.limits.items()}
        pose.update({name: -.1 for name in pose if is_finger_swing(name)})
        s.pose = dict(pose)
        s._reset_planner()
        name = f"{side}_hand_index_PIP_joint"
        widget(s, name, pose[name])
        tick(s)
        raw = device.prepare_targets(s.pose)
        sent = next(call for call in reversed(backend.calls) if call[0] == "targets")
        preview = s._preview_targets(s.pose)
        for channel in device.channels:
            joint = channel["joint"]
            finger = joint.removeprefix(f"{side}_hand_").split("_")[0]
            scale = MOTOR_FINGER_SCALES[side][finger] if compensation else 1.
            key, slot = ("self" if channel["channel"] == "self" else "mit_pos"), channel["index"]
            expected = raw[key][slot] * scale
            assert channel["limits"][0] <= expected <= channel["limits"][1]
            assert sent[1 if key == "self" else 2][slot] == pytest.approx(expected)
            assert preview[key][slot] == pytest.approx(expected)
            assert s.downlink_ranges[joint]["motor_scale"] == scale
        assert s.pose == pytest.approx(pose)
        s.persist({"test": {"pose": pose, "transition": .1, "interval": 1}})
        # Exercise real UI paths: load, initial pose, and synchronized playback.
        for action in ("loadmotion", "a32Init", "motionstreamplay"):
            if action == "motionstreamplay":
                widget(s, "isConnectOffline", True)
            widget(s, action)
            for _ in range(3):
                tick(s)
                sent = next(call for call in reversed(backend.calls) if call[0] == "targets")
                raw = device.prepare_targets(s.pose)
                for channel in device.channels:
                    key = "self" if channel["channel"] == "self" else "mit_pos"
                    lo, hi = channel["limits"]
                    expected = min(hi, max(lo, raw[key][channel["index"]] * s.motor_scales[channel["joint"]]))
                    assert sent[1 if key == "self" else 2][channel["index"]] == pytest.approx(expected)
        for enabled in (not compensation, compensation):
            count = len(backend.calls)
            widget(s, "a32BypassMotorCompensation", not enabled)
            assert len(backend.calls) == count  # The checkbox itself sends no frame.
            tick(s)
            sent = next(call for call in reversed(backend.calls) if call[0] == "targets")
            expected = device.prepare_targets(s.pose, motor_scales=s.motor_scales)
            assert sent[1] == pytest.approx(expected["self"])
            assert sent[2] == pytest.approx(expected["mit_pos"])
            assert s.snapshot()["controls"]["a32BypassMotorCompensation"]["value"] is (not enabled)
            for channel in device.channels:
                assert s.downlink_ranges[channel["joint"]]["motor_scale"] == s.motor_scales[channel["joint"]]
        for endpoint in (0, 1):
            edge = {name: limits[endpoint] for name, limits in s.limits.items()}
            raw = device.prepare_targets(edge)
            scaled = s._preview_targets(edge)
            for channel in device.channels:
                key = "self" if channel["channel"] == "self" else "mit_pos"
                lo, hi = channel["limits"]
                expected = min(hi, max(lo, raw[key][channel["index"]] * s.motor_scales[channel["joint"]]))
                assert scaled[key][channel["index"]] == pytest.approx(expected)
    finally:
        s.close()
