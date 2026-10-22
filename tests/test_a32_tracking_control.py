"""A32 native downlink lifecycle; no hardware is connected by these tests."""
from pathlib import Path
import json
import sys
import time
from contextlib import nullcontext
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
pytest.importorskip("ruckig")

from a32_kinematics import A32StrokeMapper
from hand_profiles import A32_MODEL_IDS, load_hand_profile
from online_control.online_motion_track.a32.control import A32Control
from online_control.online_motion_track.main import Application, WebSession, model_description


class FakeDevice:
    def __init__(self, model_id, side="left"):
        self.mapper = A32StrokeMapper(side, model_id=model_id)
        self._limits = dict(zip(self.mapper.joint_names, self.mapper._limits))
        self.is_connected = True
        self.enabled = False
        self.writes = []
        self.fail_write = False
        self.state = {"rx_ok": 1, "telemetry_age_ms": 0, "mit": []}
        self.feedback = load_hand_profile(model_id).init_pose(side)
        self.feedback.update({n: .15 for n in self._limits if "PIP" in n})

    def joint_limits(self):
        return dict(self._limits)

    def constrain_pose(self, pose):
        return self.mapper.convert(pose, clamp=True).joint_angles_rad

    def starting_pose(self, fallback_pose=None):
        return dict(self.feedback)

    def enable_all(self, *, start_pose=None):
        self.enabled = True

    def stop(self):
        self.enabled = False

    def close(self):
        self.is_connected = False

    def get_state(self):
        return {"connected": self.is_connected, "ready": True, "enabled": self.enabled, **self.state}

    def send_joint_targets(self, pose, *, clamp):
        assert not clamp
        if self.fail_write:
            raise RuntimeError("injected transport failure")
        assert self.enabled
        self.writes.append(dict(pose))
        return self.mapper.convert(pose).as_dict()


class PassGuard:
    def __init__(self):
        self.guard = SimpleNamespace(rebase=lambda pose: None)
        self.filter = SimpleNamespace(sync_pose_from_dict=lambda pose: None, data=SimpleNamespace(ncon=0))
        self.diagnostics = {"mode": "stop", "limited": False}

    def project(self, pose):
        return dict(pose)


def control(model_id="TsenseI_v3_2", side="left"):
    device = FakeDevice(model_id, side)
    item = A32Control(load_hand_profile(model_id), side, "off", device=device, start=False)
    item.guard = PassGuard()
    item.start_tracking()
    return item, device


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("collision_mode", ["stop", "mink"])
def test_protected_control_can_seed_before_first_collision_frame(model_id, side, collision_mode):
    profile = load_hand_profile(model_id)
    device = FakeDevice(model_id, side)
    item = A32Control(profile, side, collision_mode, device=device, start=False)
    try:
        pose = profile.init_pose(side)
        assert item.guard.guard._previous_safe == pose
        # Startup and feedback reseeding precede the first projected frame.
        pose[f"{side}_hand_index_PIP_joint"] = .1
        item._seed(pose)
        assert item.guard.guard._previous_safe == pose
        assert item.guard.project(pose) == pose
        assert item.guard.diagnostics["output_max_penetration_mm"] == 0
        item._seed(pose)
        assert not item.armed and not device.enabled and not device.writes
    finally:
        item.close()


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
@pytest.mark.parametrize("collision_mode", ["off", "monitor"])
def test_unprotected_mode_requires_manual_arm_sends_and_stops_on_heartbeat_loss(model_id, collision_mode):
    device = FakeDevice(model_id)
    item = A32Control(load_hand_profile(model_id), "left", collision_mode, device=device, start=False)
    try:
        assert item.guard is None
        item.start_tracking()
        target = {**device.feedback, "left_hand_index_PIP_joint": .6}
        item.push(target, 1)
        item.step(.002)
        assert not item.armed and not device.enabled and not device.writes
        item.arm(True)
        item.push(target, 2)
        for _ in range(10):
            item.step(.002)
        assert item.armed and device.enabled and len(device.writes) == 10
        assert device.writes[-1]["left_hand_index_PIP_joint"] > device.feedback["left_hand_index_PIP_joint"]
        snapshot = item.snapshot()
        assert snapshot["collision_mode"] == collision_mode
        assert not snapshot["sample_collision_ready"]
        assert snapshot["sample_collision"]["checked"] is False
        assert snapshot["sample_collision"]["protection_enabled"] is False
        item.client_at = time.monotonic() - item.CLIENT_TIMEOUT - 1
        item.step(.002)
        assert not item.armed and not device.enabled and len(device.writes) == 10
        assert "心跳超时" in item.last_error
        item.push(target, 3)
        item.step(.002)
        assert len(device.writes) == 10  # Fresh gestures never re-arm a fault.
    finally:
        item.close()


@pytest.mark.parametrize("collision_mode", ["off", "monitor"])
def test_unprotected_mode_retains_sdk_telemetry_and_motor_fault_gates(collision_mode):
    device = FakeDevice("TsenseI_v3_2")
    item = A32Control(load_hand_profile("TsenseI_v3_2"), "left", collision_mode, device=device, start=False)
    try:
        with pytest.raises(ValueError, match="识别"):
            item.arm(True)
        item.start_tracking()
        device.state["telemetry_age_ms"] = -1
        with pytest.raises(ValueError, match="遥测"):
            item.arm(True)
        device.state["telemetry_age_ms"] = 0
        device.state["mit"] = [{"online": True, "fault": 1}]
        with pytest.raises(ValueError, match="故障"):
            item.arm(True)
        assert not device.enabled and not device.writes
        device.state["mit"] = []
        item.arm(True)
        device.state["mit"] = [{"online": True, "fault": 1}]
        item.step(.002)
        assert not item.armed and not device.enabled and not device.writes
        assert "故障" in item.last_error
    finally:
        item.close()


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
@pytest.mark.parametrize("side", ["left", "right"])
def test_shared_motion_limits_cover_all_a32_axes(model_id, side):
    item, _ = control(model_id, side)
    try:
        assert item.input.max_velocity == [20.] * 20
        assert item.input.max_acceleration == [400.] * 20
        assert item.input.max_jerk == [2400.] * 20
        assert item.PERIOD == .002
    finally:
        item.close()


@pytest.mark.parametrize('side', ['left', 'right'])
def test_captured_sync_failure_recovers_before_guard_and_sdk_write(side):
    capture = json.loads((ROOT / 'tests/fixtures/ruckig_sync_step2.json').read_text())
    item, device = control(capture['model_id'], side)
    checked = []
    try:
        item.arm(True)
        indices = item.otg.groups[capture['finger']]
        for field in ('current_position', 'current_velocity', 'current_acceleration',
                      'target_position', 'target_velocity', 'target_acceleration'):
            values = list(getattr(item.input, field))
            for index, value in zip(indices, capture[field]):
                values[index] = value
            setattr(item.input, field, values)
        item.pose = dict(zip(item.names, item.input.current_position))
        item.target = dict(zip(item.names, item.input.target_position))
        item.target_at = time.monotonic()
        item.frame_id = capture['frame_id']
        def check(pose):
            checked.append(dict(pose))
            return dict(pose)
        item.guard.project = check

        item.step(capture['control_dt_s'])
        assert item.armed and device.enabled and not item.last_error
        assert len(device.writes) == 1 and checked == device.writes
        assert item.pose == device.writes[-1]
        assert item.ruckig_output_joint_angles_rad is not None
        assert item.limit_recoveries == 0
        for index, target in zip(indices, capture['target_position']):
            assert item.pose[item.names[index]] == pytest.approx(target, abs=1e-10)
        for name, value in device.writes[-1].items():
            lo, hi = item.limits[name]
            assert lo <= value <= hi
        for _ in range(20):
            item.step(.002)
        assert item.armed and not item.last_error and len(device.writes) == 21
    finally:
        item.close()


@pytest.mark.parametrize('as_exception', [True, False])
def test_unrecoverable_sync_failure_stops_without_sending(as_exception):
    import ruckig
    item, device = control()
    calls = []
    def fail(parameters, output):
        calls.append(parameters.duration_discretization)
        if as_exception:
            raise ruckig.RuckigError('[ruckig] error in step 2 in dof: 0')
        return ruckig.Result.ErrorSynchronizationCalculation
    try:
        item.arm(True)
        indices, _, parameters, output = item.otg.solvers[0]
        item.otg.solvers[0] = (indices, SimpleNamespace(update=fail, reset=lambda: None), parameters, output)
        item.step(.002)
        assert calls == [ruckig.DurationDiscretization.Continuous, ruckig.DurationDiscretization.Discrete]
        assert not item.armed and not device.enabled and not device.writes
        assert item.last_error
    finally:
        item.close()


def test_worker_uses_high_resolution_clock_when_windows_monotonic_repeats(monkeypatch):
    from online_control.online_motion_track.a32 import control as module
    coarse_time = 100.
    fine_time = [100.]
    waits = [0]
    intervals = []
    def fine_clock():
        fine_time[0] += .001
        return fine_time[0]
    def wait(_):
        waits[0] += 1
        return waits[0] > 20
    monkeypatch.setattr(module, 'time', SimpleNamespace(
        monotonic=lambda: coarse_time, perf_counter=fine_clock))
    monkeypatch.setattr(module, 'precise_control_timer', nullcontext)
    monkeypatch.setattr(module, 'control_waiter', lambda done: nullcontext(done.wait))
    worker = SimpleNamespace(PERIOD=.005, done=SimpleNamespace(wait=wait), step=intervals.append)
    module.A32Control._run(worker)
    assert len(intervals) == 20 and all(dt > 0 for dt in intervals)


def test_rejected_input_does_not_repeat_stop_after_control_is_disarmed():
    item, device = control()
    stops = []
    stop = device.stop
    device.stop = lambda: (stops.append(True), stop())
    try:
        item.arm(True)
        item.reject_input('invalid pose')
        assert len(stops) == 1 and not item.armed and not device.enabled
        item.reject_input('another invalid pose')
        assert len(stops) == 1 and item.last_error == 'another invalid pose'
        item.stop()  # Explicit emergency/disable still always reaches hardware.
        assert len(stops) == 2
    finally:
        item.close()


def test_failed_hardware_stop_remains_pending_for_retry():
    item, device = control()
    original = device.stop
    try:
        item.arm(True)
        device.stop = lambda: (_ for _ in ()).throw(RuntimeError('I/O failed'))
        with pytest.raises(RuntimeError, match='I/O failed'):
            item.reject_input('bad input')
        assert item.stop_pending and not item.armed
        device.stop = original
        item.reject_input('bad input')
        assert not item.stop_pending and not device.enabled
    finally:
        device.stop = original
        item.close()


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
def test_feedback_seeded_smooth_samples_and_stop(model_id):
    item, device = control(model_id)
    try:
        target = {n: .6 if "PIP" in n else 0. for n in item.names}
        item.push(target, 5)
        item.step(.005)
        assert not device.writes  # Preview never writes.
        item.arm(True)
        assert item.pose == device.feedback
        item.push(target, 6)
        for _ in range(8):
            item.step(.005)
        assert len(device.writes) == 8
        assert device.writes[0]["left_hand_index_PIP_joint"] < .151
        assert device.writes[-1]["left_hand_index_PIP_joint"] > .15
        assert item.snapshot()["ruckig_joint_angles_rad"] == device.writes[-1]
        pending = item.snapshot(after_tick=3)
        assert [s["tick"] for s in pending["command_samples"]] == list(range(4, 9))
        assert pending["command_samples"][-1]["joints"] == device.writes[-1]
        assert pending["dropped_command_samples"] == 0
        item.arm(False)
        item.step(.005)
        assert len(device.writes) == 8 and not device.enabled
    finally:
        item.close()


def test_tracking_required_but_no_hand_needed_and_failure_latches():
    item, device = control()
    try:
        item.tracking_started = False
        with pytest.raises(ValueError, match="识别"):
            item.arm(True)
        item.start_tracking()
        item.arm(True)
        device.fail_write = True
        item.step(.005)
        assert not item.armed and not device.enabled
        assert "transport failure" in item.last_error
        item.push(device.feedback, 1)
        item.step(.005)
        assert not device.writes and not item.armed
    finally:
        item.close()


def test_missing_browser_heartbeat_stops_device():
    item, device = control()
    try:
        item.push(device.feedback, 1)
        item.arm(True)
        item.client_at = time.monotonic() - 3
        item.step(.005)
        assert not device.writes and not device.enabled
        assert "心跳" in item.last_error
    finally:
        item.close()


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
@pytest.mark.parametrize("delayed_dt", [.101, .5, 2.])
def test_delayed_sample_keeps_downlink_and_does_not_catch_up(model_id, delayed_dt):
    item, device = control(model_id)
    reference, expected_device = control(model_id)
    try:
        target = {n: .6 if "PIP" in n else 0. for n in item.names}
        for worker in (item, reference):
            worker.push(target, 5)
            worker.arm(True)
            worker.push(target, 6)
            worker.step(worker.PERIOD)
        item.step(delayed_dt)
        reference.step(reference.MAX_PLANNER_STEP)
        assert item.armed and device.enabled and not item.last_error
        assert item.ticks == 2 and len(device.writes) == 2
        assert device.writes[-1] == pytest.approx(expected_device.writes[-1])
        assert item.otg.delta_time == item.MAX_PLANNER_STEP
        item.step(item.PERIOD)
        reference.step(reference.PERIOD)
        assert device.writes[-1] == pytest.approx(expected_device.writes[-1])
    finally:
        item.close()
        reference.close()


@pytest.mark.parametrize("dt", [0., -.01, float("nan"), float("inf")])
def test_invalid_sample_interval_still_stops(dt):
    item, device = control()
    try:
        item.push(device.feedback, 1)
        item.arm(True)
        item.step(dt)
        assert not item.armed and not device.enabled and not device.writes
        assert "间隔无效" in item.last_error
    finally:
        item.close()


def test_delayed_sample_still_honors_real_input_and_client_expiry():
    item, device = control()
    try:
        item.push({n: .6 if "PIP" in n else 0. for n in item.names}, 1)
        item.arm(True)
        item.push({n: .6 if "PIP" in n else 0. for n in item.names}, 2)
        item.step(item.PERIOD)
        written = dict(item.pose)
        item.target_at = time.monotonic() - 1.
        item.step(.5)
        assert item.armed and item.pose["left_hand_index_PIP_joint"] > written["left_hand_index_PIP_joint"]
        assert not item.snapshot()["hand_recent"]
        assert item.snapshot()["target_ready"]
        item.client_at = time.monotonic() - 3.
        count = len(device.writes)
        item.step(2.)
        assert not item.armed and len(device.writes) == count
        assert "心跳" in item.last_error
    finally:
        item.close()


def test_missing_hand_finishes_last_target_and_resumes():
    item, device = control()
    try:
        item.push({n: .6 if "PIP" in n else 0. for n in item.names}, 1)
        item.arm(True)
        target = {n: .6 if "PIP" in n else 0. for n in item.names}
        item.push(target, 2)
        item.step(.01)
        written = dict(item.pose)
        item.hold()
        item.step(.01)
        assert item.pose["left_hand_index_PIP_joint"] > written["left_hand_index_PIP_joint"]
        assert item.snapshot()["target_ready"]
        for _ in range(400):
            item.touch()
            item.step(.005)
        assert item.pose == pytest.approx(item.device.constrain_pose(target))
        assert item.armed
        item.push(device.feedback, 3)
        for _ in range(400):
            item.touch()
            item.step(.005)
        assert item.pose == pytest.approx(device.feedback)
    finally:
        item.close()


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
def test_real_driver_waits_for_current_feedback_before_start_and_resume(model_id):
    from a32_driver import A32Device, A32DriverError
    from test_a32_driver import SimulatedNativeHand
    backend = SimulatedNativeHand()
    device = A32Device(model_id=model_id, backend=backend)
    device.connect_usb("SIM_ONLY")
    item = A32Control(load_hand_profile(model_id), "left", "off", device=device, start=False)
    item.guard = PassGuard()
    item.start_tracking()
    try:
        assert item.snapshot()["telemetry_seen"]
        backend.state["telemetry_age_ms"] = -1
        before = list(backend.calls)
        with pytest.raises(A32DriverError, match="反馈"):
            item.arm(True)
        assert not item.armed and backend.calls == before
        backend.state["telemetry_age_ms"] = 0
        item.arm(True)
        seed = dict(item.pose)
        item.push({**seed, "left_hand_index_PIP_joint": .6}, 1)
        for _ in range(8):
            item.step(.005)
        assert item.armed and item.ticks == 8
        assert item.pose["left_hand_index_PIP_joint"] > seed["left_hand_index_PIP_joint"]
        item.stop()
        backend.state["self"][0]["online"] = False
        before = list(backend.calls)
        with pytest.raises(A32DriverError, match="反馈"):
            item.arm(True)
        assert not item.armed and backend.calls == before
        backend.state["self"][0]["online"] = True
        item.arm(True)
        assert item.pose == seed  # Fresh stationary feedback, not the last command.
        item.step(.005)
        assert item.armed and item.ticks == 9
    finally:
        item.close()


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
@pytest.mark.parametrize("side", ["left", "right"])
def test_both_web_models_retarget_and_serve_all_meshes(model_id, side):
    description = model_description(side, model_id)
    assert len(description["initial_pose"]) == 20
    assert sum(j["type"] == "revolute" for j in description["joints"]) == 20
    item = WebSession(side, recording=False, model_id=model_id, collision_mode="off")
    try:
        solver = item.pipeline.session.retargeter
        result = solver.retarget(solver.fk.compute(), include_artifacts=False)
        assert result.success and len(result.joint_values) == 20
        assert item.pipeline.stroke_mapper.preview(result.joint_values)["model_id"] == model_id
    finally:
        item.close()


def test_public_preview_never_acquires_native_hardware():
    app = Application(public=True)
    try:
        created = app.create({"model_id": "TsenseI_v3_2_withid", "recording": False})
        assert not hasattr(app.get(created["session"]), "control")
        with pytest.raises(ValueError, match="本地服务"):
            app.a32_command({"session": created["session"], "action": "connect"})
    finally:
        app.close()


def test_disconnect_error_releases_control_owner():
    app = Application()
    def command(action, data):
        assert action == "disconnect"
        raise RuntimeError("estop failed after link loss")
    fake = SimpleNamespace(model_id="TsenseI_v3_2", control=object(), command=command, close=lambda: None)
    app.sessions["session"] = fake
    app.hardware_owner = "session"
    try:
        with pytest.raises(RuntimeError, match="link loss"):
            app.a32_command({"session": "session", "action": "disconnect"})
        assert app.hardware_owner is None
    finally:
        app.close()


def test_first_packet_required_then_stale_partial_feedback_allowed():
    item, device = control()
    try:
        device.state.update(rx_ok=123, telemetry_age_ms=-1)
        with pytest.raises(ValueError, match="一包有效"):
            item.arm(True)
        assert not device.enabled
        device.state.update(rx_ok=1, telemetry_age_ms=10)
        assert item.snapshot()["telemetry_seen"]
        device.state.update(rx_ok=0, telemetry_age_ms=-1, self=[], mit=[])
        item.arm(True)
        item.step(.005)
        assert item.armed and len(device.writes) == 1
        item.stop()
        item.reset_connection()
        with pytest.raises(ValueError, match="一包有效"):
            item.arm(True)
    finally:
        item.close()


@pytest.mark.parametrize("state,reason", [
    ({"mit": [{"online": True, "fault": 4}]}, "故障"),
    ({"mit": [{"online": True, "state": 5}]}, "故障"),
    ({"hand_side": "right"}, "左右手"),
    ({"homing_active": True}, "找零"),
])
def test_device_issue_blocks_arm_and_stops_active_stream(state, reason):
    item, device = control()
    try:
        clean = dict(device.state)
        device.state.update(state)
        with pytest.raises(ValueError, match=reason):
            item.arm(True)
        device.state = clean
        item.arm(True)
        device.state.update(state)
        item.step(.005)
        assert not item.armed and not device.enabled and not device.writes
        assert reason in item.last_error
    finally:
        item.close()


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
def test_collision_holds_one_finger_and_retries_after_neighbour_moves(model_id):
    item, device = control(model_id)
    index, mid = 'left_hand_index_PIP_joint', 'left_hand_mid_PIP_joint'
    device.feedback = item.profile.init_pose('left')
    index_boundary = device.feedback[index] + .0001
    mid_clearance = device.feedback[mid] + .1
    item.guard.guard._contact_fingers = lambda contacts: {'index', 'mid'}
    def barrier(pose):
        hit = pose[index] > index_boundary and pose[mid] < mid_clearance
        item.guard.diagnostics = {'limited': hit, 'contacts': [{'test': 'index_mid'}] if hit else []}
        return dict(item.pose) if hit else dict(pose)
    item.guard.project = barrier
    try:
        item.arm(True)
        target = {**item.pose, index: .2}
        item.push(target, 0)
        for _ in range(100):
            item.touch()
            item.step(.005)
        assert set(item.finger_holds) == {'index'}
        target[mid] = .3
        for frame in range(40):
            item.touch()
            item.push(target, frame + 1)
            for _ in range(10):
                item.step(.005)
                assert item.armed, item.last_error
                assert not (item.pose[index] > index_boundary and item.pose[mid] < mid_clearance)
        assert not item.finger_holds
        assert item.pose[index] == pytest.approx(.2, abs=1e-7)
        assert item.pose[mid] == pytest.approx(.3, abs=1e-7)
    finally:
        item.close()


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
def test_limit_overshoot_replans_before_any_sdk_write(model_id):
    item, device = control(model_id)
    try:
        item.arm(True)
        name = 'left_hand_index_PIP_joint'
        lo, hi = item.limits[name]
        seed = {**item.pose, name: hi - .0001}
        item._seed(seed)
        item.push({**seed, name: lo}, 1)
        i = item.names.index(name)
        velocity = list(item.input.current_velocity)
        velocity[i] = 1.
        item.input.current_velocity = velocity
        item.step(.005)
        assert item.armed, item.last_error
        assert item.limit_recoveries > 0
        assert len(device.writes) == 1
        for n, value in device.writes[0].items():
            assert item.limits[n][0] <= value <= item.limits[n][1]
    finally:
        item.close()


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
def test_real_a32_collision_model_checks_every_published_sample(model_id):
    device = FakeDevice(model_id)
    device.feedback = load_hand_profile(model_id).init_pose('left')
    item = A32Control(load_hand_profile(model_id), 'left', 'stop', device=device, start=False)
    try:
        item.start_tracking()
        item.arm(True)
        target = {n: lo + .8 * (hi - lo) for n, (lo, hi) in item.limits.items()}
        for frame in range(30):
            item.touch()
            item.push(target, frame)
            for _ in range(5):
                item.step(.005)
                assert item.armed, item.last_error
                item.guard.filter.sync_pose_from_dict(item.pose)
                assert item.guard.filter.data.ncon == 0
        assert len(device.writes) == 150
    finally:
        item.close()
