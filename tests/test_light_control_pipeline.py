from pathlib import Path
from types import SimpleNamespace
import sys


ROOT = Path(__file__).resolve().parents[1]
LIGHT_TEST = ROOT / "example" / "light_test"
PROJECT_SRC = ROOT / "src"
for path in (LIGHT_TEST, PROJECT_SRC):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

import control_pipeline as module  # noqa: E402
from control_pipeline import LightControlPipeline  # noqa: E402
from utils.ruckig_interpolator import (  # noqa: E402
    PhaseSynchronizedRuckigInterpolator,
    RuckigJointInterpolator,
)


class _FakeInterpolator:
    def __init__(self):
        self.target = None
        self.reset_count = 0
        self.synced = None
        self.sample_count = 0

    def push(self, cfg):
        self.target = dict(cfg)

    def sample(self):
        self.sample_count += 1
        return None if self.target is None else dict(self.target)

    def pop_reset_flag(self):
        return False

    @property
    def last_faults(self):
        return frozenset()

    def reset(self):
        self.reset_count += 1
        self.target = None

    def seed_ramp(self, start, end, duration_s=0.35):
        self.target = dict(end)

    def sync_current_pose(self, cfg, **kwargs):
        self.synced = dict(cfg)

    def shutdown(self):
        pass


class _FakeGuard:
    def __init__(self, events=None):
        self.events = events if events is not None else []

    def filter(self, cfg, **kwargs):
        self.events.append(("guard", dict(cfg)))
        return dict(cfg)

    def reset(self):
        pass

    def stats(self):
        return {"total_ticks": len(self.events)}


class _FakeUsb:
    def __init__(self, *, linear_sent=14):
        self.is_connected = True
        self.current_device = None
        self.linear_sent = linear_sent
        self.calls = []
        self.slot = SimpleNamespace(home_label=0, iq_ref=0.0, pos_actual_mm=4.0)

    def reconcile_connection(self):
        return self.is_connected

    def send_joint_angles(self, cfg, **kwargs):
        self.calls.append(("send", dict(cfg), dict(kwargs)))
        return True

    def send_hand_type(self, side):
        self.calls.append(("hand_type", side))
        return True

    def send_all_lm_homing(self, cmd, iq_seek):
        self.calls.append(("linear_home", cmd, iq_seek))
        return self.linear_sent

    def send_rm_drive_motor(self, bus, motor, enabled):
        self.calls.append(("drive", bus, motor, enabled))
        return True

    def send_rm_zero(self, bus, motor):
        self.calls.append(("rotary_zero", bus, motor))
        return True

    def read_latest_telemetry(self):
        return {"rotary": [self.slot] * 7}

    def emergency_stop(self):
        self.calls.append(("emergency",))
        return True

    def disconnect(self):
        self.is_connected = False


def test_control_tick_orders_ruckig_conversion_guard_stroke_and_send(monkeypatch):
    events = []
    usb = _FakeUsb()
    interp = _FakeInterpolator()
    guard = _FakeGuard(events)

    def convert(cfg, hand_side):
        events.append(("convert", dict(cfg), hand_side))
        return {"left_hand_index_MCP_joint": cfg["right_hand_index_MCP_joint"]}

    def strokes(cfg):
        events.append(("strokes", dict(cfg)))
        return [1.25] * 14, [2.5] * 7

    monkeypatch.setattr(module, "convert_ui_to_hardware", convert)
    monkeypatch.setattr(module, "joint_angles_to_hand_positions", strokes)
    control = LightControlPipeline(
        usb=usb,
        interpolator=interp,
        downlink_guard=guard,
        hand_side="right",
    )
    control.set_tracking_active(True)
    control.push_target({"right_hand_index_MCP_joint": 0.4})
    assert control.set_armed(True)
    control._control_tick(10.0)

    assert [event[0] for event in events] == ["convert", "guard", "strokes"]
    assert usb.calls[-1][0] == "send"
    snapshot = control.snapshot()
    assert snapshot["ruckig_joint_angles_rad"]["right_hand_index_MCP_joint"] == 0.4
    assert snapshot["hardware_joint_angles_rad"]["left_hand_index_MCP_joint"] == 0.4
    assert snapshot["linear_strokes_mm"] == [1.25] * 14
    assert snapshot["send_ticks"] == 1


def test_post_ruckig_collision_gate_reseeds_interpolator_to_accepted_pose(
    monkeypatch,
):
    class SampleGate:
        def reset(self):
            pass

        def project(self, cfg):
            safe = dict(cfg)
            safe["right_hand_index_MCP_joint"] = 0.2
            return safe, {"limited": True, "resolution": "stop"}

    usb = _FakeUsb()
    interp = _FakeInterpolator()
    monkeypatch.setattr(
        module,
        "convert_ui_to_hardware",
        lambda cfg, hand_side: dict(cfg),
    )
    monkeypatch.setattr(
        module,
        "joint_angles_to_hand_positions",
        lambda cfg: ([0.0] * 14, [0.0] * 7),
    )
    control = LightControlPipeline(
        usb=usb,
        interpolator=interp,
        downlink_guard=_FakeGuard(),
    )
    control.set_tracking_active(True)
    control.set_sample_collision_guard(SampleGate(), mode="stop")
    control.push_target({"right_hand_index_MCP_joint": 0.8})
    assert control.set_armed(True)
    control._control_tick(10.0)

    assert interp.synced == {"right_hand_index_MCP_joint": 0.2}
    assert usb.calls[-1][1]["right_hand_index_MCP_joint"] == 0.2
    snapshot = control.snapshot()
    assert snapshot["sample_collision"]["limited"]
    assert snapshot["ruckig_output_joint_angles_rad"]["right_hand_index_MCP_joint"] == .8
    assert snapshot["ruckig_joint_angles_rad"]["right_hand_index_MCP_joint"] == .2
    assert snapshot["direct_joint_angles_rad"] is None


def test_control_boundary_forces_pinky_rotation_to_zero(monkeypatch):
    usb = _FakeUsb()
    interp = _FakeInterpolator()
    monkeypatch.setattr(
        module,
        "convert_ui_to_hardware",
        lambda cfg, hand_side: dict(cfg),
    )
    monkeypatch.setattr(
        module,
        "joint_angles_to_hand_positions",
        lambda cfg: ([0.0] * 14, [0.0] * 7),
    )
    control = LightControlPipeline(
        usb=usb,
        interpolator=interp,
        downlink_guard=_FakeGuard(),
    )
    control.set_tracking_active(True)
    control.push_target({"right_hand_pinky_rota_joint": 1.1})
    assert interp.target["right_hand_pinky_rota_joint"] == 0.0
    assert control.set_armed(True)
    control._control_tick(10.0)
    assert usb.calls[-1][1]["right_hand_pinky_rota_joint"] == 0.0


def test_handedness_invalidation_revokes_target_and_physical_send(monkeypatch):
    usb = _FakeUsb()
    interp = _FakeInterpolator()
    monkeypatch.setattr(
        module,
        "convert_ui_to_hardware",
        lambda cfg, hand_side: dict(cfg),
    )
    monkeypatch.setattr(
        module,
        "joint_angles_to_hand_positions",
        lambda cfg: ([0.0] * 14, [0.0] * 7),
    )
    control = LightControlPipeline(
        usb=usb,
        interpolator=interp,
        downlink_guard=_FakeGuard(),
    )
    control.set_tracking_active(True)
    control.push_target({"right_hand_index_MCP_joint": 0.4})
    assert control.set_armed(True)

    control.invalidate_target("hand mismatch")
    control._control_tick(10.0)

    snapshot = control.snapshot()
    assert not snapshot["target_ready"]
    assert not snapshot["armed"]
    assert snapshot["target_block_reason"] == "hand mismatch"
    assert interp.sample_count == 0
    assert not any(call[0] == "send" for call in usb.calls)


def test_one_key_zero_starts_linear_then_waits_for_rotary_telemetry_edge():
    usb = _FakeUsb()
    control = LightControlPipeline(
        usb=usb,
        interpolator=_FakeInterpolator(),
        downlink_guard=_FakeGuard(),
    )
    assert control.start_one_key_zero(750.0)
    assert usb.calls[:2] == [
        ("hand_type", "right"),
        ("linear_home", module.LM_HOMING_START, 750.0),
    ]
    assert not control.snapshot()["armed"]

    control._homing_t0 = 0.0
    control._tick_one_key_zero(module.ONEKEY_LM_DELAY_S + 0.01)
    assert usb.calls[-1] == ("drive", 1, 1, True)
    control._homing_deadline = 0.0
    control._tick_one_key_zero(module.ONEKEY_LM_DELAY_S + 0.02)
    assert usb.calls[-1] == ("rotary_zero", 1, 1)

    usb.slot.home_label = 0
    control._tick_one_key_zero(module.ONEKEY_LM_DELAY_S + 0.03)
    usb.slot.home_label = 1
    control._tick_one_key_zero(module.ONEKEY_LM_DELAY_S + 0.04)
    assert usb.calls[-1] == ("drive", 1, 0, True)


def test_one_key_zero_constants_and_timeout_skip_match_main_app_v3():
    assert module.ONEKEY_RM_ZERO_SEQUENCE == (
        (1, 1),
        (1, 0),
        (0, 1),
        (0, 0),
        (3, 0),
        (3, 1),
    )
    assert module.ONEKEY_LM_DELAY_S == 1.0

    usb = _FakeUsb()
    control = LightControlPipeline(
        usb=usb,
        interpolator=_FakeInterpolator(),
        downlink_guard=_FakeGuard(),
    )
    assert control.start_one_key_zero()
    control._homing_phase = "rm_seeking"
    control._homing_step = (1, 1)
    control._homing_queue = [(3, 0)]
    control._homing_total = 2
    control._homing_index = 1
    control._homing_t0 = 0.0

    control._tick_one_key_zero(module.ONEKEY_RM_ZERO_TIMEOUT_S + 0.1)

    assert control._homing_fail_caps == ["SPI2 M1"]
    assert usb.calls[-1] == ("drive", 3, 0, True)
    assert control.snapshot()["homing_active"] is True


def test_one_key_zero_rejects_partial_linear_command_batch():
    usb = _FakeUsb(linear_sent=13)
    control = LightControlPipeline(
        usb=usb,
        interpolator=_FakeInterpolator(),
        downlink_guard=_FakeGuard(),
    )
    assert not control.start_one_key_zero()
    snapshot = control.snapshot()
    assert snapshot["homing_result"] == "failed"
    assert "13/14" in snapshot["last_error"]
    assert usb.calls[-1][0:2] == ("linear_home", module.LM_HOMING_RESET)


def test_stopping_one_key_zero_sends_hardware_stop_command():
    usb = _FakeUsb()
    control = LightControlPipeline(
        usb=usb,
        interpolator=_FakeInterpolator(),
        downlink_guard=_FakeGuard(),
    )
    assert control.start_one_key_zero()
    assert control.cancel_one_key_zero()
    assert usb.calls[-1][0:2] == ("linear_home", module.LM_HOMING_RESET)


def test_ruckig_can_replan_from_collision_accepted_pose():
    interp = RuckigJointInterpolator()
    name = "right_hand_index_MCP_joint"
    try:
        interp.push({name: 1.0})
        for _ in range(20):
            assert interp.sample() is not None
        interp.sync_current_pose({name: 0.2})
        assert interp.peek()[name] == 0.2
        following = interp.sample()
        assert following is not None
        assert abs(following[name] - 0.2) < 0.01
    finally:
        interp.shutdown()


def test_phase_synchronized_ruckig_stays_on_joint_space_segment():
    first_name = "left_hand_index_MCP_joint"
    second_name = "left_hand_index_swing_joint"
    interp = PhaseSynchronizedRuckigInterpolator(
        joint_names=(first_name, second_name)
    )
    try:
        interp.push({first_name: 1.0, second_name: 0.2})
        checked = 0
        for _ in range(700):
            sample = interp.sample()
            assert sample is not None
            if sample[first_name] > 1.0e-5:
                assert abs(sample[second_name] - 0.2 * sample[first_name]) < 1.0e-7
                checked += 1
        assert checked > 100
        assert interp.last_faults == frozenset()
    finally:
        interp.shutdown()


def test_loss_cancels_inflight_target_and_resumes_from_accepted_pose(monkeypatch):
    clock = [0.0]
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])
    monkeypatch.setattr(module, "convert_ui_to_hardware", lambda cfg, hand_side: dict(cfg))
    monkeypatch.setattr(module, "joint_angles_to_hand_positions", lambda cfg: ([0.0] * 14, [0.0] * 7))
    name = "right_hand_index_MCP_joint"
    interp = PhaseSynchronizedRuckigInterpolator(joint_names=[name])
    control = LightControlPipeline(usb=_FakeUsb(), interpolator=interp, downlink_guard=_FakeGuard())
    control.set_tracking_active(True)
    for i in range(3):
        clock[0] = i * 0.03
        decision = control.observe_tracking_frame(True)
    generation = decision["generation"]
    assert control.push_target({name: 1.3}, tracking_generation=generation)
    # Lose observation during acceleration, before the faster trajectory brakes.
    for i in range(1, 75):
        clock[0] = 0.06 + i * 0.002
        if i % 15 == 0:
            control.observe_tracking_frame(True)
        control._control_tick(clock[0])
    lost_pose = control.current_ui_pose()[name]
    assert lost_pose > 0
    control.observe_tracking_frame(False)
    assert not control.push_target({name: 1.5}, tracking_generation=generation)
    for i in range(600):
        clock[0] += 0.002
        control._control_tick(clock[0])
    stopped_pose = control.current_ui_pose()[name]
    assert lost_pose <= stopped_pose < 1.3
    assert control.snapshot()["tracking_transition"]["phase"] == "holding"
    for i in range(3):
        clock[0] += 0.03
        decision = control.observe_tracking_frame(True)
        assert decision["allow_retarget"] is (i == 2)
    assert decision["reset_planner"]
    assert control.push_target({name: 0.05}, tracking_generation=decision["generation"])
    clock[0] += 0.002
    control._control_tick(clock[0])
    assert abs(control.current_ui_pose()[name] - stopped_pose) < 1e-5
    assert control.snapshot()["tracking_transition"]["phase"] == "recovering"


def test_watchdog_rejects_a_solver_result_computed_before_timeout(monkeypatch):
    clock = [0.0]
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])
    control = LightControlPipeline(usb=_FakeUsb(), interpolator=_FakeInterpolator(), downlink_guard=_FakeGuard())
    control.set_tracking_active(True)
    for _ in range(3):
        decision = control.observe_tracking_frame(True)
    clock[0] = 0.31
    control._control_tick(0.31)
    assert not control.push_target({"right_hand_index_MCP_joint": 1.0}, tracking_generation=decision["generation"])
    assert control.snapshot()["tracking_transition"]["reason"] == "observation_timeout"


def test_new_sample_guard_rebases_at_last_accepted_pose():
    class SampleGate:
        rebased = None

        def rebase(self, pose):
            self.rebased = dict(pose)

        def reset(self):
            raise AssertionError("must not reset an existing accepted pose to zero")

    control = LightControlPipeline(usb=_FakeUsb(), interpolator=_FakeInterpolator(), downlink_guard=_FakeGuard())
    control._latest_ui = {"right_hand_index_MCP_joint": 0.4}
    gate = SampleGate()
    control.set_sample_collision_guard(gate, mode="avoid")
    assert gate.rebased == control.current_ui_pose()


def test_sample_safety_failure_blocks_old_target_and_all_physical_sends():
    class BrokenGate:
        def reset(self):
            pass

        def project(self, pose):
            raise RuntimeError("collision backend unavailable")

    usb = _FakeUsb()
    interp = _FakeInterpolator()
    control = LightControlPipeline(usb=usb, interpolator=interp, downlink_guard=_FakeGuard())
    control.set_tracking_active(True)
    control.set_sample_collision_guard(BrokenGate(), mode="avoid")
    control.push_target({"right_hand_index_MCP_joint": 0.4})
    assert control.set_armed(True)
    control._control_tick(1.0)
    control._control_tick(1.002)
    assert interp.sample_count == 1
    assert not control.snapshot()["target_ready"]
    assert not control.snapshot()["armed"]
    assert control.snapshot()["tracking_transition"]["phase"] == "blocked"
    assert not any(row[0] == "send" for row in usb.calls)


def test_sdk_snapshot_and_log_use_post_guard_command_angles(tmp_path, monkeypatch):
    import json
    from utils.control_session_recorder import ControlSessionRecorder

    joint = "left_hand_index_MCP_joint"
    class ClippingGuard(_FakeGuard):
        def filter(self, cfg, **kwargs):
            return {**cfg, joint: .25}

    recorder = ControlSessionRecorder()
    path = recorder.start(output_dir=tmp_path)
    usb = _FakeUsb()
    control = LightControlPipeline(usb=usb, interpolator=_FakeInterpolator(),
        downlink_guard=ClippingGuard(), hand_side="left", recorder=recorder)
    monkeypatch.setattr(module, "joint_angles_to_hand_positions", lambda cfg: ([0.] * 14, [0.] * 7))
    try:
        control.set_tracking_active(True)
        control.push_target({joint: .8}, frame_id=13)
        assert control.set_armed(True)
        control._control_tick(10.)
        state = control.snapshot()
        assert state["ruckig_output_joint_angles_rad"][joint] == .8
        assert state["command_joint_angles_rad"][joint] == .25
        assert usb.calls[-1][1][joint] == .25
    finally:
        recorder.stop()
    angles = [row for row in map(json.loads, path.read_text().splitlines()) if row["type"] == "joint_angles"]
    assert [(row["stage"], row["joints"][joint]) for row in angles] == [("ruckig", .8), ("sdk", .25)]
    assert angles[-1]["sent"] is True and angles[-1]["frame_id"] == 13
