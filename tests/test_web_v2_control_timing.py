"""Real Ruckig timing regressions; no camera or physical serial connection."""
from pathlib import Path
from types import SimpleNamespace
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT / "src"), str(ROOT / "online_control/online_motion_track/pipeline_v2")]
from control_pipeline import LightControlPipeline
from utils import control_clock
from utils.motion_limits import DEFAULT_MOTION_LIMITS
from utils.ruckig_interpolator import PhaseSynchronizedRuckigInterpolator, RuckigJointInterpolator

JOINT = "left_hand_mid_PIP_joint"
REFERENCE_MOTION_LIMITS = (4.0, 30.0, 200.0)


def set_flexion_limits(monkeypatch, limits):
    # The fixed response-time oracle below was measured with these explicit
    # limits. Product defaults and developer-shell overrides may change.
    for suffix, value in zip(("VEL", "ACC", "JERK"), limits):
        monkeypatch.setenv(f"RUCKIG_FLEXION_{suffix}", str(value))


@pytest.mark.parametrize("factory", [PhaseSynchronizedRuckigInterpolator, RuckigJointInterpolator])
def test_step_response_depends_on_elapsed_time_not_number_of_calls(factory, monkeypatch):
    set_flexion_limits(monkeypatch, REFERENCE_MOTION_LIMITS)

    def reach(schedule, elapsed):
        interp = factory([JOINT])
        interp.seed_ramp({JOINT: 1.2}, {JOINT: 0.0})
        total = 0.0
        for n in range(3000):
            dt = schedule[n % len(schedule)]
            total += dt
            pose = interp.sample(**({"delta_time": dt} if elapsed else {}))
            assert not interp.last_faults
            if abs(pose[JOINT]) < np.radians(5):
                return total
        pytest.fail("trajectory did not reach target")
    ideal = reach([.002], True)
    irregular = reach([.016, .012, .001, .016], True)
    old = reach([.016, .012, .001, .016], False)
    assert .43 < ideal < .46
    assert abs(irregular-ideal) < .02
    assert old > ideal * 5


@pytest.mark.parametrize("limits", [REFERENCE_MOTION_LIMITS, DEFAULT_MOTION_LIMITS],
                         ids=["reference", "shared-defaults"])
def test_variable_steps_preserve_motion_constraints_and_phase_path(limits, monkeypatch):
    set_flexion_limits(monkeypatch, limits)
    names = [JOINT, "left_hand_index_MCP_joint"]
    interp = PhaseSynchronizedRuckigInterpolator(names)
    interp.seed_ramp(dict.fromkeys(names, 0.0), {names[0]: 1.2, names[1]: .6})
    velocity, acceleration, jerk = limits
    for n in range(80):
        pose = interp.sample(delta_time=(.002, .015, .008)[n % 3])
        assert not interp.last_faults
        assert pose[names[1]] == pytest.approx(pose[names[0]] / 2, abs=1e-8)
        assert max(abs(v) for v in interp._output.new_velocity) <= velocity + 1e-8
        assert max(abs(a) for a in interp._output.new_acceleration) <= acceleration + 1e-8
        assert max(abs(j) for j in interp._output.new_jerk) <= jerk + 1e-8
    assert pose == pytest.approx({names[0]: 1.2, names[1]: .6})


def test_control_uses_elapsed_time_and_holds_after_long_pause():
    interp = PhaseSynchronizedRuckigInterpolator([JOINT])
    control = LightControlPipeline(usb=SimpleNamespace(is_connected=False),
                                  interpolator=interp, hand_side="left")
    control.set_tracking_active(True)
    interp.seed_ramp({JOINT: 1.2}, {JOINT: 0.0})
    control.push_target({JOINT: 0.0})
    for now in np.arange(0, .481, .016):
        control._control_tick(float(now))
    assert control.current_ui_pose()[JOINT] < np.radians(5)
    timing = control.snapshot()["control_timing"]
    assert timing["trajectory_elapsed_s"] == pytest.approx(.482)
    pose = control.current_ui_pose()
    control._control_tick(5.0)
    state = control.snapshot()
    assert state["control_timing"]["timeout_count"] == 1
    assert not state["armed"] and not state["target_ready"]
    assert control.current_ui_pose() == pose
    control.set_tracking_active(True)
    control.push_target({JOINT: 1.0})
    control._control_tick(20.0)
    assert control.snapshot()["control_timing"]["last_step_ms"] == pytest.approx(2)
    assert abs(control.current_ui_pose()[JOINT]-pose[JOINT]) < 1e-5


def test_elapsed_output_still_passes_sample_collision_gate():
    class Gate:
        calls = 0
        def project(self, pose):
            self.calls += 1
            safe = dict(pose)
            safe[JOINT] = min(.1, pose[JOINT])
            return safe, {"limited": safe != pose}
    interp = PhaseSynchronizedRuckigInterpolator([JOINT])
    control = LightControlPipeline(usb=SimpleNamespace(is_connected=False),
                                  interpolator=interp, hand_side="left")
    control.set_tracking_active(True)
    gate = Gate()
    control.set_sample_collision_guard(gate, mode="stop")
    interp.seed_ramp({JOINT: 0.0}, {JOINT: 1.2})
    control.push_target({JOINT: 1.2})
    for now in np.arange(0, .8, .016):
        control._control_tick(float(now))
        assert control.current_ui_pose()[JOINT] <= .1
    assert gate.calls == 50
    assert interp.peek()[JOINT] <= .1


def test_windows_timer_request_released_on_exception(monkeypatch):
    calls = []
    fake = SimpleNamespace(timeBeginPeriod=lambda n: calls.append(("begin", n)) or 0,
                           timeEndPeriod=lambda n: calls.append(("end", n)) or 0)
    monkeypatch.setattr(control_clock.sys, "platform", "win32")
    monkeypatch.setattr(control_clock.ctypes, "WinDLL", lambda _: fake, raising=False)
    with pytest.raises(RuntimeError):
        with control_clock.precise_control_timer() as enabled:
            assert enabled
            raise RuntimeError("worker failed")
    assert calls == [("begin", 1), ("end", 1)]
