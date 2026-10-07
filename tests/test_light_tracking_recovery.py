"""Loss/reacquisition tests. No cameras, serial ports or hardware are opened."""

from pathlib import Path
import sys

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
for path in (ROOT / "example" / "light_test", ROOT / "src"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))

from tracking_transition import TrackingTransition  # noqa: E402
from utils.ruckig_interpolator import (  # noqa: E402
    PhaseSynchronizedRuckigInterpolator,
)


class _MotionSpy:
    ready = True
    stationary = True

    def __init__(self):
        self.brakes = 0
        self.holds = []
        self.scales = []
        self.pose = {"joint": 0.4}

    def brake(self):
        self.brakes += 1
        self.stationary = False

    def sync_current_pose(self, pose):
        self.pose = dict(pose)
        self.stationary = True

    def push(self, pose):
        self.holds.append(dict(pose))

    def peek(self):
        return dict(self.pose)

    def set_speed_scale(self, scale):
        self.scales.append(scale)

    def reset(self):
        self.ready = False


def _confirmed_start(state, current, now=0.0):
    for i in range(3):
        decision = state.observe(True, now=now + i * 0.03, current=current)
        assert decision["allow_retarget"] is (i == 2)
    return decision


def test_reacquisition_waits_for_brake_and_discards_old_motion_only_once():
    interp = _MotionSpy()
    state = TrackingTransition(interp)
    _confirmed_start(state, interp.pose)
    interp.stationary = False
    generation = state.generation
    state.observe(False, now=0.10, current=interp.pose)
    assert state.phase == "braking"
    assert state.generation == generation + 1
    for i in range(3):
        state.observe(False, now=0.11 + i * 0.01, current=interp.pose)
    assert interp.brakes == 1
    for i in range(5):
        decision = state.observe(True, now=0.15 + i * 0.03, current=interp.pose)
        assert not decision["allow_retarget"]
        assert state.phase == "braking"
    interp.stationary = True
    interp.pose = {"joint": 0.48}  # stopping distance, not the lost target
    state.tick(now=0.30, current=interp.pose)
    assert state.phase == "holding"
    decision = state.observe(True, now=0.31, current=interp.pose)
    assert decision["allow_retarget"] and decision["reset_planner"]
    assert interp.holds[-1] == {"joint": 0.48}
    assert state.speed_scale == 0.35
    assert not state.observe(True, now=0.34, current=interp.pose)["reset_planner"]


def test_confirmation_requires_consecutive_fresh_frames_and_speed_recovers_gradually():
    interp = _MotionSpy()
    state = TrackingTransition(interp)
    assert not state.observe(True, now=0.0, current=interp.pose)["allow_retarget"]
    state.observe(False, now=0.03, current=interp.pose)
    assert state.valid_streak == 0
    _confirmed_start(state, interp.pose, now=0.06)
    assert state.phase == "recovering"
    scales = []
    for i in range(12):
        now = 0.12 + i * 0.1
        state.observe(True, now=now, current=interp.pose)
        state.tick(now=now, current=interp.pose)
        scales.append(state.speed_scale)
    assert scales[0] == 0.35
    assert 0.35 < scales[5] < 1.0
    assert scales[-1] == 1.0
    assert np.all(np.diff(scales) >= 0.0)
    assert state.phase == "tracking"


def test_camera_watchdog_brakes_without_a_new_detection_callback():
    interp = _MotionSpy()
    state = TrackingTransition(interp)
    _confirmed_start(state, interp.pose)
    state.tick(now=0.37, current=interp.pose)
    assert state.phase == "braking"
    assert state.reason == "observation_timeout"
    assert interp.brakes == 1
    state.tick(now=0.8, current=interp.pose)
    assert interp.brakes == 1


def test_confirmation_streak_expires_across_a_camera_stall():
    interp = _MotionSpy()
    state = TrackingTransition(interp)
    state.observe(True, now=0.0, current=interp.pose)
    state.observe(True, now=0.03, current=interp.pose)
    decision = state.observe(True, now=2.0, current=interp.pose)
    assert not decision["allow_retarget"]
    assert state.valid_streak == 1


def test_missing_frames_do_not_release_wrong_hand_interlock():
    interp = _MotionSpy()
    state = TrackingTransition(interp)
    _confirmed_start(state, interp.pose)
    state.block("hand_mismatch", interp.pose)
    for i in range(10):
        state.observe(False, now=1.0 + i * 0.1, current=interp.pose)
        state.tick(now=1.0 + i * 0.1, current=interp.pose)
        assert state.phase == "blocked"
    assert interp.brakes == 0
    decision = _confirmed_start(state, interp.pose, now=2.0)
    assert decision["reset_planner"]


@pytest.mark.parametrize("side", ["left", "right"])
def test_ruckig_loss_brake_preserves_state_and_respects_motion_limits(side):
    name = f"{side}_hand_index_MCP_joint"
    interp = PhaseSynchronizedRuckigInterpolator(joint_names=[name])
    interp.seed_ramp({name: 0.1}, {name: 1.5})
    # Brake early enough that jerk-limited stopping still precedes the target
    # with the shared 10/80/200 limits.
    for _ in range(40):
        interp.sample()
    before = np.array([
        interp._input.current_position[0],
        interp._input.current_velocity[0],
        interp._input.current_acceleration[0],
    ])
    assert before[1] > 0.1
    interp.brake()
    np.testing.assert_allclose([
        interp._input.current_position[0],
        interp._input.current_velocity[0],
        interp._input.current_acceleration[0],
    ], before, atol=1e-12)
    samples = [before]
    for _ in range(800):
        pose = interp.sample()
        assert pose is not None and name in pose
        assert not interp.last_faults
        samples.append([
            pose[name],
            interp._input.current_velocity[0],
            interp._input.current_acceleration[0],
        ])
    samples = np.asarray(samples)
    assert interp.stationary
    assert samples[-1, 0] < 1.5 - 0.1  # no chasing the lost destination
    assert np.min(np.diff(samples[:, 0])) >= -1e-10  # no return-to-hold reversal
    assert np.max(np.abs(samples[:, 1])) <= interp._input.max_velocity[0] + 1e-6
    assert np.max(np.abs(samples[:, 2])) <= interp._input.max_acceleration[0] + 1e-6
    assert np.max(np.abs(np.diff(samples[:, 2]) / 0.002)) <= interp._input.max_jerk[0] + 1e-5
    held = interp.peek()[name]
    assert interp.sample()[name] == pytest.approx(held, abs=1e-10)
    interp.set_speed_scale(0.35)
    interp.push({name: 0.0})
    following = interp.sample()[name]
    assert 0 < held - following < 1e-5
    assert not interp.last_faults


def test_loss_before_first_control_tick_cannot_execute_the_old_target():
    name = "left_hand_index_MCP_joint"
    interp = PhaseSynchronizedRuckigInterpolator(joint_names=[name])
    state = TrackingTransition(interp)
    _confirmed_start(state, None)
    interp.push({name: 1.5})
    state.observe(False, now=0.07, current=None)
    for i in range(50):
        assert interp.sample()[name] == pytest.approx(0.0, abs=1e-12)
        state.tick(now=0.07 + i * 0.002, current=None)
    assert interp.stationary
