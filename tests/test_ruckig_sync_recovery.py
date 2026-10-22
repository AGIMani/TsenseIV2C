"""Replay the 2026-10-09 A3.2 synchronization failure without hardware."""
import json
import math
import sys
from pathlib import Path

import pytest
import ruckig

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src')]
from online_control.online_motion_track.a30.grouped_ruckig import GroupedOutput, GroupedRuckig

CAPTURE = json.loads((ROOT / 'tests/fixtures/ruckig_sync_step2.json').read_text())
NAMES = [f"left_hand_pinky_{joint}_joint" for joint in ('MCP', 'swing', 'PIP', 'DIP')]


def captured_input():
    parameters = ruckig.InputParameter(4)
    for field in GroupedRuckig.fields:
        setattr(parameters, field, CAPTURE[field])
    return parameters


@pytest.mark.parametrize('dt', [.001, .002, CAPTURE['control_dt_s'], .004])
def test_captured_step2_failure_recovers_with_same_state_and_limits(dt):
    parameters = captured_input()
    original = {field: getattr(parameters, field) for field in GroupedRuckig.fields}
    solver = GroupedRuckig(NAMES, dt)
    output = GroupedOutput(4)
    assert solver.update(parameters, output) in (ruckig.Result.Working, ruckig.Result.Finished)

    # Check the recovered trajectory itself, not just the emitted endpoint.
    trajectory = solver.solvers[0][3].trajectory
    initial = trajectory.at_time(0.)
    for actual, field in zip(initial, ('current_position', 'current_velocity', 'current_acceleration')):
        assert actual == pytest.approx(original[field], abs=1e-12)
    previous_acceleration = initial[2]
    sample_dt = trajectory.duration / 500
    for tick in range(1, 501):
        positions, velocities, accelerations = trajectory.at_time(tick * sample_dt)
        assert all(math.isfinite(value) for vector in (positions, velocities, accelerations) for value in vector)
        for index in range(4):
            assert abs(velocities[index]) <= original['max_velocity'][index] + 1e-9
            assert abs(accelerations[index]) <= original['max_acceleration'][index] + 1e-9
            assert abs(accelerations[index] - previous_acceleration[index]) <= original['max_jerk'][index] * sample_dt + 1e-9
        previous_acceleration = accelerations
    assert positions == pytest.approx(original['target_position'], abs=1e-10)
    assert velocities == pytest.approx([0.] * 4, abs=1e-10)
    assert accelerations == pytest.approx([0.] * 4, abs=1e-10)
    assert {field: getattr(parameters, field) for field in GroupedRuckig.fields} == original

    # Collision retries reuse this candidate without advancing another tick.
    before = [output.new_position[:], output.new_velocity[:], output.new_acceleration[:]]
    solver.update(parameters, output)
    assert [output.new_position, output.new_velocity, output.new_acceleration] == before

    # Subsequent jittered ticks and a new target remain usable after recovery.
    for tick in range(120):
        output.pass_to_input(parameters)
        if tick == 10:
            parameters.target_position = [.2, .3, .25, .15]
        solver.delta_time = [.001, CAPTURE['control_dt_s'], .004][tick % 3]
        assert solver.update(parameters, output) in (ruckig.Result.Working, ruckig.Result.Finished)
    assert output.new_position == pytest.approx(parameters.target_position, abs=1e-10)
    assert output.new_velocity == [0.] * 4
    assert output.new_acceleration == [0.] * 4


def test_invalid_input_is_not_hidden_by_synchronization_recovery():
    parameters = captured_input()
    parameters.max_jerk = [-1.] * 4
    solver = GroupedRuckig(NAMES, .002)
    with pytest.raises(ruckig.RuckigError, match='jerk'):
        solver.update(parameters, GroupedOutput(4))
