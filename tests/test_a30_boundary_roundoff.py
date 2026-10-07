"""Regression for the frame-1258 boundary roundoff that latched A3 tracking."""
import json
import struct
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src')]
from a30_kinematics import A30StrokeMapper
from a30_kinematics.mapper import JOINT_LIMIT_EPS_RAD
from hand_profiles import load_hand_profile
from online_control.online_motion_track.a30.control import A30Control


@pytest.mark.parametrize('side', ['left', 'right'])
def test_strict_mapper_normalizes_only_roundoff_at_every_boundary(side):
    m = A30StrokeMapper(side)
    initial = dict.fromkeys(m.joint_names, 0.)
    for name, (lo, hi) in zip(m.joint_names, m._limits):
        for boundary, direction in ((lo, -1), (hi, 1)):
            pose = dict(initial)
            pose[name] = boundary + direction * (JOINT_LIMIT_EPS_RAD * .5)
            result = m.convert(pose)
            assert result.joint_angles_rad[name] == boundary
            pose[name] = boundary + direction * (JOINT_LIMIT_EPS_RAD * 2)
            with pytest.raises(ValueError, match='AngleLimit'):
                m.convert(pose)


@pytest.mark.parametrize('side', ['left', 'right'])
def test_recorded_boundary_finish_stays_live_and_accepts_restored_hand(side):
    f = json.loads((ROOT / 'tests/fixtures/a30_boundary_roundoff_1258.json').read_text(encoding='utf-8'))
    c = A30Control(load_hand_profile('TsenseI_v3_0'), side, 'mink', start=False)
    rename = lambda pose: {n.replace('left_hand_', f'{side}_hand_'): q for n,q in pose.items()}
    try:
        c._seed(rename(f['pose'])); c.target = rename(f['target']); c.frame_id = f['frame_id']
        for key, value in f['input'].items(): setattr(c.input, key, value)
        # Tracking lost the selected hand; it should finish this target, then
        # accept the next one without rearming or clearing a hard-fault latch.
        c.hold_target()
        for tick in range(f['steps_to_failure'] + 250):
            c.step(.002)
            assert c.valid, (tick, c.error)
            assert c.phase != 'blocked' and len(c.wire) == 108
            assert [c.pose[n] for n in c.names] == c.input.current_position
            assert c.input.current_position == c.output.new_position
            assert all(lo <= c.pose[n] <= hi for n, (lo, hi) in c.limits.items())
        name = f'{side}_hand_mid_MCP_joint'
        assert c.pose[name] == 0.
        with_stream = c.open_stream(c.epoch)
        packet = next(with_stream); with_stream.close()
        assert struct.unpack_from('<f', packet, 28 + 4*c.names.index(name))[0] == 0.
        target = dict(c.pose); target[name] = .2
        c.push(target, 1289)
        for _ in range(150):
            c.step(.002)
            assert c.valid, c.error
        assert c.frame_id == 1289 and c.pose[name] > .05
    finally:
        c.close()


def test_roundoff_is_normalized_before_collision_and_shared_with_next_input():
    c = A30Control(load_hand_profile('TsenseI_v3_0'), 'left', 'mink', start=False)
    name = 'left_hand_mid_MCP_joint'; i = c.names.index(name)
    update, project = c.otg.update, c.guard.project
    checked = []
    def noisy_sample(parameters, output):
        result = update(parameters, output)
        output.new_position[i] = -1.4140965853708407e-9
        output.trajectory.position_extrema[i] = SimpleNamespace(min=output.new_position[i], max=0.)
        return result
    def collision(pose):
        checked.append(pose[name])
        return project(pose)
    try:
        c.otg.update = noisy_sample; c.guard.project = collision
        c.step(.002)
        assert c.valid and checked == [0.]
        assert c.boundary_roundoffs == 1 and c.limit_recoveries == 0
        assert c.pose[name] == c.output.new_position[i] == c.input.current_position[i] == 0.
        assert c.mapper.convert(c.pose).joint_angles_rad[name] == 0.
    finally:
        c.close()


@pytest.mark.parametrize('upper', [False, True])
def test_real_overshoot_replans_instead_of_being_normalized(upper):
    c = A30Control(load_hand_profile('TsenseI_v3_0'), 'left', 'off', start=False)
    name = 'left_hand_mid_MCP_joint'; i = c.names.index(name)
    update = c.otg.update
    calls = 0
    def overshoot(parameters, output):
        nonlocal calls
        result = update(parameters, output); calls += 1
        if calls == 1:
            q = c.limits[name][int(upper)] + (1 if upper else -1)*2*JOINT_LIMIT_EPS_RAD
            output.new_position[i] = q
            output.trajectory.position_extrema[i] = SimpleNamespace(min=q, max=q)
        return result
    try:
        c.otg.update = overshoot
        c.step(.002)
        assert c.valid and calls > 1
        assert c.limit_recoveries == 1
        assert c.last_limit_event['joints'][0]['joint'] == name
        if c.last_boundary_roundoff:
            assert all(abs(r['raw_rad']-r['applied_rad']) <= JOINT_LIMIT_EPS_RAD
                       for r in c.last_boundary_roundoff['joints'])
        assert c.pose[name] == 0.
    finally:
        c.close()
