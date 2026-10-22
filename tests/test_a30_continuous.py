"""Continuous A3 control: braking bounds, local finger holds and resumption."""
import math
import json
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src')]
from hand_profiles import load_hand_profile
from online_control.online_motion_track.a30.control import A30Control


def test_stream_timestamp_matches_planning_clock_boundary():
    c = A30Control(load_hand_profile('TsenseI_v3_0'), 'left', 'off', start=False)
    try:
        c.step(.002, sample_at=1000.)
        assert c.produced_at == 1000.
        c.step(.007, sample_at=1000.007)
        assert c.produced_at == 1000.007
    finally:
        c.close()


def test_target_collision_guard_holds_only_contact_fingers_with_real_geometry():
    from online_control.online_motion_track.pipeline_v3_0.collision import StopCollisionGuard
    case = json.loads((ROOT / 'tests/fixtures/a30_collision_685.json').read_text(encoding='utf-8'))
    g = StopCollisionGuard(load_hand_profile('TsenseI_v3_0'), 'left', finger_stop=True)
    g.current = dict(case['last_safe_pose']); g.guard.rebase(g.current)
    result = g.project(case['target'])
    assert set(g.diagnostics['held_fingers']) == {'thumb', 'pinky'}
    for n in result:
        expected = case['last_safe_pose'][n] if '_thumb_' in n or '_pinky_' in n else case['target'][n]
        assert result[n] == pytest.approx(expected, abs=1e-10)
    assert g.diagnostics['output_max_penetration_mm'] == 0


def test_mink_automatic_resume_inside_clearance_keeps_feasible_target_seed():
    from online_control.online_motion_track.main import Application
    case = json.loads((ROOT / 'tests/fixtures/a30_collision_685.json').read_text(encoding='utf-8'))
    app = Application(public=True)
    try:
        session = app.create({'model_id':'TsenseI_v3_0', 'recording':False, 'collision_mode':'mink'})
        obj = app.get(session['session']); obj.control.close()
        g = obj.pipeline.mink_guard; original = g.q.copy()
        assert not g._feasible(g._vector(case['last_safe_pose']))
        state = obj.command('browser_rebase', {'telemetry':bytes(484).hex(),
            'pose':case['last_safe_pose'], 'resume_from_commanded_pose':True})
        assert state['target_ready']
        assert g.q == pytest.approx(original)
        obj.control.push(g.project(case['target']), 685)
        for _ in range(100):
            obj.control.step(.002)
            assert obj.control.valid, obj.control.error
        # Manual enabling retains the existing stricter feedback requirement.
        with pytest.raises(ValueError, match='Mink'):
            obj.command('browser_rebase', {'telemetry':bytes(484).hex(), 'pose':case['last_safe_pose']})
    finally:
        app.close()


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('sign', [-1, 1])
def test_braking_extrema_replan_before_crossing_position_limit(side, sign):
    c = A30Control(load_hand_profile('TsenseI_v3_0'), side, 'off', start=False)
    try:
        joint = f'{side}_hand_pinky_swing_joint'
        other = f'{side}_hand_mid_PIP_joint'
        initial = c.profile.init_pose(side); initial[joint] = sign*.349
        c._seed(initial)
        velocity = list(c.input.current_velocity)
        # Keep the braking distance above the 0.001 rad limit gap.
        velocity[c.names.index(joint)] = sign*.2
        c.input.current_velocity = velocity
        target = dict(initial); target[joint] = sign*.3; target[other] = .2
        c.push(target, 1)
        for _ in range(300):
            c.step(.002)
            assert c.valid, c.error
            assert -.35 <= c.pose[joint] <= .35
            assert len(c.wire) == 108
        assert c.pose[other] > .05
        assert c.limit_recoveries > 0
        assert any(row['joint'] == joint for row in c.last_limit_event['joints'])
    finally:
        c.close()


@pytest.mark.parametrize('side', ['left', 'right'])
def test_only_contact_fingers_hold_and_new_gesture_recovers(side):
    c = A30Control(load_hand_profile('TsenseI_v3_0'), side, 'stop', start=False)
    try:
        old = dict(c.pose)
        stopped = {n for n in c.names if '_thumb_' in n or '_pinky_' in n}
        target = dict(old)
        target[f'{side}_hand_thumb_PIP_joint'] = .2
        target[f'{side}_hand_pinky_DIP_joint'] = .2
        other = f'{side}_hand_mid_PIP_joint'; target[other] = .25
        def contact(pose):
            hit = any(abs(pose[n]-old[n]) > 1e-8 for n in stopped)
            c.guard.diagnostics = {'limited': hit, 'contacts': [
                {'g1':f'{side}_hand_thumb_PIP_link_capsule', 'g2':f'{side}_hand_pinky_DIP_link_capsule'}] if hit else []}
            return dict(old) if hit else dict(pose)
        c.guard.project = contact
        c.push(target, 1)
        for _ in range(100):
            c.step(.002)
            assert c.valid and len(c.wire) == 108
            assert all(c.pose[n] == old[n] for n in stopped)
        assert set(c.finger_holds) == {'thumb', 'pinky'}
        assert c.pose[other] > .01
        c.push(target, 2); c.hold_target(); c.step(.002)
        assert set(c.finger_holds) == {'thumb', 'pinky'}
        c.guard.project = lambda pose: dict(pose)
        target[f'{side}_hand_thumb_PIP_joint'] = .1
        target[f'{side}_hand_pinky_DIP_joint'] = .1
        c.push(target, 3)
        for _ in range(100):
            c.step(.002)
            assert c.valid
        assert not c.finger_holds
        assert c.pose[f'{side}_hand_thumb_PIP_joint'] > 0
        assert c.pose[f'{side}_hand_pinky_DIP_joint'] > 0
    finally:
        c.close()


@pytest.mark.parametrize('side', ['left', 'right'])
def test_fast_reversals_at_joint_boundaries_do_not_cancel_stream(side):
    c = A30Control(load_hand_profile('TsenseI_v3_0'), side, 'off', start=False)
    try:
        for tick in range(1200):
            if tick % 17 == 0:
                target = {n: lo + (hi-lo) * (.5+.5*math.sin(tick*.08+i))
                          for i, (n, (lo, hi)) in enumerate(c.limits.items())}
                c.push(target, tick)
            c.step([.002, .003, .002, .007][tick % 4])
            assert c.valid, c.error
            assert all(lo-1e-7 <= c.pose[n] <= hi+1e-7 for n, (lo, hi) in c.limits.items())
            assert len(c.wire) == 108
        limits = c.snapshot()['motion_limits']
        for i, n in enumerate(c.names):
            assert limits[n] == dict(velocity=c.input.max_velocity[i],
                                    acceleration=c.input.max_acceleration[i],jerk=c.input.max_jerk[i])
    finally:
        c.close()
