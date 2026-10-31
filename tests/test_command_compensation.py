"""Per-joint compensation reaches bounded targets without altering IK."""
from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src')]
from hand_profiles import A30_MODEL_IDS, A32_MODEL_IDS
from online_control.online_motion_track.command_compensation import compensate_command_pose
from online_control.online_motion_track.main import WebSession
from online_control.online_motion_track.v2.session import V2WebSession
from test_web_v3_2 import packet
from test_web_v2 import MemoryUsb, packet as v2_packet


# Explicit expected commands for a 0.5 rad input, independent of production constants.
HALF_RAD_COMMANDS = {
    'thumb_MCP': .55, 'index_MCP': .6, 'mid_MCP': .55,
    'ring_MCP': .55, 'pinky_MCP': .55, 'index_PIP': .55,
    'index_DIP': .6, 'pinky_DIP': .4,
}


def expected_commands(side):
    return {f'{side}_hand_{joint}_joint': value for joint, value in HALF_RAD_COMMANDS.items()}


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('joint,half,upper', [
    ('index_PIP', .55, 1.6), ('thumb_MCP', .55, 1.6), ('index_MCP', .6, 1.6),
    ('mid_MCP', .55, 1.6), ('ring_MCP', .55, 1.6), ('pinky_MCP', .55, 1.6),
    ('index_DIP', .6, 1.6), ('pinky_DIP', .4, 1.2),
])
@pytest.mark.parametrize('angle', [.5, 0., 1.5, -.5])
def test_compensation_scales_selected_joints_and_clamps(side, joint, half, upper, angle):
    expected = half if angle == .5 else upper if angle == 1.5 else 0.
    name = f'{side}_hand_{joint}_joint'
    pose = {name: angle, f'{side}_hand_ring_DIP_joint': .7}
    before = dict(pose)
    result = compensate_command_pose(pose, {name: (0., 1.6)})
    assert result[name] == pytest.approx(expected)
    assert result[f'{side}_hand_ring_DIP_joint'] == .7
    assert pose == before


def test_pinky_reduction_retains_positive_lower_limit():
    name = 'left_hand_pinky_DIP_joint'
    assert compensate_command_pose({name: 0.}, {name: (0., 1.57)})[name] == 0.


@pytest.mark.parametrize('model', (*A30_MODEL_IDS, *A32_MODEL_IDS))
@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('direct', [False, True])
def test_a3_targets_are_compensated_once_before_collision(model, side, direct, monkeypatch):
    session = WebSession(side, recording=False, model_id=model, collision_mode='off')
    try:
        session.direct_mode = direct
        session.pipeline.collision_bypass = direct
        pipeline = session.pipeline
        name = f'{side}_hand_index_PIP_joint'
        expected = expected_commands(side)
        raw = {**pipeline.session.pose, **dict.fromkeys(expected, .5)}
        monkeypatch.setattr(pipeline.session, 'retarget', lambda *a, **kw: dict(raw))
        seen = []
        def project(pose):
            seen.append(dict(pose))
            # Simulate collision protection limiting the compensated target.
            return {**pose, name: .52}
        if not direct:
            pipeline.mink_guard = SimpleNamespace(project=project, diagnostics={})
        world = pipeline.session.retargeter.fk.compute(raw)
        for frame in range(2):
            result = session.process(packet(world, side, frame))
            assert result['tracking'], result['message']
            assert result['retarget_joint_angles_rad'] == raw
            assert result['pose'][name] == pytest.approx(.55 if direct else .52)
            assert all(result['pose'][n] == pytest.approx(v) for n, v in expected.items() if n != name)
            assert all(result['pose'][n] == v for n, v in raw.items() if n not in expected)
        if not direct:
            assert len(seen) == 2
            assert all(p[n] == pytest.approx(v) for p in seen for n, v in expected.items())
        if direct and model in A32_MODEL_IDS:
            from test_a32_tracking_control import control
            controller, device = control(model, side)
            try:
                controller.set_direct_mode(True)
                controller.arm(True)
                controller.push(result['pose'], 1)
                controller.step(.002)
                controller.step(.002)
                assert len(device.writes) == 2
                assert all(p[n] == pytest.approx(v) for p in device.writes for n, v in expected.items())
            finally:
                controller.close()
    finally:
        session.close()


@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('direct', [False, True])
def test_v2_control_target_uses_compensated_angle(side, direct, monkeypatch):
    session = V2WebSession(side, recording=False, collision_mode='off', usb=MemoryUsb())
    try:
        session.command('set_direct_mode', {'enabled': direct})
        original = session.solver.retarget
        def solve(*args, **kwargs):
            result = original(*args, **kwargs)
            angles = {**result.joint_values, **dict.fromkeys(expected_commands('left'), .5)}
            return replace(result, joint_values=angles)
        monkeypatch.setattr(session.solver, 'retarget', solve)
        session.command('tracking_start', {})
        for frame in range(4):
            result = session.process(v2_packet(session, frame))
        assert result['tracking'], result['message']
        name = f'{side}_hand_index_PIP_joint'
        assert result['retarget_joint_angles_rad'][name] == .5
        assert result['collision_safe_target_joint_angles_rad'][name] == pytest.approx(.55)
        assert session.previous_safe[name] == pytest.approx(.55)
        for joint, expected in expected_commands(side).items():
            assert result['retarget_joint_angles_rad'][joint] == .5
            assert result['collision_safe_target_joint_angles_rad'][joint] == pytest.approx(expected)
            assert session.previous_safe[joint] == pytest.approx(expected)
    finally:
        session.close()
