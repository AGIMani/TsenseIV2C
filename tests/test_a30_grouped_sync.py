"""Finger isolation, retry atomicity and display-only transport."""
import sys
from pathlib import Path

import pytest

sys.path[:0] = [str(Path(__file__).resolve().parents[1]), str(Path(__file__).resolve().parents[1] / 'src')]
from hand_profiles import load_hand_profile
from online_control.online_motion_track.a30.control import A30Control


@pytest.mark.parametrize('side', ['left', 'right'])
def test_other_fingers_do_not_delay_index_motion(side):
    profile = load_hand_profile('TsenseI_v3_0')
    isolated = A30Control(profile, side, 'off', start=False)
    combined = A30Control(profile, side, 'off', start=False)
    try:
        target = dict(isolated.pose)
        for part, value in [('MCP', .5), ('PIP', .3), ('DIP', .1)]:
            target[f'{side}_hand_index_{part}_joint'] = value
        isolated.push(target, 1)
        target[f'{side}_hand_ring_swing_joint'] = .35
        target[f'{side}_hand_ring_PIP_joint'] = 1.4
        target[f'{side}_hand_thumb_rota_block_joint'] = 1.7
        combined.push(target, 1)
        for _ in range(250):
            isolated.step(.002); combined.step(.002)
            assert isolated.valid and combined.valid
            for part in ('MCP', 'PIP', 'DIP'):
                name = f'{side}_hand_index_{part}_joint'
                assert isolated.pose[name] == pytest.approx(combined.pose[name], abs=1e-12)
        groups = combined.snapshot()['motion_synchronization']['groups']
        assert len(groups) == 5
        assert sorted(n for group in groups.values() for n in group) == sorted(combined.names)
        assert all(len(group) == 4 for group in groups.values())
        assert f'{side}_hand_index_swing_joint' in groups['index']
        assert f'{side}_hand_thumb_rota_block_joint' in groups['thumb']
    finally:
        isolated.close(); combined.close()


def test_retry_does_not_advance_unaffected_groups_twice():
    c = A30Control(load_hand_profile('TsenseI_v3_0'), 'left', 'off', start=False)
    try:
        c.push({n: lo + .4*(hi-lo) for n, (lo, hi) in c.limits.items()}, 1)
        for _ in range(40): c.step(.002)
        c.otg.update(c.input, c.output)
        before = (c.output.new_position.copy(), c.output.new_velocity.copy(), c.output.new_acceleration.copy())
        # A protective check may reject the candidate before pass_to_input.
        c.otg.update(c.input, c.output)
        after = (c.output.new_position, c.output.new_velocity, c.output.new_acceleration)
        for a, b in zip(before, after):
            assert a == pytest.approx(b, abs=1e-12)
    finally:
        c.close()


def test_preview_has_no_usb_payload_and_does_not_occupy_control_stream():
    c = A30Control(load_hand_profile('TsenseI_v3_0'), 'left', 'off', start=False)
    preview = downlink = None
    try:
        c.step(.002)
        preview = c.open_stream(c.epoch, preview=True)
        downlink = c.open_stream(c.epoch)
        p = next(iter(preview)); d = next(iter(downlink))
        assert len(p) == 108 and len(d) == 216
        assert p == d[:108]
        assert c.stream_active and c.preview_stream_active
        preview.close()
        assert c.stream_active and not c.preview_stream_active
        downlink.close()
        assert not c.stream_active
    finally:
        if preview: preview.close()
        if downlink: downlink.close()
        c.close()
