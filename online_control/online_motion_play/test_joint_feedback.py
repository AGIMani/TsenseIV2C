"""Telemetry display regressions with simulated transports only."""
import copy
import sys
from pathlib import Path

import pytest
import server
from joint_feedback import a32_feedback
from a32_session import A32Session

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tests"))
from test_a32_driver import connected_device


@pytest.mark.parametrize("model", ["TsenseI_v3_2_withid"])
def test_a32_snapshot_reads_actual_positions_without_changing_targets(tmp_path, model):
    device, backend = connected_device(model)
    session = A32Session(tmp_path, device=device, model_id=model, start=False)
    try:
        session.armed = True
        pose_before = dict(session.pose)
        calls_before = list(backend.calls)
        for fraction in (.3, .6):
            measured = {name: lo + fraction * (hi-lo) for name, (lo, hi) in device.joint_limits().items()}
            command = device.prepare_targets(measured, clamp=False)
            for channel in device.channels:
                value = command['self' if channel['channel'] == 'self' else 'mit_pos'][channel['index']]
                backend.state[channel['channel']][channel['index']]['position'] = -value if channel['unit'] == 'mm' else value
            snapshot = session.snapshot()
            assert {joint['name']: joint['feedback']['rad'] for joint in snapshot['joints']} == pytest.approx(measured, abs=2e-7)
            assert session.pose == pose_before
            assert backend.calls == calls_before
        raw = copy.deepcopy(backend.state)
        channels = device.channels
        offline = channels[0]
        backend.state[offline['channel']][offline['index']]['online'] = False
        assert offline['joint'] not in a32_feedback(device, backend.get_state())
        assert len(a32_feedback(device, backend.get_state())) == 19
        backend.state = raw
        backend.state['telemetry_age_ms'] = 501
        assert a32_feedback(device, backend.get_state()) == {}
        backend.state['telemetry_age_ms'] = 0
        assert len(a32_feedback(device, backend.get_state())) == 20
        backend.state['mit'][0]['fault'] = 1
        assert len(a32_feedback(device, backend.get_state())) == 19
    finally:
        session.close()
