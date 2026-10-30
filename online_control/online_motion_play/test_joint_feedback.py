"""Telemetry display regressions with simulated transports only."""
import copy
import struct
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest
import server
from joint_feedback import a30_feedback, a32_feedback, v2_feedback
from a32_session import A32Session
from test_a30 import device, feed, telemetry

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tests"))
from test_a32_driver import connected_device


@pytest.mark.parametrize("model", ["TsenseI_v3_2", "TsenseI_v3_2_withid"])
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


def test_a30_actual_pose_partial_feedback_and_expiry(device):
    dev, clock = device
    pose = {name: lo + .3*(hi-lo) for name, (lo, hi) in zip(dev.mapper.joint_names, dev.mapper._limits)}
    feed(dev, payload=telemetry(dev.mapper, pose))
    feedback = a30_feedback(dev)
    assert {name: row['rad'] for name, row in feedback.items()} == pytest.approx(pose, abs=2e-6)
    payload = bytearray(telemetry(dev.mapper, pose, seq=2))
    struct.pack_into('<I', payload, 8, 0x3fffe)
    feed(dev, payload=payload)
    assert len(a30_feedback(dev)) == 19
    clock.value += .501
    assert a30_feedback(dev) == {}
    feed(dev, seq=3)
    assert len(a30_feedback(dev)) == 20


def test_v2_receiver_timestamp_expires_cached_frame(monkeypatch):
    import joint_feedback
    import usb.offline
    linear = [SimpleNamespace(pos_act_mm=-1.) for _ in range(14)]
    rotary = [SimpleNamespace(pos_actual_mm=10.) for _ in range(7)]
    frame = {'linear': linear, 'rotary': rotary}
    names = [*usb.offline.LINEAR_JOINT_SLOTS, *usb.offline.ROTARY_JOINT_SLOTS]
    monkeypatch.setattr(usb.offline, 'joint_cfg_from_telemetry_frame', lambda *a, **k: dict.fromkeys(names, .25))
    monkeypatch.setattr(joint_feedback.time, 'perf_counter', lambda: 100.)
    usb = SimpleNamespace(is_connected=True, _telem_recv_mono=99.9, read_latest_telemetry=lambda: frame)
    assert len(v2_feedback(usb, 'right')) == 21
    assert all(row['rad'] == .25 for row in v2_feedback(usb, 'right').values())
    usb._telem_recv_mono = 99.4
    assert v2_feedback(usb, 'right') == {}
