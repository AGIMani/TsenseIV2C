"""No hardware: streaming ownership, expiry, common motion parameters and wire data."""
from pathlib import Path
import struct
import sys
import time
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'src'), str(ROOT/'online_control/online_motion_play')]
from browser_serial import BrowserSerial
from a30_kinematics.downlink import pack_hand_batch
from a30_kinematics import A30StrokeMapper
from a30_device import A30Device
from hand_profiles import load_hand_profile
from online_control.online_motion_track.a30.control import A30Control
from online_control.shared.downlink import encode_control_targets


def test_stream_rx_does_not_consume_tx_and_rejects_competing_reader():
    link = BrowserSerial().open('browser')
    records = link.open_stream(link.token)
    with pytest.raises(ValueError):
        link.open_stream(link.token)
    wire = pack_hand_batch(3, [0.]*18, [0.]*2, [0.]*2, [0.]*2)
    link.write(wire)
    body = b'MSR1'+link.token.encode()+struct.pack('<I',1)+b'telemetry'
    reply = link.exchange_binary(body, receive_only=True)
    assert struct.unpack_from('<H',reply,40)[0] == 0
    assert link.read(100) == b'telemetry'
    streamed = next(records)
    assert streamed[48:] == wire
    assert struct.unpack_from('<I',streamed,36)[0] == 1
    records.close()
    assert not link.is_open


def test_stream_expiry_and_reconnect_cannot_replay_old_commands():
    clock = [10.]
    link = BrowserSerial(clock=lambda:clock[0]).open('browser')
    records = link.open_stream(link.token)
    next(records)
    link.close(); link.open('browser')
    with pytest.raises(StopIteration): next(records)
    assert link.is_open
    records = link.open_stream(link.token)
    clock[0] += .4
    with pytest.raises(StopIteration): next(records)
    assert not link.is_open


def test_unstarted_stream_releases_ownership():
    link = BrowserSerial().open('browser')
    records = link.open_stream(link.token)
    records.close()
    assert not link.is_open and not link.streaming
    control = A30Control(load_hand_profile('TsenseI_v3_0'),'left','off',start=False)
    try:
        records = control.open_stream(control.epoch)
        records.close()
        newer = control.open_stream(control.epoch)
        records.close()  # An old HTTP connection cannot release the newer one.
        assert control.stream_active
        newer.close()
        assert not control.stream_active
    finally:
        control.close()


@pytest.mark.parametrize('side',['left','right'])
def test_tracking_stream_uses_same_form2_limits_and_exact_encoded_frame(side):
    control = A30Control(load_hand_profile('TsenseI_v3_0'),side,'off',start=False)
    device = A30Device(A30StrokeMapper(side))
    try:
        for field in ('max_velocity','max_acceleration','max_jerk','synchronization'):
            assert getattr(control.input,field) == getattr(device.input,field)
        for i in range(3): control.push(control.pose,i)
        control.step(.002)
        assert control.valid, control.error
        records = control.open_stream(control.epoch)
        packet = next(records)
        assert packet[:4] == b'GTR1'
        assert struct.unpack_from('<I',packet,24)[0] & 1
        wire = encode_control_targets(control.mapper.convert(control.pose),hand_side=side,seq=0)
        assert packet[108:] == wire
        control.invalidate('lost hand')
        packet = next(records)
        assert not struct.unpack_from('<I',packet,24)[0] & 1
        assert len(packet) == 108
        records.close()
    finally: control.close()
