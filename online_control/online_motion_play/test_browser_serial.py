import base64
import time
import struct

import pytest

from browser_serial import BrowserSerial
from test_server import bridge, pump_until
from test_a30 import telemetry
from a30_kinematics.downlink import pack_aa55


def packet(token, seq=1, rx=b''):
    return dict(token=token, seq=seq, rx=base64.b64encode(rx).decode())


def test_transport_coalesces_positions_without_reordering_commands():
    port = BrowserSerial()
    port.open('browser')
    pos1, pos2 = b'\xaa\x55\0\0\x03\x20a', b'\xaa\x55\0\0\x03\x20b'
    enable = b'\xaa\x55\0\0\x03\x21a'
    for frame in (pos1,pos2,enable,pos1,pos2):
        port.write(frame)
    out = port.exchange(packet(port.token,rx=b'feedback'))
    assert [base64.b64decode(item['data']) for item in out['frames']] == [pos2,enable,pos2]
    assert port.read(3) == b'fee' and port.read(99) == b'dback'
    assert port.exchange(packet(port.token,2))['frames'] == []


def test_expiration_never_replays_frames_and_reconnect_changes_token():
    now = [1.]
    port = BrowserSerial(clock=lambda:now[0])
    port.open('browser'); old = port.token
    port.write(b'\xaa\x55\0\0\x03\x22a')
    now[0] += .351
    with pytest.raises(ValueError,match='超时'): port.exchange(packet(old))
    assert not port.is_open and not port.tx
    port.open('browser')
    assert port.token != old
    with pytest.raises(ValueError,match='无效'): port.exchange(packet(old))
    assert port.exchange(packet(port.token))['frames'] == []


def test_invalid_ports_duplicate_sequences_and_bounded_buffers():
    port = BrowserSerial()
    with pytest.raises(ValueError): port.open('/dev/ttyACM0')
    port.open('browser')
    with pytest.raises(ValueError): port.exchange(packet(port.token,2))
    port.exchange(packet(port.token,1,b'x'*65536))
    with pytest.raises(ValueError): port.exchange(packet(port.token,1))
    with pytest.raises(ValueError,match='拥塞'): port.exchange(packet(port.token,2,b'x'))
    assert not port.is_open
    port.open('browser')
    for _ in range(port.MAX_TX): port.write(b'\xaa\x55\0\0\x03\x22a')
    with pytest.raises(OSError,match='拥塞'): port.write(b'\xaa\x55\0\0\x03\x22a')
    assert not port.is_open and not port.tx


def test_v2_browser_session_owner_isolation_and_stale_disconnect(bridge,monkeypatch):
    import serial
    def forbidden(*args,**kwargs): raise AssertionError('Host serial must never open')
    monkeypatch.setattr(serial,'Serial',forbidden)
    bridge.dispatch('control',{},client='owner')
    assert bridge.dispatch('ports',{},client='owner') == []
    with pytest.raises(ValueError,match='本机串口'):
        bridge.dispatch('connect',{'port':'COM3'},client='owner')
    token = bridge.dispatch('connect',{'port':'browser'},client='owner')['token']
    with pytest.raises(ValueError,match='只读'):
        bridge.dispatch('serial',packet(token),client='other')
    frames = bridge.dispatch('serial',packet(token),client='owner')['frames']
    assert len(frames) == 1 and base64.b64decode(frames[0]['data'])[5] == 0x2b
    bridge.dispatch('disconnect',{'token':token},client='owner')
    new = bridge.dispatch('connect',{'port':'browser'},client='owner')['token']
    assert token != new
    assert bridge.dispatch('disconnect',{'token':token},client='owner') is False
    assert bridge.serial.is_open
    bridge.serial.last_exchange -= 1
    bridge.check_lease()
    assert not bridge.serial.is_open and not bridge.v2.usb.is_connected


def test_a30_browser_telemetry_and_homing_reset_use_local_transport(bridge):
    bridge.dispatch('control',{},client='owner')
    bridge.dispatch('model',{'model_id':'TsenseI_v3_0'},client='owner')
    try:
        session = bridge.a30
        token = bridge.dispatch('connect',{'port':'browser'},client='owner')['token']
        out = bridge.dispatch('serial',packet(token,rx=pack_aa55(2,telemetry(session.mapper,session.device.pose))),client='owner')
        assert out['frames'] == []  # Opening A3.0 never enables/moves hardware.
        session.device.read()
        session.device.require_feedback()
        session.device.reset_home()
        frames = bridge.dispatch('serial',packet(token,2),client='owner')['frames']
        assert len(frames) == 18
        assert all(base64.b64decode(f['data'])[5] == 0x22 and base64.b64decode(f['data'])[8] == 2 for f in frames)
        bridge.serial.last_exchange -= 1; bridge.check_lease()
        assert not session.device.connected and not session.device.armed
    finally: bridge.a30.close()


def test_a30_browser_realtime_without_homing_or_all_online(bridge):
    bridge.dispatch('control', {}, client='owner')
    bridge.dispatch('model', {'model_id': 'TsenseI_v3_0'}, client='owner')
    try:
        session = bridge.a30
        name = 'left_hand_index_DIP_joint'
        session.pose[name] = .37
        token = bridge.dispatch('connect', {'port': 'browser'}, client='owner')['token']
        payload = bytearray(telemetry(session.mapper, session.device.pose)[:444])
        struct.pack_into('<I', payload, 8, 1)
        bridge.dispatch('serial', packet(token, rx=pack_aa55(2, payload)), client='owner')
        bridge.dispatch('widget', {'id': session.generation+':realtimeJointControl', 'value': True}, client='owner')
        frames = bridge.dispatch('serial', packet(token, 2), client='owner')['frames']
        assert session.device.armed and len(session.device.seed_missing) == 19
        assert session.device.pose[name] == .37
        decoded = [base64.b64decode(f['data']) for f in frames]
        assert decoded[0][:4] == b'JQP1' and len(decoded[0]) == 192
        q = struct.unpack_from('<20f', decoded[0], 4)
        assert q[session.mapper.joint_names.index(name)] == pytest.approx(.37)
        opcodes = [(f[84:] if f.startswith(b'JQP1') else f)[5] for f in decoded]
        assert opcodes == [0x20]+[0x21]*18  # No implicit homing/reset opcode.
    finally:
        bridge.a30.close()
