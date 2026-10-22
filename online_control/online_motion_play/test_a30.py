"""A3.0 editor and simulated H7; never opens a physical port."""
import json
import ast
import math
from pathlib import Path
import struct
import time
import http.cookiejar
import urllib.error
import urllib.request

import pytest
import server  # Sets the shared source paths.
from a30_kinematics import A30StrokeMapper
from a30_kinematics.downlink import channel_mapping, crc16_modbus, pack_aa55
from a30_device import A30Device
from a30_session import A30Session
from test_server import bridge
from test_deployment import live_server, encoded


class Clock:
    value = 100.

    def __call__(self):
        return self.value


class FakeH7:
    def __init__(self, *_args, **_kwargs):
        self.is_open = True
        self.rx = bytearray()
        self.writes = []

    @property
    def in_waiting(self):
        return len(self.rx)

    def read(self, count):
        data = bytes(self.rx[:count])
        del self.rx[:count]
        return data

    def write(self, frame):
        self.writes.append(frame)
        return len(frame)

    def close(self):
        self.is_open = False


def telemetry(mapper, pose, seq=1, home=0):
    targets = mapper.convert(pose)
    payload = bytearray(484)
    struct.pack_into("<HHII", payload, 0, seq, 3, seq*10, 0x3ffff)
    for row in channel_mapping(mapper.hand_side):
        value = (targets.linear_strokes_mm if row["unit"] == "mm" else targets.rotary_positions_deg)[row["joint"]]
        if row["field"] == "self_pos":
            value *= row["position_sign"]
            o = 12 + row["slot"]*24
            struct.pack_into("<4B4f2h", payload, o, 0, 0, 0, home, value, 0, 0, value, 0, 2500)
        else:
            o = 444 + row["slot"]*20
            payload[o:o+2] = bytes((1+row["slot"]*4, 5))
            struct.pack_into("<IHH", payload, o+2, round(value*row["gear_ratio"]/360*2097152)+0x80000000, 0x8000, 0x8000)
            payload[o+16] = 4
    return bytes(payload)


@pytest.fixture(params=["left", "right"])
def device(request):
    clock = Clock()
    mapper = A30StrokeMapper(request.param)
    d = A30Device(mapper, FakeH7, clock)
    d.connect("SIMULATED")
    yield d, clock
    d.disconnect()


def feed(d, seq=1, home=0, payload=None):
    d.port.rx.extend(pack_aa55(2, payload or telemetry(d.mapper, d.pose, seq, home)))
    d.read()


def test_connect_never_sends_and_arm_accepts_partial_unhomed_feedback(device):
    d, _ = device
    assert not d.port.writes
    with pytest.raises(ValueError, match="遥测"):
        d.arm()
    payload = bytearray(telemetry(d.mapper, d.pose))
    struct.pack_into("<I", payload, 8, 0)
    feed(d, payload=payload)
    pose = d.arm()
    assert d.armed and len(d.seed_missing) == 18
    assert pose == pytest.approx(d.pose)
    assert all(f[5] != 0x22 for f in d.port.writes)


@pytest.mark.parametrize("length", [444, 484])
def test_partial_feedback_uses_page_pose_and_streams_without_homing(device, length):
    d, clock = device
    actual = dict(d.pose)
    name = f"{d.mapper.hand_side}_hand_pinky_DIP_joint"
    actual[name] = .25
    fallback = dict(d.pose)
    fallback[f"{d.mapper.hand_side}_hand_index_DIP_joint"] = .4
    payload = bytearray(telemetry(d.mapper, actual, home=3)[:length])
    struct.pack_into("<H", payload, 2, 0)
    struct.pack_into("<I", payload, 8, 1)
    # Offline raw values must never enter the initial position frame.
    struct.pack_into("<f", payload, 16+24, float("nan"))
    if length == 484:
        payload[444:] = b"\xff" * 40
    feed(d, payload=payload)
    pose = d.arm(fallback)
    assert pose[name] == pytest.approx(.25)
    assert pose[f"{d.mapper.hand_side}_hand_index_DIP_joint"] == .4
    assert len(d.seed_missing) == 19 and d.armed
    assert d.status()["self_online"] == 1 and d.status()["external_online"] == 0
    assert len(d.port.writes[0]) == 108
    assert all(f[5] != 0x22 for f in d.port.writes)
    clock.value += .02
    d.tick()
    assert d.armed
    assert d.port.writes[-1][5] == 0x20


def test_zero_online_mask_can_arm_but_homing_remains_explicit(device):
    d, _ = device
    payload = bytearray(telemetry(d.mapper, d.pose))
    struct.pack_into("<H", payload, 2, 0)
    struct.pack_into("<I", payload, 8, 0)
    feed(d, payload=payload)
    assert d.arm() == d.pose and len(d.seed_missing) == 20
    d.stop()
    with pytest.raises(ValueError, match="全部"):
        d.start_home(700, 120)


def test_telemetry_positions_and_home_states_match_source_file(device):
    from typing import Optional
    d, _ = device
    tree = ast.parse((Path(__file__).resolve().parents[2]/'tests/fixtures/a30_protocol_reference.py').read_text(encoding='utf-8-sig'))
    constants = {'MIT_STATUS_BYTES', 'MIT_POS_COUNTS', 'MIT_GEAR_RATIO', 'MIT_FAULT_BITS',
                 'HAND_TELEM_HDR', 'SELF_TELEM_SLOT'}
    functions = {'parse_mit_status20', 'mit_motor_to_out', 'mit_ident_ok', 'decode_homing_or_theta'}
    nodes = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name in functions
             or isinstance(n, ast.Assign) and any(isinstance(t, ast.Name) and t.id in constants for t in n.targets)]
    namespace = {'struct': struct, 'Optional': Optional}
    exec(compile(ast.Module(body=nodes, type_ignores=[]), 'reference_telemetry_only', 'exec'), namespace)
    actual = dict(d.pose)
    actual[f'{d.mapper.hand_side}_hand_thumb_rotaback_joint2'] = .25
    actual[f'{d.mapper.hand_side}_hand_thumb_rota_block_joint'] = .3
    payload = telemetry(d.mapper, actual, home=0)
    assert namespace['HAND_TELEM_HDR'].unpack_from(payload) == (1, 3, 10, 0x3ffff)
    assert namespace['decode_homing_or_theta'](namespace['SELF_TELEM_SLOT'].unpack_from(payload, 12)[0]) == 'WAIT'
    feed(d, payload=payload)
    pose = d.arm()
    for i, suffix in enumerate(('thumb_rotaback_joint2', 'thumb_rota_block_joint')):
        parsed = namespace['parse_mit_status20'](payload[444+20*i:464+20*i])
        row = next(r for r in channel_mapping(d.mapper.hand_side)
                   if r['field'] == 'mit_pos_deg' and r['slot'] == i)
        angle = math.radians(parsed['pos_deg'] / row['gear_ratio'])
        assert pose[f'{d.mapper.hand_side}_hand_{suffix}'] == pytest.approx(angle)
        assert angle == pytest.approx(actual[f'{d.mapper.hand_side}_hand_{suffix}'], abs=1e-6)
    assert namespace['mit_ident_ok'](bytes([5,5]), 0)
    assert namespace['mit_ident_ok'](bytes([9,5]), 1)


def test_parser_crc_split_noise_and_duplicate_feedback_expiry(device):
    d, clock = device
    valid = pack_aa55(2, telemetry(d.mapper, d.pose))
    corrupt = bytearray(valid)
    corrupt[-1] ^= 1
    d.port.rx.extend(b"noise"+corrupt+valid[:200])
    d.read()
    assert d.payload is None
    d.port.rx.extend(valid[200:])
    d.read()
    d.require_feedback()
    clock.value += .6
    d.port.rx.extend(valid)
    d.read()
    with pytest.raises(ValueError, match="500 ms"):
        d.require_feedback()


@pytest.mark.parametrize("length", [444, 445, 483, 484, 485, 512, 2040])
def test_extended_telemetry_trims_like_reference_and_can_arm(device, length):
    d, _ = device
    take = 484 if length >= 484 else 444
    expected = telemetry(d.mapper, d.pose)[:take]
    payload = expected + bytes([0xA5]) * (length - take)
    wire = pack_aa55(2, payload)
    # Even when all known fields have arrived, wait for the extension and CRC.
    d.port.rx.extend(wire[:6+take])
    d.read()
    assert d.payload is None
    d.port.rx.extend(wire[6+take:])
    d.read()
    assert d.payload == expected and not d.buffer
    assert d.status()["external_online"] == (2 if take == 484 else 0)
    d.arm()
    assert d.armed


def test_extended_telemetry_crc_covers_extension_and_preserves_next_frame(device):
    d, _ = device
    expected = telemetry(d.mapper, d.pose)
    corrupt = bytearray(pack_aa55(2, expected + bytes(40)))
    corrupt[6+484+7] ^= 1  # Corruption only in the otherwise ignored extension.
    d.port.rx.extend(corrupt)
    d.read()
    assert d.payload is None
    d.port.rx.extend(pack_aa55(2, expected + bytes(40)) + pack_aa55(2, telemetry(d.mapper, d.pose, seq=2)))
    d.read()
    assert d.payload == telemetry(d.mapper, d.pose, seq=2)
    assert d.rx_count == 2 and not d.buffer


def test_extended_telemetry_rejects_short_oversized_and_other_functions(device):
    d, _ = device
    for wire in (pack_aa55(2, bytes(443)), pack_aa55(2, bytes(2041)),
                 pack_aa55(3, bytes(512)), pack_aa55(2, b"")):
        d.port.rx.extend(wire)
        d.read()
        assert d.payload is None and d.rx_count == 0
    feed(d)
    assert d.payload is not None and d.rx_count == 1


@pytest.mark.parametrize("edge", ["lower", "upper"])
def test_startup_accepts_small_rotary_boundary_offset_and_reports_raw(device, edge):
    d, _ = device
    name = f"{d.mapper.hand_side}_hand_pinky_DIP_joint"
    lo, hi = dict(zip(d.mapper.joint_names, d.mapper._limits))[name]
    boundary = lo if edge == "lower" else hi
    raw = math.degrees(boundary) + (-.16041614 if edge == "lower" else .16)
    payload = bytearray(telemetry(d.mapper, d.pose))
    struct.pack_into("<f", payload, 16, raw)
    feed(d, payload=payload)
    pose = d.arm()
    assert d.armed and pose[name] == boundary
    assert struct.unpack_from("<f", d.port.writes[0], 7)[0] == pytest.approx(math.degrees(boundary))
    detail = next(r for r in d.status()["seed_clipped"] if r["joint"] == name)
    assert detail["raw"] == pytest.approx(raw) and detail["unit"] == "deg"
    assert detail["applied"] == pytest.approx(math.degrees(boundary))
    assert struct.unpack_from("<f", d.payload, 16)[0] == pytest.approx(raw)
    assert all(f[5] != 0x22 for f in d.port.writes)
    d.disconnect()
    assert not d.status()["seed_clipped"]


@pytest.mark.parametrize("raw", [-.51, 90.5, 1000.])
def test_startup_clamps_large_rotary_offset_and_clears_report_on_rearm(device, raw):
    d, _ = device
    payload = bytearray(telemetry(d.mapper, d.pose))
    struct.pack_into("<f", payload, 16, raw)
    feed(d, payload=payload)
    pose = d.arm()
    name = f"{d.mapper.hand_side}_hand_pinky_DIP_joint"
    lo, hi = dict(zip(d.mapper.joint_names, d.mapper._limits))[name]
    assert pose[name] == (lo if raw < 0 else hi)
    assert d.armed and not d.error
    assert next(r for r in d.status()["seed_clipped"] if r["joint"] == name)["raw"] == pytest.approx(raw)
    d.stop()
    # Use an interior reading; float32 at the upper boundary can itself need
    # a tiny fresh correction and is not evidence of a stale startup report.
    feed(d, payload=telemetry(d.mapper, dict(d.pose, **{name: (lo+hi)/2}), seq=2))
    d.arm()
    assert d.armed and not d.error and not d.status()["seed_clipped"]


@pytest.mark.parametrize("channel", range(20))
@pytest.mark.parametrize("edge", ["lower", "upper"])
def test_all_channels_accept_out_of_range_feedback_with_bounded_targets(device, channel, edge):
    d, clock = device
    row = channel_mapping(d.mapper.hand_side)[channel]
    name = row["joint"]
    lo, hi = dict(zip(d.mapper.joint_names, d.mapper._limits))[name]
    boundary = lo if edge == "lower" else hi
    boundary_pose = dict(d.pose, **{name: boundary})
    targets = d.mapper.convert(boundary_pose)
    is_linear = row["unit"] == "mm"
    at_boundary = (targets.linear_strokes_mm if is_linear else targets.rotary_positions_deg)[name]
    raw = at_boundary + (-1 if edge == "lower" else 1) * (2 if is_linear else 5)
    payload = bytearray(telemetry(d.mapper, d.pose))
    if row["field"] == "self_pos":
        struct.pack_into("<f", payload, 16 + row["slot"]*24, raw * row["position_sign"])
        wire_offset = 7 + row["slot"]*4
    else:
        code = round(raw * row["gear_ratio"] / 360 * 2097152) + 0x80000000
        struct.pack_into("<I", payload, 446 + row["slot"]*20, code)
        wire_offset = 79 + row["slot"]*4
    feed(d, payload=payload)
    pose = d.arm()
    assert d.armed and not d.error
    assert pose[name] == pytest.approx(boundary, abs=1e-7)
    assert d.payload == bytes(payload)
    detail = next(r for r in d.status()["seed_clipped"] if r["joint"] == name)
    assert detail["raw"] == pytest.approx(raw, abs=1e-5)
    assert detail["applied"] == pytest.approx(at_boundary, abs=1e-5)
    assert detail["unit"] == row["unit"]
    wire_value = struct.unpack_from("<f", d.port.writes[0], wire_offset)[0]
    assert wire_value == pytest.approx(at_boundary * row["position_sign"] * row["gear_ratio"], abs=1e-4)
    # The same finite out-of-range telemetry must not stop subsequent ticks.
    clock.value += .02
    d.tick()
    assert d.armed and not d.error
    assert all(lo - 1e-7 <= d.pose[n] <= hi + 1e-7 for n, (lo, hi) in zip(d.mapper.joint_names, d.mapper._limits))


@pytest.mark.parametrize("raw", [float("nan"), float("inf"), -float("inf")])
def test_startup_still_rejects_nonfinite_online_linear_feedback(device, raw):
    d, _ = device
    payload = bytearray(telemetry(d.mapper, d.pose))
    struct.pack_into("<f", payload, 16 + 4*24, raw)  # pinky PIP
    feed(d, payload=payload)
    with pytest.raises(ValueError, match="位置反馈无效"):
        d.arm()
    assert not d.armed and not d.port.writes


def test_arming_seeds_measured_pose_and_writes_a3_only(device):
    d, clock = device
    actual = dict(d.pose)
    actual[f"{d.mapper.hand_side}_hand_thumb_rota_block_joint"] = .3
    feed(d, payload=telemetry(d.mapper, actual))
    pose = d.arm()
    assert pose == pytest.approx(actual, abs=1e-5)
    assert [f[5] for f in d.port.writes] == [0x20]+[0x21]*18
    assert struct.unpack_from("<f", d.port.writes[0], 83)[0] == pytest.approx(math.degrees(.3)*30, abs=.001)
    target = dict(actual)
    name = f"{d.mapper.hand_side}_hand_index_DIP_joint"
    target[name] = .4
    d.set_target(target)
    previous_v, previous_a = [0.]*20, [0.]*20
    for seq in range(2, 42):
        clock.value += .01
        feed(d, seq=seq)
        d.tick()
        assert all(abs(v-pv) <= 400*.01+1e-8 for v, pv in zip(d.input.current_velocity, previous_v))
        assert all(abs(a-pa) <= 2400*.01+1e-8 for a, pa in zip(d.input.current_acceleration, previous_a))
        previous_v, previous_a = d.input.current_velocity, d.input.current_acceleration
    assert 0 < d.pose[name] <= .4 + 1e-12
    for frame in d.port.writes:
        assert crc16_modbus(frame[:-2]) == struct.unpack_from("<H", frame, len(frame)-2)[0]
    count = len(d.port.writes)
    clock.value += .6
    d.tick()
    assert not d.armed and len(d.port.writes) == count


def test_500hz_positions_with_latest_only_after_scheduler_delay(device):
    d, clock = device
    feed(d)
    d.arm()
    d.port.writes.clear()
    for _ in range(100):
        clock.value += .002
        d.tick()
    assert len(d.port.writes) == 100
    assert all(frame[5] == 0x20 for frame in d.port.writes)
    clock.value += .04
    d.tick()
    assert len(d.port.writes) == 101  # Catch up trajectory, not stale USB frames.


def test_io_worker_stop_is_serialized_and_close_joins_thread():
    d = A30Device(A30StrokeMapper(), FakeH7)
    d.connect("SIMULATED")
    try:
        feed(d)
        d.arm()
        initial = d.tx_count
        d.start_io()
        until = time.perf_counter() + .2
        while d.tx_count == initial and time.perf_counter() < until:
            time.sleep(.002)
        assert d.io_running and d.tx_count > initial
        d.stop()
        stopped = d.tx_count
        time.sleep(.02)
        assert d.tx_count == stopped and not d.armed
    finally:
        d.close_io()
    assert not d.io_running and not d.connected


def test_homing_confirmation_and_separate_iq_then_external_report(device):
    d, clock = device
    feed(d)
    d.start_home(700, 120)
    seq = 2
    for slot in range(18):
        d.tick()  # reset request
        assert d.homing["phase"] == "reset"
        assert struct.unpack_from("<f", d.port.writes[-1], 10)[0] == (120 if slot in (0,1,2,3,8,9,10,11,16) else 700)
        feed(d, seq=seq, home=0)
        d.tick()  # reset ack -> enable + seek
        assert d.homing["phase"] == "seek"
        feed(d, seq=seq+1, home=2)
        d.tick()
        seq += 2
    d.tick()
    assert not d.homing and len(d.report) == 20
    assert all(r["success"] for r in d.report[:18])
    assert not any(r["success"] for r in d.report[18:])
    assert {f[5] for f in d.port.writes} == {0x20, 0x21, 0x22}


def test_homing_stale_completion_is_not_success_and_stop_cancels(device):
    d, clock = device
    feed(d, home=2)
    d.start_home(700, 120)
    d.tick()
    feed(d, seq=2, home=2)
    d.tick()
    assert d.homing["phase"] == "reset" and not d.report
    d.stop()
    assert not d.homing
    assert all(f[5] == 0x22 and f[8] == 2 for f in d.port.writes[-18:])


def test_homing_failure_and_reset_semantics(device):
    d, clock = device
    feed(d)
    d.start_home(700, 120)
    d.tick()
    feed(d, seq=2, home=0)
    d.tick()
    feed(d, seq=3, home=3)
    d.tick()
    assert d.report[0]["success"] is False
    d.stop()
    d.port.writes.clear()
    d.reset_home()
    assert len(d.port.writes) == 18 and not d.armed
    assert all(f[5] == 0x22 and f[8] == 2 for f in d.port.writes)
    d.port.writes.clear()
    d.stop(power_off=True)
    assert len(d.port.writes) == 36
    assert all(f[5] in (0x21, 0x22) and f[8] == 2 for f in d.port.writes)


def test_external_fault_or_missing_self_feedback_stops_stream(device):
    d, clock = device
    feed(d)
    d.arm()
    payload = bytearray(telemetry(d.mapper, d.pose, seq=2))
    struct.pack_into("<I", payload, 456, 1)
    feed(d, payload=payload)
    sent = len(d.port.writes)
    clock.value += .02
    d.tick()
    assert not d.armed and "外接" in d.error
    assert len(d.port.writes) == sent


def test_homing_timeout_requires_fresh_reset_and_is_reported(device):
    d, clock = device
    feed(d, home=2)
    d.start_home(700, 120)
    d.tick()
    clock.value += 1.6
    feed(d, seq=2, home=2)
    d.tick()
    assert d.report[0]["success"] is False
    assert "超时" in d.report[0]["reason"]


def widget(session, key, value=None):
    session.widget({"id": session.generation+":"+key, "value": value})


def test_model_switch_joint_limits_files_and_stale_widgets(bridge):
    old = bridge.snapshot()["joints"][0]
    bridge.dispatch("model", {"model_id": "TsenseI_v3_0"})
    s = bridge.a30
    try:
        snapshot = bridge.snapshot()
        assert len(snapshot["joints"]) == 20
        assert snapshot["model_id"] == "TsenseI_v3_0"
        assert "v3_0" in snapshot["urdf"]
        assert s.path.name == "motions_a30_left.json"
        with pytest.raises(ValueError):
            bridge.dispatch("widget", {"id": old["id"], "value": .2})
        name = "left_hand_index_DIP_joint"
        widget(s, name, .5)
        assert s.pose[name] == .5 and not s.device.armed
        widget(s, "savemotion")
        widget(s, "motionName", "A3抓握")
        widget(s, "confirm")
        exported = bridge.dispatch("export", {})
        assert exported["motions"]["A3抓握"]["pose"][name] == .5
        before = s.path.read_bytes()
        bad = json.loads(json.dumps(exported))
        bad["motions"]["A3抓握"]["pose"][name] = 999
        with pytest.raises(ValueError):
            bridge.dispatch("import", {"text": json.dumps(bad)})
        assert s.path.read_bytes() == before
        bridge.dispatch("widget", {"id": s.generation+":handSide", "value": 1})
        assert len(bridge.snapshot()["joints"]) == 20 and bridge.a30.side == "right"
        with pytest.raises(ValueError, match="手别"):
            bridge.dispatch("import", {"text": json.dumps(exported)})
        bridge.dispatch("model", {"model_id": "TsenseI_v2"})
        assert len(bridge.snapshot()["joints"]) == 21
        with pytest.raises(ValueError, match="A3.0"):
            bridge.dispatch("import", {"text": json.dumps(exported)})
    finally:
        if bridge.a30:
            bridge.a30.close()


def test_a30_lease_disarm_disconnect_and_observer_rejection(bridge):
    bridge.dispatch("control", {}, "owner")
    bridge.dispatch("model", {"model_id": "TsenseI_v3_0"}, "owner")
    s = bridge.a30
    try:
        s.device.serial_factory = FakeH7
        s.device.connect("SIMULATED")
        feed(s.device)
        s.device.arm()
        with pytest.raises(ValueError, match="只读"):
            bridge.dispatch("stop", {}, "observer")
        with pytest.raises(ValueError, match="断开"):
            bridge.dispatch("model", {"model_id": "TsenseI_v2"}, "owner")
        bridge.last_seen -= 10
        bridge.check_lease()
        assert not s.device.armed and not s.device.connected and bridge.controller is None
    finally:
        s.close()


def test_a30_playback_honors_transition_and_requires_correct_mode(bridge):
    bridge.dispatch("model", {"model_id": "TsenseI_v3_0"})
    s = bridge.a30
    try:
        s.device.serial_factory = FakeH7
        s.device.connect("SIMULATED")
        feed(s.device)
        widget(s, "realtimeJointControl", True)
        target = dict(s.pose)
        target["left_hand_index_DIP_joint"] = .3
        s.persist({"slow": {"pose": target, "transition": 4., "interval": .1}})
        with pytest.raises(ValueError, match="动作同步"):
            widget(s, "motionstreamplay")
        widget(s, "isConnectOffline", True)
        widget(s, "motionstreamplay")
        assert s.device.input.minimum_duration == 4.
        assert s.device.target == target
        assert not next(j for j in s.snapshot()["joints"] if j["name"] == "left_hand_index_DIP_joint")["enabled"]
        s.stop()
        assert not s.playback and not s.device.armed
    finally:
        s.close()


def test_bundle_has_a30_meshes_and_linkage_data():
    import package
    names = {name for _, name in package.files()}
    assert {"a30_device.py", "a30_session.py", "app/src/a32_kinematics/model/geometry_inferred.mat",
            "app/src/a32_kinematics/model/A3.2_source.urdf", "app/assets/hands/TsenseI_hand_v3_0/hand_profile.yaml"} <= names
    assert any("TsenseI_hand_v3_0/meshes/" in n for n in names)


def test_a30_authenticated_http_model_state_asset_and_export(live_server):
    client = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(http.cookiejar.CookieJar()))

    def request(path, body=None, owner="owner-0000000000000"):
        headers = {"X-Motion-Client": owner}
        if body is not None:
            headers["Content-Type"] = "application/json"
        return client.open(urllib.request.Request(live_server+path,
                           None if body is None else json.dumps(body).encode(), headers), timeout=10)

    with request("/api/login", {"password": "deployment-test-password"}):
        pass
    with request("/api/control", {}):
        pass
    with request("/api/model", {"model_id": "TsenseI_v3_0"}):
        pass
    with request("/api/state") as response:
        state = json.load(response)
    assert state["model_id"] == "TsenseI_v3_0" and len(state["joints"]) == 20
    with request(state["urdf"]) as response:
        assert b"robot" in response.read()
    with request("/api/download") as response:
        assert "attachment" in response.headers["Content-Disposition"]
        assert json.load(response)["_model_id"] == "TsenseI_v3_0"
    with pytest.raises(urllib.error.HTTPError) as denied:
        request("/api/model", {"model_id": "TsenseI_v2"}, owner="observer-0000000000")
    assert denied.value.code == 400
    with request("/api/release", {}):
        pass
