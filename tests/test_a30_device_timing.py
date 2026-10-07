"""A30 playback timing/transport regressions; all ports and clocks are simulated."""
from contextlib import nullcontext
import math
import struct
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest
from ruckig import InputParameter, OutputParameter, Result, Ruckig, Synchronization

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src"), str(ROOT / "online_control/online_motion_play")]
import a30_device
from a30_device import A30Device
from a30_kinematics import A30StrokeMapper
from online_control.shared.downlink import CONTROL_DT, CONTROL_HZ, configure_ruckig, encode_control_targets


class Clock:
    value = 0.

    def __call__(self):
        return self.value


class FakePort:
    is_open = True
    in_waiting = 0

    def __init__(self, *args, **kwargs):
        self.frames = []

    def read(self, count):
        return b""

    def write(self, frame):
        self.frames.append(frame)
        return len(frame)

    def close(self):
        self.is_open = False


class TrajectoryPort(FakePort):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.trajectories = []
        self.result = 108

    def write_trajectory(self, frame, q):
        self.trajectories.append((frame, q))
        return self.result


@pytest.fixture(params=["left", "right"])
def device(request):
    clock = Clock()
    d = A30Device(A30StrokeMapper(request.param), FakePort, clock)
    d.connect("FAKE-ONLY")
    # Absent axes deliberately use the explicit valid fallback in arm().
    d.payload = bytes(484)
    d.feedback_at = clock()
    yield d, clock
    d.disconnect()


def arm(d):
    pose = {n: lo + (.2 + i * .01) * (hi - lo)
            for i, (n, (lo, hi)) in enumerate(zip(d.mapper.joint_names, d.mapper._limits))}
    d.arm(dict(reversed(list(pose.items()))))
    d.set_target({n: lo + .7 * (hi - lo) for n, (lo, hi) in zip(d.mapper.joint_names, d.mapper._limits)})


def test_playback_1khz_does_not_change_shared_500hz_or_limits(device):
    d, _ = device
    assert d.CONTROL_DT == d.ruckig.delta_time == .001
    assert CONTROL_HZ == 500 and CONTROL_DT == .002
    assert d.input.max_velocity == [10.] * 20
    assert d.input.max_acceleration == [80.] * 20
    assert d.input.max_jerk == [200.] * 20
    assert d.input.synchronization == Synchronization.Time
    p = InputParameter(20)
    configure_ruckig(p, d.mapper.joint_names, acceleration=15., jerk=120.)
    assert p.max_acceleration == [15.] * 20 and p.max_jerk == [120.] * 20


@pytest.mark.parametrize("interval_ms", [1, 2, 3, 4])
def test_intervals_send_one_latest_frame_without_burst(device, interval_ms):
    d, clock = device
    arm(d)
    before = d.tx_count
    for i in range(1, 101):
        clock.value = i * interval_ms / 1000
        d.tick()
        assert d.armed, d.error
        assert d.tx_count == before + i
        assert d.next_send == pytest.approx(clock() + .001, abs=1e-12)
    assert all(len(f) == 108 for f in d.port.frames[19:])
    clock.value += .04
    d.tick()
    assert d.tx_count == before + 101
    sent = d.tx_count
    d.tick()
    assert d.tx_count == sent


def test_jitter_keeps_absolute_phase_and_catches_up_trajectory(device):
    d, clock = device
    arm(d)
    reference, inp, out = Ruckig(20, .001), InputParameter(20), OutputParameter(20)
    configure_ruckig(inp, d.mapper.joint_names)
    inp.current_position = d.input.current_position
    inp.target_position = [d.target[n] for n in d.mapper.joint_names]
    before = d.tx_count
    for i, now in enumerate((.0012, .0021, .0048, .0071, .0509), 1):
        clock.value = now
        d.tick()
        assert d.armed, d.error
        assert d.tx_count == before + i
        assert d.next_send == pytest.approx((math.floor(now / .001) + 1) * .001, abs=1e-12)
    for _ in range(50):
        assert reference.update(inp, out) in (Result.Working, Result.Finished)
        out.pass_to_input(inp)
    assert [d.pose[n] for n in d.mapper.joint_names] == pytest.approx(out.new_position, abs=1e-12)
    assert d.accumulator == pytest.approx(.0009, abs=1e-12)
    clock.value = .05095
    d.tick()
    assert d.tx_count == before + 5
    clock.value = .0511
    d.tick()
    assert d.tx_count == before + 6 and d.next_send == pytest.approx(.052)


def test_trajectory_capability_for_arm_and_live_ticks_preserves_frame_order_and_log(device, capsys):
    d, clock = device
    port = d.port = TrajectoryPort()
    arm(d)
    assert len(port.frames) == 18 and d.tx_count == 19
    assert [f[5] for f in port.frames] == [0x21] * 18
    assert len(port.trajectories) == 1
    initial_frame, initial_q = port.trajectories[0]
    assert initial_q == [d.pose[n] for n in d.mapper.joint_names]
    assert len(initial_q) == 20 and all(type(v) is float for v in initial_q)
    assert initial_frame == encode_control_targets(d.mapper.convert(d.pose), hand_side=d.mapper.hand_side, seq=0)
    d.log_hex = True
    d.seq = 255
    clock.value = .001
    d.tick()
    frame, q = port.trajectories[1]
    assert len(q) == 20 and all(type(v) is float for v in q)
    assert q == [d.pose[n] for n in d.mapper.joint_names]
    assert frame == encode_control_targets(d.mapper.convert(d.pose), hand_side=d.mapper.hand_side, seq=255)
    assert len(frame) == 108 and d.tx_count == 20 and len(port.frames) == 18
    assert d.seq == 0 and d.last_send == .001
    assert capsys.readouterr().out == "[A3.0 TX] " + frame.hex(" ") + "\n"
    saved_q = list(q)
    clock.value = .002
    d.tick()
    assert q == saved_q and port.trajectories[2][1] is not q
    d.log_hex = False
    d.stop(power_off=True)
    assert len(port.trajectories) == 3
    assert len(port.frames) == 18 + 36


def test_arm_trajectory_q_uses_resolved_feedback_baseline_on_every_arm(device):
    d, _ = device
    port = d.port = TrajectoryPort()
    name = f"{d.mapper.hand_side}_hand_pinky_DIP_joint"
    for angle in (.25, .4):
        d.stop()
        payload = bytearray(484)
        struct.pack_into("<I", payload, 8, 1)  # Only pinky DIP is online.
        struct.pack_into("<f", payload, 16, math.degrees(angle))
        d.payload = bytes(payload)
        fallback = dict(d.pose, **{name: .6})
        pose = d.arm(fallback)
        frame, q = port.trajectories[-1]
        assert pose[name] == pytest.approx(angle)
        assert q == [pose[n] for n in d.mapper.joint_names] == d.input.current_position
        assert q[d.mapper.joint_names.index(name)] != fallback[name]
        assert frame == encode_control_targets(d.mapper.convert(pose), hand_side=d.mapper.hand_side, seq=d.seq - 1)
    assert len(port.trajectories) == 2 and len(port.frames) == 36 and d.tx_count == 38


@pytest.mark.parametrize("result", [0, 107, 109, None])
def test_trajectory_short_write_disconnects_without_counting_or_logging(device, capsys, result):
    d, clock = device
    port = d.port = TrajectoryPort()
    arm(d)
    before = (d.tx_count, d.seq, d.last_send)
    port.result = result
    d.log_hex = True
    clock.value = .001
    d.tick()
    assert not d.armed and not d.connected and not port.is_open
    assert "写入不完整" in d.error
    assert (d.tx_count, d.seq, d.last_send) == before
    assert len(port.trajectories) == 2 and len(port.frames) == 18
    assert capsys.readouterr().out == ""


def test_plain_port_short_write_has_same_failure_semantics(device, monkeypatch):
    d, clock = device
    arm(d)
    before = (d.tx_count, d.seq)
    monkeypatch.setattr(d.port, "write", lambda frame: len(frame) - 1)
    clock.value = .001
    d.tick()
    assert not d.armed and not d.connected
    assert (d.tx_count, d.seq) == before


def test_1khz_retargeting_preserves_velocity_acceleration_jerk_and_duration(device):
    d, clock = device
    arm(d)
    target = dict(d.target)
    d.set_target(target, minimum_duration=2.)
    previous_v = previous_a = [0.] * 20
    for i in range(1, 2501):
        clock.value = i * .001
        d.feedback_at = clock()
        d.tick()
        assert d.armed, d.error
        v, a = d.input.current_velocity, d.input.current_acceleration
        assert all(abs(x) <= limit + 1e-8 for x, limit in zip(v, d.input.max_velocity))
        assert all(abs(x) <= 80 + 1e-8 for x in a)
        assert all(abs(x - y) <= 80 * .001 + 1e-8 for x, y in zip(v, previous_v))
        assert all(abs(x - y) <= 200 * .001 + 1e-8 for x, y in zip(a, previous_a))
        assert all(lo - 1e-10 <= d.pose[n] <= hi + 1e-10 for n, (lo, hi) in zip(d.mapper.joint_names, d.mapper._limits))
        previous_v, previous_a = v, a
        if i == 500:
            assert d.output.trajectory.duration >= 2.
            assert d.pose != pytest.approx(target)
        if i == 1000:
            # A reversal while moving must still preserve all derivatives.
            d.set_target({n: lo + .3 * (hi - lo) for n, (lo, hi) in zip(d.mapper.joint_names, d.mapper._limits)})


@pytest.mark.parametrize("stale", [False, True])
def test_stale_feedback_or_over_100ms_gap_stops_without_sending(device, stale):
    d, clock = device
    arm(d)
    before = d.tx_count
    clock.value = .501 if stale else .101
    if not stale:
        d.feedback_at = clock()
    d.tick()
    assert not d.armed and d.tx_count == before
    assert ("500 ms" if stale else "100 ms") in d.error


def test_io_worker_sleeps_to_absolute_deadlines_and_skips_missed_periods(device, monkeypatch):
    d, clock = device
    sleep_calls, ticks = [], []
    costs = iter((.0002, .0034, .0002))

    def tick():
        ticks.append(clock())
        clock.value += next(costs)
        if len(ticks) == 3:
            d.io_stop.set()

    def sleep(seconds):
        sleep_calls.append(seconds)
        clock.value += seconds

    monkeypatch.setattr(d, "tick", tick)
    monkeypatch.setattr(a30_device, "time", SimpleNamespace(perf_counter=clock, sleep=sleep))
    monkeypatch.setattr(a30_device, "precise_control_timer", nullcontext)
    d._io_loop()
    assert ticks == pytest.approx([0., .001, .005])
    assert sleep_calls == pytest.approx([.0008, .0006, .0008])
