"""Fake-clock and fake-USB regression tests; no device or sleeping."""
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / "src"), str(Path(__file__).resolve().parent)]
from usb.offline import LM_HOMING_START, rotary_bus_slot_index
from v2_homing import V2Homing, V2Calibration, ONEKEY_ROTARY_STEPS, CALIBRATION_STEPS


class Clock:
    value = 100.0
    def __call__(self):
        return self.value
    def advance(self, value):
        self.value += value


class FakeUSB:
    def __init__(self):
        self.is_connected = True
        self.calls = []
        self.fail = None
        self.raise_read = False
        self.telemetry = None
    def _send(self, name, *args):
        self.calls.append((name, *args))
        return name != self.fail
    def send_hand_type(self, *args): return self._send("hand", *args)
    def send_all_lm_homing(self, *args): return 14 if self._send("linear", *args) else 0
    def send_rm_drive_motor(self, *args): return self._send("drive", *args)
    def send_rm_zero(self, *args): return self._send("zero", *args)
    def send_rm_pos_motor(self, *args): return self._send("position", *args)
    def read_latest_telemetry(self):
        if self.raise_read: raise OSError("unavailable")
        return self.telemetry
    def slot(self, step, home, current=0., position=0.):
        slots = [None] * 7
        slots[rotary_bus_slot_index(*step)] = SimpleNamespace(
            home_label=home, iq_ref=current, pos_actual_mm=position)
        self.telemetry = {"rotary": slots}


@pytest.fixture
def state():
    usb, clock = FakeUSB(), Clock()
    return V2Homing(usb, clock=clock), usb, clock


def seeking(state, *, slot=None):
    homing, usb, clock = state
    homing.start("right", 850)
    clock.advance(1)
    homing.tick()
    if slot is not None: usb.slot(homing.step, *slot)
    clock.advance(.201)
    homing.tick()
    assert homing.phase == "rm_seeking"
    return homing, usb, clock


def test_start_waits_and_visits_exact_six_axis_sequence(state):
    h, usb, clock = state
    assert usb.calls == [] and not h.active
    h.start("right", 850)
    assert usb.calls == [("hand", "right"), ("linear", LM_HOMING_START, 850)]
    clock.advance(.999); h.tick()
    assert h.phase == "lm_wait" and len(usb.calls) == 2
    clock.advance(.0011); h.tick()
    for step in ONEKEY_ROTARY_STEPS:
        assert h.step == step and h.phase == "rm_enable_wait"
        before = len(usb.calls)
        clock.advance(.199); h.tick()
        assert len(usb.calls) == before
        usb.slot(step, 0)
        clock.advance(.002); h.tick()
        assert h.phase == "rm_seeking" and usb.calls[-1] == ("zero", *step)
        usb.slot(step, 1); h.tick()
    assert not h.active and h.step is None and h.phase == ""
    assert h.result and "全部完成" in h.result["text"] and not h.failures
    assert [c[1:3] for c in usb.calls if c[0] == "drive"] == list(ONEKEY_ROTARY_STEPS)
    assert (2, 0) not in ONEKEY_ROTARY_STEPS


@pytest.mark.parametrize("when", ["linear", "enable_wait", "seeking"])
def test_stop_cancels_every_future_command(state, when):
    h, usb, clock = state
    h.start()
    if when != "linear": clock.advance(1); h.tick()
    if when == "seeking": clock.advance(.201); h.tick()
    h.stop("test stop")
    count = len(usb.calls)
    clock.advance(200); h.tick()
    assert len(usb.calls) == count and not h.active and h.step is None
    assert h.result is None and h.status == "test stop"


def test_disconnection_cancels_without_more_commands(state):
    h, usb, clock = seeking(state)
    usb.is_connected = False
    count = len(usb.calls)
    h.tick()
    usb.is_connected = True; clock.advance(30); h.tick()
    assert not h.active and len(usb.calls) == count
    assert "断开" in h.result["text"]


def test_no_telemetry_times_out_each_axis_and_finishes_with_failures(state):
    h, usb, clock = seeking(state)
    usb.raise_read = True
    for index, step in enumerate(ONEKEY_ROTARY_STEPS):
        assert h.step == step
        clock.advance(15.001); h.tick()
        if index != len(ONEKEY_ROTARY_STEPS)-1:
            clock.advance(.201); h.tick()
    assert not h.active and len(h.failures) == 6
    assert "6 轴超时" in h.result["text"] and "全部完成" not in h.result["text"]


def test_home_set_without_transition_is_not_success(state):
    h, usb, clock = seeking(state, slot=(1, 0, 0))
    first=h.step
    clock.advance(2); h.tick()
    assert h.step == first and h.phase == "rm_seeking"
    usb.slot(first, 0); h.tick()
    usb.slot(first, 1); h.tick()
    assert h.step == ONEKEY_ROTARY_STEPS[1]


@pytest.mark.parametrize("evidence", ["current", "motion"])
def test_fallback_evidence_requires_minimum_seek_time(state, evidence):
    h, usb, clock = seeking(state, slot=(1, 0, 0))
    step=h.step
    usb.slot(step, 1, 200 if evidence=="current" else 0, .5 if evidence=="motion" else 0)
    h.tick()
    usb.slot(step, 1, 0, 0)
    clock.advance(1.199); h.tick()
    assert h.step == step
    clock.advance(.002); h.tick()
    assert h.step == ONEKEY_ROTARY_STEPS[1]


@pytest.mark.parametrize("failure", ["hand", "linear"])
def test_start_write_failure_does_not_start_rotaries(state, failure):
    h, usb, clock = state
    usb.fail=failure
    with pytest.raises(ValueError): h.start()
    clock.advance(30); h.tick()
    assert not h.active and all(c[0] not in ("drive", "zero") for c in usb.calls)
    assert len(usb.calls) == (1 if failure=="hand" else 2)


@pytest.mark.parametrize("failure", ["drive", "zero"])
def test_rotary_write_failure_aborts_remaining_queue(state, failure):
    h, usb, clock = state
    h.start(); usb.fail=failure
    clock.advance(1); h.tick()
    if failure=="zero": clock.advance(.201); h.tick()
    count=len(usb.calls)
    clock.advance(40); h.tick()
    assert not h.active and len(usb.calls)==count
    assert "失败" in h.result["text"]


def test_repeated_start_and_invalid_values_do_not_send(state):
    h, usb, clock=state
    for value in (-1, 5001, float("nan"), float("inf"), True, "700"):
        with pytest.raises(ValueError): h.start(iq_seek=value)
    with pytest.raises(ValueError): h.start(side="invalid")
    assert usb.calls == []
    h.start()
    count=len(usb.calls)
    with pytest.raises(ValueError): h.start()
    assert len(usb.calls)==count


def test_calibration_visits_seven_steps_and_never_auto_advances():
    usb=FakeUSB(); c=V2Calibration(usb)
    for index,step in enumerate(CALIBRATION_STEPS):
        assert c.index==index and c.step==step
        assert c.info()["progress"]==f"步骤 {index+1} / 7"
        c.enable(); c.send_position(-12.5); c.confirm_zero()
        assert usb.calls[-3:]==[
            ("drive",*step,True),("position",*step,-12.5),("zero",*step)]
        assert c.index==index
        if index<6:
            assert not c.completed
            count=len(usb.calls); c.next(); assert len(usb.calls)==count
    assert c.completed
    with pytest.raises(ValueError): c.confirm_zero()
    c.next(); assert c.index==6 and c.completed
    c.previous(); assert c.index==5 and not c.completed


def test_calibration_rejects_invalid_position_or_disconnected_commands():
    usb=FakeUSB(); c=V2Calibration(usb)
    for value in (-3601, 3601, float("nan"), True, "1"):
        with pytest.raises(ValueError): c.send_position(value)
    assert not usb.calls
    usb.is_connected=False
    for action in (c.enable,c.confirm_zero,lambda:c.send_position(0)):
        with pytest.raises(ValueError): action()
    assert not usb.calls


def test_calibration_write_failure_does_not_report_completion():
    usb=FakeUSB(); c=V2Calibration(usb)
    for _ in range(6): c.next()
    usb.fail="zero"
    with pytest.raises(ValueError): c.confirm_zero()
    assert not c.completed and c.index==6
