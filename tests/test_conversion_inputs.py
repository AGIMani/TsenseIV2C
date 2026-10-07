"""Public conversion boundaries: invalid data must not escape into motor frames."""
import math
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from a32_kinematics import A32StrokeMapper
from kinematics import finger_linkage as fl
from usb import offline


@pytest.fixture
def v2():
    mapper = fl.create_default_hand_stroke_mapper()
    offline.configure_hand_stroke_mapper(mapper)
    yield mapper
    offline.configure_hand_stroke_mapper(fl.create_default_hand_stroke_mapper())


@pytest.mark.parametrize("bad", [None, [], "bad", 1j, 10**1000])
def test_v2_malformed_joint_holds_affected_finger_without_logging_crash(v2, bad):
    pose = {n: .3 for n in v2.managed_linear_joints()}
    first = v2.linear_strokes_mm(pose)
    pose = {n: .6 for n in pose}
    pose["left_hand_mid_PIP_joint"] = bad
    after = v2.linear_strokes_mm(pose)
    for part in ("MCP", "PIP", "DIP"):
        n = f"left_hand_mid_{part}_joint"
        assert after[n] == first[n]
    assert after["left_hand_index_MCP_joint"] != first["left_hand_index_MCP_joint"]
    assert all(math.isfinite(x) for x in after.values())


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -float("inf"), 1e308, -1e308])
def test_v2_usb_rejects_unrepresentable_rotary_without_poisoning_cache(v2, bad):
    pose = {n: .3 for n in offline.JOINT_ORDER}
    first = offline.joint_angles_to_hand_positions(pose)
    broken = {**pose, fl.ROTARY_SLOT_JOINTS[0]: bad}
    with pytest.raises(ValueError):
        offline.joint_angles_to_hand_positions(broken)
    assert offline.joint_angles_to_hand_positions(pose) == first


@pytest.mark.parametrize("value", [1e308, -1e308, float("nan"), float("inf")])
def test_v2_usb_extreme_linear_inputs_do_not_overflow_cache(v2, value):
    pose = {n: value for n in fl.LINEAR_SLOT_JOINTS}
    linear, rotary = offline.joint_angles_to_hand_positions(pose)
    assert all(math.isfinite(x) for x in linear + rotary)
    assert max(abs(x) for x in linear) < 1e-10


def test_v2_overflowed_scale_holds_previous_finite_stroke(v2):
    pose = {n: .3 for n in v2.managed_linear_joints()}
    first = v2.linear_strokes_mm(pose)
    name = "left_hand_index_MCP_joint"
    v2.set_stroke_scale(name, 1e308)
    after = v2.linear_strokes_mm({n: 1.2 for n in pose})
    assert after[name] == first[name]
    assert all(math.isfinite(x) for x in after.values())


@pytest.mark.parametrize("bad", [None, [], np.zeros(21), {fl.ROTARY_SLOT_JOINTS[0]: float("nan")},
                                 {fl.LINEAR_SLOT_JOINTS[0]: "bad"}, {fl.ROTARY_SLOT_JOINTS[0]: 1e308}])
def test_v2_invalid_frame_is_rejected_before_any_device_write(v2, monkeypatch, bad):
    ctrl = offline.UsbController()
    ctrl._connected = True
    ctrl._ser = SimpleNamespace(is_open=True)
    assert ctrl.should_process_downlink
    def forbidden(*args, **kwargs):
        pytest.fail("Invalid input reached a device write")
    monkeypatch.setattr(ctrl, "send_hand_type", forbidden)
    monkeypatch.setattr(ctrl, "send_hand_positions", forbidden)
    assert ctrl.send_joint_angles(bad) is False


def test_v2_float32_overflow_is_rejected_before_packing(v2):
    name = fl.LINEAR_SLOT_JOINTS[0]
    v2.set_stroke_scale(name, 1e100)
    with pytest.raises(ValueError, match="float32"):
        offline.joint_angles_to_hand_positions({name: 1.0})


def test_v2_nonfinite_cache_key_cannot_match_finite_angles(v2):
    n = fl.LINEAR_SLOT_JOINTS[0]
    finite, _ = offline.joint_angles_to_hand_positions({n: 1e30})
    nonfinite, _ = offline.joint_angles_to_hand_positions({n: float("nan")})
    assert finite[0] != nonfinite[0]
    assert abs(nonfinite[0]) < 1e-12


def test_missing_joint_contracts_are_explicit(v2):
    # V2's established partial-pose API supplies zero for omitted joints.
    linear, rotary = offline.joint_angles_to_hand_positions({})
    assert max(abs(x) for x in linear + rotary) < 1e-12
    a32 = A32StrokeMapper()
    pose = {n: 0.0 for n in a32.joint_names}
    del pose[a32.joint_names[-1]]
    assert not a32.preview(pose)["valid"]
    # Mixed-type unknown keys still produce a controlled validation error.
    pose.update({0: 1.0, "unexpected": 1.0})
    with pytest.raises(ValueError):
        a32.convert(pose)


@pytest.mark.parametrize("bad", [None, [], (), 0, "bad"])
def test_a32_preview_rejects_non_mapping_without_exception(bad):
    mapper = A32StrokeMapper()
    result = mapper.preview(bad)
    assert not result["valid"] and result["linear_strokes_mm"] == {}
    with pytest.raises(ValueError):
        mapper.convert(bad)


@pytest.mark.parametrize("bad", [None, [], {}, "0.3", True, np.bool_(True), 1j,
                                 float("nan"), float("inf"), 10**1000])
def test_a32_invalid_joint_is_explicit_and_next_frame_recovers(bad):
    mapper = A32StrokeMapper()
    good = {n: 0.0 for n in mapper.joint_names}
    result = mapper.preview({**good, mapper.joint_names[0]: bad})
    assert not result["valid"] and not result["rotary_positions_deg"]
    with pytest.raises(ValueError):
        mapper.convert({**good, mapper.joint_names[0]: bad})
    assert mapper.preview(good)["valid"]


@pytest.mark.parametrize("method", ["lut", "exact"])
@pytest.mark.parametrize("side", ["left", "right"])
def test_a32_all_joints_at_limit_nextafter_and_just_outside(method, side):
    mapper = A32StrokeMapper(side, method=method)
    neutral = {n: (lo + hi) / 2 for n, (lo, hi) in zip(mapper.joint_names, mapper._limits)}
    for name, (lo, hi) in zip(mapper.joint_names, mapper._limits):
        for value in (lo, hi, math.nextafter(lo, -math.inf), math.nextafter(lo, math.inf),
                      math.nextafter(hi, -math.inf), math.nextafter(hi, math.inf)):
            output = mapper.convert({**neutral, name: value})
            assert all(math.isfinite(x) for x in [*output.linear_strokes_mm.values(), *output.rotary_positions_deg.values()])
        for value in (lo - 1e-8, hi + 1e-8, -1e308, 1e308):
            with pytest.raises(ValueError):
                mapper.convert({**neutral, name: value})
            result = mapper.preview({**neutral, name: value})
            assert result["valid"]
            assert name in result["clamped_joints_rad"]
            assert lo <= result["joint_angles_rad"][name] <= hi
