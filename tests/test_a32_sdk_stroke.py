"""Verify the Python mapping against the bundled SDK's actual C++ converter."""
import ctypes
import hashlib
import json
import math
from pathlib import Path
import random
import re
import shutil
import subprocess
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from a32_kinematics import A32Error, A32StrokeMapper, sdk_stroke
from a32_kinematics.sdk_stroke_data import JOINT_ROWS, LOOP_ROWS
from a32_driver.limits import sdk_channel_limits


def test_extracted_coefficients_and_provenance_match_bundled_source():
    source = (ROOT / "vendor/a3-cpp-sdk/src/Stroke.cpp").read_bytes()
    assert hashlib.sha256(source).hexdigest() == sdk_stroke.SOURCE_SHA256
    rows = re.findall(r"\{(true|false), ([^,]+), ([^,]+), ([^,]+), ([^}]+)\}", source.decode())
    extracted = tuple((kind == "true", float(lo), float(hi), int(dest), int(loop))
                      for kind, lo, hi, dest, loop in rows)
    assert extracted == JOINT_ROWS
    links = tuple(tuple(float(x.strip()) / 1000. for x in row.split(","))
                  for row in re.findall(r"make_link\(([0-9., eE+-]+)\)", source.decode()))
    assert len(links) + 1 == len(LOOP_ROWS)
    for actual, expected in zip(links, LOOP_ROWS[1:]):
        assert actual == pytest.approx(expected, rel=0, abs=1e-15)
    assert len(extracted) == 20
    assert sum(joint.linear for joint in sdk_stroke.JOINTS) == 14
    assert next(j for j in sdk_stroke.JOINTS if j.destination == 1).linkage.stroke_hi == pytest.approx(.011761489775595662)


@pytest.fixture(scope="module")
def sdk_cpp(tmp_path_factory):
    # Compile only the pure upstream geometry; no transport or hardware API is
    # linked. This also prevents an old installed .so from becoming our oracle.
    compiler = shutil.which("c++")
    if compiler is None:
        pytest.skip("A C++ compiler is required for the upstream geometry oracle")
    folder = tmp_path_factory.mktemp("a32-sdk-stroke")
    wrapper = folder / "oracle.cpp"
    wrapper.write_text('''#include "a3/Stroke.h"
extern "C" int forward(const float* in, float* out) {
    return a3::anglesDegToActuator(in, out);
}
extern "C" int inverse(const float* in, float* out) {
    return a3::actuatorToAnglesDeg(in, out);
}
extern "C" int map_hand(const float* in, float* self, float* mit) {
    return a3::actuatorToHand(in, self, mit);
}
''')
    path = folder / "stroke.so"
    subprocess.run([compiler, "-std=c++17", "-O2", "-shared", "-fPIC",
                    "-I", str(ROOT / "vendor/a3-cpp-sdk/include"), str(wrapper),
                    str(ROOT / "vendor/a3-cpp-sdk/src/Stroke.cpp"), "-o", str(path)],
                   check=True, capture_output=True, text=True)
    lib = ctypes.CDLL(str(path))
    for name, count in (("forward", 2), ("inverse", 2), ("map_hand", 3)):
        method = getattr(lib, name)
        method.argtypes = [ctypes.POINTER(ctypes.c_float)] * count
        method.restype = ctypes.c_int
    return lib


@pytest.fixture(scope="module")
def native_converter(sdk_cpp):
    return sdk_cpp.forward


def test_pure_python_matches_sdk_cpp_over_linear_domain_and_clamp_boundaries(native_converter):
    rng = random.Random(1114)
    poses = []
    # Linear angles are degrees in the SDK, including its endpoint guards.
    for angle in (-90., 0., .004999, .005, .005001, 45., 89.995, 90., 200.):
        poses.append([angle if j.linear else math.degrees((j.lo + j.hi) / 2)
                      for j in sdk_stroke.JOINTS])
    for _ in range(300):
        poses.append([rng.uniform(-1., 91.) if j.linear else
                      math.degrees(rng.uniform(j.lo + 1e-5, j.hi - 1e-5))
                      for j in sdk_stroke.JOINTS])
    array20 = ctypes.c_float * 20
    for pose in poses:
        angles = array20(*pose)
        actual = array20()
        assert native_converter(angles, actual) == 1
        # Match float32 input. C++ uses a LUT within 1e-6 mm of the same
        # analytic curve, then rounds to float32 (up to another 1e-6 mm).
        expected = sdk_stroke.angles_deg_to_actuators(list(angles))
        assert list(actual) == pytest.approx(expected, rel=0, abs=4e-6)


def test_sdk_cpp_and_python_reject_invalid_rotary_or_nonfinite_angles(native_converter):
    array20 = ctypes.c_float * 20
    for index, value in ((0, -1.), (0, 121.), (1, 101.), (5, 21.), (5, -21.),
                         (2, float("nan")), (4, float("inf"))):
        pose = [0.] * 20
        pose[index] = value
        angles, output = array20(*pose), array20(*([123.] * 20))
        assert native_converter(angles, output) == 0
        assert list(output) == [123.] * 20
        with pytest.raises(A32Error):
            sdk_stroke.angles_deg_to_actuators(pose)


@pytest.mark.parametrize("model_id", ["TsenseI_v3_2", "TsenseI_v3_2_withid"])
@pytest.mark.parametrize("side", ["left", "right"])
def test_sdk_forward_inverse_roundtrip_all_application_channels(model_id, side):
    mapper = A32StrokeMapper(side, model_id=model_id)
    for fraction in (0., .00001, .1, .3, .5, .9, .95, .97, .99, .99999, 1.):
        pose = {name: lo + fraction * (hi - lo)
                for name, (lo, hi) in zip(mapper.joint_names, mapper._limits)}
        result = mapper.convert(pose)
        for name, value in {**result.linear_strokes_mm, **result.rotary_positions_deg}.items():
            assert mapper.inverse_actuator(name, value) == pytest.approx(pose[name], abs=4e-8)


def test_all_sdk_linkages_monotonic_and_inverse_includes_telemetry_endpoints():
    for index, joint in enumerate(sdk_stroke.JOINTS):
        if not joint.linear:
            continue
        values = [sdk_stroke.joint_angle_deg_to_actuator(index, .005 + i * 89.99 / 2048)
                  for i in range(2049)]
        assert all(a < b for a, b in zip(values, values[1:]))
        assert 0. < values[0] < values[-1] < joint.linkage.stroke_hi * 1000.
        assert sdk_stroke.inverse_actuator(index, 0.) == 0.
        assert sdk_stroke.inverse_actuator(index, joint.linkage.stroke_hi * 1000.) == joint.hi
        with pytest.raises(A32Error, match="StrokeLimit"):
            sdk_stroke.inverse_actuator(index, -.1)
        with pytest.raises(A32Error, match="StrokeLimit"):
            sdk_stroke.inverse_actuator(index, joint.linkage.stroke_hi * 1000. + .1)


def test_float32_full_travel_telemetry_rounding_is_accepted_for_all_linear_channels():
    for index, joint in enumerate(sdk_stroke.JOINTS):
        if not joint.linear:
            continue
        cap = joint.linkage.stroke_hi * 1000.
        received = ctypes.c_float(cap).value
        assert sdk_stroke.inverse_actuator(index, ctypes.c_float(0.).value) == 0.
        assert sdk_stroke.inverse_actuator(index, received) == pytest.approx(joint.hi, abs=5e-7)
        if received >= cap:
            assert sdk_stroke.inverse_actuator(index, received) == joint.hi
        assert sdk_stroke.inverse_actuator(index, cap + 1e-6) == joint.hi
        assert sdk_stroke.inverse_actuator(index, -1e-6) == 0.
        assert sdk_stroke.inverse_actuator(index, cap + 8e-6) == joint.hi
        assert sdk_stroke.inverse_actuator(index, -8e-6) == 0.
        for outside in (-2e-5, cap + 2e-5):
            with pytest.raises(A32Error, match="StrokeLimit"):
                sdk_stroke.inverse_actuator(index, outside)


@pytest.mark.parametrize("bad", [True, "0", None, float("nan"), float("inf"), 10**400])
def test_feedback_inverse_rejects_invalid_numeric_inputs(bad):
    mapper = A32StrokeMapper()
    with pytest.raises(ValueError):
        mapper.inverse_actuator("left_hand_index_MCP_joint", bad)
    with pytest.raises(ValueError):
        mapper.inverse_actuator("left_hand_index_swing_joint", bad)


def test_motor_size_curves_and_all_hardware_destinations(sdk_cpp):
    hardware = json.loads((ROOT / "src/a32_driver/a32_hardware.json").read_text())
    self_channels = {c["index"]: c for c in hardware["channels"] if c["channel"] == "self"}
    expected = {
        1: ({2, 5, 10, 13}, 19.627778913882647),
        2: ({4, 9, 12, 17}, 16.939021296767194),
        3: ({0, 3, 8, 11, 16}, 12.437476671033064),
        4: ({1}, 11.761489775595662),
    }
    for size, (slots, cap) in expected.items():
        indices = [i for i, j in enumerate(sdk_stroke.JOINTS) if j.loop == size]
        assert {sdk_stroke.JOINTS[i].destination for i in indices} == slots
        for angle in (0., .005, 5., 15., 30., 45., 60., 75., 89.995, 90.):
            values = [sdk_stroke.joint_angle_deg_to_actuator(i, angle) for i in indices]
            assert len(set(values)) == 1
        for slot in slots:
            assert sdk_channel_limits("self", slot) == pytest.approx((0., cap), abs=1e-12, rel=0)
            assert self_channels[slot]["limits"] == [0., cap]
    for slot in (6, 7, 14, 15):
        assert sdk_channel_limits("self", slot) == (-20., 20.)
        assert self_channels[slot]["limits"] == pytest.approx(
            [math.degrees(-.34), math.degrees(.34)], abs=1e-12, rel=0)
    # Distinct values expose accidental swaps across all 18 SELF and 2 MIT.
    inputs = (ctypes.c_float * 20)(*range(20))
    self_out, mit_out = (ctypes.c_float * 18)(), (ctypes.c_float * 2)()
    assert sdk_cpp.map_hand(inputs, self_out, mit_out) == 1
    for i, joint in enumerate(sdk_stroke.JOINTS):
        assert (self_out[joint.destination] if joint.destination >= 0
                else mit_out[-joint.destination - 1]) == i
    assert self_out[17] == 2 and mit_out[0] == 1 and mit_out[1] == 0


def test_analytic_inverse_matches_sdk_cpp_across_all_curves_and_rotary_signs(sdk_cpp):
    array20 = ctypes.c_float * 20
    for fraction in (0., 1e-7, .001, .1, .25, .5, .75, .9, .999, 1. - 1e-7, 1.):
        values = [j.linkage.stroke_hi * 1000. * fraction if j.linear else
                  math.degrees(j.lo + (j.hi - j.lo) * fraction)
                  for j in sdk_stroke.JOINTS]
        inputs, actual = array20(*values), array20()
        assert sdk_cpp.inverse(inputs, actual) == 1
        expected = [math.degrees(sdk_stroke.inverse_actuator(i, value))
                    for i, value in enumerate(inputs)]
        assert list(actual) == pytest.approx(expected, abs=8e-6, rel=0)


def test_inverse_feedback_tolerances_and_rejection_match_cpp(sdk_cpp):
    array20 = ctypes.c_float * 20
    for index, joint in enumerate(sdk_stroke.JOINTS):
        for is_upper in (False, True):
            endpoint = (joint.linkage.stroke_hi * 1000. if is_upper else 0.) if joint.linear else \
                math.degrees(joint.hi if is_upper else joint.lo)
            tolerance = 1e-5 if joint.linear else math.degrees(1e-6)
            direction = 1 if is_upper else -1
            for fraction, accepted in ((.5, True), (2., False)):
                values = [j.linkage.stroke_hi * 500. if j.linear else
                          math.degrees((j.lo + j.hi) / 2.) for j in sdk_stroke.JOINTS]
                values[index] = endpoint + direction * tolerance * fraction
                inputs, output = array20(*values), array20(*([123.] * 20))
                assert sdk_cpp.inverse(inputs, output) == int(accepted)
                if accepted:
                    actual = sdk_stroke.inverse_actuator(index, inputs[index])
                    assert actual == pytest.approx(joint.hi if is_upper else joint.lo, abs=1e-12)
                else:
                    assert list(output) == [123.] * 20
                    with pytest.raises(A32Error):
                        sdk_stroke.inverse_actuator(index, inputs[index])
