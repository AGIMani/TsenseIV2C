"""Prepared V2 lookup parity, error bounds and mutable configuration behavior."""
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from kinematics import finger_linkage as fl


@pytest.mark.parametrize("motor", [fl.MOTOR_F1, fl.MOTOR_F2, fl.MOTOR_F3])
def test_uniform_error_bound_covers_knots_cells_and_endpoints(motor):
    p = fl.FingerLinkageParams.from_matlab_defaults()
    source = fl._get_phi_cf_lut(motor, p)
    table = fl._get_prepared_motors(p)[motor].table
    assert table.max_source_error_mm <= fl._UNIFORM_LUT_ERROR_MM
    probes = list(source.phi)
    # All new cell quarter/midpoints; maxima are bounded by the old knots.
    for i in range(len(table.values) - 1):
        probes.extend(table.lo + (i + fraction) / table.scale for fraction in (.25, .5, .75))
    maximum = max(abs(table.eval(phi) - source.eval(phi)) for phi in probes)
    assert maximum <= table.max_source_error_mm + 1e-12
    assert table.eval(table.lo - 1) == source.cf[0]
    assert table.eval(table.hi + 1) == source.cf[-1]
    with pytest.raises(fl.LinkageKinematicsError):
        table.eval(float("nan"))


@pytest.mark.parametrize("motor", [fl.MOTOR_F1, fl.MOTOR_F2, fl.MOTOR_F3])
def test_prepared_matches_original_soft_limits_and_nonfinite(motor):
    p = fl.FingerLinkageParams.from_matlab_defaults()
    kernel = fl._get_prepared_motors(p)[motor]
    edge = math.radians(kernel.hi - kernel.blend)
    radians = [i / 300 - .2 for i in range(1100)] + [edge - 1e-8, edge, edge + 1e-8,
               -1e308, 1e308, float("nan"), float("inf"), -float("inf")]
    for rad in radians:
        limited = fl.clamp_joint_rad_for_motor(rad, motor, p)
        expected = fl.phi_deg_to_cf_mm(math.degrees(limited), motor, p, exact=False)
        assert abs(kernel.joint_rad_to_cf_mm(rad) - expected) <= fl._UNIFORM_LUT_ERROR_MM + 1e-12


def test_geometry_mutation_and_explicit_clear_refresh_prepared_state():
    p = fl.FingerLinkageParams.from_matlab_defaults()
    old = fl._get_prepared_motors(p)
    # Use a list to cover in-place user configuration changes as well as tuple replacement.
    p.cf_max_mm = list(p.cf_max_mm)
    p.cf_max_mm[fl.MOTOR_F3] *= .7
    changed = fl._get_prepared_motors(p)
    assert changed is not old
    assert changed[fl.MOTOR_F3].hi < old[fl.MOTOR_F3].hi
    fl.clear_phi_cf_lut_cache()
    rebuilt = fl._get_prepared_motors(p)
    assert rebuilt is not changed
    assert rebuilt == changed


def test_prepared_runtime_does_not_resolve_tables_or_limits_again(monkeypatch):
    mapper = fl.create_default_hand_stroke_mapper()
    pose = {name: .4 for name in mapper.managed_linear_joints()}
    expected = mapper.linear_strokes_mm(pose)
    def forbidden(*args, **kwargs):
        pytest.fail("Prepared runtime unexpectedly resolves geometry/lookup")
    monkeypatch.setattr(fl, "_get_phi_cf_lut", forbidden)
    monkeypatch.setattr(fl, "motor_phi_deg_limits", forbidden)
    assert mapper.linear_strokes_mm(pose) == expected


def test_exact_mode_bypasses_prepared_lookup(monkeypatch):
    monkeypatch.setattr(fl, "_LINKAGE_FIT_DEFAULT", False)
    mapper = fl.create_default_hand_stroke_mapper()
    def forbidden(*args, **kwargs):
        pytest.fail("HAND_LINKAGE_FIT=0 must not use prepared lookup")
    monkeypatch.setattr(fl, "_get_prepared_motors", forbidden)
    pose = {name: .4 for name in mapper.managed_linear_joints()}
    result = mapper.linear_strokes_mm(pose)
    assert len(result) == 14
    assert all(value < -.1 for value in result.values())


def test_unavailable_uniform_table_keeps_original_path(monkeypatch):
    fl.clear_phi_cf_lut_cache()
    def unavailable(*args, **kwargs):
        raise fl.LinkageKinematicsError("Cannot certify uniform grid")
    monkeypatch.setattr(fl, "_uniform_phi_cf_lut", unavailable)
    try:
        mapper = fl.create_default_hand_stroke_mapper()
        pose = {name: .4 for name in mapper.managed_linear_joints()}
        result = mapper.linear_strokes_mm(pose)
        assert all(value < -.1 for value in result.values())
        assert fl._get_prepared_motors(mapper.four_finger.params) == (None, None, None)
    finally:
        fl.clear_phi_cf_lut_cache()


def test_thumb_does_not_depend_on_unused_f2_geometry():
    mapper = fl.ThumbStrokeMapper()
    values = list(mapper.params.rod_length_mm)
    values[fl.MOTOR_F2] = 1000.0
    mapper.params.rod_length_mm = tuple(values)
    result = mapper.stroke_mm_for_thumb({mapper.mcp_joint: .4, mapper.pip_joint: .3})
    assert len(result) == 2 and all(value < -.1 for value in result.values())
