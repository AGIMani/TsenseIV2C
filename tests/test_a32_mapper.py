"""A3.2 units, model isolation, CAD name mapping and deployed conversion."""
from pathlib import Path
import math
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src"), str(ROOT / "tools")]
from a32_kinematics import A32StrokeMapper, A32Error
from a32_kinematics.a32_angles_to_inputs import MODEL_DIR
from hand_profiles import A32_MODEL_IDS, load_hand_profile
from a32_kinematics.joint_coordinates import initial_pose, reverse_swing
from a32_kinematics import sdk_stroke


def test_legacy_method_selectors_use_versioned_sdk_conversion():
    mapper = A32StrokeMapper()
    assert mapper.method == "lut"
    assert mapper.description()["method"] == "lut"
    assert mapper.description()["conversion"] == "sdk-1.1.15"
    assert not hasattr(mapper, "_prepared")
    assert A32StrokeMapper(method="exact").method == "exact"


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("method,tolerance_mm", [("exact", 1e-9), ("lut", 1e-6)])
def test_named_outputs_match_reference_and_units(side, method, tolerance_mm):
    mapper = A32StrokeMapper(side, method=method)
    pose = {n: sum(limits) / 2 for n, limits in zip(mapper.joint_names, mapper._limits)}
    before = dict(pose)
    result = mapper.convert(pose)
    expected = sdk_stroke.angles_deg_to_actuators(
        [math.degrees(pose[n]) for n in mapper.joint_names])
    assert pose == before
    assert len(result.linear_strokes_mm) == 14 and len(result.rotary_positions_deg) == 6
    assert result.clamped_joints_rad == {}
    assert set(pose) == set(load_hand_profile("TsenseI_v3_2").joint_limits(side))
    for channel, value in zip(mapper.description()["channels"], expected):
        name = channel["joint"]
        if channel["type"] == "linear":
            assert result.linear_strokes_mm[name] == pytest.approx(value, abs=tolerance_mm, rel=0)
        else:
            assert result.rotary_positions_deg[name] == pytest.approx(value, abs=1e-12)
    assert f"{side}_hand_pinky_MCP_joint" in result.linear_strokes_mm
    assert f"{side}_hand_pinky_swing_joint" in result.rotary_positions_deg


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("method", ["exact", "lut"])
def test_pinky_flexion_and_signed_swing_remain_independent(side, method):
    mapper = A32StrokeMapper(side, method=method)
    pose = load_hand_profile("TsenseI_v3_2").init_pose(side)
    mcp, swing = (f"{side}_hand_pinky_{part}_joint" for part in ("MCP", "swing"))
    pose[mcp] = 1.0  # Legal flexion must not be clamped to swing's 0.35 rad.
    expected = sdk_stroke.joint_angle_deg_to_actuator(16, math.degrees(pose[mcp]))
    for angle in (-.35, -.15, 0., .15, .35):
        pose[swing] = angle
        strict = mapper.convert(pose)
        preview = mapper.preview(pose)
        assert preview["valid"] and not preview["clamped_joints_rad"]
        assert preview["joint_angles_rad"][mcp] == 1.0
        assert strict.linear_strokes_mm[mcp] == pytest.approx(expected, abs=1e-6, rel=0)
        assert preview["linear_strokes_mm"][mcp] == strict.linear_strokes_mm[mcp]
        assert strict.rotary_positions_deg[swing] == pytest.approx(math.degrees(angle), abs=1e-12)
        assert preview["rotary_positions_deg"][swing] == strict.rotary_positions_deg[swing]
        assert mcp not in strict.rotary_positions_deg and swing not in strict.linear_strokes_mm


def test_project_cad_matches_supplied_source_after_explicit_renames():
    """Names alone are insufficient: verify axes, origins, limits and chain."""
    mapper = A32StrokeMapper()
    from apply_a32_joint_ranges import apply_urdf_coordinates
    source = {j.get("name"): j for j in apply_urdf_coordinates(ET.parse(MODEL_DIR / "A3.2_source.urdf").getroot()).findall("joint")}
    target = {j.get("name"): j for j in ET.parse(load_hand_profile("TsenseI_v3_2").path("urdf")).getroot().findall("joint")}
    active_children = {j.find("child").get("link"): name for name, j in source.items() if j.get("type") != "fixed"}
    target_children = {j.find("child").get("link"): name for name, j in target.items() if j.get("type") != "fixed"}
    rename = dict(zip(mapper.source_joint_names, mapper.joint_names))
    for src_name, dst_name in rename.items():
        a, b = source[src_name], target[dst_name]
        for node, attr in (("axis", "xyz"), ("origin", "xyz"), ("origin", "rpy"), ("limit", "lower"), ("limit", "upper")):
            np.testing.assert_allclose(np.fromstring(a.find(node).get(attr), sep=" "),
                                       np.fromstring(b.find(node).get(attr), sep=" "), atol=1e-12, rtol=0)
        ancestor = active_children.get(a.find("parent").get("link"))
        assert rename.get(ancestor) == target_children.get(b.find("parent").get("link"))


def test_strict_limits_and_explicit_preview_clamp():
    mapper = A32StrokeMapper()
    pose = load_hand_profile("TsenseI_v3_2").init_pose()
    pose["left_hand_index_MCP_joint"] = 1.58
    with pytest.raises(A32Error, match="AngleLimit"):
        mapper.convert(pose)
    preview = mapper.preview(pose)
    assert preview["valid"] and pose["left_hand_index_MCP_joint"] == 1.58
    assert set(preview["clamped_joints_rad"]) == {"left_hand_index_MCP_joint"}
    assert preview["clamped_joints_rad"]["left_hand_index_MCP_joint"] == 1.57
    pose["left_hand_index_MCP_joint"] = float("nan")
    failed = mapper.preview(pose)
    assert not failed["valid"] and not failed["linear_strokes_mm"]


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("method", ["exact", "lut"])
def test_extended_mcp_range_uses_sdk_full_linear_domain(model_id, side, method):
    mapper = A32StrokeMapper(side, method=method, model_id=model_id)
    pose = load_hand_profile(model_id).init_pose(side)
    for channel in mapper.description()["channels"]:
        name = channel["joint"]
        if not name.endswith("_MCP_joint"):
            continue
        upper = 1.57
        assert channel["limits_rad"] == [0., upper]
        for angle in (0., upper):
            pose[name] = angle
            result = mapper.convert(pose)
            reference = sdk_stroke.joint_angle_deg_to_actuator(channel["motor_index"] - 1, math.degrees(angle))
            assert result.linear_strokes_mm[name] == pytest.approx(reference, abs=1e-9, rel=0)
            assert mapper.inverse_actuator(name, reference) == pytest.approx(angle, abs=4e-8)
            assert result.joint_angles_rad[name] == angle
            assert not result.clamped_joints_rad


def test_v2_and_wrong_side_poses_are_rejected():
    mapper = A32StrokeMapper()
    with pytest.raises(ValueError, match="own complete 20-joint"):
        mapper.convert(load_hand_profile("TsenseI_v2").init_pose())
    with pytest.raises(ValueError, match="own complete 20-joint"):
        mapper.convert(load_hand_profile("TsenseI_v3_2").init_pose("right"))
    with pytest.raises(ValueError, match="own complete 20-joint"):
        mapper.convert({})


def test_v2_globals_unchanged_by_a32_conversion():
    from usb import offline
    cfg = {name: .3 for name in offline.JOINT_ORDER}
    before = offline.joint_angles_to_hand_positions(cfg)
    v2_mapper = offline._ensure_hand_stroke_mapper()
    a32 = A32StrokeMapper()
    a32.convert(load_hand_profile("TsenseI_v3_2").init_pose())
    assert offline._ensure_hand_stroke_mapper() is v2_mapper
    assert offline.joint_angles_to_hand_positions(cfg) == before


def test_minimal_bundle_contains_independent_a32_runtime(tmp_path):
    from online_control.online_motion_track.package import files
    paths = dict((relative, source) for source, relative in files())
    dependencies = {"app/src/utils/__init__.py", "app/src/utils/control_session_recorder.py",
                    "app/src/utils/control_log_catalog.py", "app/src/config_path.py",
                    "app/src/utils/comm_diag.py", "app/src/utils/log_control.py", "app/src/utils/tracker_config.py"}
    assert dependencies <= paths.keys()
    assert "app/src/a32_driver/recording.py" in paths
    assert "docs/a32_motion_recording.md" in paths
    assert not any("/a32_driver/bin/staged/" in path for path in paths)
    for relative, source in paths.items():
        if (relative.startswith("app/src/a32_kinematics/") or relative in dependencies or
                relative.startswith("app/src/a32_driver/") and source.suffix in (".py", ".json")):
            target = tmp_path / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(source.read_bytes())
    code = """
import sys
import ctypes
ctypes.CDLL = lambda *a, **k: (_ for _ in ()).throw(AssertionError('native library loaded'))
from a32_kinematics import A32StrokeMapper
m = A32StrokeMapper()
from a32_kinematics.joint_coordinates import initial_pose
r = m.convert(initial_pose(dict(zip(m.joint_names, m._limits))))
assert len(r.linear_strokes_mm) == 14
assert len(r.rotary_positions_deg) == 6
from a32_driver.recording import A32SessionRecorder
import json
recorder = A32SessionRecorder('logs', model_id='TsenseI_v3_2_withid')
path = recorder.start()
recorder.record_state({'connected':False, 'telemetry_age_ms':-1})
recorder.stop()
rows = [json.loads(line) for line in path.read_text(encoding='utf-8').splitlines()]
assert [row['type'] for row in rows] == ['session', 'sdk_status', 'session_end']
assert rows[0]['metadata']['model_id'] == 'TsenseI_v3_2_withid'
assert all('wall_time' in row and 'wall_time_unix_ms' in row for row in rows)
assert not any(n.startswith(('kinematics', 'usb', 'protocol')) for n in sys.modules)
"""
    result = subprocess.run([sys.executable, "-c", code], cwd=tmp_path,
                            env={**os.environ, "PYTHONPATH": str(tmp_path / "app/src")}, capture_output=True, text=True)
    assert result.returncode == 0, result.stderr
