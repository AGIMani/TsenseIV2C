"""Validate collision envelopes against the actual transformed CAD shells."""
from pathlib import Path
import hashlib
import io
import json
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest
from scipy.spatial import ConvexHull
import trimesh
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src"), str(ROOT / "tools")]
from hand_profiles import A30_MODEL_IDS, A32_MODEL_IDS, load_hand_profile
from TsenseI_retargeting.fk.urdf import UrdfModel
from collision_outer_hull import (DEFAULT_ERROR, VERSION, containment_violation,
                                  convex_mesh, max_excess, outer_envelope,
                                  source_points, update_urdf_collisions)


@pytest.mark.parametrize("model_id", [*A30_MODEL_IDS, *A32_MODEL_IDS])
@pytest.mark.parametrize("side", ["left", "right"])
def test_every_canonical_collision_encloses_visual_after_stl_roundtrip(model_id, side):
    profile = load_hand_profile(model_id)
    urdf = UrdfModel(profile.path("urdf", side))
    report = json.loads((profile.root / "collision_mesh_report.json").read_text())
    assert report["version"] == VERSION
    rows = {row["link"]: row for row in report["links"] if row["side"] == side}
    nodes = {link.get("name"): link for link in urdf.root.findall("link")
             if link.findall("collision")}
    assert rows.keys() == nodes.keys()
    for link, node in nodes.items():
        row = rows[link]
        collision, = node.findall("collision")
        mesh_path = (urdf.urdf_path.parent / collision.find("geometry/mesh").get("filename")).resolve()
        assert mesh_path == profile.root / row["mesh"]
        assert collision.find("origin").get("xyz") == "0 0 0"
        assert collision.find("origin").get("rpy") == "0 0 0"
        assert hashlib.sha256(mesh_path.read_bytes()).hexdigest() == row["stl_sha256"]
        points = source_points(urdf, link)
        assert hashlib.sha256(points.tobytes()).hexdigest() == row["source_sha256"]
        vertices = np.unique(trimesh.load_mesh(mesh_path, process=False).vertices, axis=0)
        welded = trimesh.load_mesh(mesh_path, process=True)
        assert welded.is_watertight, link
        assert welded.is_convex, link
        assert len(vertices) <= row["full_hull_vertices"]
        assert len(vertices) == row["vertices"]
        assert containment_violation(points, ConvexHull(vertices).equations) <= 1e-10
        # Recompute the geometric bound, rather than trusting saved metadata.
        _, hull = convex_mesh(points)
        _, proxy = convex_mesh(vertices)
        assert max_excess(proxy, hull) <= DEFAULT_ERROR
        assert row["source_contained"]


@pytest.mark.parametrize("model_id", [*A30_MODEL_IDS, *A32_MODEL_IDS])
@pytest.mark.parametrize("side", ["left", "right"])
def test_runtime_meshes_share_urdf_envelopes_preserve_pairs_and_initial_pose(model_id, side):
    profile = load_hand_profile(model_id)
    geometry = yaml.safe_load((profile.root / "collision_geometry.yaml").read_text())
    spec = geometry["geoms"]
    finger_specs = [g for g in spec if g.get("finger")]
    assert len(finger_specs) == 15
    assert all(g["type"] == "mesh" for g in finger_specs)
    assert {g["region"] for g in finger_specs} == {"proximal", "intermediate", "distal"}
    assert sum(g["region"] == "distal" for g in finger_specs) == 5
    model = mujoco.MjModel.from_xml_path(str(profile.path("mjcf", side)))
    data = mujoco.MjData(model)
    expected = 15 if model_id.endswith("_withid") else 16
    assert model.ngeom == expected
    assert np.count_nonzero(model.geom_type == mujoco.mjtGeom.mjGEOM_MESH) == expected
    for name, value in profile.init_pose(side).items():
        data.qpos[model.joint(name).qposadr] = value
    mujoco.mj_forward(model, data)
    assert data.ncon == 0
    pairs = [(a, b) for a in range(model.ngeom) for b in range(a + 1, model.ngeom)
             if (model.geom_contype[a] & model.geom_conaffinity[b]) or
                (model.geom_contype[b] & model.geom_conaffinity[a])]
    assert len(pairs) == (90 if model_id.endswith("_withid") else 100)
    urdf = ET.parse(profile.path("urdf", side)).getroot()
    for g in spec:
        link = g["link"].replace("left_hand", f"{side}_hand").replace("Left_hand", f"{side.title()}_hand")
        runtime_file = profile.path("mjcf", side).parent / g["mesh_files"][side]
        urdf_file = urdf.find(f"link[@name='{link}']/collision/geometry/mesh").get("filename")
        assert runtime_file.resolve() == (profile.path("urdf", side).parent / urdf_file).resolve()


def test_outer_hull_reduces_curved_shell_and_is_deterministic():
    sphere = trimesh.creation.icosphere(subdivisions=3)
    points = sphere.vertices * np.array([.012, .009, .020]) + [.004, -.002, .008]
    raw, metrics = outer_envelope(points)
    repeated, _ = outer_envelope(points[::-1])
    assert repeated == raw
    assert metrics["vertices"] < len(points) / 2
    assert metrics["max_excess_mm"] <= DEFAULT_ERROR * 1000
    mirror = np.array([1, -1, 1])
    mirrored, _ = outer_envelope(points * mirror, fit_mirror=mirror)
    a = np.unique(trimesh.load_mesh(io.BytesIO(raw), file_type="stl", process=False).vertices, axis=0)
    b = np.unique(trimesh.load_mesh(io.BytesIO(mirrored), file_type="stl", process=False).vertices * mirror, axis=0)
    np.testing.assert_array_equal(a, b)


def test_collision_rewrite_preserves_noncollision_xml_and_skips_empty_links(tmp_path):
    path = tmp_path / "sample.urdf"
    before = '''<robot name="fixture"><link name="empty" />
  <link name="part"><visual><origin xyz="1 2 3"/><geometry><mesh filename="cad.stl" scale=".001 .001 .001"/></geometry></visual>
    <collision><origin xyz="1 2 3"/><geometry><mesh filename="cad.stl"/></geometry></collision>
    <inertial><mass value="0.023" /></inertial></link>
  <joint name="unchanged" type="fixed"><origin xyz="4 5 6"/></joint></robot>'''
    path.write_text(before)
    update_urdf_collisions(path, {"part": "../mjcf/collision_meshes/part.stl"})
    after = path.read_text()
    import re
    without = lambda text: re.sub(r"<collision\b[^>]*>.*?</collision>", "", text, flags=re.S)
    assert without(before) == without(after)
    update_urdf_collisions(path, {"part": "../mjcf/collision_meshes/part.stl"})
    assert path.read_text() == after


def test_duplicate_collision_elements_merge_without_trailing_whitespace(tmp_path):
    path = tmp_path / "duplicate.urdf"
    path.write_text('''<robot name="fixture"><link name="part">
    <collision><geometry><mesh filename="a.stl"/></geometry></collision>
    <collision><geometry><mesh filename="b.stl"/></geometry></collision>
  </link></robot>''')
    update_urdf_collisions(path, {"part": "../mjcf/collision_meshes/part.stl"})
    assert len(ET.parse(path).findall("link/collision")) == 1
    assert all(line == line.rstrip() for line in path.read_text().splitlines())


@pytest.mark.parametrize("model_id,thumb_values", [
    ("TsenseI_v3_0", (1.7712374416868377, .07868765011567211, .19510249265138538, .03854520323890325)),
    ("TsenseI_v3_2", (1.5648755431926709, .009268586544326354, .0449213929133806, .11128156333299014)),
])
@pytest.mark.parametrize("side", ["left", "right"])
def test_full_palm_mesh_contact_triggers_stop(model_id, thumb_values, side):
    from online_control.online_motion_track.pipeline_v3_0.collision import StopCollisionGuard
    profile = load_hand_profile(model_id)
    target = profile.init_pose(side)
    for suffix, value in zip(("rota_block_joint", "rotaback_joint2", "MCP_joint", "PIP_joint"), thumb_values):
        target[f"{side}_hand_thumb_{suffix}"] = value
    model = mujoco.MjModel.from_xml_path(str(profile.path("mjcf", side)))
    data = mujoco.MjData(model)
    for name, value in target.items():
        data.qpos[model.joint(name).qposadr] = value
    mujoco.mj_forward(model, data)
    assert any(contact.dist < -.0002 and any("palm" in model.geom(g).name for g in (contact.geom1, contact.geom2))
               for contact in data.contact)
    guard = StopCollisionGuard(profile, side)
    result = guard.project(target)
    assert guard.diagnostics["limited"]
    assert any("palm" in c["g1"] or "palm" in c["g2"] for c in guard.diagnostics["contacts"])
    for name, value in result.items():
        data.qpos[model.joint(name).qposadr] = value
    mujoco.mj_forward(model, data)
    assert data.ncon == 0
