"""Full ID-shell finger pads/tips participate in proximity and stop checks."""
from pathlib import Path
import sys
from xml.sax.saxutils import quoteattr

import numpy as np
import pytest
import trimesh
from scipy.spatial import ConvexHull
import yaml

mujoco = pytest.importorskip("mujoco")
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src"), str(ROOT / "tools")]
from hand_profiles import load_hand_profile
from TsenseI_retargeting.fk.urdf import UrdfModel
from prepare_hand_v3_2_withid import mesh_points

PROFILE = load_hand_profile("TsenseI_v3_2_withid")
GEOMETRY = yaml.safe_load((PROFILE.root / "collision_geometry.yaml").read_text(encoding="utf-8"))
DISTAL = [spec for spec in GEOMETRY["geoms"] if spec.get("region") == "distal"]
assert len(DISTAL) == 5


@pytest.mark.parametrize("spec", DISTAL, ids=lambda s: s["finger"])
def test_distal_hull_encloses_entire_visual_shell_and_keeps_extents(spec):
    model = UrdfModel(PROFILE.path("urdf"))
    vertices = np.unique(mesh_points(model, spec["link"]), axis=0)
    mesh = trimesh.load_mesh(PROFILE.path("mjcf").parent / spec["mesh_files"]["left"], process=False)
    hull = ConvexHull(mesh.vertices)
    # The hull contains all source triangles because it is convex and contains
    # all their vertices. No proximal housing, finger pad, or tapered tip is
    # discarded by a narrow distal sampling band.
    for batch in np.array_split(vertices, max(1, len(vertices) // 256)):
        signed = batch @ hull.equations[:, :3].T + hull.equations[:, 3]
        assert signed.max() < 2e-8
    # Generation adds 0.2 um outward and quantizes at 0.1 um before rebuilding
    # the float32 convex hull, retaining the CAD extents within 0.3 um.
    np.testing.assert_allclose(mesh.bounds, [vertices.min(axis=0), vertices.max(axis=0)], atol=3e-7)
    welded = mesh.copy()
    welded.merge_vertices(digits_vertex=15)  # Weld only identical STL face vertices.
    assert welded.is_watertight and welded.is_convex
    right = trimesh.load_mesh(PROFILE.path("mjcf", "right").parent / spec["mesh_files"]["right"], process=False)
    # Convex triangulation may select different diagonals/order on coplanar
    # faces after reflection. Equal vertex sets define the same convex body.
    np.testing.assert_array_equal(np.unique(right.vertices, axis=0),
                                  np.unique(mesh.vertices * [1, -1, 1], axis=0))


@pytest.mark.parametrize("spec", DISTAL, ids=lambda s: s["finger"])
@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("surface,axis,reflection,quat", [
    ("finger_pad", 0, [-1, -1, 1], [0, 0, 0, 1]),
    ("fingertip", 2, [1, -1, -1], [0, 1, 0, 0]),
])
def test_distal_surfaces_detect_clearance_then_contact(spec, side, surface, axis, reflection, quat):
    path = PROFILE.path("mjcf", side).parent / spec["mesh_files"][side]
    vertices = trimesh.load_mesh(path, process=False).vertices
    anchor = vertices[np.argmax(vertices[:, axis])]
    for gap in (.001, -.0005):
        translation = anchor - anchor * reflection
        translation[axis] += gap
        xml = f'''<mujoco><asset><mesh name="tip" file={quoteattr(str(path))}/></asset>
          <worldbody><body name="fixed"><geom name="fixed" type="mesh" mesh="tip" contype="1" conaffinity="2"/></body>
            <body name="moving" pos="{' '.join(map(str, translation))}" quat="{' '.join(map(str, quat))}">
              <freejoint/><geom name="moving" type="mesh" mesh="tip" mass=".01" contype="2" conaffinity="1"/>
            </body></worldbody></mujoco>'''
        model = mujoco.MjModel.from_xml_string(xml)
        data = mujoco.MjData(model)
        mujoco.mj_forward(model, data)
        distance = mujoco.mj_geomDistance(model, data, 0, 1, .1, np.zeros(6))
        if gap > 0:
            assert data.ncon == 0
            assert distance == pytest.approx(gap, abs=2e-8)
        else:
            assert data.ncon > 0, f"{spec['finger']} {surface} contact was missed"
            assert distance < -.0001


@pytest.mark.parametrize("side", ["left", "right"])
def test_full_hand_tip_contacts_reach_stop_guard(side):
    from online_control.online_motion_track.pipeline_v3_0.collision import StopCollisionGuard
    guard = StopCollisionGuard(PROFILE, side)
    assert guard.filter.data.ncon == 0
    limits = PROFILE.joint_limits(side)
    rng = np.random.default_rng(0)  # Crossing distal shells inside the new joint ranges.
    crossed = {name: rng.uniform(lo, hi) for name, (lo, hi) in limits.items()}
    guard.filter.sync_pose_from_dict(crossed)
    pairs = guard.filter.collision_pair_distances()
    tip_names = {spec["link"].replace("left_", side + "_") + "_collision" for spec in DISTAL}
    assert any(row["g1"] in tip_names and row["g2"] in tip_names
               and row["distance_mm"] < -.5 for row in pairs)
    guard.filter.sync_pose_from_dict(PROFILE.init_pose(side))
    projected = guard.project(crossed)
    assert guard.diagnostics["limited"]
    assert any(abs(projected[name] - crossed[name]) > 1e-4 for name in crossed)
    guard.filter.sync_pose_from_dict(projected)
    assert all(row["distance_mm"] >= -1e-4 for row in guard.filter.collision_pair_distances())
