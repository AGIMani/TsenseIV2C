"""A3.0 CAD preservation, reflection, retarget and web isolation regressions."""
from pathlib import Path
import json
import sys
import threading
import urllib.request

import numpy as np
import pytest
import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src"), str(ROOT / "tools")]
from hand_profiles import load_hand_profile
from TsenseI_retargeting.fk.urdf import UrdfModel
from TsenseI_retargeting.fk.fk import RobotKeypointFK
from prepare_hand_v3_0 import SOURCE_URDF, RENAMES, MIRROR, right_name
from online_control.online_motion_track.main import WebSession, make_server, model_description


def test_preserve_source_geometry_and_mirror_all_link_poses():
    profile = load_hand_profile("TsenseI_v3_0")
    source = UrdfModel(SOURCE_URDF)
    left, right = [UrdfModel(profile.path("urdf", s)) for s in ("left", "right")]
    assert len(left.joint_names()) == 20
    assert not left.missing_meshes() and not right.missing_meshes()
    reflection = np.eye(4); reflection[:3, :3] = MIRROR
    rng = np.random.default_rng(30)
    for _ in range(5):
        q = {n: rng.uniform(lo, hi) for n, (lo, hi) in source.joint_limits().items()}
        ql = {RENAMES.get(n, n): v for n, v in q.items()}
        a, b, c = source.forward_kinematics(q), left.forward_kinematics(ql), right.forward_kinematics({right_name(n): v for n, v in ql.items()})
        for name, pose in a.items():
            new = RENAMES.get(name, name)
            np.testing.assert_allclose(b[new], pose, atol=1e-12)
            np.testing.assert_allclose(c[right_name(new)], reflection @ pose @ reflection, atol=1e-12)
    for name, visuals in source.visual_meshes_by_link().items():
        ln = RENAMES.get(name, name)
        for original, lv, rv in zip(visuals, left.visual_meshes_by_link()[ln], right.visual_meshes_by_link()[right_name(ln)], strict=True):
            assert original.path.read_bytes() == lv.path.read_bytes()
            lm, rm = trimesh.load_mesh(lv.path, process=False), trimesh.load_mesh(rv.path, process=False)
            np.testing.assert_allclose(rm.triangles, lm.triangles[:, ::-1] @ MIRROR, atol=1e-8)


def test_keypoints_follow_mirrored_motion_and_display_meshes_load():
    profile = load_hand_profile("TsenseI_v3_0")
    left, right = [RobotKeypointFK(profile.path("urdf", s), profile.path("keypoints", s)) for s in ("left", "right")]
    for fraction in (0, .4, .8):
        q = {n: lo + fraction * (hi - lo) for n, (lo, hi) in profile.joint_limits().items()}
        np.testing.assert_allclose(right.compute({right_name(n): v for n, v in q.items()}), left.compute(q) @ MIRROR, atol=1e-12)
    visuals = UrdfModel(profile.path("urdf")).visual_meshes_by_link()
    meshes = [trimesh.load_mesh(v.path, process=False) for group in visuals.values() for v in group]
    assert len(meshes) == 22
    assert all(len(mesh.triangles) > 0 for mesh in meshes)
