from pathlib import Path
import sys

import numpy as np
import pytest
import mujoco

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src"), str(ROOT / "tools")]
from hand_profiles import HandModelSession, load_hand_profile
from TsenseI_retargeting.fk.urdf import UrdfModel
from prepare_hand_v3_2 import RENAMES, right_name, SOURCE_URDF, MIRROR
from a32_kinematics.joint_coordinates import legacy_to_model, thumb_rotaback_origin_offset


def test_source_manifest_matches_current_cad_package():
    import hashlib
    import yaml
    from prepare_hand_v3_2 import source_fingerprint, source_reference
    profile = load_hand_profile("TsenseI_v3_2")
    mapping = yaml.safe_load((profile.root / "source_mapping.yaml").read_text(encoding="utf-8"))
    assert mapping["source"] == source_reference()
    assert mapping["source_urdf_sha256"] == hashlib.sha256(SOURCE_URDF.read_bytes()).hexdigest()
    assert mapping["source_package_sha256"] == source_fingerprint()


def test_source_fingerprint_detects_in_place_mesh_replacement(tmp_path, monkeypatch):
    import prepare_hand_v3_2 as builder
    (tmp_path / "meshes").mkdir()
    urdf = tmp_path / "source.urdf"
    urdf.write_text('<robot><link><visual><geometry><mesh filename="../meshes/tip.STL"/></geometry></visual></link></robot>')
    mesh = tmp_path / "meshes/tip.STL"
    mesh.write_bytes(b"old mesh")
    monkeypatch.setattr(builder, "SOURCE", tmp_path)
    monkeypatch.setattr(builder, "SOURCE_URDF", urdf)
    before = builder.source_fingerprint()
    mesh.write_bytes(b"new mesh")
    assert builder.source_fingerprint() != before


def test_rename_preserves_cad_kinematics_and_pinky_chain():
    old = UrdfModel(SOURCE_URDF)
    new = UrdfModel(load_hand_profile("TsenseI_v3_2").path("urdf"))
    assert len(new.joint_names()) == 20
    assert not new.missing_meshes()
    parent = "base_link"
    for part in ("MCP", "swing", "PIP", "DIP"):
        joint = new.joint_by_name[f"left_hand_pinky_{part}_joint"]
        assert joint.parent == parent
        parent = joint.child
    for fraction in (0, .2, .7, 1):
        q = {j.name: j.lower + fraction * (j.upper - j.lower) for j in old.joints if j.type != "fixed"}
        a = old.forward_kinematics(q)
        b = new.forward_kinematics({RENAMES.get(k, k): legacy_to_model("TsenseI_v3_2", k, v)
                                   - thumb_rotaback_origin_offset("TsenseI_v3_2", k)
                                   for k, v in q.items()})
        for link, pose in a.items():
            np.testing.assert_allclose(b[RENAMES.get(link, link)], pose, atol=1e-12)


def test_right_is_exact_equal_angle_mirror():
    profile = load_hand_profile("TsenseI_v3_2")
    left, right = (UrdfModel(profile.path("urdf", side)) for side in ("left", "right"))
    assert not right.missing_meshes()
    mirror = np.diag([-1, 1, 1, 1])
    rng = np.random.default_rng(42)
    for _ in range(8):
        q = {name: rng.uniform(lo, hi) for name, (lo, hi) in left.joint_limits().items()}
        a, b = left.forward_kinematics(q), right.forward_kinematics({right_name(k): v for k, v in q.items()})
        for link, pose in a.items():
            np.testing.assert_allclose(b[right_name(link)], mirror @ pose @ mirror, atol=1e-12)
    # Reflection changes triangle winding as well as vertex positions.
    import trimesh
    for link, visuals in left.visual_meshes_by_link().items():
        for a, b in zip(visuals, right.visual_meshes_by_link()[right_name(link)], strict=True):
            lm = trimesh.load_mesh(a.path, process=False)
            rm = trimesh.load_mesh(b.path, process=False)
            expected = lm.triangles[:, ::-1] @ MIRROR
            np.testing.assert_allclose(rm.triangles, expected, atol=1e-8)


@pytest.mark.parametrize("side", ["left", "right"])
def test_mjcf_matches_urdf_fk_limits_and_contacts(side):
    profile = load_hand_profile("TsenseI_v3_2")
    urdf = UrdfModel(profile.path("urdf", side))
    model = mujoco.MjModel.from_xml_path(str(profile.path("mjcf", side)))
    data = mujoco.MjData(model)
    assert model.nq == model.nu == 20
    assert model.ngeom == 16
    rng = np.random.default_rng(8)
    contact_found = False
    for iteration in range(35):
        q = {name: 0 if iteration == 0 else rng.uniform(lo, hi) for name, (lo, hi) in urdf.joint_limits().items()}
        for name, value in q.items():
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            data.qpos[model.jnt_qposadr[jid]] = value
            np.testing.assert_allclose(model.jnt_range[jid], urdf.joint_limits()[name])
        mujoco.mj_forward(model, data)
        if iteration == 0:
            assert data.ncon == 0, [(data.contact[i].geom.tolist(), data.contact[i].dist) for i in range(data.ncon)]
        contact_found |= data.ncon > 0
        for name, pose in urdf.forward_kinematics(q).items():
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            np.testing.assert_allclose(data.xpos[bid], pose[:3, 3], atol=1e-10)
            np.testing.assert_allclose(data.xmat[bid].reshape(3, 3), pose[:3, :3], atol=1e-10)
    assert contact_found, "Collision masks should detect crossed/folded fingers"


@pytest.mark.parametrize("side", ["left", "right"])
def test_profile_retarget_and_hardware_placeholder(side):
    session = HandModelSession(hand_side=side)
    solver = session.profile.create_retargeter(side)
    assert len(solver.joint_names) == 20
    assert solver._analytic_kinematics is not None
    points = solver.fk.compute()
    assert points.shape == (21, 3)
    pose = session.retarget(points)
    assert set(pose) == set(session.limits)
    assert np.isfinite(list(pose.values())).all()
    filtered = session.set_pose(pose, collision=True)
    assert np.isfinite(list(filtered.values())).all()
    with pytest.raises(RuntimeError, match="hardware adapter"):
        session.send_to_hardware()
    with pytest.raises(ValueError, match="do not belong"):
        session.set_pose({"left_hand_pinky_rota_joint": 0})
    with pytest.raises(ValueError, match="finite"):
        session.set_pose({next(iter(session.limits)): float("nan")})


def test_keypoint_mirroring_and_no_v2_state_mutation():
    profile = load_hand_profile("TsenseI_v3_2")
    from TsenseI_retargeting.fk.fk import RobotKeypointFK
    a, b = [RobotKeypointFK(profile.path("urdf", s), profile.path("keypoints", s)) for s in ("left", "right")]
    q = {n: .12 for n in a.model.joint_names()}
    np.testing.assert_allclose(b.compute({right_name(k): v for k, v in q.items()}),
                               a.compute(q) @ MIRROR, atol=1e-12)
    from config_path import HAND_MODEL_DIR
    from joint_schema import URDF_JOINT_MAP
    before = (HAND_MODEL_DIR, dict(URDF_JOINT_MAP))
    for name in ("TsenseI_v2", "TsenseI_v3_2", "TsenseI_v2"):
        session = HandModelSession(name)
        session.ensure_collision()
        assert len(session.pose) == (21 if name == "TsenseI_v2" else 20)
    assert before == (HAND_MODEL_DIR, URDF_JOINT_MAP)
