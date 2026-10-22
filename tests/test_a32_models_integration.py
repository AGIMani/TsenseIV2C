"""A3.2 SDK bare and uploaded ID-shell assets, frames, and motor conventions."""
from pathlib import Path
import hashlib
import sys

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src"), str(ROOT / "tools")]
from a32_kinematics import A32StrokeMapper, model_joint_to_source_sign
from hand_profiles import A32_MODEL_IDS, HandModelSession, load_hand_profile
from TsenseI_retargeting.fk.urdf import UrdfModel, axis_angle_matrix
from a32_kinematics.joint_coordinates import is_finger_swing, requested_limits, reverse_swing, thumb_origin_offset


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
@pytest.mark.parametrize("side", ["left", "right"])
def test_profiles_load_all_assets_without_missing_meshes(model_id, side):
    profile = load_hand_profile(model_id)
    assert len(profile.joint_limits(side)) == 20
    for name, value in profile.init_pose(side).items():
        assert value == (0. if is_finger_swing(name) else .005 if requested_limits(name) else 0.)
    for key in ("urdf", "keypoints", "eval_config", "mjcf"):
        assert profile.path(key, side).is_file()
    model = UrdfModel(profile.path("urdf", side))
    assert not model.missing_meshes()
    assert set(model.joint_names()) == set(A32StrokeMapper(side, model_id=model_id).joint_names)
    settings = yaml.safe_load(profile.path("eval_config", side).read_text(encoding="utf-8"))
    assert set(settings["active_joints"]) == set(model.joint_names())
    points = yaml.safe_load(profile.path("keypoints", side).read_text(encoding="utf-8"))
    assert len(points["keypoints"]) == 21
    assert all(point["link"] in model.links for point in points["keypoints"])


def test_sdk_bare_provenance_retains_existing_calibrated_landmarks():
    profile = load_hand_profile("TsenseI_v3_2")
    manifest = yaml.safe_load((profile.root / "sdk_source_mapping.yaml").read_text(encoding="utf-8"))
    assert manifest["geometry_matches"] and manifest["existing_keypoint_annotations_preserved"]
    for side in ("left", "right"):
        assert hashlib.sha256(profile.path("urdf", side).read_bytes()).hexdigest() == manifest["sides"][side]["installed_urdf_sha256"]
    assert profile.calibration_status()["reviewed_count"] > 0


@pytest.mark.parametrize("side", ["left", "right"])
def test_id_import_retains_every_joint_frame_mesh_and_base_scale(side):
    profile = load_hand_profile("TsenseI_v3_2_withid")
    source = UrdfModel(profile.root / "source_20_outputs/TsenseI_hand_left_v3_2_withid.urdf")
    if side == "right":
        # Source snapshot is intentionally the supplied left CAD; the installed
        # right model is its equal-angle XZ mirror.
        source = UrdfModel(profile.root / "urdf/TsenseI_hand_right_v3_2_withid.urdf")
    target = UrdfModel(profile.path("urdf", side))
    assert len(source.joints) == len(target.joints)
    for a, b in zip(source.joints, target.joints, strict=True):
        assert (a.name, a.parent, a.child, a.type) == (b.name, b.parent, b.child, b.type)
        expected_limits = requested_limits(a.name) or (a.lower, a.upper)
        if expected_limits[0] is None:
            assert (b.lower, b.upper) == expected_limits
        else:
            np.testing.assert_allclose((b.lower, b.upper), expected_limits, atol=1e-14)
        expected_origin = (a.origin if profile.root.joinpath("urdf/TsenseI_hand_left_v3_2_withid.urdf").read_text().find("middle-dip-standard-20-outputs-v1") >= 0
                           else a.origin @ axis_angle_matrix(a.axis, thumb_origin_offset(profile.model_id, a.name)))
        np.testing.assert_allclose(expected_origin, b.origin, atol=1e-14)
        np.testing.assert_array_equal(a.axis, b.axis)
    for link, meshes in source.visual_meshes_by_link().items():
        for a, b in zip(meshes, target.visual_meshes_by_link()[link], strict=True):
            assert hashlib.sha256(a.path.read_bytes()).digest() == hashlib.sha256(b.path.read_bytes()).digest()
            np.testing.assert_array_equal(a.origin, b.origin)
            np.testing.assert_array_equal(a.scale, b.scale)
    np.testing.assert_array_equal(target.visual_meshes_by_link()["base_link"][0].scale,
                                  [1.] * 3 if profile.root.joinpath("source_20_outputs").exists() else [.001] * 3)
    manifest = yaml.safe_load((profile.root / "source_mapping.yaml").read_text(encoding="utf-8"))
    for name, digest in manifest["source_sha256"].items():
        path = profile.root / "source_20_outputs" / Path(name).name
        if name.startswith("meshes/"):
            path = profile.root / "source_20_outputs" / name
        assert hashlib.sha256(path.read_bytes()).hexdigest() == digest


@pytest.mark.parametrize("side", ["left", "right"])
def test_bare_and_id_swing_axes_have_same_direction_after_root_alignment(side):
    bare, shell = (UrdfModel(load_hand_profile(name).path("urdf", side)) for name in A32_MODEL_IDS)
    neutral = {name: reverse_swing(name, 0.) for name in bare.joint_names()}
    a, b = bare.forward_kinematics(neutral), shell.forward_kinematics(neutral)
    rotate_z_90 = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    swings = []
    for name in bare.joint_names():
        x, y = bare.joint_by_name[name], shell.joint_by_name[name]
        source_axis = rotate_z_90 @ a[x.parent][:3, :3] @ x.origin[:3, :3] @ x.axis
        shell_axis = b[y.parent][:3, :3] @ y.origin[:3, :3] @ y.axis
        cosine = np.dot(source_axis, shell_axis) / np.linalg.norm(source_axis) / np.linalg.norm(shell_axis)
        assert cosine > .95
        if is_finger_swing(name):
            assert cosine > .9999
            assert model_joint_to_source_sign("TsenseI_v3_2", name) == 1
            assert model_joint_to_source_sign("TsenseI_v3_2_withid", name) == 1
            swings.append(name)
    assert swings == [f"{side}_hand_{f}_swing_joint" for f in ("index", "mid", "ring", "pinky")]
    # Different frames and link dimensions are deliberate; same names do not
    # justify copying the bare shell's landmarks or its X mirror.
    assert load_hand_profile("TsenseI_v3_2").mirror_axis == "x"
    assert load_hand_profile("TsenseI_v3_2_withid").mirror_axis == "y"
    assert load_hand_profile("TsenseI_v3_2_withid").right_hand_provisional


def test_id_right_frames_are_equal_angle_xz_mirrors():
    profile = load_hand_profile("TsenseI_v3_2_withid")
    left, right = (UrdfModel(profile.path("urdf", side)) for side in ("left", "right"))
    reflection = np.diag([1, -1, 1, 1])
    rng = np.random.default_rng(39)
    for _ in range(5):
        pose = {name: rng.uniform(lo, hi) for name, (lo, hi) in left.joint_limits().items()}
        a = left.forward_kinematics(pose)
        b = right.forward_kinematics({name.replace("left_hand", "right_hand"): value for name, value in pose.items()})
        for name, transform in a.items():
            reflected_name = name.replace("left_hand", "right_hand").replace("Left_hand", "Right_hand")
            np.testing.assert_allclose(b[reflected_name], reflection @ transform @ reflection, atol=1e-12)


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
def test_generated_right_keypoints_are_equal_angle_mirrors(model_id, tmp_path):
    from TsenseI_retargeting.fk.fk import RobotKeypointFK
    from sync_all_right_keypoints import prepare, right_name

    profile = load_hand_profile(model_id)
    installed = {side: profile.path("keypoints", side).read_bytes() for side in ("left", "right")}
    # Stored annotations are independently calibrated per hand. Exercise the
    # explicit mirror operation without overwriting either hand's calibration.
    _, _, mirrored = prepare(profile.root / "hand_profile.yaml")
    generated = tmp_path / "right_keypoints.yaml"
    generated.write_text(yaml.safe_dump(mirrored), encoding="utf-8")
    left = RobotKeypointFK(profile.path("urdf", "left"), profile.path("keypoints", "left"))
    right = RobotKeypointFK(profile.path("urdf", "right"), generated)
    reflection = np.diag([-1, 1, 1] if model_id == "TsenseI_v3_2" else [1, -1, 1])
    rng = np.random.default_rng(39)
    poses = [profile.init_pose("left")]
    poses += [{name: rng.uniform(lo, hi) for name, (lo, hi) in left.model.joint_limits().items()}
              for _ in range(5)]
    for pose in poses:
        np.testing.assert_allclose(right.compute({right_name(name): value for name, value in pose.items()}),
                                   left.compute(pose) @ reflection, atol=1e-12)
    assert installed == {side: profile.path("keypoints", side).read_bytes() for side in ("left", "right")}


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
@pytest.mark.parametrize("side", ["left", "right"])
def test_retargeter_loads_each_hands_own_calibration(model_id, side):
    profile = load_hand_profile(model_id)
    before = {hand: profile.path("keypoints", hand).read_bytes() for hand in ("left", "right")}
    expected = yaml.safe_load(before[side])["keypoints"]
    solver = profile.create_retargeter(side)
    assert solver.keypoint_path == profile.path("keypoints", side)
    actual = {point.id: point.to_dict() for point in solver.fk.keypoints.keypoints}
    assert actual == {point["id"]: point for point in expected}
    assert before == {hand: profile.path("keypoints", hand).read_bytes() for hand in ("left", "right")}


@pytest.mark.parametrize("side", ["left", "right"])
def test_id_mjcf_preserves_urdf_fk_limits_and_has_functional_contacts(side):
    mujoco = pytest.importorskip("mujoco")
    profile = load_hand_profile("TsenseI_v3_2_withid")
    urdf = UrdfModel(profile.path("urdf", side))
    model = mujoco.MjModel.from_xml_path(str(profile.path("mjcf", side)))
    data = mujoco.MjData(model)
    assert model.nq == model.nu == 20 and model.ngeom == 15
    rng = np.random.default_rng(4)
    contacts = False
    for iteration in range(50):
        pose = {name: 0.0 if iteration == 0 else rng.uniform(lo, hi) for name, (lo, hi) in urdf.joint_limits().items()}
        for name, angle in pose.items():
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            data.qpos[model.jnt_qposadr[jid]] = angle
            np.testing.assert_allclose(model.jnt_range[jid], urdf.joint_limits()[name])
        mujoco.mj_forward(model, data)
        if iteration == 0:
            assert data.ncon == 0
        contacts |= data.ncon > 0
        for name, transform in urdf.forward_kinematics(pose).items():
            bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
            np.testing.assert_allclose(data.xpos[bid], transform[:3, 3], atol=1e-10)
            np.testing.assert_allclose(data.xmat[bid].reshape(3, 3), transform[:3, :3], atol=1e-10)
    assert contacts, "Collision masks must detect crossing fingers"


@pytest.mark.parametrize("model_id", A32_MODEL_IDS)
@pytest.mark.parametrize("side", ["left", "right"])
def test_retarget_collision_and_motor_preview_are_finite(model_id, side):
    pytest.importorskip("mujoco")
    session = HandModelSession(model_id, side)
    solver = session.profile.create_retargeter(side)
    pose = session.retarget(solver.fk.compute(), collision=True)
    preview = A32StrokeMapper(side, model_id=model_id).preview(pose)
    assert preview["valid"] and preview["model_id"] == model_id
    assert len(preview["linear_strokes_mm"]) == 14 and len(preview["rotary_positions_deg"]) == 6
    assert np.isfinite(list(pose.values())).all()


@pytest.mark.parametrize("method", ["exact", "lut"])
@pytest.mark.parametrize("side", ["left", "right"])
def test_id_mapper_commands_same_signed_swing_without_second_reversal(method, side):
    bare = A32StrokeMapper(side, method=method)
    shell = A32StrokeMapper(side, method=method, model_id="TsenseI_v3_2_withid")
    assert load_hand_profile(A32_MODEL_IDS[0]).joint_limits(side) == load_hand_profile(A32_MODEL_IDS[1]).joint_limits(side)
    pose = {name: (lo + hi) * .25 for name, (lo, hi) in zip(shell.joint_names, shell._limits)}
    for finger in ("index", "mid", "ring", "pinky"):
        pose[f"{side}_hand_{finger}_swing_joint"] = .2
    expected = bare.convert(pose)
    actual = shell.convert(pose)
    assert actual.joint_angles_rad == pose
    assert actual.linear_strokes_mm == expected.linear_strokes_mm
    assert actual.rotary_positions_deg == expected.rotary_positions_deg
    assert actual.as_dict()["model_id"] == shell.model_id
    assert all(row["source_sign"] == 1 for row in shell.description()["channels"])
    pose[f"{side}_hand_index_swing_joint"] = .9
    clipped = shell.preview(pose)
    assert clipped["clamped_joints_rad"][f"{side}_hand_index_swing_joint"] == .35
    assert clipped["rotary_positions_deg"][f"{side}_hand_index_swing_joint"] == pytest.approx(np.rad2deg(.35))


def test_mapper_rejects_wrong_model_identity():
    with pytest.raises(ValueError, match="Unsupported A3.2 model"):
        A32StrokeMapper(model_id="TsenseI_v3_0")
