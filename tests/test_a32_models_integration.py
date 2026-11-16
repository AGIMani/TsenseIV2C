"""A3.2 SDK bare and uploaded ID-shell assets, frames, and motor conventions."""
from pathlib import Path
import hashlib
import sys
import xml.etree.ElementTree as ET

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
        assert value == 0.
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


def test_only_shell_profile_is_registered_and_retired_models_fail_clearly():
    from hand_profiles import PROFILE_PATHS
    assert tuple(PROFILE_PATHS) == ("TsenseI_v3_2_withid",)
    profile = load_hand_profile()
    assert profile.mirror_axis == "y" and not profile.right_hand_provisional
    for name in ("TsenseI_v2", "TsenseI_v3_0", "TsenseI_v3_0_withid", "TsenseI_v3_2"):
        with pytest.raises(ValueError, match="not installed"):
            load_hand_profile(name)


@pytest.mark.parametrize("side", ["left", "right"])
def test_id_import_retains_supplied_frames_inertias_and_metre_scale(side):
    profile = load_hand_profile("TsenseI_v3_2_withid")
    model = UrdfModel(profile.path("urdf", side))
    metadata = yaml.safe_load((profile.root / "source_mapping.yaml").read_text())
    aliases = metadata["name_mapping"][side]
    for name, attrs in metadata["source_joint_attributes"][side].items():
        joint = model.root.find("./joint[@name='" + aliases["joints"][name] + "']")
        for tag in ("origin", "axis", "limit"):
            assert joint.find(tag).attrib == attrs[tag]
    assert len(model.root.findall('.//visual')) == 37
    assert sum(float(n.get('value')) for n in model.root.findall('.//mass')) == pytest.approx(1.053011880784252)
    np.testing.assert_array_equal(model.visual_meshes_by_link()["base_link"][0].scale, [1.] * 3)
    for name, digest in metadata["imported_files_sha256"].items():
        assert hashlib.sha256((profile.root / name).read_bytes()).hexdigest() == digest


def test_id_right_frames_are_equal_angle_xz_mirrors():
    profile = load_hand_profile("TsenseI_v3_2_withid")
    left, right = (UrdfModel(profile.path("urdf", side)) for side in ("left", "right"))
    reflection = np.diag([1., -1., 1., 1.])
    offset_reflection = reflection.copy()
    metadata = yaml.safe_load((profile.root / 'hand_profile.yaml').read_text())
    offset_reflection[1, 3] = 2 * metadata.get('mirror_plane_offset_m', 0.)
    rng = np.random.default_rng(39)
    for _ in range(5):
        pose = {name: rng.uniform(lo, hi) for name, (lo, hi) in left.joint_limits().items()}
        a = left.forward_kinematics(pose)
        b = right.forward_kinematics({name.replace("left_hand", "right_hand"): value for name, value in pose.items()})
        for name, transform in a.items():
            reflected_name = name.replace("left_hand", "right_hand").replace("Left_hand", "Right_hand")
            expected = np.eye(4) if name == left.base_link else offset_reflection @ transform @ reflection
            np.testing.assert_allclose(b[reflected_name], expected, atol=1e-12)


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
        metadata = yaml.safe_load((profile.root / 'hand_profile.yaml').read_text())
        offset = np.zeros(3)
        offset['xyz'.index(profile.mirror_axis)] = 2 * metadata.get('mirror_plane_offset_m', 0.)
        np.testing.assert_allclose(right.compute({right_name(name): value for name, value in pose.items()}),
                                   left.compute(pose) @ reflection + offset, atol=1e-12)
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
    # Both models share SDK channels, while their CAD limits are independent.
    bare_limits = dict(zip(bare.joint_names, bare._limits))
    shell_limits = load_hand_profile("TsenseI_v3_2_withid").joint_limits(side)
    assert shell_limits[f'{side}_hand_thumb_rota_block_joint'][1] == pytest.approx(np.deg2rad(130))
    pose = {name: (max(lo, bare_limits[name][0]) + min(hi, bare_limits[name][1])) * .25
            for name, (lo, hi) in shell_limits.items()}
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
    assert clipped["clamped_joints_rad"][f"{side}_hand_index_swing_joint"] == pytest.approx(np.deg2rad(20))
    assert clipped["rotary_positions_deg"][f"{side}_hand_index_swing_joint"] == pytest.approx(20.)


def test_mapper_rejects_wrong_model_identity():
    with pytest.raises(ValueError, match="Unsupported A3.2 model"):
        A32StrokeMapper(model_id="TsenseI_v3_0")


@pytest.mark.parametrize('side', ['left', 'right'])
def test_sampled_collision_gate_keeps_exact_authored_ranges(side):
    from online_control.online_motion_track.pipeline_v3_0.collision import StopCollisionGuard
    profile = load_hand_profile('TsenseI_v3_2_withid')
    gate = StopCollisionGuard(profile, side).guard
    for name, bounds in profile.joint_limits(side).items():
        np.testing.assert_allclose(gate._joint_bounds(name), bounds, atol=1e-14)
        if side == 'left':
            # Historical shared-left simulations can still accept right aliases.
            np.testing.assert_allclose(gate._joint_bounds(name.replace('left_hand_', 'right_hand_')),
                                       bounds, atol=1e-14)
        for endpoint in bounds:
            assert gate._clamp_joint(name, endpoint) == pytest.approx(endpoint, abs=1e-14)
