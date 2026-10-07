"""Bare A3.2 thumb zero is a mesh stop contact and a zero motor command."""
from pathlib import Path
import math
import sys

import numpy as np
import pytest
import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from a32_kinematics import A32StrokeMapper
from hand_profiles import load_hand_profile
from TsenseI_retargeting.fk.urdf import UrdfModel, axis_angle_matrix


@pytest.mark.parametrize("side", ["left", "right"])
def test_rotaback_zero_touches_stop_and_positive_motion_opens_gap(side):
    profile = load_hand_profile("TsenseI_v3_2")
    model = UrdfModel(profile.path("urdf", side))
    joint = model.joint_by_name[f"{side}_hand_thumb_rotaback_joint2"]
    visuals = model.visual_meshes_by_link()
    parent, child = (trimesh.load_mesh(visuals[name][0].path, process=False)
                     for name in (joint.parent, joint.child))
    # Locate the actual CAD vertex on the fixed stop's chamfer. Coordinates
    # identify the illustrated stop, excluding the concentric bearing meshes.
    hint = np.array([.00251821, -.01723753, .0116032])
    if side == "right":
        hint[0] *= -1
    stop = parent.vertices[np.linalg.norm(parent.vertices - hint, axis=1).argmin()]
    assert np.linalg.norm(stop - hint) < 1e-8
    stop = visuals[joint.parent][0].origin @ np.r_[stop * visuals[joint.parent][0].scale, 1]

    def signed_gap(angle):
        moving = joint.origin @ axis_angle_matrix(joint.axis, angle) @ visuals[joint.child][0].origin
        local = (np.linalg.inv(moving) @ stop)[:3] / visuals[joint.child][0].scale
        points, distances, faces = trimesh.proximity.closest_point(child, [local])
        return np.dot(local - points[0], child.face_normals[faces[0]]), distances[0]

    assert signed_gap(0.)[1] < 1e-8  # contact to better than 0.01 micrometre
    assert signed_gap(.001)[0] > 1e-6  # positive travel releases the stop
    assert signed_gap(-.001)[0] < -1e-6  # negative travel penetrates it
    assert (joint.lower, joint.upper) == (0., 1.74)


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("angle", [0., .6, 1.74])
def test_origin_calibration_does_not_offset_motor_commands(side, angle):
    profile = load_hand_profile("TsenseI_v3_2")
    name = f"{side}_hand_thumb_rotaback_joint2"
    pose = profile.init_pose(side)
    pose[name] = angle
    result = A32StrokeMapper(side, model_id=profile.model_id).convert(pose)
    assert result.rotary_positions_deg[name] == pytest.approx(math.degrees(angle), abs=1e-10)
