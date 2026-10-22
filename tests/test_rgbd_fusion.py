"""Metric depth, occlusion and shape regressions without a camera or Qt."""

import json

import numpy as np
import pytest

from online_control.online_motion_track.rgbd_fusion import (
    BONES, FusionUnavailable, SURFACE_OFFSETS_M, deproject_pixel, fuse_hand,
)


CAMERA = {"fx": 600.0, "fy": 600.0, "ppx": 320.0, "ppy": 240.0,
          "width": 640, "height": 480}


def hand_scene(*, mirror=False, scale=1.0, prior_wrong_z=False):
    truth = np.zeros((21, 3))
    truth[1:5] = [[-.026, -.018, 0], [-.043, -.027, -.004],
                  [-.056, -.038, -.008], [-.068, -.045, -.012]]
    for start, x, y, length in ((5, -.027, -.046, .024), (9, -.007, -.052, .027),
                               (13, .015, -.047, .025), (17, .035, -.036, .020)):
        for joint in range(4):
            truth[start + joint] = [x, y - length * joint, -.006 * joint]
    prior = truth.copy()
    if prior_wrong_z:
        prior[:, 2] *= -1
    truth *= scale
    if mirror:
        truth[:, 0] *= -1
        prior[:, 0] *= -1
    absolute = truth + [0, 0, .55]
    uv = absolute[:, :2] / absolute[:, 2, None] * [CAMERA["fx"], CAMERA["fy"]] + [320, 240]
    normalized = np.column_stack((uv / [640, 480], prior[:, 2]))
    depth = np.zeros((480, 640), dtype=np.float32)
    for i, (u, v) in enumerate(uv):
        x, y = np.rint([u, v]).astype(int)
        depth[y - 4:y + 5, x - 4:x + 5] = absolute[i, 2] - SURFACE_OFFSETS_M[i]
    return normalized, prior, depth, truth


def test_deprojectz_matches_original_metric_formula():
    np.testing.assert_allclose(deproject_pixel([380, 210], .5, CAMERA), [.05, -.025, .5])


def test_depth_corrects_relative_joint_z_not_just_root_translation():
    normalized, prior, depth, truth = hand_scene(prior_wrong_z=True)
    result = fuse_hand(normalized, prior, depth, CAMERA)
    fitted = np.asarray(result["world"])
    initial_error = np.linalg.norm(prior[:, 2] - truth[:, 2])
    assert np.linalg.norm(fitted[:, 2] - truth[:, 2]) < initial_error * .30
    assert fitted[8, 2] < -.010 < prior[8, 2]
    np.testing.assert_array_equal(fitted[0], [0, 0, 0])
    assert result["diagnostics"]["measured_count"] == 21
    assert result["diagnostics"]["relative_z_correction_rms_m"] > .005
    assert json.loads(json.dumps(result, allow_nan=False))["diagnostics"]["method"] == "realsense_depth_constrained_21"


@pytest.mark.parametrize("mirror", [False, True])
@pytest.mark.parametrize("scale", [.8, 1.0, 1.25])
def test_preserves_handedness_and_metric_bone_lengths(mirror, scale):
    normalized, prior, depth, truth = hand_scene(mirror=mirror, scale=scale)
    result = fuse_hand(normalized, prior, depth, CAMERA)
    fitted = np.asarray(result["world"])
    assert np.sign(fitted[5, 0]) == np.sign(truth[5, 0])
    for a, b in BONES:
        assert np.linalg.norm(fitted[b] - fitted[a]) == pytest.approx(np.linalg.norm(truth[b] - truth[a]), abs=.003)
    assert result["diagnostics"]["scale"] == pytest.approx(scale, abs=.05)


def test_rejects_foreground_occlusion_and_background_samples():
    normalized, prior, depth, truth = hand_scene()
    for i, value in ((8, .30), (12, 1.2)):
        x, y = np.rint(normalized[i, :2] * [640, 480]).astype(int)
        depth[y - 4:y + 5, x - 4:x + 5] = value
    result = fuse_hand(normalized, prior, depth, CAMERA)
    for i in (8, 12):
        assert result["diagnostics"]["points"][i]["inferred"]
        assert result["diagnostics"]["points"][i]["reason"] == "occluded_or_inconsistent_depth"
        assert np.asarray(result["world"])[i, 2] == pytest.approx(truth[i, 2], abs=.005)
    assert result["diagnostics"]["measured_count"] == 19


def test_depth_holes_leave_explicitly_inferred_points():
    normalized, prior, depth, _ = hand_scene()
    for i in (3, 4, 7, 8):
        x, y = np.rint(normalized[i, :2] * [640, 480]).astype(int)
        depth[y - 4:y + 5, x - 4:x + 5] = np.nan
    result = fuse_hand(normalized, prior, depth, CAMERA)
    assert result["diagnostics"]["inferred_count"] == 4
    assert result["diagnostics"]["points"][4]["surface_depth_m"] is None
    assert result["diagnostics"]["points"][4]["reason"] == "depth_hole"
    assert np.isfinite(result["world"]).all()


@pytest.mark.parametrize("invalid", [0, np.nan, np.inf, 10.0, -.5])
def test_no_depth_never_silently_returns_mediapipe(invalid):
    normalized, prior, depth, _ = hand_scene()
    depth.fill(invalid)
    with pytest.raises(FusionUnavailable, match="insufficient depth"):
        fuse_hand(normalized, prior, depth, CAMERA)


def test_robust_local_sample_ignores_isolated_bad_pixel():
    normalized, prior, depth, truth = hand_scene()
    x, y = np.rint(normalized[8, :2] * [640, 480]).astype(int)
    depth[y, x] = 1.5
    result = fuse_hand(normalized, prior, depth, CAMERA)
    assert result["diagnostics"]["points"][8]["depth_valid"]
    assert result["world"][8][2] == pytest.approx(truth[8, 2], abs=.004)


def test_crossing_finger_uses_foreground_depth_only_for_front_finger():
    normalized, prior, depth, _ = hand_scene()
    normalized[12, :2] = normalized[8, :2]
    prior[12] = deproject_pixel(normalized[12, :2] * [640, 480], .568, CAMERA) - [0, 0, .55]
    result = fuse_hand(normalized, prior, depth, CAMERA)
    assert result["diagnostics"]["points"][8]["depth_valid"]
    assert result["diagnostics"]["points"][12]["reason"] == "occluded_by_nearer_joint"
    assert result["world"][12][2] > result["world"][8][2] + .01


def test_surface_depth_is_not_copied_as_joint_center():
    normalized, prior, depth, _ = hand_scene()
    result = fuse_hand(normalized, prior, depth, CAMERA)
    # At the wrist, the skin is 8 mm closer than the desired bone center.
    assert result["diagnostics"]["root_depth_m"] == pytest.approx(.55, abs=.002)
    assert result["diagnostics"]["points"][0]["surface_depth_m"] == pytest.approx(.542)


def test_output_is_deterministic():
    normalized, prior, depth, _ = hand_scene(prior_wrong_z=True)
    assert fuse_hand(normalized, prior, depth, CAMERA) == fuse_hand(normalized, prior, depth, CAMERA)


def test_world_and_image_reflection_mismatch_rejected():
    normalized, prior, depth, _ = hand_scene()
    prior[:, 0] *= -1
    with pytest.raises(FusionUnavailable, match="handedness"):
        fuse_hand(normalized, prior, depth, CAMERA)


def test_camera_sdk_deprojection_callback_is_used():
    normalized, prior, depth, _ = hand_scene()
    calls = []

    def sdk(uv, z):
        calls.append((uv, z))
        return deproject_pixel(uv, z, CAMERA)

    result = fuse_hand(normalized, prior, depth, CAMERA, deproject=sdk)
    assert len(calls) == 21
    assert all(z == 1 for _, z in calls)
    assert result["diagnostics"]["solver_evaluations"] <= 45


@pytest.mark.parametrize("mutation,reason", [
    ("size", "dimensions"), ("fx", "dimensions"), ("missing", "intrinsics"),
    ("nan", "finite XYZ"), ("short", "21 finite"), ("degenerate", "degenerate"),
    ("sdk", "invalid rays"),
])
def test_rejects_invalid_inputs(mutation, reason):
    normalized, prior, depth, _ = hand_scene()
    camera = dict(CAMERA)
    deproject = None
    if mutation == "size":
        depth = depth[:100]
    elif mutation == "fx":
        camera["fx"] = 0
    elif mutation == "missing":
        del camera["ppx"]
    elif mutation == "nan":
        prior[8, 2] = np.nan
    elif mutation == "short":
        normalized = normalized[:20]
    elif mutation == "degenerate":
        prior[8] = prior[7]
    elif mutation == "sdk":
        deproject = lambda uv, z: [0, 0, 0]
    with pytest.raises(FusionUnavailable, match=reason):
        fuse_hand(normalized, prior, depth, camera, deproject=deproject)
