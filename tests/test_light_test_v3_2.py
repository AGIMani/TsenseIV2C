"""Exercise the headless V3.2 solver boundary and observation validation."""
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from online_control.online_motion_track.pipeline_v3_2.pipeline import MediaPipeHandPipeline, prepare_points
from TsenseI_retargeting.toporetarget import _canonical_hand_points


def observation(world, side="left", **kwargs):
    data = dict(world_points=world, points=world, normalized_points=np.zeros((21, 3)),
                detected_label=side.title(), expected_label=side.title(),
                held_last_valid=False, used_fallback_hand=False)
    data.update(kwargs)
    return SimpleNamespace(**data)


@pytest.mark.parametrize("side", ["left", "right"])
def test_world_landmarks_drive_calibrated_model_in_camera_coordinates(side):
    pipeline = MediaPipeHandPipeline(side, "off")
    session = pipeline.session
    solver = session.retargeter
    rotation = np.array([[0., -1., 0.], [0., 0., -1.], [1., 0., 0.]])
    # Rotation + translation + human/robot scale must not change hand shape.
    assert np.linalg.det(rotation) == 1
    for fraction in (0., .2, .5):
        q = {n: np.clip(fraction * hi, lo, hi) for n, (lo, hi) in session.limits.items()}
        target = solver.fk.compute(q)
        source = target @ rotation * 1.3 + [.2, -.1, .4]
        frame = observation(source, side, points=np.full((21, 3), np.nan))
        for _ in range(10):
            result = pipeline.process(frame)
        assert result.tracking, result.message
        assert result.depth_source == "mediapipe_world"
        assert len(result.pose) == 20
        assert all(lo <= result.pose[n] <= hi for n, (lo, hi) in session.limits.items())
        actual = solver.fk.compute(result.pose)
        a = _canonical_hand_points(actual - actual[0])
        b = _canonical_hand_points(target - target[0])
        assert np.mean(np.linalg.norm(a-b, axis=1)) < .005
        assert any(abs(v) > .1 for v in result.pose.values()) if fraction else True


def test_loss_wrong_side_invalid_points_hold_pose_without_solving(monkeypatch):
    pipeline = MediaPipeHandPipeline("left", "off")
    points = pipeline.session.retargeter.fk.compute()
    good = pipeline.process(observation(points))
    def forbidden(*args, **kwargs):
        pytest.fail("Blocked observation reached solver")
    monkeypatch.setattr(pipeline.session, "retarget", forbidden)
    for frame in (None, observation(points, held_last_valid=True),
                  observation(points, detected_label="Right"),
                  observation(points, expected_label="Right"),
                  observation(points, used_fallback_hand=True),
                  observation(np.full((21, 3), np.nan)),
                  observation(np.zeros((21, 3)))):
        result = pipeline.process(frame)
        assert not result.tracking
        assert result.pose == good.pose


def test_world_fallback_is_explicit_and_closed_fingertips_are_allowed():
    pipeline = MediaPipeHandPipeline("left", "off")
    points = pipeline.session.retargeter.fk.compute()
    result = pipeline.process(observation(None, points=points))
    assert result.tracking
    assert result.depth_source == "normalized_z_fallback"
    points[4] = points[8]
    assert np.isfinite(prepare_points(points, "left")).all()


@pytest.mark.parametrize("mode", ["monitor", "filter"])
def test_collision_path_uses_v3_model_and_holds_on_solver_failure(mode, monkeypatch):
    pipeline = MediaPipeHandPipeline("right", mode)
    points = pipeline.session.retargeter.fk.compute()
    result = pipeline.process(observation(points, "right"))
    assert result.tracking
    # The legacy capsule overlay is empty with fitted mesh collision envelopes.
    assert not result.capsules
    import mujoco
    assert np.count_nonzero(pipeline.session.collision.model.geom_type == mujoco.mjtGeom.mjGEOM_MESH) == 16
    assert all(n.startswith("right_hand_") for n in result.pose)
    assert pipeline.session.collision.model.nq == 20
    def broken(*args, **kwargs):
        raise RuntimeError("injected solver failure")
    monkeypatch.setattr(pipeline.session, "retarget", broken)
    failed = pipeline.process(observation(points, "right"))
    assert not failed.tracking
    assert failed.pose == result.pose
    assert "injected solver failure" in failed.message



def test_pipeline_import_does_not_load_camera_gui_or_hardware_backends():
    import subprocess
    code = """
import sys
sys.path.insert(0, 'src')
from online_control.online_motion_track.pipeline_v3_2.pipeline import MediaPipeHandPipeline
assert 'online_control.online_motion_track.pipeline_v2.control_pipeline' not in sys.modules
assert 'utils.retarget_runtime' not in sys.modules
assert 'tracking.handoccnet' not in sys.modules
assert 'tracking.wilor' not in sys.modules
assert not {'torch', 'cv2', 'PySide6', 'mediapipe'} & sys.modules.keys()
"""
    subprocess.run([sys.executable, '-c', code], cwd=ROOT, check=True,
                   capture_output=True, text=True)
