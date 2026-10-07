"""Metric observations retain current geometry; camera smoothing stays active."""
from copy import deepcopy
from pathlib import Path
import sys
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src"), str(ROOT / "tools")]
from hand_profiles import A30_MODEL_IDS, A32_MODEL_IDS
from online_control.online_motion_track.main import WebSession, StaleFrameError
from online_control.online_motion_track.a32.solver_process import SolverProcess
from online_control.online_motion_track.calibration import CalibrationSession
from online_control.online_motion_track.pipeline_v3_2.pipeline import prepare_points
from keypoint_mediapipe_input import align_world_landmarks

SOURCES = ("webxr", "wuji_glove", "leapmotion", "mediapipe")
MODELS = (*A30_MODEL_IDS, *A32_MODEL_IDS)


def moving_points(rest, index):
    # Deform fingertips, not a rigid translation eliminated by wrist alignment.
    points = rest.copy()
    points[[4, 8, 12, 16, 20], 2] += index * .004
    return points


def packet(source, side, index, world):
    hand = dict(label=side.title(), world=world.tolist())
    if source == "mediapipe":
        hand["normalized"] = world.tolist()
    return dict(source=source, sequence=index, timestamp_ms=1000 + index * 16,
                capture_unix_ms=1789700000000 + index * 16, hand=hand)


@pytest.mark.parametrize("model", MODELS)
@pytest.mark.parametrize("source", SOURCES)
@pytest.mark.parametrize("side", ("left", "right"))
def test_real_web_pipeline_keeps_each_metric_sample_and_filters_camera(model, source, side):
    session = WebSession(side, recording=False, model_id=model,
                         collision_mode="off", input_source=source)
    observations = []
    original_process = session.pipeline.process
    def observe(value):
        observations.append(value)
        return original_process(value)
    session.pipeline.process = observe
    try:
        rest = session.pipeline.session.retargeter.fk.compute() * .9
        for index in range(4):
            world = moving_points(rest, index)
            result = session.process(packet(source, side, index, world))
            observation = observations[-1]
            assert result["tracking"], result["message"]
            np.testing.assert_array_equal(observation.raw_world_points, world)
            if source != "mediapipe" or index == 0:
                np.testing.assert_array_equal(observation.world_points, world)
                np.testing.assert_allclose(result["points"], prepare_points(world, side), atol=1e-12)
            else:
                assert np.max(np.abs(observation.world_points - world)) > 1e-5
                assert np.max(np.abs(observation.normalized_points - world)) > 1e-5
            if source != "mediapipe":
                assert observation.normalized_points is None
        latest = packet(source, side, 3, world)
        with pytest.raises(StaleFrameError):
            session.process(latest)
        wrong = packet(source, side, 4, world)
        wrong["hand"]["label"] = "Right" if side == "left" else "Left"
        assert not session.process(wrong)["tracking"]
        loss = packet(source, side, 5, world) | {"hand": None}
        assert not session.process(loss)["tracking"]
        recovered = session.process(packet(source, side, 6, world))
        assert recovered["tracking"]
        np.testing.assert_array_equal(observations[-1].world_points, world)
        invalid = packet(source, side, 7, world)
        invalid["hand"]["world"][8][0] = float("nan")
        with pytest.raises(ValueError):
            session.process(invalid)
    finally:
        session.close()


@pytest.mark.parametrize("model", A32_MODEL_IDS)
@pytest.mark.parametrize("source", SOURCES)
def test_separate_solver_process_preserves_metric_motion(model, source):
    baseline = WebSession(recording=False, model_id=model, collision_mode="off", input_source=source)
    try:
        rest = baseline.pipeline.session.retargeter.fk.compute() * .9
    finally:
        baseline.close()
    solver = SolverProcess(side="left", model_id=model, collision_mode="off", input_source=source)
    try:
        for index in range(4):
            world = moving_points(rest, index)
            result = solver.process(packet(source, "left", index, world))
            assert result["tracking"], result["message"]
            expected = prepare_points(world, "left")
            if source != "mediapipe" or index == 0:
                np.testing.assert_allclose(result["points"], expected, atol=1e-12)
            else:
                assert np.max(np.abs(np.asarray(result["points"]) - expected)) > 1e-5
    finally:
        solver.close()


@pytest.mark.parametrize("model", MODELS)
@pytest.mark.parametrize("source", SOURCES)
@pytest.mark.parametrize("side", ("left", "right"))
def test_calibration_keeps_current_metric_geometry_and_wrist_alignment(model, source, side):
    session = CalibrationSession(side, model_id=model, input_source=source)
    try:
        epoch = session.set_input(dict(enabled=True, source=source, side=side))["input_epoch"]
        rest = session.reference * .8 + np.array([.12, -.04, .2])
        for index in range(3):
            world = moving_points(rest, index)
            result = session.process(packet(source, side, index, world) | {"input_epoch": epoch})
            assert result["tracking"]
            expected = align_world_landmarks(world, session.reference, side, side)
            if source != "mediapipe" or index == 0:
                np.testing.assert_allclose(result["input_points"], expected, atol=1e-12)
            else:
                assert np.max(np.abs(np.asarray(result["input_points"]) - expected)) > 1e-5
    finally:
        session.close()


@pytest.mark.parametrize("source", SOURCES)
@pytest.mark.parametrize("side", ("left", "right"))
def test_v2_keeps_metric_points_before_wrist_transform_and_camera_correction(source, side, monkeypatch):
    from online_control.online_motion_track.v2 import session as module
    from test_web_v2 import MemoryUsb
    monkeypatch.setattr(module, "UsbController", MemoryUsb)
    monkeypatch.setattr(module.LightControlPipeline, "start", lambda self: None)
    session = module.V2WebSession(side, recording=False, collision_mode="off", input_source=source)
    frames = []
    session.frame_logger = SimpleNamespace(record_frame=lambda row: frames.append(deepcopy(row)),
                                           close=lambda **kwargs: None)
    try:
        rest = session.solver.fk.compute() * .8
        if side == "right":
            rest[:, 0] *= -1
        session.command("tracking_start", {})
        for index in range(4):
            world = moving_points(rest, index)
            session.process(packet(source, side, index, world))
            frame = frames[-1]
            np.testing.assert_array_equal(frame["raw_world_points_21_m"], world)
            actual = frame["smoothed_world_points_21_m"]
            if source != "mediapipe":
                np.testing.assert_array_equal(actual, world)
            elif index == 0:
                # The unchanged V2 camera filter stores its initial state as float32.
                np.testing.assert_allclose(actual, world, atol=1e-8)
            else:
                assert np.max(np.abs(actual - world)) > 1e-5
            if source != "mediapipe":
                np.testing.assert_array_equal(frame["source_points_21_m"], world - world[0])
                assert frame["raw_normalized_points_21"] is None
    finally:
        session.close()
