from pathlib import Path
import csv
import json
import sys
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from online_control.online_motion_track.pipeline_v3_2.skeleton_angles import skeleton_joint_angles, ANGLE_KEYS
from online_control.online_motion_track.pipeline_v3_2.pipeline import MediaPipeHandPipeline
from online_control.online_motion_track.pipeline_v3_2.recording import SessionRecorder


def radial_hand():
    points = np.zeros((21, 3))
    for start, x in zip((1, 5, 9, 13, 17), (-4, -3, -1, 1, 3)):
        base = np.array([x, 5., 0.]) * .01
        direction = base / np.linalg.norm(base)
        for offset in range(4):
            points[start + offset] = base + offset * .02 * direction
    return points


def frame(points, side="left", **overrides):
    fields = dict(world_points=points, raw_world_points=points,
                  points=points, raw_normalized_points=np.zeros((21, 3)),
                  normalized_points=np.zeros((21, 3)), detected_label=side.title(),
                  expected_label=side.title(), held_last_valid=False, used_fallback_hand=False)
    fields.update(overrides)
    return SimpleNamespace(**fields)


def test_geometric_angles_straight_and_right_angle():
    points = radial_hand()
    angles = skeleton_joint_angles(points)
    assert set(angles) == set(ANGLE_KEYS)
    for name, value in angles.items():
        if name != "thumb_CMC_spread_proxy":
            assert value == pytest.approx(0., abs=1e-7)
    points[8] = points[7] + [0, 0, .02]
    assert skeleton_joint_angles(points)["index_DIP"] == pytest.approx(np.pi / 2)


def test_angles_are_invariant_to_camera_pose_scale_and_hand_mirror():
    points = radial_hand()
    points[8] = points[7] + [0, 0, .02]
    expected = skeleton_joint_angles(points)
    for matrix in (np.array([[0, -1, 0], [0, 0, -1], [1, 0, 0]]), np.diag([-1, 1, 1])):
        actual = skeleton_joint_angles(1.5 * points @ matrix + [.7, .2, -.3])
        np.testing.assert_allclose(list(actual.values()), list(expected.values()), atol=1e-7)


def test_unobservable_angles_are_null():
    points = radial_hand()
    points[8] = points[7]
    assert skeleton_joint_angles(points)["index_DIP"] is None
    points[6] = points[5] + [0, 0, .02]
    assert skeleton_joint_angles(points)["index_swing"] is None
    assert skeleton_joint_angles(None) is None
    assert skeleton_joint_angles(np.full((21, 3), np.nan)) is None


def test_log_distinguishes_retarget_collision_output_and_held_frames(tmp_path, monkeypatch):
    pipeline = MediaPipeHandPipeline("left", "filter")
    points = pipeline.session.retargeter.fk.compute()
    raw = {n: .2 for n in pipeline.session.limits}
    monkeypatch.setattr(pipeline.session, "retarget", lambda *args, **kwargs: dict(raw))
    monkeypatch.setattr(pipeline.session.collision, "step_dict", lambda q: {n: v / 2 for n, v in q.items()})
    obs = frame(points)
    result = pipeline.process(obs)
    assert result.tracking
    assert result.retarget_pose == raw
    assert result.pose == {n: .1 for n in raw}
    recorder = SessionRecorder(tmp_path, pipeline, camera_source=0, smoothing=True)
    recorder.record_frame(0, obs, result, capture_timestamp_ns=123, camera_size=[640, 480])
    held = pipeline.process(None)
    recorder.record_frame(1, None, held, capture_timestamp_ns=456, camera_size=[640, 480])
    wrong = frame(points, detected_label="Right", used_fallback_hand=True)
    recorder.record_frame(2, wrong, pipeline.process(wrong), capture_timestamp_ns=789, camera_size=[640, 480])
    recorder.close()
    recorder.close()  # Idempotent cleanup.
    records = [json.loads(line) for line in recorder.path.read_text(encoding="utf-8").splitlines()]
    assert records[0]["calibration_yaml"]
    assert records[0]["retarget_config"]["lambda_reg"] == pipeline.session.retargeter.config.lambda_reg
    assert records[0]["retarget_config"]["urdf_path"] == str(pipeline.session.retargeter.config.urdf_path)
    assert records[-1]["frame_count"] == 3
    good, lost, wrong = records[1:4]
    assert good["retarget_joint_angles_rad"] == raw
    assert good["output_joint_angles_rad"] == result.pose
    assert good["source_points_21_m"] == points.tolist()
    assert good["retarget_joint_angles_deg"][next(iter(raw))] == pytest.approx(np.degrees(.2))
    assert lost["skeleton_joint_angles_rad"] is None
    assert lost["retarget_joint_angles_rad"] is None
    assert lost["output_joint_angles_rad"] == result.pose
    assert not wrong["tracking"] and wrong["retarget_joint_angles_rad"] is None
    assert wrong["skeleton_joint_angles_rad"] is not None
    with recorder.csv_path.open(encoding="utf-8-sig", newline="") as f:
        rows = list(csv.DictReader(f))
    assert [r["frame_id"] for r in rows] == ["0", "1", "2"]
    assert rows[1][f"retarget.{next(iter(raw))}.rad"] == ""
    assert float(rows[0][f"retarget.{next(iter(raw))}.deg"]) == pytest.approx(np.degrees(.2))



def test_recorder_creation_failure_closes_opened_json_stream(tmp_path, monkeypatch):
    pipeline = MediaPipeHandPipeline('left', 'off')
    original_open = Path.open
    streams = []
    def open_with_csv_failure(path, *args, **kwargs):
        if path.suffix == '.csv' and args and args[0] == 'x':
            raise OSError('disk full')
        stream = original_open(path, *args, **kwargs)
        if path.suffix == '.jsonl' and args and args[0] == 'x':
            streams.append(stream)
        return stream
    monkeypatch.setattr(Path, 'open', open_with_csv_failure)
    with pytest.raises(OSError, match='disk full'):
        SessionRecorder(tmp_path, pipeline, camera_source=None, smoothing=False)
    assert len(streams) == 1 and streams[0].closed


def test_recorder_failed_end_record_still_closes_both_files(tmp_path, monkeypatch):
    pipeline = MediaPipeHandPipeline('left', 'off')
    recorder = SessionRecorder(tmp_path, pipeline, camera_source=None, smoothing=False)
    def broken_write(record):
        raise OSError('disk full')
    monkeypatch.setattr(recorder, '_write', broken_write)
    with pytest.raises(OSError, match='disk full'):
        recorder.close(reason='error', error='disk full')
    assert recorder._json.closed and recorder._csv_stream.closed
    recorder.close()  # Cleanup stays idempotent after an I/O failure.
