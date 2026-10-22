from __future__ import annotations

import json
import sys
from datetime import datetime
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = PROJECT_ROOT / "src"
if str(SRC_DIR) not in sys.path:
    sys.path.insert(0, str(SRC_DIR))

from utils.control_session_recorder import ControlSessionRecorder


def test_records_complete_control_chain(tmp_path: Path) -> None:
    recorder = ControlSessionRecorder()
    path = recorder.start(output_dir=tmp_path, metadata={"camera_type": "realsense"})
    recorder.record_mediapipe(
        [(0.0, 0.1, 0.2)] * 21,
        frame_id=7,
        hand_side="left",
        depth_source="realsense",
        timestamp_ms=1234,
    )
    recorder.record_retarget(
        {"left_hand_thumb_rota_joint": 0.25},
        frame_id=7,
        hand_side="left",
        solve_ms=9.58,
        landmark_repairs=[
            {"finger": "mid", "signed_angle_rad": -0.7, "corrected_to_rad": 0.0}
        ],
    )
    recorder.record_h7(
        joints_ui={"left_hand_thumb_rota_joint": 0.2},
        joints_hw={"left_hand_thumb_rota_joint": 0.2},
        frame_id=7,
        hand_side="left",
        sent=True,
        usb_connected=True,
        guard_limited=False,
    )
    assert recorder.stop(reason="test") == path

    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    assert rows[0]["units"]["current"] == "firmware_native"
    assert [row["type"] for row in rows] == [
        "session", "mediapipe", "retarget", "joint_angles", "h7", "joint_angles", "session_end"
    ]
    assert rows[1]["frame_id"] == rows[2]["frame_id"] == rows[3]["frame_id"] == 7
    assert len(rows[1]["landmarks"]) == 21
    assert rows[3]["joints"]["left_hand_thumb_rota_joint"] == 0.25
    assert rows[3]["stage"] == "retarget"
    assert "joints" not in rows[2]
    assert rows[2]["landmark_repairs"][0]["finger"] == "mid"
    assert rows[4]["sent"] is True
    assert rows[5]["sent"] is True and rows[5]["stage"] == "sdk"
    assert rows[-1]["dropped_events"] == 0
    assert [row["seq"] for row in rows] == sorted(row["seq"] for row in rows)


def test_inactive_recorder_is_a_noop(tmp_path: Path) -> None:
    recorder = ControlSessionRecorder()
    recorder.record_retarget(
        {"left_hand_thumb_rota_joint": 1.0},
        frame_id=1,
        hand_side="left",
    )
    assert list(tmp_path.iterdir()) == []


def test_timestamps_are_captured_before_writer_queue_delay(tmp_path, monkeypatch):
    import threading
    import utils.control_session_recorder as module
    recorder = ControlSessionRecorder()
    release = threading.Event()
    write = recorder._writer_main
    def delayed_write(stream, event_queue):
        release.wait(timeout=5)
        write(stream, event_queue)
    monkeypatch.setattr(recorder, "_writer_main", delayed_write)
    clock = [1_790_582_400_123_456_789]
    monkeypatch.setattr(module.time, "time_ns", lambda: clock[0])
    path = recorder.start(output_dir=tmp_path)
    recorder.record_stage("test", {}, frame_id=1, hand_side="left")
    clock[0] += 9_000_000_000
    release.set()
    recorder.stop()
    events = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    sample = next(row for row in events if row["type"] == "test")
    assert sample["wall_time_unix_ms"] == 1_790_582_400_123_456_789 / 1_000_000
    assert all(datetime.fromisoformat(row["wall_time"]).utcoffset() is not None for row in events)
    assert abs(datetime.fromisoformat(sample["wall_time"]).timestamp() * 1000 - sample["wall_time_unix_ms"]) < 1
    assert events[-1]["wall_time_unix_ms"] - sample["wall_time_unix_ms"] == 9000
    assert 0 <= events[-1]["t"] - sample["t"] < 5


def test_size_rotation_retains_all_events_and_session_identity(tmp_path, monkeypatch):
    import utils.control_session_recorder as module
    monkeypatch.setattr(module, "LOG_PART_BYTES", 8192)
    recorder = ControlSessionRecorder()
    recorder.start(output_dir=tmp_path, metadata={"model_id": "TsenseI_v3_0", "hand_side": "left"})
    for index in range(50):
        recorder.record_stage("fixture", {}, frame_id=index, hand_side="left", text="测试" * 100)
    recorder.stop(reason="test_complete")
    files = sorted(tmp_path.glob("*.jsonl"))
    assert len(files) > 1
    assert all(path.stat().st_size <= 8192 for path in files)
    parts = [[json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()] for path in files]
    assert len({rows[0]["session_id"] for rows in parts}) == 1
    assert [rows[0]["recording_part"]["index"] for rows in parts] == list(range(1, len(parts) + 1))
    rows = [row for part in parts for row in part[1:]]
    assert [row["frame_id"] for row in rows if row["type"] == "fixture"] == list(range(50))
    assert rows[-1]["type"] == "session_end"


def test_default_model_directories():
    from utils.control_session_recorder import control_log_directory, LOG_PART_BYTES
    assert LOG_PART_BYTES == 512 * 1024 * 1024
    for model, folder in [("TsenseI_v3_0", "A30"), ("TsenseI_v3_0_withid", "A30"), ("TsenseI_v3_2", "A32"), ("TsenseI_v2", "V2")]:
        directory = control_log_directory({"model_id": model})
        assert directory.name == folder
        if sys.platform == "win32":
            assert directory.parent == Path("E:/TsenseI_hand_logs")


def test_queue_overflow_is_reported_and_stop_drains(tmp_path: Path, monkeypatch) -> None:
    import threading
    import utils.control_session_recorder as module

    monkeypatch.setattr(module, "_QUEUE_CAPACITY", 4)
    recorder = ControlSessionRecorder()
    release_writer = threading.Event()
    writer = recorder._writer_main

    def delayed_writer(stream, event_queue):
        release_writer.wait(timeout=5)
        writer(stream, event_queue)

    monkeypatch.setattr(recorder, "_writer_main", delayed_writer)
    path = recorder.start(output_dir=tmp_path)
    try:
        for i in range(10):
            recorder.record_joint_angles("retarget", {"left_hand_thumb_rota_joint": .1}, frame_id=i, hand_side="left")
    finally:
        release_writer.set()
        recorder.stop()
    rows = [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]
    assert rows[-1]["type"] == "session_end"
    assert rows[-1]["dropped_events"] == 7
    assert len(rows) == 5
    assert recorder.error is None
