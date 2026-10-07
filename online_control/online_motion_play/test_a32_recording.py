"""Recording regressions using synthetic SDK snapshots, without hardware I/O."""
import copy
import io
import json
from pathlib import Path
import sys
from datetime import datetime

import pytest
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "src"))
from a32_recording import A32SessionRecorder, default_channels, export_state_capture, telemetry_event


def state():
    return {"connected": True, "enabled": True, "telemetry_age_ms": 10, "sequence": 1, "rx_ok": 10,
            "tick_ms": 20, "rx_mask": 0x3ffff,
            "self": [{"online": True, "position": i + .25, "target": i + .5,
                      "homing": 254, "iq": .2, "iq_ref": .4, "temperature_centi": 3500}
                     for i in range(18)],
            "mit": [{"online": True, "valid": True, "position": 1.25 + i, "iq": 2.,
                     "velocity_rpm": 3., "voltage": 24., "fault": 0} for i in range(2)]}


def rows(path):
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]


@pytest.mark.parametrize("model", ["TsenseI_v3_2", "TsenseI_v3_2_withid"])
def test_full_command_telemetry_and_action_log(tmp_path, model):
    recorder = A32SessionRecorder(tmp_path, model_id=model)
    path = recorder.start({"serial_port": "synthetic"})
    recorder.record_action("connected")
    target = {"self": list(range(18)), "mit_pos": [70., 80.], "applied_pose": {"left_hand_thumb_rota_block_joint": .1}}
    recorder.record_command(target, control_dt_s=.01)
    current = state()
    recorder.record_state(current)
    recorder.record_state(current)
    recorder.record_command(target, sent=False, error="synthetic write failure")
    current["sequence"] += 1
    recorder.record_state(current)
    recorder.record_error("synthetic write failure", action="tick")
    recorder.stop("disconnected")
    log = rows(path)
    assert log[0]["metadata"]["model_id"] == model
    assert len(log[0]["metadata"]["slot_maps"]["linear"]) == 14
    assert len(log[0]["metadata"]["slot_maps"]["rotary"]) == 6
    assert log[-1]["type"] == "session_end"
    assert [row["seq"] for row in log] == list(range(1, len(log) + 1))
    assert all(a["t"] <= b["t"] for a, b in zip(log, log[1:]))
    assert all(isinstance(row["wall_time_unix_ms"], (int, float)) for row in log)
    assert all(datetime.fromisoformat(row["wall_time"]).utcoffset() is not None for row in log)
    command = next(row for row in log if row["type"] == "h7")
    assert command["control_period_ms"] == 10
    telemetry = [r for r in log if r["type"] == "telemetry"]
    assert len(telemetry) == 2  # polling a cached frame must not add fake RX frames
    writes = [r for r in log if r["type"] == "downlink"]
    assert writes[0]["rotary_deg"] == [6., 7., 14., 15., 70., 80.]
    assert writes[0]["linear_mm"] == [float(i) for i in [0, 1, 2, 3, 4, 5, 8, 9, 10, 11, 12, 13, 16, 17]]
    assert writes[1]["sent"] is False
    assert telemetry[0]["latest_host_command"] == telemetry[1]["latest_host_command"]
    assert telemetry[0]["latest_host_command"]["downlink_seq"] == writes[0]["seq"]
    assert recorder.snapshot()["active"] is False
    assert recorder.snapshot()["error"] is None
    assert recorder.read_download()[1] == path.read_bytes()
    with pytest.raises(ValueError):
        recorder.read_download("../secret.jsonl")


def test_offline_or_stale_motor_has_no_fabricated_curve_value():
    sample = state()
    sample["self"][8]["online"] = False
    sample["mit"][0]["valid"] = False
    sample["self"][0]["temperature_centi"] = 32767
    converted = telemetry_event(sample, default_channels())
    assert converted["linear"][0]["pos_act_mm"] == -.25
    assert converted["linear"][0]["mcu_temp_c"] is None
    assert converted["linear"][6]["position"] == 8.25  # preserve raw cache
    assert converted["linear"][6]["pos_act_mm"] is None
    assert converted["linear"][6]["iq_meas"] is None
    assert converted["rotary"][4]["pos_actual_mm"] is None
    assert converted["rotary"][5]["pos_actual_mm"] == 2.25  # already output degrees
    sample["telemetry_age_ms"] = 600
    assert all(r["pos_act_mm"] is None for r in telemetry_event(sample, default_channels())["linear"])


def test_recording_keeps_zero_ui_swing_and_clipped_command_separate(tmp_path):
    recorder = A32SessionRecorder(tmp_path)
    path = recorder.start()
    name = "left_hand_index_swing_joint"
    recorder.record_command({"applied_pose": {name: .005}}, pose={name: 0.})
    recorder.stop()
    log = rows(path)
    conversion = next(row for row in log if row["type"] == "conversion")
    frame = next(row for row in log if row["type"] == "h7")
    assert conversion["joints_hw"] == frame["joints_hw"] == {name: .005}
    assert frame["joints_ui"] == {name: 0.}


def test_cached_feedback_emits_status_at_stale_boundary(tmp_path):
    recorder = A32SessionRecorder(tmp_path)
    path = recorder.start()
    sample = state()
    sample["telemetry_age_ms"] = 500
    recorder.record_state(sample)
    sample["telemetry_age_ms"] = 501
    recorder.record_state(sample)
    recorder.stop()
    log = rows(path)
    statuses = [row["sdk_state"] for row in log if row["type"] == "sdk_status"]
    assert [item["telemetry_age_ms"] for item in statuses] == [500, 501]
    assert len([row for row in log if row["type"] == "telemetry"]) == 1


def test_event_copies_snapshot_and_nonfinite_values_are_valid_json(tmp_path):
    recorder = A32SessionRecorder(tmp_path)
    path = recorder.start()
    sample = state()
    sample["self"][0]["position"] = float("nan")
    recorder.record_state(sample)
    sample["self"][1]["position"] = 900.
    recorder.stop()
    event = next(r for r in rows(path) if r["type"] == "telemetry")
    assert event["sdk_state"]["self"][0]["position"] is None
    assert event["linear"][1]["pos_act_mm"] == -1.25
    assert event["linear"][1]["position"] == 1.25
    assert event["linear_feedback_sign"] == -1
    assert event["decode_issues"][0]["reason"] == "nonfinite_sdk_value"


def test_read_only_disconnected_capture_preserves_time_without_fake_telemetry(tmp_path):
    sample = {"connected": False, "enabled": False, "self": [], "mit": [], "telemetry_age_ms": -1}
    path = export_state_capture([
        {"type": "diagnostic_session"},
        {"type": "http_snapshot", "elapsed_s": .2, "state": {"device": sample, "pose": {"joint": .5}}},
        {"type": "http_snapshot", "elapsed_s": 45., "state": {"device": sample}},
        {"type": "diagnostic_session_end"},
        {"type": "diagnostic_end", "duration_s": 45.2, "samples": 2}], tmp_path,
        metadata={"started_at": "2026-09-28T16:08:55+08:00"})
    log = rows(path)
    assert [r["type"] for r in log] == ["session", "sdk_status", "sdk_status", "session_end"]
    assert log[0]["started_at"] == "2026-09-28T16:08:55+08:00"
    assert log[0]["metadata"]["read_only_capture"] is True
    assert log[1]["t"] == .2
    assert log[-1]["duration_s"] == 45.
    assert log[1]["wall_time_unix_ms"] == datetime.fromisoformat(log[0]["started_at"]).timestamp() * 1000 + 200
    assert log[1]["wall_time_source"] == "derived_from_session_start"
    assert datetime.fromisoformat(log[-1]["ended_at"]).timestamp() * 1000 == log[-1]["wall_time_unix_ms"]


def test_read_only_capture_keeps_original_wall_time(tmp_path):
    path = export_state_capture([{"t": .2, "at": "2026-09-28T16:08:55.250+08:00",
                                  "state": {"connected": False}}], tmp_path,
                                metadata={"started_at": "2026-09-28T16:08:55+08:00"})
    log = rows(path)
    assert log[1]["t"] == .2
    assert log[1]["wall_time_source"] == "captured"
    assert log[1]["wall_time_unix_ms"] == datetime.fromisoformat("2026-09-28T16:08:55.250+08:00").timestamp() * 1000


def test_default_a32_recording_uses_shared_viewer_parent(tmp_path, monkeypatch):
    monkeypatch.setattr("utils.control_session_recorder.control_log_parent", lambda: tmp_path)
    recorder = A32SessionRecorder()
    path = recorder.start()
    recorder.stop()
    assert path.parent == tmp_path / "A32"


def viewer_handler(path, *, origin="http://127.0.0.1:8767", peer="127.0.0.1", body=None):
    import server
    handler = object.__new__(server.Handler)
    payload = json.dumps(body or {}).encode()
    handler.path = path
    handler.headers = {"Host": "127.0.0.1:8767", "Origin": origin,
                       "Content-Type": "application/json", "Content-Length": str(len(payload))}
    handler.client_address = (peer, 1234)
    handler.allowed = lambda: True
    handler.wfile, handler.rfile = io.BytesIO(), io.BytesIO(payload)
    handler.server = SimpleNamespace()  # Catalog access must not enter the control bridge.
    replies = []
    handler.reply = lambda status, data, *args, **kwargs: replies.append((status, data))
    handler.send_response = lambda status: replies.append((status, "stream"))
    handler.send_header = lambda *args: None
    handler.end_headers = lambda: None
    return handler, replies


def test_motion_viewer_catalog_parent_and_file_routes(tmp_path, monkeypatch):
    from utils import control_log_catalog as catalog
    monkeypatch.setattr(catalog, "PARENT_PATH_FILE", tmp_path / "parent.txt")
    root = tmp_path / "requested-log-root"
    handler, replies = viewer_handler("/api/viewer/log-parent", body={"parent": str(root)})
    handler.do_POST()
    assert replies[0][0] == 200 and catalog.control_log_parent() == root
    recorder = A32SessionRecorder()
    path = recorder.start()
    recorder.record_state(state())
    recorder.stop()
    handler, replies = viewer_handler("/api/viewer/logs?model=A32")
    handler.do_GET()
    assert replies[0][0] == 200
    assert replies[0][1]["files"][0]["name"] == path.name
    assert replies[0][1]["records"][0]["parts"][0]["closed"] is True
    handler, replies = viewer_handler("/api/viewer/log-file?model=A32&name=" + path.name)
    handler.do_GET()
    assert replies == [(200, "stream")]
    assert handler.wfile.getvalue() == path.read_bytes()
    handler, replies = viewer_handler("/api/viewer/log-file?model=A32&name=../secret.jsonl")
    handler.do_GET()
    assert replies[0][0] == 404


@pytest.mark.parametrize("path", ["/api/viewer/logs?model=A32", "/api/viewer/log-file?model=A32&name=hand_control_test.jsonl", "/api/viewer/log-parent"])
@pytest.mark.parametrize("origin,peer", [("http://other.example", "127.0.0.1"), ("http://127.0.0.1:8767", "192.0.2.1")])
def test_motion_viewer_private_logs_require_local_same_origin(path, origin, peer):
    handler, replies = viewer_handler(path, origin=origin, peer=peer, body={"parent": "E:/unwanted"})
    (handler.do_POST if path.endswith("log-parent") else handler.do_GET)()
    assert replies[0][0] == 403


def test_read_only_capture_deduplicates_cache_and_keeps_raw_values(tmp_path):
    sample = state()
    updated = copy.deepcopy(sample)
    updated["sequence"] = 2
    path = export_state_capture([{"t": 0, "device": sample}, {"t": .2, "device": sample},
                                 {"t": .4, "device": updated}], tmp_path)
    log = rows(path)
    assert len([row for row in log if row["type"] == "sdk_status"]) == 3
    assert len([row for row in log if row["type"] == "telemetry"]) == 2
    assert not any(row["type"] in ("downlink", "conversion", "h7") for row in log)
