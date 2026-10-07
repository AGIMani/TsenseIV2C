"""Web adapter regression checks; never connect to physical hardware."""
import json
import math
import time

import pytest

import server


@pytest.fixture
def bridge(tmp_path, monkeypatch):
    monkeypatch.setattr(server, "HERE", tmp_path)
    adapter = server.Bridge(data_dir=tmp_path / "runtime")
    yield adapter
    adapter.close()


def pump_until(predicate, timeout=3, tick=None):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if tick:
            tick()
        if predicate():
            return
        time.sleep(.005)
    raise AssertionError("Control operation did not complete")


def test_all_joint_controls_update_session_and_browser_pose(bridge):
    before = bridge.snapshot()
    assert len(before["joints"]) == 21
    thumb = next(j for j in before["joints"] if j["name"] == "left_hand_thumb_swing_joint")
    assert (thumb["min"], thumb["max"]) == (0.0, 2.0)
    assert all(value == 0.0 for value in bridge.snapshot()["pose"].values())
    for spec in before["joints"]:
        value = round((spec["min"] + spec["max"]) / 2, 3)
        bridge.dispatch("widget", {"id": spec["id"], "value": value, "commit": True})
        assert bridge.snapshot()["pose"][spec["name"]] == pytest.approx(value)
    assert {joint["name"]: joint["value"] for joint in bridge.snapshot()["joints"]} == bridge.snapshot()["pose"]


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), -99, 99])
def test_invalid_joint_values_never_change_pose(bridge, bad):
    spec = bridge.snapshot()["joints"][0]
    with pytest.raises(ValueError):
        bridge.dispatch("widget", {"id": spec["id"], "value": bad})
    assert bridge.snapshot()["pose"][spec["name"]] == 0


def test_right_hand_file_roundtrip_and_legacy_list(bridge):
    payload = {"_hand_side": "right", "姿态一": {
        "left_hand_thumb_swing_joint": -.4,
        "left_hand_pinky_rota_joint": -.3,
        "left_hand_index_swing_joint": .2,
        "_transition_s": .012, "_interval_s": 0,
    }}
    bridge.dispatch("import", {"text": "\ufeff" + json.dumps(payload)})
    assert bridge.snapshot()["side"] == "right"
    assert bridge.v2.motions["姿态一"]["left_hand_thumb_swing_joint"] == .6
    assert bridge.v2.motions["姿态一"]["left_hand_pinky_rota_joint"] == .3
    assert bridge.dispatch("export", {}) == payload
    bridge.dispatch("import", {"text": json.dumps([{"name": "旧动作", "left_hand_index_MCP_joint": .6}])})
    assert list(bridge.v2.motions) == ["旧动作"]
    assert bridge.snapshot()["side"] == "left"


@pytest.mark.parametrize("payload", [[], "bad", {"bad": {"left_hand_index_MCP_joint": math.nan}},
                                      {"bad": {"left_hand_index_MCP_joint": "oops"}}])
def test_bad_import_preserves_existing_working_copy(bridge, payload):
    valid = {"saved": {"left_hand_index_MCP_joint": .1}}
    bridge.dispatch("import", {"text": json.dumps(valid)})
    path = server.HERE / "runtime/motions.json"
    before = path.read_bytes()
    with pytest.raises(ValueError):
        bridge.dispatch("import", {"text": json.dumps(payload)})
    assert path.read_bytes() == before
    assert list(bridge.v2.motions) == ["saved"]


def test_dialog_ids_and_values_survive_repeated_polling(bridge):
    save = bridge.snapshot()["controls"]["savemotion"]
    bridge.dispatch("widget", {"id": save["id"]})
    first = bridge.snapshot()["dialogs"][0]
    for _ in range(10):
        assert bridge.snapshot()["dialogs"][0] == first
    entry = next(i for i in first["items"] if i["kind"] == "text")
    bridge.dispatch("widget", {"id": entry["id"], "value": "动作名称"})
    updated = bridge.snapshot()["dialogs"][0]
    assert next(i for i in updated["items"] if i["kind"] == "text")["value"] == "动作名称"
    # A pending confirmation continues to block unrelated controls.
    joint = bridge.snapshot()["joints"][0]
    with pytest.raises(ValueError, match="确认|对话"):
        bridge.dispatch("widget", {"id": joint["id"], "value": .1})
    cancel = next(i for i in updated["items"] if i["kind"] == "button" and i["text"] == "取消")
    bridge.dispatch("widget", {"id": cancel["id"]})
    assert not bridge.snapshot()["dialogs"]


def test_sequence_playback_reaches_final_pose_and_stop(bridge):
    payload = {"起点": {"left_hand_index_MCP_joint": .1, "_transition_s": .02},
               "终点": {"left_hand_index_MCP_joint": .6, "_transition_s": .03}}
    bridge.dispatch("import", {"text": json.dumps(payload)})
    spec = bridge.snapshot()["controls"]["motionstreamplay"]
    bridge.dispatch("widget", {"id": spec["id"]})
    pump_until(lambda: bridge.snapshot()["playing"], tick=bridge.tick)
    pump_until(lambda: not bridge.snapshot()["playing"], tick=bridge.tick)
    assert bridge.snapshot()["pose"]["left_hand_index_MCP_joint"] == pytest.approx(.6, abs=.002)
    loop = bridge.snapshot()["controls"]["motionStreamLoop"]
    bridge.dispatch("widget", {"id": loop["id"], "value": True})
    bridge.dispatch("widget", {"id": spec["id"]})
    pump_until(lambda: bridge.snapshot()["playing"], tick=bridge.tick)
    bridge.dispatch("stop", {})
    assert not bridge.snapshot()["playing"]
    assert not bridge.snapshot()["controls"]["realtimeJointControl"]["value"]


@pytest.mark.parametrize("interval", [0, .000001])
def test_online_dense_downlink_uses_shared_timeline_and_stream_writes(bridge, interval):
    from usb.offline import UsbController, verify_hand_pos_frame

    class SerialSink:
        is_open = True
        out_waiting = 0

        def __init__(self):
            self.frames = []
            self.flushes = 0

        def write(self, frame):
            self.frames.append(bytes(frame))
            return len(frame)

        def flush(self):
            self.flushes += 1

        def close(self):
            self.is_open = False

    usb = UsbController()
    sink = SerialSink()
    usb._ser, usb._connected = sink, True
    bridge.v2.usb = usb
    payload = {f"frame_{i}": {
        "left_hand_index_MCP_joint": .1,
        "_transition_s": .002, "_interval_s": interval,
    } for i in range(120)}
    bridge.dispatch("import", {"text": json.dumps(payload)})
    controls = bridge.snapshot()["controls"]
    bridge.dispatch("widget", {"id": controls["isConnectOffline"]["id"], "value": True})
    writes = []
    send = usb.send_joint_angles

    def tracked_send(cfg, **kwargs):
        writes.append((kwargs["source"], kwargs["stream"]))
        return send(cfg, **kwargs)

    usb.send_joint_angles = tracked_send
    bridge.dispatch("widget", {"id": controls["motionstreamplay"]["id"]})
    pump_until(lambda: bridge.snapshot()["playing"], tick=bridge.tick)
    pump_until(lambda: not bridge.snapshot()["playing"], timeout=8, tick=bridge.tick)
    snapshot = bridge.snapshot()
    timing = snapshot["playback_timing"]
    assert timing["nominal_s"] == pytest.approx(120 * (.002 + interval))
    assert timing["sent"] > 0 and not timing["stopped"]
    # Only the preserved final convergence may use reliable/flush writes.
    trajectory = [stream for source, stream in writes if "最终态" not in source]
    assert len(trajectory) == timing["sent"] and all(trajectory)
    assert any("最终态" in source for source, _ in writes)
    assert snapshot["controls"]["choosemotion"]["value"] == 119
    assert snapshot["pose"]["left_hand_index_MCP_joint"] == pytest.approx(.1, abs=.002)
    hand_frames = [frame for frame in sink.frames if len(frame) == 96]
    assert hand_frames and all(verify_hand_pos_frame(frame) for frame in hand_frames)
    usb.disconnect()


def test_watchdog_stops_playback_and_rejects_pending_confirmation(bridge):
    bridge.dispatch("control", {}, client="owner")
    save = bridge.snapshot()["controls"]["savemotion"]
    bridge.dispatch("widget", {"id": save["id"]}, client="owner")
    assert bridge.snapshot()["dialogs"]
    bridge.last_seen = time.monotonic() - 6
    bridge.check_lease()
    assert not bridge.snapshot()["dialogs"]
    assert not bridge.snapshot()["connected"]
    assert not bridge.snapshot()["playing"]
    assert bridge.controller is None


def test_only_allowed_same_origin_hosts():
    from types import SimpleNamespace
    handler = object.__new__(server.Handler)
    handler.server = SimpleNamespace(server_port=8765)
    for host, origin, expected in [
        ("127.0.0.1:8765", "http://127.0.0.1:8765", True),
        ("localhost:8765", "http://localhost:8765", True),
        ("evil.example:8765", "http://evil.example:8765", False),
        ("127.0.0.1:8765", "https://evil.example", False),
    ]:
        handler.headers = {"Host": host, "Origin": origin}
        assert handler.allowed() is expected


def test_download_is_a_named_attachment_with_compatible_payload(bridge):
    from io import BytesIO
    from types import SimpleNamespace
    bridge.dispatch("import", {"text": json.dumps({"_hand_side": "right", "pose": {
        "left_hand_thumb_swing_joint": -.25, "_transition_s": .2}})})
    handler = object.__new__(server.Handler)
    handler.server = SimpleNamespace(server_port=8765, bridge=SimpleNamespace(
        call=lambda action, **kwargs: bridge.dispatch(action, {})))
    handler.path = "/api/download"
    handler.headers = {"Host": "127.0.0.1:8765"}
    handler.wfile = BytesIO()
    headers = {}
    statuses = []
    handler.send_response = statuses.append
    handler.send_header = lambda key, value: headers.update({key: value})
    handler.end_headers = lambda: None
    handler.do_GET()
    assert statuses == [200]
    assert headers["Content-Disposition"] == 'attachment; filename="motions_right.json"'
    assert json.loads(handler.wfile.getvalue()) == bridge.dispatch("export", {})


def test_control_queue_serializes_requests_and_runs_tick_without_gui(bridge, monkeypatch):
    import concurrent.futures
    import threading
    identities = []
    ticks = []
    original = bridge.dispatch
    def record(*args, **kwargs):
        identities.append(threading.get_ident())
        return original(*args, **kwargs)
    monkeypatch.setattr(bridge, "dispatch", record)
    monkeypatch.setattr(bridge.v2, "tick", lambda: ticks.append(threading.get_ident()))
    stopped = threading.Event()
    worker = threading.Thread(target=bridge.run, args=(stopped,))
    worker.start()
    try:
        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
            results = list(pool.map(lambda _: bridge.call("state", client="viewer"), range(8)))
        assert all(result["model_id"] == "TsenseI_v2" for result in results)
        assert set(identities) == {worker.ident}
        pump_until(lambda: len(ticks) >= 2)
        assert set(ticks) == {worker.ident}
    finally:
        stopped.set()
        worker.join(timeout=1)
    assert not worker.is_alive()


def test_expired_queued_command_never_changes_pose(bridge):
    import queue
    spec = bridge.snapshot()["joints"][0]
    response = queue.Queue(maxsize=1)
    bridge.execute({"action": "widget", "body": {"id": spec["id"], "value": .1},
                    "client": None, "deadline": time.monotonic() - 1, "response": response})
    ok, error = response.get_nowait()
    assert not ok and "过期" in error
    assert bridge.snapshot()["joints"][0]["value"] == spec["value"]


def test_close_rejects_queued_calls_and_invalidates_browser_transport(bridge):
    import queue
    bridge.dispatch("control", {}, client="owner")
    bridge.dispatch("connect", {"port": "browser"}, client="owner")
    response = queue.Queue(maxsize=1)
    bridge.jobs.put({"response": response})
    bridge.close()
    assert not bridge.serial.is_open and not bridge.v2.connected
    assert bridge.controller is None
    assert response.get_nowait()[0] is False
    with pytest.raises(ValueError, match="关闭"):
        bridge.call("state")
