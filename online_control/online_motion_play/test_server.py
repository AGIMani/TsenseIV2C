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
    assert len(before["joints"]) == 20
    assert before["model_id"] == "TsenseI_v3_2_withid"
    for spec in before["joints"]:
        value = (spec["min"] + spec["max"]) / 2
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
    side = bridge.snapshot()["controls"]["handSide"]
    bridge.dispatch("widget", {"id": side["id"], "value": 1})
    pose = dict(bridge.a32.pose)
    payload = bridge.a32.payload({"姿态一": {"pose": pose, "transition": .012, "interval": 0}})
    bridge.dispatch("import", {"text": "\ufeff" + json.dumps(payload)})
    assert bridge.snapshot()["side"] == "right"
    assert bridge.a32.motions["姿态一"]["pose"] == pose
    assert bridge.dispatch("export", {}) == payload
    with pytest.raises(ValueError):
        bridge.dispatch("import", {"text": json.dumps([{"name": "旧动作"}])})
    assert bridge.dispatch("export", {}) == payload


@pytest.mark.parametrize("bad", [float("nan"), float("inf"), "oops", None])
def test_bad_import_preserves_existing_working_copy(bridge, bad):
    pose = dict(bridge.a32.pose)
    valid = bridge.a32.payload({"saved": {"pose": pose, "transition": .1, "interval": 0}})
    bridge.dispatch("import", {"text": json.dumps(valid)})
    before = bridge.a32.path.read_bytes()
    invalid = bridge.a32.payload({"bad": {"pose": {**pose, next(iter(pose)): bad}, "transition": .1, "interval": 0}})
    with pytest.raises(ValueError):
        bridge.dispatch("import", {"text": json.dumps(invalid)})
    assert bridge.a32.path.read_bytes() == before
    assert list(bridge.a32.motions) == ["saved"]


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
    first = dict(bridge.a32.pose)
    last = {**first, "left_hand_index_MCP_joint": .6}
    payload = bridge.a32.payload({
        "起点": {"pose": first, "transition": .02, "interval": 0},
        "终点": {"pose": last, "transition": .03, "interval": 0},
    })
    bridge.dispatch("import", {"text": json.dumps(payload)})
    spec = bridge.snapshot()["controls"]["motionstreamplay"]
    bridge.dispatch("widget", {"id": spec["id"]})
    pump_until(lambda: not bridge.snapshot()["playing"], timeout=5)
    assert bridge.snapshot()["pose"]["left_hand_index_MCP_joint"] == pytest.approx(.6, abs=.002)
    loop = bridge.snapshot()["controls"]["motionStreamLoop"]
    bridge.dispatch("widget", {"id": loop["id"], "value": True})
    bridge.dispatch("widget", {"id": spec["id"]})
    pump_until(lambda: bridge.snapshot()["playing"])
    bridge.dispatch("stop", {})
    assert not bridge.snapshot()["playing"]




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
    side = bridge.snapshot()["controls"]["handSide"]
    bridge.dispatch("widget", {"id": side["id"], "value": 1})
    payload = bridge.a32.payload({"pose": {"pose": dict(bridge.a32.pose), "transition": .2, "interval": 0}})
    bridge.dispatch("import", {"text": json.dumps(payload)})
    handler = object.__new__(server.Handler)
    handler.server = SimpleNamespace(server_port=8765, bridge=SimpleNamespace(
        call=lambda action, **kwargs: bridge.dispatch(action, {})))
    handler.path = "/api/download"
    handler.headers = {"Host": "127.0.0.1:8765"}
    handler.wfile = BytesIO()
    headers, statuses = {}, []
    handler.send_response = statuses.append
    handler.send_header = lambda key, value: headers.update({key: value})
    handler.end_headers = lambda: None
    handler.do_GET()
    assert statuses == [200]
    assert headers["Content-Disposition"] == 'attachment; filename="motions_TsenseI_v3_2_withid_right.json"'
    assert json.loads(handler.wfile.getvalue()) == payload


def test_control_queue_serializes_requests_and_runs_tick_without_gui(bridge, monkeypatch):
    import concurrent.futures
    import threading
    identities, ticks = [], []
    original_dispatch, original_tick = bridge.dispatch, bridge.tick
    def record(*args, **kwargs):
        identities.append(threading.get_ident())
        return original_dispatch(*args, **kwargs)
    def record_tick():
        ticks.append(threading.get_ident())
        original_tick()
    monkeypatch.setattr(bridge, "dispatch", record)
    monkeypatch.setattr(bridge, "tick", record_tick)
    stopped = threading.Event()
    worker = threading.Thread(target=bridge.run, args=(stopped,))
    worker.start()
    try:
        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
            results = list(pool.map(lambda _: bridge.call("state", client="viewer"), range(8)))
        assert all(result["model_id"] == "TsenseI_v3_2_withid" for result in results)
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


def test_close_rejects_queued_calls_and_releases_owner(bridge):
    import queue
    bridge.dispatch("control", {}, client="owner")
    response = queue.Queue(maxsize=1)
    bridge.jobs.put({"response": response})
    bridge.close()
    assert not bridge.a32.connected
    assert bridge.controller is None
    assert response.get_nowait()[0] is False
    with pytest.raises(ValueError, match="关闭"):
        bridge.call("state")
