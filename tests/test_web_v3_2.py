from pathlib import Path
import json
import sys
import threading
import urllib.error
import urllib.request

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from online_control.online_motion_track.main import Application, WebSession, make_server, model_description


def packet(world, side="left", sequence=0):
    return {"sequence": sequence, "timestamp_ms": 1000 + sequence * 50,
            "capture_unix_ms": 1789600000000 + sequence * 50,
            "camera_size": [640, 480], "hand": {"label": side.title(),
            "world": world.tolist(), "normalized": np.zeros((21, 3)).tolist()}}


@pytest.mark.parametrize("side", ["left", "right"])
def test_browser_points_drive_real_solver_and_recording(side, tmp_path):
    session = WebSession(side, True, tmp_path)
    solver = session.pipeline.session.retargeter
    pose = {n: min(.5, hi) if "_mid_" in n else 0
            for n, (lo, hi) in session.pipeline.session.limits.items()}
    world = solver.fk.compute(pose)
    try:
        for i in range(5):
            result = session.process(packet(world, side, i))
            assert result["tracking"]
            assert len(result["pose"]) == 20
            assert len(result["points"]) == 21
            assert result["actuation"]["model_id"] == "TsenseI_v3_2"
            assert result["actuation"]["method"] == "lut"
            assert result["actuation"]["valid"]
            assert len(result["actuation"]["linear_strokes_mm"]) == 14
            assert len(result["actuation"]["rotary_positions_deg"]) == 6
            assert result["pose"][f"{side}_hand_mid_PIP_joint"] > .1
            assert all(lo <= result["pose"][n] <= hi for n, (lo, hi) in session.pipeline.session.limits.items())
        previous = result["pose"]
        lost = packet(world, side, 5); lost["hand"] = None
        result = session.process(lost)
        assert not result["tracking"] and result["pose"] == previous
        wrong = packet(world, "right" if side == "left" else "left", 6)
        result = session.process(wrong)
        assert not result["tracking"] and result["pose"] == previous
    finally:
        session.close()
    rows = [json.loads(line) for line in session.recorder.path.read_text(encoding="utf-8").splitlines()]
    assert rows[0]["camera_source"] == "browser-webcam"
    assert rows[0]["retarget_config"]["lambda_reg"] == .1
    assert rows[-1]["frame_count"] == 7
    assert rows[1]["raw_world_points_21_m"] == world.tolist()
    assert rows[0]["actuation"]["hardware_enabled"] is False
    assert rows[0]["actuation"]["method"] == "lut"
    assert rows[1]["actuation"]["valid"]
    angles = [row for row in rows if row["type"] == "joint_angles"]
    assert len(angles) == 5 and {row["stage"] for row in angles} == {"retarget"}
    assert [row["frame_id"] for row in angles] == list(range(5))


def test_web_session_rejects_stale_malformed_and_closed_frames():
    session = WebSession(recording=False)
    world = session.pipeline.session.retargeter.fk.compute()
    try:
        good = packet(world)
        session.process(good)
        with pytest.raises(ValueError, match="过期"):
            session.process(good)
        broken = packet(world, sequence=1)
        broken["hand"]["world"][0][0] = float("nan")
        with pytest.raises(ValueError, match="有限"):
            session.process(broken)
        assert session.last_sequence == 0
    finally:
        session.close()
    with pytest.raises(ValueError, match="停止"):
        session.process(packet(world, sequence=2))


@pytest.mark.parametrize("side", ["left", "right"])
@pytest.mark.parametrize("model_id,folder,prefix,count", [
    ('TsenseI_v3_2', 'TsenseI_hand_v3_2', '/assets/', 22),
    ('TsenseI_v3_2_withid', 'TsenseI_hand_v3_2_withid', '/assets-v3-2-withid/', 367),
])
def test_descriptor_has_all_original_visuals_and_joint_names(side, model_id, folder, prefix, count):
    model = model_description(side, model_id)
    assert len(model["links"]) == 22
    assert len(model["joints"]) == 21
    assert sum(j["type"] == "revolute" for j in model["joints"]) == 20
    assert len(model["initial_pose"]) == 20
    assert sum(len(link['visuals']) for link in model['links']) == count
    for link in model["links"]:
        for visual in link["visuals"]:
            path = ROOT / 'assets/hands' / folder / visual["mesh"].removeprefix(prefix)
            assert path.is_file()


def test_http_lifecycle_static_boundary_and_same_origin(tmp_path):
    server = make_server(0, tmp_path)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"
    def request(path, data=None, headers=None):
        payload = json.dumps(data).encode() if data is not None else None
        req = urllib.request.Request(base + path, data=payload, headers=headers or {})
        with urllib.request.urlopen(req, timeout=10) as response:
            return response.read()
    try:
        assert json.loads(request("/api/health"))["ok"]
        assert "手势映射工作台" in request("/").decode()
        assert b"HandLandmarker" in request("/detector.worker.js")
        with pytest.raises(urllib.error.HTTPError) as error:
            request("/api/session", {}, {"Origin": "https://unrelated.invalid"})
        assert error.value.code == 403
        with pytest.raises(urllib.error.HTTPError) as error:
            request("/assets/%2e%2e/%2e%2e/%2e%2e/requirements.txt")
        assert error.value.code == 404
        created = json.loads(request("/api/session", {"side": "right", "recording": False}))
        assert len(created["pose"]) == 20
        request("/api/stop", {"session": created["session"]})
        assert not server.application.sessions
    finally:
        server.shutdown(); server.server_close(); server.application.close()
        thread.join(timeout=5)


def test_public_sessions_are_isolated_bounded_and_never_record(tmp_path):
    app = Application(tmp_path, public=True, max_sessions=3)
    try:
        first = app.create({"recording": True}, "192.0.2.1")
        second = app.create({}, "192.0.2.1")
        assert first["recording"] is False and first["log_file"] is None
        with pytest.raises(ValueError, match="最多"):
            app.create({}, "192.0.2.1")
        third = app.create({"side": "right"}, "192.0.2.2")
        with pytest.raises(ValueError, match="繁忙"):
            app.create({}, "192.0.2.3")
        a, b = app.get(first["session"]), app.get(second["session"])
        assert a.pipeline.session.retargeter is not b.pipeline.session.retargeter
        a.process(packet(a.pipeline.session.retargeter.fk.compute()))
        assert b.last_sequence == -1
        a.created_at -= 901
        app.get(third["session"]).last_seen -= 121
        app.expire()
        assert list(app.sessions) == [second["session"]]
        assert list(app.owners) == [second["session"]]
        assert not list(tmp_path.iterdir())
    finally:
        app.close()


def test_public_http_requires_configured_host_and_same_origin(tmp_path):
    server = make_server(0, tmp_path, public=True, allowed_hosts=["hand.example.com"])
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"
    def request(path, data=None, host="hand.example.com", origin=None):
        headers = {"Host": host, "X-Real-IP": "192.0.2.1"}
        if origin:
            headers["Origin"] = origin
        req = urllib.request.Request(base + path, headers=headers,
                                     data=None if data is None else json.dumps(data).encode())
        with urllib.request.urlopen(req, timeout=10) as response:
            return json.load(response)
    try:
        assert request("/api/health")["recording_allowed"] is False
        for host, origin in [("other.example.com", "https://other.example.com"),
                             ("hand.example.com", None), ("hand.example.com", "https://attacker.invalid")]:
            with pytest.raises(urllib.error.HTTPError) as error:
                request("/api/session", {}, host, origin)
            assert error.value.code == 403
        created = request("/api/session", {"recording": True}, origin="https://hand.example.com")
        assert created["recording"] is False
        assert server.application.owners[created["session"]] == "192.0.2.1"
        request("/api/stop", {"session": created["session"]}, origin="https://hand.example.com")
    finally:
        server.shutdown(); server.server_close(); server.application.close()
        thread.join(timeout=5)
