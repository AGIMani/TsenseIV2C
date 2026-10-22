"""Quest ADB HTTP routes through the trusted relay and real preview solver."""
from contextlib import contextmanager
from copy import deepcopy
import json
from pathlib import Path
import sys
import threading
import urllib.error
import urllib.request

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src"), str(ROOT / "src/tracking")]
from online_control.online_motion_track.main import make_server
from test_online_webxr_input import FakeSession
from test_quest_adb_input import event, skeleton


class HTTPClient:
    def __init__(self, server):
        self.base = f"http://127.0.0.1:{server.server_port}"
        self.opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def post(self, path, data, *, origin=None):
        request = urllib.request.Request(self.base + path, data=json.dumps(data).encode(),
            headers={"Content-Type": "application/json", "Origin": origin or self.base})
        with self.opener.open(request, timeout=5) as response:
            return json.load(response)

    def error(self, path, data, status=400, *, origin=None):
        with pytest.raises(urllib.error.HTTPError) as failure:
            self.post(path, data, origin=origin)
        assert failure.value.code == status
        with failure.value as response:
            return json.load(response)["error"]


@contextmanager
def running_server(tmp_path, **options):
    server = make_server(0, tmp_path, **options)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    try:
        yield server, HTTPClient(server)
    finally:
        server.shutdown()
        server.server_close()
        server.application.close()
        thread.join(timeout=5)


def publish(client, token, message):
    return client.post("/api/quest-adb/publish", {"token": token, "message": message})


def start_stream(client, token):
    for sequence, kind in enumerate(("client_hello", "session_started", "tracking_restored")):
        assert publish(client, token, event(kind, sequence))["accepted"]


def test_loopback_adb_entry_aliases_pairing_and_capture_lifecycle(tmp_path):
    with running_server(tmp_path) as (server, client):
        server.application.sessions["pc"] = session = FakeSession()
        pairing = client.post("/api/webxr/create", {"session": "pc", "transport": "adb"})
        expected_url = f"http://localhost:{server.server_port}/q/adb"
        assert pairing["entry_url"] == pairing["adb_entry_url"] == expected_url
        assert pairing["entry_path"] == pairing["adb_entry_path"] == "/q/adb"
        assert pairing["capture_path"] == "/q/adb#code=123456"
        assert pairing["transport"] == "adb"
        assert "connection_error" not in pairing
        for path in ("/q/adb", "/q/adb/"):
            with client.opener.open(client.base + path, timeout=5) as response:
                assert response.status == 200
                assert "xr-spatial-tracking=(self)" in response.headers["Permissions-Policy"]
                assert b"/quest-capture.js" in response.read()
        token = client.post("/api/quest-adb/pair", {"code": pairing["pairing_code"]})["token"]
        assert token == pairing["token"]
        start_stream(client, token)
        assert client.post("/api/webxr/latest", {"session": "pc"})["hand"] is None
        message = skeleton(3)
        assert publish(client, token, message)["accepted"]
        latest = client.post("/api/webxr/latest", {"session": "pc"})
        result = client.post("/api/frame", {"session": "pc", "source": "webxr",
                                             "webxr_revision": latest["revision"]})
        assert result["tracking"]
        assert session.last["raw_webxr"] == message
        assert session.last["source_metadata"]["transport"] == "adb"
        assert session.last["camera_size"] == [0, 0]
        assert publish(client, token, event("tracking_lost", 4))["accepted"]
        assert client.post("/api/webxr/latest", {"session": "pc"})["hand"] is None
        assert publish(client, token, event("session_ended", 5))["accepted"]
        assert publish(client, token, skeleton(6))["reason"] == "stream_ended"
        client.post("/api/stop", {"session": "pc"})
        assert "失效" in client.error("/api/quest-adb/publish", {"token": token, "message": skeleton(7)})


def test_public_preview_receives_original_21_point_metadata_and_expires_without_input(tmp_path, monkeypatch):
    with running_server(tmp_path, public=True) as (server, client):
        now = [10.0]
        server.application.webxr.clock = lambda: now[0]
        created = client.post("/api/session", {"input_source": "webxr", "side": "right", "recording": False})
        session_id = created["session"]
        session = server.application.get(session_id)
        pairing = client.post("/api/webxr/create", {"session": session_id, "transport": "adb"})
        assert pairing["entry_url"] == ""
        assert "回环 HTTP" in pairing["connection_error"]
        token = client.post("/api/quest-adb/pair", {"code": pairing["pairing_code"]})["token"]
        assert token == pairing["token"]
        start_stream(client, token)
        message = skeleton(3, side="right")
        world = session.pipeline.session.retargeter.fk.compute()
        message["hands"][0]["landmarks"] = (world + [10, 1, -2]).tolist()
        observations = []
        process = session.pipeline.process

        def inspect_observation(observation):
            observations.append(deepcopy(observation))
            return process(observation)

        monkeypatch.setattr(session.pipeline, "process", inspect_observation)
        assert publish(client, token, message)["accepted"]
        latest = client.post("/api/webxr/latest", {"session": session_id})
        assert latest["raw_webxr"] == message
        assert latest["source_metadata"]["joint_mapping"] == list(range(21))
        request = {"session": session_id, "source": "webxr", "webxr_revision": latest["revision"],
                   "sequence": 1, "timestamp_ms": 1, "capture_unix_ms": 1_700_000_000_000,
                   "hand": {"world": "forged"}, "source_metadata": {"forged": True},
                   "raw_webxr": {"forged": True}}
        result = client.post("/api/frame", request)
        assert result["tracking"] and result["source"] == "webxr"
        assert result["depth_source"] == "webxr_metric"
        assert len(result["pose"]) == 20
        assert result["source_metadata"]["transport"] == "adb"
        assert result["source_metadata"]["source_joint_set"] == "mediapipe-21"
        assert result["source_metadata"]["source_joint_count"] == 21
        assert observations[0].raw_webxr == message
        assert observations[0].source_metadata == latest["source_metadata"]
        assert observations[0].raw_normalized_points is None
        assert observations[0].world_points.shape == (21, 3)
        assert observations[0].world_points[0] == pytest.approx([0, 0, 0])
        now[0] += .36
        expired = client.post("/api/webxr/latest", {"session": session_id})
        assert expired["hand"] is None and not expired["connected"]
        assert expired["source_metadata"]["stale"]
        assert expired["age_ms"] == pytest.approx(360)
        lost = client.post("/api/frame", {**request, "sequence": 2, "timestamp_ms": 2})
        assert not lost["tracking"]
        assert observations[-1] is None


def test_adb_http_rejects_cross_origin_invalid_schema_and_revoked_tokens(tmp_path):
    with running_server(tmp_path) as (server, client):
        server.application.sessions["pc"] = FakeSession()
        token = client.post("/api/webxr/create", {"session": "pc", "transport": "adb"})["token"]
        for path, data in (
                ("/api/quest-adb/pair", {"code": "123456"}),
                ("/api/quest-adb/publish", {"token": token, "message": event()})):
            assert "同源" in client.error(path, data, 403, origin="https://unrelated.invalid")
        start_stream(client, token)
        publish(client, token, skeleton(3))
        previous = server.application.webxr.channels["pc"].revision
        invalid = skeleton(4)
        invalid["schema"] = "teleop.webxr.hand_skeleton.v1"
        assert "schema" in client.error("/api/quest-adb/publish", {"token": token, "message": invalid})
        assert "schema" in client.error("/api/webxr/publish", {"token": token, "message": skeleton(4)})
        invalid = skeleton(4)
        invalid["hands"][0]["landmarks"].pop()
        assert "21" in client.error("/api/quest-adb/publish", {"token": token, "message": invalid})
        assert "失效" in client.error("/api/quest-adb/publish", {"token": "invalid-token", "message": event()})
        assert server.application.webxr.channels["pc"].revision == previous
        client.post("/api/webxr/create", {"session": "pc", "transport": "adb"})
        assert "失效" in client.error("/api/quest-adb/publish", {"token": token, "message": skeleton(4)})


@pytest.mark.parametrize("full", [False, True])
def test_bundles_include_native_21_point_parser(full):
    from online_control.online_motion_track.package import files
    names = {name for _, name in files(full=full)}
    assert "app/online_control/online_motion_track/quest_adb_input.py" in names
