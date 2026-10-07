"""Quest relay and same-origin HTTP tests; no headset or hardware required."""
from copy import deepcopy
import json
from pathlib import Path
import ssl
import sys
import threading
import urllib.error
import urllib.request

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src"), str(ROOT / "src/tracking")]
from webxr import WEBXR_25_TO_MEDIAPIPE_21, WEBXR_HAND_JOINT_NAMES, WEBXR_PROTOCOL_SCHEMA
from online_control.online_motion_track.webxr_input import WebXRInputRelay
from online_control.online_motion_track.main import Application, make_server


def event(kind="client_hello", sequence=0, stream="quest-a", **fields):
    return {"schema": WEBXR_PROTOCOL_SCHEMA, "type": kind, "sequence": sequence,
            "stream_id": stream, "timestamp_ms": 1_700_000_000_000 + sequence, **fields}


def skeleton(sequence=1, side="left", stream="quest-a"):
    joints = [{"name": name, "valid": True, "position": [10 + i * .001, 1 + i * .002, -2 + i * .003],
               "orientation": [0, 0, 0, 1], "radius": .004}
              for i, name in enumerate(WEBXR_HAND_JOINT_NAMES)]
    return event("hand_skeleton", sequence, stream, joint_set="webxr-hand-25",
                 coordinate_space="webxr_local_floor", client_perf_ms=100 + sequence,
                 xr_predicted_display_time_ms=101 + sequence,
                 hands=[{"handedness": side, "tracked": True, "joints": joints}])


@pytest.fixture
def relay():
    now = [10.0]
    relay = WebXRInputRelay(clock=lambda: now[0])
    token = relay.create("pc", "left")["token"]
    relay.publish(token, event())
    return relay, token, now


def test_native_mapping_metric_axes_raw_preservation_and_no_image_coordinates(relay):
    relay, token, _ = relay
    raw = skeleton()
    raw["normalized"] = [[999, 999, 999]] * 21
    relay.publish(token, raw)
    latest = relay.latest("pc")
    assert latest["tracking"] and latest["connected"]
    assert latest["hand"]["world"][0] == [0, 0, 0]
    for output, source in zip(latest["hand"]["world"], WEBXR_25_TO_MEDIAPIPE_21):
        assert output == pytest.approx([source * .001, source * .002, source * .003])
    assert latest["hand"]["normalized"] == latest["hand"]["world"]
    assert latest["hand"]["normalized_is_image"] is False
    assert latest["source_metadata"]["source_joint_count"] == 25
    assert latest["source_metadata"]["output_coordinate_space"] == "webxr_wrist_relative"
    assert latest["raw_webxr"] == raw
    latest["raw_webxr"]["hands"][0]["joints"][0]["position"][0] = -1
    raw["hands"][0]["joints"][0]["position"][0] = -2
    assert relay.latest("pc")["raw_webxr"]["hands"][0]["joints"][0]["position"][0] == 10


def test_age_uses_server_clock_never_quest_clock_or_polling(relay):
    relay, token, now = relay
    relay.publish(token, skeleton())
    now[0] += .34
    assert relay.latest("pc")["hand"] is not None
    now[0] += .02
    latest = relay.latest("pc")
    assert latest["hand"] is None and not latest["connected"]
    assert latest["age_ms"] == pytest.approx(360)
    assert latest["source_metadata"]["stale"]
    assert latest["raw_webxr"]["hands"][0]["joints"]


def test_reordering_restarts_and_end_do_not_restore_old_skeleton(relay):
    relay, token, _ = relay
    relay.publish(token, skeleton(10))
    for sequence in (10, 9):
        assert relay.publish(token, skeleton(sequence))["reason"] == "stale_sequence"
    relay.publish(token, event("session_ended", 11))
    assert relay.latest("pc")["hand"] is None
    assert relay.publish(token, skeleton(12))["reason"] == "stream_ended"
    relay.publish(token, event("session_started", 0, "quest-b"))
    relay.publish(token, skeleton(1, stream="quest-b"))
    assert relay.latest("pc")["tracking"]
    assert relay.publish(token, event("client_hello", 99))["reason"] == "retired_stream"
    assert relay.publish(token, skeleton(100))["reason"] == "retired_stream"


def test_selected_hand_loss_and_partial_joints_clear_slot(relay):
    relay, token, _ = relay
    relay.publish(token, skeleton())
    relay.publish(token, event("tracking_lost", 2, handedness="right"))
    assert relay.latest("pc")["tracking"]
    relay.publish(token, event("tracking_lost", 3, handedness="left"))
    assert relay.latest("pc")["hand"] is None
    relay.publish(token, skeleton(4, side="right"))
    assert relay.latest("pc")["hand"] is None
    partial = skeleton(5)
    partial["hands"][0]["joints"][5]["valid"] = False
    relay.publish(token, partial)
    assert relay.latest("pc")["hand"] is None


@pytest.mark.parametrize("mutation", [
    lambda m: m.update(sequence=-1),
    lambda m: m.update(sequence=1.5),
    lambda m: m.update(sequence=True),
    lambda m: m.update(stream_id=""),
    lambda m: m.update(schema="unrelated"),
    lambda m: m["hands"][0]["joints"].pop(),
    lambda m: m["hands"][0]["joints"][0].update(position=[float("nan"), 1, 2]),
    lambda m: m["hands"][0]["joints"][0].update(name="palm"),
    lambda m: m["hands"].append(deepcopy(m["hands"][0])),
    lambda m: m.update(extra_metadata=float("inf")),
    lambda m: m.pop("client_perf_ms"),
    lambda m: m.update(client_perf_ms=True),
    lambda m: m.update(client_perf_ms=-1),
])
def test_invalid_packets_do_not_overwrite_latest(relay, mutation):
    relay, token, _ = relay
    relay.publish(token, skeleton())
    before = relay.latest("pc")
    invalid = skeleton(2)
    mutation(invalid)
    with pytest.raises(ValueError):
        relay.publish(token, invalid)
    assert relay.latest("pc") == before


def test_capture_can_advance_while_last_polled_frame_waits_for_solver(relay):
    relay, token, now = relay
    relay.publish(token, skeleton())
    first = relay.latest("pc")
    relay.publish(token, skeleton(2))
    frame = relay.consume("pc", first["revision"])
    assert frame["tracking"]
    assert frame["raw_webxr"]["sequence"] == 1
    assert relay.consume("pc", first["revision"])["hand"] is None
    latest = relay.latest("pc")
    relay.publish(token, skeleton(3))
    now[0] += .36
    assert relay.consume("pc", latest["revision"])["hand"] is None


def test_tracking_loss_invalidates_unconsumed_polled_sample(relay):
    relay, token, _ = relay
    relay.publish(token, skeleton())
    first = relay.latest("pc")
    relay.publish(token, event("tracking_lost", 2))
    assert relay.consume("pc", first["revision"])["hand"] is None


def test_transport_delay_is_counted_without_matching_device_wall_clocks(relay):
    relay, token, now = relay
    relay.publish(token, skeleton(1))  # Establish the fastest-delivery baseline.
    previous = relay.latest("pc")
    now[0] += 1
    relay.publish(token, skeleton(2))  # Captured 1 ms later, delivered 1 s later.
    latest = relay.latest("pc")
    assert latest["hand"] is None
    assert latest["source_metadata"]["transport_delay_ms"] == pytest.approx(999)
    assert latest["source_metadata"]["discard_reason"] == "late_transport"
    assert latest["age_ms"] == pytest.approx(999)
    assert relay.consume("pc", previous["revision"])["hand"] is None
    recovery = skeleton(3)
    recovery["client_perf_ms"] = 1101
    relay.publish(token, recovery)
    assert relay.latest("pc")["tracking"]


def test_client_clock_regression_clears_observation_and_polled_slot(relay):
    relay, token, _ = relay
    relay.publish(token, skeleton(1))
    previous = relay.latest("pc")
    regressed = skeleton(2)
    regressed["client_perf_ms"] = 50
    relay.publish(token, regressed)
    latest = relay.latest("pc")
    assert latest["hand"] is None
    assert latest["source_metadata"]["discard_reason"] == "client_clock_regressed"
    assert relay.consume("pc", previous["revision"])["hand"] is None


def test_transport_age_survives_poll_to_submit_and_new_stream_resets_clock(relay):
    relay, token, now = relay
    relay.publish(token, skeleton(1))
    now[0] += .201
    relay.publish(token, skeleton(2))
    latest = relay.latest("pc")
    assert latest["age_ms"] == pytest.approx(200)
    now[0] += .16
    assert relay.consume("pc", latest["revision"])["hand"] is None
    relay.publish(token, event("client_hello", 0, "quest-b", client_perf_ms=1))
    restarted = skeleton(1, stream="quest-b")
    restarted["client_perf_ms"] = 2
    relay.publish(token, restarted)
    assert relay.latest("pc")["tracking"]


def test_pair_rotation_and_close_revoke_capability(relay):
    relay, token, _ = relay
    next_pair = relay.create("pc", "right")
    with pytest.raises(ValueError, match="失效"):
        relay.publish(token, event())
    relay.close("pc")
    with pytest.raises(ValueError, match="失效"):
        relay.publish(next_pair["token"], event())
    assert not relay.channels and not relay.tokens


class FakeSession:
    model_id = "TsenseI_v3_2"
    input_source = "webxr"
    input_side = "left"
    closed = False
    last_seen = 0.0
    created_at = 0.0

    def process(self, data):
        self.last = data
        return {"tracking": data["hand"] is not None, "hand": data["hand"],
                "source_metadata": data["source_metadata"], "raw_webxr": data["raw_webxr"]}

    def close(self):
        self.closed = True


def test_application_uses_server_sample_and_drops_forged_stale_data(tmp_path):
    app = Application(tmp_path)
    session = FakeSession()
    app.sessions["pc"] = session
    token = app.webxr_create({"session": "pc"})["token"]
    app.webxr.publish(token, event())
    app.webxr.publish(token, skeleton())
    latest = app.webxr_latest({"session": "pc"})
    request = {"session": "pc", "source": "webxr", "webxr_revision": latest["revision"],
               "hand": {"world": "forged"}, "source_metadata": {"age_ms": -1}, "raw_webxr": {"forged": True}}
    assert app.frame(request)["hand"] == latest["hand"]
    assert app.frame(request)["hand"] is None
    assert session.last["camera_size"] == [0, 0]
    assert "forged" not in session.last["raw_webxr"]
    with pytest.raises(ValueError, match="不一致"):
        app.frame({**request, "source": "mediapipe"})
    app.remove("pc")
    assert session.closed
    with pytest.raises(ValueError, match="失效"):
        app.webxr.publish(token, skeleton(2))


def test_api_source_selection_reaches_real_retarget_pipeline(tmp_path):
    # Public mode always selects the preview solver; no SDK or USB is opened.
    app = Application(tmp_path, public=True)
    created = app.create({"input_source": "webxr", "side": "right", "recording": False})
    session_id = created["session"]
    session = app.get(session_id)
    try:
        token = app.webxr_create({"session": session_id})["token"]
        app.webxr.publish(token, event())
        message = skeleton(side="right")
        world = session.pipeline.session.retargeter.fk.compute()
        for source, point in zip(WEBXR_25_TO_MEDIAPIPE_21, world):
            message["hands"][0]["joints"][source]["position"] = (point + [10, 1, -2]).tolist()
        app.webxr.publish(token, message)
        latest = app.webxr_latest({"session": session_id})
        result = app.frame({"session": session_id, "source": "webxr", "webxr_revision": latest["revision"],
                            "sequence": 1, "timestamp_ms": 1, "capture_unix_ms": 1_700_000_000_000})
        assert result["tracking"]
        assert result["source"] == "webxr"
        assert result["depth_source"] == "webxr_metric"
        assert len(result["pose"]) == 20
    finally:
        app.close()


def test_http_routes_same_origin_and_session_close(tmp_path):
    server = make_server(0, tmp_path)
    server.application.sessions["pc"] = FakeSession()
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

    def request(path, data=None, origin=base):
        req = urllib.request.Request(base + path,
            data=None if data is None else json.dumps(data).encode(),
            headers={"Origin": origin, "Content-Type": "application/json"})
        with opener.open(req, timeout=5) as response:
            assert "xr-spatial-tracking=(self)" in response.headers["Permissions-Policy"]
            return json.load(response)

    try:
        assert "webxr" in request("/api/health")["input_sources"]
        pairing = request("/api/webxr/create", {"session": "pc"})
        assert pairing['entry_url'] == ''
        assert '局域网 HTTPS' in pairing['connection_error']
        token = request("/api/webxr/pair", {"code": pairing["pairing_code"]})["token"]
        assert token == pairing["token"]
        for path in ("/q", "/q/"):
            with opener.open(base + path) as response:
                assert b'/quest-capture.js' in response.read()
        with pytest.raises(urllib.error.HTTPError) as failure:
            request("/api/webxr/publish", {"token": token, "message": event()}, "https://unrelated.invalid")
        assert failure.value.code == 403
        assert request("/api/webxr/publish", {"token": token, "message": event()})["accepted"]
        request("/api/webxr/publish", {"token": token, "message": skeleton()})
        latest = request("/api/webxr/latest", {"session": "pc"})
        assert request("/api/frame", {"session": "pc", "source": "webxr", "webxr_revision": latest["revision"]})["tracking"]
        request("/api/stop", {"session": "pc"})
        with pytest.raises(urllib.error.HTTPError) as failure:
            request("/api/webxr/publish", {"token": token, "message": skeleton(2)})
        assert failure.value.code == 400
    finally:
        server.shutdown()
        server.server_close()
        server.application.close()
        thread.join(timeout=5)


def test_tls_options_require_a_matching_pair(tmp_path):
    with pytest.raises(ValueError, match="同时"):
        make_server(0, tmp_path, tls_cert=tmp_path / "missing.crt")


def test_lan_pairing_advertises_https_address_to_loopback_pc(tmp_path):
    from webxr import ensure_webxr_certificate
    cert, key, _ = ensure_webxr_certificate(tmp_path / 'server.crt', tmp_path / 'server.key',
                                          advertise_hosts=('192.168.88.85',))
    server = make_server(0, tmp_path, host='0.0.0.0', tls_cert=cert, tls_key=key,
                         allowed_hosts=('localhost', '192.168.88.85'))
    server.application.sessions['pc'] = FakeSession()
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f'https://127.0.0.1:{server.server_port}'
    context = ssl.create_default_context(cafile=str(tmp_path / 'quest_webxr_ca.crt'))
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}),
                                        urllib.request.HTTPSHandler(context=context))
    try:
        request = urllib.request.Request(base + '/api/webxr/create',
            data=json.dumps({'session': 'pc'}).encode(),
            headers={'Origin': base, 'Content-Type': 'application/json'})
        with opener.open(request, timeout=5) as response:
            pairing = json.load(response)
        assert pairing['entry_url'] == f'https://192.168.88.85:{server.server_port}/q'
        assert 'connection_error' not in pairing
        request = urllib.request.Request(base + '/q', headers={'Host': f'192.168.88.85:{server.server_port}'})
        with opener.open(request, timeout=5) as response:
            assert response.status == 200
    finally:
        server.shutdown()
        server.server_close()
        server.application.close()
        thread.join(timeout=5)


def test_https_serves_quest_page_with_trusted_local_ca(tmp_path, capsys):
    pytest.importorskip("cryptography")
    from webxr import ensure_webxr_certificate
    cert, key, _ = ensure_webxr_certificate(tmp_path / "server.crt", tmp_path / "server.key")
    context = ssl.create_default_context(cafile=str(tmp_path / "quest_webxr_ca.crt"))
    server = make_server(0, tmp_path, host="127.0.0.1", tls_cert=cert, tls_key=key)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}), urllib.request.HTTPSHandler(context=context))
    try:
        # A client without our CA must still reject TLS, without a server traceback.
        untrusted = urllib.request.build_opener(
            urllib.request.ProxyHandler({}),
            urllib.request.HTTPSHandler(context=ssl.create_default_context()))
        with pytest.raises(urllib.error.URLError) as rejected:
            untrusted.open(f"https://127.0.0.1:{server.server_port}/q", timeout=5)
        assert isinstance(rejected.value.reason, ssl.SSLCertVerificationError)
        with opener.open(f"https://127.0.0.1:{server.server_port}/quest.html", timeout=5) as response:
            assert response.status == 200
            assert "xr-spatial-tracking=(self)" in response.headers["Permissions-Policy"]
            assert "Quest 3" in response.read().decode("utf-8")
    finally:
        server.shutdown()
        server.server_close()
        server.application.close()
        thread.join(timeout=5)
    errors = capsys.readouterr().err
    assert "rejected the server certificate" in errors
    assert "quest_webxr_ca.crt" in errors
    assert "Traceback" not in errors


def test_lightweight_bundle_contains_shared_protocol_and_relay():
    from online_control.online_motion_track.package import files
    names = {name for _, name in files()}
    assert "app/src/tracking/webxr.py" in names
    assert "app/src/tracking/cloudxr.py" in names
    assert "app/online_control/online_motion_track/webxr_input.py" in names
    assert "docs/online_webxr_quest3.md" in names
    assert "docs/webxr_quest3_input.md" in names
    assert "app/src/tracking/__init__.py" not in names
