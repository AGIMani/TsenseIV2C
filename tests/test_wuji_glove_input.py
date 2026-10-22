"""Wuji source validation, freshness, real retargeting and HTTP bridge tests."""
from copy import deepcopy
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src"), str(ROOT / "src/tracking")]
from online_control.online_motion_track.wuji_input import SCHEMA, WujiInputRelay, validate_frame
from online_control.online_motion_track.main import Application, Handler, WebSession
sys.path.insert(0, str(ROOT / "tools"))
from wuji_glove_bridge import SkeletonStore, make_server


def sample(side="left", sequence=1, stream="connection-a"):
    return {"schema": SCHEMA, "state": "receiving", "connected": True, "age_ms": 5,
            "frame": {"schema": SCHEMA, "side": side, "sequence": sequence, "stream_id": stream,
                      "frame_id": "l_wrist" if side == "left" else "r_wrist", "units": "m",
                      "timestamp_us": 123, "serial_number": "test",
                      "positions_xyz": [[i * .001, i * .002, i * .003] for i in range(21)],
                      "confidences": [1.] * 21}}


@pytest.mark.parametrize("stage", ["headers", "body"])
@pytest.mark.parametrize("error", [BrokenPipeError, ConnectionResetError, ConnectionAbortedError])
def test_cancelled_json_response_closes_connection(stage, error):
    statuses = []

    def disconnected(*args):
        raise error("preview closed")

    handler = Handler.__new__(Handler)
    handler.close_connection = False
    handler.send_response = statuses.append
    handler.send_header = lambda *args: None
    handler.end_headers = disconnected if stage == "headers" else lambda: None
    handler.wfile = SimpleNamespace(write=disconnected)
    handler.send_json({"update": None})
    assert handler.close_connection
    assert statuses == [200]


def test_json_response_does_not_hide_serialization_failure():
    handler = Handler.__new__(Handler)
    with pytest.raises(TypeError):
        handler.send_json({"unsupported": object()})


@pytest.fixture
def relay():
    now, wire = [10.], [sample()]
    relay = WujiInputRelay(clock=lambda: now[0], fetch=lambda side: deepcopy(wire[0]))
    relay.create("pc", "left")
    return relay, now, wire


@pytest.mark.parametrize("change", [
    {"side": "right"}, {"frame_id": "waist"}, {"units": "mm"}, {"sequence": True},
    {"stream_id": ""}, {"positions_xyz": [[0, 0, 0]] * 25},
    {"positions_xyz": [[float("nan"), 0, 0]] * 21}, {"confidences": [-1] * 21},
])
def test_reject_invalid_frames(change):
    raw = sample()["frame"] | change
    with pytest.raises(ValueError):
        validate_frame(raw, "left")


@pytest.mark.parametrize("side", ["left", "right"])
def test_thumb_second_point_is_midpoint_independent_of_its_measured_position(side):
    raw = sample(side)["frame"]
    raw["positions_xyz"][0] = [.1, -.2, .3]
    raw["positions_xyz"][1] = [.12, -.19, .34]
    raw["positions_xyz"][3] = [.16, -.17, .38]
    expected = np.array(raw["positions_xyz"]) - raw["positions_xyz"][0]
    expected[2] = (expected[1] + expected[3]) * .5
    for outlier in ([.8, -.9, .7], [-.7, .8, -.6], raw["positions_xyz"][1]):
        raw["positions_xyz"][2] = list(outlier)
        before = deepcopy(raw)
        hand = validate_frame(raw, side)
        np.testing.assert_allclose(hand["world"], expected, atol=1e-15)
        assert len(hand["world"]) == 21 and hand["label"] == side.title()
        assert raw == before


def test_relay_carries_corrected_points_and_unmodified_raw_measurements(relay):
    r, _, wire = relay
    wire[0]["frame"]["positions_xyz"][2] = [.5, -.4, .3]
    raw = deepcopy(wire[0]["frame"])
    latest = r.latest("pc")
    consumed = r.consume("pc", latest["revision"])
    for snapshot in (latest, consumed):
        world = np.array(snapshot["hand"]["world"])
        np.testing.assert_allclose(world[2], (world[1] + world[3]) * .5)
        assert snapshot["raw_wuji"] == raw
        assert snapshot["source_metadata"]["raw_wuji"] == raw


def test_age_deduplication_delayed_consumption_and_reconnect(relay):
    r, now, wire = relay
    first = r.latest("pc")
    assert first["hand"]["world"][0] == [0, 0, 0]
    assert "normalized" not in first["hand"]
    assert r.consume("pc", first["revision"])["hand"] is not None
    assert r.consume("pc", first["revision"])["hand"] is None
    now[0] += .4
    assert r.latest("pc")["hand"] is None  # same frame cannot be refreshed by polling
    wire[0] = sample(sequence=0, stream="connection-b")
    second = r.latest("pc")
    assert second["revision"] > first["revision"] and second["hand"] is not None
    now[0] += .36
    assert r.consume("pc", second["revision"])["hand"] is None
    wire[0] = sample(sequence=100, stream="connection-a")
    assert r.latest("pc")["hand"] is None  # retired connection replay


def test_request_duration_old_revision_and_transport_loss(relay):
    r, now, wire = relay
    first = r.latest("pc")
    wire[0] = sample(sequence=2)
    r.latest("pc")
    assert r.consume("pc", first["revision"])["hand"] is not None
    def slow(_):
        now[0] += .4
        return sample(sequence=3)
    r.fetch = slow
    assert r.latest("pc")["hand"] is None
    r.fetch = lambda _: (_ for _ in ()).throw(TimeoutError("offline"))
    loss = r.latest("pc")
    assert loss["hand"] is None and loss["source_metadata"]["event"] == "transport_error"
    assert r.consume("pc", 999)["hand"] is None
    r.close("pc")
    with pytest.raises(ValueError):
        r.latest("pc")


def test_http_bridge_sdk_callback_contract_and_disconnect():
    store = SkeletonStore()
    raw = sample()["frame"]
    skeleton = SimpleNamespace(header=SimpleNamespace(seq=1, timestamp_us=123, frame_id="l_wrist"),
        joints=[SimpleNamespace(pose=SimpleNamespace(position=p), confidence=1.) for p in raw["positions_xyz"]])
    store.offer(skeleton, side="left", serial="test", stream_id="test")
    server = make_server(store, "127.0.0.1", 0)
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    try:
        r = WujiInputRelay(f"http://127.0.0.1:{server.server_port}")
        r.create("pc", "left")
        deadline = time.monotonic() + 2
        while r.latest("pc")["hand"] is None and time.monotonic() < deadline:
            time.sleep(.005)
        assert r.latest("pc")["hand"] is not None
        store.clear("disconnected", "unplugged")
        deadline = time.monotonic() + 2
        while r.latest("pc")["hand"] is not None and time.monotonic() < deadline:
            time.sleep(.005)
        assert r.latest("pc")["hand"] is None
        assert r.consume("pc", 1)["hand"] is None
    finally:
        r.close("pc")
        server.shutdown()
        server.server_close()
        worker.join()


@pytest.mark.parametrize("model", ["TsenseI_v3_0", "TsenseI_v3_0_withid", "TsenseI_v3_2", "TsenseI_v3_2_withid"])
@pytest.mark.parametrize("side", ["left", "right"])
def test_real_solver_metric_geometry_and_recording(model, side, tmp_path):
    session = WebSession(side, True, tmp_path, model_id=model, input_source="wuji_glove")
    world = session.pipeline.session.retargeter.fk.compute() * .8
    packet = {"sequence": 1, "timestamp_ms": 1000., "capture_unix_ms": 1_700_000_000_000,
              "source": "wuji_glove", "source_metadata": {"raw_wuji": {"sequence": 1}},
              "hand": {"label": side.title(), "world": world.tolist()}}
    try:
        result = session.process(packet)
        assert result["tracking"], result["message"]
        assert result["source"] == "wuji_glove" and result["depth_source"] == "wuji_glove_metric"
        np.testing.assert_allclose(np.linalg.norm(np.diff(result["points"], axis=0), axis=1),
                                   np.linalg.norm(np.diff(world, axis=0), axis=1), atol=1e-8)
        lost = session.process(packet | {"sequence": 2, "timestamp_ms": 1100., "hand": None})
        assert not lost["tracking"] and lost["retarget_joint_angles_rad"] is None
    finally:
        session.close()
    import json
    rows = [json.loads(line) for line in session.recorder.path.read_text(encoding="utf-8").splitlines()]
    assert rows[0]["camera_source"] == "wuji-glove"
    assert rows[1]["raw_wuji"] == {"sequence": 1}
    assert rows[1]["raw_normalized_points_21"] is None


def test_application_uses_server_observation_not_browser_forgery():
    app = Application()
    try:
        created = app.create({"model_id": "TsenseI_v3_2", "side": "left", "input_source": "wuji_glove", "recording": False})
        token = created["session"]
        app.wuji_create({"session": token})
        session = app.get(token)
        wire = sample()
        wire["frame"]["positions_xyz"] = session.pipeline.session.retargeter.fk.compute().tolist()
        app.wuji.fetch = lambda _: wire
        snap = app.wuji_latest({"session": token})
        request = {"session": token, "sequence": 0, "timestamp_ms": 1000., "capture_unix_ms": 1_700_000_000_000,
                   "source": "wuji_glove", "wuji_revision": snap["revision"],
                   "hand": {"label": "Right", "world": [[999, 999, 999]] * 21}}
        result = app.frame(request)
        assert result["tracking"]
        pts = np.array(result["points"])
        np.testing.assert_allclose(pts[2], (pts[1] + pts[3]) * .5, atol=1e-12)
        assert not app.frame(request | {"sequence": 1, "timestamp_ms": 1010.})["tracking"]
        app.remove(token)
        assert token not in app.wuji.channels
    finally:
        app.close()


def test_combined_wuji_request_deduplicates_and_never_uses_browser_skeleton():
    app = Application()
    try:
        created = app.create({"model_id": "TsenseI_v3_2_withid", "side": "left",
                              "input_source": "wuji_glove", "recording": False})
        token = created['session']
        app.wuji_create({'session': token})
        session = app.get(token)
        wire = sample()
        wire['frame']['positions_xyz'] = session.pipeline.session.retargeter.fk.compute().tolist()
        app.wuji.fetch = lambda side: wire
        request = dict(session=token, sequence=0, timestamp_ms=1000., capture_unix_ms=1700000000000,
                       hand={'label':'Right','world':[[999,999,999]]*21})
        first = app.wuji_frame(request)
        assert first['update']['tracking'] and first['update']['input_path']=='server_wuji_frame'
        target_at = session.control.target_at
        assert app.wuji_frame(request | dict(sequence=1,timestamp_ms=1010.))['update'] is None
        assert session.control.target_at == target_at
        wire['age_ms'] = 400
        lost = app.wuji_frame(request | dict(sequence=2,timestamp_ms=1400.))
        assert not lost['update']['tracking'] and lost['hand'] is None
        wire['age_ms'] = 5
        wire['frame']['sequence'] = 2
        assert app.wuji_frame(request | dict(sequence=3,timestamp_ms=1450.))['update']['tracking']
        app.remove(token)
        with pytest.raises(ValueError):
            app.wuji_frame(request)
    finally:
        app.close()


def test_combined_wuji_request_rejects_non_wuji_session():
    app = Application()
    try:
        token = app.create({'model_id':'TsenseI_v3_2','recording':False})['session']
        with pytest.raises(ValueError, match='Wuji'):
            app.wuji_frame({'session':token})
    finally:
        app.close()


@pytest.mark.parametrize("side", ["left", "right"])
def test_v2_glove_preserves_metric_bones_and_records_source(side, tmp_path, monkeypatch):
    from online_control.online_motion_track.v2 import session as module
    from test_web_v2 import MemoryUsb
    monkeypatch.setattr(module, "UsbController", MemoryUsb)
    # This test checks metric geometry/recording. A real 500 Hz watchdog is
    # nondeterministic during cold solver initialization in the test process.
    monkeypatch.setattr(module.LightControlPipeline, "start", lambda self: None)
    def camera_preprocess(*_):
        pytest.fail("Glove metric bones must bypass camera correction")
    monkeypatch.setattr(module, "preprocess_wuji_world_points", camera_preprocess)
    session = module.V2WebSession(side, True, tmp_path, collision_mode="off", input_source="wuji_glove")
    world = session.solver.fk.compute() * .8
    if side == "right":
        world[:, 0] *= -1
    packet = {"source": "wuji_glove", "capture_unix_ms": 1_700_000_000_000,
              "hand": {"label": side.title(), "world": world.tolist()}, "raw_wuji": {"sequence": 1}}
    try:
        session.command("tracking_start", {})
        for i in range(5):
            result = session.process(packet | {"sequence": i, "timestamp_ms": 1000 + i * 50})
        assert result["tracking"] and result["depth_source"] == "wuji_glove_metric", result.get("message")
        assert not session.control.snapshot()["armed"]
        assert not session.process(packet | {"sequence": 5, "timestamp_ms": 1250, "hand": None})["tracking"]
    finally:
        session.close()
    import json
    rows = [json.loads(line) for line in session.recorder.path.read_text(encoding="utf-8").splitlines()]
    assert {"wuji_glove_raw", "wuji_glove", "retarget"} <= {r["type"] for r in rows}
    assert not {"mediapipe", "mediapipe_raw"} & {r["type"] for r in rows}


def test_a32_glove_native_recording_does_not_require_image_points(tmp_path, monkeypatch):
    from test_a32_tracking_recording import make_session, records
    session, _ = make_session(tmp_path, monkeypatch)
    session.input_source = "wuji_glove"
    world = session.pipeline.session.retargeter.fk.compute()
    try:
        session.command("connect", {"port": "SIM_ONLY"})
        packet = {"sequence": 0, "timestamp_ms": 1000., "capture_unix_ms": 1_700_000_000_000,
                  "source": "wuji_glove", "hand": {"label": "Left", "world": world.tolist()},
                  "raw_wuji": {"sequence": 1}}
        assert session.process(packet)["tracking"]
        assert not session.process(packet | {"sequence": 1, "timestamp_ms": 1050, "hand": None})["tracking"]
    finally:
        session.close()
    rows = records(session)
    assert any(r["type"] == "wuji_glove_raw" and r["raw_wuji"] == {"sequence": 1} for r in rows)
    assert not any(r["type"] == "mediapipe_raw" for r in rows)
