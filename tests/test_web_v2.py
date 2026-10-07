"""Real V2 solver/Ruckig/protocol tests with memory-only serial writes."""
from pathlib import Path
from types import SimpleNamespace
import json
import sys
import time
import threading
import urllib.request
import urllib.error

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from online_control.online_motion_track.main import Application, make_server, model_description
from online_control.online_motion_track.v2 import session as module
from usb.offline import UsbController, UsbDeviceInfo


class MemoryUsb(UsbController):
    """Keep production encoders and recorder hooks; never open a real port."""
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.writes = []

    def scan_devices(self):
        return [UsbDeviceInfo("MOCK-H7", "模拟 H7（测试）")]

    def connect(self, device):
        self._current_info = device
        self._connected = True
        self._ser = SimpleNamespace(is_open=True)
        return True

    def disconnect(self):
        self._connected = False
        self._ser = None
        self._current_info = None

    def _write_frame_impl(self, frame, *args, **kwargs):
        if not self.is_connected:
            return False
        self.writes.append(bytes(frame))
        return True


def packet(s, i, *, label=None):
    q = s.solver.fk.compute()
    if s.side == "right":
        q[:, 0] *= -1
    return {"sequence": i, "timestamp_ms": 1000 + i * 50,
            "capture_unix_ms": 1789700000000 + i * 50,
            "hand": {"label": label or s.side.title(), "score": .99,
                     "world": q.tolist(), "normalized": q.tolist()}}


@pytest.mark.parametrize("side", ["left", "right"])
def test_v2_full_chain_native_recording_and_wrong_hand_interlock(side, tmp_path, monkeypatch):
    monkeypatch.setattr(module, "UsbController", MemoryUsb)
    app = Application(tmp_path)
    created = app.create({"model_id": "TsenseI_v2", "side": side, "collision_mode": "stop"})
    token = created["session"]
    s = app.get(token)
    command = lambda action, **kw: app.v2_command({"session": token, "action": action, **kw})
    try:
        assert command("ports")["ports"][0]["port"] == "MOCK-H7"
        command("connect", port="MOCK-H7")
        with pytest.raises(ValueError, match="有效目标"):
            command("arm", enabled=True)
        command("tracking_start")
        for i in range(5):
            result = s.process(packet(s, i))
            time.sleep(.025)
        assert result["tracking"], result["message"]
        assert len(result["pose"]) == 21
        assert result["pose"][f"{side}_hand_pinky_rota_joint"] == 0
        assert command("arm", enabled=True)["armed"]
        time.sleep(.08)
        assert command("status")["send_ticks"] > 0
        s.process(packet(s, 5, label="Right" if side == "left" else "Left"))
        assert not command("status")["armed"]
        before = len(s.control.usb.writes)
        time.sleep(.03)
        assert len(s.control.usb.writes) == before
        broken = packet(s, 6)
        broken["hand"]["world"][0][0] = float("nan")
        with pytest.raises(ValueError):
            s.process(broken)
        assert not s.control.snapshot()["armed"] and not s.control.snapshot()["target_ready"]
    finally:
        app.close()
    rows = [json.loads(l) for l in Path(created["log_file"]).read_text(encoding="utf-8").splitlines()]
    assert rows[0]["schema"] == "hand-control-session/v2"
    assert rows[0]["metadata"]["control_timing"]["mode"] == "elapsed"
    assert rows[-1]["type"] == "session_end"
    assert {"mediapipe_raw", "mediapipe", "retarget", "control_target", "h7", "conversion", "downlink", "usb_tx"} <= {r["type"] for r in rows}
    assert all(r["hand_side"] == side for r in rows if "hand_side" in r)
    assert all(0 < r["control_dt_s"] <= .1 for r in rows if r["type"] == "h7")
    commands = {r["seq"]: r for r in rows if r["type"] == "conversion"}
    sent = [r for r in rows if r["type"] == "downlink" and r["sent"]]
    assert sent and all(r["command_id"] in commands for r in sent)
    assert all(r["frame_id"] == commands[r["command_id"]]["frame_id"] for r in sent)
    assert all(len(r["linear_mm"]) == 14 and len(r["rotary_deg"]) == 7 for r in sent)
    frames = [json.loads(l) for l in Path(created["frame_log_file"]).read_text(encoding="utf-8").splitlines()]
    assert frames[0]["schema_version"] == 2 and frames[-1]["type"] == "session_end"
    assert frames[-2]["hand_selection_match"] is False


def test_exclusive_hardware_homing_stop_and_client_expiry(tmp_path, monkeypatch):
    monkeypatch.setattr(module, "UsbController", MemoryUsb)
    app = Application(tmp_path)
    tokens = [app.create({"model_id": "TsenseI_v2", "recording": False, "collision_mode": "stop"})["session"] for _ in range(2)]
    first = app.get(tokens[0])
    cmd = lambda token, action, **kw: app.v2_command({"session": token, "action": action, **kw})
    try:
        cmd(tokens[0], "connect", port="MOCK-H7")
        with pytest.raises(ValueError, match="另一个页面"):
            cmd(tokens[1], "connect", port="MOCK-H7")
        with pytest.raises(ValueError, match="当前页面"):
            cmd(tokens[1], "home")
        assert cmd(tokens[0], "home", iq_seek=700)["homing_active"]
        with pytest.raises(ValueError, match="有效目标"):
            cmd(tokens[0], "arm", enabled=True)
        assert not cmd(tokens[0], "cancel_home")["homing_active"]
        cmd(tokens[0], "home", iq_seek=700)
        first.last_seen = time.monotonic() - 4
        app.expire()
        assert first.closed and not first.control.is_connected
        assert not first.control.snapshot()["homing_active"]
        assert app.hardware_owner is None
        cmd(tokens[1], "connect", port="MOCK-H7")
        assert cmd(tokens[1], "emergency_stop")["armed"] is False
    finally:
        app.close()


def test_public_v2_is_preview_only_and_logs_are_isolated(tmp_path, monkeypatch):
    monkeypatch.setattr(module, "UsbController", MemoryUsb)
    app = Application(tmp_path, public=True, allowed_hosts=["hand.test"])
    try:
        a = app.create({"model_id": "TsenseI_v2", "collision_mode": "stop"})
        b = app.create({"model_id": "TsenseI_v2", "collision_mode": "stop"})
        assert not a["recording"] and not b["recording"]
        assert app.get(a["session"]).solver is not app.get(b["session"]).solver
        assert app.get(a["session"]).recorder is not app.get(b["session"]).recorder
        for action in ("ports", "connect", "home", "arm"):
            with pytest.raises(ValueError, match="公共服务"):
                app.v2_command({"session": a["session"], "action": action})
    finally:
        app.close()


def test_collision_off_cannot_arm_and_invalid_iq_cannot_home(tmp_path, monkeypatch):
    monkeypatch.setattr(module, "UsbController", MemoryUsb)
    app = Application(tmp_path)
    try:
        token = app.create({"model_id": "TsenseI_v2", "collision_mode": "off", "recording": False})["session"]
        app.v2_command({"session": token, "action": "connect", "port": "MOCK-H7"})
        with pytest.raises(ValueError, match="碰撞保护"):
            app.v2_command({"session": token, "action": "arm", "enabled": True})
        for value in (float("nan"), -1, 5001):
            with pytest.raises(ValueError, match="Iq"):
                app.v2_command({"session": token, "action": "home", "iq_seek": value})
    finally:
        app.close()


def test_http_model_static_and_control_origin(tmp_path, monkeypatch):
    monkeypatch.setattr(module, "UsbController", MemoryUsb)
    server = make_server(0, tmp_path)
    thread = threading.Thread(target=server.serve_forever, daemon=True); thread.start()
    base = f"http://127.0.0.1:{server.server_port}"
    def post(route, data, origin=base):
        headers = {"Content-Type": "application/json"}
        if origin: headers["Origin"] = origin
        with urllib.request.urlopen(urllib.request.Request(base + route, json.dumps(data).encode(), headers)) as r:
            return json.load(r)
    try:
        for side in ("left", "right"):
            desc = json.load(urllib.request.urlopen(base + f"/api/model?model=TsenseI_v2&side={side}"))
            assert len(desc["initial_pose"]) == 21
            mesh = desc["links"][0]["visuals"][0]["mesh"]
            assert urllib.request.urlopen(base + mesh).status == 200
        assert urllib.request.urlopen(base + "/data-viewer").read() == (ROOT / "tools/hand_control_log_viewer.html").read_bytes()
        s = post("/api/session", {"model_id": "TsenseI_v2", "collision_mode": "stop"})
        for origin in (None, "http://evil.test"):
            with pytest.raises(urllib.error.HTTPError) as exc:
                post("/api/v2/control", {"session": s["session"], "action": "ports"}, origin)
            assert exc.value.code == 403
        assert post("/api/v2/control", {"session": s["session"], "action": "ports"})["ports"]
        with pytest.raises(urllib.error.HTTPError) as exc:
            urllib.request.urlopen(base + "/api/recording?id=" + s["download_id"])
        assert exc.value.code == 409
        post("/api/stop", {"session": s["session"]})
        saved = urllib.request.urlopen(base + "/api/recording?id=" + s["download_id"])
        assert "attachment" in saved.headers["Content-Disposition"]
        assert b"hand-control-session/v2" in saved.read()
    finally:
        server.shutdown(); server.server_close(); server.application.close(); thread.join()
