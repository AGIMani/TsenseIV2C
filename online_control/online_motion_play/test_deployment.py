"""Authentication, control ownership, deployment configuration and HTTP checks."""
import http.cookiejar
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
import urllib.error
import urllib.request

import pytest

from access import AccessControl
import deploy_config
import package
from server import ControlHTTPServer, Handler
from test_server import bridge  # Reuse the no-hardware session fixture.


@pytest.fixture(scope="module")
def encoded():
    return "pbkdf2_sha256$600000$old-salt$old-hash"


def test_password_is_ignored_including_old_cookies(encoded):
    access = AccessControl(["http://192.168.1.10:8765"], encoded)
    assert not access.required
    assert access.authenticated({})
    assert access.authenticated({"Cookie": "motion_session=expired"})
    assert access.token({"Cookie": "motion_session=expired"}) == ""


def test_explicit_lan_host_and_origin_allowlist(encoded):
    access = AccessControl(["http://192.168.1.10:8765", "https://hand.example.lan"], encoded, secure=True)
    assert access.allowed({"Host": "192.168.1.10:8765", "Origin": "http://192.168.1.10:8765"})
    assert access.allowed({"Host": "hand.example.lan", "Origin": "https://hand.example.lan"})
    assert not access.allowed({"Host": "evil.example:8765"})
    assert not access.allowed({"Host": "192.168.1.10:8765", "Origin": "https://evil.example"})
    assert not access.allowed({"Host": "192.168.1.10:8765", "Sec-Fetch-Site": "cross-site"})


def test_observers_cannot_control_or_extend_owner_lease(bridge):
    bridge.dispatch("control", {}, client="owner")
    assert bridge.dispatch("state", {}, client="owner")["control"]["owned"]
    with pytest.raises(ValueError, match="另一个"):
        bridge.dispatch("control", {}, client="observer")
    spec = bridge.snapshot()["joints"][0]
    with pytest.raises(ValueError, match="只读"):
        bridge.dispatch("widget", {"id": spec["id"], "value": .5}, client="observer")
    stamp = bridge.last_seen
    assert not bridge.dispatch("state", {}, client="observer")["control"]["owned"]
    bridge.dispatch("health", {}, client="")
    assert bridge.last_seen == stamp
    bridge.last_seen = time.monotonic() - 6
    assert bridge.dispatch("state", {}, client="observer")["control"]["available"]
    assert bridge.controller is None
    bridge.dispatch("control", {}, client="observer")
    assert bridge.controller == "observer"
    bridge.dispatch("release", {}, client="observer")
    assert bridge.controller is None


def test_configure_removes_old_password_preserves_port_and_data(tmp_path, monkeypatch):
    monkeypatch.setattr(deploy_config, "detected_hosts", lambda: {"192.168.1.10", "handserver"})
    config_path = tmp_path / "etc/config.json"
    data = tmp_path / "data"
    data.mkdir()
    motion = data / "motions.json"
    motion.write_text('{"saved": {}}')
    first = deploy_config.configure(config_path, data, port=9876)
    assert "password_hash" not in first
    old = {**first, "password_hash": "old-hash"}
    config_path.write_text(json.dumps(old))
    credential_path = config_path.parent / "access-password.txt"
    credential_path.write_text("old-password")
    second = deploy_config.configure(config_path, data, extra_hosts=["hand.example.lan"])
    assert "password_hash" not in second
    assert not credential_path.exists()
    assert second["port"] == 9876
    assert "hand.example.lan" in second["allowed_hosts"]
    assert motion.read_text() == '{"saved": {}}'


def test_bundle_manifest_excludes_runtime_secrets_and_environment():
    manifest = dict((relative, path) for path, relative in package.files())
    assert {"server.py", "deploy.sh", "requirements-server.txt", "app/src/joint_schema.py",
            "serial-worker.js", "worker-client.js", "position-guard.js", "motion-stream.js"} <= manifest.keys()
    assert not any(name.startswith('bench-worker') for name in manifest)
    assert "app/assets/hands/TsenseI_hand/urdf/TsenseI__hand_right_v1.urdf" in manifest
    assert not any(part in ("runtime", ".venv", "__pycache__", ".git") for name in manifest for part in name.split("/"))
    assert "config.json" not in manifest and "access-password.txt" not in manifest


def test_headless_import_does_not_load_camera_or_other_windows():
    code = "import sys; import server; assert not any(m in sys.modules for m in ['PySide6','shiboken6','cv2','torch','mediapipe','skeleton.tracker','ui','ui.form3_window','urdf.display']); print('OK')"
    output = subprocess.check_output([sys.executable, "-c", code], cwd=Path(__file__).parent, text=True)
    assert "OK" in output


def test_server_refuses_a_port_already_in_use():
    with ControlHTTPServer(("127.0.0.1", 0), Handler) as running:
        with pytest.raises(OSError):
            ControlHTTPServer(running.server_address, Handler)


@pytest.fixture(scope="module")
def live_server(tmp_path_factory, encoded):
    folder = tmp_path_factory.mktemp("lan-server")
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    config = folder / "config.json"
    config.write_text(json.dumps({"host": "0.0.0.0", "port": port,
                                 "data_dir": str(folder / "data"), "password_hash": encoded}))
    process = subprocess.Popen([sys.executable, str(Path(__file__).with_name("server.py")),
                                "--config", str(config), "--no-browser"],
                               stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                               env=os.environ.copy())
    base = f"http://127.0.0.1:{port}"
    try:
        deploy_config.probe(config, timeout=15)
        yield base
    finally:
        process.terminate()
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)


def test_http_password_free_control_download_and_legacy_logout(live_server):
    with urllib.request.urlopen(live_server + "/shared/serial-timing.js", timeout=5) as response:
        assert b"export function makePacer(" in response.read()
    with urllib.request.urlopen(live_server + "/shared/a30-telemetry.js", timeout=5) as response:
        assert b"export function telemetry(" in response.read()
    cookiejar = http.cookiejar.CookieJar()
    client = urllib.request.build_opener(urllib.request.HTTPCookieProcessor(cookiejar))

    def request(action, body=None, extra=None):
        headers = {"X-Motion-Client": "a" * 32, **(extra or {})}
        if body is not None:
            headers["Content-Type"] = "application/json"
        req = urllib.request.Request(live_server + "/api/" + action,
                                     None if body is None else json.dumps(body).encode(), headers)
        return client.open(req, timeout=5)

    with request("state") as response:
        assert not json.load(response)["control"]["owned"]
    with request("session") as response:
        assert json.load(response) == {"authenticated": True, "required": False}
    # Compatibility route accepts any value; no cookie or login is required.
    with request("login", {"password": "not-the-old-password"}) as response:
        assert json.load(response)["ok"]
    assert not cookiejar
    with request("control", {}) as response:
        assert json.load(response)["ok"]
    with request("state") as response:
        assert json.load(response)["control"]["owned"]
    with request("state", extra={"X-Motion-Client": "b" * 32}) as response:
        assert not json.load(response)["control"]["owned"]
    with pytest.raises(urllib.error.HTTPError) as denied:
        request("state", extra={"Origin": "https://evil.example"})
    assert denied.value.code == 403
    with request("import", {"text": '{"pose":{"left_hand_index_MCP_joint":0.3}}'}) as response:
        assert json.load(response)["ok"]
    with request("download") as response:
        assert "attachment" in response.headers["Content-Disposition"]
        assert json.load(response)["pose"]["left_hand_index_MCP_joint"] == .3
    with request("logout", {}) as response:
        assert json.load(response)["ok"]
    with request("state") as response:
        assert not json.load(response)["control"]["owned"]
