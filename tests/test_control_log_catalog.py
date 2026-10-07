from __future__ import annotations

import json
from pathlib import Path
import sys
import threading
import urllib.error
import urllib.parse
import urllib.request


ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]

from utils import control_log_catalog as catalog
from utils.control_session_recorder import control_log_directory
from online_control.online_motion_track.main import make_server


def _write(path: Path, *, model="daxian_v3_0", side="left", part=None, ended=True):
    header = {"type": "session", "schema": "hand-control-session/v2", "started_at": "2026-09-24T00:00:00Z",
              "metadata": {"model_id": model, "hand_side": side}}
    if part:
        header["recording_part"] = part
    rows = [header, {"type": "retarget", "t": 60 if part and part["index"] == 2 else 0, "hand_side": side},
            {"type": "retarget", "t": 120 if part and part["index"] == 2 else 59, "hand_side": side}]
    if ended:
        rows.append({"type": "session_end", "t": 120, "duration_s": 120})
    path.write_text("".join(json.dumps(row) + "\n" for row in rows), encoding="utf-8")


def test_catalog_reads_config_and_merges_only_a30_control_files(tmp_path, monkeypatch):
    config = tmp_path / "parent.txt"
    parent = tmp_path / "custom-log-parent"
    parent.mkdir()
    folder = parent / "A30"
    folder.mkdir()
    config.write_text(str(parent), encoding="utf-8")
    monkeypatch.setattr(catalog, "PARENT_PATH_FILE", config)
    _write(folder / "hand_control_test_part001.jsonl", part={"index": 1, "count": 2, "session_id": "group"}, ended=False)
    _write(folder / "hand_control_test_part002.jsonl", part={"index": 2, "count": 2, "session_id": "group"})
    _write(folder / "hand_control_v2.jsonl", model="daxian_v2")
    (folder / "hand_diagnostic_sample.jsonl").write_text('{"type":"session","schema":"hand-diagnostic/v1"}\n', encoding="utf-8")
    result = catalog.a30_catalog()
    assert result["parent"] == str(parent)
    assert len(result["files"]) == len(result["records"]) == 2
    assert {record["id"] for record in result["records"]} == {"log:group:A30:left"}
    assert [record["parts"][0]["index"] for record in result["records"]] == [1, 2]
    assert result["records"][1]["parts"][0]["closed"] is True
    assert control_log_directory({"model_id": "TsenseI_v3_0"}) == folder
    assert catalog.a30_file("../hand_control_test_part001.jsonl") is None
    assert catalog.a30_file("hand_control_v2.jsonl") is None
    config.write_text(str(tmp_path / "other"), encoding="utf-8")
    assert catalog.a30_catalog()["files"] == []
    assert control_log_directory({"model_id": "TsenseI_v3_0"}) == tmp_path / "other" / "A30"


def test_all_models_and_parent_editor_create_directories(tmp_path, monkeypatch):
    config = tmp_path / "parent.txt"
    monkeypatch.setattr(catalog, "PARENT_PATH_FILE", config)
    try:
        catalog.set_control_log_parent("relative/logs")
        assert False, "Relative path was accepted"
    except ValueError:
        assert not config.exists()
    parent = catalog.set_control_log_parent(str(tmp_path / "chosen parent"))
    assert config.read_text(encoding="utf-8").strip() == str(parent)
    assert all((parent / folder).is_dir() for folder in ("A30", "A32", "V2"))
    _write(parent / "A32" / "hand_control_a32.jsonl", model="daxian_v3_2", side="right")
    _write(parent / "V2" / "hand_control_v2.jsonl", model="daxian_v2", side="left")
    assert catalog.control_log_catalog("A32")["records"][0]["side"] == "right"
    assert catalog.control_log_catalog("V2")["records"][0]["model"] == "V2"
    assert catalog.control_log_file("A32", "hand_control_a32.jsonl") is not None
    assert catalog.control_log_file("V2", "hand_control_a32.jsonl") is None
    try:
        catalog.control_log_catalog("invalid")
        assert False, "Unknown model was accepted"
    except ValueError:
        pass


def test_catalog_endpoint_is_local_only(tmp_path, monkeypatch):
    config = tmp_path / "parent.txt"
    parent = tmp_path / "logs"
    folder = parent / "A30"
    folder.mkdir(parents=True)
    config.write_text(str(parent), encoding="utf-8")
    monkeypatch.setattr(catalog, "PARENT_PATH_FILE", config)
    path = folder / "hand_control_test.jsonl"
    _write(path)
    server = make_server(0)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    base = f"http://127.0.0.1:{server.server_port}"
    opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        with opener.open(base + "/api/viewer/a30-usage", timeout=5) as response:
            result = json.load(response)
        assert result["files"] == [{"name": path.name, "bytes": path.stat().st_size}]
        with opener.open(base + "/api/viewer/logs?model=A30", timeout=5) as response:
            assert json.load(response)["records"][0]["model"] == "A30"
        with opener.open(base + "/api/viewer/logs?model=A32", timeout=5) as response:
            assert json.load(response)["files"] == []
        with opener.open(base + "/api/viewer/a30-file?name=" + urllib.parse.quote(path.name), timeout=5) as response:
            assert response.read() == path.read_bytes()
        with opener.open(base + "/api/viewer/log-file?model=A30&name=" + urllib.parse.quote(path.name), timeout=5) as response:
            assert response.read() == path.read_bytes()
        try:
            opener.open(base + "/api/viewer/a30-file?name=..%2Fsecret.jsonl", timeout=5)
            assert False, "Path traversal was accepted"
        except urllib.error.HTTPError as error:
            assert error.code == 404
        other = tmp_path / "new parent"
        payload = json.dumps({"parent": str(other)}).encode()
        without_origin = urllib.request.Request(base + "/api/viewer/log-parent", payload,
                                                {"Content-Type": "application/json"})
        try:
            opener.open(without_origin, timeout=5)
            assert False, "Missing origin was accepted"
        except urllib.error.HTTPError as error:
            assert error.code == 403
        request = urllib.request.Request(base + "/api/viewer/log-parent", payload,
                                         {"Content-Type": "application/json", "Origin": base})
        with opener.open(request, timeout=5) as response:
            assert json.load(response)["parent"] == str(other)
        assert all((other / folder).is_dir() for folder in ("A30", "A32", "V2"))
        assert config.read_text(encoding="utf-8").strip() == str(other)
        server.application.public = True
        try:
            opener.open(base + "/api/viewer/a30-usage", timeout=5)
            assert False, "Public access was accepted"
        except urllib.error.HTTPError as error:
            assert error.code == 403
        try:
            opener.open(request, timeout=5)
            assert False, "Public write was accepted"
        except urllib.error.HTTPError as error:
            assert error.code == 403
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=5)
