"""Native Linux browser motion service with a serialized, Qt-free control loop."""
from __future__ import annotations

import argparse
import hashlib
import ipaddress
import json
import mimetypes
import os
from pathlib import Path
import queue
import shutil
import signal
import socket
import ssl
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, unquote, urlsplit
import webbrowser

from access import AccessControl

HERE = Path(__file__).resolve().parent
ROOT = HERE / "app" if (HERE / "app" / "src").is_dir() else HERE.parents[1]
sys.path.insert(0, str(ROOT / "src"))
sys.path.insert(0, str(ROOT))
from online_control.shared.version import downlink_version
from utils.control_clock import precise_control_timer
from utils.control_log_catalog import control_log_catalog, control_log_file, set_control_log_parent


class Bridge:
    """Serialize page commands while the A3.2 SDK keeps its own control clock."""
    TICK_S = .005
    WATCHDOG_S = .05
    CONTROLLER_LEASE_S = 5.

    def __init__(self, data_dir=None, *, session=None):
        from a32_session import A32Session
        self.data_dir = Path(data_dir) if data_dir else HERE / "runtime"
        self.data_dir.mkdir(parents=True, exist_ok=True)
        self.a32 = session if session is not None else A32Session(self.data_dir, model_id="TsenseI_v3_2_withid")
        self.controller = None
        self.last_seen = time.monotonic()
        self.jobs = queue.Queue()
        self._ownership = threading.RLock()
        self._shutdown = threading.Event()
        self._closed = False
        self._last_watchdog = time.monotonic()

    def call(self, action, body=None, client=None, *, trusted_local=True):
        if action in ("serial_binary", "serial_rx", "serial_stream", "serial"):
            raise ValueError("本服务使用原生 SDK，不支持浏览器串口")
        response = queue.Queue(maxsize=1)
        job = {"action": action, "body": body or {}, "response": response,
               "deadline": time.monotonic() + 10, "client": client, "trusted_local": trusted_local}
        with self._ownership:
            if self._shutdown.is_set():
                raise ValueError("控制服务正在关闭")
            self.jobs.put(job)
        try:
            ok, value = response.get(timeout=12)
        except queue.Empty:
            raise ValueError("控制服务响应超时，请刷新页面") from None
        if not ok:
            raise ValueError(value)
        return value

    def execute(self, job):
        try:
            if self._shutdown.is_set():
                raise ValueError("控制服务正在关闭")
            if time.monotonic() > job["deadline"]:
                raise ValueError("请求已过期，请重试")
            value = self.dispatch(job["action"], job["body"], job["client"], trusted_local=job.get("trusted_local", True))
            job["response"].put_nowait((True, value))
        except Exception as exc:
            job["response"].put_nowait((False, str(exc)))

    def run(self, stop_event=None):
        next_tick = time.monotonic()
        while not self._shutdown.is_set() and not (stop_event and stop_event.is_set()):
            now = time.monotonic()
            if now >= next_tick:
                self.tick()
                next_tick = time.monotonic() + self.TICK_S
            try:
                job = self.jobs.get(timeout=max(0., next_tick - time.monotonic()))
            except queue.Empty:
                continue
            self.execute(job)

    def tick(self):
        now = time.monotonic()
        if now - self._last_watchdog >= self.WATCHDOG_S:
            self._last_watchdog = now
            self.check_lease()

    def request_shutdown(self):
        self._shutdown.set()

    def close(self):
        if self._closed:
            return
        self._closed = True
        self.request_shutdown()
        with self._ownership:
            self.controller = None
        try:
            self.a32.close()
        finally:
            while True:
                try:
                    job = self.jobs.get_nowait()
                except queue.Empty:
                    break
                job["response"].put_nowait((False, "控制服务已关闭"))

    def stop(self, reason="user_stop"):
        self.a32.stop(reason=reason)

    def check_lease(self):
        with self._ownership:
            expired = self.controller is not None and time.monotonic() - self.last_seen > self.CONTROLLER_LEASE_S
            if not expired:
                return
            self.controller = None
        self.stop(reason="controller_heartbeat_timeout")
        self.a32.message = "控制页面心跳超时，已停止下发并释放控制权；SDK 连接保留，获取控制后可手动恢复"

    def snapshot(self):
        return self.a32.snapshot()

    def dispatch(self, action, body, client=None, *, trusted_local=True):
        if not trusted_local and (
            action == "connect" or self.a32.connected and action in ("widget", "import", "stop", "disconnect", "release")
        ):
            raise ValueError("A3.2 with ID 实机操作仅允许服务电脑本机同源浏览器；远程页面可离线查看和编辑")
        if client is not None:
            self.check_lease()
            with self._ownership:
                if action == "control":
                    if self.controller and self.controller != client:
                        raise ValueError("另一个页面正在控制设备，可先只读查看")
                    self.controller = client
                    self.last_seen = time.monotonic()
                    return True
                if action == "release":
                    owned = self.controller == client
                    if owned:
                        self.controller = None
                elif action not in ("state", "ports", "export", "health", "recording_download") and self.controller != client:
                    raise ValueError("当前为只读查看，请先获取控制权")
            if action == "release":
                if owned:
                    self.stop(reason="controller_release")
                    self.a32.device.disconnect()
                return True
        with self._ownership:
            if client is None or client == self.controller:
                self.last_seen = time.monotonic()
        if action == "health":
            return {"status": "ok", "service": "online-motion-play", "serial_mode": "native-sdk", "downlink_sha256": downlink_version(), "binary_stream": False}
        if action == "recording_download":
            if not trusted_local:
                raise ValueError("A3.2 控制记录请在本机页面下载")
            return self.a32.recording_download()
        if action == "state":
            result = self.snapshot()
            result["control"] = {"owned": client is None or client == self.controller,
                                 "available": self.controller is None}
            result["serial_mode"] = "native-sdk"
            result["sdk_local_access"] = bool(trusted_local)
            return result
        if action == "ports":
            return []
        if action == "model":
            if body.get("model_id") != "TsenseI_v3_2_withid":
                raise ValueError("仅支持 A3.2 with ID")
            return True
        if action == "widget" and body.get("id") == self.a32.generation + ":handSide":
            if self.a32.connected or type(body.get("value")) is not int or body["value"] not in (0, 1):
                raise ValueError("请断开 SDK 后选择有效手别")
            from a32_session import A32Session
            replacement = A32Session(self.data_dir, ("left", "right")[body["value"]], model_id=self.a32.model_id)
            self.a32.close()
            self.a32 = replacement
            return True
        return self.a32.dispatch(action, body)


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def setup(self):
        super().setup()
        self.connection.settimeout(15)
        self.connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    def handle_one_request(self):
        # The browser may cancel a keep-alive socket while replacing a model.
        # Limit this guard to transport shutdowns; malformed requests and other
        # server failures retain their normal diagnostics.
        try:
            super().handle_one_request()
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError,
                ssl.SSLEOFError, ssl.SSLZeroReturnError):
            self.close_connection = True

    def log_message(self, *_args):
        pass

    def reply(self, status, data, content_type="application/json; charset=utf-8", filename=None, cookie=None):
        if not isinstance(data, bytes):
            data = json.dumps(data, ensure_ascii=False, allow_nan=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        if status >= 400:
            # Invalid requests can leave unread bodies. Do not reuse that socket.
            self.close_connection = True
            self.send_header("Connection", "close")
        if filename:
            self.send_header("Content-Disposition", f'attachment; filename="{filename}"')
        if cookie:
            self.send_header("Set-Cookie", cookie)
        self.send_header("Cache-Control", "no-store" if content_type.startswith("application/json") else "no-cache")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("Content-Security-Policy", "frame-ancestors 'none'")
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
            pass

    def allowed(self):
        if hasattr(self.server, "access"):
            return self.server.access.allowed(self.headers)
        host = self.headers.get("Host", "")
        port = self.server.server_port
        return host in (f"127.0.0.1:{port}", f"localhost:{port}") and self.headers.get("Origin", f"http://{host}") == f"http://{host}"

    def client_key(self):
        client = self.headers.get("X-Motion-Client", "")
        if not 16 <= len(client) <= 80 or not all(c.isalnum() or c == "-" for c in client):
            raise ValueError("页面标识无效，请刷新页面")
        principal = hashlib.sha256(self.server.access.token(self.headers).encode()).hexdigest()
        return principal + ":" + client

    def local_sdk_request(self):
        try:
            local = ipaddress.ip_address(self.client_address[0]).is_loopback
            host = urlsplit("//" + self.headers.get("Host", "")).hostname
            local_host = host == "localhost" or bool(host and ipaddress.ip_address(host).is_loopback)
        except ValueError:
            return False
        origin = self.headers.get("Origin")
        same_origin = not origin or urlsplit(origin).netloc.lower() == self.headers.get("Host", "").lower()
        return local and local_host and same_origin and self.headers.get("Sec-Fetch-Site", "") != "cross-site"

    def do_GET(self):
        if not self.allowed():
            return self.reply(403, {"error": "访问地址未被允许，请检查服务器 allowed_hosts 配置"})
        path = unquote(urlsplit(self.path).path)
        try:
            if path in ("/api/viewer/logs", "/api/viewer/log-file"):
                if not self.local_sdk_request():
                    return self.reply(403, {"error": "本机日志仅允许本机同源页面查看"})
                query = parse_qs(urlsplit(self.path).query)
                model = query.get("model", [""])[0]
                if path == "/api/viewer/logs":
                    return self.reply(200, control_log_catalog(model))
                log = control_log_file(model, query.get("name", [""])[0])
                if log is None:
                    return self.reply(404, {"error": "日志文件不存在"})
                # The HTTP thread streams a large log without touching the
                # control clock or allocating the entire file in memory.
                with log.open("rb") as stream:
                    self.send_response(200)
                    self.send_header("Content-Type", "application/x-ndjson; charset=utf-8")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Connection", "close")
                    self.end_headers()
                    self.close_connection = True
                    shutil.copyfileobj(stream, self.wfile)
                return
            if path == "/api/health":
                return self.reply(200, self.server.bridge.call("health", client=""))
            if path == "/api/session":
                return self.reply(200, {"authenticated": self.server.access.authenticated(self.headers),
                                        "required": self.server.access.required})
            if path == "/api/download":
                payload = self.server.bridge.call("export", client="")
                data = (json.dumps(payload, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
                model_prefix = payload.get("_model_id", "") + "_" if payload.get("_model_id") == "TsenseI_v3_2_withid" else ""
                return self.reply(200, data, filename=f"motions_{model_prefix}{payload['_hand_side']}.json")
            if path == "/api/a32-log":
                data, mime, name = self.server.bridge.call("recording_download", client=self.client_key(), trusted_local=self.local_sdk_request())
                return self.reply(200, data, mime, filename=name)
            if path == "/control-log-viewer":
                return self.reply(200, (ROOT / "tools/hand_control_log_viewer.html").read_bytes(), "text/html; charset=utf-8")
            if path in ("/api/state", "/api/ports", "/api/export"):
                return self.reply(200, self.server.bridge.call(path.rsplit("/", 1)[-1], client=self.client_key(), trusted_local=self.local_sdk_request()))
            if path in ("/", "/index.html"):
                target = HERE / "index.html"
            elif path.startswith("/shared/") and path.rsplit("/", 1)[-1] in ("ui-language.js", "ui-translations.js", "aluminum.js"):
                target = ROOT / "online_control/shared" / path.rsplit("/", 1)[-1]
            elif path.startswith("/vendor/"):
                target = (HERE / path.lstrip("/")).resolve()
                if not target.is_relative_to(HERE / "vendor"):
                    raise ValueError("无效路径")
            elif path.startswith("/assets/"):
                target = (ROOT / path.lstrip("/")).resolve()
                if not target.is_relative_to(ROOT / "assets/hands/TsenseI_hand_v3_2_withid") or target.suffix.lower() not in (".urdf", ".stl"):
                    raise ValueError("无效模型路径")
            else:
                return self.reply(404, {"error": "Not found"})
            mime = "text/javascript" if target.suffix == ".js" else mimetypes.guess_type(str(target))[0] or "application/octet-stream"
            self.reply(200, target.read_bytes(), mime)
        except (ValueError, OSError) as exc:
            self.reply(400, {"error": str(exc)})

    def do_POST(self):
        if not self.allowed() or self.headers.get("Content-Type", "").split(";")[0] != "application/json":
            return self.reply(403, {"error": "只接受允许地址的同源 JSON 请求"})
        try:
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 8 * 1024 * 1024:
                raise ValueError("请求为空或超过 8 MB")
            body = json.loads(self.rfile.read(length))
            if not isinstance(body, dict):
                raise ValueError("请求须为 JSON 对象")
            path = urlsplit(self.path).path
            if path == "/api/viewer/log-parent":
                if not self.local_sdk_request() or not self.headers.get("Origin"):
                    return self.reply(403, {"error": "日志目录仅允许本机同源页面修改"})
                parent = set_control_log_parent(body["parent"])
                return self.reply(200, {"parent": str(parent), "folders": [str(parent / "A32")]})
            if path == "/api/login":
                # Compatibility for pages cached before password removal.
                return self.reply(200, {"ok": True, "required": False})
            if path == "/api/logout":
                self.server.bridge.call("release", client=self.client_key(), trusted_local=self.local_sdk_request())
                return self.reply(200, {"ok": True})
            if path not in {"/api/" + name for name in ("widget", "import", "connect", "disconnect", "stop", "control", "release", "model")}:
                return self.reply(404, {"error": "Not found"})
            result = self.server.bridge.call(path.rsplit("/", 1)[-1], body, client=self.client_key(), trusted_local=self.local_sdk_request())
            self.reply(200, {"ok": True, "result": result})
        except (ValueError, KeyError, TypeError, OSError) as exc:
            self.reply(400, {"error": str(exc)})


class ControlHTTPServer(ThreadingHTTPServer):
    allow_reuse_address = os.name != "nt"

    def server_bind(self):
        if os.name == "nt":
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_EXCLUSIVEADDRUSE, 1)
        super().server_bind()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, help="Deployment JSON configuration")
    parser.add_argument("--host", help="Bind address (default: localhost)")
    parser.add_argument("--port", type=int)
    parser.add_argument("--check", action="store_true", help="Check dependencies, models and configuration, then exit")
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()
    config = json.loads(args.config.read_text(encoding="utf-8-sig")) if args.config else {}
    host = args.host or config.get("host", "127.0.0.1")
    port = args.port if args.port is not None else config.get("port", 8765)
    if not isinstance(port, int) or not 1 <= port <= 65535:
        parser.error("port must be between 1 and 65535")
    cert, key = config.get("tls_cert"), config.get("tls_key")
    if bool(cert) != bool(key):
        parser.error("tls_cert and tls_key must be provided together")
    scheme = "https" if cert else "http"
    allowed_hosts = set(config.get("allowed_hosts", [])) | {"127.0.0.1", "localhost"}
    origins = {f"{scheme}://{name}:{port}" for name in allowed_hosts}
    public_origins = list(config.get("public_origins", []))
    if config.get("public_origin"):
        public_origins.append(config["public_origin"])
    for origin in public_origins:
        public = urlsplit(origin)
        if public.scheme not in ("http", "https") or not public.netloc or public.path not in ("", "/") or public.query or public.fragment or public.username:
            parser.error("public_origin must be an http(s) origin without a path")
        origins.add(origin.rstrip("/"))
    access = AccessControl(origins)
    data_dir = Path(config.get("data_dir", str(HERE / "runtime"))).resolve()
    data_dir.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryFile(dir=data_dir) as writable:
        writable.write(b"deployment write check")
    bridge = Bridge(data_dir)
    if args.check:
        try:
            from a32_session import A32Session
            for side in ("left", "right"):
                check_session = A32Session(data_dir, side, model_id="TsenseI_v3_2_withid", start=False)
                try:
                    state = check_session.snapshot()
                    check_session.validate_pose(check_session.pose)
                    if len(state["joints"]) != 20 or not (ROOT / state["urdf"].lstrip("/")).is_file():
                        raise ValueError("A3.2 with ID model or joint schema is incomplete")
                finally:
                    check_session.close()
            print("CHECK OK: Qt-free A3.2 with ID, left/right models, Ruckig and writable data directory; no hardware connected")
        finally:
            bridge.close()
        return
    server = None
    serving = False
    try:
        server = ControlHTTPServer((host, port), Handler)
        server.daemon_threads = True
        server.bridge = bridge
        server.access = access
        if cert:
            context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            context.minimum_version = ssl.TLSVersion.TLSv1_2
            context.load_cert_chain(cert, key)
            server.socket = context.wrap_socket(server.socket, server_side=True)
        signal.signal(signal.SIGINT, lambda *_: bridge.request_shutdown())
        signal.signal(signal.SIGTERM, lambda *_: bridge.request_shutdown())
        threading.Thread(target=server.serve_forever, daemon=True).start()
        serving = True
        url = f"{scheme}://127.0.0.1:{server.server_port}"
        print(f"Hand Motion Web: {url}", flush=True)
        if not args.no_browser:
            webbrowser.open(url)
        with precise_control_timer():
            bridge.run()
    finally:
        try:
            bridge.close()
        finally:
            if server is not None:
                if serving:
                    server.shutdown()
                server.server_close()


if __name__ == "__main__":
    main()
