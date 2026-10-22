"""HTTP/TLS peer shutdowns must not become a second error response."""
import io
from pathlib import Path
import ssl
import sys
from types import SimpleNamespace

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src"), str(ROOT / "src/tracking")]
from online_control.online_motion_track import main


class RequestSocket:
    def __init__(self, request, *, fail_write=None, error=None, fail_read=False):
        class RequestBody(io.BytesIO):
            def read(self, size=-1):
                if fail_read:
                    raise error
                return super().read(size)

        self.input = RequestBody(request)
        self.writes = []
        self.fail_write = fail_write
        self.error = error

    def makefile(self, *args):
        return self.input

    def setsockopt(self, *args):
        pass

    def settimeout(self, *args):
        pass

    def sendall(self, data):
        self.writes.append(bytes(data))
        if len(self.writes) == self.fail_write:
            raise self.error


def post_request(body=b'{"session":"pc"}'):
    return (b'POST /api/webxr/latest HTTP/1.1\r\nHost: localhost\r\n'
            b'Content-Type: application/json\r\nContent-Length: '
            + str(len(body)).encode() + b'\r\n\r\n' + body)


def application(**kwargs):
    return SimpleNamespace(allowed_hosts={"localhost"}, public=False,
                           webxr_latest=lambda data: {"tracking": False}, **kwargs)


@pytest.mark.parametrize("stage", [1, 2])
@pytest.mark.parametrize("error_type", [BrokenPipeError, ConnectionResetError,
    ConnectionAbortedError, ssl.SSLEOFError, ssl.SSLZeroReturnError])
def test_cancelled_post_response_never_retries_with_500(stage, error_type, capsys):
    sock = RequestSocket(post_request(), fail_write=stage, error=error_type("peer closed"))
    handler = main.Handler(sock, ("127.0.0.1", 1234), None, application=application())
    assert handler.close_connection
    assert len(sock.writes) == stage
    assert sum(write.startswith(b"HTTP/1.1 ") for write in sock.writes) == 1
    assert b"500" not in sock.writes[0]
    assert "Traceback" not in capsys.readouterr().err


def test_bad_length_write_never_attempts_a_second_response():
    error = ssl.SSLError(ssl.SSL_ERROR_SSL, "[SSL: BAD_LENGTH] bad length")
    error.reason = "BAD_LENGTH"
    sock = RequestSocket(post_request(), fail_write=2, error=error)
    with pytest.raises(ssl.SSLError, match="BAD_LENGTH"):
        main.Handler(sock, ("127.0.0.1", 1234), None, application=application())
    assert len(sock.writes) == 2
    assert sum(write.startswith(b"HTTP/1.1 ") for write in sock.writes) == 1


@pytest.mark.parametrize("error_type", [ssl.SSLEOFError, ConnectionResetError])
def test_cancelled_post_upload_never_attempts_an_error_response(error_type):
    sock = RequestSocket(post_request(), fail_read=True, error=error_type("peer closed"))
    handler = main.Handler(sock, ("127.0.0.1", 1234), None, application=application())
    assert handler.close_connection
    assert not sock.writes


def test_incomplete_upload_closes_without_response():
    sock = RequestSocket(post_request()[:-4])
    handler = main.Handler(sock, ("127.0.0.1", 1234), None, application=application())
    assert handler.close_connection
    assert not sock.writes


def test_cancelled_static_asset_response_closes_connection():
    sock = RequestSocket(b"GET /q HTTP/1.1\r\nHost: localhost\r\n\r\n",
                         fail_write=2, error=ssl.SSLEOFError("peer closed"))
    handler = main.Handler(sock, ("127.0.0.1", 1234), None, application=application())
    assert handler.close_connection
    assert len(sock.writes) == 2


@pytest.mark.parametrize("error_type", [RuntimeError, OSError, BrokenPipeError, ssl.SSLEOFError])
def test_business_failure_still_returns_500(error_type):
    app = application()

    def failed(data):
        raise error_type("solver failed")

    app.webxr_latest = failed
    sock = RequestSocket(post_request())
    main.Handler(sock, ("127.0.0.1", 1234), None, application=app)
    assert sock.writes[0].startswith(b"HTTP/1.1 500 ")
    assert b"solver failed" in sock.writes[1]


def test_invalid_request_still_returns_400():
    sock = RequestSocket(post_request(b"{broken}"))
    main.Handler(sock, ("127.0.0.1", 1234), None, application=application())
    assert sock.writes[0].startswith(b"HTTP/1.1 400 ")


def test_response_state_resets_between_keep_alive_requests():
    sock = RequestSocket(post_request() + post_request(b"{broken}"))
    main.Handler(sock, ("127.0.0.1", 1234), None, application=application())
    assert sock.writes[0].startswith(b"HTTP/1.1 200 ")
    assert sock.writes[2].startswith(b"HTTP/1.1 400 ")


def report_error(server, error, peer="127.0.0.1"):
    try:
        raise error
    except Exception:
        server.handle_error(None, (peer, 1234))


def test_certificate_rejection_warns_once_per_peer_per_interval(monkeypatch, capsys):
    now = [100.0]
    monkeypatch.setattr(main, "time", SimpleNamespace(monotonic=lambda: now[0]))
    server = main.LimitedHTTPServer(("127.0.0.1", 0), lambda *args: None)
    error = ssl.SSLError(ssl.SSL_ERROR_SSL, "certificate unknown")
    error.reason = "SSLV3_ALERT_CERTIFICATE_UNKNOWN"
    try:
        for _ in range(20):
            report_error(server, error)
        first = capsys.readouterr().err
        assert first.count("rejected the server certificate") == 1
        assert "quest_webxr_ca.crt" in first
        report_error(server, error, peer="127.0.0.2")
        assert capsys.readouterr().err.count("rejected the server certificate") == 1
        now[0] += server.warning_interval
        report_error(server, error)
        repeated = capsys.readouterr().err
        assert repeated.count("rejected the server certificate") == 1
        assert "19 repeated warnings suppressed" in repeated
    finally:
        server.server_close()


@pytest.mark.parametrize("path", ["/api/webxr/publish", "/api/quest-adb/publish"])
def test_publish_400_diagnostic_is_limited_but_every_response_retains_error(path, monkeypatch, capsys):
    now = [100.0]
    monkeypatch.setattr(main, "time", SimpleNamespace(monotonic=lambda: now[0]))
    server = main.LimitedHTTPServer(("127.0.0.1", 0), lambda *args: None)
    request = post_request(b"{broken}").replace(b"/api/webxr/latest", path.encode())
    try:
        for _ in range(20):
            sock = RequestSocket(request)
            main.Handler(sock, ("127.0.0.1", 1234), server, application=application())
            assert sock.writes[0].startswith(b"HTTP/1.1 400 ")
            assert b"Expecting property name" in sock.writes[1]
        errors = capsys.readouterr().err
        assert errors.count("rejected (400)") == 1
        assert errors.count("Expecting property name") == 1
        assert 'HTTP/1.1" 400' not in errors
        now[0] += server.warning_interval
        main.Handler(RequestSocket(request), ("127.0.0.1", 1234), server, application=application())
        assert "19 repeated warnings suppressed" in capsys.readouterr().err
    finally:
        server.server_close()


def test_publish_diagnostics_do_not_log_token_or_payload(capsys):
    server = main.LimitedHTTPServer(("127.0.0.1", 0), lambda *args: None)

    def failed(token, message):
        raise ValueError(f"expired token: {token}\nretry")

    app = application(webxr=SimpleNamespace(publish=failed))
    request = post_request(b'{"token":"secret-capability","message":{"private":"sample"}}')
    request = request.replace(b"/api/webxr/latest", b"/api/webxr/publish")
    try:
        main.Handler(RequestSocket(request), ("127.0.0.1", 1234), server, application=app)
        errors = capsys.readouterr().err
        assert "expired token: [redacted] retry" in errors
        assert "secret-capability" not in errors
        assert "sample" not in errors
        assert len(errors.splitlines()) == 1
    finally:
        server.server_close()


def test_server_silences_peer_shutdown_but_logs_unexpected_errors(capsys):
    server = main.LimitedHTTPServer(("127.0.0.1", 0), lambda *args: None)
    try:
        report_error(server, ssl.SSLEOFError("peer closed"))
        assert capsys.readouterr().err == ""
        report_error(server, ssl.SSLError(ssl.SSL_ERROR_SSL, "unexpected TLS failure"))
        errors = capsys.readouterr().err
        assert "Traceback" in errors
        assert "unexpected TLS failure" in errors
    finally:
        server.server_close()
