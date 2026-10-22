"""Ordinary model-fetch cancellations must not log HTTP handler tracebacks."""
import io
import ssl

import pytest

import server


class RequestSocket:
    def __init__(self, error):
        class InterruptedReader(io.BytesIO):
            def readline(self, *args):
                raise error
        self.input = InterruptedReader()

    def makefile(self, *_args):
        return self.input

    def setsockopt(self, *_args):
        pass

    def settimeout(self, *_args):
        pass

    def sendall(self, _data):
        raise AssertionError("A cancelled request must not receive an error response")


@pytest.mark.parametrize("error_type", [BrokenPipeError, ConnectionResetError,
    ConnectionAbortedError, ssl.SSLEOFError, ssl.SSLZeroReturnError])
def test_cancelled_keep_alive_request_closes_without_response(error_type):
    handler = server.Handler(RequestSocket(error_type("peer closed")),
                             ("127.0.0.1", 1234), None)
    assert handler.close_connection


@pytest.mark.parametrize("error", [OSError("unexpected socket failure"),
    ssl.SSLError(ssl.SSL_ERROR_SSL, "unexpected TLS failure")])
def test_unexpected_transport_failure_is_not_silenced(error):
    with pytest.raises(type(error), match="unexpected"):
        server.Handler(RequestSocket(error), ("127.0.0.1", 1234), None)
