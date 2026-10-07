"""Bounded binary HTTP streaming, independent of Qt and UI refresh rates."""
import struct

MAX_RECORD = 264000


class StreamLease:
    """Release ownership even when a client closes before the first iteration."""
    def __init__(self, records, release):
        self.records, self.release = records, release
        self.closed = False

    def __iter__(self):
        return self

    def __next__(self):
        try:
            return next(self.records)
        except BaseException:
            self.close()
            raise

    def close(self):
        if not self.closed:
            self.closed = True
            try:
                self.records.close()
            finally:
                self.release()


def serve_records(handler, records):
    try:
        handler.send_response(200)
        handler.send_header('Content-Type', 'application/octet-stream')
        handler.send_header('Transfer-Encoding', 'chunked')
        handler.send_header('Cache-Control', 'no-store, no-transform')
        handler.send_header('X-Accel-Buffering', 'no')
        # Chrome otherwise may buffer the first bytes to sniff their MIME type.
        handler.send_header('X-Content-Type-Options', 'nosniff')
        handler.end_headers()
        handler.connection.settimeout(.2)
        for payload in records:
            if not 0 < len(payload) <= MAX_RECORD:
                raise ValueError('Invalid control record length')
            data = struct.pack('<I', len(payload)) + payload
            handler.wfile.write(f'{len(data):x}\r\n'.encode() + data + b'\r\n')
        handler.wfile.write(b'0\r\n\r\n')
    except (OSError, ValueError):
        pass
    finally:
        records.close()
        handler.close_connection = True
