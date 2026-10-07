"""Test-only HTTP server with production encoders and memory-only USB."""
import sys
import threading
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_web_v2 import MemoryUsb, module, make_server

module.UsbController = MemoryUsb
server = make_server(8768, Path(__file__).resolve().parents[1] / "logs/web_v2_browser")
def sweep():
    while True:
        threading.Event().wait(.25)
        server.application.expire()
threading.Thread(target=sweep, daemon=True).start()
try:
    server.serve_forever()
finally:
    server.server_close()
    server.application.close()
