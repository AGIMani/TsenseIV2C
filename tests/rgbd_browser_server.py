"""Test-only RGB-D HTTP server with synthetic depth and real fusion/retargeting.

No camera SDK or robot is opened. RGB pixel (0, 0) encodes the paired frame ID,
so the browser detector fixture can independently prove bitmap/depth pairing.
"""
import json
from pathlib import Path
import sys
import threading
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src')]
from online_control.online_motion_track.main import make_server
from online_control.online_motion_track.realsense_source import encode_frame
from test_realsense_application import StoredCamera, CAMERA


class BrowserCamera(StoredCamera):
    def __init__(self):
        super().__init__()
        self.frames = {}
        self.lock = threading.RLock()
        self.delivered = {}
        self.closed_count = 0

    def create(self, token):
        with self.lock:
            super().create(token)
            self.delivered.setdefault(token, set())

    def close(self, token):
        with self.lock:
            if token in self.sessions:
                self.closed_count += 1
            super().close(token)
            self.delivered.pop(token, None)

    def frame(self, token, after_revision=-1):
        # Native long-poll is driven by capture. Keep the synthetic source paced
        # independently from the deliberately slower worker used by the smoke.
        time.sleep(.04)
        with self.lock:
            if token not in self.sessions:
                raise ValueError('test capture closed')
            self.sequence += 1
            frame_id = self.sequence
            at = time.monotonic()
            self.frames[frame_id] = at
            self.delivered[token].add(frame_id)
            self.frames = {key: value for key, value in self.frames.items() if at-value < 2}
            rgb = np.full((480, 640, 3), [29, 53, 65], dtype=np.uint8)
            # Distinct marker channel order also detects an RGB/BGR conversion.
            rgb[0, 0] = [frame_id & 255, (frame_id >> 8) & 255, (frame_id >> 16) & 255]
            for u, v, _ in self.normalized:
                x, y = int(u*640), int(v*480)
                rgb[max(0,y-5):y+6, max(0,x-5):x+6] = [67, 207, 199]
            return encode_frame(dict(has_frame=True, format='rgb8', width=640, height=480,
                frame_id=frame_id, stream_id='capture-stream', revision=frame_id,
                capture_unix_ms=time.time()*1000, timestamp_ms=3000+frame_id,
                age_ms=0, stale_after_ms=500, capture_fps=25, connected=True), rgb.tobytes())

    def consume(self, token, frame_id, stream_id):
        with self.lock:
            if (self.expired or token not in self.sessions or stream_id != 'capture-stream'
                    or frame_id not in self.delivered[token] or (token, frame_id) in self.consumed
                    or time.monotonic()-self.frames.get(frame_id, float('-inf')) >= .5):
                raise ValueError('expired, duplicate or mismatched capture')
            self.consumed.add((token, frame_id))
            return dict(depth_m=self.depth, intrinsics=CAMERA, frame_id=frame_id,
                stream_id=stream_id, timestamp_ms=3000+frame_id, capture_unix_ms=time.time()*1000)


def main():
    server = make_server(0, public=True)
    camera = BrowserCamera()
    server.application.realsense = camera
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print('RGBD_FIXTURE_DATA='+json.dumps(dict(world=camera.prior.tolist(), normalized=camera.normalized.tolist())), flush=True)
    print(f'RGBD_TEST_URL=http://127.0.0.1:{server.server_port}', flush=True)
    try:
        for command in sys.stdin:
            command = command.strip()
            if command == 'quit':
                break
            if command in ('pause', 'resume'):
                camera.expired = command == 'pause'
            if command == 'state':
                print('RGBD_STATE='+json.dumps(dict(sessions=len(camera.sessions), closed=camera.closed_count,
                    consumed=len(camera.consumed), sequence=camera.sequence)), flush=True)
    finally:
        server.shutdown()
        server.server_close()
        server.application.close()
        thread.join(timeout=5)


if __name__ == '__main__':
    main()
