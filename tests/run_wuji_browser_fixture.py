"""Loopback-only synthetic bridge + real app for browser integration tests.

This fixture is never deployed to node0. stdin commands pause/resume/quit control
synthetic input, allowing the browser test to verify actual transport expiry.
"""
from pathlib import Path
import json
import os
import sys
import threading
import time
from types import SimpleNamespace


ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src"), str(ROOT / "src/tracking"), str(ROOT / "tools")]
from wuji_glove_bridge import SkeletonStore, make_server as bridge_server
from online_control.online_motion_track.main import make_server
from hand_profiles import load_hand_profile
from a32_driver.native import NativeHand

def main():
    def forbidden_robot_connection(*args, **kwargs):
        raise AssertionError('synthetic browser fixture must not open robot hardware')
    NativeHand.connect_usb = forbidden_robot_connection
    NativeHand.connect_eth = forbidden_robot_connection
    store = SkeletonStore()
    bridge = bridge_server(store, "127.0.0.1", 0)
    os.environ["WUJI_BRIDGE_URL"] = f"http://127.0.0.1:{bridge.server_port}"
    app = make_server(0)
    stopped, paused = threading.Event(), threading.Event()
    profile = load_hand_profile("TsenseI_v3_2")
    points = {side: profile.create_retargeter(side).fk.compute() for side in ("left", "right")}


    def publish():
        seq = 0
        hz = float(os.environ.get('WUJI_TEST_HZ', '30'))
        while not stopped.wait(1 / hz):
            if paused.is_set():
                continue
            seq += 1
            for side, pts in points.items():
                sample = SimpleNamespace(header=SimpleNamespace(seq=seq, timestamp_us=seq * 33333,
                    frame_id="l_wrist" if side == "left" else "r_wrist"),
                    joints=[SimpleNamespace(pose=SimpleNamespace(position=p), confidence=1.) for p in pts])
                store.offer(sample, side=side, serial="SIMULATION_ONLY", stream_id="browser-test-" + side)


    for target in (bridge.serve_forever, app.serve_forever, publish):
        threading.Thread(target=target, daemon=True).start()
    print("WUJI_TEST_URL=" + f"http://127.0.0.1:{app.server_port}", flush=True)
    try:
        for command in sys.stdin:
            if command.strip() == "pause":
                paused.set()
            elif command.strip() == "resume":
                paused.clear()
            elif command.strip() == "quit":
                break
    finally:
        stopped.set()
        app.shutdown()
        app.server_close()
        app.application.close()
        bridge.shutdown()
        bridge.server_close()


if __name__ == '__main__':
    main()
