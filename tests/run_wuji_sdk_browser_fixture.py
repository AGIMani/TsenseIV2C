"""Fake SDK callbacks -> real Linux solver and HTTP UI; no device I/O."""
from pathlib import Path
import json
import os
import sys
import threading
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'src'), str(ROOT/'src/tracking')]
from online_control.online_motion_track.main import make_server
from online_control.online_motion_track.wuji_transport import LatestWujiClient
from a32_driver.native import NativeHand
from hand_profiles import load_hand_profile


def main():
    forbidden_attempts = []
    def forbidden(*args, **kwargs):
        forbidden_attempts.append('robot_connection_or_http_bridge')
        raise AssertionError('offline fixture forbids real hardware and HTTP glove bridge')
    NativeHand.connect_usb = forbidden
    NativeHand.connect_eth = forbidden
    LatestWujiClient.__init__ = forbidden

    class Glove:
        def __init__(self, side):
            self.side = side
            self.serial_number = 'OFFLINE_SIMULATION_' + side
            self.callback = None
            self.is_connected = True
        def hand_side(self):
            return SimpleNamespace(get=lambda: self.side)
        def hand_skeleton(self):
            def subscribe(callback):
                self.callback = callback
                return SimpleNamespace(close=lambda: setattr(self, 'callback', None))
            return SimpleNamespace(subscribe_with_callback=subscribe)

    class Manager:
        def __init__(self):
            self.gloves = {}
        def scan(self):
            return [SimpleNamespace(sn=side, device_type='glove') for side in ('left','right')]
        def connect(self, *, sn, device_name, options):
            assert sn in ('left', 'right') and not options.enable_bridge
            glove = Glove(sn)
            self.gloves[device_name] = glove
            return glove
        def disconnect(self, *, device_name):
            glove = self.gloves.pop(device_name)
            glove.is_connected = False
        def sdk(self):
            return SimpleNamespace(SdkManager=SimpleNamespace(instance=lambda: self),
                DeviceType=SimpleNamespace(WujiGlove='glove'), ConnectOptions=lambda **kw: SimpleNamespace(**kw))

    manager = Manager()
    app = make_server(0, wuji_source='sdk')
    app.application.wuji.client.sdk_loader = manager.sdk
    stopped, paused = threading.Event(), threading.Event()
    profile = load_hand_profile('TsenseI_v3_2')
    points = {side: profile.create_retargeter(side).fk.compute().tolist() for side in ('left','right')}
    state = {'emitted': 0}
    def publish():
        sequence = 0
        while not stopped.wait(1/30):
            if paused.is_set():
                continue
            sequence += 1
            for glove in list(manager.gloves.values()):
                callback = glove.callback
                if callback is None or not glove.is_connected:
                    continue
                frame = SimpleNamespace(header=SimpleNamespace(seq=sequence,
                    timestamp_us=sequence*33333, frame_id='l_wrist' if glove.side=='left' else 'r_wrist'),
                    joints=[SimpleNamespace(pose=SimpleNamespace(position=p),confidence=1.) for p in points[glove.side]])
                callback(frame)
                state['emitted'] += 1
    threading.Thread(target=app.serve_forever, daemon=True).start()
    threading.Thread(target=publish, daemon=True).start()
    print('WUJI_TEST_URL=http://127.0.0.1:' + str(app.server_port), flush=True)
    try:
        for command in sys.stdin:
            command = command.strip()
            if command == 'pause':
                paused.set()
            elif command == 'resume':
                paused.clear()
            elif command == 'inspect':
                workers = list(app.application.metric_workers.values())
                result = dict(state, forbidden_attempts=list(forbidden_attempts),
                    workers=[dict(update_revision=w.update_revision, client_age=time.monotonic()-w.last_client,
                        cancelled=w.cancelled, armed=w.session.control.armed,
                        connected=w.session.control.device.is_connected) for w in workers])
                print('WUJI_TEST_STATE=' + json.dumps(result), flush=True)
            elif command == 'quit':
                break
    finally:
        stopped.set()
        app.shutdown()
        app.server_close()
        app.application.close()
        assert not forbidden_attempts, forbidden_attempts
        deadline = time.monotonic() + 2
        while manager.gloves and time.monotonic() < deadline:
            time.sleep(.01)
        assert not manager.gloves, 'fake SDK owner was not closed'
        print('WUJI_TEST_SAFETY=passed', flush=True)


if __name__ == '__main__':
    main()
