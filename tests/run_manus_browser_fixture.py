"""Customer Manus Python SDK contract, real relay/solver/UI; all hardware simulated."""
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'src')]
from a32_driver import A32Device
from hand_profiles import load_hand_profile
from online_control.online_motion_track import main as web
from online_control.online_motion_track.a32 import session as session_module
from online_control.online_motion_track.a32.control import A32Control
from test_a32_driver import SimulatedNativeHand
from test_manus_input import FakeGlove


def simulated_control(profile, side, mode, **options):
    return A32Control(profile, side, mode, device=A32Device(side, profile.model_id,
        backend=SimulatedNativeHand()), **options)


def main():
    session_module.A32Control = simulated_control
    glove = FakeGlove(time.monotonic)
    server = web.make_server(0, manus_options=dict(sdk_loader=lambda:
        SimpleNamespace(ManusGlove=lambda:glove), retry_delay=.01))
    profile = load_hand_profile('TsenseI_v3_2_withid')
    points = {side:profile.create_retargeter(side).fk.compute() for side in ('left', 'right')}
    stopped, paused = threading.Event(), threading.Event()
    def publish():
        sequence = 0
        while not stopped.wait(.025):
            if not paused.is_set():
                sequence += 1
                for side in points:
                    glove.publish(points[side], side=side, sequence=sequence,
                                  glove_id=42 if side=='left' else 43)
    producer = threading.Thread(target=publish, daemon=True)
    service = threading.Thread(target=server.serve_forever, daemon=True)
    producer.start()
    service.start()
    print(f'MANUS_TEST_URL=http://127.0.0.1:{server.server_port}', flush=True)
    try:
        for line in sys.stdin:
            command = line.strip()
            if command == 'quit':
                break
            if command == 'pause':
                paused.set()
            if command == 'resume':
                paused.clear()
    finally:
        stopped.set()
        server.shutdown()
        server.server_close()
        server.application.close()
        producer.join(2)
        service.join(2)


if __name__ == '__main__':
    main()
