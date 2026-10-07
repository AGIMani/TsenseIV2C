"""Offline replay using the real SDK and a compiled TEST_ONLY transport."""
import argparse
import ctypes
import json
from pathlib import Path
import sys
import time
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'src')]


def main():
    from a32_driver import A32Device
    from a32_driver.native import NativeHand
    from online_control.online_motion_track.a32 import session as module
    from online_control.online_motion_track.a32.control import A32Control
    from analyze_a32_packet_timing import analyze
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=12.)
    parser.add_argument('--replay', type=Path)
    parser.add_argument('--inline', action='store_true')
    args = parser.parse_args()
    class TestNative(NativeHand):
        def __init__(self):
            super().__init__(ROOT/'.work/a32-native-lowlatency/test-bin/a32_native_test.dll')
            self._dll.a32_test_attach.argtypes = [ctypes.c_void_p, ctypes.c_int]
            self._dll.a32_test_attach.restype = ctypes.c_int
        def connect_usb(self, port='SIM_ONLY'):
            if port != 'SIM_ONLY':
                raise ValueError('Benchmark accepts only TEST_ONLY transport')
            self._call('a32_test_attach', 170)
        def connect_eth(self, *args):
            raise ValueError('No hardware/network connections in benchmark')
    folder = ROOT/'.work'/('a32-2ms-bench-'+time.strftime('%Y%m%d-%H%M%S'))
    backend = TestNative()
    device = A32Device(model_id='TsenseI_v3_2_withid', backend=backend)
    def factory(profile, side, mode, **kw):
        return A32Control(profile, side, mode, device=device, **kw)
    with patch.object(module, 'A32Control', factory):
        session = module.A32WebSession(model_id='TsenseI_v3_2_withid', collision_mode='mink',
                                      input_source='wuji_glove', log_dir=folder)
    try:
        if args.inline:
            session.solver.close()
            session.solver = None
        frames = []
        if args.replay:
            for line in args.replay.open(encoding='utf8'):
                row = json.loads(line)
                if row.get('type') == 'wuji_glove_raw' and row.get('world_landmarks'):
                    frames.append(row['world_landmarks'][0])
        if not frames:
            frames = [session.pipeline.session.retargeter.fk.compute().tolist()]
        session.command('connect', {'port': 'SIM_ONLY'})
        session.command('tracking_start', {})
        session.command('arm', {'enabled': True})
        began = time.perf_counter()
        seq = 0
        while time.perf_counter()-began < args.seconds:
            # Replay gestures near the original 30 Hz input cadence; the
            # independent controller must keep sending between solved frames.
            world = frames[min(int((time.perf_counter()-began)*30), len(frames)-1)]
            session.process(dict(sequence=seq, timestamp_ms=time.perf_counter()*1000,
                capture_unix_ms=time.time()*1000, source='wuji_glove',
                hand=dict(label='Left', world=world)))
            seq += 1
        active, error = session.control.armed, session.control.last_error
        path = session.control.recorder.writer.path
        session.command('disconnect', {})
        result = analyze(path)
        result.update(transport='TEST_ONLY_NO_HARDWARE', simulated_send_cost_us=170,
                      isolated_solver=not args.inline, solved_frames=seq,
                      active_at_end=active, control_error=error)
        output = ROOT/'logs/wuji_validation'/('a32-2ms-inline.json' if args.inline else 'a32-2ms-isolated.json')
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf8')
        output.with_name(output.stem+'-'+time.strftime('%Y%m%d-%H%M%S')+'.json').write_text(
            json.dumps(result, ensure_ascii=False, indent=2), encoding='utf8')
        print(json.dumps({k: result[k] for k in ('transport', 'isolated_solver', 'solved_frames',
            'active_at_end', 'control_error', 'cadence', 'native_cadence', 'trajectory_progress')}, ensure_ascii=False))
        print(json.dumps({k: v for k, v in result['timing'].items()
                         if k in ('native_write_interval_ms', 'encode_write_ms', 'lock_wait_ms', 'solve_ms')}, ensure_ascii=False))
    finally:
        session.close()


if __name__ == '__main__':
    main()
