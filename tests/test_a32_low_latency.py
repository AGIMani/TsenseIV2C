"""Control cadence, solver isolation and actual native ABI; no real hardware."""
import ctypes
import os
from pathlib import Path
import sys
import threading
from unittest.mock import Mock

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'src')]
from a32_driver.native import NativeHand, A32DriverError
from online_control.online_motion_track.a32.solver_process import SolverProcess
from online_control.online_motion_track.main import StaleFrameError
from test_a32_tracking_control import control


def test_elapsed_time_is_preserved_inside_4ms_and_large_stalls_stay_bounded():
    item, device = control()
    try:
        item.arm(True)
        item.push({**item.pose, 'left_hand_index_PIP_joint': .6}, 1)
        for dt in (.002, .003, .004, .05):
            before = dict(item.pose)
            item.step(dt)
            assert item.armed, item.last_error
            assert item.control_timing['planner_dt_ms'] == pytest.approx(min(dt, .004)*1000)
            for i, name in enumerate(item.names):
                assert abs(item.pose[name]-before[name]) <= item.input.max_velocity[i]*min(dt, .004)+2e-6
    finally:
        item.close()


def test_invalid_large_candidate_is_never_written():
    item, device = control()
    try:
        item.arm(True)
        item._checked_step = lambda dt: {**item.pose, 'left_hand_index_PIP_joint': .8}
        item.step(.002)
        assert not item.armed and not device.writes
        assert '步进超限' in item.last_error
    finally:
        item.close()


def test_raw_recording_does_not_wait_for_control_lock():
    item, _ = control()
    result = threading.Event()
    try:
        with item.lock:
            worker = threading.Thread(target=lambda: (item.record_event('test'), result.set()))
            worker.start()
            assert result.wait(.5)
        worker.join()
    finally:
        item.close()


def test_solver_runs_without_hardware_and_rejects_duplicates_and_dead_worker():
    solver = SolverProcess(side='left', collision_mode='mink', model_id='TsenseI_v3_2_withid', input_source='wuji_glove')
    try:
        assert solver.process_handle.pid != os.getpid()
        packet = dict(sequence=1, timestamp_ms=1., capture_unix_ms=1., source='wuji_glove', hand=None)
        result = solver.process(packet)
        assert result['sequence'] == 1 and not result['tracking']
        with pytest.raises(StaleFrameError):
            solver.process(packet)
        solver.call('reset')
        assert solver.process(packet)['sequence'] == 1
        solver.process_handle.terminate()
        solver.process_handle.join(timeout=3)
        with pytest.raises(RuntimeError, match='已停止'):
            solver.process(packet)
    finally:
        solver.close()
    assert not solver.process_handle.is_alive()


def test_solver_timeout_discards_late_response_and_closes_worker():
    solver = SolverProcess.__new__(SolverProcess)
    solver.lock, solver.closed = threading.Lock(), False
    solver.connection = Mock()
    solver.connection.poll.return_value = False
    solver.process_handle = Mock()
    with pytest.raises(TimeoutError):
        solver.call('process', {})
    assert solver.closed
    solver.process_handle.terminate.assert_called_once()
    with pytest.raises(RuntimeError):
        solver.call('process', {})


@pytest.mark.skipif(sys.platform != 'win32', reason='Windows SDK')
def test_compiled_fast_control_and_send_timestamps_with_test_transport():
    dll = ROOT/'.work/a32-native-lowlatency/test-bin/a32_native_test.dll'
    if not dll.exists():
        pytest.skip('Build with A32_BUILD_TEST_TRANSPORT=ON')
    hand = NativeHand(dll)
    try:
        assert hand.fast_control
        assert not hand.get_control_state()['connected']
        hand._dll.a32_test_attach.argtypes = [ctypes.c_void_p, ctypes.c_int]
        hand._dll.a32_test_attach.restype = ctypes.c_int
        hand._call('a32_test_attach', 0)
        assert hand.get_control_state()['connected']
        first = hand.send_targets([0.]*18, [20., 60.])
        second = hand.send_targets([0.]*18, [20.001, 60.001])
        assert second['sequence'] == first['sequence']+1
        assert second['started_ns'] > first['started_ns']
        assert second['finished_ns'] >= second['started_ns']
        assert second['interval_ms'] == pytest.approx((second['started_ns']-first['started_ns'])/1e6)
        assert second['interval_ms'] >= 1.
        assert hand.get_state()['targets']['mit_pos'] == pytest.approx([20.001, 60.001])
        with pytest.raises(A32DriverError):
            hand.send_targets([float('nan')]*18, [0., 0.])
        assert hand.get_state()['diagnostics']['send_successes'] == 2
    finally:
        hand.close()
