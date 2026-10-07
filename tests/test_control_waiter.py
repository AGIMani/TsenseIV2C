import sys
import threading
import time
from pathlib import Path
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'))
from utils import control_clock


def test_unsupported_platform_uses_interruptible_event(monkeypatch):
    monkeypatch.setattr(control_clock.sys, 'platform', 'unsupported')
    done = Mock()
    with control_clock.control_waiter(done) as wait:
        wait(.005)
    done.wait.assert_called_once_with(.005)


def test_real_wait_releases_on_timer_and_observes_shutdown():
    done = threading.Event()
    with control_clock.control_waiter(done) as wait:
        start=time.perf_counter()
        assert not wait(.003)
        assert time.perf_counter()-start>=.002
        done.set()
        assert wait(.1)


def test_failed_timer_arm_falls_back_and_closes_handle(monkeypatch):
    monkeypatch.setattr(control_clock.sys,'platform','win32')
    api=Mock()
    api.CreateWaitableTimerExW.return_value=123
    api.SetWaitableTimer.return_value=0
    monkeypatch.setattr(control_clock.ctypes,'WinDLL',lambda *a,**k:api,raising=False)
    done=Mock()
    done.is_set.return_value=False
    with control_clock.control_waiter(done) as wait:
        wait(.005)
    done.wait.assert_called_once_with(.005)
    api.CloseHandle.assert_called_once_with(123)
