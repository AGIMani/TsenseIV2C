"""Scoped Windows timer precision for short, interruptible control waits."""
from contextlib import contextmanager
import ctypes
import sys


@contextmanager
def control_waiter(done):
    """A per-thread high-resolution wait, without changing motion step sizes.

    Windows 10 1803+ supports an explicit high-resolution waitable timer. Each
    wait is capped at 50 ms so shutdown is bounded even for a distant deadline.
    Unsupported systems/failing APIs retain the interruptible Event.wait path.
    """
    timer, kernel = None, None
    try:
        if sys.platform == 'win32':
            try:
                kernel = ctypes.WinDLL('kernel32', use_last_error=True)
                kernel.CreateWaitableTimerExW.argtypes = [ctypes.c_void_p, ctypes.c_wchar_p, ctypes.c_uint32, ctypes.c_uint32]
                kernel.CreateWaitableTimerExW.restype = ctypes.c_void_p
                kernel.SetWaitableTimer.argtypes = [ctypes.c_void_p, ctypes.POINTER(ctypes.c_int64), ctypes.c_long, ctypes.c_void_p, ctypes.c_void_p, ctypes.c_int]
                kernel.SetWaitableTimer.restype = ctypes.c_int
                kernel.WaitForSingleObject.argtypes = [ctypes.c_void_p, ctypes.c_uint32]
                kernel.WaitForSingleObject.restype = ctypes.c_uint32
                kernel.CloseHandle.argtypes = [ctypes.c_void_p]
                kernel.CloseHandle.restype = ctypes.c_int
                timer = kernel.CreateWaitableTimerExW(None, None, 2, 0x100002)
            except (OSError, AttributeError):
                timer = None

        def wait(delay):
            if timer is None:
                return done.wait(delay)
            if done.is_set() or delay <= 0:
                return done.is_set()
            remaining = delay
            while remaining > 0 and not done.is_set():
                chunk = min(remaining, .05)
                due = ctypes.c_int64(-max(1, int(chunk * 10_000_000)))
                if not kernel.SetWaitableTimer(timer, ctypes.byref(due), 0, None, None, False):
                    return done.wait(remaining)
                if kernel.WaitForSingleObject(timer, int(chunk * 1000) + 100) != 0:
                    return done.wait(remaining)
                remaining -= chunk
            return done.is_set()
        yield wait
    finally:
        if timer:
            kernel.CloseHandle(timer)


@contextmanager
def precise_control_timer():
    """Improve Event.wait resolution, always releasing the process request.

    This is only a scheduling aid. Trajectories must still use measured time;
    the OS is not a real-time scheduler and can miss a requested deadline.
    """
    winmm = None
    enabled = False
    try:
        if sys.platform == "win32":
            try:
                winmm = ctypes.WinDLL("winmm")
                enabled = winmm.timeBeginPeriod(1) == 0
            except (OSError, AttributeError):
                pass
        yield enabled
    finally:
        if enabled:
            winmm.timeEndPeriod(1)
