"""One bounded RPC at a time to an input solver that never owns robot hardware."""
import os
from pathlib import Path
import pickle
import queue
import struct
import subprocess
import sys
import threading
import time


class _FramedPipe:
    """Private inherited pipes; at most one response is outstanding."""
    def __init__(self, reader, writer):
        self.reader, self.writer = reader, writer

    def send(self, value):
        data = pickle.dumps(value, protocol=5)
        if len(data) > 8*1024*1024:
            raise ValueError('Solver message too large')
        self.writer.write(struct.pack('<I', len(data)) + data)
        self.writer.flush()

    def recv(self):
        def read(size):
            parts = bytearray()
            while len(parts) < size:
                chunk = self.reader.read(size-len(parts))
                if not chunk:
                    raise EOFError('Solver pipe closed')
                parts.extend(chunk)
            return parts
        size, = struct.unpack('<I', read(4))
        if size > 8*1024*1024:
            raise ValueError('Solver message too large')
        return pickle.loads(read(size))

    def close(self):
        for stream in (self.writer, self.reader):
            try:
                stream.close()
            except (OSError, ValueError):
                pass


class _ResponsePipe(_FramedPipe):
    def __init__(self, reader, writer):
        super().__init__(reader, writer)
        self.responses = queue.Queue(maxsize=1)
        self.pending = None
        self.pump = threading.Thread(target=self._pump, daemon=True)
        self.pump.start()

    def _pump(self):
        try:
            while True:
                self.responses.put(super().recv())
        except (EOFError, OSError, ValueError) as exc:
            try:
                self.responses.put_nowait(exc)
            except queue.Full:
                pass

    def poll(self, timeout):
        if self.pending is not None:
            return True
        try:
            self.pending = self.responses.get(timeout=timeout)
            return True
        except queue.Empty:
            return False

    def recv(self):
        value, self.pending = self.pending, None
        if isinstance(value, Exception):
            raise value
        return value


class _ProcessHandle:
    def __init__(self, process):
        self.process, self.pid = process, process.pid
    def is_alive(self):
        return self.process.poll() is None
    def terminate(self):
        self.process.terminate()
    def join(self, timeout):
        try:
            self.process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            pass


def _worker(connection, options):
    from online_control.online_motion_track.main import WebSession
    session = None
    try:
        session = WebSession(recording=False, **options)
        connection.send(('ok', None))
        while True:
            action, value = connection.recv()
            if action == 'close':
                break
            try:
                if action == 'process':
                    result = session.process(value)
                    result['direct_mode'] = bool(getattr(session.pipeline, 'collision_bypass', False))
                elif action == 'set_direct_mode':
                    if not isinstance(value, bool):
                        raise ValueError('enabled 必须为布尔值')
                    session.pipeline.collision_bypass = value
                    session.direct_mode = value
                    session.pipeline.session.retargeter.reset()
                    session.filter.reset()
                    session.normalized_filter.reset()
                    result = None
                elif action == 'reset':
                    session.last_sequence = session.last_timestamp = -1
                    session.filter.reset()
                    session.normalized_filter.reset()
                    session.pipeline.session.retargeter.reset()
                    result = None
                elif action == 'seed':
                    guard = None if getattr(session.pipeline, 'collision_bypass', False) else session.pipeline.mink_guard
                    if guard and hasattr(guard, 'q'):
                        q = guard._vector(value)
                        if not guard._feasible(q):
                            raise ValueError('反馈姿态不满足 Mink 最小间距')
                        guard.q = q.copy()
                    elif guard and hasattr(guard, 'current'):
                        guard.current = dict(value)
                        guard.guard.rebase(value)
                    session.pipeline.session.set_pose(value)
                    session.pipeline.session.retargeter.reset()
                    session.filter.reset()
                    session.normalized_filter.reset()
                    result = None
                else:
                    raise ValueError('Unknown solver operation')
                connection.send(('ok', result))
            except Exception as exc:
                connection.send(('error', (type(exc).__name__, str(exc))))
    except (EOFError, BrokenPipeError):
        pass
    except Exception as exc:
        try:
            connection.send(('error', (type(exc).__name__, str(exc))))
        except (OSError, EOFError):
            pass
    finally:
        if session is not None:
            session.close()
        connection.close()


class SolverProcess:
    def __init__(self, **options):
        self.lock = threading.Lock()
        self.closed = False
        process = subprocess.Popen([sys.executable, '-u', '-X', 'utf8', '-m',
            'online_control.online_motion_track.a32.solver_process'],
            cwd=Path(__file__).resolve().parents[3], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            creationflags=subprocess.CREATE_NO_WINDOW if os.name == 'nt' else 0)
        self.process_handle = _ProcessHandle(process)
        self.connection = _ResponsePipe(process.stdout, process.stdin)
        try:
            self.connection.send(options)
            self._receive(30.)
        except Exception:
            self.close()
            raise

    def _receive(self, timeout):
        if not self.connection.poll(timeout):
            raise TimeoutError('骨架求解进程超时，停止下发后请重新开始识别')
        kind, value = self.connection.recv()
        if kind == 'error':
            name, message = value
            if name == 'StaleFrameError':
                from online_control.online_motion_track.main import StaleFrameError
                raise StaleFrameError(message)
            raise ValueError(f'{name}: {message}')
        return value

    def call(self, action, value=None):
        with self.lock:
            if self.closed or not self.process_handle.is_alive():
                raise RuntimeError('骨架求解进程已停止，请重新开始识别')
            try:
                self.connection.send((action, value))
                return self._receive(1.5)
            except (TimeoutError, EOFError, OSError):
                # No subsequent request may consume a timed-out request's reply.
                self.closed = True
                self.process_handle.terminate()
                self.process_handle.join(timeout=2.)
                raise

    def process(self, packet):
        began = time.perf_counter()
        result = self.call('process', packet)
        result['solver_process_roundtrip_ms'] = (time.perf_counter()-began)*1000
        return result

    def close(self):
        with self.lock:
            if not self.closed:
                self.closed = True
                try:
                    self.connection.send(('close', None))
                except (OSError, EOFError):
                    pass
            self.process_handle.join(timeout=2.)
            if self.process_handle.is_alive():
                self.process_handle.terminate()
                self.process_handle.join(timeout=2.)
            self.connection.close()


if __name__ == '__main__':
    pipe = _FramedPipe(sys.stdin.buffer, sys.stdout.buffer)
    # Library status output must never enter the framed response channel.
    sys.stdout = sys.stderr
    _worker(pipe, pipe.recv())
