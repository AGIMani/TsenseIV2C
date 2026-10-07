"""Fetch the newest SDK frame off the browser request path, with bounded age."""
from copy import deepcopy
from http.client import HTTPConnection, HTTPSConnection
import json
import threading
import time
from urllib.parse import urlsplit
from .wuji_sdk_source import SkeletonStore, acquire, cleanup_resources


class LatestWujiSdkClient:
    """One local SDK worker, shared by all browser input sessions and both hands."""
    def __init__(self, address=None, serial=None, *, clock=time.perf_counter,
                 sdk_loader=None, close_timeout=1.5, **acquire_options):
        self.address, self.serial, self.clock = address, serial, clock
        self.sdk_loader, self.acquire_options = sdk_loader, acquire_options
        self.close_timeout = close_timeout
        self.lock = threading.Lock()
        self.worker = None
        self.on_change = None

    def _run(self, worker):
        try:
            acquire(worker['store'], worker['stop'], self.address, self.serial,
                    sdk_loader=self.sdk_loader, **self.acquire_options)
        except Exception as exc:
            worker['store'].clear('sdk_error', str(exc), invalidate=True)

    def latest(self, side):
        if side not in ('left', 'right'):
            raise ValueError('Wuji 手别无效')
        with self.lock:
            worker = self.worker
            # A cancelled native scan/connect may still be finishing. Keep its
            # owner until it exits; never overlap it with another worker.
            if worker is None or (worker['stop'].is_set() and not worker['thread'].is_alive()
                                  and not worker['store'].pending_cleanup):
                worker = dict(stop=threading.Event(), store=SkeletonStore(self.clock,
                    on_change=lambda: self.on_change() if self.on_change else None))
                worker['thread'] = threading.Thread(target=self._run, args=(worker,),
                                                     daemon=True, name='wuji-sdk-input')
                self.worker = worker
                worker['thread'].start()
            result = worker['store'].latest(side)
            if worker['stop'].is_set() and not worker['store'].pending_cleanup:
                result.update(frame=None, age_ms=None, connected=False, state='stopping')
            result['transport'].update(address=self.address, serial=self.serial)
            return result

    def commit_if_current(self, side, snapshot, action):
        with self.lock:
            worker = self.worker
            if worker is None or worker['stop'].is_set():
                return False
            store = worker['store']
            with store.lock:
                raw = snapshot.get('raw_wuji')
                item = store.frames.get(side)
                expected = snapshot.get('source_metadata', {}).get('transport', {}).get('generation')
                if expected is not None and expected != store.generation:
                    return False
                epoch = snapshot.get('source_metadata', {}).get('transport', {}).get('source_epoch')
                if epoch is not None and epoch != store.source_epoch:
                    return False
                if snapshot.get('hand') is not None:
                    if item is None or raw is None or item[0]['stream_id'] != raw['stream_id']:
                        return False
                elif item is not None and (self.clock()-item[1])*1000 < 350:
                    return False
                action()
                return True

    def close(self, *, wait=True):
        with self.lock:
            worker = self.worker
            if worker is None:
                return
            worker['stop'].set()
            if worker['store'].pending_cleanup and not worker['thread'].is_alive():
                worker['thread'] = threading.Thread(target=self._cleanup, args=(worker,),
                                                     daemon=True, name='wuji-sdk-cleanup')
                worker['thread'].start()
            elif not worker['store'].pending_cleanup:
                worker['store'].clear('stopped', invalidate=True)
        if wait:
            worker['thread'].join(timeout=self.close_timeout)

    def _cleanup(self, worker):
        if cleanup_resources(worker['store']):
            worker['store'].clear('stopped', invalidate=True)


class LatestWujiClient:
    def __init__(self, url, *, clock=time.perf_counter, interval=1 / 120, timeout=.25):
        self.url, self.clock = url, clock
        self.interval, self.timeout = interval, timeout
        self.lock = threading.Lock()
        self.streams = {}

    def _request(self, connection, side):
        path = urlsplit(self.url).path.rstrip('/') + '/latest?side=' + side
        connection.request('GET', path, headers={'Accept': 'application/json'})
        response = connection.getresponse()
        if response.status != 200:
            raise ValueError(f'Wuji HTTP {response.status}')
        data = response.read(65537)
        if len(data) > 65536:
            raise ValueError('Wuji 数据包过大')
        result = json.loads(data)
        if not isinstance(result, dict):
            raise ValueError('Wuji 数据包无效')
        return result

    def _run(self, side, state):
        parsed = urlsplit(self.url)
        factory = HTTPSConnection if parsed.scheme == 'https' else HTTPConnection
        connection = factory(parsed.hostname, parsed.port, timeout=self.timeout)
        try:
            while not state['stop'].is_set():
                began = self.clock()
                try:
                    result = self._request(connection, side)
                    received = self.clock()
                    rtt = (received - began) * 1000
                    with self.lock:
                        state.update(response=result, received=received, request_ms=rtt,
                                     error=None, requests=state['requests'] + 1)
                except Exception as exc:
                    # A missed poll does not make a still-fresh observation disappear.
                    # Never reset its reception time or age when the request fails.
                    connection.close()
                    with self.lock:
                        state.update(error=str(exc), errors=state['errors'] + 1)
                state['stop'].wait(max(.002, self.interval - (self.clock() - began)))
        finally:
            connection.close()

    def latest(self, side):
        with self.lock:
            state = self.streams.get(side)
            if state is None:
                state = dict(stop=threading.Event(), response=None, received=None,
                             request_ms=None, error=None, errors=0, requests=0)
                state['thread'] = threading.Thread(target=self._run, args=(side, state), daemon=True,
                                                   name=f'wuji-input-{side}')
                self.streams[side] = state
                state['thread'].start()
            result = deepcopy(state['response'])
            now = self.clock()
            cache_ms = (now - state['received']) * 1000 if state['received'] is not None else None
            transport = dict(request_ms=state['request_ms'], cache_ms=cache_ms,
                             errors=state['errors'], requests=state['requests'], last_error=state['error'])
        if result is None:
            return dict(schema='tsensei.wuji_glove.skeleton.v1', connected=False, frame=None,
                        age_ms=None, state='transport_error' if transport['last_error'] else 'connecting',
                        error=transport['last_error'], transport=transport)
        bridge_age = result.get('age_ms')
        # Include the full network round trip as a conservative transport bound.
        # Local elapsed time ages this cache even when node0 is unavailable.
        if type(bridge_age) in (int, float):
            result['age_ms'] = bridge_age + transport['request_ms'] + cache_ms
        transport['bridge_age_ms'] = bridge_age
        result['transport'] = transport
        if transport['last_error'] and (result.get('age_ms') is None or result['age_ms'] >= 350):
            result.update(connected=False, state='transport_error', error=transport['last_error'])
        return result

    def close(self, *, wait=True):
        with self.lock:
            states = list(self.streams.values())
            self.streams.clear()
            for state in states:
                state['stop'].set()
        if wait:
            for state in states:
                state['thread'].join(timeout=self.timeout + .1)
