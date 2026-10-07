"""One latest-only metric input consumer per A3.2 session, independent of UI polling."""
from copy import deepcopy
import threading
import time


class SourceWorker:
    CLIENT_TIMEOUT = 2.

    def __init__(self, session_id, session, relay, source, *, clock=time.monotonic):
        self.session_id, self.session, self.relay, self.source = session_id, session, relay, source
        self.clock = clock
        self.lock = threading.RLock()
        self.wake = threading.Event()
        self.cancelled = False
        self.last_client = clock()
        self.thread = None
        self.unsubscribe = None
        self.update_revision = 0
        self.result_at = clock()
        self.result = dict(source=source, revision=0, update_revision=0, update=None,
            hand=None, age_ms=None, connected=False, tracking=False, background_processing=True,
            combined_frame=True, source_metadata=dict(source=source, event='waiting', tracking=False))

    def touch_client(self):
        with self.lock:
            if self.cancelled:
                raise ValueError('输入会话已停止')
            self.last_client = self.clock()
            self.session.last_seen = self.last_client
            self.session.control.touch()
            if self.thread is None:
                self.unsubscribe = self.relay.subscribe(self.session_id, self.wake.set)
                self.thread = threading.Thread(target=self._run, daemon=True,
                    name=f'{self.source}-solver-{self.session_id[:8]}')
                self.thread.start()
            self.wake.set()

    def cancel(self):
        with self.lock:
            self.cancelled = True
            self.wake.set()
        if self.unsubscribe:
            self.unsubscribe()

    def close(self):
        self.cancel()
        if self.thread and self.thread is not threading.current_thread():
            self.thread.join(timeout=2.)

    def _commit(self, snapshot, observed_at, action):
        # Source publication uses the relay's source lock. Keep that lock through
        # the short target update, as well as this lease/cancel lock, to exclude
        # cancellation or stream replacement between checking and committing.
        with self.lock:
            if self.cancelled or self.clock()-self.last_client >= self.CLIENT_TIMEOUT:
                return False
            age = snapshot.get('age_ms')
            if snapshot.get('hand') is not None and (age is None or
                    age+(self.clock()-observed_at)*1000 >= self.relay.stale_after_ms):
                return False
            return self.relay.commit_if_current(self.session_id, snapshot, action)

    def _publish_result(self, snapshot, update, observed_at):
        with self.lock:
            if self.cancelled:
                return
            self.update_revision += 1
            result = deepcopy(snapshot)
            age = result.get('age_ms')
            if age is not None:
                result['age_ms'] = age+(self.clock()-observed_at)*1000
            if update.get('background_discarded') or not update.get('tracking'):
                result.update(hand=None, tracking=False, connected=False)
            result.update(update=update, update_revision=self.update_revision,
                          combined_frame=True, background_processing=True)
            self.result, self.result_at = result, self.clock()

    def read(self, after_revision=None):
        with self.lock:
            acknowledged = type(after_revision) is int and after_revision == self.update_revision
            result = deepcopy({**self.result, 'update': None} if acknowledged else self.result)
            age = result.get('age_ms')
            if age is not None:
                age += (self.clock()-self.result_at)*1000
                result['age_ms'] = age
                result.setdefault('source_metadata', {})['age_ms'] = age
            if age is None or age >= self.relay.stale_after_ms:
                result.update(hand=None, connected=False, tracking=False)
                result.setdefault('source_metadata', {}).update(tracking=False, stale=True)
                if result.get('update'):
                    if result['update'].get('tracking'):
                        result['update']['display_discarded'] = True
                    result['update']['tracking'] = False
            return result

    def _run(self):
        previous = None
        previous_epoch = None
        while True:
            with self.lock:
                if self.cancelled:
                    return
                lease_left = self.CLIENT_TIMEOUT-(self.clock()-self.last_client)
            if lease_left <= 0:
                self.wake.wait(.1)
                self.wake.clear()
                continue
            try:
                snapshot = self.relay.latest(self.session_id)
                observed_at = self.clock()
                hand = snapshot.get('hand')
                metadata = snapshot.get('source_metadata', {})
                epoch = (metadata.get('transport', {}).get('source_epoch')
                         if self.source == 'wuji_glove' else metadata.get('source_epoch'))
                key = (snapshot['revision'], hand is not None,
                       epoch,
                       metadata.get('event') if hand is None else None,
                       metadata.get('error') if hand is None else None)
                if key != previous:
                    previous = key
                    consumed = self.relay.consume(self.session_id, snapshot['revision'])
                    # Expiry/loss can retain the source revision already consumed
                    # for its valid frame. It still needs one explicit hold.
                    if hand is None:
                        consumed = snapshot
                    with self.session.lock:
                        with self.lock:
                            if self.cancelled:
                                return
                        if epoch is not None and epoch != previous_epoch:
                            # Only trusted relay state reaches this method; a
                            # browser frame cannot request algorithm resets.
                            self.session.reset_metric_source()
                            previous_epoch = epoch
                        sequence = getattr(self.session, '_metric_sequence', -1)+1
                        self.session._metric_sequence = sequence
                        packet = dict(session=self.session_id, sequence=sequence,
                            timestamp_ms=self.clock()*1000, capture_unix_ms=time.time()*1000,
                            source=self.source, hand=consumed.get('hand'), camera_size=[0, 0],
                            source_metadata=consumed.get('source_metadata'),
                            raw_wuji=consumed.get('raw_wuji'), raw_webxr=consumed.get('raw_webxr'),
                            raw_leapmotion=consumed.get('raw_leapmotion'))
                        update = self.session.process_background(packet,
                            lambda action: self._commit(snapshot, observed_at, action))
                    update['input_path'] = 'server_metric_worker'
                    update['source_age_at_solve_ms'] = snapshot.get('age_ms')
                    self._publish_result(snapshot, update, observed_at)
                age = snapshot.get('age_ms')
                stale_wait = max(.001, (self.relay.stale_after_ms-age)/1000) if hand is not None and age is not None else .35
                timeout = min(stale_wait, max(.001, lease_left))
            except Exception as exc:
                with self.lock:
                    if self.cancelled:
                        return
                loss = dict(source=self.source, revision=0, hand=None, connected=False,
                    tracking=False, age_ms=None, source_metadata=dict(source=self.source,
                    event='processing_error', error=str(exc), tracking=False, stale=True))
                self._publish_result(loss, dict(tracking=False, source=self.source,
                    message=str(exc), background_discarded=True), self.clock())
                timeout = .1
            self.wake.wait(timeout)
            self.wake.clear()
