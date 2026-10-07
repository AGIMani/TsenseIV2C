"""Measure native LeapC tracking without a browser or robot connection."""
import argparse
from datetime import datetime
import json
import math
from pathlib import Path
import statistics
import threading
import time


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration', type=float, default=60, help='Seconds per connection')
    parser.add_argument('--cycles', type=int, default=1)
    parser.add_argument('--idle', type=float, default=5, help='Disconnected seconds between cycles')
    parser.add_argument('--output', type=Path, help='Save per-second measurements and SDK events as JSONL')
    parser.add_argument('--require-hand', action='store_true', help='Fail if a cycle has no detected hand')
    args = parser.parse_args()
    if (not math.isfinite(args.duration) or not math.isfinite(args.idle)
            or args.duration <= 0 or args.cycles < 1 or args.idle < 0):
        parser.error('duration and cycles must be positive; idle must be nonnegative')

    import leap
    from leap.exceptions import LeapTimeoutError

    output = None
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        output = args.output.open('x', encoding='utf-8')
    write_lock = threading.Lock()
    started = time.monotonic()

    def emit(kind, console=True, **values):
        row = dict(event=kind, wall_time=datetime.now().astimezone().isoformat(),
                   elapsed_s=round(time.monotonic()-started, 3), **values)
        line = json.dumps(row, ensure_ascii=False)
        with write_lock:
            if output:
                output.write(line+'\n')
                output.flush()
            if console:
                print(line, flush=True)

    class Probe(leap.Listener):
        def __init__(self, cycle):
            self.cycle = cycle
            self.lock = threading.Lock()
            self.frames = self.hand_frames = self.left = self.right = 0
            self.last_sequence = None
            self.last_frame_at = None
            self.largest_gap_ms = 0
            self.losses = self.errors = self.duplicates = self.stale_frames = 0
            self.dropped_events = self.skipped_sequences = self.out_of_order = 0
            self.ages = []
            self.first_frame_at = None

        def on_connection_event(self, event):
            emit('connected', cycle=self.cycle)

        def on_connection_lost_event(self, event):
            with self.lock:
                self.losses += 1
            emit('connection_lost', cycle=self.cycle)

        def on_device_lost_event(self, event):
            with self.lock:
                self.losses += 1
            emit('device_lost', cycle=self.cycle)

        def on_device_failure_event(self, event):
            with self.lock:
                self.errors += 1
            emit('device_failure', cycle=self.cycle)

        def on_dropped_frame_event(self, event):
            with self.lock:
                self.dropped_events += 1
            emit('dropped_frame', cycle=self.cycle)

        def on_error(self, error):
            if isinstance(error, LeapTimeoutError):
                return
            with self.lock:
                self.errors += 1
            emit('sdk_error', cycle=self.cycle, message=str(error))

        def on_tracking_event(self, event):
            now = time.monotonic()
            age = (leap.get_now()-event.timestamp)/1000
            sides = [str(hand.type) for hand in event.hands]
            with self.lock:
                if self.last_sequence == event.tracking_frame_id:
                    self.duplicates += 1
                    return
                if self.last_sequence is not None:
                    step = event.tracking_frame_id-self.last_sequence
                    self.skipped_sequences += max(0, step-1)
                    self.out_of_order += step < 0
                if self.first_frame_at is None:
                    self.first_frame_at = now
                if self.last_frame_at is not None:
                    self.largest_gap_ms = max(self.largest_gap_ms, (now-self.last_frame_at)*1000)
                self.last_frame_at = now
                self.last_sequence = event.tracking_frame_id
                self.frames += 1
                self.hand_frames += bool(sides)
                self.left += 'HandType.Left' in sides
                self.right += 'HandType.Right' in sides
                self.ages.append(age)
                self.stale_frames += age >= 350 or age < 0

        def sample(self):
            with self.lock:
                ages = sorted(self.ages)
                self.ages.clear()
                return dict(frames=self.frames, hand_frames=self.hand_frames,
                    left_frames=self.left, right_frames=self.right,
                    last_sequence=self.last_sequence, losses=self.losses,
                    errors=self.errors, duplicates=self.duplicates,
                    dropped_events=self.dropped_events, skipped_sequences=self.skipped_sequences,
                    out_of_order=self.out_of_order,
                    stale_frames=self.stale_frames,
                    gap_since_last_ms=None if self.last_frame_at is None else
                        round((time.monotonic()-self.last_frame_at)*1000, 3),
                    largest_gap_ms=round(self.largest_gap_ms, 3),
                    age_median_ms=round(statistics.median(ages), 3) if ages else None,
                    age_p95_ms=round(ages[min(len(ages)-1, int(len(ages)*.95))], 3) if ages else None)

    failed = False
    try:
        emit('diagnostic_start', duration_s=args.duration, cycles=args.cycles, idle_s=args.idle)
        for cycle in range(1, args.cycles+1):
            probe = Probe(cycle)
            connection = leap.Connection(poll_timeout=.1)
            connection.add_listener(probe)
            emit('cycle_start', cycle=cycle)
            try:
                connection.connect(timeout=3)
                connection.set_tracking_mode(leap.TrackingMode.Desktop)
                begin = previous_at = time.monotonic()
                previous_frames = 0
                tick = 0
                while time.monotonic()-begin < args.duration:
                    time.sleep(min(1, max(0, args.duration-(time.monotonic()-begin))))
                    now = time.monotonic()
                    sample = probe.sample()
                    sample['received_hz'] = round((sample['frames']-previous_frames)/(now-previous_at), 2)
                    previous_frames, previous_at = sample['frames'], now
                    tick += 1
                    emit('sample', console=tick%10 == 0, cycle=cycle, **sample)
                sample = probe.sample()
                failed |= (sample['frames'] == 0 or sample['losses'] > 0 or sample['errors'] > 0
                           or sample['out_of_order'] > 0
                           or (args.require_hand and sample['hand_frames'] == 0)
                           or sample['stale_frames'] > 0 or sample['largest_gap_ms'] >= 350
                           or (sample['gap_since_last_ms'] or 0) >= 350)
                emit('cycle_complete', cycle=cycle, **sample)
            finally:
                connection.disconnect()
                connection.remove_listener(probe)
                emit('client_closed', cycle=cycle)
            if cycle < args.cycles:
                emit('idle', duration_s=args.idle)
                time.sleep(args.idle)
        emit('diagnostic_complete', passed=not failed)
    finally:
        if output:
            output.close()
    return int(failed)


if __name__ == '__main__':
    raise SystemExit(main())
