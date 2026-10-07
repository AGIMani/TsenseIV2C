"""Synthetic metric input into the real preview server; never opens hardware."""
from copy import deepcopy
from pathlib import Path
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src')]
from online_control.online_motion_track.main import make_server
from hand_profiles import load_hand_profile


class SyntheticLeapRelay:
    stale_after_ms = 350
    supports_background = False

    def __init__(self):
        self.channels = {}
        self.paused = False
        profile = load_hand_profile('TsenseI_v3_2')
        self.points = {side: profile.create_retargeter(side).fk.compute() for side in ('left', 'right')}

    def create(self, session, side):
        self.channels[session] = dict(side=side, consumed=-1, frame=None)
        return dict(source='leapmotion', transport='sdk', stale_after_ms=350)

    def latest(self, session):
        channel = self.channels[session]
        now = time.monotonic()
        if not self.paused or channel['frame'] is None:
            channel.update(at=now, revision=int(now * 30))
            points = self.points[channel['side']]
            channel['frame'] = dict(world=(points - points[0]).tolist(), label=channel['side'].title(),
                                    source='leapmotion', normalized_is_image=False)
        age = (now - channel['at']) * 1000
        hand = deepcopy(channel['frame']) if age < 350 else None
        raw = dict(simulation_only=True, sequence=channel['revision'])
        return dict(source='leapmotion', revision=channel['revision'], age_ms=age,
                    hand=hand, tracking=hand is not None, connected=hand is not None,
                    raw_leapmotion=raw, source_metadata=dict(source='leapmotion', source_epoch=1,
                        tracking=hand is not None, stale=hand is None, age_ms=age,
                        event='tracking' if hand else 'stale', raw_leapmotion=raw))

    def consume(self, session, revision):
        channel = self.channels[session]
        snapshot = self.latest(session)
        if type(revision) is not int or revision <= channel['consumed']:
            snapshot.update(hand=None, tracking=False, connected=False)
        else:
            channel['consumed'] = revision
        return snapshot

    def close(self, session):
        self.channels.pop(session, None)

    def commit_if_current(self, session, snapshot, action):
        if session not in self.channels:
            return False
        fresh = self.latest(session)
        if (snapshot['hand'] is not None) != fresh['tracking']:
            return False
        action()
        return True


def main():
    server = make_server(0, public=True)
    server.application.leapmotion = SyntheticLeapRelay()
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    print(f'LEAPMOTION_TEST_URL=http://127.0.0.1:{server.server_port}', flush=True)
    try:
        for command in sys.stdin:
            command = command.strip()
            if command == 'quit':
                break
            if command in ('pause', 'resume'):
                server.application.leapmotion.paused = command == 'pause'
    finally:
        server.shutdown()
        server.server_close()
        server.application.close()
        thread.join(timeout=5)


if __name__ == '__main__':
    main()
