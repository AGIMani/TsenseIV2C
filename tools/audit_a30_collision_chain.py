"""Deterministic model/side/policy/controller retreat audit; no USB or server."""
import argparse
import itertools
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src')]
import numpy as np
from hand_profiles import load_hand_profile
from online_control.online_motion_track.a30.control import A30Control
from online_control.online_motion_track.mink_guard import MinkCollisionGuard
from online_control.online_motion_track.pipeline_v3_0.collision import StopCollisionGuard


def audit():
    results = []
    for model, side, mode, direct in itertools.product(
            ('TsenseI_v3_0', 'TsenseI_v3_0_withid'), ('left', 'right'),
            ('stop', 'mink'), (False, True)):
        profile = load_hand_profile(model)
        control = A30Control(profile, side, mode, start=False)
        control.set_direct_mode(direct)
        guard = (MinkCollisionGuard(profile, side) if mode == 'mink'
                 else StopCollisionGuard(profile, side, finger_stop=True))
        rng = np.random.default_rng(731)
        held = 0
        open_pose = profile.init_pose(side)
        worst_error = 0.
        try:
            for trial in range(8):
                target = {n: rng.uniform(lo, hi) for n, (lo, hi) in control.limits.items()}
                for frame in range(70):
                    control.push(guard.project(target if frame < 10 else open_pose), trial*70+frame)
                    for _ in range(10):
                        control.step(.003)
                        assert control.valid, (model, side, mode, direct, control.error)
                        held += bool(control.finger_holds)
                        control.guard.filter.sync_pose_from_dict(control.pose)
                        assert control.guard.filter.data.ncon == 0, (model, side, trial, frame)
                        assert len(control.wire) == 108
                error = max(abs(control.pose[n]-q) for n, q in open_pose.items())
                worst_error = max(worst_error, error)
                assert error < 1e-5, (model, side, mode, direct, trial, error)
            row = dict(model=model, side=side, mode=mode, direct=direct,
                       cycles=8, samples=5600, max_final_error_rad=worst_error,
                       held_samples=held)
            results.append(row)
            print(json.dumps(row), flush=True)
        finally:
            control.close()
    return dict(seed=731, dt_s=.003, configurations=16, cycles=128,
                checked_samples=89600, results=results)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = audit()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
