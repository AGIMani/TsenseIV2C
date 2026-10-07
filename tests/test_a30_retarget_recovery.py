"""Transport rebases must not restart the gesture estimator (no hardware I/O)."""
import copy
from functools import partial
import json
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from online_control.online_motion_track.a30 import session as session_module

CASE = json.loads((ROOT / "tests/fixtures/a30_stationary_170707.json").read_text())


@pytest.fixture
def sessions(monkeypatch):
    # Step the real controller explicitly; a wall-clock thread would introduce
    # nondeterministic scheduling into the estimator continuity comparison.
    monkeypatch.setattr(session_module, "A30Control", partial(session_module.A30Control, start=False))
    opened = []
    def create(mode="mink", direct=False):
        s = session_module.A30WebSession(recording=False, collision_mode=mode)
        if direct:
            s.command("set_direct_mode", {"enabled": True})
        opened.append(s)
        return s
    yield create
    for s in opened:
        s.close()


def packet(i, *, noisy=False):
    points = np.array(CASE["world"])
    if noisy:
        # Recorded geometry with deterministic sub-mm, non-rigid variation.
        points += np.random.default_rng(i).normal(0, .0002, points.shape)
    return {"sequence": i, "timestamp_ms": 1000+i*1000/30, "capture_unix_ms": 0,
            "hand": {"label": "Left", "world": points.tolist(), "normalized": CASE["normalized"]}}


def recover(s, **changes):
    data = {"telemetry": bytes(484).hex(), "pose": s.control.profile.init_pose("left"),
            "resume_from_commanded_pose": True, **changes}
    return s.command("browser_rebase", data)


@pytest.mark.parametrize("mode,direct", [("off", False), ("stop", False), ("mink", False), ("mink", True)])
def test_repeated_recovery_matches_uninterrupted_noisy_tracking(sessions, mode, direct):
    recovered, reference = sessions(mode, direct), sessions(mode, direct)
    for i in range(70):
        if i >= 20 and i % 10 == 0:
            previous = dict(recovered.pipeline.session.pose)
            epoch = recovered.control.epoch
            state = recover(recovered)
            assert state["browser_epoch"] == epoch+1
            assert state["ruckig_joint_angles_rad"] == recovered.control.profile.init_pose("left")
            assert recovered.pipeline.session.pose == previous
        a, b = recovered.process(packet(i, noisy=True)), reference.process(packet(i, noisy=True))
        assert a["tracking"] and b["tracking"]
        np.testing.assert_array_equal(a["points"], b["points"])
        assert a["retarget_joint_angles_rad"] == pytest.approx(b["retarget_joint_angles_rad"], abs=1e-12)
        assert a["collision_safe_target_joint_angles_rad"] == pytest.approx(b["collision_safe_target_joint_angles_rad"], abs=1e-12)
    # Automatic recovery still seeds/checks the actual command path separately.
    for _ in range(60):
        recovered.control.step(.002)
        assert recovered.control.valid, recovered.control.error
        assert len(recovered.control.wire) == 108
        if recovered.control.guard:
            recovered.control.guard.filter.sync_pose_from_dict(recovered.control.pose)
            assert recovered.control.guard.filter.data.ncon == 0


def test_constant_recorded_landmarks_have_no_cold_start_spike(sessions):
    s = sessions()
    for i in range(80):
        before = s.process(packet(i))
    recover(s)
    after = s.process(packet(80))
    step = max(abs(after["retarget_joint_angles_rad"][n]-q)
               for n,q in before["retarget_joint_angles_rad"].items())
    assert np.rad2deg(step) < 1e-6  # Old browser_rebase produced >20 degrees.


def test_manual_rebase_still_resets_estimator_and_filters(sessions):
    s = sessions()
    for i in range(8):
        s.process(packet(i))
    assert s.filter.at is not None
    solver = s.pipeline.session.retargeter
    assert solver.previous_init_q is not None
    recover(s, resume_from_commanded_pose=False)
    assert solver.previous_q is None and solver.previous_init_q is None
    assert solver._previous_interaction_weight_matrix is None
    assert s.filter.at is None
    assert s.normalized_filter._last_time is None
    assert s.pipeline.session.pose == s.control.pose
    result = s.process(packet(8))
    assert result["tracking"]


@pytest.mark.parametrize('direct',[False,True])
def test_transport_recovery_preserves_open_target_without_another_camera_frame(sessions,direct):
    s=sessions('stop',direct)
    c=s.control
    goal=c.profile.init_pose('left')
    origin={**goal,'left_hand_index_PIP_joint':.4}
    c._seed(origin)
    c.push(goal,42)
    observed_at=c.target_at
    recover(s,pose=origin)
    assert c.target==goal  # Recovering transport must not discard the open gesture.
    assert c.frame_id==42 and c.target_at==observed_at
    for _ in range(1 if direct else 600):
        c.step(.002)
        assert c.valid,c.error
        if direct:
            assert c.guard is None and not c.finger_holds
            assert c.pose == goal
        else:
            c.guard.filter.sync_pose_from_dict(c.pose)
            assert c.guard.filter.data.ncon==0
    assert c.pose==pytest.approx(goal,abs=1e-6)


def test_recovery_does_not_hide_a_real_tracking_gap_or_hand_loss(sessions):
    s = sessions()
    s.process(packet(0)); s.process(packet(1, noisy=True))
    old_time = s.filter.at
    recover(s)
    assert s.filter.at == old_time  # Do not fake a fresh observation timestamp.
    s.process(packet(20))  # Actual >300 ms observation gap still resets filter.
    np.testing.assert_array_equal(s.filter.value, np.array(CASE["world"]))
    missing = packet(21); missing["hand"] = None
    assert not s.process(missing)["tracking"]
    assert s.filter.at is None


@pytest.mark.parametrize("change", [{"telemetry":"00"}, {"pose":{}}, {"pose":None}])
def test_invalid_recovery_still_rejects_without_erasing_tracking(sessions, change):
    s = sessions(); s.process(packet(0))
    previous = copy.deepcopy(s.pipeline.session.retargeter.previous_q)
    with pytest.raises(ValueError):
        recover(s, **change)
    np.testing.assert_array_equal(s.pipeline.session.retargeter.previous_q, previous)
    assert s.filter.at is not None
