import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src"), str(ROOT / "online_control/online_motion_track/pipeline_v2")]
from online_control.online_motion_track.direct_filter import DirectJointFilter


def test_fast_motion_small_jitter_and_no_overshoot():
    f = DirectJointFilter(); f.reset({"q":0.})
    values = [f.sample({"q":.5}, .002)["q"] for _ in range(25)]
    assert values[1] > .25  # >50% of a large move within 4 ms.
    assert values[5] > .45  # >90% within 12 ms.
    assert all(0 < q <= .5 for q in values)
    assert values == sorted(values)
    f.reset({"q":0.})
    values = [f.sample({"q": .005*(-1)**i}, .002)["q"] for i in range(200)]
    assert max(abs(q) for q in values[100:]) < .001


def test_accepted_collision_pose_prevents_hidden_filter_windup():
    f = DirectJointFilter(); f.reset({"q":0.})
    for _ in range(100):
        f.sample({"q":1.},.002); f.accept({"q":0.})
    assert f.sample({"q":0.},.002) == {"q":0.}
    f.reset({"q":-.2})
    assert f.sample({"q":-.2},.002) == {"q":-.2}


@pytest.mark.parametrize("dt", [0,-1,.2,math.nan])
def test_invalid_sample_interval_rejected(dt):
    f=DirectJointFilter();f.reset({"q":0.})
    with pytest.raises(ValueError):f.sample({"q":1.},dt)


def test_invalid_values_and_joint_change_rejected():
    f=DirectJointFilter();f.reset({"q":0.})
    for target in ({"q":math.nan},{"q":math.inf},{"wrong_joint":1.}):
        with pytest.raises(ValueError):f.sample(target,.002)


def test_v2_direct_filter_runs_before_collision_and_rebases():
    from types import SimpleNamespace
    from hand_profiles import load_hand_profile
    from online_control.online_motion_track.pipeline_v2.control_pipeline import LightControlPipeline
    c = LightControlPipeline(hand_side="left", ruckig_enabled=False, allow_direct_downlink=True)
    initial = load_hand_profile("TsenseI_v2").init_pose("left")
    c.rebase_browser_preview(initial)
    c._tracking_transition = SimpleNamespace(phase="tracking",tick=lambda **_:None,
                                            snapshot=lambda:{"phase":"tracking"})
    n = "left_hand_mid_PIP_joint"
    target = {**initial,n:.7}
    observed=[]
    def project(pose):
        observed.append(pose[n]);return {**pose,n:min(.1,pose[n])}, {"limited":pose[n]>.1}
    c._sample_collision_guard=SimpleNamespace(project=project)
    assert c.push_target(target,frame_id=1)
    c._control_tick(now=1.)
    assert .1 < observed[-1] < .7
    status=c.snapshot()
    assert status['ruckig_joint_angles_rad'][n] == .1
    assert status['direct_filter']['input_joint_angles_rad'][n] == .7
    assert status['direct_filter']['filtered_joint_angles_rad'][n] == observed[-1]
    assert c._direct_filter.pose[n] == .1
    c.push_target({**initial,n:.1},frame_id=2)
    c._control_tick(now=1.002)
    assert c.current_ui_pose()[n] == .1
