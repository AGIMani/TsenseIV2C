"""A fist-to-middle-extension must not creep through successive live frames."""
from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from hand_profiles import load_hand_profile
from TsenseI_retargeting import TopoRetargeter
from online_control.online_motion_track.pipeline_v3_2.analyze_middle import extension_events, crossing_delay


@pytest.mark.parametrize("side", ["left", "right"])
def test_middle_extension_reaches_target_in_two_frames_with_stable_tail(side):
    profile = load_hand_profile("TsenseI_v3_2")
    solver = profile.create_retargeter(side)
    legacy = TopoRetargeter(replace(solver.config, lambda_smooth=2.5, lambda_reg=2.5,
                                  init_max_iters=1, topo_max_iters=2))
    limits = profile.joint_limits(side)
    fist = profile.init_pose(side)
    for name, (lo, hi) in limits.items():
        if (any(f"_{f}_" in name for f in ("index", "mid", "ring", "pinky"))
                and any(f"_{j}_" in name for j in ("MCP", "PIP", "DIP"))):
            fist[name] = float(np.clip(1.2, lo, hi))
    extended = {n: float(np.clip(0.0, *limits[n])) if "_mid_" in n else v for n, v in fist.items()}
    # Current assets stop at 0.005 rad; measure error from the reachable target.
    target = np.degrees([extended[f"{side}_hand_mid_{j}_joint"] for j in ("MCP", "PIP", "DIP")])
    source_fist, source_extended = [solver.fk.compute(q) for q in (fist, extended)]
    traces = []
    for engine in (legacy, solver):
        for _ in range(25):
            assert engine.retarget(source_fist, include_artifacts=False).success
        angles = []
        for _ in range(12):
            result = engine.retarget(source_extended, include_artifacts=False)
            assert result.success
            assert all(np.isfinite(v) and limits[n][0] <= v <= limits[n][1]
                       for n, v in result.joint_values.items())
            angles.append([result.joint_values[f"{side}_hand_mid_{j}_joint"] for j in ("MCP", "PIP", "DIP")])
        traces.append(np.degrees(angles)-target)
    old, fixed = traces
    assert old[1].max() > 30  # The regression input actually exposes old lag.
    assert np.abs(fixed[1]).max() < 5
    assert np.abs(fixed[-3:]).max() < .1
    assert np.abs(np.diff(fixed[-3:], axis=0)).max() < .1


def test_tuning_is_scoped_to_profiles_and_does_not_override_assets():
    from TsenseI_retargeting import TopoRetargetConfig
    assert load_hand_profile("TsenseI_v2").retarget_options["lambda_reg"] == .1
    assert TopoRetargetConfig().lambda_reg == 2.5
    bad = replace(load_hand_profile("TsenseI_v3_2"), retarget_options={"urdf_path": "wrong.urdf"})
    with pytest.raises(ValueError, match="must not override model assets"):
        bad.create_retargeter()


def test_latency_events_ignore_short_spikes_and_use_sustained_crossings():
    t = np.arange(30) * .1
    raw = np.full(30, 90.)
    raw[5] = 20  # A single erroneous landmark detection is not an extension.
    raw[15:] = 20
    events = extension_events(t, raw)
    assert events == [15]
    output = np.full(30, 90.)
    output[16] = 30
    output[19:] = 30
    assert crossing_delay(t, output, events[0]) == pytest.approx(.4)
    assert crossing_delay(t, np.full(30, 90.), events[0]) is None
