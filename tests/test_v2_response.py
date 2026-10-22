"""V2 must release a fist promptly without bypassing collision or OTG limits."""
from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src"), str(ROOT / "tools")]
from benchmark_v2_response import extension_case
from utils import retarget_runtime as runtime
from hand_profiles import load_hand_profile
from TsenseI_retargeting import TopoRetargeter


@pytest.mark.parametrize("collision", [False, True])
def test_fist_release_removes_solver_creep_and_preserves_collision(collision):
    config = runtime._build_topo_config()
    # Historical response baseline predates independent fingertip position targets.
    old = replace(config, lambda_fingertip=0, lambda_smooth=2.5, lambda_reg=2.5,
                  init_max_iters=1, topo_max_iters=2)
    before = extension_case(old, collision=collision, frames=30)
    after = extension_case(config, collision=collision, frames=30)
    assert before["frames_to_5deg"]["physics"] >= (12 if collision else 7)
    assert after["frames_to_5deg"]["physics"] <= (6 if collision else 2)
    assert after["frames_to_5deg"]["solver"] <= 3
    # The 20/400/2400 limits shorten the old ten-frame response, but OTG must
    # still interpolate rather than instantly copy the collision-safe target.
    assert after["trace"][0]["ruckig_error_deg"] > 5.
    assert after["frames_to_5deg"]["ruckig"] > after["frames_to_5deg"]["physics"]
    assert max(r["solver_error_deg"] for r in after["trace"][-5:]) < .1
    if collision:
        assert any(r["contacts"] for r in after["trace"][:3])
        assert after["frames_to_5deg"]["ruckig"] < before["frames_to_5deg"]["ruckig"]


def test_desktop_profile_and_form3_reset_use_same_response_defaults(monkeypatch):
    monkeypatch.setattr(runtime, "_topo_values", dict(runtime._topo_values))
    monkeypatch.setattr(runtime, "_topo_retargeter", None)
    runtime.set_param_value("lambda_reg", 2.5)
    runtime.reset_params_to_defaults()
    config = runtime._build_topo_config()
    for key, value in load_hand_profile("TsenseI_v2").retarget_options.items():
        assert getattr(config, key) == value
    assert config.lambda_smooth > 0 and config.lambda_reg > 0


def test_small_landmark_noise_does_not_trigger_large_joint_steps():
    solver = TopoRetargeter(runtime._build_topo_config())
    q = {n: .5 if any(f"_{j}_" in n for j in ("MCP", "PIP", "DIP")) else .2
         for n in solver.joint_names}
    q["left_hand_thumb_swing_joint"] = 1.
    points = solver.fk.compute(q)
    for _ in range(40):
        solver.retarget(points, include_artifacts=False)
    rng = np.random.default_rng(20260918)
    trace = []
    for _ in range(30):
        result = solver.retarget(points + rng.normal(0, .0001, points.shape), include_artifacts=False)
        values = np.array([result.joint_values[n] for n in solver.joint_names])
        assert np.isfinite(values).all()
        assert np.all(values >= solver.lower - 1e-12)
        assert np.all(values <= solver.upper + 1e-12)
        trace.append(values)
    assert np.max(np.abs(np.diff(trace, axis=0))) < np.radians(5)
