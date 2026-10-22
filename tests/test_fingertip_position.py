"""Weighted fingertip tracking in the actual bounded retarget solver."""
from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "src")]
from hand_profiles import PROFILE_PATHS, load_hand_profile
from TsenseI_retargeting.toporetarget import (
    TopoRetargetConfig, TopoRetargeter, _canonical_hand_points,
    _build_interaction_graph, _laplacian_coordinates, _relative_bone_terms,
)
from utils import retarget_runtime as runtime

TIPS = np.array([4, 8, 12, 16, 20])


def solver_for(model="TsenseI_v3_2", side="left", **options):
    prototype = load_hand_profile(model).create_retargeter(side)
    return TopoRetargeter(replace(prototype.config, **options))


def objective_args(solver, source, q):
    points = solver._source_points_in_robot_scale(source)
    canonical = _canonical_hand_points(points)
    graph = _build_interaction_graph(canonical, solver.config,
                                    distance_scale_m=solver.robot_palm_width_m)
    return (q.copy(), q.copy(), _laplacian_coordinates(canonical, graph),
            _relative_bone_terms(points), graph)


@pytest.mark.parametrize("model", tuple(PROFILE_PATHS))
@pytest.mark.parametrize("side", ["left", "right"])
def test_full_jacobian_loss_and_profile_weights(model, side):
    solver = solver_for(model, side, pinch_enabled=False)
    source = solver._robot_points_palm_m(.7 * solver.lower + .3 * solver.upper)
    result = solver.retarget(source)
    assert result.diagnostics["parameters"]["fingertip_weights"] == [2.] * 5
    assert result.diagnostics["parameters"]["fingertip_position_effective"]
    q = .6 * solver.lower + .4 * solver.upper
    args = objective_args(solver, source, q)
    residual, analytic = solver._weighted_residual_and_jacobian(q, *args, eps=1e-6)
    np.testing.assert_allclose(residual, solver._weighted_residual(q, *args), atol=1e-12)
    numerical = np.stack([
        (solver._weighted_residual(q+d, *args)-solver._weighted_residual(q-d, *args))/2e-6
        for d in np.eye(len(q))*1e-6
    ], axis=-1)
    np.testing.assert_allclose(analytic, numerical, atol=1e-5, rtol=1e-4)
    loss = solver._loss_breakdown(q, *args)
    off = TopoRetargeter(replace(solver.config, lambda_fingertip=0))
    off.retarget(source)
    off_residual = off._weighted_residual(q, *args)
    # Existing bone diagnostics use a mean; compare the new contribution alone.
    added_objective = .5 * (residual @ residual - off_residual @ off_residual)
    assert loss["fingertip_position_weighted"] == pytest.approx(added_objective)
    assert loss["total"] - off._loss_breakdown(q, *args)["total"] == pytest.approx(added_objective)
    target = _canonical_hand_points(solver._source_points_in_robot_scale(source))[TIPS]
    error = (solver._robot_points_topo_m(q)[TIPS] - target) / solver.robot_palm_width_m
    assert loss["fingertip_position_weighted"] == pytest.approx(np.sum(error**2))


def custom_weights(tmp_path, prototype, weights):
    data = yaml.safe_load(prototype.keypoint_path.read_text())
    for point in data["keypoints"]:
        if point["id"] in TIPS:
            point["weight"] = float(weights[list(TIPS).index(point["id"])])
    # Prove matching is by MediaPipe id, not YAML order.
    data["keypoints"].reverse()
    path = tmp_path / "keypoints.yaml"
    path.write_text(yaml.safe_dump(data))
    return TopoRetargeter(replace(prototype.config, keypoints_path=path))


def test_yaml_weights_scale_each_tip_loss_once_and_zero_disables(tmp_path):
    weights = np.array([0., .5, 1., 2., 3.])
    solver = custom_weights(tmp_path, solver_for(pinch_enabled=False), weights)
    source = solver._robot_points_palm_m(solver.init_q)
    solver.retarget(source)
    target = _canonical_hand_points(solver._source_points_in_robot_scale(source))
    for index, tip in enumerate(TIPS):
        displaced = target.copy()
        displaced[tip, 0] += solver.robot_palm_width_m * .1
        residual = solver._fingertip_residual(displaced)
        assert .5 * residual @ residual == pytest.approx(.5 * weights[index] * .1**2)


@pytest.mark.parametrize("weight", [-1., float("nan"), float("inf")])
def test_invalid_yaml_weights_rejected(tmp_path, weight):
    with pytest.raises(ValueError, match="fingertip keypoint weights"):
        custom_weights(tmp_path, solver_for(), [weight]*5)


@pytest.mark.parametrize("weight", [-1., float("nan"), float("inf")])
def test_invalid_global_weight_rejected(weight):
    with pytest.raises(ValueError, match="lambda_fingertip"):
        TopoRetargetConfig(lambda_fingertip=weight)


def test_position_target_is_invariant_to_scale_rotation_translation_and_resets():
    solver = solver_for(pinch_enabled=False)
    source = solver._robot_points_palm_m(.7 * solver.lower + .3 * solver.upper)
    rotation = np.array([[0., -1, 0], [1, 0, 0], [0, 0, 1]])
    solver.retarget(source)
    expected = solver._fingertip_targets.copy()
    for scale in (.5, 2.):
        solver.retarget(scale * (source @ rotation.T) + [1., -2., 3.])
        np.testing.assert_allclose(solver._fingertip_targets, expected, atol=1e-12)
    # Next frame must replace targets; invalid frames and reset must clear them.
    changed = source.copy()
    changed[8] += [.001, 0, 0]
    solver.retarget(changed)
    assert not np.array_equal(solver._fingertip_targets, expected)
    with pytest.raises(ValueError):
        solver.retarget(np.full((21, 3), np.nan))
    assert solver._fingertip_targets is None
    solver.retarget(source)
    solver.reset()
    assert solver._fingertip_targets is None


def test_contact_fades_only_its_own_position_targets_continuously():
    solver = solver_for()
    source = solver._robot_points_palm_m(solver.init_q).copy()
    width = np.linalg.norm(source[5] - source[17])
    # Keep the other three contacts inactive.
    source[[12, 16, 20]] = source[4] + [2 * width, 0, 0]
    full, start = solver.config.pinch_full_ratio, solver.config.pinch_start_ratio
    for ratio, alpha in ((start, 0), ((full + start) / 2, .5), (full, 1), (start, 0)):
        source[8] = source[4] + [ratio * width, 0, 0]
        result = solver.retarget(source)
        assert result.diagnostics["pinch"]["pairs"]["index"]["alpha"] == pytest.approx(alpha)
        effective = (solver._fingertip_gains[:, 0] * solver.robot_palm_width_m)**2
        np.testing.assert_allclose(effective, [2*(1-alpha), 2*(1-alpha), 2, 2, 2], atol=1e-12)


@pytest.mark.parametrize("options", [{"lambda_fingertip": 0}, {"use_topo_optimization": False}])
def test_disabled_position_term_has_no_loss_or_targets(options):
    solver = solver_for(**options)
    result = solver.retarget(solver._robot_points_palm_m(solver.init_q))
    assert result.loss["fingertip_position_weighted"] == 0
    assert not result.diagnostics["parameters"]["fingertip_position_effective"]
    assert solver._fingertip_targets is None


def test_analytic_and_finite_difference_solvers_track_same_target():
    analytic = solver_for(pinch_enabled=False)
    numeric = TopoRetargeter(replace(analytic.config, use_analytic_jacobian=False))
    source = analytic._robot_points_palm_m(.8 * analytic.lower + .2 * analytic.upper)
    for solver in (analytic, numeric):
        result = solver.retarget(source, include_artifacts=False)
        assert result.success
        assert solver._fingertip_targets is not None
        assert np.all(solver.previous_q >= solver.lower)
        assert np.all(solver.previous_q <= solver.upper)
    np.testing.assert_allclose(analytic.previous_q, numeric.previous_q, atol=2e-3)


@pytest.mark.parametrize("model", tuple(PROFILE_PATHS))
@pytest.mark.parametrize("side", ["left", "right"])
def test_position_objective_reduces_tracking_error(model, side):
    on = solver_for(model, side, pinch_enabled=False)
    off = TopoRetargeter(replace(on.config, lambda_fingertip=0))
    # A reachable bent pose, with the same temporal/initialization priors for both runs.
    source = on._robot_points_palm_m(.75 * on.lower + .25 * on.upper)
    target = _canonical_hand_points(on._source_points_in_robot_scale(source))[TIPS]
    errors = []
    for solver in (off, on):
        for _ in range(20):
            result = solver.retarget(source, include_artifacts=False)
            assert result.success
            assert np.all(solver.previous_q >= solver.lower)
            assert np.all(solver.previous_q <= solver.upper)
        errors.append(np.linalg.norm(solver._robot_points_topo_m(solver.previous_q)[TIPS] - target))
    assert errors[1] < errors[0], (model, side, errors)


def test_runtime_default_and_disable(monkeypatch):
    monkeypatch.setattr(runtime, "_topo_values", {s.key: s.default for s in runtime.TOPO_PARAM_SPECS})
    monkeypatch.setattr(runtime, "_topo_retargeter", None)
    assert runtime._build_topo_config().lambda_fingertip == 1
    assert runtime.set_param_value("lambda_fingertip", 0)
    assert runtime._build_topo_config().lambda_fingertip == 0
    for value in (float("nan"), float("inf")):
        assert not runtime.set_param_value("lambda_fingertip", value)
    assert runtime._build_topo_config().lambda_fingertip == 0
