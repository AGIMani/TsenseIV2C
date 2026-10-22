"""Independent thumb/four-finger contact on calibrated project models."""
from dataclasses import replace
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / 'src')]
from TsenseI_retargeting.toporetarget import (TopoRetargetConfig, TopoRetargeter, pinch_observation,
    PINCH_FINGERS, _canonical_hand_points, _relative_bone_terms, _build_interaction_graph, _laplacian_coordinates)
from TsenseI_retargeting.landmarks import PINCH_PAIRS
from hand_profiles import PROFILE_PATHS, load_hand_profile
from utils import retarget_runtime as runtime


def solver_for(model='TsenseI_v3_0', side='left', **options):
    prototype = load_hand_profile(model).create_retargeter(side)
    return TopoRetargeter(replace(prototype.config, kinematics_backend='numpy', **options))


def contact_input(solver, tip=8):
    source = solver._robot_points_palm_m(solver.init_q).copy()
    source[4] = source[tip] + [.001, 0, 0]
    return source


@pytest.mark.parametrize('model', ['TsenseI_v2', 'TsenseI_v3_0', 'TsenseI_v3_2'])
@pytest.mark.parametrize('side', ['left', 'right'])
def test_profile_defaults_and_actual_contact_improvement(model, side):
    on = solver_for(model, side)
    assert (on.config.pinch_enabled, on.config.pinch_full_ratio,
            on.config.pinch_start_ratio, on.config.pinch_weight) == (True, .25 if model == "TsenseI_v3_2" else .15, .5, 75 if model == "TsenseI_v3_2" else 65)
    assert on.config.pinch_index_weight == load_hand_profile(model).retarget_options["pinch_index_weight"]
    off = TopoRetargeter(replace(on.config, pinch_enabled=False))
    source = contact_input(on)
    gaps = []
    for solver in (off, on):
        for _ in range(30):
            result = solver.retarget(source, include_artifacts=False)
            assert result.success
            assert np.all(solver.previous_q >= solver.lower)
            assert np.all(solver.previous_q <= solver.upper)
        p = solver._robot_points_palm_m(solver.previous_q)
        gaps.append(np.linalg.norm(p[4]-p[8]))
    assert result.diagnostics['pinch']['alpha'] == 1
    assert gaps[1] < gaps[0] * .15
    # A3.0 calibrated points have a nonzero compromise gap at the chosen weight.
    assert gaps[1] < .004


def test_ratios_are_invariant_to_scale_rotation_translation():
    source = contact_input(solver_for())
    config = TopoRetargetConfig()
    width = np.linalg.norm(source[5]-source[17])
    rotation = np.array([[0., -1, 0], [1, 0, 0], [0, 0, 1]])
    for ratio, alpha in ((.15, 1), (.325, .5), (.5, 0)):
        source[4] = source[8] + [width*ratio, 0, 0]
        for scale in (.5, 1, 2):
            d = pinch_observation(scale*(source @ rotation.T)+[1, -2, 3], config)
            assert d['alpha'] == pytest.approx(alpha, abs=1e-10)
            assert d['full_threshold_m'] == pytest.approx(.15*width*scale)


@pytest.mark.parametrize('backend', ['numpy', 'auto'])
@pytest.mark.parametrize('tips', [(8,), (12,), (16,), (20,), (8, 12, 16, 20)])
def test_contact_jacobian_and_full_residual_are_consistent(backend, tips):
    solver = solver_for(pinch_index_weight=130)
    solver = TopoRetargeter(replace(solver.config, kinematics_backend=backend))
    source = contact_input(solver)
    width = np.linalg.norm(source[5]-source[17])
    for _, tip in PINCH_PAIRS:
        # Isolate each pair, then exercise the sum of all four contacts.
        source[tip] = source[4] + [width*(.05 if tip in tips else 1.5), 0, 0]
    solver.retarget(source)
    points = solver._source_points_in_robot_scale(source)
    canonical = _canonical_hand_points(points)
    graph = _build_interaction_graph(canonical, solver.config, distance_scale_m=solver.robot_palm_width_m)
    q = .4*solver.lower + .6*solver.upper
    args = (q.copy(), q.copy(), _laplacian_coordinates(canonical, graph), _relative_bone_terms(points), graph)
    residual, analytic = solver._weighted_residual_and_jacobian(q, *args, eps=1e-6)
    np.testing.assert_allclose(residual, solver._weighted_residual(q, *args), atol=1e-12)
    numerical = np.stack([(solver._weighted_residual(q+d, *args)-solver._weighted_residual(q-d, *args))/2e-6
                          for d in np.eye(len(q))*1e-6], axis=-1)
    np.testing.assert_allclose(analytic, numerical, atol=1e-5, rtol=1e-4)
    loss = solver._loss_breakdown(q, *args)
    p = solver._robot_points_palm_m(q)
    expected = .5/solver.robot_palm_width_m**2*sum(
        (130 if tip == 8 else solver.config.pinch_weight)*np.sum((p[4]-p[tip])**2) for tip in tips)
    assert loss['pinch_weighted'] == pytest.approx(expected)
    inactive = TopoRetargeter(replace(solver.config, pinch_enabled=False))
    # Isolate contact loss with identical position targets and contact-fade gains.
    inactive.retarget(source)
    inactive._fingertip_targets = solver._fingertip_targets
    inactive._fingertip_gains = solver._fingertip_gains.copy()
    no_contact = inactive._weighted_residual(q, *args)
    np.testing.assert_array_equal(residual[:-3*len(tips)], no_contact)
    assert loss['total']-inactive._loss_breakdown(q, *args)['total'] == pytest.approx(expected)
    assert .5*(residual@residual-no_contact@no_contact) == pytest.approx(expected)


def test_off_zero_release_reset_and_invalid_input():
    off, zero = solver_for(pinch_enabled=False), solver_for(pinch_weight=0, pinch_index_weight=0)
    source = contact_input(off)
    for _ in range(4):
        a, b = off.retarget(source), zero.retarget(source)
        assert a.joint_values == b.joint_values
    solver = solver_for()
    first = solver.retarget(source)
    assert first.loss['pinch_weighted'] > 0
    solver.reset()
    assert not solver._pinch_tips.size
    assert solver.retarget(source).joint_values == first.joint_values
    source[4] = source[8] + [1, 0, 0]
    assert solver.retarget(source).diagnostics['pinch']['alpha'] == 0
    assert not solver._pinch_tips.size
    with pytest.raises(ValueError):solver.retarget(np.zeros((21, 3)))
    assert not solver._pinch_tips.size
    init_only = solver_for(use_topo_optimization=False)
    assert not init_only.retarget(contact_input(init_only)).diagnostics['pinch']['effective']


def test_runtime_defaults_and_invalid_threshold_updates(monkeypatch):
    monkeypatch.setattr(runtime, '_topo_values', {s.key: s.default for s in runtime.TOPO_PARAM_SPECS})
    monkeypatch.setattr(runtime, '_topo_retargeter', None)
    cfg = runtime._build_topo_config()
    assert (cfg.pinch_enabled, cfg.pinch_full_ratio, cfg.pinch_start_ratio, cfg.pinch_weight) == (True, .15, .5, 65)
    for key, value in [('pinch_full_ratio', .6), ('pinch_start_ratio', .05), ('pinch_weight', float('nan'))]:
        assert not runtime.set_param_value(key, value)
    assert runtime._build_topo_config() == cfg
    assert runtime.set_param_value('pinch_weight', 70)
    assert runtime._build_topo_config().pinch_weight == 70


@pytest.mark.parametrize('kwargs', [{'pinch_full_ratio': .6}, {'pinch_weight': -1},
                                  {'pinch_start_ratio': float('inf')}])
def test_invalid_configuration_rejected(kwargs):
    with pytest.raises(ValueError):TopoRetargetConfig(**kwargs)


@pytest.mark.parametrize('finger,tip', tuple(zip(PINCH_FINGERS, (8, 12, 16, 20))))
def test_each_contact_has_independent_continuous_thresholds(finger, tip):
    solver = solver_for()
    source = solver._robot_points_palm_m(solver.init_q).copy()
    width = np.linalg.norm(source[5]-source[17])
    rotation = np.array([[0., -1, 0], [1, 0, 0], [0, 0, 1]])
    for _, other in PINCH_PAIRS:
        source[other] = source[4] + [2*width, 0, 0]
    for ratio, alpha in ((.04, 1), (.15, 1), (.325, .5), (.5, 0), (.7, 0)):
        source[tip] = source[4] + [ratio*width, 0, 0]
        for scale in (.5, 1, 2):
            observation = pinch_observation(scale*(source @ rotation.T)+[1, -2, 3], TopoRetargetConfig())
            assert observation['pairs'][finger]['alpha'] == pytest.approx(alpha, abs=1e-10)
            assert observation['pairs'][finger]['pair'] == [4, tip]
            assert all(pair['alpha'] == 0 for name, pair in observation['pairs'].items() if name != finger)


@pytest.mark.parametrize('model', tuple(PROFILE_PATHS))
@pytest.mark.parametrize('side', ['left', 'right'])
@pytest.mark.parametrize('finger,tip', tuple(zip(PINCH_FINGERS, (8, 12, 16, 20))))
def test_all_models_reduce_each_contact_gap_without_exceeding_limits(model, side, finger, tip):
    on = solver_for(model, side)
    off = TopoRetargeter(replace(on.config, pinch_enabled=False))
    source = contact_input(on, tip)
    gaps = []
    for solver in (off, on):
        for _ in range(30):
            result = solver.retarget(source, include_artifacts=False)
            assert result.success
            assert np.isfinite(solver.previous_q).all()
            assert np.all(solver.previous_q >= solver.lower)
            assert np.all(solver.previous_q <= solver.upper)
        points = solver._robot_points_palm_m(solver.previous_q)
        gaps.append(float(np.linalg.norm(points[4]-points[tip])))
    assert result.diagnostics['pinch']['pairs'][finger]['alpha'] == 1
    assert gaps[1] < gaps[0], (model, side, finger, gaps)


def test_switching_and_releasing_one_pair_does_not_latch_another():
    solver = solver_for()
    source = solver._robot_points_palm_m(solver.init_q).copy()
    width = np.linalg.norm(source[5]-source[17])
    for selected in ((8, 12, 16, 20), (12,), (16, 20), ()):
        for _, tip in PINCH_PAIRS:
            source[tip] = source[4] + [width*(.05 if tip in selected else 1), 0, 0]
        result = solver.retarget(source)
        assert solver._pinch_tips.tolist() == list(selected)
        points = np.array(result.robot_keypoints_21)
        for name, (_, tip) in zip(PINCH_FINGERS, PINCH_PAIRS):
            pair = result.diagnostics['pinch']['pairs'][name]
            assert pair['alpha'] == (1 if tip in selected else 0)
            assert pair['output_distance_m'] == pytest.approx(np.linalg.norm(points[4]-points[tip]))
        if not selected:
            assert result.loss['pinch_weighted'] == 0
    # An invalid next frame must also clear every pair before raising.
    solver.retarget(contact_input(solver, 20))
    with pytest.raises(ValueError):solver.retarget(np.full((21, 3), np.nan))
    assert not solver._pinch_tips.size


@pytest.mark.parametrize('options', [{'pinch_enabled':False}, {'pinch_weight':0, 'pinch_index_weight':0}, {'use_topo_optimization':False}])
def test_disabling_contact_removes_all_four_pairs(options):
    solver = solver_for(**options)
    source = contact_input(solver)
    source[[8, 12, 16, 20]] = source[4]+[.001, 0, 0]
    result = solver.retarget(source)
    assert all(pair['intent_alpha'] == 1 and pair['alpha'] == 0 for pair in result.diagnostics['pinch']['pairs'].values())
    assert not solver._pinch_tips.size
    assert result.loss['pinch_weighted'] == 0


def test_four_pair_residual_also_supports_finite_difference_solver():
    solver = solver_for(use_analytic_jacobian=False)
    source = contact_input(solver)
    source[[8, 12, 16, 20]] = source[4]+[.001, 0, 0]
    result = solver.retarget(source)
    assert result.success
    assert result.diagnostics['parameters']['residual_jacobian'] == 'finite_difference'
    assert result.loss['pinch_weighted'] > 0
    assert np.isfinite(list(result.joint_values.values())).all()


@pytest.mark.parametrize('model', tuple(PROFILE_PATHS))
@pytest.mark.parametrize('side', ['left', 'right'])
def test_index_weight_is_independent_for_each_model(model, side):
    base = solver_for(model, side, pinch_weight=65, pinch_index_weight=65)
    stronger = TopoRetargeter(replace(base.config, pinch_index_weight=130))
    source = contact_input(base)
    source[[8, 12, 16, 20]] = source[4] + [.001, 0, 0]
    for solver in (base, stronger):
        result = solver.retarget(source)
        assert result.success
    points = base._robot_points_palm_m(.4*base.lower + .6*base.upper)
    a = base._pinch_residual(points).reshape(4, 3)
    b = stronger._pinch_residual(points).reshape(4, 3)
    np.testing.assert_allclose(b[0], np.sqrt(2)*a[0])
    np.testing.assert_array_equal(b[1:], a[1:])
    pairs = result.diagnostics['pinch']['pairs']
    assert [p['weight'] for p in pairs.values()] == [130, 65, 65, 65]
    assert result.diagnostics['parameters']['pinch_index_weight'] == 130


@pytest.mark.parametrize('index_weight,other_weight,active', [(0, 65, [12, 16, 20]), (130, 0, [8])])
def test_zero_pair_weight_disables_only_its_contacts(index_weight, other_weight, active):
    solver = solver_for(pinch_index_weight=index_weight, pinch_weight=other_weight)
    source = contact_input(solver)
    source[[8, 12, 16, 20]] = source[4] + [.001, 0, 0]
    result = solver.retarget(source)
    assert solver._pinch_tips.tolist() == active
    assert result.diagnostics['pinch']['effective']
    for pair in result.diagnostics['pinch']['pairs'].values():
        assert pair['alpha'] == (1 if pair['pair'][1] in active else 0)
    # A disabled contact must leave the corresponding fingertip tracking active.
    gains = solver._fingertip_gains.ravel()
    assert (gains[1] > 0) == (8 not in active)
    assert (gains[2] > 0) == (12 not in active)


def test_legacy_index_weight_inherits_general_weight():
    source = contact_input(solver_for())
    for weight in (0, 42, 100):
        observation = pinch_observation(source, TopoRetargetConfig(pinch_weight=weight))
        assert all(p['weight'] == weight for p in observation['pairs'].values())
        assert observation['effective'] == (weight > 0)


@pytest.mark.parametrize('weight', [-1, 1001, float('nan'), float('inf')])
def test_invalid_index_weight_rejected(weight):
    with pytest.raises(ValueError, match='pinch_index_weight'):
        TopoRetargetConfig(pinch_index_weight=weight)


def test_runtime_index_weight_update_is_independent(monkeypatch):
    monkeypatch.setattr(runtime, '_topo_values', {s.key: s.default for s in runtime.TOPO_PARAM_SPECS})
    monkeypatch.setattr(runtime, '_topo_retargeter', None)
    assert runtime.set_param_value('pinch_index_weight', 130)
    cfg = runtime._build_topo_config()
    assert cfg.pinch_index_weight == 130
    assert cfg.pinch_weight == 65
    assert not runtime.set_param_value('pinch_index_weight', float('nan'))
    assert runtime._build_topo_config() == cfg


@pytest.mark.parametrize('model', tuple(PROFILE_PATHS))
@pytest.mark.parametrize('side', ['left', 'right'])
def test_index_full_ratio_is_independent_in_solver(model, side):
    profile = load_hand_profile(model)
    assert profile.create_retargeter(side).config.pinch_index_full_ratio == profile.retarget_options['pinch_index_full_ratio']
    solver = solver_for(model, side, pinch_index_full_ratio=.3, pinch_full_ratio=.1)
    source = contact_input(solver)
    width = np.linalg.norm(source[5] - source[17])
    for ratio, index_alpha, other_alpha in ((.1, 1, 1), (.3, 1, .5), (.4, .5, .15625), (.5, 0, 0), (.6, 0, 0)):
        source[[8, 12, 16, 20]] = source[4] + [ratio * width, 0, 0]
        result = solver.retarget(source)
        assert result.success
        observation = result.diagnostics['pinch']
        assert observation['full_threshold_m'] == pytest.approx(.3 * width)
        assert result.diagnostics['parameters']['pinch_index_full_ratio'] == .3
        for name, pair in observation['pairs'].items():
            expected = index_alpha if name == 'index' else other_alpha
            full = .3 if name == 'index' else .1
            assert pair['alpha'] == pytest.approx(expected, abs=1e-12)
            assert pair['full_ratio'] == full
            assert pair['full_threshold_m'] == pytest.approx(full * width)
        if ratio < .5:
            expected_gains = np.sqrt([pair['weight'] * pair['alpha']
                                      for pair in observation['pairs'].values()]) / solver.robot_palm_width_m
            np.testing.assert_allclose(solver._pinch_gains.ravel(), expected_gains)
        else:
            # Reconstructing a distance exactly at start can round just below it.
            np.testing.assert_allclose(solver._pinch_gains, 0, atol=1e-10)
            if ratio > .5:
                assert not solver._pinch_tips.size
        expected_tip_gain = np.sqrt(solver.config.lambda_fingertip * solver._fingertip_weights[1]
                                    * (1 - index_alpha)) / solver.robot_palm_width_m
        assert solver._fingertip_gains[1, 0] == pytest.approx(expected_tip_gain, abs=1e-10)


def test_legacy_index_full_ratio_inherits_shared_threshold():
    source = contact_input(solver_for())
    width = np.linalg.norm(source[5] - source[17])
    source[[8, 12, 16, 20]] = source[4] + [.35 * width, 0, 0]
    observation = pinch_observation(source, TopoRetargetConfig(pinch_full_ratio=.2))
    assert all(pair['full_ratio'] == .2 for pair in observation['pairs'].values())
    assert all(pair['alpha'] == pytest.approx(.5) for pair in observation['pairs'].values())


@pytest.mark.parametrize('ratio', [-.01, .5, .6, float('nan'), float('inf')])
def test_invalid_index_full_ratio_rejected(ratio):
    with pytest.raises(ValueError, match='pinch_index_full_ratio'):
        TopoRetargetConfig(pinch_index_full_ratio=ratio)


def test_runtime_index_threshold_validates_both_full_thresholds(monkeypatch):
    monkeypatch.setattr(runtime, '_topo_values', {s.key: s.default for s in runtime.TOPO_PARAM_SPECS})
    monkeypatch.setattr(runtime, '_topo_retargeter', None)
    assert runtime.set_param_value('pinch_index_full_ratio', .3)
    cfg = runtime._build_topo_config()
    assert cfg.pinch_index_full_ratio == .3
    assert cfg.pinch_full_ratio == .15
    for key, value in [('pinch_index_full_ratio', .5), ('pinch_index_full_ratio', float('nan')),
                       ('pinch_start_ratio', .3), ('pinch_start_ratio', .2)]:
        assert not runtime.set_param_value(key, value)
        assert runtime._build_topo_config() == cfg
    assert runtime.set_param_value('pinch_full_ratio', .4)
    assert not runtime.set_param_value('pinch_start_ratio', .35)
    assert runtime.set_param_value('pinch_start_ratio', .45)
    assert runtime._build_topo_config().pinch_start_ratio == .45
