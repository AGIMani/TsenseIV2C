"""Causal tracking regressions; no serial devices or camera are opened."""
from pathlib import Path
import sys

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))
from online_control.online_motion_track.landmark_filter import ResponsiveLandmarkFilter


def test_rejected_sample_does_not_poison_next_valid_sample():
    f, reference = ResponsiveLandmarkFilter(), ResponsiveLandmarkFilter()
    origin = np.zeros((21, 3))
    for item in (f, reference):
        item.filter(origin, 1.)
    for points, timestamp in [(np.full((21, 3), np.nan), 1.1),
                              (np.zeros((20, 3)), 1.1), (origin, np.inf),
                              (origin, 1.), (origin, .9)]:
        with pytest.raises(ValueError):
            f.filter(points, timestamp)
    np.testing.assert_array_equal(f.filter(origin + .01, 1.1), reference.filter(origin + .01, 1.1))


def test_reset_and_tracking_gap_drop_previous_motion():
    f = ResponsiveLandmarkFilter()
    origin = np.zeros((21, 3))
    f.filter(origin, 1.)
    f.filter(origin + .05, 1.03)
    np.testing.assert_array_equal(f.filter(origin - .03, 1.5), origin - .03)
    f.reset()
    np.testing.assert_array_equal(f.filter(origin + .04, 0.), origin + .04)


def test_rotation_and_translation_do_not_change_filter_response():
    rng = np.random.default_rng(391)
    rotation, _ = np.linalg.qr(rng.normal(size=(3, 3)))
    shift = np.array([.2, -.1, .05])
    a, b = ResponsiveLandmarkFilter(), ResponsiveLandmarkFilter()
    for i in range(80):
        points = rng.normal(0, .02, (21, 3))
        expected = a.filter(points, i / 30) @ rotation + shift
        np.testing.assert_allclose(b.filter(points @ rotation + shift, i / 30), expected, atol=1e-14)


def test_abrupt_reversals_stay_between_previous_output_and_current_measurement():
    rng = np.random.default_rng(43)
    f = ResponsiveLandmarkFilter()
    previous = f.filter(np.zeros((21, 3)), 0.)
    for i in range(1, 150):
        raw = rng.normal(0, .05, (21, 3))
        result = f.filter(raw, i / 30)
        assert np.isfinite(result).all()
        assert np.all(result >= np.minimum(raw, previous) - 1e-14)
        assert np.all(result <= np.maximum(raw, previous) + 1e-14)
        previous = result.copy()
        result[:] = 999  # Returned arrays cannot corrupt the next state.


@pytest.mark.parametrize('rate', [15, 30, 60])
def test_stationary_noise_and_moving_error_improve_over_legacy(rate):
    from online_control.online_motion_track.main import _OneEuroLandmarkFilter
    rng = np.random.default_rng(197 + rate)
    t = np.arange(0, 12, 1 / rate)
    truth = np.zeros((len(t), 21, 3))
    moving = (t >= 3) & (t < 9)
    truth[moving, :, 0] = (.025 * np.sin(2 * np.pi * .8 * (t[moving] - 3)))[:, None]
    noisy = truth + rng.normal(0, .0005, truth.shape)
    baseline = _OneEuroLandmarkFilter(min_cutoff=1.5, beta=.25, derivative_cutoff=1.)
    adaptive = ResponsiveLandmarkFilter()
    old = np.array([baseline.filter(p, float(at)) for p, at in zip(noisy, t)])
    new = np.array([adaptive.filter(p, float(at)) for p, at in zip(noisy, t)])
    rest = ((t > 1) & (t < 2.9)) | (t > 10.5)
    mse = lambda y, mask: np.mean((y[mask] - truth[mask]) ** 2)
    assert mse(new, rest) < mse(old, rest)
    assert mse(new, moving) < mse(old, moving) * .5


def test_bundle_includes_filter_and_queue():
    from online_control.online_motion_track.package import files
    for full in (False, True):
        entries = {relative: path for path, relative in files(full=full)}
        prefix = 'app/online_control/online_motion_track/'
        for name in ('landmark_filter.py', 'static/latest-frame-queue.js'):
            assert entries[prefix + name].is_file()


@pytest.mark.parametrize('model', ['TsenseI_v3_0', 'TsenseI_v3_2'])
def test_real_web_session_filter_selection_loss_and_skipped_sequences(model):
    from online_control.online_motion_track.main import WebSession
    session = WebSession(recording=False, model_id=model)
    try:
        assert isinstance(session.filter, ResponsiveLandmarkFilter)
        solver = session.pipeline.session.retargeter
        raw = solver.fk.compute()
        def packet(sequence, world):
            return dict(sequence=sequence, timestamp_ms=1000 + sequence * 33.333,
                        capture_unix_ms=1789600000000 + sequence * 33.333,
                        hand=None if world is None else dict(label='Left', world=world.tolist(),
                                                             normalized=np.zeros((21, 3)).tolist()))
        result = session.process(packet(0, raw))
        assert result['tracking']
        initial_points = result['points']
        result = session.process(packet(3, raw + .001))
        assert result['tracking'] and np.isfinite(list(result['pose'].values())).all()
        assert not session.process(packet(4, None))['tracking']
        result = session.process(packet(5, raw))
        assert result['tracking']
        # The pipeline canonicalizes coordinates after filtering; compare to
        # the same first observation rather than to raw camera coordinates.
        np.testing.assert_allclose(result['points'], initial_points, atol=1e-7)
    finally:
        session.close()


def test_arithmetic_optimization_preserves_previous_tuning_on_irregular_frames():
    from benchmark_landmark_latency import BaselineFilter
    before = BaselineFilter()
    after = ResponsiveLandmarkFilter(min_cutoff=1.2, beta=80.)
    rng = np.random.default_rng(813751)
    timestamp = 0.
    for i in range(700):
        timestamp += float(rng.choice([.008, .016, .033, .067, .15]))
        raw = rng.normal(0, .006, (21, 3)) + i*.00005
        np.testing.assert_allclose(after.filter(raw, timestamp),
                                   before.filter(raw, timestamp), atol=1e-14, rtol=1e-13)


@pytest.mark.parametrize('rate', [15, 30, 60])
def test_previous_production_latency_budget_and_noise_reduction(rate):
    from benchmark_landmark_latency import BaselineFilter, signal_metrics, ratio
    before = signal_metrics(BaselineFilter, rate)
    after = signal_metrics(ResponsiveLandmarkFilter, rate)
    for kind in ('sine_delay_s', 'step_delay_s', 'ramp_delay_s'):
        assert max(ratio(a, b) for a, b in zip(after[kind], before[kind])) <= 1.05
    for kind in ('white_noise_rmse_mm', 'colored_noise_rmse_mm'):
        assert after[kind] < before[kind]
    # Small deliberate motion is attenuated at most 5% more than before;
    # holding the output constant must not pass as successful smoothing.
    assert min(a/b for a, b in zip(after['sine_gain'], before['sine_gain'])) >= .95
    assert after['step_overshoot_m'] <= 1e-12


def test_local_time_constant_budget_including_speed_adaptation_and_saturation():
    from benchmark_landmark_latency import BaselineFilter
    before, after = BaselineFilter(), ResponsiveLandmarkFilter()
    speed = np.r_[0., np.geomspace(1e-7, 100., 1000)]
    old_cutoff = np.minimum(before.min_cutoff + before.beta * np.maximum(speed-before.noise_speed, 0), before.max_cutoff)
    new_cutoff = np.minimum(after.min_cutoff + after.beta * np.maximum(speed-after.noise_speed, 0), after.max_cutoff)
    assert np.max(old_cutoff/new_cutoff) <= 1.05
