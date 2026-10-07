from __future__ import annotations

import sys
from pathlib import Path

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from TsenseI_retargeting.toporetarget import (  # noqa: E402
    NUM_LANDMARKS,
    TopoRetargetConfig,
    TopoRetargeter,
    _interaction_graph_from_weight_matrix,
)


def _candidate(a: int, b: int):
    matrix = np.zeros((NUM_LANDMARKS, NUM_LANDMARKS), dtype=float)
    for idx in range(NUM_LANDMARKS):
        matrix[idx, (idx + a) % NUM_LANDMARKS] = 0.75
        matrix[idx, (idx + b) % NUM_LANDMARKS] = 0.25
    return _interaction_graph_from_weight_matrix(matrix, delaunay_ok=True)


def _bare_retargeter(alpha: float) -> TopoRetargeter:
    retargeter = object.__new__(TopoRetargeter)
    retargeter.config = TopoRetargetConfig(interaction_graph_ema_alpha=alpha)
    retargeter.previous_init_q = None
    retargeter.previous_q = None
    retargeter._previous_interaction_weight_matrix = None
    return retargeter


def test_interaction_graph_blends_edge_flips_without_changing_row_weight_sum():
    retargeter = _bare_retargeter(0.5)
    first = retargeter._stabilize_interaction_graph(_candidate(1, 2))
    second_candidate = _candidate(2, 3)
    second = retargeter._stabilize_interaction_graph(second_candidate)

    expected = 0.5 * first.weight_matrix + 0.5 * second_candidate.weight_matrix
    assert np.allclose(second.weight_matrix, expected, atol=1.0e-12)
    assert np.allclose(second.weight_matrix.sum(axis=1), 1.0, atol=1.0e-12)
    assert len(second.edges) >= len(first.edges)


def test_interaction_graph_alpha_one_preserves_legacy_current_frame_behavior():
    retargeter = _bare_retargeter(1.0)
    retargeter._stabilize_interaction_graph(_candidate(1, 2))
    current = _candidate(4, 5)
    actual = retargeter._stabilize_interaction_graph(current)
    assert np.array_equal(actual.weight_matrix, current.weight_matrix)


def test_retarget_reset_clears_graph_history_with_joint_warm_start():
    retargeter = _bare_retargeter(0.5)
    retargeter.previous_init_q = np.ones(2)
    retargeter.previous_q = np.ones(2)
    retargeter._stabilize_interaction_graph(_candidate(1, 2))

    retargeter.reset()

    assert retargeter.previous_init_q is None
    assert retargeter.previous_q is None
    assert retargeter._previous_interaction_weight_matrix is None
