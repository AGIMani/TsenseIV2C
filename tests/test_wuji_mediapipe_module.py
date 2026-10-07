from types import SimpleNamespace

import numpy as np

from tracking.wuji_mediapipe import (
    WUJI_FINGER_INDICES,
    WUJI_REFERENCE_SEGMENT_LENGTHS,
    WujiMediaPipe21,
    WujiMediaPipeConfig,
    preprocess_wuji_normalized_points,
    preprocess_wuji_world_points,
)


def _normalized_hand(offset: float = 0.0) -> np.ndarray:
    """Non-degenerate synthetic MediaPipe landmarks in normalized image space."""
    points = np.zeros((21, 3), dtype=np.float32)
    points[0] = [0.45 + offset, 0.78, 0.01]
    finger_x = [0.31, 0.39, 0.48, 0.57, 0.65]
    for finger, (mcp, pip, dip, tip) in enumerate(WUJI_FINGER_INDICES.values()):
        x = finger_x[finger] + offset
        points[mcp] = [x, 0.62, -0.01 * finger]
        points[pip] = [x + 0.008, 0.47, -0.018 * finger - 0.01]
        points[dip] = [x + 0.015, 0.36, -0.023 * finger - 0.02]
        points[tip] = [x + 0.022, 0.25, -0.027 * finger - 0.03]
    return points


def _world_hand(normalized: np.ndarray) -> np.ndarray:
    """Create coherent metric landmarks independent of image dimensions."""
    points = np.asarray(normalized, dtype=np.float32).copy()
    points -= points[0:1]
    points *= np.asarray([0.32, 0.32, 0.55], dtype=np.float32)
    points += np.asarray([0.01, -0.02, 0.03], dtype=np.float32)
    return points


def _wuji_reference(points: np.ndarray, width: int, height: int) -> np.ndarray:
    """Literal reference port of VideoMediaPipe's three geometry methods."""
    kp = np.asarray(points, dtype=np.float32).copy()
    kp[:, 0] *= width
    kp[:, 1] *= height
    kp[:, 2] *= width * 2.5
    kp = kp - kp[0:1, :]
    dist = np.linalg.norm(kp[9])
    if dist < 1e-6:
        return kp
    kp = kp * (0.09 / dist)

    corrected = kp.copy()
    for finger, (mcp, pip, dip, tip) in WUJI_FINGER_INDICES.items():
        lengths = WUJI_REFERENCE_SEGMENT_LENGTHS[finger]
        base = corrected[mcp].copy()
        for source_a, source_b, target_a, target_b, length in (
            (mcp, pip, mcp, pip, lengths[0]),
            (pip, dip, pip, dip, lengths[1]),
            (dip, tip, dip, tip, lengths[2]),
        ):
            segment = kp[source_b] - kp[source_a]
            segment_norm = np.linalg.norm(segment)
            if segment_norm <= 1e-6:
                continue
            if target_a == mcp:
                corrected[target_b] = base + segment / segment_norm * length
            else:
                corrected[target_b] = (
                    corrected[target_a] + segment / segment_norm * length
                )
    return corrected


def _landmarks(points: np.ndarray):
    return [SimpleNamespace(x=x, y=y, z=z) for x, y, z in points]


def _handedness(label: str):
    return [SimpleNamespace(category_name=label, score=0.99)]


def _results(hands=(), labels=(), world_hands=None):
    if world_hands is None:
        world_hands = [_world_hand(hand) for hand in hands]
    return SimpleNamespace(
        hand_landmarks=[_landmarks(hand) for hand in hands],
        handedness=[_handedness(label) for label in labels],
        hand_world_landmarks=[_landmarks(hand) for hand in world_hands],
    )


class _FakeDetector:
    def __init__(self, results):
        self.results = results
        self.last_rgb = None

    def process(self, rgb):
        self.last_rgb = rgb.copy()
        return self.results


def test_normalized_fallback_is_numerically_identical_to_wuji_reference():
    points = _normalized_hand()
    actual = preprocess_wuji_normalized_points(points, 640, 480)
    expected = _wuji_reference(points, 640, 480)
    np.testing.assert_array_equal(actual, expected)


def test_segment_lengths_and_global_scale_match_wuji_constants():
    output = preprocess_wuji_world_points(_world_hand(_normalized_hand()))
    np.testing.assert_array_equal(output[0], np.zeros(3, dtype=np.float32))
    np.testing.assert_allclose(np.linalg.norm(output[9]), 0.09, atol=1e-7)
    for finger, (mcp, pip, dip, tip) in WUJI_FINGER_INDICES.items():
        actual = [
            np.linalg.norm(output[pip] - output[mcp]),
            np.linalg.norm(output[dip] - output[pip]),
            np.linalg.norm(output[tip] - output[dip]),
        ]
        np.testing.assert_allclose(
            actual, WUJI_REFERENCE_SEGMENT_LENGTHS[finger], atol=1e-7
        )


def test_unmirrored_camera_right_hand_selects_swapped_tasks_label():
    wrong = _normalized_hand(-0.12)
    expected = _normalized_hand(0.08)
    result = _results([wrong, expected], ["Right", "Left"])
    tracker = WujiMediaPipe21(
        hand_side="right",
        detector=_FakeDetector(result),
    )
    frame = tracker.process_results(result, 640, 480)
    assert frame is not None
    assert frame.expected_label == "Left"
    assert frame.detected_label == "Left"
    assert not frame.used_fallback_hand
    np.testing.assert_array_equal(frame.normalized_points, expected)


def test_wrong_handedness_falls_back_to_first_detection():
    first = _normalized_hand(-0.04)
    second = _normalized_hand(0.04)
    tracker = WujiMediaPipe21(hand_side="right", detector=_FakeDetector(None))
    frame = tracker.process_results(
        _results([first, second], ["Right", "Right"]), 640, 480
    )
    assert frame is not None
    assert frame.detected_label == "Right"
    assert frame.used_fallback_hand
    np.testing.assert_array_equal(frame.normalized_points, first)


def test_explicitly_mirrored_input_uses_direct_handedness_label():
    right = _normalized_hand()
    tracker = WujiMediaPipe21(
        hand_side="right",
        config=WujiMediaPipeConfig(input_is_mirrored=True),
        detector=_FakeDetector(None),
    )
    frame = tracker.process_results(_results([right], ["Right"]), 640, 480)
    assert frame is not None
    assert frame.expected_label == "Right"
    assert not frame.used_fallback_hand


def test_calibrated_unmirrored_pipeline_can_disable_label_swap():
    right = _normalized_hand()
    tracker = WujiMediaPipe21(
        hand_side="right",
        config=WujiMediaPipeConfig(
            input_is_mirrored=False,
            swap_handedness=False,
        ),
        detector=_FakeDetector(None),
    )
    frame = tracker.process_results(_results([right], ["Right"]), 640, 480)
    assert frame is not None
    assert frame.expected_label == "Right"
    assert frame.detected_label == "Right"
    assert not frame.used_fallback_hand


def test_detection_loss_holds_last_valid_frame_and_reset_clears_it():
    tracker = WujiMediaPipe21(hand_side="left", detector=_FakeDetector(None))
    valid = tracker.process_results(
        _results([_normalized_hand()], ["Left"]), 640, 480
    )
    assert valid is not None

    held = tracker.process_results(_results(), 640, 480)
    assert held is not None
    assert held.held_last_valid
    np.testing.assert_array_equal(held.points, valid.points)

    held.points[1, 0] += 1.0
    np.testing.assert_array_equal(tracker.last_points, valid.points)
    tracker.reset()
    assert tracker.process_results(_results(), 640, 480) is None


def test_bgr_entrypoint_converts_to_rgb_before_detection():
    import pytest
    pytest.importorskip("cv2", reason="Optional local-camera BGR conversion; web tracking receives browser landmarks")
    results = _results([_normalized_hand()], ["Right"])
    detector = _FakeDetector(results)
    tracker = WujiMediaPipe21(
        hand_side="right",
        config=WujiMediaPipeConfig(correct_segments=False),
        detector=detector,
    )
    bgr = np.zeros((2, 3, 3), dtype=np.uint8)
    bgr[0, 0] = [11, 22, 33]
    points = tracker.process_bgr(bgr)
    assert points is not None and points.shape == (21, 3)
    np.testing.assert_array_equal(detector.last_rgb[0, 0], [33, 22, 11])


def test_one_euro_smoothing_reduces_normalized_and_world_jump():
    first = _normalized_hand()
    jumped = first.copy()
    jumped[8, 0] += 0.15
    first_world = _world_hand(first)
    jumped_world = first_world.copy()
    jumped_world[8, 2] += 0.08
    tracker = WujiMediaPipe21(
        hand_side="right",
        config=WujiMediaPipeConfig(
            correct_segments=False,
            smoothing_enabled=True,
            smoothing_min_cutoff=0.5,
            smoothing_beta=0.0,
        ),
        detector=_FakeDetector(None),
    )
    tracker.process_results(
        _results([first], ["Right"], [first_world]), 640, 480
    )
    filtered = tracker.process_results(
        _results([jumped], ["Right"], [jumped_world]), 640, 480
    )
    assert filtered is not None
    assert filtered.world_points is not None
    np.testing.assert_array_equal(filtered.raw_normalized_points, jumped)
    np.testing.assert_array_equal(filtered.raw_world_points, jumped_world)
    assert first[8, 0] < filtered.normalized_points[8, 0] < jumped[8, 0]
    assert first_world[8, 2] < filtered.world_points[8, 2] < jumped_world[8, 2]


def test_smoothing_can_be_disabled_for_exact_frame_output():
    first = _normalized_hand()
    second = first.copy()
    second[8, 2] -= 0.2
    second_world = _world_hand(second)
    tracker = WujiMediaPipe21(
        hand_side="right",
        config=WujiMediaPipeConfig(smoothing_enabled=False),
        detector=_FakeDetector(None),
    )
    tracker.process_results(_results([first], ["Right"]), 640, 480)
    output = tracker.process_results(
        _results([second], ["Right"], [second_world]), 640, 480
    )
    assert output is not None
    np.testing.assert_array_equal(output.raw_normalized_points, second)
    np.testing.assert_array_equal(output.normalized_points, second)
    np.testing.assert_array_equal(output.raw_world_points, second_world)
    np.testing.assert_array_equal(output.world_points, second_world)


def test_tasks_world_landmarks_are_primary_3d_source():
    normalized = _normalized_hand()
    normalized[:, 2] = np.linspace(-4.0, 4.0, 21, dtype=np.float32)
    world = _world_hand(_normalized_hand())
    config = WujiMediaPipeConfig(correct_segments=False, smoothing_enabled=False)
    tracker = WujiMediaPipe21(
        hand_side="right",
        config=config,
        detector=_FakeDetector(None),
    )
    frame = tracker.process_results(
        _results([normalized], ["Right"], [world]), 1920, 1080
    )
    assert frame is not None
    assert frame.depth_source == "mediapipe_world"
    np.testing.assert_array_equal(frame.raw_world_points, world)
    np.testing.assert_allclose(
        frame.points,
        preprocess_wuji_world_points(world, config=config),
        atol=1.0e-7,
    )


def test_normalized_z_is_used_only_when_world_landmarks_are_missing():
    normalized = _normalized_hand()
    config = WujiMediaPipeConfig(correct_segments=False, smoothing_enabled=False)
    tracker = WujiMediaPipe21(
        hand_side="right",
        config=config,
        detector=_FakeDetector(None),
    )
    result = _results([normalized], ["Right"], [])
    frame = tracker.process_results(result, 640, 480)
    assert frame is not None
    assert frame.depth_source == "normalized_z_fallback"
    assert frame.raw_world_points is None
    assert frame.world_points is None
    np.testing.assert_array_equal(
        frame.points,
        preprocess_wuji_normalized_points(normalized, 640, 480, config=config),
    )


def test_thumb_cmc_has_no_single_landmark_depth_override():
    world = _world_hand(_normalized_hand())
    world[1, 2] = 0.24
    config = WujiMediaPipeConfig(correct_segments=False)
    output = preprocess_wuji_world_points(world, config=config)

    wrist_relative = world - world[0:1]
    scale = config.reference_wrist_to_mid_mcp / np.linalg.norm(wrist_relative[9])
    expected = wrist_relative * scale
    np.testing.assert_array_equal(output, expected)
    assert output[1, 2] == expected[1, 2]
