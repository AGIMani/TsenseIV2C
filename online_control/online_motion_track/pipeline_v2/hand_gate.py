"""Strict handedness gate shared by live capture and offline replay."""

from __future__ import annotations


LIGHT_TEST_SWAP_HANDEDNESS = False


def expected_label_for_selected_hand(selected_hand_side: str) -> str:
    """Return the raw Tasks label used by this calibrated capture path.

    The project webcam path has been verified against recorded 3-D landmark
    chirality and uses MediaPipe's label directly (the main application also
    keeps ``_SWAP_MP_HANDEDNESS`` disabled).  Keeping this policy here lets
    live capture and offline replay make the same decision.
    """
    return (
        "Right"
        if str(selected_hand_side).strip().lower().startswith("r")
        else "Left"
    )


def hand_selection_matches(
    detected_label: str | None,
    expected_label: str | None,
    *,
    used_fallback_hand: bool = False,
) -> bool:
    """Return True only for an explicit, non-fallback handedness match.

    MediaPipe labels may already have been adjusted for mirrored/unmirrored
    input by the tracker.  This gate therefore compares the tracker's expected
    and detected labels rather than guessing physical handedness again.
    """
    detected = str(detected_label or "").strip().lower()
    expected = str(expected_label or "").strip().lower()
    return bool(
        detected
        and expected
        and detected == expected
        and not bool(used_fallback_hand)
    )


def hand_gate_block_reason(
    selected_hand_side: str,
    detected_label: str | None,
    expected_label: str | None,
    *,
    used_fallback_hand: bool = False,
) -> str | None:
    if hand_selection_matches(
        detected_label,
        expected_label,
        used_fallback_hand=used_fallback_hand,
    ):
        return None
    selected = "右手" if str(selected_hand_side).lower().startswith("r") else "左手"
    detected = str(detected_label or "unknown")
    expected = str(expected_label or "unknown")
    suffix = "（fallback）" if used_fallback_hand else ""
    return (
        f"手型门禁：当前选择{selected}，MediaPipe label={detected}{suffix}，"
        f"期望={expected}；已阻断 retarget 与下发"
    )


__all__ = [
    "LIGHT_TEST_SWAP_HANDEDNESS",
    "expected_label_for_selected_hand",
    "hand_gate_block_reason",
    "hand_selection_matches",
]
