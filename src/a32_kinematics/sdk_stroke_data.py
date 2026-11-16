"""SDK 1.1.15 calibrated linkage data from ``vendor/a3-cpp-sdk``.

The current SDK uses four linkage curves. ``LOOP_ROWS`` remains as a
compatibility alias for callers that used the old three-curve name; the last
field in ``JOINT_ROWS`` is now a linkage index (1..4). Joint limits include
the project's 2026-11-16 CAD patch; the calibrated linkage rows are unchanged.
"""

SDK_VERSION = "1.1.15"
SOURCE_SHA256 = "288fb085d461e32515a62c6df9fae54c6050196f8d7bd4183f1a755accdcf743"

SOURCE_JOINT_NAMES = (
    "left_hand_thumb_rota_block_joint",
    "left_hand_thumb_rotaback_joint2",
    "left_hand_thumb_MCP_joint",
    "left_hand_thumb_PIP_joint",
    "left_hand_index_MCP_joint",
    "left_hand_index_swing_joint",
    "left_hand_index_PIP_joint",
    "left_hand_index_DIP_joint",
    "left_hand_mid_MCP_joint",
    "left_hand_mid_swing_joint",
    "left_hand_mid_PIP_joint",
    "left_hand_mid_DIP_joint",
    "left_hand_ring_MCP_joint",
    "left_hand_ring_swing_joint",
    "left_hand_ring_PIP_joint",
    "left_hand_ring_DIP_joint",
    "left_hand_pinky_MCP_joint",
    "left_hand_pinky_SWING_joint",
    "left_hand_pinky_PIP_joint",
    "left_hand_pinky_DIP_joint",
)

# (b, c, d, L), in metres. The SDK source expresses these constants in
# millimetres; the project keeps the historical Python linkage units in metres
# and converts actuator outputs to millimetres at the API boundary.
LINK_ROWS = (
    None,
    (0.0055, 0.012, 0.011, 0.008),
    (0.004, 0.010190119226, 0.011, 0.0102),
    (0.002, 0.007998645498, 0.008, 0.0086),
    (0.002, 0.008000119226, 0.008, 0.0111),
)

# Compatibility name retained for existing imports.
LOOP_ROWS = LINK_ROWS

# Local CAD 2026-11-16 joint limits; linkage curves remain SDK 1.1.15.
# (linear, lower_rad, upper_rad, SDK destination, linkage index)
JOINT_ROWS = (
    (False, 0.0, 2.2689280275926285, -2, 0),
    (False, 0.0, 1.7453292519943295, -1, 0),
    (True, 0.0, 1.5707963267948966, 17, 2),
    (True, 0.0, 1.5707963267948966, 16, 3),
    (True, 0.0, 1.57, 13, 1),
    (False, -0.3490658503988659, 0.3490658503988659, 15, 0),
    (True, 0.0, 1.57, 12, 2),
    (True, 0.0, 1.57, 11, 3),
    (True, 0.0, 1.57, 10, 1),
    (False, -0.3490658503988659, 0.3490658503988659, 14, 0),
    (True, 0.0, 1.57, 9, 2),
    (True, 0.0, 1.57, 8, 3),
    (True, 0.0, 1.57, 5, 1),
    (False, -0.3490658503988659, 0.3490658503988659, 7, 0),
    (True, 0.0, 1.57, 4, 2),
    (True, 0.0, 1.57, 3, 3),
    (True, 0.0, 1.57, 2, 1),
    (False, -0.3490658503988659, 0.3490658503988659, 6, 0),
    (True, 0.0, 1.57, 1, 4),
    (True, 0.0, 1.57, 0, 3),
)
