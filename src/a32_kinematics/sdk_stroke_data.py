"""SDK 1.1.15 calibrated linkage data from ``vendor/a3-cpp-sdk``.

The current SDK uses four linkage curves. ``LOOP_ROWS`` remains as a
compatibility alias for callers that used the old three-curve name; the last
field in ``JOINT_ROWS`` is now a linkage index (1..4).
"""

SDK_VERSION = "1.1.15"
SOURCE_SHA256 = "b06f545a97146ea3fc5fb5086ff11d7230566e1c248a3b0fb1d94f32b6e196a2"

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

# (linear, lower_rad, upper_rad, SDK destination, linkage index)
JOINT_ROWS = (
    (False, 0.0, 2.09, -2, 0),
    (False, 0.0, 1.74, -1, 0),
    (True, 0.0, 1.5707963267948966, 17, 2),
    (True, 0.0, 1.5707963267948966, 16, 3),
    (True, 0.0, 1.5707963267948966, 13, 1),
    (False, -0.35, 0.35, 15, 0),
    (True, 0.0, 1.5707963267948966, 12, 2),
    (True, 0.0, 1.5707963267948966, 11, 3),
    (True, 0.0, 1.5707963267948966, 10, 1),
    (False, -0.35, 0.35, 14, 0),
    (True, 0.0, 1.5707963267948966, 9, 2),
    (True, 0.0, 1.5707963267948966, 8, 3),
    (True, 0.0, 1.5707963267948966, 5, 1),
    (False, -0.35, 0.35, 7, 0),
    (True, 0.0, 1.5707963267948966, 4, 2),
    (True, 0.0, 1.5707963267948966, 3, 3),
    (True, 0.0, 1.5707963267948966, 2, 1),
    (False, -0.35, 0.35, 6, 0),
    (True, 0.0, 1.5707963267948966, 1, 4),
    (True, 0.0, 1.5707963267948966, 0, 3),
)
