"""Shared joint names and limits without camera or inference dependencies."""
import math

URDF_JOINT_MAP = {
    "thumb_rota":  "left_hand_thumb_rota_joint",
    "thumb_swing": "left_hand_thumb_swing_joint",
    "thumb_mcp":   "left_hand_thumb_MCP_joint",
    "thumb_pip":   "left_hand_thumb_PIP_joint",

    "index_swing": "left_hand_index_swing_joint",
    "index_mcp":   "left_hand_index_MCP_joint",
    "index_pip":   "left_hand_index_PIP_joint",
    "index_dip":   "left_hand_index_DIP_joint",

    "mid_swing":   "left_hand_mid_swing_joint",
    "mid_mcp":     "left_hand_mid_MCP_joint",
    "mid_pip":     "left_hand_mid_PIP_joint",
    "mid_dip":     "left_hand_mid_DIP_joint",

    "ring_swing":  "left_hand_ring_swing_joint",
    "ring_mcp":    "left_hand_ring_MCP_joint",
    "ring_pip":    "left_hand_ring_PIP_joint",
    "ring_dip":    "left_hand_ring_DIP_joint",

    "pinky_rota":  "left_hand_pinky_rota_joint",
    "pinky_swing": "left_hand_pinky_swing_joint",
    "pinky_mcp":   "left_hand_pinky_MCP_joint",
    "pinky_pip":   "left_hand_pinky_PIP_joint",
    "pinky_dip":   "left_hand_pinky_DIP_joint",
}
_URDF_TO_LIMIT_KEY = {v: k for k, v in URDF_JOINT_MAP.items()}

# ──────────── 关节限位（弧度，取自 URDF）────────────
_DEG_SWING_MAX = 0.696  # 与 URDF / MJCF 四指 swing 上限一致（约 39.88°）


JOINT_LIMITS = {
    "thumb_rota":  (0.0, 1.57),
    "thumb_swing": (0.0, 2.0),  # q_new = 1 - q_old；旧中位对应 1 rad
    "thumb_mcp":   (0.0, 1.57),
    "thumb_pip":   (0.0, 1.57),

    "index_swing": (0.0, _DEG_SWING_MAX),
    "index_mcp":   (0.0, 1.57),
    "index_pip":   (0.0, 1.57),
    "index_dip":   (0.0, 1.57),

    "mid_swing":   (0.0, _DEG_SWING_MAX),
    "mid_mcp":     (0.0, 1.57),
    "mid_pip":     (0.0, 1.57),
    "mid_dip":     (0.0, 1.57),

    "ring_swing":  (0.0, _DEG_SWING_MAX),
    "ring_mcp":    (0.0, 1.57),
    "ring_pip":    (0.0, 1.57),
    "ring_dip":    (0.0, 1.57),

    "pinky_rota":  (0.0, math.radians(50.0)),
    "pinky_swing": (0.0, _DEG_SWING_MAX),
    "pinky_mcp":   (0.0, 1.57),
    "pinky_pip":   (0.0, 1.57),
    "pinky_dip":   (0.0, 1.57),
}


def default_joint_position(name: str) -> float:
    """The only init rule: every joint starts at zero in UI/URDF space."""
    return 0.0


def get_init_pose(hand_side: str = "left", *, joint_names=None) -> dict[str, float]:
    """Return a fresh all-zero init pose; hands differ only in joint names."""
    if joint_names is None:
        side = "right" if str(hand_side or "left").strip().lower().startswith("r") else "left"
        joint_names = (name.replace("left_hand_", f"{side}_hand_", 1)
                       for name in URDF_JOINT_MAP.values())
    return {name: default_joint_position(name) for name in joint_names}
