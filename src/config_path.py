"""项目文件路径配置（单一来源）。

所有模块应从此处导入路径常量，避免在代码中硬编码。
"""
from pathlib import Path

# 项目根目录（src 的上一级）
PROJECT_ROOT = Path(__file__).resolve().parent.parent

# ── 手部模型资源 assets/hands/TsenseI_hand ──
ASSETS_ROOT = PROJECT_ROOT / "assets"
HANDS_DIR = ASSETS_ROOT / "hands"
HAND_MODEL_DIR = HANDS_DIR / "TsenseI_hand"

URDF_DIR = HAND_MODEL_DIR / "urdf"
MESHES_DIR = HAND_MODEL_DIR / "meshes"
MJCF_DIR = HAND_MODEL_DIR / "mjcf"
COLLISION_MESHES_DIR = MJCF_DIR / "collision_meshes"

URDF_FILENAME = "TsenseI__hand_left_v1.urdf"
URDF_FILENAME_RIGHT = "TsenseI__hand_right_v1.urdf"
MJCF_FILENAME = "mjmodel.xml"

DEFAULT_URDF_PATH = URDF_DIR / URDF_FILENAME
RIGHT_URDF_PATH = URDF_DIR / URDF_FILENAME_RIGHT
MJCF_MODEL_PATH = MJCF_DIR / MJCF_FILENAME

# 相对 PROJECT_ROOT 的路径（供 yaml、pinocchio 等）
URDF_REL_PATH = DEFAULT_URDF_PATH.relative_to(PROJECT_ROOT).as_posix()
RIGHT_URDF_REL_PATH = RIGHT_URDF_PATH.relative_to(PROJECT_ROOT).as_posix()
HAND_MODEL_REL_PATH = HAND_MODEL_DIR.relative_to(PROJECT_ROOT).as_posix()

# Pinocchio RobotWrapper package_dirs（重定向 FK 用；运行时碰撞为 MuJoCo 胶囊）
# 遗留：collision_meshes/ 由 tools/generate_hand_mjcf.py 生成，非运行时路径
PINOCCHIO_PACKAGE_DIRS = [str(HAND_MODEL_DIR)]

# ── 重定向配置 ──
CONFIG_DIR = PROJECT_ROOT / "config"
RETARGETING_CONFIG_PATH = CONFIG_DIR / "left_hand_TsenseI.yml"

# ── MediaPipe 手部关键点模型 ──
HAND_LANDMARKER_PATH = PROJECT_ROOT / "hand_landmarker.task"

# ── 动作序列文件 ──
MOTION_FILE_PATH = PROJECT_ROOT / "motion.txt"

# ── 日志目录 ──
LOGS_DIR = PROJECT_ROOT / "logs"

HAND_PICTURE_DIR = HAND_MODEL_DIR / "picture"
HAND_RIGHT_SNAPSHOT_PATH = HAND_PICTURE_DIR / "right_snapshot.png"
