"""手部追踪模型切换配置。

修改 DEFAULT_TRACKER_BACKEND 即可在 WiLoR / MediaPipe / HandOccNet 之间切换。
也可通过环境变量 HAND_TRACKER_BACKEND 覆盖（无需改代码）。

可选值:
  "wilor"       — WiLoR-mini（可选，3D 手姿态）
  "mediapipe"   — Google MediaPipe HandLandmarker（轻量备用）
  "handoccnet"  — HandOccNet（遮挡鲁棒，需 PyTorch + 外部仓库）
  "egohand_icl" — EgoHandICL（预留）
"""

import os

# ── 模型切换变量（改这里即可）──
DEFAULT_TRACKER_BACKEND = "mediapipe"

# 环境变量覆盖，便于部署脚本切换
if os.environ.get("HAND_TRACKER_BACKEND"):
    DEFAULT_TRACKER_BACKEND = os.environ["HAND_TRACKER_BACKEND"].strip().lower()

# ── WiLoR（可选后端）──
# pip install git+https://github.com/warmshao/WiLoR-mini
# GPU：必须安装 CUDA 版 PyTorch（当前若是 torch*+cpu 则只会跑 CPU）
#   pip install --force-reinstall torch torchvision --index-url https://download.pytorch.org/whl/cu118
# 启动前可设：
#   set WILOR_DEVICE=cuda
#   set WILOR_DTYPE=float16
WILOR_DEVICE = os.environ.get("WILOR_DEVICE", "").strip()  # "" → auto cuda/cpu
WILOR_DTYPE = os.environ.get("WILOR_DTYPE", "float16").strip()
# True：优先选左手（本项目默认左手 URDF）
WILOR_PREFER_LEFT = os.environ.get("WILOR_PREFER_LEFT", "1").strip().lower() not in (
    "0",
    "false",
    "no",
)
# HuggingFace 镜像（WiLoR 首次下载权重）；可被环境变量覆盖
if not os.environ.get("HF_ENDPOINT"):
    os.environ["HF_ENDPOINT"] = "https://hf-mirror.com"

# ── HandOccNet 路径（使用 handoccnet 后端时需配置）──
# 默认假设 HandOccNet 克隆在 hand 项目同级目录
try:
    from config_path import PROJECT_ROOT
except ModuleNotFoundError:
    from src.config_path import PROJECT_ROOT

_PROJECT_DIR = str(PROJECT_ROOT)
HANDOCCNET_ROOT = os.environ.get(
    "HANDOCCNET_ROOT",
    os.path.abspath(os.path.join(_PROJECT_DIR, "..", "HandOccNet")),
)
HANDOCCNET_SNAPSHOT = os.environ.get(
    "HANDOCCNET_SNAPSHOT",
    os.path.join(HANDOCCNET_ROOT, "demo", "snapshot_demo.pth.tar"),
)
HANDOCCNET_GPU = os.environ.get("HANDOCCNET_GPU", "0")
