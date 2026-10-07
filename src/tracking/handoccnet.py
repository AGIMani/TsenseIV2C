"""HandOccNet 推理后端封装。

将 HandOccNet 的 MANO 21 关节输出转换为与 MediaPipe 兼容的
(norm_landmarks, world_landmarks) 格式，供 skeleton_tracker 统一处理。

依赖:
  - PyTorch
  - 克隆 https://github.com/namepllet/HandOccNet 并下载 snapshot_demo.pth.tar
  - MANO_RIGHT.pkl 放到 HandOccNet/common/utils/manopth/mano/models/

路径通过 tracker_config.py 或环境变量 HANDOCCNET_ROOT / HANDOCCNET_SNAPSHOT 配置。
"""

from __future__ import annotations

import os
import sys
import threading
import traceback
from typing import List, Optional, Tuple

import cv2
import numpy as np

from utils.log_control import emit as log_emit
from utils.tracker_config import HANDOCCNET_GPU, HANDOCCNET_ROOT, HANDOCCNET_SNAPSHOT


def _log(msg: str, tag: str = "HandOccNet"):
    log_emit(msg, tag)


class _Lm:
    """与 MediaPipe Landmark 兼容的简单关键点容器。"""

    __slots__ = ("x", "y", "z")

    def __init__(self, x: float, y: float, z: float):
        self.x, self.y, self.z = float(x), float(y), float(z)


def _affine_pts(trans: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """2D 仿射变换: pts (N,2) + trans (2,3) -> (N,2)。"""
    return pts @ trans[:, :2].T + trans[:, 2]


def _default_bbox(img_w: int, img_h: int) -> np.ndarray:
    size = min(img_w, img_h) * 0.55
    cx, cy = img_w * 0.5, img_h * 0.5
    return np.array([cx - size * 0.5, cy - size * 0.5, size, size], dtype=np.float32)


class HandOccNetBackend:
    """HandOccNet 单例后端。"""

    def __init__(self) -> None:
        self._model = None
        self._device = None
        self._transform = None
        self._cfg = None
        self._initialized = False
        self._init_error = ""
        self._bbox: Optional[np.ndarray] = None
        self._root = HANDOCCNET_ROOT
        self._snapshot = HANDOCCNET_SNAPSHOT

    @property
    def is_available(self) -> bool:
        return self._initialized and self._model is not None

    @property
    def init_error(self) -> str:
        return self._init_error

    def init(self, root: str = "", snapshot: str = "", gpu: str = "") -> bool:
        """加载 HandOccNet 模型。失败时返回 False 并记录 init_error。"""
        self._initialized = False
        self._model = None
        self._init_error = ""

        root = root or self._root
        snapshot = snapshot or self._snapshot
        gpu = gpu or HANDOCCNET_GPU

        if not os.path.isdir(root):
            self._init_error = f"HandOccNet 目录不存在: {root}"
            _log(self._init_error)
            return False
        if not os.path.isfile(snapshot):
            self._init_error = f"权重文件不存在: {snapshot}"
            _log(self._init_error)
            return False

        main_dir = os.path.join(root, "main")
        common_dir = os.path.join(root, "common")
        if main_dir not in sys.path:
            sys.path.insert(0, main_dir)
        if common_dir not in sys.path:
            sys.path.insert(0, common_dir)

        try:
            import torch
            import torchvision.transforms as transforms
            from torch.nn.parallel.data_parallel import DataParallel

            from config import cfg
            from model import get_model
            from utils.preprocessing import generate_patch_image, process_bbox

            self._process_bbox = process_bbox
            self._generate_patch = generate_patch_image
            self._cfg = cfg
            cfg.set_args(gpu)

            use_cuda = torch.cuda.is_available() and gpu != "-1"
            self._device = torch.device("cuda" if use_cuda else "cpu")
            if not use_cuda:
                _log("CUDA 不可用，HandOccNet 将在 CPU 上运行（较慢）")

            model = get_model("test")
            if use_cuda:
                model = DataParallel(model).cuda()
            else:
                model = model.to(self._device)

            ckpt = torch.load(snapshot, map_location=self._device)
            model.load_state_dict(ckpt["network"], strict=False)
            model.eval()

            self._model = model
            self._transform = transforms.ToTensor()
            self._root = root
            self._snapshot = snapshot
            self._bbox = None
            self._initialized = True
            _log(f"模型加载成功: {snapshot}")
            return True

        except Exception as e:
            self._init_error = str(e)
            _log(f"初始化失败: {e}\n{traceback.format_exc()}")
            return False

    def reset_tracking(self) -> None:
        self._bbox = None

    def detect(
        self, color_bgr: np.ndarray, timestamp_ms: float = 0.0
    ) -> Tuple[Optional[List[_Lm]], Optional[List[_Lm]]]:
        """推理单帧，返回 (norm_landmarks, world_landmarks)，均为 21 个 _Lm。"""
        if not self.is_available:
            return None, None

        import torch

        img_rgb = cv2.cvtColor(color_bgr, cv2.COLOR_BGR2RGB).astype(np.float32)
        img_h, img_w = img_rgb.shape[:2]

        bbox = self._bbox
        if bbox is None:
            bbox = _default_bbox(img_w, img_h)
        bbox = self._process_bbox(bbox, img_w, img_h, expansion_factor=1.25)
        if bbox is None:
            bbox = _default_bbox(img_w, img_h)
            bbox = self._process_bbox(bbox, img_w, img_h, expansion_factor=1.25)
        if bbox is None:
            return None, None

        patch, _img2bb, bb2img = self._generate_patch(
            img_rgb, bbox, 1.0, 0.0, False, self._cfg.input_img_shape
        )
        tensor = self._transform(patch.astype(np.float32)) / 255.0
        tensor = tensor.to(self._device)[None, :, :, :]

        with torch.no_grad():
            out = self._model({"img": tensor}, {}, {}, "test")

        joints_cam = out["joints_coord_cam"][0].detach().cpu().numpy()
        joints_img = out["joints_coord_img"][0].detach().cpu().numpy()

        if joints_cam.shape[0] != 21:
            return None, None

        joints_2d = joints_img[:, :2] if joints_img.shape[1] >= 2 else joints_img
        pts_full = _affine_pts(bb2img, joints_2d)

        # 更新 bbox 供下一帧跟踪
        joint_valid = np.ones(21, dtype=np.float32)
        try:
            from utils.preprocessing import get_bbox

            new_bbox = get_bbox(pts_full, joint_valid, expansion_factor=1.25)
            processed = self._process_bbox(new_bbox, img_w, img_h, expansion_factor=1.0)
            if processed is not None:
                self._bbox = processed
        except Exception:
            self._bbox = bbox

        # 世界坐标：腕部为原点（与 MediaPipe world 一致，供 retargeting 使用）
        world = joints_cam - joints_cam[0:1]
        world_lms = [_Lm(p[0], p[1], p[2]) for p in world]

        # 归一化图像坐标（0~1），z 用相对深度
        norm_lms = []
        for i, (px, py) in enumerate(pts_full):
            z_rel = float(world[i, 2])
            norm_lms.append(_Lm(px / img_w, py / img_h, z_rel))

        return norm_lms, world_lms


_backend = HandOccNetBackend()


def get_handoccnet_backend() -> HandOccNetBackend:
    return _backend
