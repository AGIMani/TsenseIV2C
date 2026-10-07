"""动作文件读写：解析/写出左右手标记与动作字典。

内容均为 JSON（编码 UTF-8，可带 BOM）；扩展名支持 ``.txt`` / ``.json``。

文件格式（dict）：
  {
    "_hand_side": "left" | "right",   # 可选，缺省 left
    "motion_name": { 关节角..., "_transition_s", "_interval_s" },
    ...
  }

也兼容旧版 list：[{"name": "...", ...}, ...]，此时手别默认为 left。
"""
from __future__ import annotations

import json
import os
from typing import Any, Dict, Optional, Tuple

HAND_SIDE_LEFT = "left"
HAND_SIDE_RIGHT = "right"

# 读写均接受：内容是 JSON，扩展名 txt 或 json
MOTION_FILE_EXTENSIONS = (".txt", ".json")
MOTION_OPEN_FILTER = (
    "动作文件 (*.txt *.json);;"
    "文本文件 (*.txt);;"
    "JSON 文件 (*.json);;"
    "所有文件 (*.*)"
)
MOTION_SAVE_FILTER = (
    "文本文件 (*.txt);;"
    "JSON 文件 (*.json);;"
    "所有文件 (*.*)"
)

# 文件级元数据键（不是动作名）
_FILE_META_KEYS = frozenset({
    "_hand_side",
    "hand_side",
    "_meta",
})


def normalize_hand_side(value: Any, *, default: str = HAND_SIDE_LEFT) -> str:
    """将任意表示规范为 'left' / 'right'。"""
    if value is None:
        return default
    s = str(value).strip().lower()
    if s in ("right", "r", "右手", "rh"):
        return HAND_SIDE_RIGHT
    if s in ("left", "l", "左手", "lh"):
        return HAND_SIDE_LEFT
    return default


def extract_hand_side_from_motion_data(
    data: Any, *, default: str = HAND_SIDE_LEFT
) -> str:
    """从动作 JSON 根对象读取左右手。"""
    if not isinstance(data, dict):
        return default
    if "_hand_side" in data:
        return normalize_hand_side(data.get("_hand_side"), default=default)
    if "hand_side" in data:
        return normalize_hand_side(data.get("hand_side"), default=default)
    meta = data.get("_meta")
    if isinstance(meta, dict):
        if "_hand_side" in meta:
            return normalize_hand_side(meta.get("_hand_side"), default=default)
        if "hand_side" in meta:
            return normalize_hand_side(meta.get("hand_side"), default=default)
    return default


def parse_motion_file_data(
    data: Any, *, default_hand_side: str = HAND_SIDE_LEFT
) -> Optional[Tuple[Dict[str, dict], str]]:
    """解析动作文件 → (motions, hand_side)；格式非法返回 None。"""
    hand_side = extract_hand_side_from_motion_data(
        data, default=default_hand_side
    )
    if isinstance(data, dict):
        motions: Dict[str, dict] = {}
        for key, value in data.items():
            if key in _FILE_META_KEYS:
                continue
            if not isinstance(value, dict):
                continue
            # 跳过纯元数据对象
            if key.startswith("_") and not any(
                k.startswith("left_hand_") or k.startswith("right_hand_")
                for k in value
            ):
                # 允许动作名以 _ 开头且含关节；否则若无关节键则跳过
                joint_like = any(
                    isinstance(k, str)
                    and (k.startswith("left_hand_") or k.startswith("right_hand_"))
                    for k in value
                )
                if not joint_like and "_transition_s" not in value and "_interval_s" not in value:
                    continue
            motions[key] = dict(value)
        return motions, hand_side
    if isinstance(data, list):
        motions = {}
        for i, item in enumerate(data):
            if not isinstance(item, dict):
                continue
            name = item.get("name", f"动作{i + 1}")
            motions[str(name)] = dict(item)
        return motions, hand_side
    return None


def build_motion_file_payload(
    motions: Dict[str, dict], hand_side: str
) -> Dict[str, Any]:
    """写出动作文件根对象：先写 _hand_side，再写各动作（保序）。"""
    payload: Dict[str, Any] = {
        "_hand_side": normalize_hand_side(hand_side),
    }
    for name, motion in motions.items():
        if name in _FILE_META_KEYS:
            continue
        payload[name] = motion
    return payload


def is_supported_motion_path(path: str) -> bool:
    """是否为已登记的动作文件扩展名（空扩展名视为可用，便于兼容无后缀路径）。"""
    if not path:
        return False
    ext = os.path.splitext(path)[1].lower()
    return ext in MOTION_FILE_EXTENSIONS or ext == ""


def ensure_motion_save_extension(path: str, selected_filter: str = "") -> str:
    """另存为时补全扩展名：优先选中过滤器，否则默认 .txt。"""
    if not path:
        return path
    _root, ext = os.path.splitext(path)
    if ext.lower() in MOTION_FILE_EXTENSIONS:
        return path
    filt = (selected_filter or "").lower()
    if "*.json" in filt and "*.txt" not in filt:
        return f"{path}.json"
    if "*.txt" in filt and "*.json" not in filt:
        return f"{path}.txt"
    return f"{path}.txt"


def load_motion_json(path: str) -> Any:
    """从 .txt / .json 读取 JSON（utf-8-sig，兼容 BOM）。"""
    with open(path, "r", encoding="utf-8-sig") as f:
        return json.load(f)


def save_motion_json(path: str, payload: Dict[str, Any]) -> None:
    """写出 JSON 到 .txt 或 .json（UTF-8，缩进）。"""
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)
        f.write("\n")
