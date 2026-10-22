"""当前项目专用的下发角度补偿，在碰撞检查和轨迹规划前应用一次。

本文件的关节倍率仅适用于当前项目的网页手势控制，不属于 SDK 的通用行为。
其他项目引用、集成或复用 SDK 时，应移除本补偿模块及其调用，使用未乘倍率的
关节目标角；不要将这些倍率迁入 SDK 驱动、角度/行程转换或公共下发接口。
移除补偿时仍须保留关节限位、碰撞检查及轨迹规划等原有处理。
"""
import math

INDEX_PIP_GAIN = 1.1
MCP_GAIN = 1.1
INDEX_MCP_GAIN = 1.2
INDEX_DIP_GAIN = 1.2
PINKY_DIP_GAIN = 0.8


def compensate_command_pose(pose, limits):
    """Apply per-joint command gains once in joint space, retaining model limits."""
    target = dict(pose)
    for side in ("left", "right"):
        gains = {f"{side}_hand_{finger}_MCP_joint": MCP_GAIN
                 for finger in ("thumb", "index", "mid", "ring", "pinky")}
        gains[f"{side}_hand_index_PIP_joint"] = INDEX_PIP_GAIN
        gains[f"{side}_hand_index_MCP_joint"] = INDEX_MCP_GAIN
        gains[f"{side}_hand_index_DIP_joint"] = INDEX_DIP_GAIN
        gains[f"{side}_hand_pinky_DIP_joint"] = PINKY_DIP_GAIN
        for name, gain in gains.items():
            if name not in target:
                continue
            angle = float(target[name])
            if not math.isfinite(angle):
                raise ValueError(f"Non-finite command angle: {name}")
            lo, hi = limits[name]
            target[name] = min(hi, max(lo, angle * gain))
    return target
