"""统一的全零 init 姿态及切换时序；不从动作文件读取默认姿态。

关节初值唯一由 joint_schema 定义。左右手只更换键名前缀，没有独立姿态表。
"""
from joint_schema import get_init_pose

# 手部消失 → init 的过渡时间（秒）。
INIT_RETURN_TRANSITION_S = 1.0

# 同步开启时：当前姿态 → 新手势首帧。
HAND_RESUME_TRANSITION_S = 0.35

# 手再次出现：init → 画面首帧，再进入跟随。
HAND_REAPPEAR_TRANSITION_S = 0.85

# 手再现完成后：首帧 → 实时跟随首个目标的交接缓冲（秒）。
HAND_REAPPEAR_HANDOVER_S = 0.18
