"""A3.0 profile/calibration and nine-linear/eleven-rotary actuation conversion."""
from online_control.online_motion_track.pipeline_v3_2.pipeline import MediaPipeHandPipeline as BasePipeline
from .actuation import A30Actuation


class MediaPipeHandPipeline(BasePipeline):
    def __init__(self, hand_side="left", collision_mode="off", *, model_id="TsenseI_v3_0", **kwargs):
        if collision_mode not in ("off", "monitor", "stop", "mink"):
            raise ValueError("A3.0 碰撞模式应为 off / monitor / stop / mink")
        guard = None
        if collision_mode == "stop":
            from hand_profiles import load_hand_profile
            from .collision import StopCollisionGuard
            guard = StopCollisionGuard(load_hand_profile(model_id), hand_side, finger_stop=True)
        super().__init__(hand_side, collision_mode, model_id=model_id,
                         stroke_mapper=A30Actuation(hand_side, model_id=model_id), collision_guard=guard, **kwargs)
