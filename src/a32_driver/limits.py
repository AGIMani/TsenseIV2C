"""SDK 1.1.15 bounds shared by planning and hardware-map generation.

Linear caps are the calibrated motor-size curves evaluated at each CAD joint
limit: 90 degrees for the thumb and 1.57 radians for the four fingers.
"""
import math
from a32_kinematics.sdk_stroke import JOINTS, joint_angle_deg_to_actuator

_LINEAR_BOUNDS = {
    joint.destination: (joint_angle_deg_to_actuator(index, math.degrees(joint.lo)),
                        joint_angle_deg_to_actuator(index, math.degrees(joint.hi)))
    for index, joint in enumerate(JOINTS) if joint.linear
}


def sdk_channel_limits(channel, index):
    if channel == "mit":
        return (-180., 180.)
    return _LINEAR_BOUNDS.get(index, (-20., 20.))
