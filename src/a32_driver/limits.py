"""SDK 1.1.15 bounds shared by planning and hardware-map generation.

Linear caps come from the calibrated motor-size table, including the medium
16.95 mm thumb MCP (slot 17). The application's swing range is narrower than
the SDK's +/-20 degree cap and is applied separately.
"""
from a32_kinematics.sdk_stroke import JOINTS

_LINEAR_BOUNDS = {
    joint.destination: (joint.linkage.stroke_lo * 1000., joint.linkage.stroke_hi * 1000.)
    for joint in JOINTS if joint.linear
}


def sdk_channel_limits(channel, index):
    if channel == "mit":
        return (-180., 180.)
    return _LINEAR_BOUNDS.get(index, (-20., 20.))
