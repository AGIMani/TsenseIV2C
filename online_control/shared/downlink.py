"""Shared A3.0 motion-playback and gesture-control wire policy; no device I/O."""
from dataclasses import replace
import struct

from a30_kinematics.downlink import _finite_float32, channel_mapping, encode_targets
from utils.motion_limits import configure_ruckig

CONTROL_HZ = 500
CONTROL_DT = 1.0 / CONTROL_HZ


def encode_control_targets(targets, **kwargs):
    """Use nonpositive linear strokes and the shared external gear ratios."""
    # The low-level encoder negates once. Normalize to magnitudes first, exactly
    # as online_motion_play does, without changing the geometry conversion.
    # Negative zero here becomes positive zero after the encoder's sign flip.
    targets = replace(targets, linear_strokes_mm={
        name: abs(_finite_float32(value)) or -0.0
        for name, value in targets.linear_strokes_mm.items()})
    return encode_targets(targets, **kwargs)


def downlink_ranges(mapper):
    """Actual float32 wire endpoints for the mapper's effective joint limits."""
    frames = [encode_control_targets(mapper.convert(dict(zip(mapper.joint_names, values))),
                                     hand_side=mapper.hand_side)
              for values in zip(*mapper._limits)]
    result = {}
    for row in channel_mapping(mapper.hand_side):
        offset = 7 + 4 * (row["slot"] if row["field"] == "self_pos" else 18 + row["slot"])
        values = [struct.unpack_from("<f", frame, offset)[0] for frame in frames]
        result[row["joint"]] = {
            "min": min(values), "max": max(values), "unit": row["unit"],
            "slot": f'{row["field"]}[{row["slot"]}]',
            "at_ui_min": values[0], "at_ui_max": values[1],
        }
    return result
