"""Default joint-space motion limits shared by all hand controllers."""

# rad/s, rad/s², rad/s³; identical for all models, sides and joint types.
DEFAULT_MOTION_LIMITS = (20.0, 400.0, 2400.0)


def configure_ruckig(parameters, names, *,
                     acceleration=DEFAULT_MOTION_LIMITS[1],
                     jerk=DEFAULT_MOTION_LIMITS[2]):
    """Apply shared limits without changing the caller's clock or duration."""
    from ruckig import Synchronization

    parameters.max_velocity = [DEFAULT_MOTION_LIMITS[0]] * len(names)
    parameters.max_acceleration = [acceleration] * len(names)
    parameters.max_jerk = [jerk] * len(names)
    parameters.synchronization = Synchronization.Time
