"""Display the retarget solution's 21 calibrated landmarks in metres."""
import numpy as np


def optimized_skeleton(solver, joint_values, *, shared_left_right=False):
    if joint_values is None:
        return None
    # Evaluate the explicit solution: collision handling may have already seeded
    # solver.previous_q with a different pose. Reuse its lightweight FK backend.
    pose = ({name.replace("right_hand_", "left_hand_", 1): value
             for name, value in joint_values.items()}
            if shared_left_right else joint_values)
    q = np.array([pose[name] for name in solver.joint_names], dtype=float)
    points = solver._robot_points_palm_m(q).copy()
    if shared_left_right:
        # Inverse of V2's right-hand reflection into its shared left solver.
        points[:, 0] *= -1
    return points
