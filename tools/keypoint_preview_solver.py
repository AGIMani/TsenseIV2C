"""Headless in-memory solver shared by the desktop and browser editors."""
import copy
from time import perf_counter

import numpy as np
from TsenseI_retargeting import TopoRetargeter
from TsenseI_retargeting.fk.types import RobotKeypointSet


class PreviewSolver:
    """Own all mutable solver state independently of the static editor."""

    def __init__(self, config):
        self.solver = TopoRetargeter(copy.deepcopy(config))
        self.signature = None

    def solve(self, points, calibration):
        start = perf_counter()
        points = np.asarray(points, dtype=float)
        if points.shape != (21, 3) or not np.isfinite(points).all():
            raise ValueError("输入必须是 21 个有限的三维坐标")
        if np.linalg.norm(points[5] - points[17]) < 1e-6:
            raise ValueError("输入掌宽过小：请分开 5 和 17 号点")
        signature = tuple((row['id'], row['link'], *row['local_xyz'])
                          for row in calibration['keypoints'])
        solver = self.solver
        if signature != self.signature:
            self.signature = None
            keypoints = RobotKeypointSet.from_dict(calibration)
            zero_poses = solver.fk.model.forward_kinematics({})
            for point in keypoints.keypoints:
                if point.link not in zero_poses or not np.isfinite(point.local_xyz).all():
                    raise ValueError(f"标定点 {point.id} 的连杆或坐标无效")
            solver.fk.keypoints = keypoints
            neutral = solver.fk.compute(solver._q_to_dict(solver.init_q))
            if np.linalg.norm(neutral[5] - neutral[17]) < 1e-6:
                raise ValueError("标定掌宽过小：请分开 5 和 17 号点")
            # Rebuild analytic frame bindings/local offsets BEFORE computing the
            # topology reference frame; otherwise it silently uses old points.
            solver._analytic_kinematics, solver._analytic_kinematics_error = solver._make_analytic_kinematics()
            solver.robot_palm_width_m = solver._robot_palm_width_m()
            solver.robot_topo_frame = solver._robot_topo_frame()
            solver.reset()
            self.signature = signature
        previous = None if solver.previous_q is None else solver.previous_q.copy()
        result = solver.retarget(points, include_artifacts=False)
        q = np.asarray(list(result.joint_values.values()))
        if not result.success or not np.isfinite(q).all():
            raise ValueError("求解未成功；请检查输入姿态与标定位置")
        world = solver.fk.compute(result.joint_values)
        return {
            'joint_values': result.joint_values,
            'points': world,
            'poses': solver.fk.model.forward_kinematics(result.joint_values),
            'delta': float(np.max(np.abs(q - previous))) if previous is not None else float('inf'),
            'elapsed_ms': (perf_counter() - start) * 1000,
            'calibration_signature': self.signature,
        }


