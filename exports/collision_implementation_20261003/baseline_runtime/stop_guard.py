"""A3.0 stop policy reuses V2 joint-path checking with its own MJCF."""
import time
from online_control.online_motion_track.pipeline_v2.collision_guard import SelfCollisionGuard


class StopCollisionGuard:
    def __init__(self, profile, side, *, finger_stop=False):
        self.finger_stop = finger_stop
        self.side = side
        self.current = profile.init_pose(side)
        self.filter = profile.create_collision_filter(side)
        self.guard = SelfCollisionGuard(
            physics_filter=self.filter, strategy="stop", allowed_penetration_mm=0.0,
            path_step_rad=.025, max_path_steps=128, bisection_iterations=14)
        self.filter.sync_pose_from_dict(self.current)
        if self.filter.data.ncon:
            raise ValueError("A3.0 初始姿态碰撞，请检查碰撞模型")
        self.diagnostics = {"mode": "stop", "limited": False}

    def project(self, pose):
        start = time.perf_counter()
        result, info = self.guard.project(pose, current_values=self.current)
        held = set()
        contacts = list(info.get('contacts', []))
        if self.finger_stop:
            candidate = dict(pose)
            for _ in range(5):
                if not info.get('limited'):
                    break
                fingers = self.guard._contact_fingers(info.get('contacts', []))
                added = fingers-held
                if not added:
                    added = {'thumb', 'index', 'mid', 'ring', 'pinky'}-held
                held.update(added)
                for n in candidate:
                    if any(n.startswith(f'{self.side}_hand_{f}_') for f in held):
                        candidate[n] = self.current[n]
                result, info = self.guard.project(candidate, current_values=self.current)
                contacts.extend(info.get('contacts', []))
            info = {**info, 'limited': bool(held), 'held_fingers': sorted(held),
                    'contacts': contacts, 'resolution': 'finger_hold' if held else 'pass'}
        self.current = dict(result)
        self.diagnostics = {**info, "mode": "stop", "solve_ms": (time.perf_counter() - start) * 1000}
        return result
