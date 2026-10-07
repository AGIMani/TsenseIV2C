"""A3 stop policy with cached kinematic checks and the shared path policy."""
import time
from online_control.online_motion_track.pipeline_v2.collision_guard import SelfCollisionGuard
from online_control.online_motion_track.collision_queries import KinematicContactChecker


class _KinematicStopGuard(SelfCollisionGuard):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # Controllers rebase their seed pose before the first projected frame.
        self.begin_frame()

    def begin_frame(self):
        self._frame_cache = {}
        self._cache_hits = 0
        self._filter.reset_counters()

    def _evaluate(self, pose):
        key = tuple(float(pose[name]) for name in self._filter.names)
        cache = self._frame_cache
        if key in cache:
            self._cache_hits += 1
            return cache[key]
        result = super()._evaluate(pose)
        cache[key] = result
        return result

    def rebase(self, current_values):
        self.begin_frame()
        super().rebase(current_values)
        if current_values:
            self._filter.sync_pose_from_dict(self._pose(current_values))

    def reset(self):
        super().reset()
        self.begin_frame()


class StopCollisionGuard:
    def __init__(self, profile, side, *, finger_stop=False):
        self.finger_stop = finger_stop
        self.side = side
        self.current = profile.init_pose(side)
        self.filter = KinematicContactChecker(profile.create_collision_filter(side))
        self.guard = _KinematicStopGuard(
            physics_filter=self.filter, strategy="stop", allowed_penetration_mm=0.0,
            path_step_rad=.025, max_path_steps=128, bisection_iterations=14)
        self.filter.sync_pose_from_dict(self.current)
        if self.filter.data.ncon:
            raise ValueError("A3.0 初始姿态碰撞，请检查碰撞模型")
        self.diagnostics = {"mode": "stop", "limited": False}

    def project(self, pose):
        start = time.perf_counter()
        self.guard.begin_frame()
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
        # A cache hit only returns a previous query result; restore the exposed
        # filter state to the published pose for control/rebase callers.
        self.filter.sync_pose_from_dict(result)
        self.diagnostics = {**info, "mode": "stop", "solve_ms": (time.perf_counter() - start) * 1000,
                            "kinematics_calls": self.filter.kinematics_calls,
                            "collision_calls": self.filter.collision_calls,
                            "cache_hits": self.guard._cache_hits}
        return result
