"""Shared A3 gesture collision and joint-limit recovery policy."""
import math

import ruckig
from a30_kinematics.mapper import JOINT_LIMIT_EPS_RAD


class SafeTrajectoryMixin:
    def _finger_joints(self, finger):
        return [n for n in self.names if n.startswith(f'{self.side}_hand_{finger}_')]

    def _remember_collision_hold(self, finger):
        names = set(self._finger_joints(finger))
        self.finger_holds[finger] = {n: self.target[n] for n in names}
        # Only accepted command positions count, not desired positions that
        # another collision may prevent a neighbouring finger from reaching.
        self.finger_hold_context[finger] = {n: q for n, q in self.pose.items() if n not in names}

    def _stop_joints(self, names, *, hold=False):
        positions = list(self.input.current_position)
        velocity = list(self.input.current_velocity)
        acceleration = list(self.input.current_acceleration)
        target = list(self.input.target_position)
        for n in names:
            i = self.names.index(n)
            positions[i] = self.pose[n]
            velocity[i] = acceleration[i] = 0.
            if hold:
                target[i] = self.pose[n]
        self.input.current_position = positions
        self.input.current_velocity = velocity
        self.input.current_acceleration = acceleration
        self.input.target_position = target

    def _collision_hold_fingers(self, candidate, fingers):
        """Let a separating finger move if holding its partner is sufficient.

        A contact names both bodies, not the body whose motion is unsafe.
        Freezing both can deadlock a retreat at the contact boundary. Probe
        whole-finger holds against the same accepted starting pose; the chosen
        hold is then replanned and checked again by _checked_step.
        """
        available = fingers - set(self.finger_holds)
        if len(available) < 2:
            return available
        # Prefer preserving the larger requested movement. At most five path
        # probes; never relax the penetration threshold to escape a contact.
        ordered = sorted(available, key=lambda f: (
            sum((candidate[n] - self.pose[n]) ** 2 for n in self._finger_joints(f)), f))
        for finger in ordered:
            trial = dict(candidate)
            for n in self._finger_joints(finger):
                trial[n] = self.pose[n]
            self.guard.current = dict(self.pose)
            safe = self.guard.project(trial)
            if all(abs(safe[n] - trial[n]) <= 1e-8 for n in self.names):
                return {finger}
        return available

    def _checked_step(self, dt):
        # Replan before publishing, rather than clipping an already-generated
        # sample while silently retaining its old Ruckig derivatives.
        reset_limits = set()
        contacts = []
        for _ in range(len(self.names)+7):
            self.otg.delta_time = dt
            result = self.otg.update(self.input, self.output)
            if result not in (ruckig.Result.Working, ruckig.Result.Finished):
                raise ValueError(f"Ruckig failed: {result}")
            pose = dict(zip(self.names, self.output.new_position))
            if any(not math.isfinite(v) for v in pose.values()):
                raise ValueError("Ruckig 输出非有限值")
            # Check the whole remaining trajectory, including braking extrema.
            # A legal target alone does not imply a legal intermediate path.
            violations = []
            for n, extrema in zip(self.names, self.output.trajectory.position_extrema):
                lo, hi = self.limits[n]
                if (extrema.min < lo-JOINT_LIMIT_EPS_RAD or extrema.max > hi+JOINT_LIMIT_EPS_RAD
                        or not lo-JOINT_LIMIT_EPS_RAD <= pose[n] <= hi+JOINT_LIMIT_EPS_RAD):
                    violations.append(n)
            if violations:
                self.last_limit_event = {"frame_id": self.frame_id, "dt_s": dt, "joints": [
                    {"joint": n, "sample_rad": pose[n], "lower_rad": self.limits[n][0],
                     "trajectory_min_rad": self.output.trajectory.position_extrema[self.names.index(n)].min,
                     "trajectory_max_rad": self.output.trajectory.position_extrema[self.names.index(n)].max,
                     "upper_rad": self.limits[n][1], "velocity_rad_s": self.input.current_velocity[self.names.index(n)],
                     "acceleration_rad_s2": self.input.current_acceleration[self.names.index(n)]}
                    for n in violations]}
                self.limit_recoveries += 1
                for n in violations:
                    # First retry from rest toward the bounded target. If a
                    # solver still cannot do that, hold just this joint.
                    self._stop_joints([n], hold=n in reset_limits)
                reset_limits.update(violations)
                continue
            # All samples/extrema have passed the shared tolerance above. Snap
            # only numerical boundary noise BEFORE collision checking, and
            # commit that same position to Ruckig, the UI and the wire encoder.
            # Real overshoot must go through the replan branch, never clipping.
            roundoffs = []
            for n, (lo, hi) in self.limits.items():
                q = pose[n]
                bounded = min(hi, max(lo, q))
                if bounded != q:
                    pose[n] = bounded
                    roundoffs.append({"joint": n, "raw_rad": q, "applied_rad": bounded})
            if self.guard:
                self.guard.current = dict(self.pose)
                safe = self.guard.project(pose)
                info = dict(self.guard.diagnostics)
                if any(abs(safe[n]-pose[n]) > 1e-8 for n in self.names):
                    contacts.extend(info.get('contacts', []))
                    fingers = self.guard.guard._contact_fingers(info.get('contacts', []))
                    if not fingers:
                        fingers = {'thumb', 'index', 'mid', 'ring', 'pinky'}
                    new_fingers = self._collision_hold_fingers(pose, fingers)
                    if not new_fingers:
                        # Unknown/interacting contact: conservatively extend
                        # the hold and recheck the complete combined pose.
                        new_fingers = {'thumb', 'index', 'mid', 'ring', 'pinky'}-set(self.finger_holds)
                    if not new_fingers:
                        raise ValueError("碰撞检查无法确认保持姿态安全")
                    for finger in new_fingers:
                        names = self._finger_joints(finger)
                        self._remember_collision_hold(finger)
                        self._stop_joints(names, hold=True)
                    continue
                self.sample_collision = {**info, "limited": bool(self.finger_holds),
                    "resolution": "finger_hold" if self.finger_holds else "pass",
                    "held_fingers": sorted(self.finger_holds),
                    "contacts": contacts or (self.sample_collision.get('contacts', []) if self.finger_holds else [])}
            if roundoffs:
                self.output.new_position = [pose[n] for n in self.names]
                self.boundary_roundoffs += 1
                self.last_boundary_roundoff = {"frame_id": self.frame_id, "joints": roundoffs}
            return pose
        raise ValueError("安全轨迹重规划未收敛")

