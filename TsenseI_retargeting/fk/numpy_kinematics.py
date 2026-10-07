"""Analytic serial/tree keypoint Jacobians without a robotics native runtime."""
from __future__ import annotations
import numpy as np


class NumpyKeypointKinematics:
    def __init__(self, urdf_path, keypoints, joint_names, urdf_model):
        del urdf_path
        self.model = urdf_model
        self.names = list(joint_names)
        self.links = list(urdf_model.links)
        indices = {name: i for i, name in enumerate(self.links)}
        self.base = indices[urdf_model.base_link]
        controlled = {name: i for i, name in enumerate(self.names)}
        dependencies = {}

        def dependency(name, visited=()):
            if name in visited:
                raise ValueError("Cyclic mimic dependency")
            if name in dependencies:
                return dependencies[name]
            joint = urdf_model.joint_by_name[name]
            vector, offset = np.zeros(len(self.names)), 0.
            if name in controlled:
                if joint.mimic_joint:
                    raise ValueError("Controlled joints cannot be mimic joints")
                vector[controlled[name]] = 1.
            elif joint.mimic_joint:
                parent, bias = dependency(joint.mimic_joint, (*visited, name))
                vector = parent * joint.mimic_multiplier
                offset = bias * joint.mimic_multiplier + joint.mimic_offset
            dependencies[name] = vector, offset
            return vector, offset

        self.joints = []
        chain = {urdf_model.base_link: []}
        queue = [urdf_model.base_link]
        while queue:
            parent = queue.pop(0)
            for joint in urdf_model.children_by_parent.get(parent, []):
                if joint.type not in ("fixed", "revolute", "continuous", "prismatic"):
                    raise ValueError(f"Unsupported joint: {joint.type}")
                chain[joint.child] = chain[parent] + [len(self.joints)]
                self.joints.append(joint)
                queue.append(joint.child)
        if set(controlled) - {j.name for j in self.joints}:
            raise ValueError("Controlled joint missing from URDF tree")
        self.parent = [indices[j.parent] for j in self.joints]
        self.child = [indices[j.child] for j in self.joints]
        self.axes = np.array([j.axis for j in self.joints], dtype=float)
        norms = np.linalg.norm(self.axes, axis=1)
        self.axes /= np.where(norms > 0, norms, 1)[:, None]
        self.dependencies = np.array([dependency(j.name)[0] for j in self.joints])
        self.offsets = np.array([dependency(j.name)[1] for j in self.joints])
        self.skew = np.zeros((len(self.joints), 3, 3))
        for i, (x, y, z) in enumerate(self.axes):
            self.skew[i] = [[0, -z, y], [z, 0, -x], [-y, x, 0]]
        self.skew2 = self.skew @ self.skew
        kp = sorted(keypoints.keypoints, key=lambda p: p.id)
        if [p.id for p in kp] != list(range(len(kp))):
            raise ValueError("Noncontiguous keypoint ids")
        self.frames = np.array([indices[p.link] for p in kp])
        self.local = np.array([p.local_xyz for p in kp])
        self.mask = np.zeros((len(kp), len(self.joints)))
        for i, point in enumerate(kp):
            self.mask[i, chain[point.link]] = 1.
        self.prismatic = np.array([j.type == "prismatic" for j in self.joints])
        self.revolute = np.array([j.type in ("revolute", "continuous") for j in self.joints])
        self.origins = np.array([j.origin for j in self.joints])
        self.parent = np.asarray(self.parent)
        self.child = np.asarray(self.child)
        # All nodes in one depth depend only on the previous depth. Evaluate
        # the five finger trees in batches, without changing their geometry.
        depths = {self.base: 0}
        levels = {}
        for i, (parent, child) in enumerate(zip(self.parent, self.child)):
            depths[child] = depths[parent]+1
            levels.setdefault(depths[child], []).append(i)
        self.levels = [np.asarray(v) for _, v in sorted(levels.items())]

    def _evaluate(self, q, jacobian):
        values = np.asarray(q, dtype=float).reshape(-1)
        if len(values) != len(self.names):
            raise ValueError("Wrong joint vector size")
        angles = self.dependencies @ values + self.offsets
        transforms = np.repeat(np.eye(4)[None], len(self.links), axis=0)
        rotations = (np.eye(3)[None] + np.sin(angles)[:, None, None] * self.skew
                     + (1 - np.cos(angles))[:, None, None] * self.skew2)
        motion = np.repeat(np.eye(4)[None], len(self.joints), axis=0)
        motion[self.revolute, :3, :3] = rotations[self.revolute]
        motion[self.prismatic, :3, 3] = self.axes[self.prismatic] * angles[self.prismatic, None]
        local = self.origins @ motion
        for level in self.levels:
            transforms[self.child[level]] = transforms[self.parent[level]] @ local[level]
        frames = transforms[self.frames]
        points = np.einsum('pij,pj->pi', frames[:, :3, :3], self.local) + frames[:, :3, 3]
        if not jacobian:
            return points
        zero = transforms[self.parent] @ self.origins
        axes = (zero[:, :3, :3] @ self.axes[:, :, None])[:, :, 0]
        origins = zero[:, :3, 3]
        derivatives = np.cross(axes[None], points[:, None] - origins[None])
        derivatives[:, self.prismatic] = axes[self.prismatic]
        derivatives *= self.mask[:, :, None]
        return points, np.einsum('pjc,jn->pcn', derivatives, self.dependencies)

    def compute_points(self, q):
        return self._evaluate(q, False)

    def compute_points_and_jacobian(self, q):
        return self._evaluate(q, True)
