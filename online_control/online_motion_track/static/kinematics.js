import * as THREE from "three";

export function applyOrigin(object, { xyz, rpy }) {
  object.position.fromArray(xyz);
  // URDF fixed-axis RPY = Rz(yaw) Ry(pitch) Rx(roll).
  object.quaternion.setFromEuler(new THREE.Euler(...rpy, "ZYX"));
}

export function buildRobot(description) {
  const links = new Map(
    description.links.map((link) => {
      const group = new THREE.Group();
      group.name = link.name;
      return [link.name, group];
    }),
  );
  const joints = new Map();
  for (const spec of description.joints) {
    const origin = new THREE.Group();
    applyOrigin(origin, spec);
    const motion = new THREE.Group();
    motion.name = spec.name;
    origin.add(motion);
    motion.add(links.get(spec.child));
    links.get(spec.parent).add(origin);
    joints.set(spec.name, {
      ...spec,
      motion,
      vector: new THREE.Vector3(...spec.axis).normalize(),
    });
  }
  const root = links.get(description.root);
  function setPose(pose) {
    for (const [name, value] of Object.entries(pose)) {
      const joint = joints.get(name);
      if (!joint || joint.type === "fixed" || !Number.isFinite(value)) continue;
      const angle = Math.max(joint.lower, Math.min(joint.upper, value));
      joint.motion.quaternion.setFromAxisAngle(joint.vector, angle);
    }
    root.updateMatrixWorld(true);
  }
  setPose(description.initial_pose);
  return { root, links, joints, setPose };
}
