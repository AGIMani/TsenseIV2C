import test from 'node:test';
import assert from 'node:assert/strict';
import * as THREE from 'three';
import {palmFacingSkeleton, SkeletonViewer} from '../static/viewer.js';

function hand(side) {
  const points = [[0, 0, 0]];
  for (const [x, y] of [[.055, .015], [.033, .065], [.011, .075], [-.011, .075], [-.033, .065]]) {
    for (let joint = 0; joint < 4; joint++) {
      // Non-planar fingers: flexion toward the palm must remain toward the viewer.
      points.push([side === 'right' ? -x : x, y + .018 * joint, .004 * joint * joint]);
    }
  }
  return points;
}
const distance = (a, b) => Math.hypot(...a.map((v, i) => v - b[i]));

for (const side of ['left', 'right']) {
  test(`${side}: fingers point up, palm flexion faces the viewer, dimensions are preserved`, () => {
    const input = hand(side), before = structuredClone(input);
    const output = palmFacingSkeleton(input, side).map(v => v.toArray());
    assert.deepEqual(input, before, 'display transform must not mutate tracking data');
    assert.deepEqual(output[0].map(v => v || 0), [0, 0, 0]);
    for (const base of [5, 9, 13, 17]) {
      assert(output[base + 3][2] > output[base][2], 'fingers should extend upward');
      assert(output[base + 3][1] < output[base][1], 'palm-side flexion should approach the camera');
    }
    assert(side === 'left' ? output[4][0] < 0 : output[4][0] > 0, 'mirror both hands: left thumb screen-left, right thumb screen-right');
    for (let i = 0; i < 21; i++) for (let j = i + 1; j < 21; j++) {
      assert(Math.abs(distance(input[i], input[j]) - distance(output[i], output[j])) < 1e-12);
    }
    for (const rotation of [[.7, -.5, 1.2], [Math.PI, 0, 0], [0, 0, -Math.PI / 2]]) {
      const q = new THREE.Quaternion().setFromEuler(new THREE.Euler(...rotation));
      const moved = input.map(p => new THREE.Vector3(...p).applyQuaternion(q).add(new THREE.Vector3(1, -2, .4)).toArray());
      const oriented = palmFacingSkeleton(moved, side.toUpperCase());
      oriented.forEach((p, i) => assert(p.distanceTo(new THREE.Vector3(...output[i])) < 1e-12));
    }
  });
}

test('missing or degenerate palms do not produce invalid render coordinates', () => {
  assert.equal(palmFacingSkeleton(null), null);
  assert.equal(palmFacingSkeleton([]), null);
  assert.equal(palmFacingSkeleton(Array.from({length: 21}, (_, i) => [0, i, 0])), null);
  const points = hand('left'); points[8][0] = NaN;
  assert.equal(palmFacingSkeleton(points), null);
});

test('default skeleton camera looks straight at the palm with screen-up along Z', () => {
  const camera = new THREE.PerspectiveCamera(34, 1, .001, 20);
  camera.up.set(0, 0, 1);
  const controls = {target: new THREE.Vector3(), update() { camera.lookAt(this.target); }};
  const stage = {camera, controls, center: new THREE.Vector3(0, 0, .045), extent: .18,
    element: {clientWidth: 600, clientHeight: 400}, resize() {}};
  for (const side of ['left', 'right']) {
    SkeletonViewer.prototype.resetView.call(stage, side);
    assert(camera.getWorldDirection(new THREE.Vector3()).distanceTo(new THREE.Vector3(0, 1, 0)) < 1e-12);
    assert(new THREE.Vector3(0, 1, 0).applyQuaternion(camera.quaternion).distanceTo(new THREE.Vector3(0, 0, 1)) < 1e-12);
  }
});
