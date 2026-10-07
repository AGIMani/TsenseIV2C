import {createAluminumMaterial, createAluminumEnvironment} from "../../shared/aluminum.js";
import {isA30Model, A30_MODEL_IDS} from "../../shared/hand-models.js";
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { STLLoader } from "three/addons/loaders/STLLoader.js";
import { applyOrigin, buildRobot } from "./kinematics.js";

function disposeTree(root) {
  if (!root) return;
  const geometries = new Set(),
    materials = new Set();
  root.traverse((object) => {
    if (object.geometry) geometries.add(object.geometry);
    if (object.material) materials.add(object.material);
  });
  for (const geometry of geometries) geometry.dispose();
  for (const material of materials) material.dispose();
  root.removeFromParent();
}

export class Stage {
  constructor(element) {
    this.element = element;
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(34, 1, 0.001, 20);
    this.camera.up.set(0, 0, 1);
    this.renderer = new THREE.WebGLRenderer({
      antialias: true,
      alpha: true,
      powerPreference: "high-performance",
    });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 1.5));
    this.renderer.setClearColor(0, 0);
    this.renderer.outputColorSpace = THREE.SRGBColorSpace;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.05;
    element.prepend(this.renderer.domElement);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.12;
    this.controls.minDistance = 0.08;
    this.controls.maxDistance = 2;
    this.scene.add(new THREE.HemisphereLight(0xf6fbff, 0x627e8b, 1.4));
    const key = new THREE.DirectionalLight(0xf4fbff, 2.6);
    key.position.set(-0.5, -0.6, 1.2);
    this.scene.add(key);
    const fill = new THREE.DirectionalLight(0xcdddf7, 1.5);
    fill.position.set(0.5, 0.4, 0.5);
    this.scene.add(fill);
    this.grid = new THREE.GridHelper(0.7, 28, 0xc5dce5, 0xdceaf0);
    this.grid.rotation.x = Math.PI / 2;
    this.grid.material.transparent = true;
    this.grid.material.opacity = 0.38;
    this.scene.add(this.grid);
    this.resizeObserver = new ResizeObserver(() => this.resize());
    this.resizeObserver.observe(element);
    this.center = new THREE.Vector3(0, 0, 0.05);
    this.extent = 0.3;
    this.resetView();
    this.renderer.setAnimationLoop(() => {
      if (
        element.clientWidth === 0 ||
        element.clientHeight === 0 ||
        document.hidden
      )
        return;
      this.controls.update();
      this.beforeRender?.();
      this.renderer.render(this.scene, this.camera);
    });
  }
  resize() {
    const w = this.element.clientWidth,
      h = this.element.clientHeight;
    if (!w || !h) return;
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h, false);
  }
  resetView(side = this.side || "left") {
    this.side = side;
    const aspect = Math.max(
      0.5,
      this.element.clientWidth / Math.max(1, this.element.clientHeight),
    );
    const distance = (this.extent * 1.95) / Math.min(1, aspect);
    const direction = new THREE.Vector3(
      side === "right" ? -0.3 : 0.3,
      -1,
      0.25,
    ).normalize();
    if (["TsenseI_v2", "TsenseI_v3_2_withid", ...A30_MODEL_IDS].includes(this.modelId)) {
      direction.set(side === "right" ? -1 : 1, -0.3, 0.25).normalize();
    }
    this.camera.position.copy(this.center).addScaledVector(direction, distance);
    this.controls.target.copy(this.center);
    this.controls.update();
    this.resize();
  }
  dispose() {
    this.renderer.setAnimationLoop(null);
    this.resizeObserver.disconnect();
    this.controls.dispose();
    disposeTree(this.scene);
    this.scene.environment = null;
    this.environmentTarget?.dispose();
    this.renderer.dispose();
    this.renderer.domElement.remove();
  }
}

export class HandViewer extends Stage {
  constructor(element) {
    super(element);
    this.environmentTarget = createAluminumEnvironment(THREE, this.renderer);
    this.scene.environment = this.environmentTarget.texture;
  }
  async load(side, model = "TsenseI_v3_2") {
    const response = await fetch(`/api/model?side=${side}&model=${encodeURIComponent(model)}`);
    if (!response.ok) throw new Error("无法读取模型配置");
    const description = await response.json();
    const robot = buildRobot(description);
    const loader = new STLLoader();
    try {
      // Every original STL triangle is loaded, without mesh simplification.
      const results = await Promise.allSettled(
        description.links.flatMap((link) =>
          link.visuals.map(async (visual) => {
            const geometry = await loader.loadAsync(visual.mesh);
            const material = createAluminumMaterial(THREE, visual.color[3]);
            const mesh = new THREE.Mesh(geometry, material);
            applyOrigin(mesh, visual);
            mesh.scale.fromArray(visual.scale);
            robot.links.get(link.name).add(mesh);
          }),
        ),
      );
      const failure = results.find((result) => result.status === "rejected");
      if (failure) throw failure.reason;
    } catch (error) {
      disposeTree(robot.root);
      throw error;
    }
    disposeTree(this.robot?.root);
    this.robot = robot;
    this.modelId = model;
    this.scene.add(robot.root);
    const bounds = new THREE.Box3().setFromObject(robot.root);
    bounds.getCenter(this.center);
    this.extent = Math.max(...bounds.getSize(new THREE.Vector3()).toArray());
    this.grid.position.z = bounds.min.z - 0.003;
    this.grid.position.x = this.center.x;
    this.grid.position.y = this.center.y;
    this.resetView(side);
    return description;
  }
  setPose(pose) {
    this.robot?.setPose(pose);
  }
}

const FINGER_COLORS = [0xc88c6a, 0xb5a149, 0x609c7a, 0x6f95b7, 0xaa7fb0];

/** Display-only mirrored palm frame: fingers +Z, palm depth toward camera (-Y). */
export function palmFacingSkeleton(points, side = "left") {
  if (!Array.isArray(points) || points.length !== 21 ||
      points.some(p => !Array.isArray(p) || p.length !== 3 || !p.every(Number.isFinite))) return null;
  const origin = new THREE.Vector3(...points[0]);
  const up = new THREE.Vector3();
  for (const index of [5, 9, 13, 17]) up.add(new THREE.Vector3(...points[index]).sub(origin));
  if (up.lengthSq() < 1e-12) return null;
  up.normalize();
  const right = new THREE.Vector3(...points[5]).sub(new THREE.Vector3(...points[17]));
  right.addScaledVector(up, -right.dot(up));
  if (right.lengthSq() < 1e-12) return null;
  right.normalize();
  // Build the palm frame before applying the requested horizontal mirror.
  if (side.toLowerCase() === "right") right.negate();
  const front = new THREE.Vector3().crossVectors(right, up);
  return points.map(p => {
    const relative = new THREE.Vector3(...p).sub(origin);
    return new THREE.Vector3(-relative.dot(right), -relative.dot(front), relative.dot(up));
  });
}

export class SkeletonViewer extends Stage {
  constructor(element) {
    super(element);
    this.hand = new THREE.Group();
    this.scene.add(this.hand);
    this.nodes = Array.from({ length: 21 }, (_, i) => {
      const color = i ? FINGER_COLORS[Math.floor((i - 1) / 4)] : 0x5b6b55;
      const mesh = new THREE.Mesh(
        new THREE.SphereGeometry(0.0023, 12, 8),
        new THREE.MeshStandardMaterial({ color, roughness: 0.7 }),
      );
      this.hand.add(mesh);
      return mesh;
    });
    this.lines = FINGER_COLORS.map((color) => {
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute(
        "position",
        new THREE.BufferAttribute(new Float32Array(15), 3),
      );
      const line = new THREE.Line(
        geometry,
        new THREE.LineBasicMaterial({ color }),
      );
      line.frustumCulled = false;
      this.hand.add(line);
      return line;
    });
    this.hand.visible = false;
    this.grid.position.z = -0.025;
    this.center.set(0, 0, 0.08);
    this.extent = 0.2;
    this.resetView();
  }
  resetView(side = this.side || "left") {
    super.resetView(side);
    const distance = this.camera.position.distanceTo(this.center);
    this.camera.position.copy(this.center).add(new THREE.Vector3(0, -distance, 0));
    this.controls.update();
  }
  setPoints(points, side = "left") {
    const vectors = palmFacingSkeleton(points, side);
    this.hand.visible = !!vectors;
    if (!vectors) return;
    this.nodes.forEach((mesh, i) => mesh.position.copy(vectors[i]));
    this.lines.forEach((line, finger) => {
      const array = line.geometry.attributes.position;
      [
        0,
        1 + finger * 4,
        2 + finger * 4,
        3 + finger * 4,
        4 + finger * 4,
      ].forEach((index, i) => {
        const v = vectors[index];
        array.setXYZ(i, v.x, v.y, v.z);
      });
      array.needsUpdate = true;
    });
  }
}
