/** Shared satin titanium finish; legacy export names keep existing viewers compatible. */
export function createAluminumMaterial(THREE, opacity = 1) {
  return new THREE.MeshStandardMaterial({
    name: 'Satin titanium alloy',
    color: 0x929397,
    metalness: 1,
    roughness: 0.4,
    envMapIntensity: 1.0,
    transparent: opacity < 1,
    opacity,
  });
}

/** Bake a small studio environment once per renderer, with no external assets. */
export function createAluminumEnvironment(THREE, renderer) {
  renderer.outputColorSpace = THREE.SRGBColorSpace;
  renderer.toneMapping = THREE.ACESFilmicToneMapping;
  renderer.toneMappingExposure = 1.05;
  const studio = new THREE.Scene();
  const room = new THREE.Mesh(new THREE.BoxGeometry(10, 10, 10),
    new THREE.MeshBasicMaterial({color: 0x737b86, side: THREE.BackSide}));
  studio.add(room);
  // Broad, neutral softboxes produce highlights that move with the orbit view.
  for (const [position, size, intensity] of [
    [[-4, -2, 2], [0.1, 5, 4], 5],
    [[3, 1, 3], [0.1, 3, 5], 3],
    [[0, 0, 4.8], [4, 3, 0.1], 4],
    [[0, 4.8, 0], [2, 0.1, 4], 2],
  ]) {
    const panel = new THREE.Mesh(new THREE.BoxGeometry(...size),
      new THREE.MeshBasicMaterial({color: new THREE.Color().setScalar(intensity)}));
    panel.position.set(...position);
    studio.add(panel);
  }
  const generator = new THREE.PMREMGenerator(renderer);
  try {
    return generator.fromScene(studio, 0.04, 0.1, 20);
  } finally {
    generator.dispose();
    studio.traverse(object => { object.geometry?.dispose(); object.material?.dispose(); });
  }
}
