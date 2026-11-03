/**
 * Constructs studio lighting, reference floor grid, bounding cage, and coordinate axes.
 */
class SceneEnvironmentBuilder {
  /**
   * Populates the Three.js scene with lighting and toggleable reference helpers.
   * @param {THREE.Scene} scene Target Three.js scene.
   * @returns {Object} References to floorGridGroup, boundingCage, and axesGroup.
   */
  static buildEnvironment(scene) {
    scene.add(new THREE.HemisphereLight(0xdbeafe, 0x1e293b, 1.35));
    const keyLight = new THREE.DirectionalLight(0xffffff, 1.6);
    keyLight.position.set(4, 7, 5);
    const rimLight = new THREE.DirectionalLight(0x38bdf8, 0.85);
    rimLight.position.set(-5, 3, -4);
    const underLight = new THREE.PointLight(0x22d3ee, 1.1, 12);
    underLight.position.set(0, -1.2, 0);
    scene.add(keyLight, rimLight, underLight);

    const floorGridGroup = new THREE.Group();
    const floorGrid = new THREE.GridHelper(20, 40, 0x22d3ee, 0x1e4a7a);
    floorGrid.position.y = -1.4;
    floorGrid.material.transparent = true;
    floorGrid.material.opacity = 0.45;

    const floorRing = new THREE.Mesh(
      new THREE.RingGeometry(1.5, 1.56, 64),
      new THREE.MeshBasicMaterial({
        color: 0x22d3ee,
        side: THREE.DoubleSide,
        transparent: true,
        opacity: 0.7
      })
    );
    floorRing.rotation.x = -Math.PI / 2;
    floorRing.position.y = -1.39;
    floorGridGroup.add(floorGrid, floorRing);

    const boundingCage = new THREE.LineSegments(
      new THREE.EdgesGeometry(new THREE.BoxGeometry(3.4, 2.8, 3.4)),
      new THREE.LineBasicMaterial({
        color: 0x22d3ee,
        transparent: true,
        opacity: 0.55
      })
    );

    const axesGroup = SceneEnvironmentBuilder.createCoordinateAxesGroup();
    scene.add(floorGridGroup, boundingCage, axesGroup);
    return { floorGridGroup, boundingCage, axesGroup };
  }

  /**
   * Builds the X, Y, Z coordinate arrows and glowing text sprites.
   * @returns {THREE.Group} Group containing arrows and labels.
   */
  static createCoordinateAxesGroup() {
    const axesGroup = new THREE.Group();
    const axisSpecs = [
      [1, 0, 0, 0xf87171, "X", "#f87171"],
      [0, 1, 0, 0xbfe9ff, "Z", "#7dd3fc"],
      [0, 0, -1, 0x34d399, "Y", "#34d399"]
    ];
    axisSpecs.forEach(([x, y, z, hexColor, labelText, textColor]) => {
      const direction = new THREE.Vector3(x, y, z);
      axesGroup.add(new THREE.ArrowHelper(direction, new THREE.Vector3(), 1.6, hexColor, 0.28, 0.16));
      const canvas = document.createElement("canvas");
      canvas.width = 128;
      canvas.height = 128;
      const context = canvas.getContext("2d");
      context.font = "bold 96px Inter, sans-serif";
      context.textAlign = "center";
      context.textBaseline = "middle";
      context.shadowColor = textColor;
      context.shadowBlur = 18;
      context.fillStyle = textColor;
      context.fillText(labelText, 64, 70);
      const sprite = new THREE.Sprite(
        new THREE.SpriteMaterial({
          map: new THREE.CanvasTexture(canvas),
          transparent: true,
          depthWrite: false
        })
      );
      sprite.scale.set(0.5, 0.5, 1);
      sprite.position.copy(direction.multiplyScalar(2.05));
      axesGroup.add(sprite);
    });
    return axesGroup;
  }
}

window.SceneEnvironmentBuilder = SceneEnvironmentBuilder;

