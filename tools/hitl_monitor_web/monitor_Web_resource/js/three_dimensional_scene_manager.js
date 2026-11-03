/**
 * Manages the Three.js 3D scene, orbit camera controls, and UAV attitude rendering.
 */
class ThreeDimensionalSceneManager {
  /**
   * Creates the 3D scene inside the given host element and loads the GLB drone.
   * @param {HTMLElement} hostContainer DOM element to mount the WebGL canvas.
   */
  constructor(hostContainer) {
    this.hostContainer = hostContainer;
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
    this.renderer.outputEncoding = THREE.sRGBEncoding;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.3;
    hostContainer.appendChild(this.renderer.domElement);

    this.scene = new THREE.Scene();
    this.scene.fog = new THREE.Fog(0x050b18, 9, 22);
    this.camera = new THREE.PerspectiveCamera(42, 1, 0.1, 100);
    this.azimuthRadians = 0.75;
    this.elevationRadians = 0.42;
    this.cameraDistance = 6.0;
    this.propellerNodes = [];
    this.spinDirections = [-1, -1, 1, 1];
    this.targetQuaternion = new THREE.Quaternion();
    this.eulerAngles = new THREE.Euler(0, 0, 0, "YXZ");

    const references = SceneEnvironmentBuilder.buildEnvironment(this.scene);
    this.floorGridGroup = references.floorGridGroup;
    this.boundingCage = references.boundingCage;
    this.axesGroup = references.axesGroup;

    this.droneGroup = new THREE.Group();
    this.scene.add(this.droneGroup);
    DroneModelLoader.loadQuadcopterModel(this.droneGroup, (propellers) => {
      this.propellerNodes = propellers;
    });
    this.bindCameraOrbitEvents();
    this.handleContainerResize();
  }

  /**
   * Binds pointer drag, wheel zoom, and container ResizeObserver listeners.
   */
  bindCameraOrbitEvents() {
    let dragStart = null;
    this.hostContainer.addEventListener("pointerdown", (event) => {
      dragStart = [event.clientX, event.clientY];
      this.hostContainer.setPointerCapture(event.pointerId);
    });
    this.hostContainer.addEventListener("pointerup", () => {
      dragStart = null;
    });
    this.hostContainer.addEventListener("pointermove", (event) => {
      if (!dragStart) return;
      this.azimuthRadians -= (event.clientX - dragStart[0]) * 0.006;
      this.elevationRadians = Math.max(-0.2, Math.min(1.4, this.elevationRadians + (event.clientY - dragStart[1]) * 0.006));
      dragStart = [event.clientX, event.clientY];
    });
    this.hostContainer.addEventListener("wheel", (event) => {
      event.preventDefault();
      this.cameraDistance = Math.max(3.0, Math.min(14, this.cameraDistance + event.deltaY * 0.004));
    }, { passive: false });
    window.addEventListener("resize", () => this.handleContainerResize());
    if (window.ResizeObserver) new ResizeObserver(() => this.handleContainerResize()).observe(this.hostContainer);
  }

  /**
   * Updates renderer viewport dimensions to match the FlexLayout container size.
   */
  handleContainerResize() {
    const width = Math.max(1, this.hostContainer.clientWidth || window.innerWidth);
    const height = Math.max(1, this.hostContainer.clientHeight || window.innerHeight);
    this.renderer.setSize(width, height);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
  }

  /**
   * Updates UAV orientation, spins the 4 propellers, and renders the 3D frame.
   */
  renderSceneFrame(sample, deltaSeconds) {
    const degreesToRadians = Math.PI / 180;
    this.eulerAngles.set(sample.pitch * degreesToRadians, -sample.yaw * degreesToRadians, -sample.roll * degreesToRadians, "YXZ");
    this.targetQuaternion.setFromEuler(this.eulerAngles);
    this.droneGroup.quaternion.slerp(this.targetQuaternion, Math.min(1, deltaSeconds * 18));
    this.propellerNodes.forEach((propeller, index) => {
      propeller.rotateZ(this.spinDirections[index] * (sample.m[index] || 0) * deltaSeconds * 0.08);
    });
    this.camera.position.set(
      this.cameraDistance * Math.sin(this.azimuthRadians) * Math.cos(this.elevationRadians),
      this.cameraDistance * Math.sin(this.elevationRadians),
      this.cameraDistance * Math.cos(this.azimuthRadians) * Math.cos(this.elevationRadians)
    );
    this.camera.lookAt(0, -0.05, 0);
    this.renderer.render(this.scene, this.camera);
  }
}

window.ThreeDimensionalSceneManager = ThreeDimensionalSceneManager;
