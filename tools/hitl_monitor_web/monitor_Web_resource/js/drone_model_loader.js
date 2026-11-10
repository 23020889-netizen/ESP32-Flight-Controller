/**
 * Loads and manages 4 REAL ONLINE 3D Quadcopter GLB models from Sketchfab / GitHub:
 * 1. "glb_cesium"   — ISILAB Cesium-Titanium (Analytical Graphics / Ed Mackey, CC-BY 4.0)
 * 2. "glb_annelida" — ISILAB Phantom-Aero    (Annelida, Sketchfab CC-BY 4.0)
 * 3. "glb_laser"    — ISILAB Laser-Racer     (Nick Reyntjens, GitHub)
 * 4. "glb_s9"       — ISILAB S9-Mini Pocket  (VapTor-Studio, Sketchfab CC-BY 4.0)
 */
class DroneModelLoader {
  static _modelCache = {};

  static MODEL_DEFINITIONS = {
    glb_cesium: {
      name: "ISILAB Cesium-Titanium (PBR Industrial)",
      file: "monitor_Web_resource/assets/quadcopter_drone.glb",
      targetSpan: 2.65,
      rotationY: Math.PI,
      propellerNames: [
        "PropCounterClockwiseFront_low", // M1 FR (CCW)
        "PropCounterClockwiseBack_low",  // M2 RL (CCW)
        "PropClockwiseFront_low",        // M3 FL (CW)
        "PropClockwiseBack_low"          // M4 RR (CW)
      ],
      spinAxis: "y",
      hasBakedIsilabLivery: true
    },
    glb_annelida: {
      name: "ISILAB Phantom-Aero (White Commercial)",
      file: "monitor_Web_resource/assets/model_annelida.glb",
      targetSpan: 2.60,
      rotationY: Math.PI,
      propellerNames: [
        "Propeller1", // M1 FR
        "Propeller4", // M2 RL
        "Propeller2", // M3 FL
        "Propeller3"  // M4 RR
      ],
      spinAxis: "z",
      badgeOffset: { y: 0.14, z: -0.05, rx: -Math.PI / 2, scale: 0.42 }
    },
    glb_laser: {
      name: "ISILAB Laser-Racer (Racing Aerodynamic)",
      file: "monitor_Web_resource/assets/model_laser.glb",
      targetSpan: 2.50,
      rotationY: Math.PI,
      propellerNames: [
        "Rotor_FR", // M1 FR
        "Rotor_BL", // M2 RL
        "Rotor_FL", // M3 FL
        "Rotor_BR"  // M4 RR
      ],
      spinAxis: "y",
      badgeOffset: { y: 0.08, z: -0.12, rx: -Math.PI / 2, scale: 0.38 }
    },
    glb_s9: {
      name: "ISILAB S9-Mini (Folding Pocket Drone)",
      file: "monitor_Web_resource/assets/model_s9.glb",
      targetSpan: 2.55,
      rotationY: Math.PI / 2,
      propellerNames: [
        "Circle.002",
        "Circle.003",
        "Circle.004",
        "Circle.005"
      ],
      spinAxis: "z",
      badgeOffset: { y: 0.12, z: 0, rx: -Math.PI / 2, scale: 0.35 }
    }
  };

  /**
   * Loads the selected online 3D GLB model into the Three.js scene.
   * @param {THREE.Group} droneGroup Parent group representing attitude.
   * @param {Function} onPropellersReady Callback receiving the 4 propeller nodes [M1..M4].
   * @param {string} modelKey Key ("glb_cesium", "glb_annelida", "glb_laser", "glb_s9").
   */
  static loadQuadcopterModel(droneGroup, onPropellersReady, modelKey = "glb_cesium") {
    // Map legacy key aliases
    if (modelKey === "glb_aero") modelKey = "glb_cesium";
    if (modelKey === "mavic_recon") modelKey = "glb_annelida";
    if (modelKey === "fpv_apex") modelKey = "glb_laser";
    if (modelKey === "lab_heavy") modelKey = "glb_s9";

    const def = DroneModelLoader.MODEL_DEFINITIONS[modelKey] || DroneModelLoader.MODEL_DEFINITIONS.glb_cesium;

    const clearGroup = () => {
      while (droneGroup.children.length > 0) {
        droneGroup.remove(droneGroup.children[0]);
      }
    };

    const attachModelInstance = (sourceScene) => {
      clearGroup();
      const wrapper = new THREE.Group();
      const modelRoot = sourceScene.clone(true);

      // Căn chỉnh góc quay, tỷ lệ kích thước thực tế
      DroneModelLoader.alignAndScaleModel(modelRoot, wrapper, def);

      // Gắn bảng tên ISILAB phát sáng lên nắp lưng thân máy bay
      if (!def.hasBakedIsilabLivery) {
        DroneModelLoader.attachIsilabFuselageBadge(wrapper, def.badgeOffset);
      }

      // Gắn vòng LED phát sáng định danh 4 Motor: M1 (Cyan), M2 (Green), M3 (Amber), M4 (Purple)
      DroneModelLoader.attachFourMotorLedRings(wrapper);

      droneGroup.add(wrapper);

      // Trích xuất 4 cánh quạt
      const propellers = DroneModelLoader.extractPropellerNodes(modelRoot, def);
      onPropellersReady(propellers);
      console.log(`[3D] Loaded Real Online 3D Model: "${def.name}" with ISILAB branding.`);
    };

    // 1. Kiểm tra bộ nhớ cache
    if (DroneModelLoader._modelCache[modelKey]) {
      attachModelInstance(DroneModelLoader._modelCache[modelKey]);
      return;
    }

    // 2. Tải file GLB thực tế
    const gltfLoader = new THREE.GLTFLoader();

    // Hỗ trợ gói nhúng Base64 cho Cesium nếu có
    if (modelKey === "glb_cesium" && window.QUADCOPTER_GLB_BASE64) {
      try {
        const binStr = window.atob(window.QUADCOPTER_GLB_BASE64);
        const bytes = new Uint8Array(binStr.length);
        for (let i = 0; i < binStr.length; i++) bytes[i] = binStr.charCodeAt(i);
        gltfLoader.parse(
          bytes.buffer,
          "",
          (gltf) => {
            DroneModelLoader._modelCache[modelKey] = gltf.scene;
            attachModelInstance(gltf.scene);
          },
          (err) => console.warn("[3D] GLB parse error, falling back to fetch:", err)
        );
        return;
      } catch (_) {}
    }

    gltfLoader.load(
      def.file,
      (gltf) => {
        DroneModelLoader._modelCache[modelKey] = gltf.scene;
        attachModelInstance(gltf.scene);
      },
      undefined,
      (error) => {
        console.error(`[3D] Error loading GLB "${def.file}":`, error);
      }
    );
  }

  /**
   * Căn chỉnh góc quay, căn giữa trọng tâm và chuẩn hóa sải cánh ~2.5m.
   */
  static alignAndScaleModel(modelRoot, wrapper, def) {
    if (def.rotationY) {
      modelRoot.rotation.y = def.rotationY;
    }
    modelRoot.updateMatrixWorld(true);

    modelRoot.traverse((child) => {
      if (child.isMesh && child.material) {
        child.castShadow = true;
        child.receiveShadow = true;
        if (child.material.isMeshStandardMaterial) {
          child.material.envMapIntensity = 1.35;
          child.material.needsUpdate = true;
        }
      }
    });

    const bbox = new THREE.Box3().setFromObject(modelRoot);
    const size = new THREE.Vector3();
    const center = new THREE.Vector3();
    bbox.getSize(size);
    bbox.getCenter(center);

    const span = Math.max(size.x, size.z, 0.001);
    const scaleFactor = def.targetSpan / span;
    modelRoot.scale.multiplyScalar(scaleFactor);
    modelRoot.position.set(
      -center.x * scaleFactor,
      -center.y * scaleFactor,
      -center.z * scaleFactor
    );

    wrapper.add(modelRoot);
  }

  /**
   * Tạo bảng tên thương hiệu "ISILAB" độ nét cao bằng HTML5 Canvas để dán lên nắp lưng drone.
   */
  static createIsilabBadgeTexture() {
    const canvas = document.createElement("canvas");
    canvas.width = 512;
    canvas.height = 192;
    const ctx = canvas.getContext("2d");

    // Nền thẻ Carbon Titanium đen bóng
    ctx.fillStyle = "#0a0e17";
    ctx.fillRect(0, 0, 512, 192);

    // Viền xanh phát sáng công nghệ
    ctx.strokeStyle = "#0a84ff";
    ctx.lineWidth = 8;
    ctx.strokeRect(10, 10, 492, 172);

    // Vạch trang trí hai đầu
    ctx.fillStyle = "#0a84ff";
    ctx.fillRect(20, 24, 12, 144);
    ctx.fillRect(480, 24, 12, 144);

    // Chữ in đậm "ISILAB" phát quang
    ctx.font = "900 96px Inter, Arial, sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.shadowColor = "#0a84ff";
    ctx.shadowBlur = 16;
    ctx.fillStyle = "#ffffff";
    ctx.fillText("ISILAB", 256, 82);

    // Dòng phụ
    ctx.shadowBlur = 0;
    ctx.font = "700 24px 'JetBrains Mono', monospace";
    ctx.fillStyle = "#38bdf8";
    ctx.fillText("UAV FLIGHT LAB · PX4", 256, 150);

    const texture = new THREE.CanvasTexture(canvas);
    texture.anisotropy = 4;
    texture.needsUpdate = true;
    return texture;
  }

  /**
   * Gắn huy hiệu phát sáng "ISILAB" lên thân lưng máy bay.
   */
  static attachIsilabFuselageBadge(wrapper, offset = {}) {
    const badgeTex = DroneModelLoader.createIsilabBadgeTexture();
    const scale = offset.scale || 0.45;
    const badgeMesh = new THREE.Mesh(
      new THREE.PlaneGeometry(scale * 1.5, scale * 0.58),
      new THREE.MeshBasicMaterial({
        map: badgeTex,
        side: THREE.DoubleSide,
        transparent: true,
        depthWrite: false
      })
    );
    badgeMesh.rotation.x = offset.rx !== undefined ? offset.rx : -Math.PI / 2;
    badgeMesh.position.set(
      offset.x || 0,
      offset.y !== undefined ? offset.y : 0.16,
      offset.z || 0
    );
    wrapper.add(badgeMesh);
  }

  /**
   * Gắn 4 vòng LED định danh màu sắc 4 kênh Motor:
   * M1 FR (Trước Phải): Cyan (#32ade6)
   * M2 RL (Sau Trái):   Green (#30d158)
   * M3 FL (Trước Trái): Amber (#ffd60a)
   * M4 RR (Sau Phải):   Purple (#bf5af2)
   */
  static attachFourMotorLedRings(wrapper) {
    const armRadius = 1.05;
    const ledSpecs = [
      { x:  armRadius * 0.707, z: -armRadius * 0.707, color: 0x32ade6 }, // M1 FR
      { x: -armRadius * 0.707, z:  armRadius * 0.707, color: 0x30d158 }, // M2 RL
      { x: -armRadius * 0.707, z: -armRadius * 0.707, color: 0xffd60a }, // M3 FL
      { x:  armRadius * 0.707, z:  armRadius * 0.707, color: 0xbf5af2 }  // M4 RR
    ];

    ledSpecs.forEach((spec) => {
      const ring = new THREE.Mesh(
        new THREE.RingGeometry(0.18, 0.24, 24),
        new THREE.MeshBasicMaterial({
          color: spec.color,
          side: THREE.DoubleSide,
          transparent: true,
          opacity: 0.85
        })
      );
      ring.rotation.x = -Math.PI / 2;
      ring.position.set(spec.x, 0.02, spec.z);
      wrapper.add(ring);
    });
  }

  /**
   * Trích xuất chính xác 4 node cánh quạt từ cây phân cấp 3D của model GLB.
   */
  static extractPropellerNodes(modelRoot, def) {
    const list = def.propellerNames
      .map((name) => modelRoot.getObjectByName(name))
      .filter(Boolean);

    list.forEach((node) => {
      node.userData.spinAxis = def.spinAxis || "y";
    });

    return list;
  }
}

window.DroneModelLoader = DroneModelLoader;
