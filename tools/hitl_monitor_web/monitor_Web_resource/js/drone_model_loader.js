/**
 * Loads and builds the 3D Quadcopter model with instant procedural fallback and GLB support.
 */
class DroneModelLoader {
  /**
   * Loads or generates the 3D quadcopter model into the target Three.js parent group.
   * @param {THREE.Group} droneGroup Parent Three.js group representing the UAV attitude.
   * @param {Function} onPropellersReady Callback receiving the 4 propeller nodes [M1, M2, M3, M4].
   */
  static loadQuadcopterModel(droneGroup, onPropellersReady) {
    // 1. Tạo NGAY LẬP TỨC mô hình 3D Procedural Quadcopter mượt mà (0ms latency, không chờ tải file)
    const proceduralResult = DroneModelLoader.buildProceduralQuadcopter();
    while (droneGroup.children.length > 0) {
      droneGroup.remove(droneGroup.children[0]);
    }
    droneGroup.add(proceduralResult.root);
    onPropellersReady(proceduralResult.propellers);

    // 2. Tải ngầm file GLB thực tế qua HTTP (Không làm đơ giao diện)
    try {
      const gltfLoader = new THREE.GLTFLoader();
      gltfLoader.load(
        "monitor_Web_resource/assets/quadcopter_drone.glb",
        (gltfAsset) => {
          const modelRoot = gltfAsset.scene;
          DroneModelLoader.configureMaterialsAndOrientation(modelRoot);
          while (droneGroup.children.length > 0) {
            droneGroup.remove(droneGroup.children[0]);
          }
          droneGroup.add(modelRoot);
          const glbProps = DroneModelLoader.extractPropellerNodes(modelRoot);
          if (glbProps.length === 4) {
            onPropellersReady(glbProps);
          } else {
            onPropellersReady(proceduralResult.propellers);
          }
          console.log("[3D] GLB Quadcopter model loaded successfully.");
        },
        undefined,
        (error) => {
          console.log("[3D] Using high-performance Procedural Quadcopter model.");
        }
      );
    } catch (_e) {
      // Dùng procedural model
    }
  }

  /**
   * Xây dựng mô hình Quadcopter 3D chi tiết cao bằng Three.js geometry gốc.
   * Cấu hình X chuẩn PX4:
   * M1 (FR): Trước Phải (CCW)
   * M2 (RL): Sau Trái (CCW)
   * M3 (FL): Trước Trái (CW)
   * M4 (RR): Sau Phải (CW)
   */
  static buildProceduralQuadcopter() {
    const root = new THREE.Group();

    // Thân trung tâm (Central Carbon Plate)
    const bodyGeo = new THREE.CylinderGeometry(0.55, 0.65, 0.16, 8);
    const bodyMat = new THREE.MeshStandardMaterial({
      color: 0x1e293b,
      roughness: 0.35,
      metalness: 0.8
    });
    const body = new THREE.Mesh(bodyGeo, bodyMat);
    body.rotation.y = Math.PI / 8;
    root.add(body);

    // Nắp trên (Top Shell / Battery / Flight Controller)
    const topGeo = new THREE.BoxGeometry(0.48, 0.14, 0.48);
    const topMat = new THREE.MeshStandardMaterial({
      color: 0x0f172a,
      roughness: 0.2,
      metalness: 0.9
    });
    const top = new THREE.Mesh(topGeo, topMat);
    top.position.y = 0.13;
    root.add(top);

    // Mũi máy bay (Nose / Forward Indicator) - Trục -Z
    const noseGeo = new THREE.ConeGeometry(0.16, 0.35, 4);
    const noseMat = new THREE.MeshStandardMaterial({
      color: 0x00e5ff,
      emissive: 0x0088aa,
      emissiveIntensity: 0.6
    });
    const nose = new THREE.Mesh(noseGeo, noseMat);
    nose.rotation.x = -Math.PI / 2;
    nose.position.set(0, 0.1, -0.62);
    root.add(nose);

    // 4 Cánh tay Carbon (X-Frame Layout)
    const armGeo = new THREE.CylinderGeometry(0.045, 0.045, 1.8, 12);
    const armMat = new THREE.MeshStandardMaterial({
      color: 0x334155,
      roughness: 0.4,
      metalness: 0.7
    });

    // Arm 1: Trước Phải sang Sau Trái (M1 <-> M2)
    const arm1 = new THREE.Mesh(armGeo, armMat);
    arm1.rotation.z = Math.PI / 2;
    arm1.rotation.y = Math.PI / 4;
    root.add(arm1);

    // Arm 2: Trước Trái sang Sau Phải (M3 <-> M4)
    const arm2 = new THREE.Mesh(armGeo, armMat);
    arm2.rotation.z = Math.PI / 2;
    arm2.rotation.y = -Math.PI / 4;
    root.add(arm2);

    // Vị trí 4 motor (X-Frame span = ~1.25m từ tâm)
    const armRadius = 1.25;
    const motorPositions = [
      { x:  armRadius * 0.707, z: -armRadius * 0.707, name: "M1 FR", color: 0x00e5ff, ledColor: 0x22c55e }, // M0 Trước Phải
      { x: -armRadius * 0.707, z:  armRadius * 0.707, name: "M2 RL", color: 0xff4444, ledColor: 0xef4444 }, // M1 Sau Trái
      { x: -armRadius * 0.707, z: -armRadius * 0.707, name: "M3 FL", color: 0x00e5ff, ledColor: 0x22c55e }, // M2 Trước Trái
      { x:  armRadius * 0.707, z:  armRadius * 0.707, name: "M4 RR", color: 0xff4444, ledColor: 0xef4444 }  // M3 Sau Phải
    ];

    const propellers = [];

    motorPositions.forEach((pos) => {
      // Động cơ Brushless
      const motorBaseGeo = new THREE.CylinderGeometry(0.12, 0.13, 0.14, 16);
      const motorBaseMat = new THREE.MeshStandardMaterial({ color: 0x0f172a, metalness: 0.85 });
      const motorBase = new THREE.Mesh(motorBaseGeo, motorBaseMat);
      motorBase.position.set(pos.x, 0.05, pos.z);
      root.add(motorBase);

      const motorBellGeo = new THREE.CylinderGeometry(0.11, 0.11, 0.09, 16);
      const motorBellMat = new THREE.MeshStandardMaterial({ color: pos.color, metalness: 0.9, roughness: 0.2 });
      const motorBell = new THREE.Mesh(motorBellGeo, motorBellMat);
      motorBell.position.set(pos.x, 0.14, pos.z);
      root.add(motorBell);

      // Đèn LED định vị dẫn đường (Front Green / Rear Red)
      const ledGeo = new THREE.SphereGeometry(0.05, 8, 8);
      const ledMat = new THREE.MeshBasicMaterial({ color: pos.ledColor });
      const led = new THREE.Mesh(ledGeo, ledMat);
      led.position.set(pos.x, -0.06, pos.z);
      root.add(led);

      // Cánh quạt quay (Propeller Hub + 3 Blades)
      const propGroup = new THREE.Group();
      propGroup.position.set(pos.x, 0.22, pos.z);

      const hubGeo = new THREE.CylinderGeometry(0.045, 0.045, 0.05, 12);
      const hubMat = new THREE.MeshStandardMaterial({ color: 0xffffff, metalness: 0.8 });
      const hub = new THREE.Mesh(hubGeo, hubMat);
      propGroup.add(hub);

      // 3 lá cánh quạt khí động học
      const bladeGeo = new THREE.BoxGeometry(0.07, 0.015, 0.65);
      const bladeMat = new THREE.MeshStandardMaterial({
        color: pos.color,
        transparent: true,
        opacity: 0.85,
        roughness: 0.3
      });

      for (let b = 0; b < 3; b++) {
        const blade = new THREE.Mesh(bladeGeo, bladeMat);
        blade.rotation.y = (b * Math.PI * 2) / 3;
        blade.position.y = 0.01;
        propGroup.add(blade);
      }

      root.add(propGroup);
      propellers.push(propGroup);
    });

    return { root, propellers };
  }

  /**
   * Căn chỉnh mô hình GLB tải về
   */
  static configureMaterialsAndOrientation(modelRoot) {
    modelRoot.rotation.y = Math.PI;
    modelRoot.updateMatrixWorld(true);
    const boundingBox = new THREE.Box3().setFromObject(modelRoot);
    const modelSize = new THREE.Vector3();
    const modelCenter = new THREE.Vector3();
    boundingBox.getSize(modelSize);
    boundingBox.getCenter(modelCenter);
    const targetSpan = 2.45;
    const scaleFactor = targetSpan / Math.max(modelSize.x, modelSize.z, 0.001);
    modelRoot.scale.multiplyScalar(scaleFactor);
    modelRoot.position.set(
      -modelCenter.x * scaleFactor,
      -modelCenter.y * scaleFactor,
      -modelCenter.z * scaleFactor
    );
  }

  static extractPropellerNodes(modelRoot) {
    const orderedNames = ["Propeller1", "Propeller4", "Propeller2", "Propeller3"];
    return orderedNames.map((name) => modelRoot.getObjectByName(name)).filter(Boolean);
  }
}

window.DroneModelLoader = DroneModelLoader;
