/**
 * Loads and builds the 3D Quadcopter model (Cesium PBR Quadcopter GLB + Tactical Carbon Procedural Fallback).
 */
class DroneModelLoader {
  /**
   * Loads the 3D quadcopter model into the target Three.js parent group.
   * @param {THREE.Group} droneGroup Parent Three.js group representing the UAV attitude.
   * @param {Function} onPropellersReady Callback receiving the 4 propeller nodes [M1, M2, M3, M4].
   */
  static loadQuadcopterModel(droneGroup, onPropellersReady) {
    // 1. Khởi tạo tức thì mô hình 3D Tactical Carbon Quadcopter (0ms latency)
    const proceduralResult = DroneModelLoader.buildProceduralQuadcopter();
    while (droneGroup.children.length > 0) {
      droneGroup.remove(droneGroup.children[0]);
    }
    droneGroup.add(proceduralResult.root);
    onPropellersReady(proceduralResult.propellers);

    const applyGltfScene = (gltfAsset) => {
      const wrapper = new THREE.Group();
      const modelRoot = gltfAsset.scene;
      DroneModelLoader.configureMaterialsAndOrientation(modelRoot, wrapper);

      while (droneGroup.children.length > 0) {
        droneGroup.remove(droneGroup.children[0]);
      }
      droneGroup.add(wrapper);

      const glbProps = DroneModelLoader.extractPropellerNodes(modelRoot);
      if (glbProps.length === 4) {
        onPropellersReady(glbProps);
      } else {
        onPropellersReady(proceduralResult.propellers);
      }
      console.log("[3D] Loaded PBR Quadcopter 3D model (CesiumDrone GLB) with 4 animated rotors.");
    };

    try {
      const gltfLoader = new THREE.GLTFLoader();

      // 2. Ưu tiên giải mã trực tiếp từ gói Base64 nhúng sẵn (Chạy mượt cả trên file:// lẫn http://)
      if (window.QUADCOPTER_GLB_BASE64) {
        const binaryString = window.atob(window.QUADCOPTER_GLB_BASE64);
        const len = binaryString.length;
        const bytes = new Uint8Array(len);
        for (let i = 0; i < len; i++) {
          bytes[i] = binaryString.charCodeAt(i);
        }
        gltfLoader.parse(
          bytes.buffer,
          "",
          (gltfAsset) => applyGltfScene(gltfAsset),
          () => console.log("[3D] Fallback to HTTP GLB loader.")
        );
        return;
      }

      // 3. Dự phòng tải qua HTTP nếu không nhúng Base64
      gltfLoader.load(
        "monitor_Web_resource/assets/quadcopter_drone.glb",
        (gltfAsset) => applyGltfScene(gltfAsset),
        undefined,
        () => console.log("[3D] Using high-detail Tactical Carbon Quadcopter model.")
      );
    } catch (_e) {
      // Giữ mô hình Procedural nếu trình duyệt chặn WebGL/GLTF
    }
  }

  /**
   * Xây dựng mô hình Tactical Carbon Quadcopter 3D chi tiết cao bằng Three.js geometry gốc.
   * Cấu hình X chuẩn PX4:
   * M1 (FR): Trước Phải (CCW) — Cyan (#32ade6)
   * M2 (RL): Sau Trái   (CCW) — Emerald (#30d158)
   * M3 (FL): Trước Trái (CW)  — Amber (#ffd60a)
   * M4 (RR): Sau Phải   (CW)  — Purple (#bf5af2)
   */
  static buildProceduralQuadcopter() {
    const root = new THREE.Group();

    // Khung thân Carbon trung tâm
    const lowerPlate = new THREE.Mesh(
      new THREE.BoxGeometry(0.62, 0.04, 0.92),
      new THREE.MeshStandardMaterial({ color: 0x181b24, roughness: 0.35, metalness: 0.75 })
    );
    root.add(lowerPlate);

    // Vỏ khí động học Titanium phía trên
    const canopyGeo = new THREE.CylinderGeometry(0.28, 0.42, 0.18, 8);
    const canopyMat = new THREE.MeshStandardMaterial({ color: 0x2c3344, roughness: 0.25, metalness: 0.85 });
    const canopy = new THREE.Mesh(canopyGeo, canopyMat);
    canopy.scale.set(1.0, 1.0, 1.45);
    canopy.position.set(0, 0.11, 0);
    root.add(canopy);

    // Cụm Camera Gimbal FPV phía trước (-Z)
    const gimbal = new THREE.Mesh(
      new THREE.SphereGeometry(0.13, 16, 16),
      new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 0.15, metalness: 0.9 })
    );
    gimbal.position.set(0, 0.02, -0.48);
    root.add(gimbal);

    const lens = new THREE.Mesh(
      new THREE.CylinderGeometry(0.065, 0.065, 0.06, 16),
      new THREE.MeshStandardMaterial({ color: 0x0a84ff, emissive: 0x0a84ff, emissiveIntensity: 0.55 })
    );
    lens.rotation.x = Math.PI / 2;
    lens.position.set(0, 0.02, -0.58);
    root.add(lens);

    // 4 Cánh tay Carbon (X-Frame)
    const armMat = new THREE.MeshStandardMaterial({ color: 0x222734, roughness: 0.4, metalness: 0.8 });
    const arm1 = new THREE.Mesh(new THREE.CylinderGeometry(0.042, 0.042, 2.5, 12), armMat);
    arm1.rotation.z = Math.PI / 2;
    arm1.rotation.y = Math.PI / 4;
    root.add(arm1);

    const arm2 = new THREE.Mesh(new THREE.CylinderGeometry(0.042, 0.042, 2.5, 12), armMat);
    arm2.rotation.z = Math.PI / 2;
    arm2.rotation.y = -Math.PI / 4;
    root.add(arm2);

    const armRadius = 1.25;
    const motorPositions = [
      { x:  armRadius * 0.707, z: -armRadius * 0.707, color: 0x32ade6 }, // M1 FR (CCW)
      { x: -armRadius * 0.707, z:  armRadius * 0.707, color: 0x30d158 }, // M2 RL (CCW)
      { x: -armRadius * 0.707, z: -armRadius * 0.707, color: 0xffd60a }, // M3 FL (CW)
      { x:  armRadius * 0.707, z:  armRadius * 0.707, color: 0xbf5af2 }  // M4 RR (CW)
    ];

    const propellers = [];

    motorPositions.forEach((pos) => {
      const motorBell = new THREE.Mesh(
        new THREE.CylinderGeometry(0.12, 0.13, 0.14, 18),
        new THREE.MeshStandardMaterial({ color: pos.color, metalness: 0.85, roughness: 0.2 })
      );
      motorBell.position.set(pos.x, 0.09, pos.z);
      root.add(motorBell);

      const propGroup = new THREE.Group();
      propGroup.position.set(pos.x, 0.18, pos.z);
      propGroup.userData.spinAxis = "y";

      const hub = new THREE.Mesh(
        new THREE.CylinderGeometry(0.04, 0.04, 0.05, 12),
        new THREE.MeshStandardMaterial({ color: 0xf8fafc, metalness: 0.9 })
      );
      propGroup.add(hub);

      const bladeGeo = new THREE.BoxGeometry(0.68, 0.012, 0.075);
      const bladeMat = new THREE.MeshStandardMaterial({
        color: pos.color,
        transparent: true,
        opacity: 0.85,
        roughness: 0.25
      });

      for (let b = 0; b < 3; b++) {
        const blade = new THREE.Mesh(bladeGeo, bladeMat);
        blade.rotation.y = (b * Math.PI * 2) / 3;
        propGroup.add(blade);
      }

      root.add(propGroup);
      propellers.push(propGroup);
    });

    return { root, propellers };
  }

  /**
   * Chuẩn hóa kích thước, vật liệu PBR và gắn vòng LED định danh 4 Motor (M1..M4) cho mô hình GLB.
   */
  static configureMaterialsAndOrientation(modelRoot, wrapper) {
    // Xoay 180 độ quanh trục Y để ống kính Camera trước (Lens_low) hướng về trục -Z (Mũi máy bay)
    modelRoot.rotation.y = Math.PI;
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

    const boundingBox = new THREE.Box3().setFromObject(modelRoot);
    const modelSize = new THREE.Vector3();
    const modelCenter = new THREE.Vector3();
    boundingBox.getSize(modelSize);
    boundingBox.getCenter(modelCenter);

    const targetSpan = 2.65;
    const scaleFactor = targetSpan / Math.max(modelSize.x, modelSize.z, 0.001);
    modelRoot.scale.multiplyScalar(scaleFactor);
    modelRoot.position.set(
      -modelCenter.x * scaleFactor,
      -modelCenter.y * scaleFactor,
      -modelCenter.z * scaleFactor
    );
    wrapper.add(modelRoot);

    // Gắn vòng LED phát sáng định danh màu 4 kênh động cơ (M1 Cyan, M2 Green, M3 Amber, M4 Purple)
    const motorAccentSpec = [
      { nodeName: "MotorFrontRight_low", color: 0x32ade6 }, // M1 FR (CCW)
      { nodeName: "MotorBackLeft_low",   color: 0x30d158 }, // M2 RL (CCW)
      { nodeName: "MotorFrontLeft_low",  color: 0xffd60a }, // M3 FL (CW)
      { nodeName: "MotorBackRight_low",  color: 0xbf5af2 }  // M4 RR (CW)
    ];

    motorAccentSpec.forEach((spec) => {
      const motorNode = modelRoot.getObjectByName(spec.nodeName);
      if (motorNode) {
        const ring = new THREE.Mesh(
          new THREE.RingGeometry(0.28, 0.36, 32),
          new THREE.MeshBasicMaterial({
            color: spec.color,
            side: THREE.DoubleSide,
            transparent: true,
            opacity: 0.9
          })
        );
        ring.rotation.x = -Math.PI / 2;
        ring.position.set(0, 0.04, 0);
        motorNode.add(ring);
      }
    });
  }

  /**
   * Trích xuất chính xác 4 node cánh quạt theo thứ tự [M1 FR, M2 RL, M3 FL, M4 RR].
   */
  static extractPropellerNodes(modelRoot) {
    // 1. Bộ tên chuẩn của mô hình PBR CesiumDrone.glb
    const cesiumOrder = [
      "PropCounterClockwiseFront_low", // M1: Trước Phải (FR - CCW)
      "PropCounterClockwiseBack_low",  // M2: Sau Trái   (RL - CCW)
      "PropClockwiseFront_low",        // M3: Trước Trái (FL - CW)
      "PropClockwiseBack_low"          // M4: Sau Phải   (RR - CW)
    ];
    const cesiumProps = cesiumOrder.map((name) => modelRoot.getObjectByName(name)).filter(Boolean);
    if (cesiumProps.length === 4) {
      cesiumProps.forEach((p) => {
        p.userData.spinAxis = "y";
      });
      return cesiumProps;
    }

    // 2. Dự phòng cho các mô hình GLB khác dùng tên Propeller1..4
    const legacyOrder = ["Propeller1", "Propeller4", "Propeller2", "Propeller3"];
    const legacyProps = legacyOrder.map((name) => modelRoot.getObjectByName(name)).filter(Boolean);
    legacyProps.forEach((p) => {
      p.userData.spinAxis = "z";
    });
    return legacyProps;
  }
}

window.DroneModelLoader = DroneModelLoader;
