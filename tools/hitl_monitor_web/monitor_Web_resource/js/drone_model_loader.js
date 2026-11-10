/**
 * Loads and builds 4 selectable 3D Quadcopter models all branded with "ISILAB" on the fuselage:
 * 1. "glb_aero"    — ISILAB Aero-X1 (PBR GLB Quadcopter with custom ISILAB Titanium-Blue UV livery)
 * 2. "mavic_recon" — ISILAB Mavic-Recon (Stealth Angular Recon UAV inspired by DJI Mavic/Matrice)
 * 3. "fpv_apex"    — ISILAB FPV-Apex (Carbon-Fiber 5" Freestyle/Racing Drone with GoPro & LiPo)
 * 4. "lab_heavy"   — ISILAB Heavy-Lift (PX4 Research Quadcopter with Landing Skids & Prop Guards)
 */
class DroneModelLoader {
  static _cachedGltfScene = null;

  /**
   * Loads the selected 3D quadcopter model into the target Three.js parent group.
   * @param {THREE.Group} droneGroup Parent Three.js group representing the UAV attitude.
   * @param {Function} onPropellersReady Callback receiving the 4 propeller nodes [M1, M2, M3, M4].
   * @param {string} [modelType="glb_aero"] Model identifier.
   */
  static loadQuadcopterModel(droneGroup, onPropellersReady, modelType = "glb_aero") {
    const clearGroup = () => {
      while (droneGroup.children.length > 0) {
        droneGroup.remove(droneGroup.children[0]);
      }
    };

    if (modelType === "mavic_recon") {
      clearGroup();
      const res = DroneModelLoader.buildMavicReconQuadcopter();
      droneGroup.add(res.root);
      onPropellersReady(res.propellers);
      return;
    }

    if (modelType === "fpv_apex") {
      clearGroup();
      const res = DroneModelLoader.buildFpvApexQuadcopter();
      droneGroup.add(res.root);
      onPropellersReady(res.propellers);
      return;
    }

    if (modelType === "lab_heavy") {
      clearGroup();
      const res = DroneModelLoader.buildHeavyLiftLabQuadcopter();
      droneGroup.add(res.root);
      onPropellersReady(res.propellers);
      return;
    }

    // Mặc định: "glb_aero" (ISILAB Aero-X1 PBR GLB)
    // Khởi tạo tức thì bằng Mavic-Recon trong vài mili-giây khi giải mã GLB lần đầu
    if (!DroneModelLoader._cachedGltfScene) {
      clearGroup();
      const fallback = DroneModelLoader.buildMavicReconQuadcopter();
      droneGroup.add(fallback.root);
      onPropellersReady(fallback.propellers);
    }

    const mountGltfClone = (sourceScene) => {
      const wrapper = new THREE.Group();
      const modelRoot = sourceScene.clone(true);
      DroneModelLoader.configureMaterialsAndOrientation(modelRoot, wrapper);
      clearGroup();
      droneGroup.add(wrapper);
      const glbProps = DroneModelLoader.extractPropellerNodes(modelRoot);
      if (glbProps.length === 4) {
        onPropellersReady(glbProps);
      }
    };

    if (DroneModelLoader._cachedGltfScene) {
      mountGltfClone(DroneModelLoader._cachedGltfScene);
      return;
    }

    try {
      const gltfLoader = new THREE.GLTFLoader();
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
          (gltfAsset) => {
            DroneModelLoader._cachedGltfScene = gltfAsset.scene;
            mountGltfClone(gltfAsset.scene);
          },
          () => console.log("[3D] Fallback to procedural ISILAB model.")
        );
        return;
      }

      gltfLoader.load(
        "monitor_Web_resource/assets/quadcopter_drone.glb",
        (gltfAsset) => {
          DroneModelLoader._cachedGltfScene = gltfAsset.scene;
          mountGltfClone(gltfAsset.scene);
        },
        undefined,
        () => console.log("[3D] Using procedural ISILAB model.")
      );
    } catch (_e) {
      // Giữ mô hình dự phòng
    }
  }

  /**
   * Tạo Texture chữ "ISILAB" độ phân giải cao để dán lên lưng và hông thân máy bay.
   */
  static createIsilabBadgeTexture(bgColor = "#0f172a", borderColor = "#0a84ff", subText = "UAV FLIGHT LAB") {
    const canvas = document.createElement("canvas");
    canvas.width = 512;
    canvas.height = 256;
    const ctx = canvas.getContext("2d");

    // Nền tấm铭牌 Carbon/Titanium
    ctx.fillStyle = bgColor;
    ctx.fillRect(0, 0, 512, 256);

    // Viền kỹ thuật số phát sáng
    ctx.strokeStyle = borderColor;
    ctx.lineWidth = 10;
    ctx.strokeRect(14, 14, 484, 228);

    // Dải nhấn hai bên
    ctx.fillStyle = borderColor;
    ctx.fillRect(28, 36, 16, 184);
    ctx.fillRect(468, 36, 16, 184);

    // Chữ chính "ISILAB" cực sắc nét
    ctx.font = "900 112px Inter, Arial, sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.shadowColor = borderColor;
    ctx.shadowBlur = 18;
    ctx.fillStyle = "#ffffff";
    ctx.fillText("ISILAB", 256, 112);

    // Dòng phụ bên dưới
    ctx.shadowBlur = 0;
    ctx.font = "700 28px 'JetBrains Mono', monospace";
    ctx.fillStyle = borderColor;
    ctx.fillText(subText, 256, 196);

    const texture = new THREE.CanvasTexture(canvas);
    texture.anisotropy = 4;
    texture.needsUpdate = true;
    return texture;
  }

  /**
   * Tạo cụm 4 cánh quạt chuẩn PX4 Quad-X [M1 FR, M2 RL, M3 FL, M4 RR].
   */
  static createStandardFourRotors(root, motorPositions, bladeCount = 3, bladeLength = 0.72, withGuards = false) {
    const propellers = [];

    motorPositions.forEach((pos) => {
      // Đế & Chuông động cơ Brushless
      const stator = new THREE.Mesh(
        new THREE.CylinderGeometry(0.125, 0.135, 0.12, 20),
        new THREE.MeshStandardMaterial({ color: 0x111827, metalness: 0.85, roughness: 0.25 })
      );
      stator.position.set(pos.x, pos.y, pos.z);
      root.add(stator);

      const bell = new THREE.Mesh(
        new THREE.CylinderGeometry(0.118, 0.122, 0.08, 20),
        new THREE.MeshStandardMaterial({ color: pos.color, metalness: 0.9, roughness: 0.18 })
      );
      bell.position.set(pos.x, pos.y + 0.08, pos.z);
      root.add(bell);

      // Vòng LED định danh màu động cơ
      const ring = new THREE.Mesh(
        new THREE.RingGeometry(0.15, 0.20, 28),
        new THREE.MeshBasicMaterial({ color: pos.color, side: THREE.DoubleSide })
      );
      ring.rotation.x = -Math.PI / 2;
      ring.position.set(pos.x, pos.y - 0.05, pos.z);
      root.add(ring);

      // Vòng bảo vệ cánh quạt (Prop Guard) nếu bật
      if (withGuards) {
        const guard = new THREE.Mesh(
          new THREE.TorusGeometry(bladeLength * 0.56, 0.022, 10, 36),
          new THREE.MeshStandardMaterial({ color: pos.color, metalness: 0.5, roughness: 0.35 })
        );
        guard.rotation.x = Math.PI / 2;
        guard.position.set(pos.x, pos.y + 0.12, pos.z);
        root.add(guard);
      }

      // Nhóm cánh quạt quay quanh trục Y
      const propGroup = new THREE.Group();
      propGroup.position.set(pos.x, pos.y + 0.15, pos.z);
      propGroup.userData.spinAxis = "y";

      const hub = new THREE.Mesh(
        new THREE.CylinderGeometry(0.042, 0.042, 0.055, 14),
        new THREE.MeshStandardMaterial({ color: 0xf8fafc, metalness: 0.9, roughness: 0.15 })
      );
      propGroup.add(hub);

      const bladeGeo = new THREE.BoxGeometry(bladeLength, 0.012, 0.078);
      const bladeMat = new THREE.MeshStandardMaterial({
        color: pos.color,
        transparent: true,
        opacity: 0.86,
        roughness: 0.22,
        metalness: 0.3
      });

      for (let b = 0; b < bladeCount; b++) {
        const blade = new THREE.Mesh(bladeGeo, bladeMat);
        blade.rotation.y = (b * Math.PI * 2) / bladeCount;
        propGroup.add(blade);
      }

      root.add(propGroup);
      propellers.push(propGroup);
    });

    return propellers;
  }

  /**
   * MẪU 2: ISILAB Mavic-Recon (Lấy cảm hứng từ DJI Mavic 3 / Matrice — Góc cạnh Stealth Titanium)
   */
  static buildMavicReconQuadcopter() {
    const root = new THREE.Group();
    const bodyMat = new THREE.MeshStandardMaterial({ color: 0x232936, roughness: 0.28, metalness: 0.82 });
    const darkMat = new THREE.MeshStandardMaterial({ color: 0x0f141d, roughness: 0.35, metalness: 0.75 });

    // Thân chính hình thoi khí động học
    const core = new THREE.Mesh(new THREE.BoxGeometry(0.68, 0.22, 1.32), bodyMat);
    core.position.set(0, 0.06, 0.04);
    root.add(core);

    // Giáp lưng vát cạnh + Tem chữ "ISILAB" nổi bật trên lưng
    const topDeck = new THREE.Mesh(new THREE.BoxGeometry(0.58, 0.08, 1.05), darkMat);
    topDeck.position.set(0, 0.20, 0.05);
    root.add(topDeck);

    const badgeTex = DroneModelLoader.createIsilabBadgeTexture("#0b1324", "#0a84ff", "MAVIC RECON · PX4");
    const badgePlate = new THREE.Mesh(
      new THREE.PlaneGeometry(0.52, 0.88),
      new THREE.MeshBasicMaterial({ map: badgeTex })
    );
    badgePlate.rotation.x = -Math.PI / 2;
    badgePlate.position.set(0, 0.245, 0.06);
    root.add(badgePlate);

    // Tem "ISILAB" hai bên hông thân máy bay
    const sideBadgeGeo = new THREE.PlaneGeometry(0.76, 0.18);
    const sideBadgeMat = new THREE.MeshBasicMaterial({ map: badgeTex });
    const leftSide = new THREE.Mesh(sideBadgeGeo, sideBadgeMat);
    leftSide.rotation.y = -Math.PI / 2;
    leftSide.position.set(-0.345, 0.07, 0.04);
    root.add(leftSide);

    const rightSide = new THREE.Mesh(sideBadgeGeo, sideBadgeMat);
    rightSide.rotation.y = Math.PI / 2;
    rightSide.position.set(0.345, 0.07, 0.04);
    root.add(rightSide);

    // Cụm Camera Gimbal Hasselblad 2 ống kính phía trước (-Z)
    const gimbalHousing = new THREE.Mesh(new THREE.BoxGeometry(0.28, 0.20, 0.24), darkMat);
    gimbalHousing.position.set(0, 0.02, -0.66);
    root.add(gimbalHousing);

    const mainLens = new THREE.Mesh(
      new THREE.CylinderGeometry(0.075, 0.075, 0.06, 20),
      new THREE.MeshStandardMaterial({ color: 0x0a84ff, emissive: 0x0a84ff, emissiveIntensity: 0.65 })
    );
    mainLens.rotation.x = Math.PI / 2;
    mainLens.position.set(0, 0.04, -0.78);
    root.add(mainLens);

    // 4 Cánh tay vát chéo kiểu Mavic
    const armGeo = new THREE.BoxGeometry(0.075, 0.055, 1.45);
    const armPositions = [
      { x:  0.52, z: -0.52, rotY: -Math.PI / 4 },
      { x: -0.52, z:  0.52, rotY: -Math.PI / 4 },
      { x: -0.52, z: -0.52, rotY:  Math.PI / 4 },
      { x:  0.52, z:  0.52, rotY:  Math.PI / 4 }
    ];
    armPositions.forEach((a) => {
      const arm = new THREE.Mesh(armGeo, bodyMat);
      arm.position.set(a.x, 0.05, a.z);
      arm.rotation.y = a.rotY;
      root.add(arm);
    });

    const motorPositions = [
      { x:  0.98, y: 0.08, z: -0.98, color: 0x32ade6 }, // M1 FR (CCW)
      { x: -0.98, y: 0.08, z:  0.98, color: 0x30d158 }, // M2 RL (CCW)
      { x: -0.98, y: 0.08, z: -0.98, color: 0xffd60a }, // M3 FL (CW)
      { x:  0.98, y: 0.08, z:  0.98, color: 0xbf5af2 }  // M4 RR (CW)
    ];

    const propellers = DroneModelLoader.createStandardFourRotors(root, motorPositions, 2, 0.82, false);
    return { root, propellers };
  }

  /**
   * MẪU 3: ISILAB FPV-Apex (Drone Đua / Freestyle Khung Carbon 5" + GoPro + Pin LiPo)
   */
  static buildFpvApexQuadcopter() {
    const root = new THREE.Group();
    const carbonMat = new THREE.MeshStandardMaterial({ color: 0x14171f, roughness: 0.38, metalness: 0.72 });
    const cncRedMat = new THREE.MeshStandardMaterial({ color: 0xe11d48, roughness: 0.22, metalness: 0.88 });

    // Tấm Carbon dưới & trên cùng 8 trụ Standoff nhôm đỏ CNC
    const bottomPlate = new THREE.Mesh(new THREE.BoxGeometry(0.56, 0.035, 1.15), carbonMat);
    bottomPlate.position.set(0, -0.02, 0);
    root.add(bottomPlate);

    const topPlate = new THREE.Mesh(new THREE.BoxGeometry(0.50, 0.03, 0.98), carbonMat);
    topPlate.position.set(0, 0.14, 0.04);
    root.add(topPlate);

    [-0.21, 0.21].forEach((sx) => {
      [-0.38, -0.12, 0.16, 0.42].forEach((sz) => {
        const standoff = new THREE.Mesh(new THREE.CylinderGeometry(0.018, 0.018, 0.16, 10), cncRedMat);
        standoff.position.set(sx, 0.06, sz);
        root.add(standoff);
      });
    });

    // Viên Pin LiPo 6S trên lưng có nhãn "ISILAB"
    const lipoTex = DroneModelLoader.createIsilabBadgeTexture("#111827", "#f43f5e", "6S 1300mAh · FPV APEX");
    const lipoPack = new THREE.Mesh(
      new THREE.BoxGeometry(0.42, 0.19, 0.64),
      new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.3 })
    );
    lipoPack.position.set(0, 0.25, 0.12);
    root.add(lipoPack);

    const lipoLabel = new THREE.Mesh(
      new THREE.PlaneGeometry(0.40, 0.60),
      new THREE.MeshBasicMaterial({ map: lipoTex })
    );
    lipoLabel.rotation.x = -Math.PI / 2;
    lipoLabel.position.set(0, 0.348, 0.12);
    root.add(lipoLabel);

    // Camera Action GoPro gắn nghiêng 25 độ ở đầu máy bay (-Z)
    const goproGroup = new THREE.Group();
    goproGroup.position.set(0, 0.24, -0.34);
    goproGroup.rotation.x = 0.38;
    const goproBody = new THREE.Mesh(new THREE.BoxGeometry(0.32, 0.22, 0.16), carbonMat);
    goproGroup.add(goproBody);
    const goproLens = new THREE.Mesh(
      new THREE.CylinderGeometry(0.065, 0.065, 0.05, 16),
      new THREE.MeshStandardMaterial({ color: 0x38bdf8, emissive: 0x0284c7, emissiveIntensity: 0.6 })
    );
    goproLens.rotation.x = Math.PI / 2;
    goproLens.position.set(-0.07, 0.02, -0.09);
    goproGroup.add(goproLens);
    root.add(goproGroup);

    // 2 Thanh cánh tay Carbon dày cắt X
    const armGeo = new THREE.BoxGeometry(0.11, 0.045, 2.65);
    const arm1 = new THREE.Mesh(armGeo, carbonMat);
    arm1.rotation.y = Math.PI / 4;
    arm1.position.y = -0.02;
    root.add(arm1);

    const arm2 = new THREE.Mesh(armGeo, carbonMat);
    arm2.rotation.y = -Math.PI / 4;
    arm2.position.y = -0.02;
    root.add(arm2);

    const motorPositions = [
      { x:  0.92, y: 0.03, z: -0.92, color: 0x32ade6 },
      { x: -0.92, y: 0.03, z:  0.92, color: 0x30d158 },
      { x: -0.92, y: 0.03, z: -0.92, color: 0xffd60a },
      { x:  0.92, y: 0.03, z:  0.92, color: 0xbf5af2 }
    ];

    const propellers = DroneModelLoader.createStandardFourRotors(root, motorPositions, 3, 0.72, false);
    return { root, propellers };
  }

  /**
   * MẪU 4: ISILAB Heavy-Lift (Trạm Bay Nghiên Cứu PX4 Dual-IMU Có Càng Đáp & Vòng Bảo Vệ Cánh)
   */
  static buildHeavyLiftLabQuadcopter() {
    const root = new THREE.Group();
    const frameMat = new THREE.MeshStandardMaterial({ color: 0x1e293b, roughness: 0.32, metalness: 0.8 });

    // Đĩa trung tâm bát giác + Vòm bảo vệ cảm biến Xsens/MPU
    const centerOctagon = new THREE.Mesh(new THREE.CylinderGeometry(0.52, 0.56, 0.12, 8), frameMat);
    centerOctagon.rotation.y = Math.PI / 8;
    root.add(centerOctagon);

    const dome = new THREE.Mesh(
      new THREE.CylinderGeometry(0.34, 0.44, 0.14, 16),
      new THREE.MeshStandardMaterial({ color: 0x0f172a, roughness: 0.2, metalness: 0.9 })
    );
    dome.position.y = 0.13;
    root.add(dome);

    // Bảng tên "ISILAB" cỡ lớn đặt trên đỉnh vòm trung tâm
    const labTex = DroneModelLoader.createIsilabBadgeTexture("#091526", "#30d158", "ESP32-C6 · DUAL IMU");
    const topBadge = new THREE.Mesh(
      new THREE.PlaneGeometry(0.62, 0.34),
      new THREE.MeshBasicMaterial({ map: labTex })
    );
    topBadge.rotation.x = -Math.PI / 2;
    topBadge.position.set(0, 0.205, 0);
    root.add(topBadge);

    // Mũi tên chỉ hướng bay phía trước (-Z)
    const noseCone = new THREE.Mesh(
      new THREE.ConeGeometry(0.12, 0.28, 4),
      new THREE.MeshStandardMaterial({ color: 0x30d158, emissive: 0x16a34a, emissiveIntensity: 0.7 })
    );
    noseCone.rotation.x = -Math.PI / 2;
    noseCone.position.set(0, 0.06, -0.62);
    root.add(noseCone);

    // Càng đáp (Landing Skids)
    const skidMat = new THREE.MeshStandardMaterial({ color: 0x334155, metalness: 0.8, roughness: 0.35 });
    [-0.36, 0.36].forEach((sx) => {
      const leg = new THREE.Mesh(new THREE.CylinderGeometry(0.022, 0.022, 0.42, 10), skidMat);
      leg.position.set(sx, -0.22, 0);
      root.add(leg);

      const runner = new THREE.Mesh(new THREE.CylinderGeometry(0.024, 0.024, 1.15, 10), skidMat);
      runner.rotation.x = Math.PI / 2;
      runner.position.set(sx, -0.42, 0);
      root.add(runner);
    });

    // 4 Ống tay Carbon tròn
    const armGeo = new THREE.CylinderGeometry(0.038, 0.038, 2.55, 14);
    const arm1 = new THREE.Mesh(armGeo, skidMat);
    arm1.rotation.z = Math.PI / 2;
    arm1.rotation.y = Math.PI / 4;
    root.add(arm1);

    const arm2 = new THREE.Mesh(armGeo, skidMat);
    arm2.rotation.z = Math.PI / 2;
    arm2.rotation.y = -Math.PI / 4;
    root.add(arm2);

    const motorPositions = [
      { x:  0.90, y: 0.05, z: -0.90, color: 0x32ade6 },
      { x: -0.90, y: 0.05, z:  0.90, color: 0x30d158 },
      { x: -0.90, y: 0.05, z: -0.90, color: 0xffd60a },
      { x:  0.90, y: 0.05, z:  0.90, color: 0xbf5af2 }
    ];

    const propellers = DroneModelLoader.createStandardFourRotors(root, motorPositions, 3, 0.66, true);
    return { root, propellers };
  }

  /**
   * Chuẩn hóa kích thước, vật liệu PBR và gắn vòng LED định danh 4 Motor (M1..M4) cho mô hình GLB.
   */
  static configureMaterialsAndOrientation(modelRoot, wrapper) {
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

    const legacyOrder = ["Propeller1", "Propeller4", "Propeller2", "Propeller3"];
    const legacyProps = legacyOrder.map((name) => modelRoot.getObjectByName(name)).filter(Boolean);
    legacyProps.forEach((p) => {
      p.userData.spinAxis = "z";
    });
    return legacyProps;
  }
}

window.DroneModelLoader = DroneModelLoader;
