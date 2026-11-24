/**
 * DroneModelLoader: Quản lý và tải các mô hình 3D Quadcopter cho UAV HITL Web Monitor:
 * 1. "sci_fi_bubble" — ISILAB Sci-Fi Bubble Quad-X (Mô hình PBR Procedural siêu nét, không phụ thuộc file ngoài)
 * 2. "glb_cesium"     — ISILAB Cesium-Titanium (Analytical Graphics / Ed Mackey, CC-BY 4.0)
 * 3. "glb_annelida"   — ISILAB Phantom-Aero    (Annelida, Sketchfab CC-BY 4.0)
 * 4. "glb_laser"      — ISILAB Laser-Racer     (Nick Reyntjens, GitHub)
 * 5. "glb_s9"         — ISILAB S9-Mini Pocket  (VapTor-Studio, Sketchfab CC-BY 4.0)
 */

/* ============================================================================
 *  PROCEDURAL GENERATOR HELPERS FOR SCI-FI BUBBLE QUAD-X
 * ============================================================================ */
let _cachedStudioEnvMap = null;

function createStudioEnvironment(renderer) {
      const pmremGenerator = new THREE.PMREMGenerator(renderer);
      pmremGenerator.compileEquirectangularShader();

      const envScene = new THREE.Scene();
      envScene.background = new THREE.Color(0x06101e);

      // Overhead bright cyan-white softbox panel (gives glossy top highlight on bubble)
      const topPanel = new THREE.Mesh(
        new THREE.PlaneGeometry(18, 12),
        new THREE.MeshBasicMaterial({ color: 0xdaf8ff, side: THREE.DoubleSide })
      );
      topPanel.position.set(0, 9, 0);
      topPanel.rotation.x = Math.PI / 2;
      envScene.add(topPanel);

      // Front-left aqua rim light strip
      const leftPanel = new THREE.Mesh(
        new THREE.PlaneGeometry(14, 7),
        new THREE.MeshBasicMaterial({ color: 0x38e8ff, side: THREE.DoubleSide })
      );
      leftPanel.position.set(-10, 3, 6);
      leftPanel.lookAt(0, 0, 0);
      envScene.add(leftPanel);

      // Front-right bright studio reflector
      const rightPanel = new THREE.Mesh(
        new THREE.PlaneGeometry(12, 6),
        new THREE.MeshBasicMaterial({ color: 0xbef5ff, side: THREE.DoubleSide })
      );
      rightPanel.position.set(10, 4, -5);
      rightPanel.lookAt(0, 0, 0);
      envScene.add(rightPanel);

      // Subtle emerald-cyan lower accent strip (bounces on bottom metallic hull)
      const bottomAccent = new THREE.Mesh(
        new THREE.PlaneGeometry(16, 16),
        new THREE.MeshBasicMaterial({ color: 0x0b2a3c, side: THREE.DoubleSide })
      );
      bottomAccent.position.set(0, -8, 0);
      bottomAccent.rotation.x = -Math.PI / 2;
      envScene.add(bottomAccent);

      const envRT = pmremGenerator.fromScene(envScene, 0.035);
      pmremGenerator.dispose();
      return envRT.texture;
    }

    /**
     * Generates procedural carbon/olive micro-weave texture for cockpit bucket seats.
     */

function createSeatWeaveTexture() {
      const canvas = document.createElement('canvas');
      canvas.width = 128;
      canvas.height = 128;
      const ctx = canvas.getContext('2d');

      ctx.fillStyle = '#333d1f';
      ctx.fillRect(0, 0, 128, 128);

      for (let y = 0; y < 128; y += 4) {
        for (let x = 0; x < 128; x += 4) {
          const checker = ((x + y) / 4) % 2 === 0;
          ctx.fillStyle = checker ? 'rgba(92, 112, 46, 0.42)' : 'rgba(18, 24, 12, 0.45)';
          ctx.fillRect(x, y, 3, 3);
        }
      }
      const tex = new THREE.CanvasTexture(canvas);
      tex.wrapS = THREE.RepeatWrapping;
      tex.wrapT = THREE.RepeatWrapping;
      tex.repeat.set(6, 6);
      return tex;
    }

    /**
     * Generates a sleek glowing "ISILAB" decal badge texture for the drone chassis.
     */
    function createIsilabBadgeTexture() {
      const canvas = document.createElement('canvas');
      canvas.width = 512;
      canvas.height = 160;
      const ctx = canvas.getContext('2d');

      ctx.clearRect(0, 0, 512, 160);

      // Dark metallic pill backdrop
      ctx.fillStyle = 'rgba(6, 18, 32, 0.85)';
      ctx.strokeStyle = '#22d3ee';
      ctx.lineWidth = 5;
      ctx.beginPath();
      ctx.roundRect(14, 18, 484, 124, 28);
      ctx.fill();
      ctx.stroke();

      // Subtle glowing accent lines
      ctx.fillStyle = '#39ff14';
      ctx.fillRect(40, 44, 10, 72);
      ctx.fillRect(462, 44, 10, 72);

      // Crisp ISILAB typography
      ctx.font = '800 72px Inter, Arial, sans-serif';
      ctx.textAlign = 'center';
      ctx.textBaseline = 'middle';
      ctx.shadowColor = '#22d3ee';
      ctx.shadowBlur = 18;
      ctx.fillStyle = '#e0faff';
      ctx.fillText('ISILAB', 256, 76);

      ctx.shadowBlur = 0;
      ctx.font = '600 18px JetBrains Mono, monospace';
      ctx.fillStyle = '#38bdf8';
      ctx.fillText('VNU · HITL QUAD-X', 256, 122);

      const tex = new THREE.CanvasTexture(canvas);
      tex.anisotropy = 4;
      return tex;
    }

    /**
     * Generates glowing cockpit HUD instrument display texture.
     */

function createCockpitHudTexture() {
      const canvas = document.createElement('canvas');
      canvas.width = 256;
      canvas.height = 128;
      const ctx = canvas.getContext('2d');

      ctx.fillStyle = '#03121e';
      ctx.fillRect(0, 0, 256, 128);

      // Artificial horizon ring
      ctx.strokeStyle = '#22d3ee';
      ctx.lineWidth = 3;
      ctx.beginPath();
      ctx.arc(128, 64, 38, 0, Math.PI * 2);
      ctx.stroke();

      // Horizon bar
      ctx.strokeStyle = '#39ff14';
      ctx.lineWidth = 2.5;
      ctx.beginPath();
      ctx.moveTo(76, 64);
      ctx.lineTo(180, 64);
      ctx.stroke();

      // Side telemetry bars
      ctx.fillStyle = '#0ea5e9';
      for (let i = 0; i < 6; i++) {
        ctx.fillRect(18, 22 + i * 14, 32 - i * 2, 7);
        ctx.fillRect(206 + i * 2, 22 + i * 14, 32 - i * 2, 7);
      }

      const tex = new THREE.CanvasTexture(canvas);
      return tex;
    }

    /**
     * Generates a radial motion-blur propeller disc texture for high-RPM realism.
     */

function createRotorBlurTexture() {
      const canvas = document.createElement('canvas');
      canvas.width = 256;
      canvas.height = 256;
      const ctx = canvas.getContext('2d');
      const cx = 128;
      const cy = 128;

      ctx.clearRect(0, 0, 256, 256);

      // Swept translucent turbine blade arcs
      const blades = 8;
      for (let i = 0; i < blades; i++) {
        const startA = (i / blades) * Math.PI * 2;
        const grad = ctx.createRadialGradient(cx, cy, 24, cx, cy, 122);
        grad.addColorStop(0.0, 'rgba(180, 245, 255, 0.05)');
        grad.addColorStop(0.35, 'rgba(120, 235, 255, 0.34)');
        grad.addColorStop(0.85, 'rgba(56, 225, 255, 0.45)');
        grad.addColorStop(0.96, 'rgba(57, 255, 20, 0.55)');
        grad.addColorStop(1.0, 'rgba(56, 225, 255, 0.0)');

        ctx.fillStyle = grad;
        ctx.beginPath();
        ctx.moveTo(cx, cy);
        ctx.arc(cx, cy, 122, startA, startA + 0.42);
        ctx.closePath();
        ctx.fill();
      }

      // Outer subtle ring shimmer
      ctx.strokeStyle = 'rgba(130, 245, 255, 0.55)';
      ctx.lineWidth = 4;
      ctx.beginPath();
      ctx.arc(cx, cy, 114, 0, Math.PI * 2);
      ctx.stroke();

      const tex = new THREE.CanvasTexture(canvas);
      return tex;
    }

    /**
     * Generates a vertical soft alpha gradient texture for downward exhaust plasma cones.
     */

function createPlumeTexture() {
      const canvas = document.createElement('canvas');
      canvas.width = 64;
      canvas.height = 256;
      const ctx = canvas.getContext('2d');

      const grad = ctx.createLinearGradient(0, 0, 0, 256);
      grad.addColorStop(0.0, 'rgba(220, 255, 255, 0.95)');
      grad.addColorStop(0.25, 'rgba(34, 235, 255, 0.72)');
      grad.addColorStop(0.65, 'rgba(45, 255, 130, 0.25)');
      grad.addColorStop(1.0, 'rgba(0, 200, 255, 0.0)');

      ctx.fillStyle = grad;
      ctx.fillRect(0, 0, 64, 256);

      const tex = new THREE.CanvasTexture(canvas);
      return tex;
    }

    /**
     * Helper to build crisp 3D floating axis sprite labels (X, Y, Z, NOSE).
     */

function createSciFiBubbleQuad(envMap) {
      const root = new THREE.Group();
      root.name = 'SciFiBubbleQuadX';

      // Theme palette configurations ('cyan' inspired by reference orb craft vs 'stealth')
      const THEMES = {
        cyan: {
          shroudColor: 0x2fd8f0,
          shroudEmissive: 0x052c3b,
          armColor: 0x1fb8d4,
          hullColor: 0x0b2532,
          glassColor: 0x56f0ff,
          fresnelColor: 0x3ef5ff,
          intakeRing: 0x39ff14,
          exhaustRing: 0x00e5ff,
          seatColor: 0x3f4c24,
          ribColor: 0x0a2b3a
        },
        stealth: {
          shroudColor: 0x5d6e85,
          shroudEmissive: 0x08101c,
          armColor: 0x3b485c,
          hullColor: 0x0f172a,
          glassColor: 0x38bdf8,
          fresnelColor: 0x38bdf8,
          intakeRing: 0x06e6ff,
          exhaustRing: 0x3b82f6,
          seatColor: 0x263142,
          ribColor: 0x1e293b
        }
      };

      // Shared procedural textures
      const seatWeaveTex = createSeatWeaveTexture();
      const isilabTex = createIsilabBadgeTexture();
      const hudTex = createCockpitHudTexture();
      const blurTex = createRotorBlurTexture();
      const plumeTex = createPlumeTexture();

      // Shared PBR Materials (stored for live theme switching)
      const mats = {
        shroud: new THREE.MeshPhysicalMaterial({
          color: THEMES.cyan.shroudColor,
          emissive: THEMES.cyan.shroudEmissive,
          metalness: 0.72,
          roughness: 0.14,
          clearcoat: 1.0,
          clearcoatRoughness: 0.06,
          envMap: envMap,
          envMapIntensity: 1.85
        }),
        arm: new THREE.MeshPhysicalMaterial({
          color: THEMES.cyan.armColor,
          metalness: 0.78,
          roughness: 0.18,
          clearcoat: 0.9,
          clearcoatRoughness: 0.1,
          envMap: envMap,
          envMapIntensity: 1.65
        }),
        hull: new THREE.MeshPhysicalMaterial({
          color: THEMES.cyan.hullColor,
          metalness: 0.65,
          roughness: 0.24,
          clearcoat: 0.85,
          clearcoatRoughness: 0.12,
          envMap: envMap,
          envMapIntensity: 1.4
        }),
        darkTitanium: new THREE.MeshStandardMaterial({
          color: 0x131d2b,
          metalness: 0.85,
          roughness: 0.3,
          envMap: envMap,
          envMapIntensity: 1.2
        }),
        chromeMetal: new THREE.MeshStandardMaterial({
          color: 0xdbeafe,
          metalness: 0.95,
          roughness: 0.12,
          envMap: envMap,
          envMapIntensity: 2.0
        }),
        glass: new THREE.MeshPhysicalMaterial({
          color: THEMES.cyan.glassColor,
          metalness: 0.08,
          roughness: 0.04,
          clearcoat: 1.0,
          clearcoatRoughness: 0.02,
          transparent: true,
          opacity: 0.30,
          depthWrite: false,
          side: THREE.DoubleSide,
          envMap: envMap,
          envMapIntensity: 2.3
        }),
        rib: new THREE.MeshPhysicalMaterial({
          color: THEMES.cyan.ribColor,
          metalness: 0.8,
          roughness: 0.18,
          clearcoat: 0.8,
          envMap: envMap,
          envMapIntensity: 1.5
        }),
        intakeRing: new THREE.MeshStandardMaterial({
          color: THEMES.cyan.intakeRing,
          emissive: THEMES.cyan.intakeRing,
          emissiveIntensity: 2.2,
          roughness: 0.2,
          metalness: 0.1
        }),
        headlightCore: new THREE.MeshBasicMaterial({
          color: 0xf4ffff
        }),
        seat: new THREE.MeshStandardMaterial({
          color: THEMES.cyan.seatColor,
          map: seatWeaveTex,
          roughness: 0.68,
          metalness: 0.15
        }),
        turbineBlade: new THREE.MeshStandardMaterial({
          color: 0x1e2c3d,
          metalness: 0.85,
          roughness: 0.22,
          envMap: envMap,
          envMapIntensity: 1.4
        })
      };

      // Fresnel rim shader for that iconic luminous cyan glass orb edge
      const fresnelUniforms = {
        uColor: { value: new THREE.Color(THEMES.cyan.fresnelColor) },
        uIntensity: { value: 1.15 }
      };
      const fresnelMat = new THREE.ShaderMaterial({
        uniforms: fresnelUniforms,
        vertexShader: `
          varying vec3 vNormalW;
          varying vec3 vViewDirW;
          void main() {
            vec4 worldPos = modelMatrix * vec4(position, 1.0);
            vNormalW = normalize(mat3(modelMatrix) * normal);
            vViewDirW = normalize(cameraPosition - worldPos.xyz);
            gl_Position = projectionMatrix * viewMatrix * worldPos;
          }
        `,
        fragmentShader: `
          uniform vec3 uColor;
          uniform float uIntensity;
          varying vec3 vNormalW;
          varying vec3 vViewDirW;
          void main() {
            float ndotv = clamp(abs(dot(normalize(vNormalW), normalize(vViewDirW))), 0.0, 1.0);
            float rim = pow(1.0 - ndotv, 2.6);
            gl_FragColor = vec4(uColor * (0.85 + rim * 0.6), rim * 0.68 * uIntensity);
          }
        `,
        transparent: true,
        depthWrite: false,
        blending: THREE.AdditiveBlending
      });

      /* ----------------------------------------------------------------------
       * 1. CENTRAL POD: LOWER SCULPTED CHASSIS + COCKPIT + CYAN BUBBLE CANOPY
       * ---------------------------------------------------------------------- */
      const centerPod = new THREE.Group();
      centerPod.position.y = 0.08;
      root.add(centerPod);

      const podRadius = 0.62;

      // 1A. Lower aerodynamic dark gunmetal-teal hull hemisphere
      const lowerHullGeo = new THREE.SphereGeometry(
        podRadius * 1.015, 48, 28,
        0, Math.PI * 2,
        Math.PI * 0.46, Math.PI * 0.54
      );
      const lowerHull = new THREE.Mesh(lowerHullGeo, mats.hull);
      lowerHull.scale.set(1.0, 0.94, 1.08);
      lowerHull.castShadow = true;
      centerPod.add(lowerHull);

      // Sculpted metallic waist collar ring where glass meets lower chassis
      const collarGeo = new THREE.TorusGeometry(podRadius * 1.01, 0.028, 16, 64);
      const collar = new THREE.Mesh(collarGeo, mats.rib);
      collar.rotation.x = Math.PI / 2;
      collar.scale.set(1.0, 1.08, 1.0);
      collar.position.y = 0.05;
      centerPod.add(collar);

      // Ventral sensor belly plate & subtle titanium landing skids
      const bellyPlate = new THREE.Mesh(
        new THREE.CylinderGeometry(0.40, 0.32, 0.10, 32),
        mats.darkTitanium
      );
      bellyPlate.position.y = -0.56;
      centerPod.add(bellyPlate);

      // Ventral optical flow / LIDAR micro dome
      const bellySensor = new THREE.Mesh(
        new THREE.SphereGeometry(0.09, 20, 12, 0, Math.PI * 2, Math.PI * 0.5, Math.PI * 0.5),
        new THREE.MeshBasicMaterial({ color: 0x00e5ff })
      );
      bellySensor.position.y = -0.60;
      centerPod.add(bellySensor);

      // Twin sleek dark titanium landing runners underneath
      [-0.28, 0.28].forEach((xOff) => {
        const runner = new THREE.Mesh(
          new THREE.BoxGeometry(0.045, 0.045, 0.72),
          mats.darkTitanium
        );
        runner.position.set(xOff, -0.64, 0.02);
        centerPod.add(runner);
      });

      // 1B. Front Optical Headlight / LiDAR Module (Nose = -Z) with vivid NEON GREEN RING
      const headlightGroup = new THREE.Group();
      headlightGroup.position.set(0, -0.16, -0.64);
      headlightGroup.rotation.x = 0.22;
      centerPod.add(headlightGroup);

      const hlHousing = new THREE.Mesh(
        new THREE.CylinderGeometry(0.155, 0.175, 0.14, 32),
        mats.darkTitanium
      );
      hlHousing.rotation.x = Math.PI / 2;
      headlightGroup.add(hlHousing);

      const hlGreenRing = new THREE.Mesh(
        new THREE.TorusGeometry(0.135, 0.022, 16, 40),
        mats.intakeRing
      );
      hlGreenRing.position.z = -0.068;
      headlightGroup.add(hlGreenRing);

      const hlLens = new THREE.Mesh(
        new THREE.CircleGeometry(0.115, 32),
        mats.headlightCore
      );
      hlLens.rotation.y = Math.PI;
      hlLens.position.z = -0.072;
      headlightGroup.add(hlLens);

      // Rear tail status bar (+Z rear)
      const tailMat = new THREE.MeshStandardMaterial({
        color: 0xff2a5f,
        emissive: 0xff1e56,
        emissiveIntensity: 1.9
      });
      const tailLight = new THREE.Mesh(new THREE.BoxGeometry(0.28, 0.045, 0.04), tailMat);
      tailLight.position.set(0, -0.14, 0.655);
      centerPod.add(tailLight);

      // ISILAB Glowing Hull Badge (mounted on rear upper fairing lip & front lower fairing)
      const badgeMat = new THREE.MeshBasicMaterial({
        map: isilabTex,
        transparent: true,
        side: THREE.DoubleSide
      });
      const rearBadge = new THREE.Mesh(new THREE.PlaneGeometry(0.36, 0.115), badgeMat);
      rearBadge.position.set(0, -0.02, 0.665);
      rearBadge.rotation.x = -0.12;
      centerPod.add(rearBadge);

      const topBadge = new THREE.Mesh(new THREE.PlaneGeometry(0.32, 0.10), badgeMat);
      topBadge.position.set(0, -0.35, -0.53);
      topBadge.rotation.y = Math.PI;
      topBadge.rotation.x = -0.58;
      centerPod.add(topBadge);

      // 1C. Detailed Cockpit Interior (Visible through translucent cyan glass bubble)
      const cockpitGroup = new THREE.Group();
      centerPod.add(cockpitGroup);

      // Interior deck floor
      const cockpitFloor = new THREE.Mesh(
        new THREE.CylinderGeometry(0.54, 0.50, 0.06, 32),
        mats.darkTitanium
      );
      cockpitFloor.position.y = -0.22;
      cockpitGroup.add(cockpitFloor);

      // Helper to build each futuristic racing bucket seat with twin metallic harness cutouts
      function buildRacingSeat(xPos) {
        const seat = new THREE.Group();
        seat.position.set(xPos, -0.16, 0.08);

        // Seat cushion pan
        const cushion = new THREE.Mesh(new THREE.BoxGeometry(0.21, 0.07, 0.25), mats.seat);
        cushion.position.set(0, 0.02, -0.04);
        seat.add(cushion);

        // Side thigh bolsters
        [-0.105, 0.105].forEach((bx) => {
          const bolster = new THREE.Mesh(new THREE.BoxGeometry(0.035, 0.09, 0.26), mats.seat);
          bolster.position.set(bx, 0.04, -0.03);
          bolster.rotation.z = bx > 0 ? 0.22 : -0.22;
          seat.add(bolster);
        });

        // Contoured backrest (tilted slightly back)
        const backrestGroup = new THREE.Group();
        backrestGroup.position.set(0, 0.05, 0.08);
        backrestGroup.rotation.x = 0.18;
        seat.add(backrestGroup);

        const lowerBack = new THREE.Mesh(new THREE.BoxGeometry(0.21, 0.26, 0.065), mats.seat);
        lowerBack.position.y = 0.13;
        backrestGroup.add(lowerBack);

        // Tapered winged shoulder / headrest silhouette
        const shoulderShape = new THREE.Shape();
        shoulderShape.moveTo(-0.115, 0.0);
        shoulderShape.lineTo(0.115, 0.0);
        shoulderShape.quadraticCurveTo(0.12, 0.14, 0.065, 0.26);
        shoulderShape.quadraticCurveTo(0.0, 0.31, -0.065, 0.26);
        shoulderShape.quadraticCurveTo(-0.12, 0.14, -0.115, 0.0);

        const upperGeo = new THREE.ExtrudeGeometry(shoulderShape, {
          depth: 0.055,
          bevelEnabled: true,
          bevelThickness: 0.012,
          bevelSize: 0.01,
          bevelSegments: 3
        });
        upperGeo.center();
        const upperBack = new THREE.Mesh(upperGeo, mats.seat);
        upperBack.position.set(0, 0.35, 0);
        backrestGroup.add(upperBack);

        // Twin metallic harness slot cutouts (iconic visual detail from reference!)
        [-0.046, 0.046].forEach((hx) => {
          const bezel = new THREE.Mesh(new THREE.BoxGeometry(0.046, 0.068, 0.082), mats.chromeMetal);
          bezel.position.set(hx, 0.33, 0);
          backrestGroup.add(bezel);

          const slotInner = new THREE.Mesh(
            new THREE.BoxGeometry(0.028, 0.048, 0.088),
            new THREE.MeshBasicMaterial({ color: 0x0b1f2e })
          );
          slotInner.position.set(hx, 0.33, 0);
          backrestGroup.add(slotInner);
        });

        // Subtle gold-yellow seat belt clasp
        const buckle = new THREE.Mesh(
          new THREE.BoxGeometry(0.036, 0.022, 0.022),
          new THREE.MeshStandardMaterial({ color: 0xfacc15, metalness: 0.5, roughness: 0.3 })
        );
        buckle.position.set(0, 0.06, -0.01);
        seat.add(buckle);

        return seat;
      }

      cockpitGroup.add(buildRacingSeat(-0.175));
      cockpitGroup.add(buildRacingSeat(0.175));

      // Futuristic Front Dashboard & Glowing Cyan HUD Display
      const dashCowl = new THREE.Mesh(
        new THREE.BoxGeometry(0.62, 0.12, 0.20),
        mats.darkTitanium
      );
      dashCowl.position.set(0, -0.06, -0.34);
      dashCowl.rotation.x = 0.25;
      cockpitGroup.add(dashCowl);

      const hudScreen = new THREE.Mesh(
        new THREE.PlaneGeometry(0.30, 0.13),
        new THREE.MeshBasicMaterial({ map: hudTex, transparent: true, opacity: 0.92 })
      );
      hudScreen.position.set(0, 0.03, -0.34);
      hudScreen.rotation.x = -0.25;
      cockpitGroup.add(hudScreen);

      // Steering Wheel / Flight Yoke in front of left pilot seat
      const yokeGroup = new THREE.Group();
      yokeGroup.position.set(-0.16, 0.04, -0.22);
      yokeGroup.rotation.x = -0.35;
      cockpitGroup.add(yokeGroup);

      const yokeRing = new THREE.Mesh(
        new THREE.TorusGeometry(0.085, 0.013, 12, 28),
        mats.darkTitanium
      );
      yokeGroup.add(yokeRing);

      const yokeBar = new THREE.Mesh(
        new THREE.BoxGeometry(0.15, 0.018, 0.018),
        mats.chromeMetal
      );
      yokeGroup.add(yokeBar);

      const yokeShaft = new THREE.Mesh(
        new THREE.CylinderGeometry(0.014, 0.018, 0.15, 12),
        mats.darkTitanium
      );
      yokeShaft.rotation.x = Math.PI / 2;
      yokeShaft.position.z = -0.075;
      yokeGroup.add(yokeShaft);

      // Soft interior cockpit ambient cyan glow light
      const cockpitLight = new THREE.PointLight(0x38f0ff, 0.9, 1.8);
      cockpitLight.position.set(0, 0.15, -0.15);
      cockpitGroup.add(cockpitLight);

      // 1D. Translucent Glossy Cyan Bubble Glass Canopy + Fresnel Rim + Canopy Ribs
      const glassSphereGeo = new THREE.SphereGeometry(podRadius, 56, 40);
      const glassBubble = new THREE.Mesh(glassSphereGeo, mats.glass);
      glassBubble.scale.set(1.0, 0.98, 1.06);
      glassBubble.position.y = 0.06;
      glassBubble.renderOrder = 3;
      centerPod.add(glassBubble);

      const fresnelShell = new THREE.Mesh(glassSphereGeo, fresnelMat);
      fresnelShell.scale.set(1.006, 0.986, 1.066);
      fresnelShell.position.y = 0.06;
      fresnelShell.renderOrder = 4;
      centerPod.add(fresnelShell);

      // Signature arched side door hatch ribs & spine arch wrapping around bubble
      [-1, 1].forEach((side) => {
        const hatchRib = new THREE.Mesh(
          new THREE.TorusGeometry(podRadius * 0.77, 0.014, 12, 52),
          mats.rib
        );
        hatchRib.position.set(side * 0.39, 0.12, 0.02);
        hatchRib.rotation.y = side * (Math.PI * 0.44);
        centerPod.add(hatchRib);
      });

      const dorsalArch = new THREE.Mesh(
        new THREE.TorusGeometry(podRadius * 1.01, 0.014, 12, 48, Math.PI * 0.72),
        mats.rib
      );
      dorsalArch.position.y = 0.06;
      dorsalArch.rotation.y = Math.PI / 2;
      dorsalArch.rotation.z = Math.PI * 0.14;
      dorsalArch.scale.set(1.06, 0.98, 1.0);
      centerPod.add(dorsalArch);

      /* ----------------------------------------------------------------------
       * 2. FOUR SWEPT QUAD-X ARMS + VERTICAL UPWARD-FACING DUCTED FAN NACELLES
       * ---------------------------------------------------------------------- */

      // Reusable curved streamlined turbine nacelle profile (LatheGeometry)
      // Vertical axis = +Y (intake at top, flared nozzle exhaust at bottom)
      const shroudPoints = [
        new THREE.Vector2(0.32, -0.20), // Bottom inner exhaust edge
        new THREE.Vector2(0.35, -0.22), // Bottom nozzle lip
        new THREE.Vector2(0.41, -0.17), // Lower rounded flare
        new THREE.Vector2(0.47, -0.05), // Bulging futuristic sci-fi mid-pod
        new THREE.Vector2(0.48,  0.06), // Upper shoulder curve
        new THREE.Vector2(0.44,  0.17), // Tapering intake curve
        new THREE.Vector2(0.38,  0.22), // Top outer intake lip
        new THREE.Vector2(0.33,  0.20), // Top inner intake curve
        new THREE.Vector2(0.31,  0.08), // Throat wall
        new THREE.Vector2(0.30, -0.06), // Impeller shroud clearance
        new THREE.Vector2(0.32, -0.20)  // Close profile at bottom inner edge
      ];
      const shroudGeo = new THREE.LatheGeometry(shroudPoints, 48);
      shroudGeo.computeVertexNormals();

      // Reusable sculpted turbine blade shape (airfoil blade extending along +X)
      const bladeShape = new THREE.Shape();
      bladeShape.moveTo(0.085, -0.018);
      bladeShape.quadraticCurveTo(0.18, -0.036, 0.285, -0.022);
      bladeShape.lineTo(0.292, 0.018);
      bladeShape.quadraticCurveTo(0.18, 0.034, 0.085, 0.020);
      bladeShape.closePath();

      const bladeGeo = new THREE.ExtrudeGeometry(bladeShape, {
        depth: 0.008,
        bevelEnabled: true,
        bevelThickness: 0.003,
        bevelSize: 0.003,
        bevelSegments: 2
      });
      // Center along Z so thickness is balanced around Y after rotation
      bladeGeo.translate(0, 0, -0.004);
      // Lay blade initially flat in the XZ plane (chord along Z, span along +X)
      bladeGeo.rotateX(Math.PI / 2);

      // PX4 Quad-X exact specification
      // Cấu hình Quad-X chuẩn xác theo khung UAV thực tế (đầu ở giữa động cơ 1 và 4):
      //   M1: GPIO 7 (Trước-Trái, CCW) -> x = -diag, z = -diag (Cyan)
      //   M4: GPIO 4 (Trước-Phải, CW)  -> x = +diag, z = -diag (Purple)
      //   M3: GPIO 5 (Sau-Trái,   CW)  -> x = -diag, z = +diag (Amber)
      //   M2: GPIO 6 (Sau-Phải,   CCW) -> x = +diag, z = +diag (Green)
      // Đầu UAV (mũi/cabin/đèn pha tại -Z) nằm chính giữa động cơ 1 (FL) và 4 (FR)!
      const armReach = 1.16;
      const diag = armReach * Math.SQRT1_2; // 0.820
      const motorSpecs = [
        { id: 'M1', name: 'M1 FL (P7-CCW)', x: -diag, z: -diag, dir: +1, color: 0x22d3ee, spinAxis: 'y' }, // M1 Trước-Trái (Cyan)
        { id: 'M2', name: 'M2 RR (P6-CCW)', x: +diag, z: +diag, dir: +1, color: 0x10b981, spinAxis: 'y' }, // M2 Sau-Phải   (Green)
        { id: 'M3', name: 'M3 RL (P5-CW)',  x: -diag, z: +diag, dir: -1, color: 0xf59e0b, spinAxis: 'y' }, // M3 Sau-Trái   (Amber)
        { id: 'M4', name: 'M4 FR (P4-CW)',  x: +diag, z: -diag, dir: -1, color: 0xa855f7, spinAxis: 'y' }  // M4 Trước-Phải (Purple)
      ];

      const motors = [];
      const spinArrowsGroup = new THREE.Group();
      root.add(spinArrowsGroup);

      // Build each arm + vertical ducted turbine pod
      motorSpecs.forEach((spec) => {
        const armAngle = Math.atan2(spec.x, spec.z);

        // --- 2A. Sculpted Swept Aerodynamic Arm ---
        const armPivot = new THREE.Group();
        armPivot.rotation.y = armAngle;
        root.add(armPivot);

        // Sleek aerodynamic strut extending along local +Z from hull to nacelle
        const armLen = armReach - 0.34;
        const armCore = new THREE.Mesh(
          new THREE.BoxGeometry(0.17, 0.09, armLen),
          mats.arm
        );
        armCore.position.set(0, -0.14, 0.34 + armLen * 0.5);
        armCore.scale.set(1.0, 0.9, 1.0);
        armPivot.add(armCore);

        // Tapered fairing fins smoothing arm into body
        const upperFairing = new THREE.Mesh(
          new THREE.CylinderGeometry(0.06, 0.11, armLen * 0.95, 16),
          mats.shroud
        );
        upperFairing.rotation.x = Math.PI / 2;
        upperFairing.scale.set(1.45, 1.0, 0.65);
        upperFairing.position.set(0, -0.11, 0.32 + armLen * 0.48);
        armPivot.add(upperFairing);

        // Glowing cyan energy trim line running along the top of each arm
        const armTrimMat = new THREE.MeshStandardMaterial({
          color: spec.color,
          emissive: spec.color,
          emissiveIntensity: 1.4
        });
        const armTrim = new THREE.Mesh(
          new THREE.BoxGeometry(0.022, 0.015, armLen * 0.78),
          armTrimMat
        );
        armTrim.position.set(0, -0.055, 0.42 + armLen * 0.36);
        armPivot.add(armTrim);

        // --- 2B. Vertical Upward-Facing Ducted Fan Nacelle ---
        const nacelle = new THREE.Group();
        nacelle.position.set(spec.x, -0.06, spec.z);
        root.add(nacelle);

        // Main glossy cyan-silver rounded turbine shroud
        const shroudMesh = new THREE.Mesh(shroudGeo, mats.shroud);
        shroudMesh.castShadow = true;
        nacelle.add(shroudMesh);

        // Dark titanium internal duct liner ring
        const innerLiner = new THREE.Mesh(
          new THREE.CylinderGeometry(0.305, 0.305, 0.28, 36, 1, true),
          new THREE.MeshStandardMaterial({
            color: 0x091320,
            metalness: 0.7,
            roughness: 0.45,
            side: THREE.BackSide
          })
        );
        nacelle.add(innerLiner);

        // Vivid NEON-GREEN Intake LED Ring along top inner lip (iconic detail!)
        const topGreenRing = new THREE.Mesh(
          new THREE.TorusGeometry(0.352, 0.017, 16, 48),
          mats.intakeRing
        );
        topGreenRing.rotation.x = Math.PI / 2;
        topGreenRing.position.y = 0.205;
        nacelle.add(topGreenRing);

        // Dynamic CYAN-BLUE Exhaust Plasma Ring at bottom outlet
        const exhaustRingMat = new THREE.MeshStandardMaterial({
          color: THEMES.cyan.exhaustRing,
          emissive: THEMES.cyan.exhaustRing,
          emissiveIntensity: 0.9,
          roughness: 0.2
        });
        const bottomPlasmaRing = new THREE.Mesh(
          new THREE.TorusGeometry(0.325, 0.015, 14, 44),
          exhaustRingMat
        );
        bottomPlasmaRing.rotation.x = Math.PI / 2;
        bottomPlasmaRing.position.y = -0.202;
        nacelle.add(bottomPlasmaRing);

        // 3 Internal aerodynamic stator vanes supporting the motor hub below the fan
        for (let s = 0; s < 3; s++) {
          const vane = new THREE.Mesh(
            new THREE.BoxGeometry(0.29, 0.022, 0.032),
            mats.darkTitanium
          );
          const a = (s / 3) * Math.PI * 2;
          vane.position.set(Math.cos(a) * 0.145, -0.11, Math.sin(a) * 0.145);
          vane.rotation.y = -a;
          nacelle.add(vane);
        }

        // Brushless motor stator core housing underneath spinner
        const motorBody = new THREE.Mesh(
          new THREE.CylinderGeometry(0.088, 0.072, 0.16, 24),
          mats.darkTitanium
        );
        motorBody.position.y = -0.07;
        nacelle.add(motorBody);

        // --- 2C. 8-Blade Physically Pitched Turbine Impeller ---
        const rotorHub = new THREE.Group();
        rotorHub.position.y = 0.03;
        nacelle.add(rotorHub);

        // Metallic spinner cone pointing UP
        const spinner = new THREE.Mesh(
          new THREE.ConeGeometry(0.088, 0.14, 24),
          mats.chromeMetal
        );
        spinner.position.y = 0.05;
        rotorHub.add(spinner);

        // Glowing cyan tip cap on center spinner cone
        const spinnerTip = new THREE.Mesh(
          new THREE.SphereGeometry(0.032, 16, 12),
          new THREE.MeshBasicMaterial({ color: 0x7ef9ff })
        );
        spinnerTip.position.y = 0.11;
        rotorHub.add(spinnerTip);

        // 8 Individually pitched blades:
        //   CCW (dir = +1): blade along +X moves toward -Z -> rotate +alpha around X so -Z leading edge is UP!
        //   CW  (dir = -1): blade along +X moves toward +Z -> rotate -alpha around X so +Z leading edge is UP!
        // Both push air cleanly DOWNWARD (-Y) for vertical thrust!
        const bladeCount = 8;
        const pitchAngleRad = 0.44; // ~25.2 deg physical pitch
        const bladesGroup = new THREE.Group();
        rotorHub.add(bladesGroup);

        for (let b = 0; b < bladeCount; b++) {
          const bladePivot = new THREE.Group();
          bladePivot.rotation.y = (b / bladeCount) * Math.PI * 2;

          const bladeMesh = new THREE.Mesh(bladeGeo, mats.turbineBlade);
          bladeMesh.rotation.x = spec.dir * pitchAngleRad;
          bladePivot.add(bladeMesh);

          // Tiny glowing cyan tip accent on blade 0 & 4 to clearly visualize spin direction
          if (b === 0 || b === 4) {
            const tipAccent = new THREE.Mesh(
              new THREE.BoxGeometry(0.032, 0.008, 0.025),
              new THREE.MeshBasicMaterial({ color: spec.color })
            );
            tipAccent.position.set(0.275, 0.006, 0);
            tipAccent.rotation.x = spec.dir * pitchAngleRad;
            bladePivot.add(tipAccent);
          }

          bladesGroup.add(bladePivot);
        }

        // --- 2D. Dynamic Motion-Blur Disc (fades in smoothly with RPM) ---
        const blurMat = new THREE.MeshBasicMaterial({
          map: blurTex,
          transparent: true,
          opacity: 0.0,
          depthWrite: false,
          side: THREE.DoubleSide,
          blending: THREE.AdditiveBlending
        });
        const blurDisc = new THREE.Mesh(new THREE.CircleGeometry(0.30, 36), blurMat);
        blurDisc.rotation.x = -Math.PI / 2;
        blurDisc.position.y = 0.045;
        nacelle.add(blurDisc);

        // --- 2E. Volumetric Downward Thrust Exhaust Plume Cone ---
        const plumeGroup = new THREE.Group();
        plumeGroup.position.y = -0.21;
        nacelle.add(plumeGroup);

        // Outer cyan-green thrust cone
        const outerPlumeGeo = new THREE.CylinderGeometry(0.26, 0.09, 1.0, 28, 1, true);
        outerPlumeGeo.translate(0, -0.5, 0); // Anchor top at y = 0 so scale.y grows downward
        const outerPlumeMat = new THREE.MeshBasicMaterial({
          map: plumeTex,
          color: 0x22f0ff,
          transparent: true,
          opacity: 0.0,
          depthWrite: false,
          side: THREE.DoubleSide,
          blending: THREE.AdditiveBlending
        });
        const outerPlume = new THREE.Mesh(outerPlumeGeo, outerPlumeMat);
        plumeGroup.add(outerPlume);

        // Bright inner plasma thrust core
        const innerPlumeGeo = new THREE.CylinderGeometry(0.14, 0.02, 0.78, 20, 1, true);
        innerPlumeGeo.translate(0, -0.39, 0);
        const innerPlumeMat = new THREE.MeshBasicMaterial({
          map: plumeTex,
          color: 0xb8ffff,
          transparent: true,
          opacity: 0.0,
          depthWrite: false,
          side: THREE.DoubleSide,
          blending: THREE.AdditiveBlending
        });
        const innerPlume = new THREE.Mesh(innerPlumeGeo, innerPlumeMat);
        plumeGroup.add(innerPlume);

        // --- 2F. 3D Curved Spin Direction Arrow Above Nacelle ---
        const arrowHolder = new THREE.Group();
        arrowHolder.position.set(spec.x, 0.28, spec.z);
        spinArrowsGroup.add(arrowHolder);

        const arcRadius = 0.36;
        const arcSpan = Math.PI * 1.15;
        const arcCurvePts = [];
        const segs = 28;
        for (let i = 0; i <= segs; i++) {
          const t = i / segs;
          // CCW (+1): angle sweeps positively (x=cos, z=-sin moves +X -> -Z = CCW from top!)
          // CW  (-1): angle sweeps negatively (x=cos, z=-sin moves +X -> +Z = CW from top!)
          const theta = -0.4 + spec.dir * t * arcSpan;
          arcCurvePts.push(new THREE.Vector3(Math.cos(theta) * arcRadius, 0, -Math.sin(theta) * arcRadius));
        }
        const arcCurve = new THREE.CatmullRomCurve3(arcCurvePts);
        const arrowMat = new THREE.MeshBasicMaterial({
          color: spec.color,
          transparent: true,
          opacity: 0.88
        });
        const arcTube = new THREE.Mesh(new THREE.TubeGeometry(arcCurve, 28, 0.012, 8, false), arrowMat);
        arrowHolder.add(arcTube);

        // Arrowhead cone tangent to curve end
        const endPt = arcCurvePts[arcCurvePts.length - 1];
        const prevPt = arcCurvePts[arcCurvePts.length - 2];
        const tangent = new THREE.Vector3().subVectors(endPt, prevPt).normalize();
        const arrowHead = new THREE.Mesh(new THREE.ConeGeometry(0.04, 0.11, 12), arrowMat);
        arrowHead.position.copy(endPt);
        arrowHead.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), tangent);
        arrowHolder.add(arrowHead);

        // Label anchor point in 3D space (projected to HUD tags)
        const labelAnchor = new THREE.Object3D();
        labelAnchor.position.set(0, 0.44, 0);
        nacelle.add(labelAnchor);

        motors.push({
          ...spec,
          nacelle,
          rotorHub,
          bladesGroup,
          blurDisc,
          blurMat,
          plumeGroup,
          outerPlumeMat,
          innerPlumeMat,
          exhaustRingMat,
          arrowHolder,
          labelAnchor,
          currentOmega: 0 // Smoothly integrated angular velocity (rad/s)
        });
      });

      /**
       * Switch color theme dynamically ('cyan' | 'stealth')
       */
      function setTheme(themeKey) {
        const t = THEMES[themeKey] || THEMES.cyan;
        mats.shroud.color.setHex(t.shroudColor);
        mats.shroud.emissive.setHex(t.shroudEmissive);
        mats.arm.color.setHex(t.armColor);
        mats.hull.color.setHex(t.hullColor);
        mats.glass.color.setHex(t.glassColor);
        mats.rib.color.setHex(t.ribColor);
        mats.intakeRing.color.setHex(t.intakeRing);
        mats.intakeRing.emissive.setHex(t.intakeRing);
        mats.seat.color.setHex(t.seatColor);
        fresnelUniforms.uColor.value.setHex(t.fresnelColor);
        motors.forEach((m) => {
          m.exhaustRingMat.color.setHex(t.exhaustRing);
          m.exhaustRingMat.emissive.setHex(t.exhaustRing);
        });
      }

      /**
       * Per-frame physics & visual update for all 4 ducted turbine rotors.
       * @param {number} dt - Delta time in seconds
       * @param {number[]} targetOmegas - Target angular speeds [M1, M2, M3, M4] in rad/s
       * @param {boolean} isArmed - Armed state
       * @param {number} elapsed - Total elapsed time for subtle plasma flicker
       */
      function update(dt, targetOmegas, isArmed, elapsed) {
        // Realistic motor spool-up / spool-down inertia time constant
        const spoolRate = isArmed ? 6.5 : 3.2;
        const alpha = 1 - Math.exp(-spoolRate * dt);

        motors.forEach((m, idx) => {
          const rawTarget = isArmed ? Math.max(0, targetOmegas[idx] || 0) : 0;
          const target = rawTarget > 1.0 ? rawTarget : rawTarget * 1000.0;
          m.currentOmega += (target - m.currentOmega) * alpha;
          if (!isArmed && m.currentOmega < 0.8) m.currentOmega = 0;

          const omega = m.currentOmega;
          const norm = Math.min(omega / 1000, 1.0);

          // Anti-aliasing visual speed cap so ultra-high RPM never strobes backward on 60Hz monitors,
          // while low-to-mid RPM integrates exact physical `dir * omega * dt`.
          const visualOmega = omega <= 45 ? omega : 45 + Math.log1p(omega - 45) * 9.5;
          m.rotorHub.rotation.y += m.dir * visualOmega * dt;

          // Slowly rotate blur disc at high speed with subtle shimmer
          m.blurDisc.rotation.z -= m.dir * (visualOmega * 0.65) * dt;
          const blurOpacity = THREE.MathUtils.clamp((omega - 90) / 480, 0, 0.78);
          m.blurMat.opacity = blurOpacity;

          // Fade solid blades slightly at extreme RPM for realistic motion-blur blend
          m.bladesGroup.visible = true;

          // Scale downward exhaust thrust cone & plasma brightness proportionally to rad/s
          const shimmer = 0.94 + Math.sin(elapsed * 34 + idx * 1.7) * 0.06;
          if (norm > 0.02) {
            m.plumeGroup.visible = true;
            const lengthScale = (0.18 + norm * 1.18) * shimmer;
            const radiusScale = 0.82 + norm * 0.24;
            m.plumeGroup.scale.set(radiusScale, lengthScale, radiusScale);
            m.outerPlumeMat.opacity = Math.min(0.85, norm * 0.85) * shimmer;
            m.innerPlumeMat.opacity = Math.min(0.95, norm * 1.05) * shimmer;
          } else {
            m.plumeGroup.visible = false;
            m.outerPlumeMat.opacity = 0;
            m.innerPlumeMat.opacity = 0;
          }

          // Exhaust plasma ring intensifies with thrust
          m.exhaustRingMat.emissiveIntensity = 0.55 + norm * 2.8;
        });
      }

      return {
        root,
        motors,
        spinArrowsGroup,
        setTheme,
        update
      };
    }


class DroneModelLoader {
  static _modelCache = {};

  static MODEL_DEFINITIONS = {
    sci_fi_bubble: {
      name: "ISILAB Sci-Fi Bubble Quad-X (PBR Vũ Trụ)",
      isProcedural: true
    },
    glb_annelida: {
      name: "ISILAB Phantom-Aero (White Commercial)",
      file: "monitor_Web_resource/assets/model_annelida.glb",
      targetSpan: 2.60,
      rotationY: Math.PI,
      propellerNames: [
        "Propeller2", // M1 FL
        "Propeller3", // M2 RR
        "Propeller4", // M3 RL
        "Propeller1"  // M4 FR
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
        "Rotor_FL", // M1 FL
        "Rotor_BR", // M2 RR
        "Rotor_BL", // M3 RL
        "Rotor_FR"  // M4 FR
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
        "Circle.004", // M1 FL
        "Circle.005", // M2 RR
        "Circle.003", // M3 RL
        "Circle.002"  // M4 FR
      ],
      spinAxis: "z",
      badgeOffset: { y: 0.12, z: 0, rx: -Math.PI / 2, scale: 0.35 }
    }
  };

  /**
   * Tải mô hình 3D được chọn (mặc định Sci-Fi Bubble Quad-X hoặc các model GLB).
   * @param {THREE.Group} droneGroup Nhóm cha đại diện cho hướng bay (Attitude).
   * @param {Function} onPropellersReady Callback nhận 4 node rotor [M1..M4].
   * @param {string} modelKey Khóa mô hình ("sci_fi_bubble", "glb_cesium", "glb_annelida", ...).
   * @param {THREE.WebGLRenderer} renderer Bộ dựng Three.js (dùng tạo môi trường PBR studio).
   */
  static loadQuadcopterModel(droneGroup, onPropellersReady, modelKey = "sci_fi_bubble", renderer = null) {
    // Ánh xạ các tên alias cũ và thay thế hoàn toàn mẫu con ruồi cũ (glb_cesium) bằng sci_fi_bubble
    if (!modelKey || modelKey === "glb_cesium" || modelKey === "glb_aero") {
      modelKey = "sci_fi_bubble";
    }
    if (modelKey === "mavic_recon") modelKey = "glb_annelida";
    if (modelKey === "fpv_apex") modelKey = "glb_laser";
    if (modelKey === "lab_heavy") modelKey = "glb_s9";

    const clearGroup = () => {
      while (droneGroup.children.length > 0) {
        droneGroup.remove(droneGroup.children[0]);
      }
      droneGroup.userData.sciFiQuad = null;
    };

    // 1. Nếu là mẫu Procedural Sci-Fi Bubble Quad-X (Mẫu mặc định cao cấp nhất)
    if (modelKey === "sci_fi_bubble") {
      clearGroup();
      let envMap = _cachedStudioEnvMap;
      if (!envMap && renderer) {
        try {
          envMap = createStudioEnvironment(renderer);
          _cachedStudioEnvMap = envMap;
        } catch (err) {
          console.warn("[3D] Studio environment generation skipped:", err);
        }
      }
      const sciFiQuad = createSciFiBubbleQuad(envMap);
      droneGroup.add(sciFiQuad.root);
      droneGroup.userData.sciFiQuad = sciFiQuad;

      // Trích xuất 4 rotor hub [M1, M2, M3, M4]
      const propellers = sciFiQuad.motors.map((m) => m.rotorHub);
      onPropellersReady(propellers);
      console.log("[3D] Loaded ISILAB Sci-Fi Bubble Quad-X (Procedural High-Tech 3D Model with Cockpit & Ducted Fans).");
      return;
    }

    // 2. Nếu là các mô hình file GLB
    const def = DroneModelLoader.MODEL_DEFINITIONS[modelKey] || DroneModelLoader.MODEL_DEFINITIONS.glb_cesium;

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

    // Kiểm tra bộ nhớ cache
    if (DroneModelLoader._modelCache[modelKey]) {
      attachModelInstance(DroneModelLoader._modelCache[modelKey]);
      return;
    }

    const gltfLoader = new THREE.GLTFLoader();

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
   * Gắn 4 vòng LED định danh màu sắc 4 kênh Motor chuẩn hình học chữ Z:
   * M1 FL (Trước Trái): Cyan (#32ade6)
   * M2 RR (Sau Phải):   Green (#30d158)
   * M3 RL (Sau Trái):   Amber (#ffd60a)
   * M4 FR (Trước Phải): Purple (#bf5af2)
   */
  static attachFourMotorLedRings(wrapper) {
    const armRadius = 1.05;
    const ledSpecs = [
      { x: -armRadius * 0.707, z: -armRadius * 0.707, color: 0x32ade6 }, // M1 FL (Trước-Trái - Cyan)
      { x:  armRadius * 0.707, z:  armRadius * 0.707, color: 0x30d158 }, // M2 RR (Sau-Phải   - Green)
      { x: -armRadius * 0.707, z:  armRadius * 0.707, color: 0xffd60a }, // M3 RL (Sau-Trái   - Amber)
      { x:  armRadius * 0.707, z: -armRadius * 0.707, color: 0xbf5af2 }  // M4 FR (Trước-Phải - Purple)
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
