/**
 * Manages telemetry samples, synthetic demo signals, WebSerial, and WebSocket streaming.
 */
class TelemetryDataStore {
  constructor() {
    this.sampleBuffer = [];
    this.timeSpanSeconds = 5;
    this.totalPacketCount = 0;
    this.lastReceivedTimestampSeconds = 0;
    this.connectionMode = "demo";
    this.webSocketConnection = null;
    this.serialPort = null;
    this.serialReader = null;
    this.serialWriter = null;
    this.packetFrequencyHertz = 0;
    this.packetsInCurrentSecond = 0;

    // Cache trạng thái xác nhận từ ESP32 firmware
    this.lastEspState = "DISARMED";
    this.lastRcSequence = 0;
    this.lastTargetThrottle = 0;

    // Cache giá trị trạng thái tức thời
    this.currentRoll = 0;
    this.currentPitch = 0;
    this.currentYaw = 0;
    this.currentZ = 0;
    this.currentVz = 0;
    this.currentMotors = [0, 0, 0, 0];
    this.currentGyro = [0, 0, 0];
    this.currentAcc = [0, 0, 0];

    // Chẩn đoán đường truyền IMU (ESP32 -> Web)
    this.jsonParseErrors = 0;      // Gói JSON hỏng (trước đây bị nuốt im lặng)
    this.lastEspTimestampMs = null;
    this.espGapDropCount = 0;      // Số lần khoảng cách giữa 2 gói ESP (theo đồng hồ ESP) > 60 ms
    this.espMaxGapMs = 0;

    this.startPeriodicTimers();
  }

  startPeriodicTimers() {
    setInterval(() => {
      this.packetFrequencyHertz = this.packetsInCurrentSecond;
      this.packetsInCurrentSecond = 0;
    }, 1000);

    setInterval(() => {
      if (this.connectionMode !== "demo") return;
      const elapsedSeconds = performance.now() / 1000;
      const rollDegrees = 14 * Math.sin(elapsedSeconds * 1.1) + 3 * Math.sin(elapsedSeconds * 5);
      const pitchDegrees = 9 * Math.sin(elapsedSeconds * 0.8 + 1) + 2 * Math.sin(elapsedSeconds * 4);
      const yawDegrees = ((elapsedSeconds * 18) % 360) - 180;
      const baseSpeed = 770 + 25 * Math.sin(elapsedSeconds * 2);
      this.pushSample({
        roll: rollDegrees,
        pitch: pitchDegrees,
        yaw: yawDegrees,
        z: 2 + 0.3 * Math.sin(elapsedSeconds * 0.7),
        vz: 0.2 * Math.cos(elapsedSeconds * 0.7),
        m: [baseSpeed + rollDegrees, baseSpeed - rollDegrees, baseSpeed + pitchDegrees, baseSpeed - pitchDegrees]
      });
    }, 25);
  }

  pushSample(rawSample) {
    const timestampSeconds = performance.now() / 1000;
    const num = (v, fallback = 0) => {
      const n = Number(v);
      return Number.isFinite(n) ? n : fallback;
    };
    const arr = (v, len) => (Array.isArray(v) ? v.slice(0, len).map((x) => num(x)) : null);
    this.sampleBuffer.push({
      // GIỮ NGUYÊN các trường IMU/PID: bản cũ chỉ giữ góc + motor nên gx/gy/gz, setpoint,
      // tau, u và khối "tune" bị loại bỏ => panel Auto-Tune trên Web luôn đọc gx = 0 và
      // không bao giờ hiện trạng thái tune từ ESP32.
      espT: rawSample.t !== undefined ? num(rawSample.t) : null,
      gx: rawSample.gx !== undefined ? num(rawSample.gx) : this.currentGyro[0],
      gy: rawSample.gy !== undefined ? num(rawSample.gy) : this.currentGyro[1],
      gz: rawSample.gz !== undefined ? num(rawSample.gz) : this.currentGyro[2],
      ax: rawSample.ax !== undefined ? num(rawSample.ax) : this.currentAcc[0],
      ay: rawSample.ay !== undefined ? num(rawSample.ay) : this.currentAcc[1],
      az: rawSample.az !== undefined ? num(rawSample.az) : this.currentAcc[2],
      sp_r: num(rawSample.sp_r),
      sp_p: num(rawSample.sp_p),
      sp_y: num(rawSample.sp_y),
      sp_t: num(rawSample.sp_t),
      rsp: arr(rawSample.rsp, 3),
      pid_i: arr(rawSample.pid_i, 3),
      tau: arr(rawSample.tau, 3),
      u: arr(rawSample.u, 4),
      us: arr(rawSample.us, 4),
      yaw_ok: rawSample.yaw_ok,
      imu_src: rawSample.imu_src,
      loop_hz: rawSample.loop_hz !== undefined ? num(rawSample.loop_hz) : undefined,
      loop_max_us: rawSample.loop_max_us !== undefined ? num(rawSample.loop_max_us) : undefined,
      xs_age: rawSample.xs_age !== undefined ? num(rawSample.xs_age) : undefined,
      state: rawSample.state,
      tune: rawSample.tune,
      t: timestampSeconds,
      roll: Number(rawSample.roll) || 0,
      pitch: Number(rawSample.pitch) || 0,
      yaw: Number(rawSample.yaw) || 0,
      mpu_r: rawSample.mpu_r !== undefined ? Number(rawSample.mpu_r) : (this.currentMpuR || 0),
      mpu_p: rawSample.mpu_p !== undefined ? Number(rawSample.mpu_p) : (this.currentMpuP || 0),
      mpu_y: rawSample.mpu_y !== undefined ? Number(rawSample.mpu_y) : (this.currentMpuY || 0),
      xs_r: rawSample.xs_r !== undefined ? Number(rawSample.xs_r) : (this.currentXsR || 0),
      xs_p: rawSample.xs_p !== undefined ? Number(rawSample.xs_p) : (this.currentXsP || 0),
      xs_y: rawSample.xs_y !== undefined ? Number(rawSample.xs_y) : (this.currentXsY || 0),
      z: Number(rawSample.z) || 0,
      vz: Number(rawSample.vz) || 0,
      m: (Array.isArray(rawSample.m) ? rawSample.m : [0, 0, 0, 0]).map((x) => num(x))
    });

    // Phát hiện mất gói theo đồng hồ ESP32 (độc lập với độ trễ WiFi/trình duyệt)
    if (rawSample.t !== undefined) {
      const espT = num(rawSample.t);
      if (this.lastEspTimestampMs !== null) {
        const gap = (espT - this.lastEspTimestampMs) >>> 0; // chịu được tràn uint32 millis()
        if (gap > 60 && gap < 600000) this.espGapDropCount += 1;
        if (gap < 600000) this.espMaxGapMs = Math.max(this.espMaxGapMs * 0.995, gap);
      }
      this.lastEspTimestampMs = espT;
    }

    while (this.sampleBuffer.length > 0 && timestampSeconds - this.sampleBuffer[0].t > 10) {
      this.sampleBuffer.shift();
    }
    this.totalPacketCount += 1;
    this.packetsInCurrentSecond += 1;
    this.lastReceivedTimestampSeconds = timestampSeconds;
  }

  /**
   * KẾT NỐI TRỰC TIẾP QUA CÁP USB SERIAL (WebSerial API - Chrome/Edge)
   * Không phụ thuộc WiFi, 0ms latency, không nghẽn mạng!
   */
  async connectWebSerial() {
    if (!('serial' in navigator)) {
      alert("Trình duyệt không hỗ trợ WebSerial API. Vui lòng dùng Google Chrome hoặc Microsoft Edge.");
      return false;
    }

    this.connectionMode = "serial";
    if (this.webSocketConnection) {
      this.webSocketConnection.close();
      this.webSocketConnection = null;
    }

    try {
      this.serialPort = await navigator.serial.requestPort();
      await this.serialPort.open({ baudRate: 115200 });

      this.serialReader = this.serialPort.readable.getReader();
      const decoder = new TextDecoder();
      let bufferAccumulator = "";

      const readLoop = async () => {
        while (this.connectionMode === "serial") {
          try {
            const { value, done } = await this.serialReader.read();
            if (done) break;
            bufferAccumulator += decoder.decode(value, { stream: true });
            const lines = bufferAccumulator.split("\n");
            bufferAccumulator = lines.pop();

            for (const line of lines) {
              this.parseSerialLine(line.trim());
            }
          } catch (readErr) {
            console.error("[WebSerial Read Error]", readErr);
            break;
          }
        }
      };

      readLoop();
      console.log("[WebSerial] Connected successfully @ 115200 baud.");
      return true;
    } catch (err) {
      console.error("[WebSerial Error]", err);
      alert("Không thể kết nối cổng Serial: " + err.message);
      this.enableDemoMode();
      return false;
    }
  }

  parseSerialLine(line) {
    if (!line) return;

    const parseVec3 = (label) => {
      const mm = line.match(new RegExp(label + ":\\[\\s*([-\\d.]+),\\s*([-\\d.]+),\\s*([-\\d.]+)\\]"));
      return mm ? [parseFloat(mm[1]), parseFloat(mm[2]), parseFloat(mm[3])] : null;
    };

    // 1. Phân tích gói MPU-6050:
    if (line.includes("[MPU6050]")) {
      const mR = line.match(/R:\s*([-\d.]+)/);
      const mP = line.match(/P:\s*([-\d.]+)/);
      const mY = line.match(/Y:\s*([-\d.]+)/);
      if (mR) this.currentMpuR = parseFloat(mR[1]);
      if (mP) this.currentMpuP = parseFloat(mP[1]);
      if (mY) this.currentMpuY = parseFloat(mY[1]);
    }
    // 2. Phân tích gói Xsens MTi-3:
    else if (line.includes("[Xsens  ]") || line.includes("[Xsens]")) {
      const mR = line.match(/R:\s*([-\d.]+)/);
      const mP = line.match(/P:\s*([-\d.]+)/);
      const mY = line.match(/Y:\s*([-\d.]+)/);
      if (mR) this.currentXsR = parseFloat(mR[1]);
      if (mP) this.currentXsP = parseFloat(mP[1]);
      if (mY) this.currentXsY = parseFloat(mY[1]);
    }
    // 3. Phân tích gói trạng thái dung hợp mà bộ điều khiển đang dùng:
    // [PX4_EKF] R:   1.2° P:  -0.5° Y:  82.4° | Acc:[  0.20, -0.16,  9.13] | Gyr:[  -0.1,   0.1,   0.0] (SRC:XSENS YAW:OK)
    // (firmware cũ in tag [ATTITUDE] => vẫn hỗ trợ để tương thích ngược)
    else if (line.includes("[PX4_EKF]") || line.includes("[ATTITUDE]")) {
      const mR = line.match(/R:\s*([-\d.]+)/);
      const mP = line.match(/P:\s*([-\d.]+)/);
      const mY = line.match(/Y:\s*([-\d.]+)/);
      if (mR && mP && mY) {
        this.currentRoll = parseFloat(mR[1]);
        this.currentPitch = parseFloat(mP[1]);
        this.currentYaw = parseFloat(mY[1]);
      }
      const gyr = parseVec3("Gyr");
      if (gyr) this.currentGyro = gyr;
      const acc = parseVec3("Acc");
      if (acc) this.currentAcc = acc;
    }
    // 4. Phân tích trạng thái động cơ (dòng cuối của mỗi chu kỳ Telemetry):
    // [ESC_QUAD] State:ARMED | M1(P7-CCW):1000us( 0.0%) | M2(P6-CCW):1000us( 0.0%) | M3(P5-CW):1000us( 0.0%) | M4(P4-CW):1000us( 0.0%) @ 50Hz
    else if (line.includes("[ESC_QUAD]")) {
      if (line.includes("State:ARMED")) {
        this.lastEspState = "ARMED";
      } else if (line.includes("State:DISARMED")) {
        this.lastEspState = "DISARMED";
      }
      const pcts = [...line.matchAll(/\(\s*([-\d.]+)%\)/g)];
      if (pcts.length >= 4) {
        this.currentMotors = [
          parseFloat(pcts[0][1]) * 10,
          parseFloat(pcts[1][1]) * 10,
          parseFloat(pcts[2][1]) * 10,
          parseFloat(pcts[3][1]) * 10
        ];
      }
      this.pushSample({
        roll: this.currentRoll,
        pitch: this.currentPitch,
        yaw: this.currentYaw,
        z: this.currentZ,
        vz: this.currentVz,
        m: this.currentMotors
      });
    }
  }

  /**
   * KẾT NỐI KHÔNG DÂY QUA SÓNG WIFI SOFTAP (WebSocket)
   */
  connectWebSocket(endpointUrl) {
    this.connectionMode = "ws";
    if (this.serialReader) {
      try { this.serialReader.cancel(); } catch (_e) {}
    }
    if (this.serialPort) {
      try { this.serialPort.close(); } catch (_e) {}
      this.serialPort = null;
    }

    try {
      if (this.webSocketConnection) {
        this.webSocketConnection.onopen = null;
        this.webSocketConnection.onclose = null;
        this.webSocketConnection.onerror = null;
        this.webSocketConnection.onmessage = null;
        this.webSocketConnection.close();
      }
      this.webSocketConnection = new WebSocket(endpointUrl);
      this.webSocketConnection.onopen = () => {
        console.log("[WebSocket] Đã kết nối thành công tới ESP32:", endpointUrl);
        const bLiveWs = document.getElementById("bLiveWs");
        if (bLiveWs) {
          bLiveWs.style.background = "#16a34a";
          bLiveWs.textContent = "✅ Đã Kết Nối WiFi";
        }
      };
      this.webSocketConnection.onmessage = (event) => {
        try {
          const parsed = JSON.parse(event.data);
          const linkState = document.getElementById("espLinkState");
          const rcState = document.getElementById("espRcState");
          if (parsed) {
            if (parsed.state !== undefined) this.lastEspState = parsed.state;
            if (parsed.rc_seq !== undefined) this.lastRcSequence = parsed.rc_seq;
            if (parsed.sp_t !== undefined) this.lastTargetThrottle = Number(parsed.sp_t);
          }
          if (linkState && parsed && parsed.state !== undefined) {
            const network = parsed.sta_connected ? `VNU ${parsed.sta_ip || ""}` : "SoftAP/STA offline";
            const pwm = parsed.pwm_ok === 0 ? "PWM lỗi" : "PWM OK";
            const imuNames = { 0: "IMU MẤT", 1: "IMU Xsens", 2: "IMU MPU" };
            const imu = parsed.imu_src !== undefined ? ` | ${imuNames[parsed.imu_src] || "IMU ?"}` : "";
            const loop = parsed.loop_hz !== undefined
              ? ` | PID ${Number(parsed.loop_hz).toFixed(0)}Hz (max ${(Number(parsed.loop_max_us || 0) / 1000).toFixed(1)}ms)`
              : "";
            const jsonErr = this.jsonParseErrors > 0 ? ` | JSON lỗi ${this.jsonParseErrors}` : "";
            linkState.textContent = `${network} | ${parsed.state} | ${pwm}${imu}${loop}${jsonErr}`;
          }
          if (rcState && parsed && parsed.rc_seq !== undefined) {
            rcState.textContent = `#${parsed.rc_seq} | ${Number(parsed.sp_t || 0).toFixed(1)}%`;
          }
          // Bỏ qua các bản tin log thuần túy {"log":"..."} để không ghi đè góc 0 vào đồ thị 3D
          if (parsed && typeof parsed.log === "string") {
            console.log("[ESP32]", parsed.log);
          }
          if (parsed && (typeof parsed.roll === "number" || Array.isArray(parsed.m))) {
            this.pushSample(parsed);
          }
        } catch (parseError) {
          // Không nuốt lỗi im lặng: đếm và log thưa để phát hiện JSON hỏng/bị cắt từ ESP32
          this.jsonParseErrors += 1;
          if (this.jsonParseErrors <= 5 || this.jsonParseErrors % 100 === 0) {
            console.warn(`[WebSocket] JSON lỗi #${this.jsonParseErrors}:`, parseError.message,
              String(event.data).slice(0, 120));
          }
        }
      };
      this.webSocketConnection.onclose = () => {
        const bLiveWs = document.getElementById("bLiveWs");
        if (bLiveWs) {
          bLiveWs.style.background = "#0284c7";
          bLiveWs.textContent = "📡 Kết Nối WiFi";
        }
        const linkState = document.getElementById("espLinkState");
        if (linkState) {
          linkState.textContent = "Mất kết nối (Disconnected)";
        }
        this.lastEspState = "DISARMED";
        if (this.connectionMode === "ws") {
          setTimeout(() => this.connectionMode === "ws" && this.connectWebSocket(endpointUrl), 2000);
        }
      };
      this.webSocketConnection.onerror = (err) => {
        console.warn("[WebSocket Lỗi]", err);
      };
    } catch (_connectionError) {}
  }

  /**
   * Kiểm tra xem đường truyền vật lý (WebSocket hoặc WebSerial) có đang mở và sẵn sàng hay không
   */
  isTransportOpen() {
    if (this.connectionMode === "ws") {
      return !!(this.webSocketConnection && this.webSocketConnection.readyState === WebSocket.OPEN);
    }
    if (this.connectionMode === "serial") {
      return !!(this.serialPort && this.serialPort.readable && this.serialPort.writable);
    }
    return false;
  }

  /**
   * Gửi lệnh điều khiển CLI (arm, disarm, set 15) qua Serial hoặc WebSocket
   */
  async sendCommand(commandString) {
    if (this.connectionMode === "ws" && this.webSocketConnection && this.webSocketConnection.readyState === WebSocket.OPEN) {
      this.webSocketConnection.send(commandString);
      return true;
    }
    if (this.connectionMode === "serial" && this.serialPort && this.serialPort.writable) {
      try {
        const writer = this.serialPort.writable.getWriter();
        const encoder = new TextEncoder();
        await writer.write(encoder.encode(commandString + "\n"));
        writer.releaseLock();
        return true;
      } catch (writeErr) {
        console.error("[WebSerial Write Error]", writeErr);
      }
    }
    return false;
  }

  enableDemoMode() {
    this.connectionMode = "demo";
    if (this.webSocketConnection) {
      this.webSocketConnection.close();
      this.webSocketConnection = null;
    }
  }
}

window.TelemetryDataStore = TelemetryDataStore;
