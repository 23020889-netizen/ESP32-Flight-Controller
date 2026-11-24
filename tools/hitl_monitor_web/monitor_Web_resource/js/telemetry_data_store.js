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
    this.lastPidEnabled = false;

    // Cache giá trị trạng thái tức thời
    this.currentRoll = 0;
    this.currentPitch = 0;
    this.currentYaw = 0;
    this.currentZ = 0;
    this.currentVz = 0;
    this.currentMotors = [0, 0, 0, 0];

    // Bộ ghi nhật ký Telemetry (CSV Recorder)
    this.isRecordingCsv = false;
    this.csvRecordingStartTime = 0;
    this.recordedCsvRows = [];

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
    const sample = {
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
      gx: rawSample.gx !== undefined ? Number(rawSample.gx) : 0,
      gy: rawSample.gy !== undefined ? Number(rawSample.gy) : 0,
      gz: rawSample.gz !== undefined ? Number(rawSample.gz) : 0,
      pid_en: rawSample.pid_en !== undefined ? Number(rawSample.pid_en) : (this.lastPidEnabled ? 1 : 0),
      sp_t: rawSample.sp_t !== undefined ? Number(rawSample.sp_t) : (this.lastTargetThrottle || 0),
      tune: rawSample.tune !== undefined ? rawSample.tune : (this.lastTuneStatus || null),
      m: (rawSample.m || [0, 0, 0, 0]).map(Number)
    };
    this.sampleBuffer.push(sample);

    // Ghi nhận dữ liệu vào bộ đệm CSV nếu đang bật chế độ ghi
    if (this.isRecordingCsv) {
      const elapsed = Math.max(0, timestampSeconds - this.csvRecordingStartTime).toFixed(3);
      const iso = new Date().toISOString();
      const m = sample.m;
      this.recordedCsvRows.push({
        iso: iso,
        elapsed: elapsed,
        roll: sample.roll.toFixed(2),
        pitch: sample.pitch.toFixed(2),
        yaw: sample.yaw.toFixed(2),
        mpu_r: sample.mpu_r.toFixed(2),
        mpu_p: sample.mpu_p.toFixed(2),
        mpu_y: sample.mpu_y.toFixed(2),
        xs_r: sample.xs_r.toFixed(2),
        xs_p: sample.xs_p.toFixed(2),
        xs_y: sample.xs_y.toFixed(2),
        m1: (m[0] || 0).toFixed(1),
        m2: (m[1] || 0).toFixed(1),
        m3: (m[2] || 0).toFixed(1),
        m4: (m[3] || 0).toFixed(1),
        z: sample.z.toFixed(2),
        vz: sample.vz.toFixed(2),
        accZ: (9.81 + sample.vz * 0.1).toFixed(3),
        state: this.lastEspState || "DISARMED"
      });
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
    // 3. Phân tích gói EKF dung hợp tối ưu:
    // [PX4_EKF] R:   1.2° P:  -0.5° Y:  82.4° | Acc:[  0.20, -0.16,  9.13] | Gyr:[  -0.1,   0.1,   0.0] (OPTIMAL)
    else if (line.includes("[PX4_EKF]")) {
      const mR = line.match(/R:\s*([-\d.]+)/);
      const mP = line.match(/P:\s*([-\d.]+)/);
      const mY = line.match(/Y:\s*([-\d.]+)/);
      if (mR && mP && mY) {
        this.currentRoll = parseFloat(mR[1]);
        this.currentPitch = parseFloat(mP[1]);
        this.currentYaw = parseFloat(mY[1]);
      }
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
            if (parsed.pid_en !== undefined) this.lastPidEnabled = (parsed.pid_en === 1);
            if (parsed.pid_enabled !== undefined) this.lastPidEnabled = !!parsed.pid_enabled;
            if (parsed.tune !== undefined) this.lastTuneStatus = parsed.tune;
            if (parsed.log && typeof this.onLogMessage === "function") {
              this.onLogMessage(parsed.log);
            }
          }
          if (linkState && parsed && parsed.state !== undefined) {
            const network = parsed.sta_connected ? `VNU ${parsed.sta_ip || ""}` : "SoftAP/STA offline";
            const pwm = parsed.pwm_ok === 0 ? "PWM lỗi" : "PWM OK";
            linkState.textContent = `${network} | ${parsed.state} | ${pwm}`;
          }
          if (rcState && parsed && parsed.rc_seq !== undefined) {
            rcState.textContent = `#${parsed.rc_seq} | ${Number(parsed.sp_t || 0).toFixed(1)}%`;
          }
          // Bỏ qua các bản tin log thuần túy {"log":"..."} để không ghi đè góc 0 vào đồ thị 3D
          if (parsed && (typeof parsed.roll === "number" || Array.isArray(parsed.m))) {
            this.pushSample(parsed);
          }
        } catch (_ignoredError) {}
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

  /**
   * BẮT ĐẦU GHI NHẬT KÝ TELEMETRY RA BỘ ĐỆM CSV
   */
  startCsvRecording() {
    this.isRecordingCsv = true;
    if (this.recordedCsvRows.length === 0) {
      this.csvRecordingStartTime = performance.now() / 1000;
    }
  }

  /**
   * TẠM DỪNG HOẶC DỪNG GHI NHẬT KÝ CSV
   */
  stopCsvRecording() {
    this.isRecordingCsv = false;
  }

  /**
   * XÓA TOÀN BỘ BỘ ĐỆM NHẬT KÝ CSV ĐÃ GHI
   */
  clearCsvRecords() {
    this.recordedCsvRows = [];
    this.csvRecordingStartTime = performance.now() / 1000;
  }

  /**
   * LẤY SỐ LƯỢNG MẪU ĐÃ GHI
   */
  getCsvRecordCount() {
    return this.recordedCsvRows.length;
  }

  /**
   * LẤY THỜI LƯỢNG GHI (TÍNH BẰNG GIÂY)
   */
  getCsvRecordingDurationSeconds() {
    if (!this.isRecordingCsv && this.recordedCsvRows.length === 0) return 0;
    if (this.isRecordingCsv) {
      return Math.max(0, (performance.now() / 1000) - this.csvRecordingStartTime);
    }
    if (this.recordedCsvRows.length > 0) {
      return parseFloat(this.recordedCsvRows[this.recordedCsvRows.length - 1].elapsed) || 0;
    }
    return 0;
  }

  /**
   * XUẤT NỘI DUNG CHUỖI CSV TỪ BỘ ĐỆM
   */
  exportCsvString() {
    const headers = [
      "Timestamp_ISO",
      "Time_Elapsed_s",
      "Roll_deg",
      "Pitch_deg",
      "Yaw_deg",
      "MPU_Roll_deg",
      "MPU_Pitch_deg",
      "MPU_Yaw_deg",
      "Xsens_Roll_deg",
      "Xsens_Pitch_deg",
      "Xsens_Yaw_deg",
      "M1_rads",
      "M2_rads",
      "M3_rads",
      "M4_rads",
      "Altitude_m",
      "Velocity_Vz",
      "Acc_Z_ms2",
      "State"
    ];
    const lines = [headers.join(",")];
    for (let i = 0; i < this.recordedCsvRows.length; i++) {
      const r = this.recordedCsvRows[i];
      lines.push(`${r.iso},${r.elapsed},${r.roll},${r.pitch},${r.yaw},${r.mpu_r},${r.mpu_p},${r.mpu_y},${r.xs_r},${r.xs_p},${r.xs_y},${r.m1},${r.m2},${r.m3},${r.m4},${r.z},${r.vz},${r.accZ},${r.state}`);
    }
    return lines.join("\r\n");
  }

  /**
   * TẢI FILE CSV XUỐNG MÁY TÍNH CỦA NGƯỜI DÙNG
   */
  downloadCsvFile(filename) {
    if (this.recordedCsvRows.length === 0) {
      alert("Không có dữ liệu log để tải về.");
      return false;
    }
    const csvContent = this.exportCsvString();
    const blob = new Blob([csvContent], { type: "text/csv;charset=utf-8;" });
    const url = URL.createObjectURL(blob);
    const downloadAnchor = document.createElement("a");
    const now = new Date();
    const timestampStr = now.getFullYear().toString() +
      String(now.getMonth() + 1).padStart(2, "0") +
      String(now.getDate()).padStart(2, "0") + "_" +
      String(now.getHours()).padStart(2, "0") +
      String(now.getMinutes()).padStart(2, "0") +
      String(now.getSeconds()).padStart(2, "0");
    downloadAnchor.href = url;
    downloadAnchor.download = filename || `UAV_Telemetry_Log_${timestampStr}.csv`;
    document.body.appendChild(downloadAnchor);
    downloadAnchor.click();
    document.body.removeChild(downloadAnchor);
    URL.revokeObjectURL(url);
    return true;
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
