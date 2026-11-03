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

    // Cache giá trị trạng thái tức thời
    this.currentRoll = 0;
    this.currentPitch = 0;
    this.currentYaw = 0;
    this.currentZ = 0;
    this.currentVz = 0;
    this.currentMotors = [0, 0, 0, 0];

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
    this.sampleBuffer.push({
      t: timestampSeconds,
      roll: Number(rawSample.roll) || 0,
      pitch: Number(rawSample.pitch) || 0,
      yaw: Number(rawSample.yaw) || 0,
      z: Number(rawSample.z) || 0,
      vz: Number(rawSample.vz) || 0,
      m: (rawSample.m || [0, 0, 0, 0]).map(Number)
    });

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

    // 1. Phân tích gói EKF tối ưu:
    // [PX4_EKF] R:   1.2° P:  -0.5° Y:  82.4° | Acc:[  0.20, -0.16,  9.13] | Gyr:[  -0.1,   0.1,   0.0] (OPTIMAL)
    if (line.includes("[PX4_EKF]")) {
      const mR = line.match(/R:\s*([-\d.]+)/);
      const mP = line.match(/P:\s*([-\d.]+)/);
      const mY = line.match(/Y:\s*([-\d.]+)/);
      if (mR && mP && mY) {
        this.currentRoll = parseFloat(mR[1]);
        this.currentPitch = parseFloat(mP[1]);
        this.currentYaw = parseFloat(mY[1]);

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
    // 2. Phân tích trạng thái động cơ:
    // [ESC_QUAD] State:DISARMED | M1(P7-CCW): 900us( 0.0%) | M2(P6-CCW): 900us( 0.0%) | ...
    else if (line.includes("[ESC_QUAD]")) {
      const pcts = [...line.matchAll(/\(\s*([-\d.]+)%\)/g)];
      if (pcts.length >= 4) {
        this.currentMotors = [
          parseFloat(pcts[0][1]) * 10,
          parseFloat(pcts[1][1]) * 10,
          parseFloat(pcts[2][1]) * 10,
          parseFloat(pcts[3][1]) * 10
        ];
      }
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
      if (this.webSocketConnection) this.webSocketConnection.close();
      this.webSocketConnection = new WebSocket(endpointUrl);
      this.webSocketConnection.onmessage = (event) => {
        try {
          this.pushSample(JSON.parse(event.data));
        } catch (_ignoredError) {}
      };
      this.webSocketConnection.onclose = () => {
        if (this.connectionMode === "ws") {
          setTimeout(() => this.connectionMode === "ws" && this.connectWebSocket(endpointUrl), 2000);
        }
      };
    } catch (_connectionError) {}
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
