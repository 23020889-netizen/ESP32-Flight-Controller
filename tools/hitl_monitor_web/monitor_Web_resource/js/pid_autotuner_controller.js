/**
 * PID Auto-Tuner Web Controller
 * Đồng bộ hóa dữ liệu Relay / Ziegler-Nichols với firmware ESP32-C6 qua WebSocket/Serial
 */
class PidAutoTunerController {
  constructor() {
    this.selectedAxis = "roll";
    this.relayAmplitudePct = 15;
    this.rateHistory = [];
    this.maxHistoryLength = 180;
    this.lastTuneState = 0;
    this.canvas = document.getElementById("tuneGraph");
    this.ctx = this.canvas ? this.canvas.getContext("2d") : null;

    this.bindEvents();
    this.startGraphLoop();
  }

  getTelemetryStore() {
    const store = window.dashboardController?.telemetryStore || window.dashboardApp?.telemetryStore;
    if (store && !store.onLogMessage) {
      store.onLogMessage = (msg) => this.log(msg);
    }
    return store;
  }

  log(msg) {
    const el = document.getElementById("tuneLog");
    if (el) {
      el.textContent = `[${new Date().toLocaleTimeString()}] ${msg}`;
    }
  }

  bindEvents() {
    const getEl = (id) => document.getElementById(id);

    // 1. CHỌN TRỤC TUNE
    const axisButtons = document.querySelectorAll(".btn-tune-axis");
    axisButtons.forEach((btn) => {
      btn.addEventListener("click", () => {
        axisButtons.forEach((b) => b.classList.remove("active"));
        btn.classList.add("active");
        this.selectedAxis = btn.dataset.axis || "roll";
        this.log(`Đã chọn trục: ${this.selectedAxis.toUpperCase()}`);
      });
    });

    // 2. SLIDER BIÊN ĐỘ RELAY
    const rngAmp = getEl("rngRelayAmp");
    const lblAmp = getEl("lblRelayAmp");
    if (rngAmp && lblAmp) {
      rngAmp.addEventListener("input", () => {
        this.relayAmplitudePct = Number(rngAmp.value);
        lblAmp.textContent = this.relayAmplitudePct + "%";
      });
    }

    // 3. NÚT BẮT ĐẦU TUNE
    const btnStart = getEl("btnTuneStart");
    if (btnStart) {
      btnStart.addEventListener("click", () => {
        const store = this.getTelemetryStore();
        if (!store || !store.isTransportOpen()) {
          this.log("⚠️ Chưa kết nối ESP32 qua WiFi hoặc Serial!");
          return;
        }

        // Kiểm tra ga an toàn
        const thr = Number(store.lastTargetThrottle || 0);
        if (thr < 15) {
          this.log("⚠️ Cảnh báo: Ga hiện tại < 15%. Vui lòng gạt ga lên 25-35% rồi thử lại!");
        }

        const cmd = `tune ${this.selectedAxis} ${this.relayAmplitudePct}`;
        store.sendCommand(cmd);
        this.log(`Đã gửi: "${cmd}" -> Đang chờ ESP32 chạy relay...`);

        // Reset UI tạm thời
        const badge = getEl("tuneStatusBadge");
        if (badge) {
          badge.className = "tune-badge tune-active";
          badge.textContent = "⏳ CHUẨN BỊ";
        }
      });
    }

    // 4. NÚT DỪNG KHẨN CẤP
    const btnStop = getEl("btnTuneStop");
    if (btnStop) {
      btnStop.addEventListener("click", () => {
        const store = this.getTelemetryStore();
        if (store) {
          store.sendCommand("tune stop");
          this.log('Đã gửi lệnh: "tune stop"');
        }
      });
    }

    // 5. NÚT ÁP DỤNG ZN CHUẨN
    const btnApply = getEl("btnTuneApply");
    if (btnApply) {
      btnApply.addEventListener("click", () => {
        const store = this.getTelemetryStore();
        if (store) {
          store.sendCommand("tune apply");
          this.log(`✅ Đã áp dụng bộ số ZN CHUẨN cho trục ${this.selectedAxis.toUpperCase()}!`);
        }
      });
    }

    // 6. NÚT ÁP DỤNG ZN MƯỢT (50% DAMPING)
    const btnApplySmooth = getEl("btnTuneApplySmooth");
    if (btnApplySmooth) {
      btnApplySmooth.addEventListener("click", () => {
        const store = this.getTelemetryStore();
        if (store) {
          store.sendCommand("tune apply 50");
          this.log(`🛡️ Đã áp dụng bộ số ZN MƯỢT (Khuyên dùng) cho trục ${this.selectedAxis.toUpperCase()}!`);
        }
      });
    }

    // 7. NÚT LƯU VÀO FIRMWARE
    const btnSave = getEl("btnTuneSave");
    if (btnSave) {
      btnSave.addEventListener("click", () => {
        const store = this.getTelemetryStore();
        if (store) {
          store.sendCommand("tune save");
          this.log("💾 Đã yêu cầu ESP in mã C++ ra Serial Monitor / Terminal!");
        }
      });
    }
  }

  updateFromTelemetry(latestSample) {
    if (!latestSample) return;
    const getEl = (id) => document.getElementById(id);

    // 1. Góc thời gian thực & Cảnh báo an toàn
    const roll = Number(latestSample.roll || 0);
    const pitch = Number(latestSample.pitch || 0);
    const rollEl = getEl("tuneRollLive");
    const pitchEl = getEl("tunePitchLive");
    if (rollEl) rollEl.textContent = roll.toFixed(1) + "°";
    if (pitchEl) pitchEl.textContent = pitch.toFixed(1) + "°";

    const warnEl = getEl("tuneSafetyWarn");
    if (warnEl) {
      warnEl.style.display = (Math.abs(roll) > 15 || Math.abs(pitch) > 15) ? "inline" : "none";
    }

    // 2. Thu thập điểm cho đồ thị dao động
    let currentRate = 0;
    if (this.selectedAxis === "roll" || this.selectedAxis === "angle") currentRate = latestSample.gx || 0;
    else if (this.selectedAxis === "pitch") currentRate = latestSample.gy || 0;
    else if (this.selectedAxis === "yaw") currentRate = latestSample.gz || 0;

    this.rateHistory.push(currentRate);
    if (this.rateHistory.length > this.maxHistoryLength) {
      this.rateHistory.shift();
    }

    // 3. Xử lý gói tin tune JSON từ ESP32
    const tune = latestSample.tune;
    if (!tune) return;

    // Trạng thái badge
    const badge = getEl("tuneStatusBadge");
    if (badge) {
      if (tune.state === 1 || tune.state === 2) {
        badge.className = "tune-badge tune-active";
        badge.textContent = `⚡ RELAY (${tune.cycle}/${tune.min})`;
      } else if (tune.state === 3) {
        badge.className = "tune-badge tune-done";
        badge.textContent = "✅ HOÀN THÀNH";
      } else if (tune.state === 4) {
        badge.className = "tune-badge tune-aborted";
        const reasonText = tune.err === 1 ? "NGHIÊNG > 25°" : (tune.err === 2 ? "TIMEOUT" : "ĐÃ DỪNG");
        badge.textContent = `❌ ${reasonText}`;
      } else {
        badge.className = "tune-badge tune-idle";
        badge.textContent = "⬛ IDLE";
      }
    }

    // Tiến độ chu kỳ
    const countEl = getEl("tuneCycleCount");
    const minEl = getEl("tuneMinCycles");
    const barEl = getEl("tuneProgressBar");
    if (countEl) countEl.textContent = tune.cycle || 0;
    if (minEl) minEl.textContent = tune.min || 8;
    if (barEl) {
      const pct = Math.min(100, Math.round(((tune.cycle || 0) / (tune.min || 8)) * 100));
      barEl.style.width = pct + "%";
    }

    // Bảng kết quả ZN
    const resBox = getEl("tuneResultBox");
    const btnApply = getEl("btnTuneApply");
    const btnApplySmooth = getEl("btnTuneApplySmooth");
    const btnSave = getEl("btnTuneSave");

    if (tune.valid && tune.state === 3) {
      if (resBox) resBox.style.display = "block";
      if (btnApply) btnApply.disabled = false;
      if (btnApplySmooth) btnApplySmooth.disabled = false;
      if (btnSave) btnSave.disabled = false;

      const setVal = (id, val) => {
        const el = getEl(id);
        if (el) el.textContent = val;
      };

      setVal("tuneKu", (tune.ku || 0).toFixed(4));
      setVal("tuneTu", (tune.tu || 0).toFixed(3) + " s");
      setVal("tuneAngleP", (tune.ang_p || 0).toFixed(2));

      setVal("tuneKp", (tune.kp || 0).toFixed(4));
      setVal("tuneKi", (tune.ki || 0).toFixed(4));
      setVal("tuneKd", (tune.kd || 0).toFixed(5));

      setVal("tuneKpSmooth", (tune.kp_s || 0).toFixed(4));
      setVal("tuneKiSmooth", (tune.ki_s || 0).toFixed(4));
      setVal("tuneKdSmooth", (tune.kd_s || 0).toFixed(5));

      if (this.lastTuneState !== 3) {
        this.log(`🎉 Tìm thấy bộ số! Tu=${tune.tu.toFixed(3)}s | Ku=${tune.ku.toFixed(3)} -> Bấm "Áp Dụng" để kích hoạt!`);
      }
    } else if (tune.state === 4 && this.lastTuneState !== 4) {
      this.log(`⚠️ Auto-tune bị dừng. Lý do: ${tune.err === 1 ? 'Nghiêng quá 25°' : (tune.err === 2 ? 'Không có dao động' : 'Người dùng dừng')}`);
    }

    this.lastTuneState = tune.state;
  }

  startGraphLoop() {
    const draw = () => {
      requestAnimationFrame(draw);
      this.renderGraph();
    };
    requestAnimationFrame(draw);
  }

  renderGraph() {
    if (!this.ctx || !this.canvas) return;
    const w = this.canvas.width;
    const h = this.canvas.height;
    const ctx = this.ctx;

    ctx.fillStyle = "#0a0e1a";
    ctx.fillRect(0, 0, w, h);

    // Đường lưới giữa (0 deg/s)
    const midY = h / 2;
    ctx.strokeStyle = "rgba(255, 255, 255, 0.12)";
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, midY);
    ctx.lineTo(w, midY);
    ctx.stroke();

    if (this.rateHistory.length < 2) return;

    // Tự động scale biên độ tối đa
    let maxVal = 20;
    for (let i = 0; i < this.rateHistory.length; i++) {
      const v = Math.abs(this.rateHistory[i]);
      if (v > maxVal) maxVal = v;
    }

    // Vẽ sóng dao động Gyro Rate
    ctx.strokeStyle = "#38bdf8";
    ctx.lineWidth = 1.8;
    ctx.shadowColor = "rgba(56, 189, 248, 0.6)";
    ctx.shadowBlur = 6;
    ctx.beginPath();

    const step = w / (this.maxHistoryLength - 1);
    const startIdx = this.maxHistoryLength - this.rateHistory.length;

    for (let i = 0; i < this.rateHistory.length; i++) {
      const x = (startIdx + i) * step;
      const y = midY - (this.rateHistory[i] / maxVal) * (midY - 6);
      if (i === 0) ctx.moveTo(x, y);
      else ctx.lineTo(x, y);
    }
    ctx.stroke();
    ctx.shadowBlur = 0;

    // Label biên độ góc trên
    ctx.fillStyle = "rgba(255, 255, 255, 0.4)";
    ctx.font = "9px monospace";
    ctx.fillText(`±${maxVal.toFixed(0)}°/s`, 4, 10);
  }
}

// Khởi tạo và gắn vào application lifecycle
document.addEventListener("DOMContentLoaded", () => {
  window.pidAutoTuner = new PidAutoTunerController();
});
