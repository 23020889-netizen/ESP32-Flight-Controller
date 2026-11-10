/**
 * High-performance telemetry rendering loop.
 * Separates 60 FPS 3D Attitude rendering from 20 FPS 2D Chart updates for zero lag.
 */
class TelemetryLoopUpdater {
  static startLoop(controller) {
    const tick = () => {
      requestAnimationFrame(tick);
      TelemetryLoopUpdater.updateFrame(controller);
    };
    requestAnimationFrame(tick);
  }

  static updateFrame(controller) {
    const nowMilliseconds = performance.now();
    const deltaSeconds = (nowMilliseconds - controller.lastFrameTimeMilliseconds) / 1000;
    controller.lastFrameTimeMilliseconds = nowMilliseconds;

    const buffer = controller.telemetryStore.sampleBuffer;
    const latestSample = buffer[buffer.length - 1] || {
      roll: 0,
      pitch: 0,
      yaw: 0,
      z: 0,
      vz: 0,
      m: [0, 0, 0, 0]
    };

    // 1. Cập nhật xoay mô hình 3D luôn chạy ở tốc độ 60 FPS mượt mà
    controller.sceneManager.renderSceneFrame(latestSample, deltaSeconds);

    // 2. Chỉ cập nhật DOM chỉ số tần số 30 FPS
    controller.frameCounter++;
    if (controller.frameCounter % 2 !== 0) {
      return;
    }

    const getElement = (id) => document.getElementById(id);
    const sFreq = getElement("sFreq");
    if (sFreq) sFreq.textContent = controller.telemetryStore.packetFrequencyHertz + " Hz";
    const sPk = getElement("sPk");
    if (sPk) sPk.textContent = String(controller.telemetryStore.totalPacketCount);

    const eR = getElement("eR");
    if (eR) eR.textContent = latestSample.roll.toFixed(1) + "°";
    const eP = getElement("eP");
    if (eP) eP.textContent = latestSample.pitch.toFixed(1) + "°";
    const eY = getElement("eY");
    if (eY) eY.textContent = latestSample.yaw.toFixed(1) + "°";

    const vz = getElement("vz");
    if (vz) vz.textContent = latestSample.z.toFixed(2);
    const vvz = getElement("vvz");
    if (vvz) vvz.textContent = latestSample.vz.toFixed(2);

    const hz = getElement("hz");
    if (hz) ArtificialHorizonRenderer.renderHorizon(hz, latestSample.roll, latestSample.pitch);

    latestSample.m.forEach((speed, index) => {
      const pct = Math.max(0, Math.min(100, speed / 10));
      const mb = getElement("mb" + index);
      if (mb) mb.style.width = pct + "%";
      const mv = getElement("mv" + index);
      if (mv) mv.textContent = String(Math.round(speed));

      const rng = getElement("rngM" + (index + 1));
      const lbl = getElement("lblM" + (index + 1));
      if (rng && document.activeElement !== rng) {
        rng.value = Math.round(pct);
      }
      if (lbl) {
        lbl.textContent = pct.toFixed(1) + "%";
      }
    });

    if (controller.isPaused) {
      return;
    }

    // 3. Chỉ vẽ đồ thị 4 kênh ở tần số 15-20 FPS để giải phóng 80% tải CPU Canvas
    if (controller.frameCounter % 4 !== 0) {
      return;
    }

    const span = controller.telemetryStore.timeSpanSeconds;
    const c1 = getElement("c1");
    if (c1) TelemetryChartRenderer.renderTimeSeriesChart(c1, buffer, span, [{ colorVariable: "--c1", extractValue: (x) => x.roll }], -30, 30);
    const c2 = getElement("c2");
    if (c2) TelemetryChartRenderer.renderTimeSeriesChart(c2, buffer, span, [{ colorVariable: "--c2", extractValue: (x) => x.pitch }], -30, 30);
    const c3 = getElement("c3");
    if (c3) TelemetryChartRenderer.renderTimeSeriesChart(c3, buffer, span, [{ colorVariable: "--c3", extractValue: (x) => x.yaw }], -180, 180);
    const c4 = getElement("c4");
    if (c4) {
      TelemetryChartRenderer.renderTimeSeriesChart(
        c4,
        buffer,
        span,
        [
          { colorVariable: "--c1", extractValue: (x) => x.m[0] || 0 },
          { colorVariable: "--c2", extractValue: (x) => x.m[1] || 0 },
          { colorVariable: "--c3", extractValue: (x) => x.m[2] || 0 },
          { colorVariable: "--c4", extractValue: (x) => x.m[3] || 0 }
        ],
        0,
        1000
      );
    }
  }
}

window.TelemetryLoopUpdater = TelemetryLoopUpdater;

// Khởi tạo toàn bộ ứng dụng Dashboard ngay khi tài liệu sẵn sàng
function initDashboard() {
  if (!window.dashboardController) {
    try {
      window.dashboardController = new DashboardApplicationController();
      TelemetryLoopUpdater.startLoop(window.dashboardController);
      console.log("[INIT] Dashboard application initialized successfully.");
    } catch (err) {
      console.error("[INIT ERROR]", err);
    }
  }
}

if (document.readyState === "loading") {
  window.addEventListener("DOMContentLoaded", initDashboard);
} else {
  initDashboard();
}

