/**
 * Coordinates UI controls, FlexLayout workspace, telemetry store, WebSerial, WebSocket, and 3D views.
 */
class DashboardApplicationController {
  constructor() {
    this.telemetryStore = new TelemetryDataStore();
    this.csvReplayer = new CsvTelemetryReplayer(this.telemetryStore);
    this.sceneManager = new ThreeDimensionalSceneManager(document.getElementById("gl"));
    this.flexLayoutManager = new FlexLayoutWorkspaceManager();
    this.isPaused = false;
    this.colorVariables = ["--c1", "--c2", "--c3", "--c4"];
    this.lastFrameTimeMilliseconds = performance.now();
    this.frameCounter = 0;
    this.initializeMotorBars();
    this.bindUserInterfaceControls();
  }

  initializeMotorBars() {
    const motorLabels = ["M1 FR (P7-CCW)", "M2 RL (P6-CCW)", "M3 FL (P5-CW)", "M4 RR (P4-CW)"];
    const motEl = document.getElementById("mot");
    if (!motEl) return;
    motEl.innerHTML = motorLabels
      .map(
        (label, index) =>
          `<div class="mr"><span>${label}</span><div class="tr"><i id="mb${index}" style="--k:var(${this.colorVariables[index]})"></i></div><b id="mv${index}">0</b></div>`
      )
      .join("");
  }

  bindUserInterfaceControls() {
    const getElement = (id) => document.getElementById(id);
    const setActiveMode = (activeId) => {
      ["bDemo", "bCsv", "bSerial", "bLiveWs", "bApWs"].forEach((id) => {
        const el = getElement(id);
        if (el) el.classList.toggle("act", id === activeId);
      });
    };

    // 1. KẾT NỐI QUA CÁP USB SERIAL (WebSerial - Trực tiếp COM32)
    const bSerial = getElement("bSerial");
    if (bSerial) {
      bSerial.onclick = async () => {
        this.csvReplayer.stopPlayback();
        const success = await this.telemetryStore.connectWebSerial();
        if (success) {
          setActiveMode("bSerial");
        }
      };
    }

    // 2. Wi-Fi VNU dùng IP cố định; SoftAP có nút riêng, không dùng IP cũ trong localStorage.
    const bLiveWs = getElement("bLiveWs");
    const bApWs = getElement("bApWs");
    if (bLiveWs) {
      bLiveWs.onclick = () => {
        this.csvReplayer.stopPlayback();
        this.telemetryStore.connectWebSocket("ws://10.11.162.210:81");
        setActiveMode("bLiveWs");
      };
    }
    if (bApWs) {
      bApWs.onclick = () => {
        this.csvReplayer.stopPlayback();
        this.telemetryStore.connectWebSocket("ws://192.168.4.1:81");
        setActiveMode("bApWs");
      };
    }

    // 3. CHẾ ĐỘ GIẢ LẬP DEMO
    const bDemo = getElement("bDemo");
    if (bDemo) {
      bDemo.onclick = () => {
        setActiveMode("bDemo");
        this.csvReplayer.stopPlayback();
        this.telemetryStore.enableDemoMode();
      };
    }

    // 4. CHẾ ĐỘ PHÁT LẠI FILE CSV
    const bCsv = getElement("bCsv");
    if (bCsv) {
      bCsv.onclick = () => {
        setActiveMode("bCsv");
        this.telemetryStore.sampleBuffer.length = 0;
        this.csvReplayer.startPlayback();
      };
    }

    const bPickCsv = getElement("bPickCsv");
    if (bPickCsv) bPickCsv.onclick = () => getElement("fCsv").click();
    const fCsv = getElement("fCsv");
    if (fCsv) fCsv.onchange = (event) => this.handleCustomCsvSelection(event, setActiveMode);

    // Điều khiển ARM / DISARM
    const btnArm = getElement("btnArm");
    if (btnArm) {
      btnArm.onclick = () => {
        if (confirm("XÁC NHẬN: Mở khóa an toàn ARM 4 động cơ Quadcopter?")) {
          this.telemetryStore.sendCommand("arm");
        }
      };
    }

    const btnDisarm = getElement("btnDisarm");
    if (btnDisarm) {
      btnDisarm.onclick = () => {
        this.telemetryStore.sendCommand("disarm");
        // Firmware trở về chế độ ổn định (PID BẬT) sau DISARM; nút đồng bộ lại từ telemetry pid_en
        this.telemetryStore.lastPidEnabled = true;
        this.updatePidButtonUi(true);
        const rng = getElement("rngThrottle");
        const lbl = getElement("lblThrottle");
        if (rng) rng.value = 0;
        if (lbl) lbl.textContent = "0.0%";
      };
    }

    // Điều khiển Ga Throttle Slider & Buttons
    const rngThrottle = getElement("rngThrottle");
    const lblThrottle = getElement("lblThrottle");
    const setThrottle = (pct) => {
      pct = Math.max(0, Math.min(100, Number(pct) || 0));
      if (rngThrottle) rngThrottle.value = pct;
      if (lblThrottle) lblThrottle.textContent = pct.toFixed(1) + "%";
      return pct;
    };

    if (rngThrottle) {
      rngThrottle.oninput = (e) => {
        const val = Number(e.target.value);
        if (lblThrottle) lblThrottle.textContent = val.toFixed(1) + "%";
      };
      rngThrottle.onchange = (e) => this.telemetryStore.sendCommand(`throttle ${setThrottle(Number(e.target.value))}`);
    }

    document.querySelectorAll(".btn-thr").forEach((b) => {
      b.onclick = () => this.telemetryStore.sendCommand(`throttle ${setThrottle(Number(b.dataset.thr))}`);
    });

    // Điều khiển Góc Setpoint (PX4 Angle Control)
    const btnLevel = getElement("btnLevel");
    if (btnLevel) {
      btnLevel.onclick = () => {
        this.telemetryStore.sendCommand("level");
        this.telemetryStore.lastPidEnabled = true;
        this.updatePidButtonUi(true);
      };
    }

    const btnInvPitch = getElement("btnInvPitch");
    if (btnInvPitch) {
      btnInvPitch.onclick = () => {
        this.telemetryStore.sendCommand("invert pitch");
      };
    }

    const btnPidView = getElement("btnPidView");
    if (btnPidView) {
      btnPidView.onclick = () => {
        const nextState = !this.telemetryStore.lastPidEnabled;
        this.telemetryStore.sendCommand(nextState ? "pid on" : "pid off");
        this.telemetryStore.lastPidEnabled = nextState;
        this.updatePidButtonUi(nextState);
      };
    }

    document.querySelectorAll(".btn-sp").forEach((b) => {
      b.onclick = () => {
        this.telemetryStore.sendCommand(`sp ${b.dataset.sp}`);
      };
    });

    // --- ĐIỀU KHIỂN TÙY BIẾN TEST TỪNG ĐỘNG CƠ (INDIVIDUAL MOTOR TEST) ---
    const setMotorTest = (motorId, val) => {
      val = Math.max(0, Math.min(100, Number(val) || 0));
      const rng = getElement(`rngM${motorId}`);
      const lbl = getElement(`lblM${motorId}`);
      if (rng) rng.value = val;
      if (lbl) lbl.textContent = val + "%";

      // KHI ĐẶT GA CHO 1 ĐỘNG CƠ (> 0), TỰ ĐỘNG ĐƯA TẤT CẢ SLIDER CÒN LẠI VỀ 0%
      if (val > 0) {
        [1, 2, 3, 4].forEach((mId) => {
          if (mId !== motorId) {
            const r = getElement(`rngM${mId}`);
            const l = getElement(`lblM${mId}`);
            if (r) r.value = 0;
            if (l) l.textContent = "0%";
          }
        });
      }

      this.telemetryStore.sendCommand(`m${motorId} ${val}`);
    };

    [1, 2, 3, 4].forEach((mId) => {
      const rng = getElement(`rngM${mId}`);
      const lbl = getElement(`lblM${mId}`);
      if (rng) {
        rng.oninput = (e) => {
          const v = e.target.value;
          if (lbl) lbl.textContent = v + "%";
          if (Number(v) > 0) {
            [1, 2, 3, 4].forEach((otherId) => {
              if (otherId !== mId) {
                const r = getElement(`rngM${otherId}`);
                const l = getElement(`lblM${otherId}`);
                if (r) r.value = 0;
                if (l) l.textContent = "0%";
              }
            });
          }
        };
        rng.onchange = (e) => {
          setMotorTest(mId, e.target.value);
        };
      }
    });

    document.querySelectorAll(".btn-mtest").forEach((b) => {
      b.onclick = () => {
        const targetMotor = Number(b.dataset.motor);
        const targetVal = Number(b.dataset.val);
        setMotorTest(targetMotor, targetVal);
      };
    });

    const btnStopMotors = getElement("btnStopMotors");
    if (btnStopMotors) {
      btnStopMotors.onclick = () => {
        [1, 2, 3, 4].forEach((mId) => {
          const rng = getElement(`rngM${mId}`);
          const lbl = getElement(`lblM${mId}`);
          if (rng) rng.value = 0;
          if (lbl) lbl.textContent = "0%";
        });
        this.telemetryStore.sendCommand("m stop");
      };
    }

    const btnReturnPid = getElement("btnReturnPid");
    if (btnReturnPid) {
      btnReturnPid.onclick = () => {
        [1, 2, 3, 4].forEach((mId) => {
          const rng = getElement(`rngM${mId}`);
          const lbl = getElement(`lblM${mId}`);
          if (rng) rng.value = 0;
          if (lbl) lbl.textContent = "0%";
        });
        this.telemetryStore.sendCommand("manual off");
        this.telemetryStore.sendCommand("pid on");
        this.telemetryStore.lastPidEnabled = true;
        this.updatePidButtonUi(true);
      };
    }

    // Điều khiển Gửi Lệnh CLI
    const btnSendCmd = getElement("btnSendCmd");
    const cmdInput = getElement("cmdInput");
    if (btnSendCmd && cmdInput) {
      const sendHandler = () => {
        const val = cmdInput.value.trim();
        if (val.length > 0) {
          this.telemetryStore.sendCommand(val);
          cmdInput.value = "";
        }
      };
      btnSendCmd.onclick = sendHandler;
      cmdInput.onkeydown = (e) => {
        if (e.key === "Enter") sendHandler();
      };
    }

    // Timebase selectors
    document.querySelectorAll("[data-s]").forEach((button) => {
      button.onclick = () => {
        this.telemetryStore.timeSpanSeconds = Number(button.dataset.s);
        document.querySelectorAll("[data-s]").forEach((item) => item.classList.toggle("act", item === button));
      };
    });

    const bPause = getElement("bPause");
    if (bPause) {
      bPause.onclick = (event) => {
        this.isPaused = !this.isPaused;
        event.target.classList.toggle("act", this.isPaused);
        event.target.textContent = this.isPaused ? "Tiếp tục" : "Tạm dừng";
      };
    }

    const bClear = getElement("bClear");
    if (bClear) {
      bClear.onclick = () => {
        this.telemetryStore.sampleBuffer.length = 0;
      };
    }

    // 5. ĐIỀU KHIỂN GHI NHẬT KÝ TELEMETRY (CSV RECORDER)
    const btnRecToggle = getElement("btnRecToggle");
    const btnRecExport = getElement("btnRecExport");
    const btnRecClear = getElement("btnRecClear");
    const recStatus = getElement("recStatus");
    const recSamples = getElement("recSamples");
    const recDuration = getElement("recDuration");

    if (btnRecToggle) {
      btnRecToggle.onclick = () => {
        if (!this.telemetryStore.isRecordingCsv) {
          this.telemetryStore.startCsvRecording();
          btnRecToggle.innerHTML = "⏹️ Dừng Ghi";
          btnRecToggle.style.background = "rgba(239, 68, 68, 0.28)";
          btnRecToggle.style.borderColor = "#ef4444";
          btnRecToggle.style.color = "#ffffff";
          if (recStatus) recStatus.innerHTML = '<span style="color:#ef4444">● Đang ghi</span>';
          if (btnRecExport) btnRecExport.disabled = false;
        } else {
          this.telemetryStore.stopCsvRecording();
          btnRecToggle.innerHTML = "🔴 Tiếp Tục Ghi";
          btnRecToggle.style.background = "rgba(239, 68, 68, 0.15)";
          btnRecToggle.style.borderColor = "rgba(239, 68, 68, 0.4)";
          btnRecToggle.style.color = "#ef4444";
          if (recStatus) recStatus.innerHTML = '<span style="color:#10b981">Đã dừng (Có dữ liệu)</span>';
          if (btnRecExport) btnRecExport.disabled = false;
        }
      };
    }

    if (btnRecExport) {
      btnRecExport.onclick = () => {
        this.telemetryStore.downloadCsvFile();
      };
    }

    if (btnRecClear) {
      btnRecClear.onclick = () => {
        if (this.telemetryStore.recordedCsvRows.length > 0) {
          if (confirm("Bạn có chắc chắn muốn xóa bộ đệm nhật ký CSV hiện tại?")) {
            this.telemetryStore.clearCsvRecords();
            if (!this.telemetryStore.isRecordingCsv) {
              if (btnRecToggle) {
                btnRecToggle.innerHTML = "🔴 Bắt Đầu Ghi";
                btnRecToggle.style.background = "rgba(239, 68, 68, 0.15)";
                btnRecToggle.style.borderColor = "rgba(239, 68, 68, 0.4)";
                btnRecToggle.style.color = "#ef4444";
              }
              if (recStatus) recStatus.innerHTML = '<span style="color:var(--mu)">Sẵn sàng</span>';
              if (btnRecExport) btnRecExport.disabled = true;
            }
            if (recSamples) recSamples.textContent = "0";
            if (recDuration) recDuration.textContent = "00:00";
          }
        }
      };
    }

    const kAx = getElement("kAx");
    if (kAx) kAx.onchange = (event) => { this.sceneManager.axesGroup.visible = event.target.checked; };
    const kBox = getElement("kBox");
    if (kBox) kBox.onchange = (event) => { this.sceneManager.boundingCage.visible = event.target.checked; };
    const kGrid = getElement("kGrid");
    if (kGrid) kGrid.onchange = (event) => { this.sceneManager.floorGridGroup.visible = event.target.checked; };

    const bCam = getElement("bCam");
    if (bCam) {
      bCam.onclick = () => {
        this.sceneManager.azimuthRadians = 0.75;
        this.sceneManager.elevationRadians = 0.42;
        this.sceneManager.cameraDistance = 6.0;
      };
    }

    // Chuyển đổi Mẫu 3D Quadcopter (mặc định Sci-Fi Bubble Quad-X)
    const selDroneModel = getElement("selDroneModel");
    if (selDroneModel) {
      try {
        let savedModel = localStorage.getItem("uav_3d_model");
        // Nâng cấp tự động nếu trình duyệt lưu cache mẫu con ruồi cũ (glb_cesium)
        if (!savedModel || savedModel === "glb_cesium" || savedModel === "glb_aero") {
          savedModel = "sci_fi_bubble";
          localStorage.setItem("uav_3d_model", "sci_fi_bubble");
        }
        selDroneModel.value = savedModel;
        this.sceneManager.setDroneModel(savedModel);
      } catch (_) { }
      selDroneModel.onchange = () => {
        const chosen = selDroneModel.value || "sci_fi_bubble";
        this.sceneManager.setDroneModel(chosen);
        try {
          localStorage.setItem("uav_3d_model", chosen);
        } catch (_) { }
      };
    }

    // Chuyển đổi Giao diện Màu sắc (Apple visionOS Titanium Dark / Light Ngoài Trời / Tactical Cyan)
    const btnThemeToggle = getElement("btnThemeToggle");
    const themeOrder = [
      { id: "titanium", label: "🍎 Titanium" },
      { id: "light", label: "☀️ Sáng" },
      { id: "tactical", label: "🌌 Tactical" }
    ];
    const applyTheme = (themeId) => {
      const found = themeOrder.find((t) => t.id === themeId) || themeOrder[0];
      document.documentElement.setAttribute("data-theme", found.id);
      if (btnThemeToggle) btnThemeToggle.textContent = found.label;
      try {
        localStorage.setItem("uav_ui_theme", found.id);
      } catch (_) { }
    };
    try {
      const savedTheme = localStorage.getItem("uav_ui_theme");
      if (savedTheme) applyTheme(savedTheme);
      else applyTheme("titanium");
    } catch (_) {
      applyTheme("titanium");
    }
    if (btnThemeToggle) {
      btnThemeToggle.onclick = () => {
        const current = document.documentElement.getAttribute("data-theme") || "titanium";
        const idx = themeOrder.findIndex((t) => t.id === current);
        const next = themeOrder[(idx + 1) % themeOrder.length];
        applyTheme(next.id);
      };
    }

    // Chuyển đổi Tab Hiển thị nhanh trên Mobile / iPhone (Segmented View Control)
    const appShell = getElement("appShell");
    const mobileTabs = document.querySelectorAll(".mobile-view-tabs button[data-view]");
    if (appShell && mobileTabs.length > 0) {
      mobileTabs.forEach((btn) => {
        btn.onclick = () => {
          mobileTabs.forEach((item) => item.classList.toggle("act", item === btn));
          appShell.setAttribute("data-mobile-tab", btn.dataset.view || "all");
          window.dispatchEvent(new Event("resize"));
        };
      });
    }

    // 5. HỖ TRỢ TAY CẦM XBOX CẮM VÀO MÁY TÍNH (HTML5 GAMEPAD API)
    this.startXboxGamepadLoop(setThrottle, setMotorTest);
  }

  static deadzone(v, th = 0.08) {
    if (Math.abs(v) < th) return 0;
    return v > 0 ? (v - th) / (1 - th) : (v + th) / (1 - th);
  }

  /**
   * Pure function extracting normalized inputs from Gamepad API object.
   * Conforms to docs/GAMEPAD_THROTTLE_MAPPING_FIX_GUIDE.md
   */
  static readGamepadInputs(gp) {
    if (!gp) {
      return {
        connected: false,
        isRcTransmitter: false,
        deviceName: "Chưa cắm",
        thrPct: 0,
        rawThr: 0,
        rollDeg: 0,
        pitchDeg: 0,
        yawDeg: 0
      };
    }

    const id = gp.id || "";
    const isRcTransmitter = /radiomaster|pocket|edgetx|opentx|jumper|boxer|tx16|t-pro/i.test(id);
    // [GP DEBUG] Log raw axes/buttons to console every 60 frames (~1s) for axis mapping diagnosis
    if (!DashboardApplicationController._dbgFrame) DashboardApplicationController._dbgFrame = 0;
    if (++DashboardApplicationController._dbgFrame % 60 === 0) {
      const activeBtns = (gp.buttons || []).map((b, i) => b.pressed ? i : null).filter(x => x !== null).join(",");
      console.log(`[GP Debug] id="${id}" isRC=${isRcTransmitter} axes=[${[...gp.axes].map(v => v.toFixed(3)).join(",")}] btns=[${activeBtns}]`);
    }
    const deviceName = gp.id ? (isRcTransmitter ? "🎮 RadioMaster (HID)" : (id.length > 16 ? id.substring(0, 16) : id)) : "Đã nhận";

    let rawThr = 0;
    let yawDeg = 0;
    let rollDeg = 0;
    let pitchDeg = 0;

    if (isRcTransmitter) {
      // --- CHẾ ĐỘ RADIOMASTER POCKET / EDGETX (CHUẨN MODE 2 - AETR) ---
      // Kênh 3: Ga (Throttle) -> axes[2]: Đáy = -1.0 (0% ga), Đỉnh = +1.0 (100% ga)
      const rawThrAxis = (gp.axes && gp.axes.length > 2) ? gp.axes[2] : -1.0;
      let normThr = (rawThrAxis + 1.0) * 0.5;
      if (normThr < 0.02) normThr = 0.0;
      rawThr = Math.min(1.0, Math.max(0.0, normThr));

      // Kênh 4: Hướng (Yaw) -> axes[3] (-45° .. +45°)
      const rawYawAxis = (gp.axes && gp.axes.length > 3) ? gp.axes[3] : 0;
      yawDeg = Math.round(DashboardApplicationController.deadzone(rawYawAxis) * 45.0 * 10) / 10;

      // Kênh 1: Nghiêng ngang (Roll) -> axes[0] (-25° .. +25°)
      const rawRollAxis = (gp.axes && gp.axes.length > 0) ? gp.axes[0] : 0;
      rollDeg = Math.round(DashboardApplicationController.deadzone(rawRollAxis) * 25.0 * 10) / 10;

      // Kênh 2: Chúc ngóc (Pitch) -> axes[1] (Đẩy lên là chúc mũi / Pitch âm)
      const rawPitchAxis = (gp.axes && gp.axes.length > 1) ? gp.axes[1] : 0;
      pitchDeg = Math.round(-DashboardApplicationController.deadzone(rawPitchAxis) * 25.0 * 10) / 10;
    } else {
      // --- CHẾ ĐỘ TAY CẦM GAMEPAD CHUẨN XBOX / A124 ---
      // RT là buttons[7].value trong [0, 1]
      const rtVal = (gp.buttons && gp.buttons[7]) ? gp.buttons[7].value : 0;
      // Cần trái Y: đẩy lên là âm, kéo xuống là dương
      const rawLy = (gp.axes && gp.axes.length > 1) ? gp.axes[1] : 0;
      const stickLy = Math.max(0, -DashboardApplicationController.deadzone(rawLy));
      rawThr = Math.max(rtVal, stickLy);

      // Cần trái X: Yaw (-45° .. +45°)
      const rawLx = (gp.axes && gp.axes.length > 0) ? gp.axes[0] : 0;
      yawDeg = Math.round(DashboardApplicationController.deadzone(rawLx) * 45.0 * 10) / 10;

      // Cần phải: Roll (-25° .. +25°) & Pitch (-25° .. +25°)
      let rawRx = 0;
      let rawRy = 0;
      if (gp.axes) {
        if (gp.axes.length >= 4) {
          rawRx = gp.axes[2];
          rawRy = gp.axes[3];
        }
        if (gp.axes.length >= 5 && Math.abs(rawRx) < 0.05 && Math.abs(rawRy) < 0.05) {
          if (Math.abs(gp.axes[3]) > 0.05 || Math.abs(gp.axes[4]) > 0.05) {
            rawRx = gp.axes[3];
            rawRy = gp.axes[4];
          }
        }
      }
      rollDeg = Math.round(DashboardApplicationController.deadzone(rawRx) * 25.0 * 10) / 10;
      pitchDeg = Math.round(-DashboardApplicationController.deadzone(rawRy) * 25.0 * 10) / 10;
    }

    const thrPct = Math.round(Math.min(100.0, Math.max(0.0, rawThr * 100.0)) * 10) / 10;

    return {
      connected: true,
      isRcTransmitter,
      deviceName,
      thrPct,
      rawThr,
      rollDeg,
      pitchDeg,
      yawDeg
    };
  }

  startXboxGamepadLoop(setThrottle, setMotorTest) {
    let prevButtons = [];
    let prevAxis4 = 0;
    let isGamepadArmed = false;
    let baseThrottle = 0;
    let activeMotor = 0; // 0 = All (Throttle), 1..4 = Motor 1..4
    let lastSentThrottle = -1;
    let lastSentRoll = 999;
    let lastSentPitch = 999;
    let lastSentYaw = 999;
    let lastSendTime = 0;

    const pollGamepad = () => {
      const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
      let gp = null;
      for (let i = 0; i < gamepads.length; i++) {
        if (gamepads[i] && gamepads[i].connected) {
          gp = gamepads[i];
          break;
        }
      }

      const lblGamepadStatus = document.getElementById("lblGamepadStatus");
      const lblGamepadArm = document.getElementById("lblGamepadArm");
      const lblStickL = document.getElementById("lblStickL");
      const lblStickR = document.getElementById("lblStickR");
      const lblEspConfirmed = document.getElementById("lblEspConfirmed");

      // Cổng an toàn kiểm tra kết nối Transport thật (WebSocket hoặc Serial)
      const hasLink = this.telemetryStore.isTransportOpen();

      // Nếu mất link hoặc transport đóng -> ép buộc DISARMED và hiện cảnh báo NO LINK
      if (!hasLink) {
        isGamepadArmed = false;
        baseThrottle = 0;
        if (lblGamepadArm) {
          lblGamepadArm.textContent = "NO LINK";
          lblGamepadArm.style.color = "#94a3b8";
        }
      } else {
        // Đồng bộ trạng thái ARM từ telemetry thật của ESP32
        if (this.telemetryStore.lastEspState === "DISARMED" && isGamepadArmed) {
          isGamepadArmed = false;
          baseThrottle = 0;
        }
      }

      // Cập nhật phản hồi ESP32 đã xác nhận vào UI (Tách riêng biệt với Gamepad Input)
      if (lblEspConfirmed) {
        const espState = this.telemetryStore.lastEspState || "OFFLINE";
        const rcSeq = this.telemetryStore.lastRcSequence || 0;
        const spT = (this.telemetryStore.lastTargetThrottle || 0).toFixed(1);
        lblEspConfirmed.textContent = `Seq: #${rcSeq} | SP_T: ${spT}% | ${espState}`;
      }

      if (gp) {
        const inputs = DashboardApplicationController.readGamepadInputs(gp);
        if (lblGamepadStatus) {
          lblGamepadStatus.textContent = inputs.deviceName;
          lblGamepadStatus.style.color = "#34d399";
        }

        const btns = gp.buttons.map((b) => b.pressed);

        // Khởi tạo trạng thái nút ban đầu để tránh tự động kích hoạt ARM/DISARM khi vừa cắm tay cầm
        if (prevButtons.length === 0) {
          prevButtons = btns;
          if (gp.axes && gp.axes.length > 4) prevAxis4 = gp.axes[4];
          requestAnimationFrame(pollGamepad);
          return;
        }

        const justPressed = (idx) => btns[idx] && !prevButtons[idx];
        const justReleased = (idx) => !btns[idx] && prevButtons[idx];

        // Công tắc SA trên RadioMaster (qua Axis 4 / Channel 5 nếu EdgeTX ở chế độ 8-axis)
        const currAxis4 = (gp.axes && gp.axes.length > 4) ? gp.axes[4] : 0;
        const saAxisArm = inputs.isRcTransmitter && gp.axes && gp.axes.length > 4 && (currAxis4 > 0.3 && prevAxis4 <= 0.3);
        const saAxisDisarm = inputs.isRcTransmitter && gp.axes && gp.axes.length > 4 && (currAxis4 < -0.3 && prevAxis4 >= -0.3);
        prevAxis4 = currAxis4;

        // 1. DISARM / DỪNG KHẨN CẤP:
        // - Trên mọi tay cầm: Nút B (1), Back (8), Guide (9)
        // - Trên RadioMaster (Công tắc SA): Gạt SA về vị trí UP (nhả Button 0 hoặc Button 4, hoặc Axis 4 âm)
        const isDisarmRequested = justPressed(1) || justPressed(8) || justPressed(9) ||
          (inputs.isRcTransmitter && (justReleased(0) || justReleased(4) || saAxisDisarm));

        // 2. ARM (Mở khóa an toàn):
        // - Trên Xbox: Nút A (0)
        // - Trên RadioMaster (Công tắc SA): Gạt SA xuống vị trí DOWN (nhấn Button 0 hoặc Button 4, hoặc Axis 4 dương)
        const isArmRequested = justPressed(0) || (inputs.isRcTransmitter && (justPressed(4) || saAxisArm));

        if (isDisarmRequested) {
          isGamepadArmed = false;
          baseThrottle = 0;
          lastSentThrottle = 0;
          lastSendTime = performance.now();
          this.telemetryStore.sendCommand("disarm");
          this.telemetryStore.lastPidEnabled = true;   // Firmware: PID BẬT mặc định sau DISARM
          this.updatePidButtonUi(true);
          setThrottle(0);
        }
        else if (isArmRequested) {
          if (!hasLink) {
            console.warn("[Gamepad] Từ chối ARM: Không có kết nối Transport tới ESP32 (WebSocket/Serial đang đóng)!");
          } else if (inputs.thrPct > 3.0) {
            console.warn(`[Gamepad] Từ chối ARM: Cần ga phải ở 0% trước khi ARM! (Hiện tại: ${inputs.thrPct}%)`);
            alert("⚠️ CẢNH BÁO AN TOÀN: Vui lòng hạ cần ga về 0% trước khi gạt công tắc SA để ARM!");
          } else {
            isGamepadArmed = true;
            baseThrottle = 0;
            lastSentThrottle = 0;
            lastSendTime = performance.now();
            this.telemetryStore.sendCommand("arm");
          }
        }

        // 3. RB (5) / LB (4): Tăng giảm ga giữ nền (+5% / -5%) - chỉ áp dụng khi đã ARMED, có Link và không phải RC Transmitter
        if (isGamepadArmed && hasLink && !inputs.isRcTransmitter) {
          if (justPressed(5)) { baseThrottle = Math.min(100, baseThrottle + 5); }
          if (justPressed(4)) { baseThrottle = Math.max(0, baseThrottle - 5); }
        }

        // 4. D-Pad chọn motor test thủ công: Up(12)=M1, Right(15)=M2, Down(13)=M3, Left(14)=M4, Y(3)=ALL
        if (justPressed(12)) { activeMotor = 1; baseThrottle = 0; this.telemetryStore.sendCommand("m1"); }
        if (justPressed(15)) { activeMotor = 2; baseThrottle = 0; this.telemetryStore.sendCommand("m2"); }
        if (justPressed(13)) { activeMotor = 3; baseThrottle = 0; this.telemetryStore.sendCommand("m3"); }
        if (justPressed(14)) { activeMotor = 4; baseThrottle = 0; this.telemetryStore.sendCommand("m4"); }
        if (justPressed(3)) { activeMotor = 0; baseThrottle = 0; this.telemetryStore.sendCommand("all"); }

        // 5. KIỂM TRA ARMED TỪ BẤT KỲ NGUỒN NÀO (tay cầm hoặc Web UI)
        // effectiveArmed = true nếu: (a) đã nhấn nút ARM trên tay cầm, HOẶC (b) ESP32 xác nhận ARMED từ Web UI
        const effectiveArmed = isGamepadArmed || (hasLink && this.telemetryStore.lastEspState === "ARMED");
        if (!effectiveArmed) {
          if (lblStickL) lblStickL.textContent = `T:${inputs.thrPct.toFixed(0)}% | Y:${inputs.yawDeg > 0 ? "+" : ""}${inputs.yawDeg.toFixed(0)}°`;
          if (lblStickR) lblStickR.textContent = `P:${inputs.pitchDeg > 0 ? "+" : ""}${inputs.pitchDeg.toFixed(0)}° | R:${inputs.rollDeg > 0 ? "+" : ""}${inputs.rollDeg.toFixed(0)}°`;
          if (lblGamepadArm && hasLink) {
            lblGamepadArm.textContent = inputs.isRcTransmitter ? "DISARM (SA)" : "DISARM";
            lblGamepadArm.style.color = "#ef4444";
          }
          prevButtons = btns;
          requestAnimationFrame(pollGamepad);
          return;
        }

        // 6. TÍNH TOÁN THROTTLE KHI ĐANG ARMED (từ tay cầm hoặc Web UI)
        const maxCap = 100;
        const gpadThrPct = inputs.rawThr * maxCap; // 0..100 từ cần ga vật lý
        // Khi stick >2%: quyền điều khiển ga thuộc về cần ga vật lý (không dùng espFloor)
        // Khi stick ở đáy (<2%) và chưa chỉnh RB/LB: giữ nguyên mức ga ESP32 xác nhận (web slider)
        const espFloor = (gpadThrPct < 2.0 && baseThrottle === 0)
          ? (this.telemetryStore.lastTargetThrottle || 0)
          : 0;
        let totalThr = (effectiveArmed && hasLink)
          ? Math.min(maxCap, Math.max(0, Math.round((baseThrottle + gpadThrPct + espFloor) * 10) / 10))
          : 0;

        // Cập nhật giao diện khi đang ARMED
        if (lblStickL) lblStickL.textContent = `T:${totalThr.toFixed(0)}% | Y:${inputs.yawDeg > 0 ? "+" : ""}${inputs.yawDeg.toFixed(0)}°`;
        if (lblStickR) lblStickR.textContent = `P:${inputs.pitchDeg > 0 ? "+" : ""}${inputs.pitchDeg.toFixed(0)}° | R:${inputs.rollDeg > 0 ? "+" : ""}${inputs.rollDeg.toFixed(0)}°`;
        if (lblGamepadArm && hasLink) {
          lblGamepadArm.textContent = inputs.isRcTransmitter ? "ARMED (SA)" : "ARMED";
          lblGamepadArm.style.color = "#22c55e";
        }

        // 7. PHÁT GÓI TIN ĐIỀU KHIỂN TỚI ESP32 (DUY NHẤT KHI ĐANG ARMED TAY CẦM)
        const now = performance.now();
        const thrChanged = Math.abs(totalThr - lastSentThrottle) >= 0.5;
        const attChanged = Math.abs(inputs.rollDeg - lastSentRoll) >= 0.5 || Math.abs(inputs.pitchDeg - lastSentPitch) >= 0.5 || Math.abs(inputs.yawDeg - lastSentYaw) >= 0.5;
        // Heartbeat mỗi 200ms để duy trì sóng và chống timeout watchdog 1500ms của ESP32
        const heartbeatDue = (now - lastSendTime >= 200);

        if (hasLink && (((thrChanged || attChanged) && (now - lastSendTime >= 30)) || heartbeatDue)) {
          lastSentThrottle = totalThr;
          lastSentRoll = inputs.rollDeg;
          lastSentPitch = inputs.pitchDeg;
          lastSentYaw = inputs.yawDeg;
          lastSendTime = now;

          if (activeMotor === 0) {
            // Gửi gói tin RC chuẩn: RC,<throttle>,<roll>,<pitch>,<yaw>,1
            const rcCommand = `RC,${totalThr.toFixed(1)},${inputs.rollDeg.toFixed(1)},${inputs.pitchDeg.toFixed(1)},${inputs.yawDeg.toFixed(1)},1`;
            this.telemetryStore.sendCommand(rcCommand);
            // Đồng bộ thanh trượt hiển thị trên Web với cần ga tay cầm khi đang ARMED bằng tay cầm
            setThrottle(totalThr);
          } else {
            setMotorTest(activeMotor, Math.min(100, totalThr));
          }
        }

        prevButtons = btns;
      } else {
        if (lblGamepadStatus) {
          lblGamepadStatus.textContent = "Chưa cắm";
          lblGamepadStatus.style.color = "#f59e0b";
        }
      }
      requestAnimationFrame(pollGamepad);
    };

    requestAnimationFrame(pollGamepad);
  }

  handleCustomCsvSelection(event, setActiveMode) {
    const selectedFile = event.target.files[0];
    if (!selectedFile) return;
    const reader = new FileReader();
    reader.onload = (loadEvent) => {
      this.csvReplayer.parseCsvContent(String(loadEvent.target.result || ""));
      setActiveMode("bCsv");
      this.telemetryStore.sampleBuffer.length = 0;
      this.csvReplayer.startPlayback();
    };
    reader.readAsText(selectedFile);
  }

  updatePidButtonUi(enabled) {
    const btn = document.getElementById("btnPidView");
    if (!btn) return;
    if (enabled) {
      btn.classList.add("active");
      btn.textContent = "✅ PID: Ổn Định (0°)";
      btn.title = "PID đang giữ ổn định ở 3 góc (0,0,0). Nhấn để chuyển sang ACRO (Tắt PID)";
    } else {
      btn.classList.remove("active");
      btn.textContent = "Bật PID (0°)";
      btn.title = "Nhấn để kéo Drone từ mất ổn định về thăng bằng 3 góc = 0°";
    }
  }
}

window.DashboardApplicationController = DashboardApplicationController;
