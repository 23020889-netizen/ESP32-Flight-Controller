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
      ["bDemo", "bCsv", "bSerial", "bLiveWs"].forEach((id) => {
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

    // 2. KẾT NỐI QUA SÓNG WIFI (WebSocket - Yêu cầu bắt WiFi ESP32-UAV-FC)
    const bLiveWs = getElement("bLiveWs");
    if (bLiveWs) {
      bLiveWs.onclick = () => {
        this.csvReplayer.stopPlayback();
        const defaultUrl = "ws://192.168.4.1:81";
        const wsUrl = prompt("Nhập URL WebSocket ESP32-C6 (Laptop phải đang kết nối vào WiFi 'ESP32-UAV-FC'):", defaultUrl) || defaultUrl;
        this.telemetryStore.connectWebSocket(wsUrl);
        setActiveMode("bLiveWs");
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
      this.telemetryStore.sendCommand(`throttle ${pct}`);
    };

    if (rngThrottle) {
      rngThrottle.oninput = (e) => {
        const val = Number(e.target.value);
        if (lblThrottle) lblThrottle.textContent = val.toFixed(1) + "%";
      };
      rngThrottle.onchange = (e) => {
        setThrottle(Number(e.target.value));
      };
    }

    document.querySelectorAll(".btn-thr").forEach((b) => {
      b.onclick = () => setThrottle(Number(b.dataset.thr));
    });

    // Điều khiển Góc Setpoint (PX4 Angle Control)
    const btnLevel = getElement("btnLevel");
    if (btnLevel) {
      btnLevel.onclick = () => {
        this.telemetryStore.sendCommand("level");
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
        this.telemetryStore.sendCommand("pid");
      };
    }

    document.querySelectorAll(".btn-sp").forEach((b) => {
      b.onclick = () => {
        this.telemetryStore.sendCommand(`sp ${b.dataset.sp}`);
      };
    });

    // --- ĐIỀU KHIỂN TÙY BIẾN TEST TỪNG ĐỘNG CƠ (INDIVIDUAL MOTOR TEST) ---
    const setMotorTest = (motorId, val) => {
      val = Math.max(0, Math.min(50, Number(val) || 0));
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

    // 5. HỖ TRỢ TAY CẦM XBOX CẮM VÀO MÁY TÍNH (HTML5 GAMEPAD API)
    this.startXboxGamepadLoop(setThrottle, setMotorTest);
  }

  startXboxGamepadLoop(setThrottle, setMotorTest) {
    let prevButtons = [];
    let isGamepadArmed = false;
    let baseThrottle = 0;
    let activeMotor = 0; // 0 = All 4 Motors (Quad-X Mixer), 1..4 = Single Motor 1..4
    let lastSentCmd = "";
    let lastSendTime = 0;

    const applyDeadzone = (val, dz = 0.08) => {
      if (!val || Math.abs(val) < dz) return 0;
      return (val - Math.sign(val) * dz) / (1 - dz);
    };

    const updateMotorSlidersUI = (m1, m2, m3, m4, thr) => {
      const rngThr = document.getElementById("rngThrottle");
      const lblThr = document.getElementById("lblThrottle");
      if (rngThr) rngThr.value = thr;
      if (lblThr) lblThr.textContent = thr.toFixed(1) + "%";

      const vals = [m1, m2, m3, m4];
      for (let i = 0; i < 4; i++) {
        const r = document.getElementById(`rngM${i + 1}`);
        const l = document.getElementById(`lblM${i + 1}`);
        if (r) r.value = Math.round(vals[i]);
        if (l) l.textContent = vals[i].toFixed(1) + "%";
      }
    };

    const pollGamepad = () => {
      const gamepads = navigator.getGamepads ? navigator.getGamepads() : [];
      let gp = null;
      for (let i = 0; i < gamepads.length; i++) {
        if (gamepads[i] && gamepads[i].connected) {
          gp = gamepads[i];
          break;
        }
      }

      if (gp) {
        const btns = gp.buttons.map((b) => b.pressed);
        const justPressed = (idx) => btns[idx] && !prevButtons[idx];

        // Nút B (1) hoặc Back (8): DISARM / DỪNG KHẨN CẤP
        if (justPressed(1) || justPressed(8)) {
          isGamepadArmed = false;
          baseThrottle = 0;
          lastSentCmd = "disarm";
          lastSendTime = performance.now();
          this.telemetryStore.sendCommand("disarm");
          updateMotorSlidersUI(0, 0, 0, 0, 0);
        }
        // Nút A (0): ARM
        else if (justPressed(0)) {
          isGamepadArmed = true;
          baseThrottle = 0;
          lastSentCmd = "arm";
          lastSendTime = performance.now();
          this.telemetryStore.sendCommand("arm");
          updateMotorSlidersUI(0, 0, 0, 0, 0);
        }

        // RB (5) / LB (4): Tăng giảm ga giữ nền (+5% / -5%)
        if (justPressed(5)) { isGamepadArmed = true; baseThrottle = Math.min(60, baseThrottle + 5); }
        if (justPressed(4)) { baseThrottle = Math.max(0, baseThrottle - 5); }

        // D-Pad: Up(12)=M1, Right(15)=M2, Down(13)=M3, Left(14)=M4, Y(3)=ALL
        if (justPressed(12)) { activeMotor = 1; baseThrottle = 0; this.telemetryStore.sendCommand("m1"); }
        if (justPressed(15)) { activeMotor = 2; baseThrottle = 0; this.telemetryStore.sendCommand("m2"); }
        if (justPressed(13)) { activeMotor = 3; baseThrottle = 0; this.telemetryStore.sendCommand("m3"); }
        if (justPressed(14)) { activeMotor = 4; baseThrottle = 0; this.telemetryStore.sendCommand("m4"); }
        if (justPressed(3))  { activeMotor = 0; baseThrottle = 0; this.telemetryStore.sendCommand("all"); }

        // 1. Đọc trục Throttle: Cò phải RT (button 7: 0..1) hoặc Cần trái đẩy lên (-axes[1]: 0..1)
        const rtVal = gp.buttons[7] ? gp.buttons[7].value : 0;
        const lyVal = gp.axes && gp.axes.length > 1 ? Math.max(0, applyDeadzone(-gp.axes[1], 0.08)) : 0;
        const stickThr = Math.max(rtVal, lyVal);
        const totalThr = Math.min(60, Math.max(0, baseThrottle + stickThr * 60));

        // 2. Đọc 3 trục điều hướng: Yaw (axes[0]), Roll (axes[2]), Pitch (-axes[3])
        const yawNorm   = gp.axes && gp.axes.length > 0 ? applyDeadzone(gp.axes[0], 0.08) : 0;
        const rollNorm  = gp.axes && gp.axes.length > 2 ? applyDeadzone(gp.axes[2], 0.08) : 0;
        const pitchNorm = gp.axes && gp.axes.length > 3 ? applyDeadzone(-gp.axes[3], 0.08) : 0;

        const maxTilt = Math.max(Math.abs(rollNorm), Math.abs(pitchNorm), Math.abs(yawNorm));
        if (stickThr > 0.02 || maxTilt > 0.05) {
          isGamepadArmed = true;
        }

        const now = performance.now();
        if (activeMotor === 0) {
          // Chế độ điều khiển cả 4 động cơ + Trộn Quad-X (Throttle + Roll + Pitch + Yaw)
          const rollMix  = rollNorm  * 20.0; // +-20%
          const pitchMix = pitchNorm * 20.0; // +-20%
          const yawMix   = yawNorm   * 15.0; // +-15%

          // Nếu chưa bóp ga (totalThr == 0) nhưng gạt cần Roll/Pitch/Yaw -> Tự cấp ga nền 20% theo độ gạt cần để test nghiêng ngay
          const effectiveThr = totalThr > 1.0 ? totalThr : (maxTilt > 0 ? maxTilt * 20.0 : 0.0);

          let m1 = 0, m2 = 0, m3 = 0, m4 = 0;
          if (effectiveThr > 0.2 || maxTilt > 0) {
            m1 = Math.min(60, Math.max(0, effectiveThr - rollMix + pitchMix - yawMix)); // M1: FR (CCW)
            m2 = Math.min(60, Math.max(0, effectiveThr + rollMix - pitchMix - yawMix)); // M2: RL (CCW)
            m3 = Math.min(60, Math.max(0, effectiveThr + rollMix + pitchMix + yawMix)); // M3: FL (CW)
            m4 = Math.min(60, Math.max(0, effectiveThr - rollMix - pitchMix + yawMix)); // M4: RR (CW)
          }

          const cmd = `x ${m1.toFixed(1)} ${m2.toFixed(1)} ${m3.toFixed(1)} ${m4.toFixed(1)}`;
          const valueChanged = (cmd !== lastSentCmd) && (now - lastSendTime >= 40 || (m1 === 0 && m2 === 0 && m3 === 0 && m4 === 0));
          const heartbeatDue = isGamepadArmed && (now - lastSendTime >= 500);

          if (valueChanged || heartbeatDue) {
            lastSentCmd = cmd;
            lastSendTime = now;
            this.telemetryStore.sendCommand(cmd);
            updateMotorSlidersUI(m1, m2, m3, m4, effectiveThr);
          }
        } else {
          // Chế độ test riêng lẻ 1 động cơ (khi bấm D-Pad)
          const singleThr = Math.min(50, Math.round(totalThr * 10) / 10);
          const cmd = `m${activeMotor} ${singleThr}`;
          const valueChanged = (cmd !== lastSentCmd) && (now - lastSendTime >= 40 || singleThr === 0);
          const heartbeatDue = isGamepadArmed && (now - lastSendTime >= 500);

          if (valueChanged || heartbeatDue) {
            lastSentCmd = cmd;
            lastSendTime = now;
            setMotorTest(activeMotor, singleThr);
          }
        }

        prevButtons = btns;
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
}

window.DashboardApplicationController = DashboardApplicationController;
