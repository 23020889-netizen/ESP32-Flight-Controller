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

    // 2. KẾT NỐI QUA SÓNG WIFI (WebSocket - Mạng TTDB hoặc SoftAP)
    const bLiveWs = getElement("bLiveWs");
    const inpWsIp = getElement("inpWsIp");
    if (inpWsIp) {
      const savedIp = localStorage.getItem("uav_esp_ip");
      if (savedIp) inpWsIp.value = savedIp;
    }
    if (bLiveWs) {
      bLiveWs.onclick = () => {
        this.csvReplayer.stopPlayback();
        const ip = (inpWsIp && inpWsIp.value.trim()) ? inpWsIp.value.trim() : "10.11.162.210";
        if (inpWsIp) localStorage.setItem("uav_esp_ip", ip);
        const wsUrl = ip.startsWith("ws://") ? ip : `ws://${ip}:81`;
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

    // Chuyển đổi Mẫu 3D Quadcopter (4 mẫu đều khắc chữ ISILAB trên thân)
    const selDroneModel = getElement("selDroneModel");
    if (selDroneModel) {
      try {
        const savedModel = localStorage.getItem("uav_3d_model");
        if (savedModel) {
          selDroneModel.value = savedModel;
          this.sceneManager.setDroneModel(savedModel);
        }
      } catch (_) {}
      selDroneModel.onchange = () => {
        const chosen = selDroneModel.value || "glb_aero";
        this.sceneManager.setDroneModel(chosen);
        try {
          localStorage.setItem("uav_3d_model", chosen);
        } catch (_) {}
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
      } catch (_) {}
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

  startXboxGamepadLoop(setThrottle, setMotorTest) {
    let prevButtons = [];
    let isGamepadArmed = false;
    let baseThrottle = 0;
    let activeMotor = 0; // 0 = All (Throttle), 1..4 = Motor 1..4
    let lastSentThrottle = -1;
    let lastSentRoll = 999;
    let lastSentPitch = 999;
    let lastSentYaw = 999;
    let lastSendTime = 0;

    const deadzone = (v, th = 0.08) => {
      if (Math.abs(v) < th) return 0;
      return v > 0 ? (v - th) / (1 - th) : (v + th) / (1 - th);
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

      const lblGamepadStatus = document.getElementById("lblGamepadStatus");
      const lblGamepadArm    = document.getElementById("lblGamepadArm");
      const lblStickL        = document.getElementById("lblStickL");
      const lblStickR        = document.getElementById("lblStickR");

      if (gp) {
        if (lblGamepadStatus) {
          lblGamepadStatus.textContent = gp.id ? gp.id.substring(0, 16) : "Đã nhận";
          lblGamepadStatus.style.color = "#34d399";
        }

        const btns = gp.buttons.map((b) => b.pressed);
        const justPressed = (idx) => btns[idx] && !prevButtons[idx];

        // 1. Nút B (1) hoặc Back (8) / Guide: DISARM / DỪNG KHẨN CẤP
        if (justPressed(1) || justPressed(8) || justPressed(9)) {
          isGamepadArmed = false;
          baseThrottle = 0;
          lastSentThrottle = 0;
          lastSendTime = performance.now();
          this.telemetryStore.sendCommand("disarm");
          setThrottle(0);
        }
        // 2. Nút A (0): ARM (Mở khóa an toàn)
        else if (justPressed(0)) {
          isGamepadArmed = true;
          baseThrottle = 0;
          lastSentThrottle = 0;
          lastSendTime = performance.now();
          this.telemetryStore.sendCommand("arm");
        }

        // 3. RB (5) / LB (4): Tăng giảm ga giữ nền (+5% / -5%)
        if (justPressed(5)) { isGamepadArmed = true; baseThrottle = Math.min(100, baseThrottle + 5); }
        if (justPressed(4)) { baseThrottle = Math.max(0, baseThrottle - 5); }

        // 4. D-Pad chọn motor test thủ công: Up(12)=M1, Right(15)=M2, Down(13)=M3, Left(14)=M4, Y(3)=ALL
        if (justPressed(12)) { activeMotor = 1; baseThrottle = 0; this.telemetryStore.sendCommand("m1"); }
        if (justPressed(15)) { activeMotor = 2; baseThrottle = 0; this.telemetryStore.sendCommand("m2"); }
        if (justPressed(13)) { activeMotor = 3; baseThrottle = 0; this.telemetryStore.sendCommand("m3"); }
        if (justPressed(14)) { activeMotor = 4; baseThrottle = 0; this.telemetryStore.sendCommand("m4"); }
        if (justPressed(3))  { activeMotor = 0; baseThrottle = 0; this.telemetryStore.sendCommand("all"); }

        // 5. ĐỌC CẦN ANALOG TRÁI (THROTTLE & YAW):
        // Throttle: axes[1] đẩy lên là âm (-1.0 -> 0)
        const rawLy = (gp.axes && gp.axes.length > 1) ? gp.axes[1] : 0;
        const rawLx = (gp.axes && gp.axes.length > 0) ? gp.axes[0] : 0;
        const rtVal = gp.buttons[7] ? gp.buttons[7].value : 0;
        const stickLy = Math.max(0, -deadzone(rawLy));
        const stickThr = Math.max(rtVal, stickLy);

        const maxCap = 100; // Trần ga toàn dải 100% theo yêu cầu
        let totalThr = isGamepadArmed ? Math.min(maxCap, Math.max(0, Math.round((baseThrottle + stickThr * maxCap) * 10) / 10)) : 0;

        // Yaw: axes[0] (-45° .. +45°)
        const yawDeg = Math.round(deadzone(rawLx) * 45.0 * 10) / 10;

        // 6. ĐỌC CẦN ANALOG PHẢI (ROLL & PITCH) - TƯƠNG THÍCH MỌI DÒNG TAY CẦM A124 / XBOX:
        let rawRx = 0;
        let rawRy = 0;
        if (gp.axes) {
          if (gp.axes.length >= 4) {
            rawRx = gp.axes[2];
            rawRy = gp.axes[3];
          }
          // Một số tay cầm USB generic A124 đặt cần phải tại axes[3] và axes[4] (khi axes[2] là trigger)
          if (gp.axes.length >= 5 && Math.abs(rawRx) < 0.05 && Math.abs(rawRy) < 0.05) {
            if (Math.abs(gp.axes[3]) > 0.05 || Math.abs(gp.axes[4]) > 0.05) {
              rawRx = gp.axes[3];
              rawRy = gp.axes[4];
            }
          }
        }
        const rollDeg  = Math.round(deadzone(rawRx) * 25.0 * 10) / 10;
        const pitchDeg = Math.round(-deadzone(rawRy) * 25.0 * 10) / 10;

        // 7. CẬP NHẬT GIAO DIỆN HIỂN THỊ TRỰC TIẾP TRÊN WEB
        if (lblStickL) lblStickL.textContent = `T:${totalThr.toFixed(0)}% | Y:${yawDeg > 0 ? "+" : ""}${yawDeg.toFixed(0)}°`;
        if (lblStickR) lblStickR.textContent = `P:${pitchDeg > 0 ? "+" : ""}${pitchDeg.toFixed(0)}° | R:${rollDeg > 0 ? "+" : ""}${rollDeg.toFixed(0)}°`;
        if (lblGamepadArm) {
          lblGamepadArm.textContent = isGamepadArmed ? "ARMED" : "DISARM";
          lblGamepadArm.style.color = isGamepadArmed ? "#22c55e" : "#ef4444";
        }

        // 8. PHÁT GÓI TIN ĐIỀU KHIỂN TỚI ESP32 QUA WEBSOCKET (30 Hz hoặc khi giá trị thay đổi)
        const now = performance.now();
        const thrChanged = Math.abs(totalThr - lastSentThrottle) >= 0.5;
        const attChanged = Math.abs(rollDeg - lastSentRoll) >= 0.5 || Math.abs(pitchDeg - lastSentPitch) >= 0.5 || Math.abs(yawDeg - lastSentYaw) >= 0.5;
        const heartbeatDue = (isGamepadArmed || totalThr > 0) && (now - lastSendTime >= 350);

        if (((thrChanged || attChanged) && (now - lastSendTime >= 30)) || heartbeatDue || totalThr === 0 && lastSentThrottle > 0) {
          lastSentThrottle = totalThr;
          lastSentRoll     = rollDeg;
          lastSentPitch    = pitchDeg;
          lastSentYaw      = yawDeg;
          lastSendTime     = now;

          if (activeMotor === 0) {
            // Gửi gói tin RC chuẩn: RC,<throttle>,<roll>,<pitch>,<yaw>,<arm>
            const armCode = isGamepadArmed ? 1 : 0;
            this.telemetryStore.sendCommand(`RC,${totalThr.toFixed(1)},${rollDeg.toFixed(1)},${pitchDeg.toFixed(1)},${yawDeg.toFixed(1)},${armCode}`);
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
}

window.DashboardApplicationController = DashboardApplicationController;
