#include <Arduino.h>
<<<<<<< HEAD
#include "src/sensors/mpu6050.h"
#include "src/sensors/xsens_mti.h"
#include "src/estimation/px4_ekf.h"
#include "src/actuators/esc_pwm.h"
#include "src/supervisor/flight_controller_manager.hpp"
#include "src/supervisor/serial_telemetry_reporter.hpp"
#include "src/supervisor/serial_command_interpreter.hpp"
=======
#include "mpu6050.h"
#include "xsens_mti.h"
#include "px4_ekf.h"
#include "px4_filters.h"
#include "px4_pid.h"
#include "esc_pwm.h"
#include "telemetry_wifi.h"
>>>>>>> ae916b5 ( update base)

// Hardware Pin Configuration (ESP32-C6)
#define MPU_SDA_PIN     14
#define MPU_SCL_PIN     15
#define XSENS_RX_PIN    2
#define XSENS_TX_PIN    3
#define XSENS_BAUDRATE  115200

<<<<<<< HEAD
=======
// Động cơ Quadcopter chuẩn PX4 (Đã định nghĩa trong esc_pwm.h):
// Motor 1: GPIO 7 (Front-Right, CCW)
// Motor 2: GPIO 6 (Rear-Left,   CCW)
// Motor 3: GPIO 5 (Front-Left,  CW)
// Motor 4: GPIO 4 (Rear-Right,  CW)

// ============================================================================
// 2. GLOBAL MODULE OBJECTS & DATA HOLDERS
// ============================================================================
>>>>>>> ae916b5 ( update base)
MpuData   mpuData   = {0};
XsensData xsensData = {0};
FusedData fusedData = {0};
PX4_EKF   ekf;

static uint32_t previousLoopMicros = 0;

<<<<<<< HEAD
=======
// Chu kỳ vòng lặp PID và Serial Telemetry
uint32_t lastPidUs = 0;
const uint32_t PID_INTERVAL_US = 5000; // 200 Hz (5ms) chu kỳ chuẩn PX4

uint32_t lastPrintMs = 0;
const uint32_t PRINT_INTERVAL_MS = 50; // 20 Hz update cho Serial Telemetry

// ============================================================================
// 3. SERIAL TELEMETRY PRINTER
// ============================================================================
void printTelemetry() {
    Serial.println("-----------------------------------------------------------------------------------------------------------------");
    
    // 1. Dữ liệu MPU-6050 (I2C)
    if (mpuData.ready) {
        Serial.printf("[MPU6050] R:%6.1f° P:%6.1f° Y:%6.1f° | Acc:[%6.2f,%6.2f,%6.2f] | Gyr:[%6.1f,%6.1f,%6.1f]\n",
                      mpuData.roll, mpuData.pitch, mpuData.yaw,
                      mpuData.ax,   mpuData.ay,   mpuData.az,
                      mpuData.gx,   mpuData.gy,   mpuData.gz);
    } else {
        Serial.println("[MPU6050] Dang khoi tao hoac mat ket noi I2C...");
    }

    // 2. Dữ liệu Xsens MTi-3 (UART)
    if (xsensData.has_orientation || xsensData.has_accel) {
        Serial.printf("[Xsens  ] R:%6.1f° P:%6.1f° Y:%6.1f° | Acc:[%6.2f,%6.2f,%6.2f] | Gyr:[%6.1f,%6.1f,%6.1f]\n",
                      xsensData.roll, xsensData.pitch, xsensData.yaw,
                      xsensData.ax,   xsensData.ay,   xsensData.az,
                      xsensData.gx,   xsensData.gy,   xsensData.gz);
    } else {
        Serial.println("[Xsens  ] Dang cho goi tin Xbus tu UART1...");
    }

    // 3. Dữ liệu dung hợp EKF chuẩn PX4 (OPTIMAL FUSION OUTPUT)
    Serial.printf("[PX4_EKF] R:%6.1f° P:%6.1f° Y:%6.1f° | Acc:[%6.2f,%6.2f,%6.2f] | Gyr:[%6.1f,%6.1f,%6.1f] (OPTIMAL)\n",
                      fusedData.roll, fusedData.pitch, fusedData.yaw,
                      fusedData.ax,   fusedData.ay,   fusedData.az,
                      fusedData.gx,   fusedData.gy,   fusedData.gz);

    // 4. Trạng thái PX4 Multicopter PID Controller
    float tr, tp, ty;
    px4Controller.getTargetAttitude(tr, tp, ty);
    Serial.printf("[PX4_PID] Target:[R:%.1f°, P:%.1f°, Y:%.1f°] Thr:%4.1f%% | GyrFilt:[%6.1f,%6.1f,%6.1f]\n",
                  tr, tp, ty, px4Controller.getTargetThrottle(),
                  px4Controller.getFilteredGx(), px4Controller.getFilteredGy(), px4Controller.getFilteredGz());

    // 5. Trạng thái điều khiển 4 Động cơ Quadcopter chuẩn PX4 (GPIO 7, 6, 5, 4)
    Serial.printf("[ESC_QUAD] State:%-8s | M1(P7-CCW):%4dus(%4.1f%%) | M2(P6-CCW):%4dus(%4.1f%%) | M3(P5-CW):%4dus(%4.1f%%) | M4(P4-CW):%4dus(%4.1f%%) @ %dHz\n",
                  quadMotors.getStateStr(),
                  quadMotors.getMotor(0).getCurrentPulseUs(), quadMotors.getMotor(0).getThrottlePct(),
                  quadMotors.getMotor(1).getCurrentPulseUs(), quadMotors.getMotor(1).getThrottlePct(),
                  quadMotors.getMotor(2).getCurrentPulseUs(), quadMotors.getMotor(2).getThrottlePct(),
                  quadMotors.getMotor(3).getCurrentPulseUs(), quadMotors.getMotor(3).getThrottlePct(),
                  quadMotors.getFrequency());
}

// ============================================================================
// 4. UNIFIED COMMAND ENGINE (HỖ TRỢ SERIAL CLI + WIFI UDP + WEBSOCKET 3D MONITOR)
// ============================================================================
void executeCommand(String cmd, const char* sourceTag) {
    cmd.trim();
    if (cmd.length() == 0) return;

    char logBuf[128];

    if (cmd.equalsIgnoreCase("arm")) {
        quadMotors.arm();
        px4Controller.reset();
        // KHÓA HƯỚNG YAW HIỆN TẠI (Chống lệch góc trên bàn thử nghiệm làm tắt động cơ M0/M1)
        px4Controller.lockCurrentHeading(fusedData.yaw);
        snprintf(logBuf, sizeof(logBuf), "[%s] Quad Armed | Heading Locked: %.1f deg", sourceTag, fusedData.yaw);
        telemetry_wifi_broadcast_msg(logBuf);
        Serial.printf("[%s] >>> DA ARM HOAN TAT! Khoa huong Yaw hien tai: %.1f° lam moc Setpoint.\n", sourceTag, fusedData.yaw);
    } else if (cmd.equalsIgnoreCase("disarm") || cmd.equalsIgnoreCase("stop") || cmd.equalsIgnoreCase("kill")) {
        quadMotors.disarm();
        px4Controller.reset();
        px4Controller.setTargetThrottle(0.0f);
        snprintf(logBuf, sizeof(logBuf), "[%s] Quad Disarmed & Motors Safe", sourceTag);
        telemetry_wifi_broadcast_msg(logBuf);
    } else if (cmd.startsWith("throttle ") || cmd.startsWith("THROTTLE ") || cmd.startsWith("set ") || cmd.startsWith("SET ")) {
        int idx = cmd.indexOf(' ');
        float pct = cmd.substring(idx + 1).toFloat();
        px4Controller.setTargetThrottle(pct);
        Serial.printf("[%s] Dat muc ga tong the PX4: %.1f%%\n", sourceTag, pct);
    } else if (cmd.equalsIgnoreCase("invert pitch") || cmd.equalsIgnoreCase("inv pitch")) {
        px4Controller.togglePitchInversion();
        Serial.printf("[%s] DAO CHIEU TRUC PITCH: Invert = %s\n", sourceTag, px4Controller.isPitchInverted() ? "TRUE (NGUOC)" : "FALSE (THUAN)");
    } else if (cmd.equalsIgnoreCase("level") || cmd.equalsIgnoreCase("balance")) {
        px4Controller.setTargetAttitude(0.0f, 0.0f, fusedData.yaw);
        px4Controller.reset();
        Serial.printf("[%s] >>> CAN BANG: Dat Roll=0.0°, Pitch=0.0°, Giu nguyen Yaw=%.1f°\n", sourceTag, fusedData.yaw);
    } else if (cmd.startsWith("sp ") || cmd.startsWith("SP ") || cmd.startsWith("setpoint ")) {
        // Cú pháp: sp <roll> <pitch> <yaw>
        int firstSpace = cmd.indexOf(' ');
        int secondSpace = cmd.indexOf(' ', firstSpace + 1);
        int thirdSpace = cmd.indexOf(' ', secondSpace + 1);
        if (secondSpace > 0 && thirdSpace > 0) {
            float r = cmd.substring(firstSpace + 1, secondSpace).toFloat();
            float p = cmd.substring(secondSpace + 1, thirdSpace).toFloat();
            float y = cmd.substring(thirdSpace + 1).toFloat();
            px4Controller.setTargetAttitude(r, p, y);
            Serial.printf("[%s] Cap nhat goc muc tieu (Setpoint): Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f°\n", sourceTag, r, p, y);
        } else {
            Serial.printf("[%s] Cu phap sp loi! Vui long dung: sp <roll> <pitch> <yaw> (Vi du: sp 0 0 90)\n", sourceTag);
        }
    } else if (cmd.startsWith("pid roll ") || cmd.startsWith("PID ROLL ")) {
        float p, i, d;
        if (sscanf(cmd.c_str() + 9, "%f %f %f", &p, &i, &d) == 3) {
            px4Controller.setRatePidRoll(p, i, d);
            Serial.printf("[%s] Cap nhat PID Roll: Kp=%.4f, Ki=%.4f, Kd=%.4f\n", sourceTag, p, i, d);
        }
    } else if (cmd.startsWith("pid pitch ") || cmd.startsWith("PID PITCH ")) {
        float p, i, d;
        if (sscanf(cmd.c_str() + 10, "%f %f %f", &p, &i, &d) == 3) {
            px4Controller.setRatePidPitch(p, i, d);
            Serial.printf("[%s] Cap nhat PID Pitch: Kp=%.4f, Ki=%.4f, Kd=%.4f\n", sourceTag, p, i, d);
        }
    } else if (cmd.startsWith("pid yaw ") || cmd.startsWith("PID YAW ")) {
        float p, i, d;
        if (sscanf(cmd.c_str() + 8, "%f %f %f", &p, &i, &d) == 3) {
            px4Controller.setRatePidYaw(p, i, d);
            Serial.printf("[%s] Cap nhat PID Yaw: Kp=%.4f, Ki=%.4f, Kd=%.4f\n", sourceTag, p, i, d);
        }
    } else if (cmd.startsWith("pid angle ") || cmd.startsWith("PID ANGLE ")) {
        float rp, pp, yp;
        if (sscanf(cmd.c_str() + 10, "%f %f %f", &rp, &pp, &yp) == 3) {
            px4Controller.setAngleGains(rp, pp, yp);
            Serial.printf("[%s] Cap nhat Outer Angle P: Roll_P=%.2f, Pitch_P=%.2f, Yaw_P=%.2f\n", sourceTag, rp, pp, yp);
        }
    } else if (cmd.equalsIgnoreCase("pid")) {
        const PidGains& gr = px4Controller.getRatePidRoll().getGains();
        const PidGains& gp = px4Controller.getRatePidPitch().getGains();
        const PidGains& gy = px4Controller.getRatePidYaw().getGains();
        Serial.println("\n================= THONG SO BO DIEU KHIEN PX4 PID =================");
        Serial.printf("Rate PID Roll  : Kp=%.4f | Ki=%.4f | Kd=%.4f | I_Max=%.2f\n", gr.kp, gr.ki, gr.kd, gr.i_max);
        Serial.printf("Rate PID Pitch : Kp=%.4f | Ki=%.4f | Kd=%.4f | I_Max=%.2f\n", gp.kp, gp.ki, gp.kd, gp.i_max);
        Serial.printf("Rate PID Yaw   : Kp=%.4f | Ki=%.4f | Kd=%.4f | I_Max=%.2f\n", gy.kp, gy.ki, gy.kd, gy.i_max);
        Serial.printf("Target Setpoint: Roll=%.1f° | Pitch=%.1f° | Yaw=%.1f° | Throttle=%.1f%%\n",
                      0.0f, 0.0f, 0.0f, px4Controller.getTargetThrottle());
        Serial.println("==================================================================\n");
    } else if (cmd.equalsIgnoreCase("m stop") || cmd.equalsIgnoreCase("stop all") || cmd.equalsIgnoreCase("test stop") || cmd.equalsIgnoreCase("stop motors")) {
        px4Controller.setTargetThrottle(0.0f);
        for (int i = 1; i <= ESC_NUM_MOTORS; i++) {
            quadMotors.setMotorThrottleUs(i, ESC_PULSE_MIN_US);
        }
        Serial.printf("[%s] [MANUAL] Da dung tat ca 4 dong co ve 0%% (1000 us).\n", sourceTag);
    } else if (cmd.equalsIgnoreCase("manual off") || cmd.equalsIgnoreCase("pid mode") || cmd.equalsIgnoreCase("auto")) {
        px4Controller.setTargetThrottle(0.0f);
        Serial.printf("[%s] [MANUAL] Da tat che do Manual -> San sang che do PID.\n", sourceTag);
    } else if (cmd.startsWith("m1 ") || cmd.startsWith("M1 ") || cmd.startsWith("m0 ") || cmd.startsWith("M0 ") ||
               cmd.startsWith("m2 ") || cmd.startsWith("M2 ") ||
               cmd.startsWith("m3 ") || cmd.startsWith("M3 ") ||
               cmd.startsWith("m4 ") || cmd.startsWith("M4 ")) {
        if (quadMotors.getState() != ESC_STATE_ARMED) {
            quadMotors.arm();
            px4Controller.reset();
            px4Controller.lockCurrentHeading(fusedData.yaw);
            Serial.printf("[%s] [AUTO-ARM] Tu dong ARM de test dong co!\n", sourceTag);
        }
        px4Controller.setTargetThrottle(0.0f); // Tắt ga PID để chế độ test hoàn toàn độc lập

        // Phân tích ID động cơ và mức ga
        int spaceIdx = cmd.indexOf(' ');
        int targetMotor = cmd.substring(1, spaceIdx).toInt();
        if (targetMotor == 0) targetMotor = 1; // Map m0 sang m1
        float pct = cmd.substring(spaceIdx + 1).toFloat();
        if (pct < 0.0f) pct = 0.0f;
        if (pct > 50.0f) pct = 50.0f;

        // KHI TEST 1 ĐỘNG CƠ: ĐỘNG CƠ ĐƯỢC CHỌN QUAY, 3 ĐỘNG CƠ CÒN LẠI ÉP VỀ 0% (1000us - ĐỨNG YÊN)
        for (int i = 1; i <= ESC_NUM_MOTORS; i++) {
            if (i == targetMotor) {
                quadMotors.setMotorThrottlePct(i, pct);
            } else if (pct > 0.0f) {
                quadMotors.setMotorThrottlePct(i, 0.0f);
            }
        }
        Serial.printf("[%s] [MANUAL TEST] Motor %d: %.1f%% | Cac motor khac: 0%% (Dung yen)\n", sourceTag, targetMotor, pct);
    } else if (cmd.startsWith("freq ") || cmd.startsWith("FREQ ")) {
        uint32_t f = (uint32_t)cmd.substring(5).toInt();
        quadMotors.setFrequency(f);
    } else if (cmd.equalsIgnoreCase("calib_start") || cmd.equalsIgnoreCase("calib")) {
        quadMotors.startCalibration();
    } else if (cmd.equalsIgnoreCase("calib_finish") || cmd.equalsIgnoreCase("calib_done")) {
        quadMotors.finishCalibration();
    } else if (cmd.equalsIgnoreCase("status")) {
        Serial.println("\n============== BANG TRANG THAI CHI TIET 4 DONG CO ==============");
        Serial.printf("Trang thai he thong: %s | Tan so PWM: %u Hz\n", quadMotors.getStateStr(), quadMotors.getFrequency());
        Serial.println("-----------------------------------------------------------------");
        for (int i = 0; i < ESC_NUM_MOTORS; i++) {
            const EscMotor& m = quadMotors.getMotor(i);
            Serial.printf("Motor %d | Chân: GPIO %-2d | Chieu quay: %-3s | Xung: %4d us | Ga: %5.1f%%\n",
                          m.getId(), m.getPin(), m.getDirectionStr(), m.getCurrentPulseUs(), m.getThrottlePct());
        }
        Serial.println("=================================================================\n");
    } else if (cmd.equalsIgnoreCase("help") || cmd.equalsIgnoreCase("?")) {
        Serial.println("\n============= DANH SACH LENH CLI CHO QUAD MOTOR & PX4 PID =============");
        Serial.println("  arm                       : Mo khoa 4 ESC va san sang bo dieu khien PID");
        Serial.println("  disarm / stop             : Khoa khan cap (Ha xung ve 900us, reset PID)");
        Serial.println("  throttle <0..100>         : Dat muc ga tong the cho Quadcopter (Vi du: 'throttle 25')");
        Serial.println("  sp <roll> <pitch> <yaw>   : Dat goc nghieng Setpoint (Vi du: 'sp 0 0 0')");
        Serial.println("  pid                       : Xem toan bo thong so he so PID PX4 hien tai");
        Serial.println("  pid roll <p> <i> <d>      : Tinh chinh PID toc do goc Roll");
        Serial.println("  pid pitch <p> <i> <d>     : Tinh chinh PID toc do goc Pitch");
        Serial.println("  pid yaw <p> <i> <d>       : Tinh chinh PID toc do goc Yaw");
        Serial.println("  pid angle <rp> <pp> <yp>  : Tinh chinh he so P vong goc ngoai");
        Serial.println("  m1 / m2 / m3 / m4 <pct>   : Test thu cong rieng le tung dong co");
        Serial.println("  freq <50|400>             : Chon tan so PWM (50Hz ESC Vang, 400Hz PX4)");
        Serial.println("  calib_start / calib_finish: Hieu chuan hanh trinh ga ESC");
        Serial.println("  status                    : Xem chi tiet thong so 4 dong co");
        Serial.println("=======================================================================\n");
    } else {
        Serial.printf("[%s] Khong ro lenh: '%s'. Nhap 'help' de xem danh sach lenh.\n", sourceTag, cmd.c_str());
    }
}

void handleSerialCommands() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    executeCommand(cmd, "SERIAL");
}

void handleWirelessCommands() {
    while (telemetry_wifi_has_command()) {
        String cmd = telemetry_wifi_get_command();
        executeCommand(cmd, "WIFI");
    }
}

// ============================================================================
// 5. MAIN SETUP
// ============================================================================
>>>>>>> ae916b5 ( update base)
void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n[SYSTEM] ESP32-C6 DUAL-IMU FUSION & ITSMC FLIGHT CONTROLLER");

<<<<<<< HEAD
    // 1. Initialize MPU-6050 via I2C (GPIO 14, 15) and calibrate Zero-Bias
=======
    Serial.println("\n==================================================================");
    Serial.println("  ESP32-C6 ALL-IN-ONE FLIGHT CONTROLLER & WIRELESS TELEMETRY");
    Serial.println("  INTEGRATED: DUAL-IMU + PX4 EKF + BUTTERWORTH FILTER + PX4 PID");
    Serial.println("==================================================================");

    // 1. Khởi tạo MPU-6050 qua I2C (GPIO 14, GPIO 15) và đo Zero-Bias
>>>>>>> ae916b5 ( update base)
    if (mpu6050_init(MPU_SDA_PIN, MPU_SCL_PIN)) {
        mpu6050_calibrate_gyro(100);
    }

    // 2. Initialize Xsens MTi-3 via UART1 (RX=GPIO 2, TX=GPIO 3)
    xsens_mti_init(XSENS_RX_PIN, XSENS_TX_PIN, XSENS_BAUDRATE);
    xsens_mti_set_axis_inversion(true, false, true);

    // 3. Configure PX4-EKF weights (Xsens 85%, MPU-6050 15%)
    ekf.setSensorWeights(0.85f, 0.15f);

    // 4. Initialize ESC PWM drivers for 4 Quad-X motors
    quadMotors.init(ESC_DEFAULT_FREQ_HZ);

<<<<<<< HEAD
    // 5. Initialize ITSMC Flight Controller Supervisor
    flightManager.initialize();

    previousLoopMicros = micros();
    Serial.println("[SYSTEM] Initialization complete. Type 'help' for CLI commands.\n");
=======
    // 5. Khởi tạo Bộ điều khiển bay PX4 Multicopter (200Hz Loop)
    px4Controller.init(200.0f);

    // 6. Khởi tạo Access Point WiFi 6, UDP Telemetry Broadcaster & WebSocket Server
    //    SSID: "ESP32-UAV-FC" | Mật khẩu: "dronebridge" | UDP: 14550 | WS: 81
    telemetry_wifi_init("ESP32-UAV-FC", "dronebridge", 14550, 81);

    lastPidUs = micros();

    Serial.println("[SETUP] He thong All-in-One FC & Telemetry da san sang hoat dong!");
    Serial.println("[SETUP] Ket noi WiFi 'ESP32-UAV-FC' de truy cap Web 3D Monitor hoac VIO.\n");
    Serial.println("==================================================================\n");
>>>>>>> ae916b5 ( update base)
}

void loop() {
<<<<<<< HEAD
    uint32_t currentMicros = micros();
    float deltaTimeSec = (currentMicros - previousLoopMicros) * 1.0e-6f;
    if (deltaTimeSec <= 0.0001f || deltaTimeSec > 0.05f) {
        deltaTimeSec = 0.005f; // Safe fallback (200 Hz)
    }
    previousLoopMicros = currentMicros;
=======
    uint32_t nowUs = micros();

    // 1. Đọc cảm biến MPU-6050 qua I2C
    mpu6050_read(mpuData);
>>>>>>> ae916b5 ( update base)

    // 1. Acquire sensor readings
    mpu6050_read(mpuData);
    xsens_mti_update(xsensData);

    // 2. Execute sensor fusion (PX4 EKF)
    ekf.update(mpuData, xsensData, fusedData);

<<<<<<< HEAD
    // 3. Execute ITSMC Flight Control (Attitude & Position loops)
    flightManager.update(fusedData, deltaTimeSec);

    // 4. Update ESC motor PWM state machine & slew limiter
    quadMotors.update();

    // 5. Parse serial CLI user commands
    commandInterpreter.processIncomingCommands(flightManager);

    // 6. Report status and telemetry periodically (20 Hz)
    telemetryReporter.update(millis(), mpuData, xsensData, fusedData, flightManager);
=======
    // 4. CHU TRÌNH ĐIỀU KHIỂN BAY THỜI GIAN THỰC PX4 PID LOOP (200 Hz / 5ms)
    if (nowUs - lastPidUs >= PID_INTERVAL_US) {
        float dt = (float)(nowUs - lastPidUs) * 1e-6f;
        lastPidUs = nowUs;

        bool isArmed = (quadMotors.getState() == ESC_STATE_ARMED);

        // Chạy bộ lọc Butterworth 2nd-order và Cascaded PID (Angle P -> Rate PID -> Mixer)
        px4Controller.update(fusedData.roll, fusedData.pitch, fusedData.yaw,
                             fusedData.gx,   fusedData.gy,    fusedData.gz,
                             dt, isArmed);

        // Chỉ khi đang Armed VÀ có mức ga bay (targetThrottle >= 5%):
        // PID mới tính toán mô-men cân bằng và truyền xung vào 4 động cơ
        if (isArmed && px4Controller.getTargetThrottle() >= 5.0f) {
            const uint16_t* motor_us = px4Controller.getMotorOutputsUs();
            for (int i = 0; i < ESC_NUM_MOTORS; i++) {
                quadMotors.setMotorThrottleUs(i + 1, motor_us[i]);
            }
        }
    }

    // 5. Cập nhật FSM điều tốc OOP cho cả 4 động cơ & Soft Ramp Limiter
    quadMotors.update();

    // 6. Đóng gói & Phát Telemetry không dây qua WiFi (UDP Port 14550 & WebSocket Port 81)
    TelemetryPacket pkt;
    pkt.roll = fusedData.roll;
    pkt.pitch = fusedData.pitch;
    pkt.yaw = fusedData.yaw;
    pkt.ax = fusedData.ax;
    pkt.ay = fusedData.ay;
    pkt.az = fusedData.az;
    pkt.gx = px4Controller.getFilteredGx(); // Xuất tốc độ góc sau lọc Butterworth
    pkt.gy = px4Controller.getFilteredGy();
    pkt.gz = px4Controller.getFilteredGz();
    pkt.z = 0.0f;
    pkt.vz = 0.0f;
    for (int i = 0; i < ESC_NUM_MOTORS; i++) {
        pkt.m_us[i] = quadMotors.getMotor(i).getCurrentPulseUs();
        pkt.m_pct[i] = quadMotors.getMotor(i).getThrottlePct();
    }
    pkt.state_str = quadMotors.getStateStr();
    pkt.frequency_hz = quadMotors.getFrequency();
    telemetry_wifi_update(pkt);

    // 7. Xử lý lệnh điều khiển động cơ từ Serial CLI & WiFi (UDP/WebSocket)
    handleSerialCommands();
    handleWirelessCommands();

    // 8. Bắn dữ liệu định kỳ qua Serial (20 Hz / 50ms)
    uint32_t nowMs = millis();
    if (nowMs - lastPrintMs >= PRINT_INTERVAL_MS) {
        lastPrintMs = nowMs;
        printTelemetry();
    }
>>>>>>> ae916b5 ( update base)
}
