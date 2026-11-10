#include <Arduino.h>
#include "mpu6050.h"
#include "xsens_mti.h"
#include "px4_filters.h"
#include "px4_pid.h"
#include "esc_pwm.h"
#include "telemetry_wifi.h"

// ============================================================================
// 1. HARDWARE PIN CONFIGURATION (ESP32-C6)
// ============================================================================
// Cảm biến IMU:
#define MPU_SDA_PIN     14
#define MPU_SCL_PIN     15

#define XSENS_RX_PIN    2
#define XSENS_TX_PIN    3
#define XSENS_BAUDRATE  115200

// Động cơ Quadcopter chuẩn PX4 (Đã định nghĩa trong esc_pwm.h):
// - Motor 1: GPIO 7 (Front-Right, CCW) - ESC Vàng 30A (Min: 1000 us, Max: 2000 us)
// - Motor 2: GPIO 6 (Rear-Left,   CCW) - ESC Vàng 30A (Min: 1000 us, Max: 2000 us)
// - Motor 3: GPIO 5 (Front-Left,  CW ) - ESC Đen  40A (Min:  850 us, Max: 1850 us)
// - Motor 4: GPIO 4 (Rear-Right,  CW ) - ESC Vàng 30A (Min: 1000 us, Max: 2000 us)

// ============================================================================
// 2. GLOBAL MODULE OBJECTS & DATA HOLDERS
// ============================================================================
MpuData   mpuData   = {0};
XsensData xsensData = {0};

// Trạng thái góc và tốc độ góc (Dung hợp nhẹ trực tiếp từ phần cứng AHRS Xsens + MPU6050)
struct DroneAttitudeState {
    float roll;
    float pitch;
    float yaw;
    float ax, ay, az;
    float gx, gy, gz;
};
DroneAttitudeState attitudeState = {0};

// Chu kỳ vòng lặp PID và Serial Telemetry
uint32_t lastPidUs = 0;
const uint32_t PID_INTERVAL_US = 5000; // 200 Hz (5ms) chu kỳ chuẩn PX4

uint32_t lastPrintMs = 0;
const uint32_t PRINT_INTERVAL_MS = 100; // 10 Hz update cho Serial Telemetry (tránh nghẽn UART)

// ============================================================================
// 3. SERIAL TELEMETRY PRINTER
// ============================================================================
void printTelemetry() {
    Serial.println("-----------------------------------------------------------------------------------------------------------------");
    
    // 1. Dữ liệu MPU-6050 (I2C)
    if (mpuData.ready) {
        Serial.printf("[MPU6050] Acc:[%6.2f,%6.2f,%6.2f] | Gyr:[%6.1f,%6.1f,%6.1f]\n",
                      mpuData.ax, mpuData.ay, mpuData.az,
                      mpuData.gx, mpuData.gy, mpuData.gz);
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

    // 3. Góc ước lượng thực tế (Xsens AHRS + MPU6050)
    Serial.printf("[ATTITUDE] R:%6.1f° P:%6.1f° Y:%6.1f° | Gyr:[%6.1f,%6.1f,%6.1f] (0%% CPU OVERHEAD)\n",
                  attitudeState.roll, attitudeState.pitch, attitudeState.yaw,
                  attitudeState.gx,   attitudeState.gy,    attitudeState.gz);

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
// 4. UNIFIED COMMAND ENGINE (HỖ TRỢ SERIAL CLI + WEB GAMEPAD A124 + WEBSOCKET)
// ============================================================================
static int activeTestMotor = 0; // 0 = ALL 4 MOTORS, 1..4 = Motor riêng lẻ
static bool safetyLimit60Pct = false;

void executeCommand(String cmd, const char* sourceTag) {
    cmd.trim();
    if (cmd.length() == 0) return;

    char logBuf[128];

    // 1. Gói tin điều khiển RC từ Gamepad A124 trên Web: RC,<thr>,<roll>,<pitch>,<yaw>,<arm>
    if (cmd.startsWith("RC,") || cmd.startsWith("rc,")) {
        float thr = 0, rsp = 0, psp = 0, ysp = 0;
        int arm = 0;
        if (sscanf(cmd.c_str() + 3, "%f,%f,%f,%f,%d", &thr, &rsp, &psp, &ysp, &arm) >= 1) {
            if (arm == 0) {
                if (quadMotors.getState() == ESC_STATE_ARMED) {
                    quadMotors.disarm();
                    px4Controller.reset();
                    px4Controller.setTargetThrottle(0.0f);
                    Serial.println("[RC] >>> DISARMED!");
                }
            } else {
                if (quadMotors.getState() != ESC_STATE_ARMED) {
                    quadMotors.arm();
                    px4Controller.reset();
                    px4Controller.lockCurrentHeading(attitudeState.yaw);
                    Serial.println("[RC] >>> ARMED!");
                }
                float maxCap = safetyLimit60Pct ? 60.0f : 100.0f;
                if (thr > maxCap) thr = maxCap;
                px4Controller.setTargetThrottle(thr);
                px4Controller.setTargetAttitude(rsp, psp, ysp);
            }
            return;
        }
    }

    // 2. Gói tin dạng <Throttle,Roll,Pitch,Yaw,Arm> (1000..2000us)
    if (cmd.startsWith("<") && cmd.endsWith(">")) {
        String inner = cmd.substring(1, cmd.length() - 1);
        int t = 1000, r = 0, p = 0, y = 0, arm = 0;
        if (sscanf(inner.c_str(), "%d,%d,%d,%d,%d", &t, &r, &p, &y, &arm) == 5) {
            if (arm == 0 || t <= 1040) {
                quadMotors.disarm();
                px4Controller.setTargetThrottle(0.0f);
            } else {
                if (quadMotors.getState() != ESC_STATE_ARMED) {
                    quadMotors.arm();
                    px4Controller.reset();
                    px4Controller.lockCurrentHeading(attitudeState.yaw);
                }
                float thr_pct = constrain((float)(t - 1000) / 10.0f, 0.0f, 100.0f);
                float roll_sp = constrain((float)r * 0.1f, -35.0f, 35.0f);
                float pitch_sp = constrain((float)p * 0.1f, -35.0f, 35.0f);
                float yaw_sp = constrain((float)y * 0.1f, -180.0f, 180.0f);
                px4Controller.setTargetThrottle(thr_pct);
                px4Controller.setTargetAttitude(roll_sp, pitch_sp, yaw_sp);
            }
            return;
        }
    }

    // 3. Gói tin nhanh 4 kênh từ script Xbox: "x <m1%> <m2%> <m3%> <m4%>"
    if (cmd.startsWith("x ") || cmd.startsWith("X ")) {
        float p1 = 0, p2 = 0, p3 = 0, p4 = 0;
        if (sscanf(cmd.c_str() + 2, "%f %f %f %f", &p1, &p2, &p3, &p4) == 4) {
            if (quadMotors.getState() != ESC_STATE_ARMED) {
                quadMotors.arm();
            }
            px4Controller.setTargetThrottle(0.0f);
            float maxCap = safetyLimit60Pct ? 60.0f : 100.0f;
            if (p1 > maxCap) p1 = maxCap;
            if (p2 > maxCap) p2 = maxCap;
            if (p3 > maxCap) p3 = maxCap;
            if (p4 > maxCap) p4 = maxCap;
            quadMotors.setMotorThrottlePct(1, p1);
            quadMotors.setMotorThrottlePct(2, p2);
            quadMotors.setMotorThrottlePct(3, p3);
            quadMotors.setMotorThrottlePct(4, p4);
            return;
        }
    }

    // 4. Lệnh ARM
    if (cmd.equalsIgnoreCase("arm") || cmd.equalsIgnoreCase("l") || cmd.equalsIgnoreCase("cal_min") || cmd.equalsIgnoreCase("calib_finish") || cmd.equalsIgnoreCase("calib_done")) {
        if (quadMotors.getState() == ESC_STATE_CALIBRATION) {
            quadMotors.finishCalibration();
        } else {
            quadMotors.arm();
        }
        px4Controller.reset();
        px4Controller.lockCurrentHeading(attitudeState.yaw);
        snprintf(logBuf, sizeof(logBuf), "[%s] Quad Armed | Heading Locked: %.1f deg", sourceTag, attitudeState.yaw);
        telemetry_wifi_broadcast_msg(logBuf);
        Serial.printf("[%s] >>> DA ARM HOAN TAT! (M1,2,4=1000us | M3=850us) | Khoa Yaw=%.1f°\n", sourceTag, attitudeState.yaw);
    } 
    // 5. Lệnh DISARM / STOP
    else if (cmd.equalsIgnoreCase("disarm") || cmd.equalsIgnoreCase("stop") || cmd.equalsIgnoreCase("kill") || cmd.equalsIgnoreCase("s")) {
        quadMotors.disarm();
        px4Controller.reset();
        px4Controller.setTargetThrottle(0.0f);
        snprintf(logBuf, sizeof(logBuf), "[%s] Quad Disarmed & Motors Safe", sourceTag);
        telemetry_wifi_broadcast_msg(logBuf);
        Serial.printf("[%s] >>> DA DISARM! Toan bo 4 dong co ngat ve 0%% an toan.\n", sourceTag);
    } 
    // 6. Lệnh giới hạn trần ga 60%
    else if (cmd.equalsIgnoreCase("limit")) {
        safetyLimit60Pct = !safetyLimit60Pct;
        Serial.printf("[%s] [LIMIT] Gioi han ga toi da: %s\n", sourceTag, safetyLimit60Pct ? "60% (An toan)" : "100% (Toan dai)");
    } 
    // 7. Chọn động cơ test
    else if (cmd.equalsIgnoreCase("all")) {
        activeTestMotor = 0;
        Serial.printf("[%s] [SELECT] Da chon CA 4 DONG CO (ALL).\n", sourceTag);
    } else if (cmd.equalsIgnoreCase("m1") || cmd.equalsIgnoreCase("m2") || cmd.equalsIgnoreCase("m3") || cmd.equalsIgnoreCase("m4")) {
        activeTestMotor = cmd.substring(1).toInt();
        px4Controller.setTargetThrottle(0.0f);
        for (int i = 1; i <= ESC_NUM_MOTORS; i++) {
            quadMotors.setMotorThrottlePct(i, 0.0f);
        }
        Serial.printf("[%s] [SELECT] Da chon MOTOR %d.\n", sourceTag, activeTestMotor);
    } 
    // 8. Đặt ga tổng thể (Throttle)
    else if (cmd.startsWith("throttle ") || cmd.startsWith("THROTTLE ") || cmd.startsWith("set ") || cmd.startsWith("SET ")) {
        int idx = cmd.indexOf(' ');
        float pct = cmd.substring(idx + 1).toFloat();
        if (safetyLimit60Pct && pct > 60.0f) pct = 60.0f;
        if (activeTestMotor >= 1 && activeTestMotor <= 4) {
            if (quadMotors.getState() != ESC_STATE_ARMED) quadMotors.arm();
            px4Controller.setTargetThrottle(0.0f);
            for (int i = 1; i <= ESC_NUM_MOTORS; i++) {
                quadMotors.setMotorThrottlePct(i, (i == activeTestMotor) ? pct : 0.0f);
            }
            Serial.printf("[%s] Dat ga Motor %d: %.1f%%\n", sourceTag, activeTestMotor, pct);
        } else {
            if (pct > 0.0f && quadMotors.getState() != ESC_STATE_ARMED) {
                quadMotors.arm();
                px4Controller.reset();
                px4Controller.lockCurrentHeading(attitudeState.yaw);
            }
            px4Controller.setTargetThrottle(pct);
            Serial.printf("[%s] Dat muc ga tong the PX4: %.1f%%\n", sourceTag, pct);
        }
    } 
    // 9. Đảo chiều trục Pitch
    else if (cmd.equalsIgnoreCase("invert pitch") || cmd.equalsIgnoreCase("inv pitch")) {
        px4Controller.togglePitchInversion();
        Serial.printf("[%s] DAO CHIEU TRUC PITCH: Invert = %s\n", sourceTag, px4Controller.isPitchInverted() ? "TRUE (NGUOC)" : "FALSE (THUAN)");
    } 
    // 10. Đặt góc cân bằng
    else if (cmd.equalsIgnoreCase("level") || cmd.equalsIgnoreCase("balance")) {
        px4Controller.setTargetAttitude(0.0f, 0.0f, attitudeState.yaw);
        px4Controller.reset();
        Serial.printf("[%s] >>> CAN BANG: Dat Roll=0.0°, Pitch=0.0°, Giu nguyen Yaw=%.1f°\n", sourceTag, attitudeState.yaw);
    } 
    // 11. Đặt Setpoint góc: sp <roll> <pitch> <yaw>
    else if (cmd.startsWith("sp ") || cmd.startsWith("SP ") || cmd.startsWith("setpoint ")) {
        int firstSpace = cmd.indexOf(' ');
        int secondSpace = cmd.indexOf(' ', firstSpace + 1);
        int thirdSpace = cmd.indexOf(' ', secondSpace + 1);
        if (secondSpace > 0 && thirdSpace > 0) {
            float r = cmd.substring(firstSpace + 1, secondSpace).toFloat();
            float p = cmd.substring(secondSpace + 1, thirdSpace).toFloat();
            float y = cmd.substring(thirdSpace + 1).toFloat();
            px4Controller.setTargetAttitude(r, p, y);
            Serial.printf("[%s] Cap nhat Setpoint: Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f°\n", sourceTag, r, p, y);
        }
    } 
    // 12. Điều chỉnh PID
    else if (cmd.startsWith("pid roll ") || cmd.startsWith("PID ROLL ")) {
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
            quadMotors.setMotorThrottlePct(i, 0.0f);
        }
        Serial.printf("[%s] [MANUAL] Da dung tat ca 4 dong co ve 0%% (M1,2,4=1000us | M3=850us).\n", sourceTag);
    } else if (cmd.startsWith("m1 ") || cmd.startsWith("M1 ") || cmd.startsWith("m0 ") || cmd.startsWith("M0 ") ||
               cmd.startsWith("m2 ") || cmd.startsWith("M2 ") ||
               cmd.startsWith("m3 ") || cmd.startsWith("M3 ") ||
               cmd.startsWith("m4 ") || cmd.startsWith("M4 ")) {
        if (quadMotors.getState() != ESC_STATE_ARMED) {
            quadMotors.arm();
            px4Controller.reset();
            px4Controller.lockCurrentHeading(attitudeState.yaw);
        }
        px4Controller.setTargetThrottle(0.0f);

        int spaceIdx = cmd.indexOf(' ');
        int targetMotor = cmd.substring(1, spaceIdx).toInt();
        if (targetMotor == 0) targetMotor = 1;
        float pct = cmd.substring(spaceIdx + 1).toFloat();
        if (pct < 0.0f) pct = 0.0f;
        if (pct > 60.0f && safetyLimit60Pct) pct = 60.0f;
        if (pct > 100.0f) pct = 100.0f;

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
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n==================================================================");
    Serial.println("  ESP32-C6 FLIGHT CONTROLLER: DIRECT WEB & GAMEPAD CONTROL");
    Serial.println("  - WIFI: AUTO-CONNECT 'VNU-IS TTDBCLKT' + SOFTAP BACKUP");
    Serial.println("  - WEBSOCKET: DIRECT CONTROL (NO PYTHON / TERMINAL NEEDED!)");
    Serial.println("  - MODE: CASCADED PID STABILIZATION (ANGLE P -> RATE PID)");
    Serial.println("  - MIXED ESC: M1,2,4=1000-2000us | M3=850-1850us");
    Serial.println("==================================================================");

    // 1. Khởi tạo MPU-6050 qua I2C (GPIO 14, GPIO 15) và đo Zero-Bias
    if (mpu6050_init(MPU_SDA_PIN, MPU_SCL_PIN)) {
        mpu6050_calibrate_gyro(100);
    }

    // 2. Khởi tạo Xsens MTi-3 qua UART1 (RX=GPIO 2, TX=GPIO 3)
    xsens_mti_init(XSENS_RX_PIN, XSENS_TX_PIN, XSENS_BAUDRATE);
    xsens_mti_set_axis_inversion(true, false, true);

    // 3. Khởi tạo hệ thống OOP điều khiển 4 động cơ Quadcopter:
    //    M1: GPIO 7 (CCW) - ESC Vàng 30A (Min: 1000 us, Max: 2000 us)
    //    M2: GPIO 6 (CCW) - ESC Vàng 30A (Min: 1000 us, Max: 2000 us)
    //    M3: GPIO 5 (CW ) - ESC Đen  40A (Min:  850 us, Max: 1850 us)
    //    M4: GPIO 4 (CW ) - ESC Vàng 30A (Min: 1000 us, Max: 2000 us)
    quadMotors.init(ESC_DEFAULT_FREQ_HZ);

    // 4. Khởi tạo Bộ điều khiển bay PX4 Multicopter Cascaded PID (200Hz Loop)
    px4Controller.init(200.0f);

    // 5. Khởi tạo WiFi: Tự động kết nối mạng TTDB phòng Lab ("VNU-IS TTDBCLKT" / "Ttdbclkt9")
    //    Đồng thời phát SoftAP "ESP32-UAV-FC" (192.168.4.1) và mở WebSocket Port 81
    telemetry_wifi_init("VNU-IS TTDBCLKT", "Ttdbclkt9", "ESP32-UAV-FC", "dronebridge", 81);

    lastPidUs = micros();

    Serial.println("[SETUP] ESP32 da san sang! Mo trinh duyet Web de bay.");
    Serial.println("==================================================================\n");
}

// ============================================================================
// 6. MAIN LOOP (REAL-TIME DETERMINISTIC EXECUTION)
// ============================================================================
void loop() {
    uint32_t nowUs = micros();

    // 1. Đọc cảm biến thô MPU-6050 qua I2C (~0.5ms)
    mpu6050_read(mpuData);

    // 2. Đọc cảm biến Xsens MTi-3 qua UART1 non-blocking (~0.1ms)
    xsens_mti_update(xsensData);

    // 3. Dung hợp trạng thái góc siêu nhẹ:
    //    Tận dụng bộ xử lý AHRS phần cứng có sẵn trên Xsens MTi-3 (100Hz)
    //    Kết hợp Gyro MPU6050 -> 0% CPU OVERHEAD cho ESP32!
    if (xsensData.has_orientation) {
        attitudeState.roll  = xsensData.roll;
        attitudeState.pitch = xsensData.pitch;
        attitudeState.yaw   = xsensData.yaw;
        attitudeState.gx    = 0.85f * xsensData.gx + 0.15f * mpuData.gx;
        attitudeState.gy    = 0.85f * xsensData.gy + 0.15f * mpuData.gy;
        attitudeState.gz    = 0.85f * xsensData.gz + 0.15f * mpuData.gz;
    } else if (mpuData.ready) {
        attitudeState.roll  = mpuData.roll;
        attitudeState.pitch = mpuData.pitch;
        attitudeState.gx    = mpuData.gx;
        attitudeState.gy    = mpuData.gy;
        attitudeState.gz    = mpuData.gz;
    }

    // 4. CHU TRÌNH ĐIỀU KHIỂN BAY THỜI GIAN THỰC PX4 CASCADED PID (200 Hz / 5ms)
    //    - Vòng ngoài (Outer Angle Loop): Tính tốc độ góc mục tiêu (rad/s)
    //    - Vòng trong (Inner Rate PID): Tính mô-men Roll, Pitch, Yaw
    //    - Mixer: Phối trộn Throttle + Torque ra 4 động cơ
    if (nowUs - lastPidUs >= PID_INTERVAL_US) {
        float dt = (float)(nowUs - lastPidUs) * 1e-6f;
        lastPidUs = nowUs;

        bool isArmed = (quadMotors.getState() == ESC_STATE_ARMED);

        px4Controller.update(attitudeState.roll, attitudeState.pitch, attitudeState.yaw,
                             attitudeState.gx,   attitudeState.gy,    attitudeState.gz,
                             dt, isArmed);

        // Khi Armed VÀ có mức ga bay (targetThrottle >= 5%):
        // PID tính toán mô-men thăng bằng và xuất ra 4 động cơ
        if (isArmed && px4Controller.getTargetThrottle() >= 5.0f) {
            const uint16_t* motor_us = px4Controller.getMotorOutputsUs();
            for (int i = 0; i < ESC_NUM_MOTORS; i++) {
                // Tự động bù dải xung riêng cho M3 (850..1850us) và M1,2,4 (1000..2000us)
                quadMotors.setMotorThrottleUs(i + 1, motor_us[i]);
            }
        } else if (isArmed) {
            // Ga < 5%: Cả 4 động cơ quay chậm không tải (Idle Spin)
            for (int i = 0; i < ESC_NUM_MOTORS; i++) {
                quadMotors.setMotorThrottleUs(i + 1, ESC_PULSE_IDLE_US);
            }
        } else {
            // Disarmed: Cả 4 động cơ ngắt về min pulse an toàn
            for (int i = 0; i < ESC_NUM_MOTORS; i++) {
                quadMotors.setMotorThrottleUs(i + 1, quadMotors.getMotor(i).getMinPulseUs());
            }
        }
    }

    // 5. Cập nhật FSM điều tốc OOP cho cả 4 động cơ & Soft Ramp Limiter
    quadMotors.update();

    // 6. Xử lý lệnh điều khiển nhận từ Web Browser qua WebSocket & Serial CLI
    handleWirelessCommands();
    handleSerialCommands();

    // 7. Đóng gói & Phát Telemetry không dây cho Web 3D HITL Testbed Monitor (25 Hz)
    TelemetryPacket pkt;
    pkt.roll = attitudeState.roll;
    pkt.pitch = attitudeState.pitch;
    pkt.yaw = attitudeState.yaw;
    pkt.ax = mpuData.ax;
    pkt.ay = mpuData.ay;
    pkt.az = mpuData.az;
    pkt.gx = px4Controller.getFilteredGx();
    pkt.gy = px4Controller.getFilteredGy();
    pkt.gz = px4Controller.getFilteredGz();

    pkt.mpu_roll  = mpuData.roll;
    pkt.mpu_pitch = mpuData.pitch;
    pkt.mpu_yaw   = mpuData.yaw;

    pkt.xs_roll   = xsensData.roll;
    pkt.xs_pitch  = xsensData.pitch;
    pkt.xs_yaw    = xsensData.yaw;
    pkt.z = 0.0f;
    pkt.vz = 0.0f;
    for (int i = 0; i < ESC_NUM_MOTORS; i++) {
        pkt.m_us[i] = quadMotors.getMotor(i).getCurrentPulseUs();
        pkt.m_pct[i] = quadMotors.getMotor(i).getThrottlePct();
    }
    pkt.state_str = quadMotors.getStateStr();
    pkt.frequency_hz = quadMotors.getFrequency();
    telemetry_wifi_update(pkt);

    // 8. In dữ liệu định kỳ qua Serial (10 Hz / 100ms)
    uint32_t nowMs = millis();
    if (nowMs - lastPrintMs >= PRINT_INTERVAL_MS) {
        lastPrintMs = nowMs;
        printTelemetry();
    }
}
