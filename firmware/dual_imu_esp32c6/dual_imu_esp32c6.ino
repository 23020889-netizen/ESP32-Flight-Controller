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
// - Motor 1: GPIO 7 (Front-Right, CCW) - (Min: 1000 us, Max: 2000 us)
// - Motor 2: GPIO 6 (Rear-Left,   CCW) - (Min: 1000 us, Max: 2000 us)
// - Motor 3: GPIO 5 (Front-Left,  CW ) - (Min: 1000 us, Max: 2000 us)
// - Motor 4: GPIO 4 (Rear-Right,  CW ) - (Min: 1000 us, Max: 2000 us)

// ============================================================================
// 2. GLOBAL MODULE OBJECTS & DATA HOLDERS
// ============================================================================
MpuData   mpuData   = {0};
XsensData xsensData = {0};

// Trạng thái góc và tốc độ góc (Dung hợp nhẹ từ phần cứng AHRS Xsens + MPU6050)
struct DroneAttitudeState {
    float roll;
    float pitch;
    float yaw;
    float ax, ay, az;
    float gx, gy, gz;
    bool  yaw_valid;
    bool  sensor_ok;
    uint32_t timestamp_us;
};
DroneAttitudeState attitudeState = {0};

// Bộ quản lý mục tiêu góc và ga (Setpoint Manager)
struct SetpointState {
    float roll_sp;
    float pitch_sp;
    float yaw_sp;
    float throttle_pct; // 0.0 .. 100.0%
    bool  heading_locked;
    uint32_t last_rc_ms;
    bool  rc_stream_active; // true chỉ khi đang bay bằng luồng RC streaming từ tay cầm
};
SetpointState setpointState = { 0.0f, 0.0f, 0.0f, 0.0f, false, 0, false };

const uint32_t RC_WATCHDOG_TIMEOUT_MS = 1500; // Failsafe timeout khi mất sóng RC quá 1.5s
uint32_t acceptedRcSequence = 0;

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
    Serial.printf("[ATTITUDE] R:%6.1f° P:%6.1f° Y:%6.1f° (Yaw:%s) | Gyr:[%6.1f,%6.1f,%6.1f] (0%% CPU)\n",
                  attitudeState.roll, attitudeState.pitch, attitudeState.yaw,
                  attitudeState.yaw_valid ? "VALID" : "INVALID",
                  attitudeState.gx,   attitudeState.gy,    attitudeState.gz);

    // 4. Trạng thái Setpoint & Allocator
    float tau_r, tau_p, tau_y;
    px4Controller.getTorques(tau_r, tau_p, tau_y);
    const float* u_norm = px4Controller.getNormalizedMotorOutputs();
    Serial.printf("[SETPOINT] Target:[R:%.1f°, P:%.1f°, Y:%.1f°] Thr:%4.1f%% | GyrFilt:[%6.1f,%6.1f,%6.1f]\n",
                  setpointState.roll_sp, setpointState.pitch_sp, setpointState.yaw_sp, setpointState.throttle_pct,
                  px4Controller.getFilteredGx(), px4Controller.getFilteredGy(), px4Controller.getFilteredGz());
    Serial.printf("[ALLOC   ] Tau:[R:%+.2f, P:%+.2f, Y:%+.2f] | Norm u:[%.2f, %.2f, %.2f, %.2f]\n",
                  tau_r, tau_p, tau_y, u_norm[0], u_norm[1], u_norm[2], u_norm[3]);

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
        if (quadMotors.getState() == ESC_STATE_CALIBRATION) {
            return; // Đang trong quá trình hiệu chuẩn ESC, không để gói tin RC làm gián đoạn
        }
        float thr = 0, rsp = 0, psp = 0, ysp = 0;
        int arm = 0;
        if (sscanf(cmd.c_str() + 3, "%f,%f,%f,%f,%d", &thr, &rsp, &psp, &ysp, &arm) == 5) {
            // Kiểm tra tính hữu hạn (chống NaN / Inf do đường truyền hỏng)
            if (isnan(thr) || isinf(thr) || isnan(rsp) || isinf(rsp) ||
                isnan(psp) || isinf(psp) || isnan(ysp) || isinf(ysp)) {
                return;
            }
            // Giới hạn giá trị biên an toàn
            thr = constrain(thr, 0.0f, 100.0f);
            rsp = constrain(rsp, -35.0f, 35.0f);
            psp = constrain(psp, -35.0f, 35.0f);
            ysp = constrain(ysp, -180.0f, 180.0f);

            if (arm == 0) {
                setpointState.rc_stream_active = false;
                if (quadMotors.getState() == ESC_STATE_ARMED) {
                    quadMotors.disarm();
                    px4Controller.reset();
                    setpointState.throttle_pct = 0.0f;
                    setpointState.heading_locked = false;
                    Serial.println("[RC] >>> DISARMED!");
                }
            } else {
                if (quadMotors.getState() != ESC_STATE_ARMED) {
                    if (thr > 3.0f) {
                        Serial.printf("[RC] Tu choi ARM: Can ga phai o muc 0%%! (Hien tai: %.1f%%)\n", thr);
                        return;
                    }
                    quadMotors.arm();
                    px4Controller.reset();
                    // Khóa hướng Yaw hiện tại đúng 1 lần khi chuyển sang ARMED
                    setpointState.yaw_sp = attitudeState.yaw;
                    setpointState.heading_locked = true;
                    setpointState.throttle_pct = 0.0f;
                    Serial.printf("[RC] >>> ARMED! Latch Heading = %.1f°\n", setpointState.yaw_sp);
                }
                float maxCap = safetyLimit60Pct ? 60.0f : 100.0f;
                if (thr > maxCap) thr = maxCap;
                setpointState.throttle_pct = thr;
                setpointState.roll_sp = rsp;
                setpointState.pitch_sp = psp;

                // Xử lý góc Yaw:
                // Nếu người dùng chủ động gạt cần xoay (|ysp| > 1.0°): dịch chuyển mốc Yaw đặt
                // Nếu người dùng nhả cần về giữa (|ysp| <= 1.0°): GIỮ NGUYÊN mốc Yaw đã khóa!
                if (fabsf(ysp) > 1.0f) {
                    setpointState.yaw_sp += ysp * 0.04f; // Tốc độ xoay thân theo độ gạt cần
                    while (setpointState.yaw_sp > 180.0f)  setpointState.yaw_sp -= 360.0f;
                    while (setpointState.yaw_sp < -180.0f) setpointState.yaw_sp += 360.0f;
                }
            }
            setpointState.last_rc_ms = millis();
            setpointState.rc_stream_active = (arm == 1);
            ++acceptedRcSequence;
            quadMotors.feedWatchdog();
            return;
        }
    }

    // 2. Gói tin nhanh 4 kênh: "x <m1%> <m2%> <m3%> <m4%>"
    if (cmd.startsWith("x ") || cmd.startsWith("X ")) {
        float p1 = 0, p2 = 0, p3 = 0, p4 = 0;
        if (sscanf(cmd.c_str() + 2, "%f %f %f %f", &p1, &p2, &p3, &p4) == 4) {
            if (quadMotors.getState() != ESC_STATE_ARMED) {
                if (!quadMotors.arm()) return;
            }
            setpointState.throttle_pct = 0.0f;
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

    // 3. Lệnh ARM & Hiệu chuẩn
    if (cmd.equalsIgnoreCase("arm") || cmd.equalsIgnoreCase("l") || cmd.equalsIgnoreCase("calib_finish") || cmd.equalsIgnoreCase("calib_done")) {
        if (quadMotors.getState() == ESC_STATE_CALIBRATION) {
            quadMotors.finishCalibration();
        } else {
            if (!quadMotors.arm()) {
                snprintf(logBuf, sizeof(logBuf), "[%s] ARM bi tu choi: kiem tra attach PWM ca 4 chan GPIO 7/6/5/4", sourceTag);
                telemetry_wifi_broadcast_msg(logBuf);
                return;
            }
        }
        px4Controller.reset();
        setpointState.roll_sp = 0.0f;
        setpointState.pitch_sp = 0.0f;
        setpointState.yaw_sp = attitudeState.yaw;
        setpointState.heading_locked = true;
        snprintf(logBuf, sizeof(logBuf), "[%s] Quad Armed | Heading Locked: %.1f deg", sourceTag, attitudeState.yaw);
        telemetry_wifi_broadcast_msg(logBuf);
        Serial.printf("[%s] >>> DA ARM HOAN TAT! (Ca 4 motor = 1000us) | Khoa Yaw=%.1f°\n", sourceTag, attitudeState.yaw);
    }
    else if (cmd.equalsIgnoreCase("calib_start") || cmd.equalsIgnoreCase("h")) {
        quadMotors.startCalibration();
    }
    // 4. Lệnh DISARM / STOP
    else if (cmd.equalsIgnoreCase("disarm") || cmd.equalsIgnoreCase("stop") || cmd.equalsIgnoreCase("kill") || cmd.equalsIgnoreCase("s")) {
        quadMotors.disarm();
        px4Controller.reset();
        activeTestMotor = 0;
        setpointState.throttle_pct = 0.0f;
        setpointState.heading_locked = false;
        setpointState.rc_stream_active = false;
        snprintf(logBuf, sizeof(logBuf), "[%s] Quad Disarmed & Motors Safe", sourceTag);
        telemetry_wifi_broadcast_msg(logBuf);
        Serial.printf("[%s] >>> DA DISARM! Toan bo 4 dong co ngat ve 0%% an toan.\n", sourceTag);
    }
    // 5. Lệnh giới hạn trần ga 60%
    else if (cmd.equalsIgnoreCase("limit")) {
        safetyLimit60Pct = !safetyLimit60Pct;
        Serial.printf("[%s] [LIMIT] Gioi han ga toi da: %s\n", sourceTag, safetyLimit60Pct ? "60% (An toan)" : "100% (Toan dai)");
    }
    // 6. Quản lý động cơ test (all hoặc m1..m4, m1 <pct>..m4 <pct>)
    else if (cmd.equalsIgnoreCase("all")) {
        activeTestMotor = 0;
        Serial.printf("[%s] [SELECT] Da chon CA 4 DONG CO (ALL).\n", sourceTag);
    }
    else if ((cmd.charAt(0) == 'm' || cmd.charAt(0) == 'M') && cmd.charAt(1) >= '1' && cmd.charAt(1) <= '4') {
        int targetMotor = cmd.charAt(1) - '0';
        if (cmd.length() == 2) {
            activeTestMotor = targetMotor;
            setpointState.throttle_pct = 0.0f;
            for (int i = 1; i <= ESC_NUM_MOTORS; i++) {
                quadMotors.setMotorThrottlePct(i, 0.0f);
            }
            Serial.printf("[%s] [SELECT] Da chon MOTOR %d.\n", sourceTag, activeTestMotor);
        } else if (cmd.charAt(2) == ' ') {
            if (quadMotors.getState() != ESC_STATE_ARMED) {
                if (!quadMotors.arm()) return;
                px4Controller.reset();
                setpointState.yaw_sp = attitudeState.yaw;
                setpointState.heading_locked = true;
            }
            setpointState.throttle_pct = 0.0f;
            px4Controller.setTargetThrottle(0.0f);
            activeTestMotor = targetMotor;

            float pct = cmd.substring(3).toFloat();
            float maxCap = safetyLimit60Pct ? 60.0f : 100.0f;
            pct = constrain(pct, 0.0f, maxCap);

            for (int i = 1; i <= ESC_NUM_MOTORS; i++) {
                quadMotors.setMotorThrottlePct(i, (i == targetMotor) ? pct : 0.0f);
            }
            Serial.printf("[%s] [MANUAL TEST] Motor %d: %.1f%% | Cac motor khac: 0%%\n", sourceTag, targetMotor, pct);
        }
    }
    // 7. Đặt ga tổng thể (Throttle)
    else if (cmd.startsWith("throttle ") || cmd.startsWith("THROTTLE ") || cmd.startsWith("set ") || cmd.startsWith("SET ")) {
        int idx = cmd.indexOf(' ');
        float pct = cmd.substring(idx + 1).toFloat();
        if (safetyLimit60Pct && pct > 60.0f) pct = 60.0f;
        pct = constrain(pct, 0.0f, 100.0f);
        if (activeTestMotor >= 1 && activeTestMotor <= 4) {
            if (quadMotors.getState() != ESC_STATE_ARMED && !quadMotors.arm()) return;
            setpointState.throttle_pct = 0.0f;
            for (int i = 1; i <= ESC_NUM_MOTORS; i++) {
                quadMotors.setMotorThrottlePct(i, (i == activeTestMotor) ? pct : 0.0f);
            }
            Serial.printf("[%s] Dat ga Motor %d: %.1f%%\n", sourceTag, activeTestMotor, pct);
        } else {
            if (pct > 0.0f && quadMotors.getState() != ESC_STATE_ARMED) {
                quadMotors.arm();
                px4Controller.reset();
                setpointState.yaw_sp = attitudeState.yaw;
                setpointState.heading_locked = true;
            }
            setpointState.throttle_pct = pct;
            setpointState.rc_stream_active = false; // Chế độ đặt ga tĩnh từ Web UI / CLI: tuân thủ setpoint, không bị watchdog ngắt
            quadMotors.feedWatchdog();
            Serial.printf("[%s] Dat muc ga tong the: %.1f%%\n", sourceTag, pct);
        }
    }
    // 8. Đảo chiều trục Pitch
    else if (cmd.equalsIgnoreCase("invert pitch") || cmd.equalsIgnoreCase("inv pitch")) {
        px4Controller.togglePitchInversion();
        Serial.printf("[%s] DAO CHIEU TRUC PITCH: Invert = %s\n", sourceTag, px4Controller.isPitchInverted() ? "TRUE (NGUOC)" : "FALSE (THUAN)");
    }
    // 9. Đặt góc cân bằng
    else if (cmd.equalsIgnoreCase("level") || cmd.equalsIgnoreCase("balance")) {
        setpointState.roll_sp = 0.0f;
        setpointState.pitch_sp = 0.0f;
        setpointState.yaw_sp = attitudeState.yaw;
        setpointState.heading_locked = true;
        px4Controller.reset();
        Serial.printf("[%s] >>> CAN BANG: Dat Roll=0.0°, Pitch=0.0°, Giu nguyen Yaw=%.1f°\n", sourceTag, attitudeState.yaw);
    }
    // 10. Đặt Setpoint góc: sp <roll> <pitch> <yaw>
    else if (cmd.startsWith("sp ") || cmd.startsWith("SP ") || cmd.startsWith("setpoint ")) {
        int firstSpace = cmd.indexOf(' ');
        int secondSpace = cmd.indexOf(' ', firstSpace + 1);
        int thirdSpace = cmd.indexOf(' ', secondSpace + 1);
        if (secondSpace > 0 && thirdSpace > 0) {
            float r = cmd.substring(firstSpace + 1, secondSpace).toFloat();
            float p = cmd.substring(secondSpace + 1, thirdSpace).toFloat();
            float y = cmd.substring(thirdSpace + 1).toFloat();
            setpointState.roll_sp = constrain(r, -35.0f, 35.0f);
            setpointState.pitch_sp = constrain(p, -35.0f, 35.0f);
            setpointState.yaw_sp = constrain(y, -180.0f, 180.0f);
            setpointState.heading_locked = true;
            Serial.printf("[%s] Cap nhat Setpoint: Roll=%.1f°, Pitch=%.1f°, Yaw=%.1f°\n", sourceTag, setpointState.roll_sp, setpointState.pitch_sp, setpointState.yaw_sp);
        }
    }
    // 11. Điều chỉnh PID
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
                      setpointState.roll_sp, setpointState.pitch_sp, setpointState.yaw_sp, setpointState.throttle_pct);
        Serial.println("==================================================================\n");
    } else if (cmd.equalsIgnoreCase("m stop") || cmd.equalsIgnoreCase("stop all") || cmd.equalsIgnoreCase("test stop") || cmd.equalsIgnoreCase("stop motors")) {
        setpointState.throttle_pct = 0.0f;
        px4Controller.setTargetThrottle(0.0f);
        for (int i = 1; i <= ESC_NUM_MOTORS; i++) {
            quadMotors.setMotorThrottlePct(i, 0.0f);
        }
        Serial.printf("[%s] [MANUAL] Da dung tat ca 4 dong co ve 0%% (Ca 4 motor = 1000us).\n", sourceTag);
    } else if (cmd.equalsIgnoreCase("manual off") || cmd.equalsIgnoreCase("pid mode") || cmd.equalsIgnoreCase("auto")) {
        activeTestMotor = 0;
        px4Controller.reset();
        setpointState.roll_sp = attitudeState.roll;
        setpointState.pitch_sp = attitudeState.pitch;
        setpointState.yaw_sp = attitudeState.yaw;
        setpointState.heading_locked = true;
        Serial.printf("[%s] [PID MODE] Da BAT che do PID Tu Can Bang (Khoa moc goc hien tai R=%.1f, P=%.1f, Y=%.1f).\n",
                      sourceTag, attitudeState.roll, attitudeState.pitch, attitudeState.yaw);
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
    Serial.println("  - UNIFORM ESC: Ca 4 motor M1..M4 deu 1000-2000us");
    Serial.println("==================================================================");

    // 1. Khởi tạo MPU-6050 qua I2C (GPIO 14, GPIO 15) và đo Zero-Bias
    if (mpu6050_init(MPU_SDA_PIN, MPU_SCL_PIN)) {
        mpu6050_calibrate_gyro(100);
    }

    // 2. Khởi tạo Xsens MTi-3 qua UART1 (RX=GPIO 2, TX=GPIO 3)
    xsens_mti_init(XSENS_RX_PIN, XSENS_TX_PIN, XSENS_BAUDRATE);
    xsens_mti_set_axis_inversion(true, false, true);

    // 3. Khởi tạo hệ thống OOP điều khiển 4 động cơ Quadcopter:
    //    M1: GPIO 7 (CCW) - (Min: 1000 us, Max: 2000 us)
    //    M2: GPIO 6 (CCW) - (Min: 1000 us, Max: 2000 us)
    //    M3: GPIO 5 (CW ) - (Min: 1000 us, Max: 2000 us)
    //    M4: GPIO 4 (CW ) - (Min: 1000 us, Max: 2000 us)
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

    // 3. Dung hợp trạng thái góc siêu nhẹ (State Estimator):
    //    Tận dụng bộ xử lý AHRS phần cứng có sẵn trên Xsens MTi-3 (100Hz)
    //    Kết hợp Gyro MPU6050 -> 0% CPU OVERHEAD cho ESP32!
    if (xsensData.has_orientation) {
        attitudeState.roll  = xsensData.roll;
        attitudeState.pitch = xsensData.pitch;
        attitudeState.yaw   = xsensData.yaw;
        attitudeState.gx    = 0.85f * xsensData.gx + 0.15f * mpuData.gx;
        attitudeState.gy    = 0.85f * xsensData.gy + 0.15f * mpuData.gy;
        attitudeState.gz    = 0.85f * xsensData.gz + 0.15f * mpuData.gz;
        attitudeState.ax    = xsensData.ax;
        attitudeState.ay    = xsensData.ay;
        attitudeState.az    = xsensData.az;
        attitudeState.yaw_valid = true;
        attitudeState.sensor_ok = true;
    } else if (mpuData.ready) {
        attitudeState.roll  = mpuData.roll;
        attitudeState.pitch = mpuData.pitch;
        attitudeState.yaw   = 0.0f; // MPU đơn thuần không có Yaw từ Mag/AHRS, ghi rõ invalid
        attitudeState.gx    = mpuData.gx;
        attitudeState.gy    = mpuData.gy;
        attitudeState.gz    = mpuData.gz;
        attitudeState.ax    = mpuData.ax;
        attitudeState.ay    = mpuData.ay;
        attitudeState.az    = mpuData.az;
        attitudeState.yaw_valid = false;
        attitudeState.sensor_ok = true;
    } else {
        attitudeState.yaw_valid = false;
        attitudeState.sensor_ok = false;
    }
    attitudeState.timestamp_us = nowUs;

    // Failsafe RC Watchdog: Mất tín hiệu RC quá 1.5s CHỈ áp dụng khi đang bay bằng luồng RC streaming từ tay cầm
    uint32_t nowMs = millis();
    if (quadMotors.getState() == ESC_STATE_ARMED && setpointState.rc_stream_active &&
        (nowMs - setpointState.last_rc_ms > RC_WATCHDOG_TIMEOUT_MS)) {
        setpointState.throttle_pct = 0.0f;
        setpointState.rc_stream_active = false;
        Serial.println("[FAILSAFE] Mat tin hieu RC Gamepad > 1.5s -> Ha ga ve 0%!");
    }

    // 4. CHU TRÌNH ĐIỀU KHIỂN BAY THỜI GIAN THỰC PX4 CASCADED PID (200 Hz / 5ms)
    //    - Vòng ngoài (Outer Angle Loop): Tính tốc độ góc mục tiêu (rad/s)
    //    - Vòng trong (Inner Rate PID): Tính mô-men Roll, Pitch, Yaw
    //    - Allocator/Mixer: Phối trộn Throttle + Torque ra lệnh chuẩn hóa u1..u4 [0..1]
    if (nowUs - lastPidUs >= PID_INTERVAL_US) {
        float dt = (float)(nowUs - lastPidUs) * 1e-6f;
        lastPidUs = nowUs;

        bool isArmed = (quadMotors.getState() == ESC_STATE_ARMED);

        // Nạp Setpoint vào bộ điều khiển
        px4Controller.setTargetThrottle(setpointState.throttle_pct);
        px4Controller.setTargetAttitude(setpointState.roll_sp, setpointState.pitch_sp, setpointState.yaw_sp);

        // Chạy bộ điều khiển bay
        px4Controller.update(attitudeState.roll, attitudeState.pitch, attitudeState.yaw,
                             attitudeState.gx,   attitudeState.gy,    attitudeState.gz,
                             dt, isArmed, attitudeState.yaw_valid);

        // OUTPUT SAFETY GATE (Đảm bảo duy nhất 1 Output Owner ghi vào ESC khi ARMED):
        if (isArmed && activeTestMotor == 0) {
            if (setpointState.throttle_pct < 2.0f) {
                // Ga < 2%: Cả 4 động cơ quay chậm không tải (Idle Spin)
                for (int i = 0; i < ESC_NUM_MOTORS; i++) {
                    quadMotors.getMotor(i).setTargetPulseUs(ESC_PULSE_IDLE_US);
                }
            } else {
                // Ga >= 2%: Nhận trực tiếp lệnh chuẩn hóa u1..u4 [0..1] từ Allocator/Mixer
                // Cả bốn ESC dùng chung dải xung 1000..2000 us.
                const float* u_norm = px4Controller.getNormalizedMotorOutputs();
                quadMotors.setAllNormalized(u_norm);
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
    pkt.ax = attitudeState.ax;
    pkt.ay = attitudeState.ay;
    pkt.az = attitudeState.az;
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

    // Telemetry bổ sung theo kiến trúc tái cấu trúc
    pkt.sp_roll     = setpointState.roll_sp;
    pkt.sp_pitch    = setpointState.pitch_sp;
    pkt.sp_yaw      = setpointState.yaw_sp;
    pkt.sp_throttle = setpointState.throttle_pct;
    px4Controller.getTorques(pkt.tau[0], pkt.tau[1], pkt.tau[2]);
    const float* u_out = px4Controller.getNormalizedMotorOutputs();
    for (int i = 0; i < 4; i++) {
        pkt.u_norm[i] = u_out[i];
    }
    pkt.yaw_valid = attitudeState.yaw_valid;
    pkt.rc_sequence = acceptedRcSequence;
    pkt.pwm_outputs_ready = quadMotors.outputsReady();

    telemetry_wifi_update(pkt);

    // 8. In dữ liệu định kỳ qua Serial (10 Hz / 100ms)
    nowMs = millis();
    if (nowMs - lastPrintMs >= PRINT_INTERVAL_MS) {
        lastPrintMs = nowMs;
        printTelemetry();
    }
}
