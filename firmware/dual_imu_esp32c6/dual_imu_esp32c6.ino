#include <Arduino.h>
#include "mpu6050.h"
#include "xsens_mti.h"
#include "px4_filters.h"
#include "px4_pid.h"
#include "esc_pwm.h"
#include "telemetry_wifi.h"
#include "pid_autotuner.h"

// ============================================================================
// 1. HARDWARE PIN CONFIGURATION (ESP32-C6)
// ============================================================================
// Cảm biến IMU:
#define MPU_SDA_PIN     14
#define MPU_SCL_PIN     15

#define XSENS_RX_PIN    2
#define XSENS_TX_PIN    3
#define XSENS_BAUDRATE  115200

// Động cơ Quadcopter chuẩn PX4 (Đồng bộ 100% 4 ESC Vàng 30A, 1000..2000 us):
// - Motor 1: GPIO 7 (Front-Left,  CCW - Trái) - ESC Vàng 30A
// - Motor 2: GPIO 6 (Rear-Right,  CCW - Phải) - ESC Vàng 30A
// - Motor 3: GPIO 5 (Rear-Left,   CW  - Trái) - ESC Vàng 30A
// - Motor 4: GPIO 4 (Front-Right, CW  - Phải) - ESC Vàng 30A

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
static uint8_t printPhase = 0;          // In xoay vòng mỗi lần 1 dòng => không dồn ~600 byte vào UART cùng lúc

// Độ "tươi" dữ liệu IMU: quá ngưỡng coi như mất cảm biến (bản cũ dùng dữ liệu Xsens cũ mãi mãi
// khi UART đứt, vì has_orientation không bao giờ bị xóa)
const uint32_t XSENS_STALE_MS = 100;    // Xsens 100 Hz => cho phép mất ~10 gói liên tiếp
const uint32_t SENSOR_LOSS_DISARM_MS = 300; // Mất toàn bộ IMU khi ARMED quá 300 ms => DISARM

// Giới hạn tốc độ tăng ga tổng (collective). Đầu ra PID giờ ghi thẳng ra ESC (không qua
// Soft-Ramp), nên bước nhảy ga lớn từ Web/CLI ("set 60") được làm mềm tại đây.
const float THROTTLE_SLEW_UP_PCT_PER_S = 120.0f;
static float throttleCmdPct = 0.0f;     // Ga đã qua bộ giới hạn tốc độ, đưa vào Mixer

// Chẩn đoán thời gian thực vòng PID
static uint32_t pidCount = 0, pidCountWindowMs = 0, pidMaxGapUs = 0, pidMaxGapWindowUs = 0;
static float pidLoopHz = 0.0f;
static uint8_t imuSource = 0;           // 0 = mất, 1 = Xsens, 2 = MPU6050
static uint32_t sensorLostSinceMs = 0;

// ============================================================================
// 3. SERIAL TELEMETRY PRINTER
// ============================================================================
// In KHÔNG CHẶN: mỗi lần gọi chỉ in 1 dòng và chỉ khi bộ đệm TX còn đủ chỗ.
// Bản cũ in ~600 byte/100 ms bằng Serial.printf liên tiếp: khi bộ đệm UART/USB-CDC đầy,
// printf CHẶN loop() hàng chục ms => vòng PID 200 Hz bị trễ/giật (dt nhảy, D-term nhiễu).
static void serialWriteLineNonBlocking(const char* line, size_t len) {
    if (len == 0) return;
    const int room = Serial.availableForWrite();
    if (room < (int)len) return; // Không đủ chỗ: bỏ dòng này, không chờ (bộ đệm TX 1024 B ở setup)
    Serial.write((const uint8_t*)line, len);
}

void printTelemetry() {
    char line[200];
    int n = 0;
    switch (printPhase) {
        case 0:
            // Định dạng chuẩn [PX4_EKF] mà Web (WebSerial) và vio_system/imu_stream.py đang parse.
            // Bản cũ in tag [ATTITUDE] kèm "(Yaw:..)" chen giữa => cả 2 phía không parse được.
            n = snprintf(line, sizeof(line),
                         "[PX4_EKF] R:%6.1f° P:%6.1f° Y:%6.1f° | Acc:[%6.2f,%6.2f,%6.2f] | Gyr:[%6.1f,%6.1f,%6.1f] (SRC:%s YAW:%s)\n",
                         attitudeState.roll, attitudeState.pitch, attitudeState.yaw,
                         attitudeState.ax, attitudeState.ay, attitudeState.az,
                         px4Controller.getFilteredGx(), px4Controller.getFilteredGy(), px4Controller.getFilteredGz(),
                         imuSource == 1 ? "XSENS" : (imuSource == 2 ? "MPU" : "NONE"),
                         attitudeState.yaw_valid ? "OK" : "INVALID");
            break;
        case 1:
            if (mpuData.ready) {
                n = snprintf(line, sizeof(line),
                             "[MPU6050] R:%6.1f° P:%6.1f° Y:%6.1f° | Acc:[%6.2f,%6.2f,%6.2f] | Gyr:[%6.1f,%6.1f,%6.1f]\n",
                             mpuData.roll, mpuData.pitch, mpuData.yaw,
                             mpuData.ax, mpuData.ay, mpuData.az,
                             mpuData.gx, mpuData.gy, mpuData.gz);
            } else {
                n = snprintf(line, sizeof(line), "[MPU6050] Dang khoi tao hoac mat ket noi I2C...\n");
            }
            break;
        case 2:
            if (xsensData.has_orientation || xsensData.has_accel) {
                n = snprintf(line, sizeof(line),
                             "[Xsens  ] R:%6.1f° P:%6.1f° Y:%6.1f° | Acc:[%6.2f,%6.2f,%6.2f] | Gyr:[%6.1f,%6.1f,%6.1f] age=%lums err=%lu\n",
                             xsensData.roll, xsensData.pitch, xsensData.yaw,
                             xsensData.ax, xsensData.ay, xsensData.az,
                             xsensData.gx, xsensData.gy, xsensData.gz,
                             (unsigned long)(millis() - xsensData.last_orient_ms),
                             (unsigned long)xsensData.checksum_errors);
            } else {
                n = snprintf(line, sizeof(line), "[Xsens  ] Dang cho goi tin Xbus tu UART1...\n");
            }
            break;
        case 3: {
            float p_sp, q_sp, r_sp;
            px4Controller.getRateSetpoints(p_sp, q_sp, r_sp);
            n = snprintf(line, sizeof(line),
                         "[SETPOINT] Att:[R:%.1f P:%.1f Y:%.1f] Thr:%4.1f%%(cmd %4.1f%%) | RateSp:[%.0f,%.0f,%.0f]dps | Loop:%.0fHz max %luus\n",
                         setpointState.roll_sp, setpointState.pitch_sp, setpointState.yaw_sp,
                         setpointState.throttle_pct, throttleCmdPct,
                         p_sp * RAD_TO_DEG, q_sp * RAD_TO_DEG, r_sp * RAD_TO_DEG,
                         pidLoopHz, (unsigned long)pidMaxGapWindowUs);
            break;
        }
        case 4: {
            float tau_r, tau_p, tau_y;
            px4Controller.getTorques(tau_r, tau_p, tau_y);
            const float* u_norm = px4Controller.getNormalizedMotorOutputs();
            n = snprintf(line, sizeof(line),
                         "[ALLOC   ] Tau:[R:%+.3f P:%+.3f Y:%+.3f] I:[%+.3f,%+.3f,%+.3f] | u:[%.2f,%.2f,%.2f,%.2f]\n",
                         tau_r, tau_p, tau_y,
                         px4Controller.getRatePidRoll().getI(), px4Controller.getRatePidPitch().getI(),
                         px4Controller.getRatePidYaw().getI(),
                         u_norm[0], u_norm[1], u_norm[2], u_norm[3]);
            break;
        }
        default:
            // Dòng [ESC_QUAD] là dòng cuối chu kỳ: Web (WebSerial) chốt 1 mẫu tại đây
            n = snprintf(line, sizeof(line),
                         "[ESC_QUAD] State:%-8s | M1(P7-CCW):%4dus(%4.1f%%) | M2(P6-CCW):%4dus(%4.1f%%) | M3(P5-CW):%4dus(%4.1f%%) | M4(P4-CW):%4dus(%4.1f%%) @ %luHz\n",
                         quadMotors.getStateStr(),
                         quadMotors.getMotor(0).getCurrentPulseUs(), quadMotors.getMotor(0).getThrottlePct(),
                         quadMotors.getMotor(1).getCurrentPulseUs(), quadMotors.getMotor(1).getThrottlePct(),
                         quadMotors.getMotor(2).getCurrentPulseUs(), quadMotors.getMotor(2).getThrottlePct(),
                         quadMotors.getMotor(3).getCurrentPulseUs(), quadMotors.getMotor(3).getThrottlePct(),
                         (unsigned long)quadMotors.getFrequency());
            break;
    }
    printPhase = (printPhase + 1) % 6;
    if (n > 0) serialWriteLineNonBlocking(line, (size_t)((n < (int)sizeof(line)) ? n : (int)sizeof(line) - 1));
}

// ============================================================================
// 4. UNIFIED COMMAND ENGINE (HỖ TRỢ SERIAL CLI + WEB GAMEPAD A124 + WEBSOCKET)
// ============================================================================
static const int MANUAL_MIX_ALL = 5; // Lệnh "x m1 m2 m3 m4": điều khiển tay cả 4 motor
static int activeTestMotor = 0; // 0 = PID (ALL 4 MOTORS), 1..4 = Motor riêng lẻ, 5 = tay cả 4
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
                    autoTuner.stop();
                    setpointState.throttle_pct = 0.0f;
                    throttleCmdPct = 0.0f;
                    setpointState.heading_locked = false;
                    Serial.println("[RC] >>> DISARMED!");
                }
            } else {
                if (quadMotors.getState() != ESC_STATE_ARMED) {
                    if (thr > 3.0f) {
                        Serial.printf("[RC] Tu choi ARM: Can ga phai o muc 0%%! (Hien tai: %.1f%%)\n", thr);
                        return;
                    }
                    if (!quadMotors.arm()) {
                        Serial.println("[RC] ARM bi tu choi: PWM chua san sang");
                        return;
                    }
                    px4Controller.reset();
                    activeTestMotor = 0;   // Bay bằng tay cầm => PID là chủ sở hữu đầu ra ESC
                    throttleCmdPct = 0.0f;
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
            // Nhận quyền ghi ESC cho chế độ tay. Bản cũ để activeTestMotor = 0 nên ngay chu kỳ
            // PID kế tiếp (ga = 0) ghi đè IDLE lên cả 4 motor => lệnh "x" không có tác dụng.
            activeTestMotor = MANUAL_MIX_ALL;
            autoTuner.stop();
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
        autoTuner.stop();
        activeTestMotor = 0;
        setpointState.throttle_pct = 0.0f;
        throttleCmdPct = 0.0f;
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
        // Không reset PID ở đây: xóa tích phân giữa lúc bay gây giật (chỉ đổi setpoint)
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
    // 12. Auto-Tuner commands: tune roll/pitch/yaw/angle/stop/apply/save
    } else if (cmd.startsWith("tune ") || cmd.startsWith("TUNE ")) {
        String sub = cmd.substring(5);
        sub.trim();
        sub.toLowerCase();

        if (sub.startsWith("roll")) {
            float amp = 0.15f;
            int spaceIdx = sub.indexOf(' ');
            if (spaceIdx > 0) {
                float val = sub.substring(spaceIdx + 1).toFloat();
                if (val > 0.0f) amp = (val > 1.0f) ? (val / 100.0f) : val;
            }
            amp = constrain(amp, 0.05f, 0.30f);
            autoTuner.configure(amp, 8, 25.0f, 0.04f);
            autoTuner.start(TUNE_ROLL_RATE);
            Serial.printf("[%s] [TUNE] Bat dau tune Roll Rate PID | amp=%.2f (%.0f%%)\n", sourceTag, amp, amp * 100.0f);
        } else if (sub.startsWith("pitch")) {
            float amp = 0.15f;
            int spaceIdx = sub.indexOf(' ');
            if (spaceIdx > 0) {
                float val = sub.substring(spaceIdx + 1).toFloat();
                if (val > 0.0f) amp = (val > 1.0f) ? (val / 100.0f) : val;
            }
            amp = constrain(amp, 0.05f, 0.30f);
            autoTuner.configure(amp, 8, 25.0f, 0.04f);
            autoTuner.start(TUNE_PITCH_RATE);
            Serial.printf("[%s] [TUNE] Bat dau tune Pitch Rate PID | amp=%.2f (%.0f%%)\n", sourceTag, amp, amp * 100.0f);
        } else if (sub.startsWith("yaw")) {
            float amp = 0.10f;
            int spaceIdx = sub.indexOf(' ');
            if (spaceIdx > 0) {
                float val = sub.substring(spaceIdx + 1).toFloat();
                if (val > 0.0f) amp = (val > 1.0f) ? (val / 100.0f) : val;
            }
            amp = constrain(amp, 0.05f, 0.25f);
            autoTuner.configure(amp, 8, 25.0f, 0.04f);
            autoTuner.start(TUNE_YAW_RATE);
            Serial.printf("[%s] [TUNE] Bat dau tune Yaw Rate PID | amp=%.2f (%.0f%%)\n", sourceTag, amp, amp * 100.0f);
        } else if (sub.startsWith("angle")) {
            autoTuner.start(TUNE_ANGLE_P);
            Serial.printf("[%s] [TUNE] Bat dau danh gia Angle P\n", sourceTag);
        } else if (sub.startsWith("stop")) {
            autoTuner.stop();
            Serial.printf("[%s] [TUNE] Da dung tune\n", sourceTag);
        } else if (sub.startsWith("apply")) {
            bool smooth = (sub.indexOf("50") >= 0 || sub.indexOf("smooth") >= 0 || sub.indexOf("half") >= 0);
            if (autoTuner.isDone()) {
                const TuneResult& r = autoTuner.getResult();
                if (r.valid) {
                    TuneAxis ax = autoTuner.getAxis();
                    float kp = smooth ? r.kp_smooth : r.kp;
                    float ki = smooth ? r.ki_smooth : r.ki;
                    float kd = smooth ? r.kd_smooth : r.kd;
                    if (ax == TUNE_ROLL_RATE)  px4Controller.setRatePidRoll(kp, ki, kd);
                    if (ax == TUNE_PITCH_RATE) px4Controller.setRatePidPitch(kp, ki, kd);
                    if (ax == TUNE_YAW_RATE)   px4Controller.setRatePidYaw(kp, ki, kd);
                    if (ax == TUNE_ANGLE_P) {
                        px4Controller.setAngleGains(r.angle_p, r.angle_p, 2.8f);
                    }
                    Serial.printf("[%s] [TUNE] DA AP DUNG (%s) → %s: Kp=%.4f Ki=%.4f Kd=%.5f | Angle_P=%.2f\n",
                                  sourceTag, smooth ? "SMOOTH" : "STANDARD", r.axis_name, kp, ki, kd, r.angle_p);
                }
            } else {
                Serial.printf("[%s] [TUNE] Khong co ket qua hop le de ap dung (chua hoan thanh)\n", sourceTag);
            }
        } else if (sub.startsWith("save")) {
            const TuneResult& r = autoTuner.getResult();
            if (r.valid) {
                Serial.println("\n// ================== PX4_PID.CPP COPY-PASTE GAINS ==================");
                Serial.printf("// Truc: %s (Tu=%.3fs, Ku=%.4f)\n", r.axis_name, r.tu, r.ku);
                Serial.printf("  _rate_pid_%s.init(%.4ff, %.4ff, %.5ff, 0.30f, 20.0f, loop_freq_hz);\n",
                              r.axis_name, r.kp, r.ki, r.kd);
                Serial.printf("// Ban smooth (khuyen nghi neu con rung): init(%.4ff, %.4ff, %.5ff...)\n",
                              r.kp_smooth, r.ki_smooth, r.kd_smooth);
                Serial.printf("  // Outer Angle P goi y: %.2ff\n", r.angle_p);
                Serial.println("// ===================================================================\n");
            }
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
    // Bộ đệm TX lớn để in telemetry không chặn (HardwareSerial & USB-CDC đều hỗ trợ, gọi trước begin)
    Serial.setTxBufferSize(1024);
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
    uint32_t nowMs = millis();

    // 1. Đọc cảm biến thô MPU-6050 qua I2C (~0.5ms)
    mpu6050_read(mpuData);

    // 2. Đọc cảm biến Xsens MTi-3 qua UART1 non-blocking (~0.1ms)
    xsens_mti_update(xsensData);

    // 3. Chọn nguồn trạng thái góc (State Estimator nhẹ):
    //    - Ưu tiên AHRS phần cứng Xsens MTi-3 nếu gói góc còn "tươi" (< 100 ms).
    //    - Gyro: trộn 85% Xsens + 15% MPU chỉ khi CẢ HAI đều có dữ liệu gyro hợp lệ.
    //      (Bản cũ trộn theo cờ has_orientation: nếu Xsens không xuất Rate-of-Turn thì gx=0
    //       => phản hồi vòng trong bị co lại còn 15% => vòng kín gần như hở.)
    //    - Mất Xsens => tự động rơi về MPU6050 (bản cũ dùng góc Xsens đóng băng mãi mãi).
    const bool xsOrientFresh = xsensData.has_orientation && (nowMs - xsensData.last_orient_ms) < XSENS_STALE_MS;
    const bool xsGyroFresh   = xsensData.has_gyro && (nowMs - xsensData.last_gyro_ms) < XSENS_STALE_MS;

    if (xsOrientFresh) {
        attitudeState.roll  = xsensData.roll;
        attitudeState.pitch = xsensData.pitch;
        attitudeState.yaw   = xsensData.yaw;
        if (xsGyroFresh && mpuData.ready) {
            attitudeState.gx = 0.85f * xsensData.gx + 0.15f * mpuData.gx;
            attitudeState.gy = 0.85f * xsensData.gy + 0.15f * mpuData.gy;
            attitudeState.gz = 0.85f * xsensData.gz + 0.15f * mpuData.gz;
        } else if (xsGyroFresh) {
            attitudeState.gx = xsensData.gx;
            attitudeState.gy = xsensData.gy;
            attitudeState.gz = xsensData.gz;
        } else if (mpuData.ready) {
            attitudeState.gx = mpuData.gx;
            attitudeState.gy = mpuData.gy;
            attitudeState.gz = mpuData.gz;
        }
        if (xsensData.has_accel) {
            attitudeState.ax = xsensData.ax;
            attitudeState.ay = xsensData.ay;
            attitudeState.az = xsensData.az;
        } else if (mpuData.ready) {
            attitudeState.ax = mpuData.ax;
            attitudeState.ay = mpuData.ay;
            attitudeState.az = mpuData.az;
        }
        attitudeState.yaw_valid = true;
        attitudeState.sensor_ok = xsGyroFresh || mpuData.ready; // không có gyro => không bay được
        imuSource = attitudeState.sensor_ok ? 1 : 0;
    } else if (mpuData.ready) {
        // Nếu vừa chuyển nguồn Xsens -> MPU khi đang ARMED: yaw không còn hợp lệ
        attitudeState.roll  = mpuData.roll;
        attitudeState.pitch = mpuData.pitch;
        attitudeState.yaw   = 0.0f; // MPU đơn thuần không có Yaw tuyệt đối (Mag/AHRS) => invalid
        attitudeState.gx    = mpuData.gx;
        attitudeState.gy    = mpuData.gy;
        attitudeState.gz    = mpuData.gz;
        attitudeState.ax    = mpuData.ax;
        attitudeState.ay    = mpuData.ay;
        attitudeState.az    = mpuData.az;
        attitudeState.yaw_valid = false;
        attitudeState.sensor_ok = true;
        imuSource = 2;
    } else {
        attitudeState.yaw_valid = false;
        attitudeState.sensor_ok = false;
        imuSource = 0;
    }
    attitudeState.timestamp_us = nowUs;

    // Failsafe mất IMU: không có phản hồi thì không thể ổn định => DISARM sau 300 ms
    if (!attitudeState.sensor_ok) {
        if (sensorLostSinceMs == 0) sensorLostSinceMs = nowMs ? nowMs : 1;
        if (quadMotors.getState() == ESC_STATE_ARMED && activeTestMotor == 0 &&
            (nowMs - sensorLostSinceMs) > SENSOR_LOSS_DISARM_MS) {
            quadMotors.disarm();
            px4Controller.reset();
            autoTuner.stop();
            setpointState.throttle_pct = 0.0f;
            setpointState.rc_stream_active = false;
            throttleCmdPct = 0.0f;
            Serial.println("[FAILSAFE] Mat du lieu IMU > 300ms -> DISARM!");
            telemetry_wifi_broadcast_msg("[FAILSAFE] Mat du lieu IMU -> DISARM");
        }
    } else {
        sensorLostSinceMs = 0;
    }

    // Failsafe RC Watchdog: Mất tín hiệu RC quá 1.5s CHỈ áp dụng khi đang bay bằng luồng RC streaming từ tay cầm
    if (quadMotors.getState() == ESC_STATE_ARMED && setpointState.rc_stream_active &&
        (nowMs - setpointState.last_rc_ms > RC_WATCHDOG_TIMEOUT_MS)) {
        setpointState.throttle_pct = 0.0f;
        setpointState.rc_stream_active = false;
        Serial.println("[FAILSAFE] Mat tin hieu RC Gamepad > 1.5s -> Ha ga ve 0%!");
    }

    // 4. CHU TRÌNH ĐIỀU KHIỂN BAY PX4 CASCADED PID (200 Hz / 5ms)
    //    - Vòng ngoài (Outer Angle P): góc -> tốc độ góc thân mục tiêu (rad/s)
    //    - Vòng trong (Inner Rate PID): tốc độ góc -> mô-men Roll, Pitch, Yaw
    //    - Allocator/Mixer: Throttle + Torque -> lệnh chuẩn hóa u1..u4 [0..1]
    if (nowUs - lastPidUs >= PID_INTERVAL_US) {
        const uint32_t gapUs = nowUs - lastPidUs;
        float dt = (float)gapUs * 1e-6f;
        lastPidUs = nowUs;

        // Thống kê tần số & jitter vòng PID (gửi lên Web để phát hiện loop bị chặn)
        pidCount++;
        if (gapUs > pidMaxGapUs) pidMaxGapUs = gapUs;
        if (nowMs - pidCountWindowMs >= 1000) {
            pidLoopHz = pidCount * 1000.0f / (float)(nowMs - pidCountWindowMs);
            pidMaxGapWindowUs = pidMaxGapUs;
            pidCount = 0;
            pidMaxGapUs = 0;
            pidCountWindowMs = nowMs;
        }

        bool isArmed = (quadMotors.getState() == ESC_STATE_ARMED);

        // Giới hạn tốc độ TĂNG ga (giảm ga thì tức thời, ưu tiên an toàn)
        float thrTarget = isArmed ? setpointState.throttle_pct : 0.0f;
        if (thrTarget > throttleCmdPct) {
            const float maxStep = THROTTLE_SLEW_UP_PCT_PER_S * (dt > 0.05f ? 0.05f : dt);
            throttleCmdPct = (thrTarget - throttleCmdPct > maxStep) ? (throttleCmdPct + maxStep) : thrTarget;
        } else {
            throttleCmdPct = thrTarget;
        }

        // Nạp Setpoint vào bộ điều khiển
        px4Controller.setTargetThrottle(throttleCmdPct);
        px4Controller.setTargetAttitude(setpointState.roll_sp, setpointState.pitch_sp, setpointState.yaw_sp);

        // Chạy bộ điều khiển bay (vòng PID chỉ điều khiển khi PID là chủ sở hữu đầu ra)
        const bool pidOwnsOutput = isArmed && activeTestMotor == 0 && attitudeState.sensor_ok;
        px4Controller.update(attitudeState.roll, attitudeState.pitch, attitudeState.yaw,
                             attitudeState.gx,   attitudeState.gy,    attitudeState.gz,
                             dt, pidOwnsOutput, attitudeState.yaw_valid);

        // AUTO-TUNER (Relay) — chỉ thay mô-men của TRỤC ĐANG TUNE, các trục còn lại vẫn do PID
        // giữ ổn định. Bản cũ đặt mô-men 2 trục còn lại = 0 => drone trôi/lật trục kia khi tune.
        if (autoTuner.isActive()) {
            TuneAxis ax = autoTuner.getAxis();
            float rate_meas = 0.0f;
            // Dùng Gyro ĐÃ LỌC (giống vòng trong) để relay không bị nhiễu rung đảo chiều giả
            if (ax == TUNE_ROLL_RATE || ax == TUNE_ANGLE_P) rate_meas = px4Controller.getFilteredGx() * DEG_TO_RAD;
            if (ax == TUNE_PITCH_RATE) rate_meas = px4Controller.getFilteredGy() * DEG_TO_RAD;
            if (ax == TUNE_YAW_RATE)   rate_meas = px4Controller.getFilteredGz() * DEG_TO_RAD;
            // An toàn: luôn giám sát độ nghiêng lớn nhất Roll/Pitch. Bản cũ dùng |yaw| (hướng la bàn)
            // khi tune Yaw => abort ngay nếu mũi drone không hướng gần 0°.
            const float tilt_abs = fmaxf(fabsf(attitudeState.roll), fabsf(attitudeState.pitch));

            if (!pidOwnsOutput || setpointState.throttle_pct < 2.0f) {
                autoTuner.stop(ABORT_LOW_THROTTLE);
            }

            float relay_torque = autoTuner.update(rate_meas, tilt_abs, dt);

            if (autoTuner.isActive() && pidOwnsOutput && throttleCmdPct >= 2.0f) {
                float tau_r, tau_p, tau_y;
                px4Controller.getTorques(tau_r, tau_p, tau_y);
                if (ax == TUNE_ROLL_RATE || ax == TUNE_ANGLE_P) tau_r = relay_torque;
                if (ax == TUNE_PITCH_RATE) tau_p = px4Controller.isPitchInverted() ? -relay_torque : relay_torque;
                if (ax == TUNE_YAW_RATE)   tau_y = relay_torque;
                float u_tune[4];
                px4Controller.getMixer().mixNormalized(throttleCmdPct / 100.0f, tau_r, tau_p, tau_y, u_tune);
                quadMotors.setAllNormalized(u_tune);
            } else if (pidOwnsOutput) {
                // Tuner vừa kết thúc/abort: trả quyền lại cho PID ngay trong chu kỳ này
                // (bản cũ ép IDLE cả 4 motor khi abort => rơi tự do giữa không trung)
                if (throttleCmdPct < 2.0f) {
                    for (int i = 0; i < ESC_NUM_MOTORS; i++)
                        quadMotors.getMotor(i).setTargetPulseUs(ESC_PULSE_IDLE_US);
                } else {
                    quadMotors.setAllNormalized(px4Controller.getNormalizedMotorOutputs());
                }
            }
        } else if (pidOwnsOutput) {
            // OUTPUT SAFETY GATE (duy nhất 1 Output Owner ghi vào ESC khi ARMED)
            if (throttleCmdPct < 2.0f) {
                for (int i = 0; i < ESC_NUM_MOTORS; i++) {
                    quadMotors.getMotor(i).setTargetPulseUs(ESC_PULSE_IDLE_US);
                }
            } else {
                quadMotors.setAllNormalized(px4Controller.getNormalizedMotorOutputs());
            }
        }
    }

    // 5. Cập nhật FSM điều tốc OOP cho cả 4 động cơ & Soft Ramp Limiter (chế độ tay)
    quadMotors.update();

    // 6. Xử lý lệnh điều khiển nhận từ Web Browser qua WebSocket & Serial CLI
    handleWirelessCommands();
    handleSerialCommands();

    // 7. Đóng gói & Phát Telemetry không dây cho Web 3D HITL Testbed Monitor (50 Hz)
    TelemetryPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.t_ms = nowMs;
    pkt.roll = attitudeState.roll;
    pkt.pitch = attitudeState.pitch;
    pkt.yaw = attitudeState.yaw;
    pkt.ax = attitudeState.ax;
    pkt.ay = attitudeState.ay;
    pkt.az = attitudeState.az;
    pkt.gx = px4Controller.getFilteredGx();   // Gyro đã lọc = đúng tín hiệu vòng trong đang dùng
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

    pkt.sp_roll     = setpointState.roll_sp;
    pkt.sp_pitch    = setpointState.pitch_sp;
    pkt.sp_yaw      = setpointState.yaw_sp;
    pkt.sp_throttle = setpointState.throttle_pct;
    px4Controller.getTorques(pkt.tau[0], pkt.tau[1], pkt.tau[2]);
    const float* u_out = px4Controller.getNormalizedMotorOutputs();
    for (int i = 0; i < 4; i++) {
        pkt.u_norm[i] = u_out[i];
    }
    float p_sp, q_sp, r_sp;
    px4Controller.getRateSetpoints(p_sp, q_sp, r_sp);
    pkt.rate_sp[0] = p_sp * RAD_TO_DEG;
    pkt.rate_sp[1] = q_sp * RAD_TO_DEG;
    pkt.rate_sp[2] = r_sp * RAD_TO_DEG;
    pkt.pid_i[0] = px4Controller.getRatePidRoll().getI();
    pkt.pid_i[1] = px4Controller.getRatePidPitch().getI();
    pkt.pid_i[2] = px4Controller.getRatePidYaw().getI();
    pkt.loop_hz = pidLoopHz;
    pkt.loop_max_us = pidMaxGapWindowUs;
    pkt.imu_src = imuSource;
    pkt.xs_age_ms = xsensData.has_orientation ? (nowMs - xsensData.last_orient_ms) : 0xFFFFFFFFu;
    pkt.xs_err = xsensData.checksum_errors;
    pkt.yaw_valid = attitudeState.yaw_valid;
    pkt.rc_sequence = acceptedRcSequence;
    pkt.pwm_outputs_ready = quadMotors.outputsReady();

    telemetry_wifi_update(pkt);

    // 8. In dữ liệu định kỳ qua Serial (mỗi 100 ms in 6 dòng xoay vòng, không chặn)
    nowMs = millis();
    if (nowMs - lastPrintMs >= PRINT_INTERVAL_MS / 6) {
        lastPrintMs = nowMs;
        printTelemetry();
    }
}
