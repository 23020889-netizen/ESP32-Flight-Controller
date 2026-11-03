#include <Arduino.h>
#include "mpu6050.h"
#include "xsens_mti.h"
#include "px4_ekf.h"
#include "esc_pwm.h"

// ============================================================================
// 1. HARDWARE PIN CONFIGURATION (ESP32-C6)
// ============================================================================
// Cảm biến IMU
#define MPU_SDA_PIN     14
#define MPU_SCL_PIN     15

#define XSENS_RX_PIN    2
#define XSENS_TX_PIN    3
#define XSENS_BAUDRATE  115200

// Động cơ Quadcopter chuẩn PX4 (Đã định nghĩa trong esc_pwm.h):
// Motor 1: GPIO 7 (CCW - Ngược chiều kim đồng hồ)
// Motor 2: GPIO 6 (CCW - Ngược chiều kim đồng hồ)
// Motor 3: GPIO 5 (CW  - Thuận chiều kim đồng hồ)
// Motor 4: GPIO 4 (CW  - Thuận chiều kim đồng hồ)

// ============================================================================
// 2. GLOBAL MODULE OBJECTS & DATA HOLDERS
// ============================================================================
MpuData   mpuData   = {0};
XsensData xsensData = {0};
FusedData fusedData = {0};

PX4_EKF ekf;

uint32_t lastPrintMs = 0;
const uint32_t PRINT_INTERVAL_MS = 50; // 20 Hz update

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

    // 4. Trạng thái điều khiển 4 Động cơ Quadcopter chuẩn PX4 (GPIO 7, 6, 5, 4)
    Serial.printf("[ESC_QUAD] State:%-8s | M1(P7-CCW):%4dus(%4.1f%%) | M2(P6-CCW):%4dus(%4.1f%%) | M3(P5-CW):%4dus(%4.1f%%) | M4(P4-CW):%4dus(%4.1f%%) @ %dHz\n",
                  quadMotors.getStateStr(),
                  quadMotors.getMotor(0).getCurrentPulseUs(), quadMotors.getMotor(0).getThrottlePct(),
                  quadMotors.getMotor(1).getCurrentPulseUs(), quadMotors.getMotor(1).getThrottlePct(),
                  quadMotors.getMotor(2).getCurrentPulseUs(), quadMotors.getMotor(2).getThrottlePct(),
                  quadMotors.getMotor(3).getCurrentPulseUs(), quadMotors.getMotor(3).getThrottlePct(),
                  quadMotors.getFrequency());
}

// ============================================================================
// 4. NON-BLOCKING SERIAL COMMAND PARSER (INTERACTIVE QUAD MOTOR CLI)
// ============================================================================
void handleSerialCommands() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd.equalsIgnoreCase("arm")) {
        quadMotors.arm();
    } else if (cmd.equalsIgnoreCase("disarm") || cmd.equalsIgnoreCase("stop") || cmd.equalsIgnoreCase("kill")) {
        quadMotors.disarm();
    } else if (cmd.startsWith("set all ") || cmd.startsWith("SET ALL ")) {
        float pct = cmd.substring(8).toFloat();
        if (quadMotors.setAllThrottlePct(pct)) {
            Serial.printf("[CMD] Dat ga dong loat 4 dong co: %.1f%%\n", pct);
        }
    } else if (cmd.startsWith("set ") || cmd.startsWith("SET ")) {
        float pct = cmd.substring(4).toFloat();
        if (quadMotors.setAllThrottlePct(pct)) {
            Serial.printf("[CMD] Dat ga dong loat 4 dong co: %.1f%%\n", pct);
        }
    } else if (cmd.startsWith("m1 ") || cmd.startsWith("M1 ")) {
        float pct = cmd.substring(3).toFloat();
        if (quadMotors.setMotorThrottlePct(1, pct)) {
            Serial.printf("[CMD] Dat ga Motor 1 (GPIO 7 - CCW): %.1f%%\n", pct);
        }
    } else if (cmd.startsWith("m2 ") || cmd.startsWith("M2 ")) {
        float pct = cmd.substring(3).toFloat();
        if (quadMotors.setMotorThrottlePct(2, pct)) {
            Serial.printf("[CMD] Dat ga Motor 2 (GPIO 6 - CCW): %.1f%%\n", pct);
        }
    } else if (cmd.startsWith("m3 ") || cmd.startsWith("M3 ")) {
        float pct = cmd.substring(3).toFloat();
        if (quadMotors.setMotorThrottlePct(3, pct)) {
            Serial.printf("[CMD] Dat ga Motor 3 (GPIO 5 - CW): %.1f%%\n", pct);
        }
    } else if (cmd.startsWith("m4 ") || cmd.startsWith("M4 ")) {
        float pct = cmd.substring(3).toFloat();
        if (quadMotors.setMotorThrottlePct(4, pct)) {
            Serial.printf("[CMD] Dat ga Motor 4 (GPIO 4 - CW): %.1f%%\n", pct);
        }
    } else if (cmd.startsWith("pwm all ") || cmd.startsWith("PWM ALL ")) {
        uint16_t us = (uint16_t)cmd.substring(8).toInt();
        if (quadMotors.setAllThrottleUs(us)) {
            Serial.printf("[CMD] Dat xung dong loat 4 dong co: %u us\n", us);
        }
    } else if (cmd.startsWith("pwm ") || cmd.startsWith("PWM ")) {
        uint16_t us = (uint16_t)cmd.substring(4).toInt();
        if (quadMotors.setAllThrottleUs(us)) {
            Serial.printf("[CMD] Dat xung dong loat 4 dong co: %u us\n", us);
        }
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
        Serial.println("\n============= DANH SACH LENH SERIAL CLI CHO QUAD MOTOR =============");
        Serial.println("  arm           : Mo khoa an toan dong loat 4 ESC (Arming chu ky 2s)");
        Serial.println("  disarm / stop : Khoa khan cap ca 4 dong co (Ngat ga ve 900us)");
        Serial.println("  set <0..100>  : Dat ga dong loat ca 4 dong co (Vi du: 'set 15')");
        Serial.println("  m1 <0..100>   : Dat ga rieng Motor 1 (GPIO 7 - CCW, vi du: 'm1 10')");
        Serial.println("  m2 <0..100>   : Dat ga rieng Motor 2 (GPIO 6 - CCW, vi du: 'm2 15')");
        Serial.println("  m3 <0..100>   : Dat ga rieng Motor 3 (GPIO 5 - CW,  vi du: 'm3 20')");
        Serial.println("  m4 <0..100>   : Dat ga rieng Motor 4 (GPIO 4 - CW,  vi du: 'm4 10')");
        Serial.println("  pwm <us>      : Dat xung dong loat 4 dong co (1000us - 2000us)");
        Serial.println("  freq <50|400> : Chon tan so PWM (50Hz tuong thich cao, hoac 400Hz PX4)");
        Serial.println("  calib_start   : Bat che do hieu chuan ga ca 4 ESC (Phat xung Max 2000us)");
        Serial.println("  calib_finish  : Hoan tat hieu chuan (Ha xung ve Min 900us)");
        Serial.println("  status        : Xem chi tiet thong so va chieu quay 4 dong co");
        Serial.println("====================================================================\n");
    } else {
        Serial.printf("[CMD] Khong ro lenh: '%s'. Nhap 'help' de xem danh sach lenh.\n", cmd.c_str());
    }
}

// ============================================================================
// 5. MAIN SETUP
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n==================================================================");
    Serial.println("  ESP32-C6 DUAL-IMU FUSION & QUAD ESC PWM MOTOR CONTROLLER (OOP)");
    Serial.println("==================================================================");

    // 1. Khởi tạo MPU-6050 qua I2C (GPIO 14, GPIO 15) và đo Zero-Bias
    if (mpu6050_init(MPU_SDA_PIN, MPU_SCL_PIN)) {
        mpu6050_calibrate_gyro(100);
    }

    // 2. Khởi tạo Xsens MTi-3 qua UART1 (RX=GPIO 2, TX=GPIO 3)
    xsens_mti_init(XSENS_RX_PIN, XSENS_TX_PIN, XSENS_BAUDRATE);
    
    // Căn chỉnh trục Xsens khớp tuyệt đối với MPU-6050 (Invert X=true, Invert Z=true)
    xsens_mti_set_axis_inversion(true, false, true);

    // 3. Cấu hình trọng số tin cậy EKF chuẩn PX4: Xsens chiếm 85%, MPU6050 chiếm 15%
    ekf.setSensorWeights(0.85f, 0.15f);

    // 4. Khởi tạo hệ thống OOP điều khiển 4 động cơ Quadcopter:
    //    M1: GPIO 7 (CCW) | M2: GPIO 6 (CCW) | M3: GPIO 5 (CW) | M4: GPIO 4 (CW)
    quadMotors.init(ESC_DEFAULT_FREQ_HZ);

    Serial.println("[SETUP] He thong da khoi tao thanh cong! Bat dau chu trinh...");
    Serial.println("[SETUP] Nhap 'help' vao Serial Monitor de xem lenh dieu khien 4 motor.\n");
    Serial.println("==================================================================\n");
}

// ============================================================================
// 6. MAIN LOOP
// ============================================================================
void loop() {
    // 1. Đọc cảm biến MPU-6050 qua I2C
    mpu6050_read(mpuData);

    // 2. Đọc cảm biến Xsens MTi-3 qua UART1 (Non-blocking FSM)
    xsens_mti_update(xsensData);

    // 3. Dung hợp dữ liệu đa cảm biến bằng bộ lọc EKF chuẩn PX4 (Yaw 100% Xsens MTi-3)
    ekf.update(mpuData, xsensData, fusedData);

    // 4. Cập nhật FSM điều tốc OOP cho cả 4 động cơ & Soft Ramp Limiter
    quadMotors.update();

    // 5. Xử lý lệnh điều khiển động cơ từ Serial CLI
    handleSerialCommands();

    // 6. Bắn dữ liệu định kỳ qua Serial (20 Hz / 50ms)
    uint32_t nowMs = millis();
    if (nowMs - lastPrintMs >= PRINT_INTERVAL_MS) {
        lastPrintMs = nowMs;
        printTelemetry();
    }
}
