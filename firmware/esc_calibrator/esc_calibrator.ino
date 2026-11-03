/**
 * ============================================================================
 * ESP32-C6 ESC CALIBRATION & MOTOR PHASE DIAGNOSTIC TOOL
 * Dự án: ISILAB UAV Flight Controller
 * ============================================================================
 * Chức năng:
 * 1. Hiệu chuẩn hành trình ga ESC (Throttle Range Calibration: 2000us -> 1000us).
 * 2. Chẩn đoán hiện tượng "MẤT PHA" (Motor Stuttering / Phase Loss Diagnostic).
 * 3. Kiểm tra riêng biệt từng động cơ M1 (GPIO 7), M2 (GPIO 6), M3 (GPIO 5), M4 (GPIO 4).
 * ============================================================================
 */

#include <Arduino.h>

// Định nghĩa chân PWM động cơ (Chuẩn Quadcopter trên bo ESP32-C6)
#define MOTOR1_PIN  7   // M1 (CCW)
#define MOTOR2_PIN  6   // M2 (CCW)
#define MOTOR3_PIN  5   // M3 (CW)
#define MOTOR4_PIN  4   // M4 (CW)

#define PWM_FREQ_HZ         50       // 50Hz (Chu kỳ 20000us) tương thích 100% mọi ESC BLDC
#define PWM_RES_BITS        14       // Độ phân giải 14-bit (0 .. 16383)
#define PWM_MAX_DUTY        16383

#define PULSE_MAX_US        2000     // Xung ga tối đa (Full Throttle)
#define PULSE_MIN_US        1000     // Xung ga tối thiểu (Zero Throttle)
#define PULSE_SAFE_DISARM   900      // Xung ngắt ga an toàn chuẩn PX4 (900us)

const uint8_t motorPins[4] = {MOTOR1_PIN, MOTOR2_PIN, MOTOR3_PIN, MOTOR4_PIN};

// Chuyển đổi microsecond sang duty cycle 14-bit @ 50Hz
uint32_t usToDuty(uint16_t pulse_us) {
    uint32_t period_us = 1000000UL / PWM_FREQ_HZ;
    if (pulse_us > period_us) pulse_us = period_us;
    return (uint32_t)(((uint64_t)pulse_us * PWM_MAX_DUTY) / period_us);
}

void writeMotorPulse(uint8_t motorIdx, uint16_t pulse_us) {
    if (motorIdx >= 4) return;
    ledcWrite(motorPins[motorIdx], usToDuty(pulse_us));
}

void writeAllMotors(uint16_t pulse_us) {
    for (int i = 0; i < 4; i++) {
        writeMotorPulse(i, pulse_us);
    }
}

void printHelp() {
    Serial.println(F("\n================= MENU CHẨN ĐOÁN & CALIBRATE ESC ================="));
    Serial.println(F("  1. calib          : BẮT ĐẦU quy trình hiệu chuẩn ga (Phát 2000us)"));
    Serial.println(F("  2. done           : HOÀN TẤT hiệu chuẩn ga (Hạ xung về 1000us/900us)"));
    Serial.println(F("  3. test <1-4>     : Test quay thử motor cụ thể ở mức ga thấp (1060us)"));
    Serial.println(F("  4. diag <1-4>     : Chẩn đoán MẤT PHA (Quét mịn từ 1000us lên 1150us)"));
    Serial.println(F("  5. set <1-4> <us> : Đặt xung cụ thể cho động cơ (ví dụ: 'set 1 1100')"));
    Serial.println(F("  6. stop           : Dừng khẩn cấp toàn bộ động cơ (xuất 900us)"));
    Serial.println(F("  7. status         : Xem cấu hình chân GPIO của 4 động cơ"));
    Serial.println(F("==================================================================\n"));
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println(F("\n=================================================================="));
    Serial.println(F("  ESP32-C6 ESC CALIBRATION & MOTOR PHASE LOSS DIAGNOSTIC TOOL"));
    Serial.println(F("=================================================================="));
    Serial.println(F("⚠️  CẢNH BÁO AN TOÀN:"));
    Serial.println(F("   - HÃY THÁO CÁNH QUẠT (PROPELLER) TRƯỚC KHI THỰC HIỆN BẤT KỲ THAO TÁC NÀO!"));
    Serial.println(F("   - CHƯA CẮM PIN LIPO VÀO ESC CHO ĐẾN KHI CÓ HƯỚNG DẪN TRÊN MÀN HÌNH."));
    Serial.println(F("=================================================================="));

    // Khởi tạo PWM phần cứng LEDC trên ESP32-C6
    for (int i = 0; i < 4; i++) {
        ledcAttach(motorPins[i], PWM_FREQ_HZ, PWM_RES_BITS);
        writeMotorPulse(i, PULSE_SAFE_DISARM);
    }

    printHelp();
}

void loop() {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd.equalsIgnoreCase("calib") || cmd.equalsIgnoreCase("calib_start")) {
        Serial.println(F("\n>>> [BƯỚC 1]: Đang phát xung MAX THROTTLE (2000us) trên cả 4 chân GPIO..."));
        writeAllMotors(PULSE_MAX_US);
        Serial.println(F(">>> [BƯỚC 2]: BÂY GIỜ HÃY CẮM NGUỒN PIN LiPo VÀO ESC!"));
        Serial.println(F(">>> [BƯỚC 3]: Lắng nghe tiếng bíp từ ESC:"));
        Serial.println(F("              - ESC kêu nhạc khởi động"));
        Serial.println(F("              - Ngay sau đó kêu 2 tiếng bíp ngắn 'Bíp - Bíp'"));
        Serial.println(F(">>> [BƯỚC 4]: Ngay khi nghe 2 tiếng bíp, gõ lệnh 'done' và nhấn Enter!"));
    }
    else if (cmd.equalsIgnoreCase("done") || cmd.equalsIgnoreCase("calib_done") || cmd.equalsIgnoreCase("finish")) {
        Serial.println(F("\n>>> [BƯỚC 5]: Đã hạ xung toàn bộ động cơ về MIN THROTTLE (1000us)..."));
        writeAllMotors(PULSE_MIN_US);
        delay(1500);
        writeAllMotors(PULSE_SAFE_DISARM);
        Serial.println(F(">>> [BƯỚC 6]: ESC sẽ phát 1 tiếng bíp dài xác nhận đã lưu dải ga vào EEPROM."));
        Serial.println(F(">>> ĐÃ HOÀN TẤT HIỆU CHUẨN GA! Động cơ đã về trạng thái an toàn (900us)."));
    }
    else if (cmd.startsWith("test ") || cmd.startsWith("TEST ")) {
        int m = cmd.substring(5).toInt();
        if (m < 1 || m > 4) {
            Serial.println(F("[LỖI] Chọn động cơ từ 1 đến 4 (Ví dụ: 'test 1')"));
            return;
        }
        uint8_t idx = m - 1;
        Serial.printf(">>> Đang kiểm tra Motor %d (GPIO %d) ở mức ga nhàn rỗi (1060us)...\n", m, motorPins[idx]);
        writeMotorPulse(idx, 1060);
        delay(3000);
        writeMotorPulse(idx, PULSE_SAFE_DISARM);
        Serial.printf(">>> Đã dừng Motor %d an toàn.\n", m);
    }
    else if (cmd.startsWith("diag ") || cmd.startsWith("DIAG ")) {
        int m = cmd.substring(5).toInt();
        if (m < 1 || m > 4) {
            Serial.println(F("[LỖI] Chọn động cơ từ 1 đến 4 (Ví dụ: 'diag 1')"));
            return;
        }
        uint8_t idx = m - 1;
        Serial.println(F("\n=================================================================="));
        Serial.printf(">>> CHẨN ĐOÁN MẤT PHA CHO MOTOR %d (GPIO %d)\n", m, motorPins[idx]);
        Serial.println(F(">>> Quá trình quét xung mịn từ 1000us -> 1150us trong 5 giây..."));
        Serial.println(F(">>> HÃY QUAN SÁT TRỤC ĐỘNG CƠ:"));
        Serial.println(F("    - NẾU MOTOR BÌNH THƯỜNG: Trục bắt đầu quay êm từ ~1040us và tăng tốc dần."));
        Serial.println(F("    - NẾU BỊ MẤT PHA: Trục KHÔNG THỂ QUAY, chỉ rung giật tới lui,"));
        Serial.println(F("      phát tiếng rè rè gằn gằn và nóng nhanh cuộn dây!"));
        Serial.println(F("=================================================================="));

        for (uint16_t pulse = 1000; pulse <= 1150; pulse += 5) {
            writeMotorPulse(idx, pulse);
            Serial.printf("  Motor %d | Xung: %4d us\n", m, pulse);
            delay(150);
        }
        delay(1000);
        writeMotorPulse(idx, PULSE_SAFE_DISARM);
        Serial.println(F(">>> Kết thúc kiểm tra chẩn đoán. Đã ngắt ga về 900us."));
    }
    else if (cmd.startsWith("set ") || cmd.startsWith("SET ")) {
        // Cú pháp: set <motor_id 1-4> <pulse_us 900-2000>
        int spaceIdx = cmd.indexOf(' ', 4);
        if (spaceIdx == -1) {
            Serial.println(F("[LỖI] Cú pháp: set <1-4> <xung_us> (Ví dụ: 'set 1 1080')"));
            return;
        }
        int m = cmd.substring(4, spaceIdx).toInt();
        int pulse = cmd.substring(spaceIdx + 1).toInt();
        if (m < 1 || m > 4 || pulse < 900 || pulse > 2000) {
            Serial.println(F("[LỖI] Động cơ phải từ 1-4, xung phải từ 900us đến 2000us!"));
            return;
        }
        writeMotorPulse(m - 1, (uint16_t)pulse);
        Serial.printf("[CMD] Motor %d (GPIO %d) -> %d us\n", m, motorPins[m - 1], pulse);
    }
    else if (cmd.equalsIgnoreCase("stop") || cmd.equalsIgnoreCase("disarm")) {
        writeAllMotors(PULSE_SAFE_DISARM);
        Serial.println(F("[CMD] ĐÃ DỪNG KHẨN CẤP TOÀN BỘ 4 ĐỘNG CƠ (900us)."));
    }
    else if (cmd.equalsIgnoreCase("status")) {
        Serial.println(F("\n--- SƠ ĐỒ CHÂN GPIO ĐỘNG CƠ ---"));
        Serial.println(F("  Motor 1 (Trước-Phải / CCW): GPIO 7"));
        Serial.println(F("  Motor 2 (Sau-Trái   / CCW): GPIO 6"));
        Serial.println(F("  Motor 3 (Trước-Trái / CW) : GPIO 5"));
        Serial.println(F("  Motor 4 (Sau-Phải   / CW) : GPIO 4"));
        Serial.println(F("--------------------------------"));
    }
    else {
        printHelp();
    }
}
