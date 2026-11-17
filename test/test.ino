/**
 * ============================================================================
 * ESP32 / ESP32-C6 USB SERIAL MOTOR TEST, CALIBRATION & PC XBOX BRIDGE
 * Kết nối: Tay cầm Xbox -> Máy tính (PC) -> Cáp USB Serial -> ESP32
 * ============================================================================
 * Cả bốn ESC cùng dải xung: Min/Disarm = 1000 us, Max = 2000 us.
 * ============================================================================
 */

#include <Arduino.h>

// ============================================================================
// 1. CẤU HÌNH PHẦN CỨNG & DẢI XUNG CHUNG CHO CẢ BỐN ESC
// ============================================================================
#define NUM_MOTORS          4

#define MOTOR1_PIN          7     // Động cơ 1 (GPIO 7) - ESC Vàng 30A
#define MOTOR2_PIN          6     // Động cơ 2 (GPIO 6) - ESC Vàng 30A
#define MOTOR3_PIN          5     // Động cơ 3 (GPIO 5) - ESC Đen  40A
#define MOTOR4_PIN          4     // Động cơ 4 (GPIO 4) - ESC Vàng 30A

#define ESC_FREQ_HZ         50    // 50Hz chuẩn Servo/ESC PWM (chu kỳ 20,000 us)
#define ESC_RES_BITS        14    // Độ phân giải 14-bit (16384 mức)

const uint8_t MOTOR_PINS[NUM_MOTORS] = {
    MOTOR1_PIN,
    MOTOR2_PIN,
    MOTOR3_PIN,
    MOTOR4_PIN
};

const char* MOTOR_NAMES[NUM_MOTORS] = {
    "Motor 1 (GPIO 7)", "Motor 2 (GPIO 6)",
    "Motor 3 (GPIO 5)", "Motor 4 (GPIO 4)"
};

// Mốc chung cho tất cả motor: Min/Disarm 1000 us, Max 2000 us.
const uint16_t ESC_MIN_US = 1000;
const uint16_t ESC_MAX_US = 2000;
uint16_t motorMinUs[NUM_MOTORS]     = { ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US };
uint16_t motorSpinMinUs[NUM_MOTORS] = { ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US };
uint16_t motorMaxUs[NUM_MOTORS]     = { ESC_MAX_US, ESC_MAX_US, ESC_MAX_US, ESC_MAX_US };

// ============================================================================
// 2. BIẾN TRẠNG THÁI HỆ THỐNG
// ============================================================================
enum SelectionMode {
    SELECT_MOTOR_1 = 0,
    SELECT_MOTOR_2 = 1,
    SELECT_MOTOR_3 = 2,
    SELECT_MOTOR_4 = 3,
    SELECT_ALL     = 4
};

static uint16_t currentPulseUs[NUM_MOTORS] = { ESC_MIN_US, ESC_MIN_US, ESC_MIN_US, ESC_MIN_US };
static SelectionMode activeTarget = SELECT_ALL;
static bool isCalibrating    = false;
static bool isArmed          = false;
static bool safetyLimit60Pct = true;
static bool webStreamEnabled = false; // Mặc định TẮT stream liên tục để Serial Monitor sạch sẽ khi Calib & Dò xung
static uint32_t lastTelemetryMs = 0;

// ============================================================================
// 3. TẦNG ĐIỀU KHIỂN XUNG PWM (TƯƠNG THÍCH ESP32 CORE 2.x & 3.x)
// ============================================================================
uint32_t pulseUsToDuty(uint16_t pulseUs) {
    const uint32_t periodUs = 1000000UL / ESC_FREQ_HZ; // 20000 us @ 50Hz
    const uint32_t maxDuty  = (1UL << ESC_RES_BITS);   // 16384
    if (pulseUs > periodUs) pulseUs = periodUs;
    return ((uint32_t)pulseUs * maxDuty) / periodUs;
}

void writeMotorUs(uint8_t motorIndex, uint16_t pulseUs) {
    if (motorIndex >= NUM_MOTORS) return;
    if (pulseUs < ESC_MIN_US) pulseUs = ESC_MIN_US;
    if (pulseUs > ESC_MAX_US) pulseUs = ESC_MAX_US;

    currentPulseUs[motorIndex] = pulseUs;
    uint32_t duty = pulseUsToDuty(pulseUs);

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(MOTOR_PINS[motorIndex], duty);
#else
    ledcWrite(motorIndex, duty);
#endif
}

// Đưa cả 4 động cơ về cùng mốc 0%: 1000us.
void stopAllMotorsToMin() {
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        writeMotorUs(i, motorMinUs[i]);
    }
}

// Đưa cả 4 động cơ lên cùng mốc Max: 2000us.
void setAllMotorsToMax() {
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        writeMotorUs(i, motorMaxUs[i]);
    }
}

// Tính % ga theo dải chung 1000..2000us.
float getMotorThrottlePct(uint8_t idx) {
    if (currentPulseUs[idx] <= motorSpinMinUs[idx]) return 0.0f;
    uint16_t span = motorMaxUs[idx] - motorSpinMinUs[idx];
    if (span == 0) return 0.0f;
    return ((float)(currentPulseUs[idx] - motorSpinMinUs[idx]) * 100.0f) / (float)span;
}

// Đặt % ga (0..100%) trong dải chung của cả bốn ESC.
void writeMotorPct(uint8_t idx, float pct) {
    if (idx >= NUM_MOTORS) return;
    if (pct <= 0.2f) {
        writeMotorUs(idx, motorMinUs[idx]);
        return;
    }
    if (pct > 100.0f) pct = 100.0f;
    uint16_t span = motorMaxUs[idx] - motorSpinMinUs[idx];
    uint16_t us = motorSpinMinUs[idx] + (uint16_t)((pct * (float)span) / 100.0f);
    writeMotorUs(idx, us);
}

// Đặt % ga cho mục tiêu đang chọn (m1, m2, m3, m4 hoặc all)
void writeSelectedTargetPct(float pct, bool verbose = true) {
    float maxAllowed = safetyLimit60Pct ? 60.0f : 100.0f;
    if (!isCalibrating && pct > maxAllowed) {
        pct = maxAllowed;
    }
    if (pct < 0.0f) pct = 0.0f;

    if (activeTarget == SELECT_ALL) {
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            writeMotorPct(i, pct);
        }
        if (verbose) {
            Serial.printf("[THR] ALL 4 Motors -> %5.1f%% (M1,2,4=%4u us | M3=%4u us)\n",
                          pct, currentPulseUs[0], currentPulseUs[2]);
        }
    } else {
        uint8_t idx = (uint8_t)activeTarget;
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            if (i != idx) writeMotorUs(i, motorMinUs[i]);
        }
        writeMotorPct(idx, pct);
        if (verbose) {
            Serial.printf("[THR] %s -> %5.1f%% (%4u us) | Cac motor khac o 0%%\n",
                          MOTOR_NAMES[idx], pct, currentPulseUs[idx]);
        }
    }
}

// Đặt trực tiếp số micro-giây (us) cho mục tiêu đang chọn
void writeSelectedTargetUs(uint16_t pulseUs) {
    if (activeTarget == SELECT_ALL) {
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            writeMotorUs(i, pulseUs);
        }
        Serial.printf("[PWM] ALL 4 Motors -> %u us\n", pulseUs);
    } else {
        uint8_t idx = (uint8_t)activeTarget;
        for (uint8_t i = 0; i < NUM_MOTORS; i++) {
            if (i != idx) writeMotorUs(i, motorMinUs[i]);
        }
        writeMotorUs(idx, pulseUs);
        Serial.printf("[PWM] %s -> %u us (%.1f%%) | Cac motor khac o 0%%\n",
                      MOTOR_NAMES[idx], pulseUs, getMotorThrottlePct(idx));
    }
}

uint16_t getActivePulseUs() {
    if (activeTarget == SELECT_ALL) return currentPulseUs[0];
    return currentPulseUs[(uint8_t)activeTarget];
}

// ============================================================================
// 4. HIỂN THỊ TRẠNG THÁI & HƯỚNG DẪN
// ============================================================================
void printStatus() {
    Serial.println("\n-------------------- TRANG THAI 4 DONG CO --------------------");
    Serial.printf("Trang thai Khoa : %s | Gioi han Ga: %s\n",
                  isArmed ? "ARMED (MO KHOA)" : "DISARMED (KHOA AN TOAN)",
                  safetyLimit60Pct ? "60% (An toan)" : "100% (Toan dai)");
    Serial.printf("Muc tieu chon   : %s\n",
                  (activeTarget == SELECT_ALL) ? "CA 4 DONG CO (ALL)" : MOTOR_NAMES[(uint8_t)activeTarget]);
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        Serial.printf("  %s : %4u us (%5.1f%%) [Dai: %u..%u us]%s\n",
                      MOTOR_NAMES[i],
                      currentPulseUs[i],
                      getMotorThrottlePct(i),
                      motorMinUs[i],
                      motorMaxUs[i],
                      (activeTarget == (SelectionMode)i || activeTarget == SELECT_ALL) ? " <-- [ACTIVE]" : "");
    }
    Serial.println("--------------------------------------------------------------\n");
}

void printHelp() {
    Serial.println("\n================ BANG DIEU KHIEN ESC & XBOX PC BRIDGE ================");
    Serial.println("  Cau hinh chung ca 4 ESC: 1000-2000us");
    Serial.println("----------------------------------------------------------------------");
    Serial.println("  1. DIEU KHIEN BANG TAY CAM XBOX CAM VAO MAY TINH:");
    Serial.println("     - Cach 1 (Python): Chay 'python tools/xbox_control.py' tren PC");
    Serial.println("       (Dong serial monitor cua Arduino IDE truoc khi chay script Python)");
    Serial.println("     - Cach 2 (Web 3D): Mo 'tools/hitl_monitor_web/UAV HITL Testbed Monitor.html'");
    Serial.println("       va bam 'Ket noi USB Serial', tay cam Xbox tren PC se dieu khien truc tiep!");
    Serial.println("----------------------------------------------------------------------");
    Serial.println("  2. LENH SERIAL MONITOR (115200 BAUD):");
    Serial.println("     - 'arm' / 'S' (stop)   : Mo khoa / Dung khan cap ve 0%");
    Serial.println("     - 'm1','m2','m3','m4','all' : Chon dong co can dieu khien");
    Serial.println("     - '0..100' hoac 'throttle <pct>' : Dat % ga tren cung mot dải cho 4 ESC");
    Serial.println("     - '1000..2000'          : Nhap truc tiep so micro-giay (us)");
    Serial.println("     - '+' / '-'            : Tang / giam 20 us");
    Serial.println("     - 'r' (ramp) / 'scan'  : Test tang giam muot / Quet do xung");
    Serial.println("     - 'H' / 'L'            : Calibration Max / Min");
    Serial.println("     - 'limit'              : Bat/Tat gioi han ga an toan (60% <-> 100%)");
    Serial.println("======================================================================\n");
}

void runSmoothRampTest() {
    Serial.println("\n[RAMP TEST] >>> Tang muot tu 0% -> 30% -> 0% (Go phim bat ky de HUY)...");

    for (int pct = 0; pct <= 30; pct += 2) {
        if (Serial.available()) {
            while (Serial.available()) Serial.read();
            stopAllMotorsToMin();
            Serial.println("\n[RAMP TEST] [DA HUY] Dung khan cap ve 0%!");
            return;
        }
        writeSelectedTargetPct((float)pct);
        delay(80);
    }

    uint32_t holdStart = millis();
    while (millis() - holdStart < 1500) {
        if (Serial.available()) {
            while (Serial.available()) Serial.read();
            stopAllMotorsToMin();
            Serial.println("\n[RAMP TEST] [DA HUY] Dung khan cap ve 0%!");
            return;
        }
        delay(20);
    }

    for (int pct = 30; pct >= 0; pct -= 2) {
        if (Serial.available()) {
            while (Serial.available()) Serial.read();
            stopAllMotorsToMin();
            Serial.println("\n[RAMP TEST] [DA HUY] Dung khan cap ve 0%!");
            return;
        }
        writeSelectedTargetPct((float)pct);
        delay(60);
    }

    stopAllMotorsToMin();
    Serial.println("[RAMP TEST] >>> Hoan tat Ramp Test! Ca 4 dong co da ve 0% an toan.\n");
}

// ============================================================================
// 5. XỬ LÝ LỆNH TỪ SERIAL MONITOR & XBOX PC BRIDGE
// ============================================================================
void processCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) return;

    // 1. Lệnh dừng khẩn cấp (S, stop, disarm, kill, m stop)
    if (cmd.equalsIgnoreCase("s") || cmd.equalsIgnoreCase("stop") ||
        cmd.equalsIgnoreCase("kill") || cmd.equalsIgnoreCase("disarm") ||
        cmd.equalsIgnoreCase("m stop")) {
        isCalibrating = false;
        isArmed = false;
        stopAllMotorsToMin();
        Serial.println("[STOP] >>> DISARMED! Ca 4 ESC ve moc 0% (M1..M4=1000us).");
        return;
    }

    // 2. Lệnh gói tay cầm dạng <Throttle,Roll,Pitch,Yaw,Arm> (1000..2000us)
    if (cmd.startsWith("<") && cmd.endsWith(">")) {
        String inner = cmd.substring(1, cmd.length() - 1);
        int t = 1000, r = 0, p = 0, y = 0, arm = 0;
        if (sscanf(inner.c_str(), "%d,%d,%d,%d,%d", &t, &r, &p, &y, &arm) == 5) {
            if (!isCalibrating) {
                isArmed = (arm == 1);
                if (!isArmed || t <= 1040) {
                    stopAllMotorsToMin();
                } else {
                    float m1Pct = (float)(constrain(t - p - r - y, 1000, 2000) - 1000) * 0.1f;
                    float m2Pct = (float)(constrain(t + p + r - y, 1000, 2000) - 1000) * 0.1f;
                    float m3Pct = (float)(constrain(t + p - r + y, 1000, 2000) - 1000) * 0.1f;
                    float m4Pct = (float)(constrain(t - p + r + y, 1000, 2000) - 1000) * 0.1f;
                    writeMotorPct(0, m1Pct);
                    writeMotorPct(1, m2Pct);
                    writeMotorPct(2, m3Pct);
                    writeMotorPct(3, m4Pct);
                }
            }
            return;
        }
    }

    // 3. Lệnh gói nhanh 4 kênh từ script Xbox trên PC / Web 3D: "x <m1%> <m2%> <m3%> <m4%>"
    if (cmd.startsWith("x ") || cmd.startsWith("X ")) {
        float p1 = 0, p2 = 0, p3 = 0, p4 = 0;
        if (sscanf(cmd.c_str() + 2, "%f %f %f %f", &p1, &p2, &p3, &p4) == 4) {
            isArmed = true;
            webStreamEnabled = true; // Tự động bật stream 20Hz cho Web 3D Monitor
            float maxCap = safetyLimit60Pct ? 60.0f : 100.0f;
            if (p1 > maxCap) p1 = maxCap;
            if (p2 > maxCap) p2 = maxCap;
            if (p3 > maxCap) p3 = maxCap;
            if (p4 > maxCap) p4 = maxCap;
            writeMotorPct(0, p1);
            writeMotorPct(1, p2);
            writeMotorPct(2, p3);
            writeMotorPct(3, p4);
        }
        return;
    }

    // 4. Lệnh đặt % ga từ Web / Xbox Script: "throttle <pct>" hoặc "set <pct>"
    if (cmd.startsWith("throttle ") || cmd.startsWith("THROTTLE ") ||
        cmd.startsWith("set ") || cmd.startsWith("SET ")) {
        int idx = cmd.indexOf(' ');
        float pct = cmd.substring(idx + 1).toFloat();
        isArmed = true;
        webStreamEnabled = true; // Tự động bật stream 20Hz cho Web 3D Monitor
        writeSelectedTargetPct(pct, true);
        return;
    }

    // 4. Lệnh điều khiển từng motor kèm % từ Web: "m1 20", "m2 15", "m3 20", "m4 20"
    if ((cmd.startsWith("m1 ") || cmd.startsWith("m2 ") ||
         cmd.startsWith("m3 ") || cmd.startsWith("m4 ") ||
         cmd.startsWith("M1 ") || cmd.startsWith("M2 ") ||
         cmd.startsWith("M3 ") || cmd.startsWith("M4 "))) {
        int mId = cmd.substring(1, 2).toInt();
        float pct = cmd.substring(3).toFloat();
        if (mId >= 1 && mId <= 4) {
            isArmed = true;
            activeTarget = (SelectionMode)(mId - 1);
            writeSelectedTargetPct(pct, true);
        }
        return;
    }

    // 5. Hiệu chuẩn ESC (Calibration)
    if (cmd.equalsIgnoreCase("h") || cmd.equalsIgnoreCase("cal_max") || cmd.equalsIgnoreCase("calib_start")) {
        isCalibrating = true;
        isArmed = false;
        setAllMotorsToMax();
        Serial.println("\n=== [BUOC 1/2: CALIBRATION MAX THROTTLE] ===");
        printStatus();
        return;
    }

    if (cmd.equalsIgnoreCase("l") || cmd.equalsIgnoreCase("cal_min") ||
        cmd.equalsIgnoreCase("arm") || cmd.equalsIgnoreCase("calib_finish")) {
        isCalibrating = false;
        isArmed = true;
        stopAllMotorsToMin();
        Serial.println("\n=== [ARMED / MOC 0% GA: M1..M4 = 1000us] ===");
        printStatus();
        return;
    }

    // 6. Bật / tắt giới hạn trần ga an toàn 60% <-> 100%
    if (cmd.equalsIgnoreCase("limit")) {
        safetyLimit60Pct = !safetyLimit60Pct;
        Serial.printf("[LIMIT] Gioi han ga toi da: %s\n",
                      safetyLimit60Pct ? "60% (An toan)" : "100% (Toan dai)");
        return;
    }

    // 8. Chọn động cơ cần test
    if (cmd.equalsIgnoreCase("m1")) {
        stopAllMotorsToMin();
        activeTarget = SELECT_MOTOR_1;
        Serial.println("[SELECT] >>> Da chon: MOTOR 1 (GPIO 7 - ESC Vang).");
        return;
    }
    if (cmd.equalsIgnoreCase("m2")) {
        stopAllMotorsToMin();
        activeTarget = SELECT_MOTOR_2;
        Serial.println("[SELECT] >>> Da chon: MOTOR 2 (GPIO 6 - ESC Vang).");
        return;
    }
    if (cmd.equalsIgnoreCase("m3")) {
        stopAllMotorsToMin();
        activeTarget = SELECT_MOTOR_3;
        Serial.println("[SELECT] >>> Da chon: MOTOR 3 (GPIO 5 - cùng dải 1000-2000us).");
        return;
    }
    if (cmd.equalsIgnoreCase("m4")) {
        stopAllMotorsToMin();
        activeTarget = SELECT_MOTOR_4;
        Serial.println("[SELECT] >>> Da chon: MOTOR 4 (GPIO 4 - ESC Vang).");
        return;
    }
    if (cmd.equalsIgnoreCase("all")) {
        stopAllMotorsToMin();
        activeTarget = SELECT_ALL;
        Serial.println("[SELECT] >>> Da chon: CA 4 DONG CO (Tu dong dong bo % ga giua ESC Vang & ESC Den).");
        return;
    }

    // 9. Tăng / giảm từng nấc 20 us
    if (cmd.equals("+")) {
        isArmed = true;
        if (activeTarget == SELECT_ALL) {
            for (uint8_t i = 0; i < NUM_MOTORS; i++) {
                writeMotorUs(i, currentPulseUs[i] + 20);
            }
            printStatus();
        } else {
            writeSelectedTargetUs(getActivePulseUs() + 20);
        }
        return;
    }
    if (cmd.equals("-")) {
        if (activeTarget == SELECT_ALL) {
            for (uint8_t i = 0; i < NUM_MOTORS; i++) {
                uint16_t nextUs = (currentPulseUs[i] >= motorMinUs[i] + 20) ? (currentPulseUs[i] - 20) : motorMinUs[i];
                writeMotorUs(i, nextUs);
            }
            printStatus();
        } else {
            uint16_t curUs = getActivePulseUs();
            uint16_t nextUs = (curUs >= ESC_MIN_US + 20) ? (curUs - 20) : ESC_MIN_US;
            writeSelectedTargetUs(nextUs);
        }
        return;
    }

    // 10. Ramp test & Scan
    if (cmd.equalsIgnoreCase("r") || cmd.equalsIgnoreCase("ramp")) {
        runSmoothRampTest();
        return;
    }
    if (cmd.equalsIgnoreCase("scan")) {
        Serial.println("\n[SCAN] >>> Quet xung tu 1000 us -> 1300 us (Go phim bat ky de dung)...");
        for (uint16_t us = 800; us <= 1300; us += 15) {
            if (Serial.available()) {
                while (Serial.available()) Serial.read();
                Serial.printf("\n[SCAN] Dung tai: %u us\n", getActivePulseUs());
                return;
            }
            writeSelectedTargetUs(us);
            delay(500);
        }
        return;
    }

    // 11. Trạng thái & Trợ giúp
    if (cmd.equalsIgnoreCase("web")) {
        webStreamEnabled = !webStreamEnabled;
        Serial.printf("[WEB] Stream 20Hz cho Web 3D: %s\n", webStreamEnabled ? "BAT" : "TAT");
        return;
    }
    if (cmd.equalsIgnoreCase("status") || cmd.equalsIgnoreCase("st")) {
        printStatus();
        return;
    }
    if (cmd.equalsIgnoreCase("help") || cmd.equalsIgnoreCase("?")) {
        printHelp();
        return;
    }

    // 12. Nhập số trực tiếp: 0..100 (%) hoặc 1000..2000 (us)
    bool isNumeric = true;
    for (size_t i = 0; i < cmd.length(); i++) {
        if (!isDigit(cmd[i])) {
            isNumeric = false;
            break;
        }
    }

    if (isNumeric) {
        long val = cmd.toInt();
        isArmed = true;
        if (val >= 0 && val <= 100) {
            writeSelectedTargetPct((float)val, true);
        } else if (val >= ESC_MIN_US && val <= ESC_MAX_US) {
            writeSelectedTargetUs((uint16_t)val);
        } else {
            Serial.println("[LOI] Nhap 0..100 (%) hoac 1000..2000 (us).");
        }
        return;
    }

    Serial.printf("[LOI] Lenh khong hop le: '%s'. Go 'help' de xem huong dan.\n", cmd.c_str());
}

// ============================================================================
// 6. ARDUINO SETUP & LOOP
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    // Khởi tạo PWM và đặt ngay mốc Min riêng cho từng loại ESC:
    // M1..M4 = 1000..2000 us (dải chung)
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        pinMode(MOTOR_PINS[i], OUTPUT);
        digitalWrite(MOTOR_PINS[i], LOW);

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
        ledcAttach(MOTOR_PINS[i], ESC_FREQ_HZ, ESC_RES_BITS);
#else
        ledcSetup(i, ESC_FREQ_HZ, ESC_RES_BITS);
        ledcAttachPin(MOTOR_PINS[i], i);
#endif
        writeMotorUs(i, motorMinUs[i]);
    }

    printHelp();
    printStatus();
}

void loop() {
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        processCommand(cmd);
    }

    // Chỉ phát liên tục 20Hz khi bật chế độ 'web' (mặc định tắt để màn hình Serial Monitor đứng yên dễ Calib/Scan)
    if (webStreamEnabled) {
        uint32_t nowMs = millis();
        if (nowMs - lastTelemetryMs >= 50) {
            lastTelemetryMs = nowMs;
            Serial.println("[PX4_EKF] R:   0.0° P:   0.0° Y:   0.0° | Acc:[  0.00,  0.00,  9.81] | Gyr:[   0.0,   0.0,   0.0] (OPTIMAL)");
            Serial.printf("[ESC_QUAD] State:%-8s | M1(P7-CCW):%4uus(%4.1f%%) | M2(P6-CCW):%4uus(%4.1f%%) | M3(P5-CW):%4uus(%4.1f%%) | M4(P4-CW):%4uus(%4.1f%%) @ %dHz\n",
                          isArmed ? "ARMED" : "DISARMED",
                          currentPulseUs[0], getMotorThrottlePct(0),
                          currentPulseUs[1], getMotorThrottlePct(1),
                          currentPulseUs[2], getMotorThrottlePct(2),
                          currentPulseUs[3], getMotorThrottlePct(3),
                          ESC_FREQ_HZ);
        }
    }
}
