/**
 * ============================================================================
 * ESP32 / ESP32-C6 - CHƯƠNG TRÌNH HIỆU CHUẨN DẢI GA (CALIBRATION) 4 ESC
 * ============================================================================
 * Tác giả: Nhóm Nghiên cứu UAV - ISILAB
 * Mục đích:
 *   1. Dạy dải ga (Throttle Range Calibration 1000µs - 2000µs) đồng bộ cho cả 4 ESC Vàng 30A.
 *   2. Hỗ trợ Calibrate ĐỒNG THỜI cả 4 ESC (nhanh và chuẩn nhất vì cả 4 ESC cùng loại Vàng 30A)
 *      hoặc CALIBRATE RIÊNG TỪNG CON (M1, M2, M3, M4) khi cần kiểm tra đường dây tín hiệu.
 *   3. Kiểm tra chiều quay (CW/CCW) và ngưỡng quay không tải (Idle Spin)
 *      ngay sau khi hiệu chuẩn.
 * 
 * SƠ ĐỒ CHÂN PHẦN CỨNG (Quad-X chuẩn PX4 - Đồng bộ 100% ESC Vàng 30A):
 *   - Motor 1 (M1): GPIO 7 (Front-Left,  CCW) - ESC Vàng 30A
 *   - Motor 2 (M2): GPIO 6 (Rear-Right,  CCW) - ESC Vàng 30A
 *   - Motor 3 (M3): GPIO 5 (Rear-Left,   CW ) - ESC Vàng 30A
 *   - Motor 4 (M4): GPIO 4 (Front-Right, CW ) - ESC Vàng 30A
 * 
 * Tần số PWM: 50 Hz (Chu kỳ 20,000 µs - Chuẩn công nghiệp cho mọi loại ESC)
 * Độ phân giải: 14-bit (16384 mức - mịn ~1.22 µs)
 * ============================================================================
 */

#include <Arduino.h>

// ============================================================================
// 1. ĐỊNH NGHĨA CHÂN VÀ THÔNG SỐ XUNG PWM
// ============================================================================
#define NUM_MOTORS          4

#define MOTOR1_PIN          7     // M1: GPIO 7
#define MOTOR2_PIN          6     // M2: GPIO 6
#define MOTOR3_PIN          5     // M3: GPIO 5
#define MOTOR4_PIN          4     // M4: GPIO 4

#define ESC_FREQ_HZ         50    // 50 Hz tiêu chuẩn
#define ESC_RES_BITS        14    // 14-bit LEDC

const uint8_t MOTOR_PINS[NUM_MOTORS] = {
    MOTOR1_PIN,
    MOTOR2_PIN,
    MOTOR3_PIN,
    MOTOR4_PIN
};

const char* MOTOR_NAMES[NUM_MOTORS] = {
    "M1 (GPIO 7 - FL CCW)",
    "M2 (GPIO 6 - RR CCW)",
    "M3 (GPIO 5 - RL CW )",
    "M4 (GPIO 4 - FR CW )"
};

// Mốc dải xung chuẩn
#define CALIB_MAX_US        2000  // Mốc ga tối đa 100% khi bắt đầu calib
#define CALIB_MIN_US        1000  // Mốc ga tối thiểu 0% kết thúc calib
#define IDLE_TEST_US        1050  // Mốc quay chậm không tải để test motor

// Lưu trữ xung hiện tại của từng motor
uint16_t currentPulseUs[NUM_MOTORS] = { CALIB_MIN_US, CALIB_MIN_US, CALIB_MIN_US, CALIB_MIN_US };

// Trạng thái hiệu chuẩn
enum CalibStep {
    STEP_IDLE = 0,
    STEP_WAIT_BATTERY_CONNECT, // Đang phát xung MAX (2000us), chờ cắm pin
    STEP_BATTERY_CONNECTED,    // Đã phát hiện tín hiệu sẵn sàng hạ xung
    STEP_COMPLETE              // Đã hạ xung MIN (1000us), hiệu chuẩn xong
};

CalibStep currentStep = STEP_IDLE;
int activeCalibMotor = 0; // 0 = CẢ 4 MOTOR; 1..4 = TỪNG MOTOR RIÊNG LẺ
int activeTestMotor  = 0; // 0 = CẢ 4 MOTOR; 1..4 = TỪNG MOTOR TEST

// ============================================================================
// 2. DRIVER PWM TƯƠNG THÍCH MỌI PHIÊN BẢN ESP32 (CORE 2.x & 3.x)
// ============================================================================
uint32_t usToDuty(uint16_t pulseUs) {
    const uint32_t periodUs = 1000000UL / ESC_FREQ_HZ; // 20000 us @ 50Hz
    const uint32_t maxDuty  = (1UL << ESC_RES_BITS) - 1UL;
    if (pulseUs > periodUs) pulseUs = (uint16_t)periodUs;
    return (uint32_t)(((uint64_t)pulseUs * maxDuty) / periodUs);
}

void writeMotorPulse(uint8_t index, uint16_t pulseUs) {
    if (index >= NUM_MOTORS) return;
    if (pulseUs < 800) pulseUs = 800;
    if (pulseUs > 2200) pulseUs = 2200;

    currentPulseUs[index] = pulseUs;
    uint32_t duty = usToDuty(pulseUs);

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
    ledcWrite(MOTOR_PINS[index], duty);
#else
    ledcWrite(index, duty);
#endif
}

void setAllMotors(uint16_t pulseUs) {
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        writeMotorPulse(i, pulseUs);
    }
}

void stopAllMotors() {
    setAllMotors(CALIB_MIN_US);
}

void initPwm() {
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        pinMode(MOTOR_PINS[i], OUTPUT);
        digitalWrite(MOTOR_PINS[i], LOW);

#if ESP_ARDUINO_VERSION >= ESP_ARDUINO_VERSION_VAL(3, 0, 0)
        ledcAttach(MOTOR_PINS[i], ESC_FREQ_HZ, ESC_RES_BITS);
#else
        ledcSetup(i, ESC_FREQ_HZ, ESC_RES_BITS);
        ledcAttachPin(MOTOR_PINS[i], i);
#endif
        writeMotorPulse(i, CALIB_MIN_US);
    }
}

// ============================================================================
// 3. GIAO DIỆN HƯỚNG DẪN SERIAL MENU
// ============================================================================
void printBanner() {
    Serial.println("\n==================================================================");
    Serial.println("     ESP32-C6 UAV FLIGHT CONTROLLER - ESC CALIBRATION TOOL");
    Serial.println("==================================================================");
    Serial.println(" [!] CANH BAO AN TOAN BAT BUOC:");
    Serial.println("     1. THAO TAT CA CANH QUAT (PROPELLERS OFF) KHOI DONG CO!");
    Serial.println("     2. CHUA CAM PIN LiPo VAO ESC O BUOC KHOI DONG NAY!");
    Serial.println("     3. Cap nguon cho ESP32 bang day cap USB Serial.");
    Serial.println("==================================================================");
}

void printMenu() {
    Serial.println("\n----------------- DANH SACH LENH DIEU KHIEN -----------------");
    Serial.println(" [1] HIỆU CHUẨN ĐỒNG LOẠT CẢ 4 ESC VÀNG 30A (KHUYÊN DÙNG - CỰC NHANH):");
    Serial.println("     cal all : Calibrate ca 4 ESC Vang 30A dong thoi (dong nhat tieng bip)");
    Serial.println("");
    Serial.println(" [2] HIỆU CHUẨN RIÊNG TỪNG ESC (KHI CẦN TEST RIÊNG TỪNG KÊNH):");
    Serial.println("     cal 1   : Calibrate rieng ESC 1 (GPIO 7 - ESC Vang 30A)");
    Serial.println("     cal 2   : Calibrate rieng ESC 2 (GPIO 6 - ESC Vang 30A)");
    Serial.println("     cal 3   : Calibrate rieng ESC 3 (GPIO 5 - ESC Vang 30A)");
    Serial.println("     cal 4   : Calibrate rieng ESC 4 (GPIO 4 - ESC Vang 30A)");
    Serial.println("");
    Serial.println(" [3] BƯỚC HẠ GA XÁC NHẬN (KHI ĐÃ NGHE TIẾNG BIP-BIP):");
    Serial.println("     cal all : Calibrate ca 4 ESC cung mot luc");
    Serial.println("");
    Serial.println(" [3] BƯỚC HẠ GA XÁC NHẬN (KHI ĐÃ NGHE TIẾNG BIP-BIP):");
    Serial.println("     min / l / 0 / [Phím Space] : Ha xung ve 1000us de luu vao EEPROM");
    Serial.println("");
    Serial.println(" [4] TEST DÒ XUNG & KIỂM TRA QUAY THỬ SAU KHI CALIB:");
    Serial.println("     spin 1..4 / spin all : Quay thu nhe o 1050us (~5% ga)");
    Serial.println("     t 10 / t 20          : Chay thu o 10% hoac 20% ga");
    Serial.println("     1000..2000           : Nhap truc tiep so micro-giay (us)");
    Serial.println("     + / -                : Tang / Giam 10 us de do nguong khoi dong");
    Serial.println("     s / stop             : DUNG KHAN CAP toan bo dong co ve 1000us");
    Serial.println("-------------------------------------------------------------");
    Serial.println(">>> Hay go 'cal all' hoac 'cal 1'..'cal 4' de bat dau!");
}

void printCurrentStatus() {
    Serial.printf("\n[STATUS] Trang thai: %s | Xung hien tai:\n", 
        (currentStep == STEP_WAIT_BATTERY_CONNECT) ? "DANG PHAT 2000us (CHO CAM PIN)" :
        (currentStep == STEP_COMPLETE) ? "CALIB HOAN TAT (1000us)" : "IDLE (SAN SANG)");
    for (int i = 0; i < NUM_MOTORS; i++) {
        Serial.printf("  - %s: %4u us\n", MOTOR_NAMES[i], currentPulseUs[i]);
    }
}

// ============================================================================
// 4. QUY TRÌNH CALIBRATION (STEP-BY-STEP WORKFLOW)
// ============================================================================
void startCalibration(int targetMotor) {
    activeCalibMotor = targetMotor;
    currentStep = STEP_WAIT_BATTERY_CONNECT;

    if (targetMotor == 0) {
        // Cả 4 motor
        setAllMotors(CALIB_MAX_US);
        Serial.println("\n==================================================================");
        Serial.println(">>> [BUOC 1/2]: DANG PHAT XUNG MAX (2000 us) TREN CA 4 CHAN!");
        Serial.println("    M1(P7), M2(P6), M3(P5), M4(P4) = 2000 us.");
    } else {
        // Chỉ motor được chọn
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (i == (targetMotor - 1)) {
                writeMotorPulse(i, CALIB_MAX_US);
            } else {
                writeMotorPulse(i, CALIB_MIN_US);
            }
        }
        Serial.println("\n==================================================================");
        Serial.printf(">>> [BUOC 1/2]: DANG PHAT XUNG MAX (2000 us) RIENG CHO %s!\n", MOTOR_NAMES[targetMotor - 1]);
        Serial.println("    Cac motor khac duoc giu yen o 1000 us de tranh quay bat ngo.");
    }

    Serial.println("------------------------------------------------------------------");
    Serial.println("  HUONG DAN THAO TAC TIEP THEO:");
    Serial.println("  1. BAY GIO HAY CAM NGUON PIN LiPo VAO ESC!");
    Serial.println("  2. Lang nghe ESC khoi dong:");
    Serial.println("     - Se co giai dieu nhac khoi dong.");
    Serial.println("     - Ngay sau do se phat 2 TIENG BIP NGAN ('Bip - Bip')");
    Serial.println("       bao hieu da nhan moc ga cao nhat 2000 us.");
    Serial.println("  3. Ngay sau khi nghe 2 tieng bip do (trong vong 2 giay):");
    Serial.println("     -> Hay nhap 'min' hoac bam [ENTER] / [PHIM SPACE]!");
    Serial.println("==================================================================\n");
}

void finishCalibration() {
    if (currentStep != STEP_WAIT_BATTERY_CONNECT) {
        Serial.println("[!] ESC chua o trang thai cho Calib. Hay go 'cal all' hoac 'cal 1'..'cal 4' truoc.");
        return;
    }

    // Hạ xung về mốc 1000us
    if (activeCalibMotor == 0) {
        setAllMotors(CALIB_MIN_US);
        Serial.println("\n==================================================================");
        Serial.println(">>> [BUOC 2/2]: DA HA XUNG CA 4 CHAN VE MOC MIN (1000 us)!");
    } else {
        writeMotorPulse(activeCalibMotor - 1, CALIB_MIN_US);
        Serial.println("\n==================================================================");
        Serial.printf(">>> [BUOC 2/2]: DA HA XUNG %s VE MOC MIN (1000 us)!\n", MOTOR_NAMES[activeCalibMotor - 1]);
    }

    currentStep = STEP_COMPLETE;

    Serial.println("------------------------------------------------------------------");
    Serial.println("  XAC NHAN TU ESC:");
    Serial.println("  - ESC se phat so tieng bip tuong ung so cell pin (vd: 3 bip cho 3S LiPo).");
    Serial.println("  - Sau do phat 1 TIENG BIP DAI ('Biiiiip') xac nhan DA LUU VAO EEPROM!");
    Serial.println("  - Hieu chuan hoan tat thanh cong! ESC da ghi nho dai ga 1000..2000 us.");
    Serial.println("------------------------------------------------------------------");
    Serial.println(">>> Gio ban co the go 'spin' hoac 't 10' de chay thu dong co!");
    Serial.println("==================================================================\n");
}

// ============================================================================
// 5. TEST QUAY THỬ & DÒ XUNG ĐỘNG CƠ (MOTOR TEST MODE)
// ============================================================================
void runMotorTest(int motorId, uint16_t pulseUs) {
    if (pulseUs < CALIB_MIN_US) pulseUs = CALIB_MIN_US;
    if (pulseUs > 1400) {
        Serial.println("[!] CANH BAO: Che do test chi cho phep xung toi da 1400 us (~40% ga) de an toan!");
        pulseUs = 1400;
    }

    if (motorId == 0) {
        setAllMotors(pulseUs);
        float pct = ((float)(pulseUs - CALIB_MIN_US) * 100.0f) / 1000.0f;
        Serial.printf("[TEST] CA 4 MOTOR -> %u us (%.1f%% ga)\n", pulseUs, pct);
    } else if (motorId >= 1 && motorId <= NUM_MOTORS) {
        for (int i = 0; i < NUM_MOTORS; i++) {
            if (i == (motorId - 1)) {
                writeMotorPulse(i, pulseUs);
            } else {
                writeMotorPulse(i, CALIB_MIN_US);
            }
        }
        float pct = ((float)(pulseUs - CALIB_MIN_US) * 100.0f) / 1000.0f;
        Serial.printf("[TEST] %s -> %u us (%.1f%% ga) | Cac motor khac = 1000 us\n", 
                      MOTOR_NAMES[motorId - 1], pulseUs, pct);
    }
}

// ============================================================================
// 6. XỬ LÝ LỆNH TỪ SERIAL MONITOR
// ============================================================================
void processCommand(String cmd) {
    cmd.trim();
    if (cmd.length() == 0) {
        // Nhấn phím Enter trắng: Nếu đang ở bước chờ hạ ga thì coi như bấm xác nhận!
        if (currentStep == STEP_WAIT_BATTERY_CONNECT) {
            finishCalibration();
        }
        return;
    }

    // A. LỆNH BẮT ĐẦU HIỆU CHUẨN
    if (cmd.equalsIgnoreCase("cal all") || cmd.equalsIgnoreCase("calall")) {
        startCalibration(0);
        return;
    }
    if (cmd.equalsIgnoreCase("cal 1") || cmd.equalsIgnoreCase("cal1")) {
        startCalibration(1);
        return;
    }
    if (cmd.equalsIgnoreCase("cal 2") || cmd.equalsIgnoreCase("cal2")) {
        startCalibration(2);
        return;
    }
    if (cmd.equalsIgnoreCase("cal 3") || cmd.equalsIgnoreCase("cal3")) {
        startCalibration(3);
        return;
    }
    if (cmd.equalsIgnoreCase("cal 4") || cmd.equalsIgnoreCase("cal4")) {
        startCalibration(4);
        return;
    }

    // B. LỆNH HẠ GA VỀ MIN KẾT THÚC HIỆU CHUẨN
    if (cmd.equalsIgnoreCase("min") || cmd.equalsIgnoreCase("l") || 
        cmd.equalsIgnoreCase("low") || cmd.equals("0")) {
        finishCalibration();
        return;
    }

    // C. DỪNG KHẨN CẤP
    if (cmd.equalsIgnoreCase("s") || cmd.equalsIgnoreCase("stop") || cmd.equalsIgnoreCase("off")) {
        stopAllMotors();
        currentStep = STEP_IDLE;
        Serial.println("\n[STOP] >>> DA DUNG TAT CA DONG CO VE 1000 us (0% GA)!");
        return;
    }

    // D. LỆNH TEST QUAY CHẬM (SPIN TEST)
    if (cmd.equalsIgnoreCase("spin") || cmd.equalsIgnoreCase("spin all") || cmd.equalsIgnoreCase("spinall")) {
        activeTestMotor = 0;
        runMotorTest(0, IDLE_TEST_US);
        return;
    }
    if (cmd.equalsIgnoreCase("spin 1") || cmd.equalsIgnoreCase("spin1")) {
        activeTestMotor = 1;
        runMotorTest(1, IDLE_TEST_US);
        return;
    }
    if (cmd.equalsIgnoreCase("spin 2") || cmd.equalsIgnoreCase("spin2")) {
        activeTestMotor = 2;
        runMotorTest(2, IDLE_TEST_US);
        return;
    }
    if (cmd.equalsIgnoreCase("spin 3") || cmd.equalsIgnoreCase("spin3")) {
        activeTestMotor = 3;
        runMotorTest(3, IDLE_TEST_US);
        return;
    }
    if (cmd.equalsIgnoreCase("spin 4") || cmd.equalsIgnoreCase("spin4")) {
        activeTestMotor = 4;
        runMotorTest(4, IDLE_TEST_US);
        return;
    }

    // E. TEST PHẦN TRĂM GA: 't <pct>' hoặc 'throttle <pct>'
    if (cmd.startsWith("t ") || cmd.startsWith("T ") || cmd.startsWith("throttle ")) {
        int spaceIdx = cmd.indexOf(' ');
        float pct = cmd.substring(spaceIdx + 1).toFloat();
        if (pct < 0.0f) pct = 0.0f;
        if (pct > 40.0f) {
            Serial.println("[!] Che do test gioi han ga toi da 40% de dam bao an toan tren ban test!");
            pct = 40.0f;
        }
        uint16_t us = CALIB_MIN_US + (uint16_t)((pct * 1000.0f) / 100.0f);
        runMotorTest(activeTestMotor, us);
        return;
    }

    // F. TĂNG / GIẢM DÒ ĐIỂM QUAY (+ / - 10us)
    if (cmd.equals("+")) {
        uint16_t current = currentPulseUs[(activeTestMotor == 0) ? 0 : (activeTestMotor - 1)];
        runMotorTest(activeTestMotor, current + 10);
        return;
    }
    if (cmd.equals("-")) {
        uint16_t current = currentPulseUs[(activeTestMotor == 0) ? 0 : (activeTestMotor - 1)];
        if (current >= CALIB_MIN_US + 10) {
            runMotorTest(activeTestMotor, current - 10);
        } else {
            runMotorTest(activeTestMotor, CALIB_MIN_US);
        }
        return;
    }

    // G. NHẬP SỐ MICRO-GIÂY TRỰC TIẾP (vd: 1060, 1100, 1200)
    int val = cmd.toInt();
    if (val >= 900 && val <= 2000) {
        runMotorTest(activeTestMotor, (uint16_t)val);
        return;
    }

    // H. LỆNH TRỢ GIÚP / TRẠNG THÁI
    if (cmd.equalsIgnoreCase("help") || cmd.equalsIgnoreCase("?")) {
        printMenu();
        return;
    }
    if (cmd.equalsIgnoreCase("status")) {
        printCurrentStatus();
        return;
    }

    Serial.printf("[?] Khong nhan dien duoc lenh: '%s'. Go 'help' de xem huong dan.\n", cmd.c_str());
}

// ============================================================================
// 7. SETUP & MAIN LOOP
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000); // Đợi Serial ổn định

    initPwm();
    stopAllMotors();

    printBanner();
    printMenu();
}

void loop() {
    // Đọc lệnh từ Serial Monitor
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        processCommand(input);
    }
}
