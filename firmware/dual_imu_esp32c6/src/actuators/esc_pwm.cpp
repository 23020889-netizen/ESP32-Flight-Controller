#include "esc_pwm.h"
#include <math.h>

// Global controller instance
QuadEscController quadMotors;

// ============================================================================
// OOP IMPLEMENTATION: EscMotor
// ============================================================================
EscMotor::EscMotor()
    : _id(0), _pin(0), _direction(MOTOR_DIR_CCW),
      _freq_hz(ESC_DEFAULT_FREQ_HZ), _period_us(20000), _max_duty(16383),
      _current_us(ESC_PULSE_DISARM_US), _target_us(ESC_PULSE_DISARM_US),
      _throttle_pct(0.0f), _attached(false) {
}

uint32_t EscMotor::usToDuty(uint16_t pulse_us) const {
    if (pulse_us > _period_us) {
        pulse_us = (uint16_t)_period_us;
    }
    return (uint32_t)(((uint64_t)pulse_us * _max_duty) / _period_us);
}

bool EscMotor::init(uint8_t id, uint8_t pin, MotorDirection dir, uint32_t freq_hz) {
    _id          = id;
    _pin         = pin;
    _direction   = dir;
    _freq_hz     = freq_hz;
    _period_us   = 1000000UL / freq_hz;
    _max_duty    = (1UL << ESC_RESOLUTION_BITS) - 1UL;
    _current_us  = ESC_PULSE_DISARM_US;
    _target_us   = ESC_PULSE_DISARM_US;
    _throttle_pct= 0.0f;

    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);

    // Gán kênh phần cứng LEDC Timer trên ESP32 Core 3.x
    _attached = ledcAttach(_pin, _freq_hz, ESC_RESOLUTION_BITS);
    if (!_attached) {
        Serial.printf("[ESC_MOTOR_%d] [LOI] Khong the cau hinh LEDC tren GPIO %d @ %u Hz!\n", _id, _pin, _freq_hz);
        return false;
    }

    // Luôn phát xung an toàn 900 µs chuẩn PX4 ngay khi khởi động
    ledcWrite(_pin, usToDuty(ESC_PULSE_DISARM_US));

    Serial.printf("[ESC_MOTOR_%d] Khoi tao thanh cong: GPIO %d | Huong: %s | Freq: %u Hz (T=%u us) | Res: 14-bit\n",
                  _id, _pin, getDirectionStr(), _freq_hz, _period_us);
    return true;
}

bool EscMotor::setFrequency(uint32_t freq_hz) {
    if (freq_hz < 50 || freq_hz > 500) {
        Serial.printf("[ESC_MOTOR_%d] Tan so khong hop le: %u Hz!\n", _id, freq_hz);
        return false;
    }

    _freq_hz   = freq_hz;
    _period_us = 1000000UL / freq_hz;

    ledcDetach(_pin);
    pinMode(_pin, OUTPUT);
    _attached = ledcAttach(_pin, _freq_hz, ESC_RESOLUTION_BITS);
    if (!_attached) {
        Serial.printf("[ESC_MOTOR_%d] [LOI] Khong the doi tan so sang %u Hz!\n", _id, freq_hz);
        return false;
    }

    ledcWrite(_pin, usToDuty(_current_us));
    return true;
}

void EscMotor::writePulseUs(uint16_t pulse_us) {
    _current_us = pulse_us;
    _target_us  = pulse_us;

    if (_current_us <= ESC_PULSE_MIN_US) {
        _throttle_pct = 0.0f;
    } else {
        _throttle_pct = ((float)(_current_us - ESC_PULSE_MIN_US) * 100.0f) / 
                        (float)(ESC_PULSE_MAX_US - ESC_PULSE_MIN_US);
    }
    ledcWrite(_pin, usToDuty(_current_us));
}

void EscMotor::setTargetPulseUs(uint16_t pulse_us) {
    if (pulse_us < ESC_PULSE_MIN_US) pulse_us = ESC_PULSE_MIN_US;
    if (pulse_us > ESC_PULSE_MAX_US) pulse_us = ESC_PULSE_MAX_US;
    _target_us = pulse_us;
}

void EscMotor::setTargetThrottlePct(float pct) {
    if (pct < 0.0f)   pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;

    uint16_t pulse_us = ESC_PULSE_MIN_US + (uint16_t)((pct * (float)(ESC_PULSE_MAX_US - ESC_PULSE_MIN_US)) / 100.0f);
    setTargetPulseUs(pulse_us);
}

void EscMotor::updateRamp(float dt) {
    if (_current_us != _target_us) {
        float max_step = (float)ESC_SLEW_RATE_US_PER_SEC * dt;
        if (max_step < 1.0f) max_step = 1.0f;

        float diff = (float)_target_us - (float)_current_us;
        if (fabsf(diff) <= max_step) {
            _current_us = _target_us;
        } else if (diff > 0) {
            _current_us += (uint16_t)max_step;
        } else {
            _current_us -= (uint16_t)max_step;
        }
        ledcWrite(_pin, usToDuty(_current_us));
    }

    if (_current_us <= ESC_PULSE_MIN_US) {
        _throttle_pct = 0.0f;
    } else {
        _throttle_pct = ((float)(_current_us - ESC_PULSE_MIN_US) * 100.0f) / 
                        (float)(ESC_PULSE_MAX_US - ESC_PULSE_MIN_US);
    }
}

void EscMotor::stop(uint16_t disarm_us) {
    _current_us   = disarm_us;
    _target_us    = disarm_us;
    _throttle_pct = 0.0f;
    ledcWrite(_pin, usToDuty(disarm_us));
}

// ============================================================================
// OOP IMPLEMENTATION: QuadEscController
// ============================================================================
QuadEscController::QuadEscController()
    : _state(ESC_STATE_UNINITIALIZED), _freq_hz(ESC_DEFAULT_FREQ_HZ), _disarmed_pulse_us(ESC_PULSE_DISARM_US),
      _arming_start_ms(0), _last_cmd_ms(0), _last_update_us(0),
      _watchdog_enabled(false), _motor_test_end_ms(0), _motor_test_active(false),
      _active_test_motor(0) {
}

bool QuadEscController::init(uint32_t freq_hz) {
    _freq_hz = freq_hz;
    bool success = true;

    Serial.println("\n[ESC_QUAD] ========================================================");
    Serial.println("[ESC_QUAD] KHOI TAO HE THONG 4 DONG CO QUADCOPTER (CHUAN PX4):");

    // Khởi tạo 4 động cơ theo đúng cấu hình yêu cầu:
    // Motor 1: GPIO 7, CCW (Ngược chiều kim đồng hồ)
    // Motor 2: GPIO 6, CCW (Ngược chiều kim đồng hồ)
    // Motor 3: GPIO 5, CW  (Thuận chiều kim đồng hồ)
    // Motor 4: GPIO 4, CW  (Thuận chiều kim đồng hồ)
    success &= _motors[0].init(1, MOTOR1_PIN, MOTOR_DIR_CCW, freq_hz);
    success &= _motors[1].init(2, MOTOR2_PIN, MOTOR_DIR_CCW, freq_hz);
    success &= _motors[2].init(3, MOTOR3_PIN, MOTOR_DIR_CW,  freq_hz);
    success &= _motors[3].init(4, MOTOR4_PIN, MOTOR_DIR_CW,  freq_hz);

    _state            = ESC_STATE_DISARMED;
    _watchdog_enabled = false;
    _last_update_us   = micros();

    Serial.println("[ESC_QUAD] Tat ca 4 ESC deu duoc khoa o xung 900us (DISARMED).");
    Serial.println("[ESC_QUAD] ========================================================\n");

    return success;
}

bool QuadEscController::setFrequency(uint32_t freq_hz) {
    if (freq_hz < 50 || freq_hz > 500) {
        Serial.printf("[ESC_QUAD] Tan so khong hop le: %u Hz!\n", freq_hz);
        return false;
    }

    _freq_hz = freq_hz;
    bool ok = true;
    for (int i = 0; i < ESC_NUM_MOTORS; i++) {
        ok &= _motors[i].setFrequency(freq_hz);
    }
    Serial.printf("[ESC_QUAD] >>> Da dong bo tan so ca 4 dong co sang %u Hz (PX4)\n", freq_hz);
    return ok;
}

bool QuadEscController::arm() {
    if (_state == ESC_STATE_ARMED) {
        Serial.println("[ESC_QUAD] Ca 4 dong co da o trang thai ARMED tu truoc!");
        return true;
    }

    _state            = ESC_STATE_ARMED;
    _arming_start_ms  = millis();
    _last_cmd_ms      = millis();
    _watchdog_enabled = true;

    for (int i = 0; i < ESC_NUM_MOTORS; i++) {
        _motors[i].setTargetPulseUs(ESC_PULSE_IDLE_US); // 1050 µs idle spin
    }

    Serial.println("\n[ESC_QUAD] ========================================================");
    Serial.println("[ESC_QUAD] >>> CA 4 ESC DA MO KHOA TUC THI (ARMED)! Idle 1050us.");
    Serial.println("[ESC_QUAD] ========================================================\n");
    return true;
}

void QuadEscController::disarm() {
    _state             = ESC_STATE_DISARMED;
    _motor_test_active = false;
    _active_test_motor = 0;
    _watchdog_enabled  = false;

    for (int i = 0; i < ESC_NUM_MOTORS; i++) {
        _motors[i].stop(_disarmed_pulse_us);
    }
    Serial.printf("[ESC_QUAD] >>> DA KHOA AN TOAN CA 4 DONG CO (DISARMED)! Xung ve %dus.\n", _disarmed_pulse_us);
}

bool QuadEscController::setAllThrottlePct(float pct) {
    if (_state != ESC_STATE_ARMED) {
        Serial.println("[ESC_QUAD] Tu choi lenh: Dong co dang DISARMED! Nhap 'arm' truoc khi len ga.");
        return false;
    }

    _motor_test_active = false;
    _active_test_motor = 0;
    for (int i = 0; i < ESC_NUM_MOTORS; i++) {
        _motors[i].setTargetThrottlePct(pct);
    }
    _last_cmd_ms      = millis();
    _watchdog_enabled = true;
    return true;
}

bool QuadEscController::setAllThrottleUs(uint16_t pulse_us) {
    if (_state != ESC_STATE_ARMED) {
        Serial.println("[ESC_QUAD] Tu choi lenh: Dong co dang DISARMED! Nhap 'arm' truoc khi len ga.");
        return false;
    }

    _motor_test_active = false;
    _active_test_motor = 0;
    for (int i = 0; i < ESC_NUM_MOTORS; i++) {
        _motors[i].setTargetPulseUs(pulse_us);
    }
    _last_cmd_ms      = millis();
    _watchdog_enabled = true;
    return true;
}

bool QuadEscController::setMotorThrottlePct(uint8_t motor_id, float pct) {
    if (_state != ESC_STATE_ARMED) {
        Serial.println("[ESC_QUAD] Tu choi lenh: Dong co dang DISARMED! Nhap 'arm' truoc khi len ga.");
        return false;
    }
    if (motor_id < 1 || motor_id > ESC_NUM_MOTORS) {
        Serial.printf("[ESC_QUAD] ID dong co khong hop le: %d (Chi ho tro 1..4)\n", motor_id);
        return false;
    }

    // Nếu đang test motor riêng lẻ khác, bỏ qua để không làm quay motor ngoài ý muốn
    if (_motor_test_active && motor_id != _active_test_motor) {
        return false;
    }

    _motors[motor_id - 1].setTargetThrottlePct(pct);
    _last_cmd_ms      = millis();
    _watchdog_enabled = true;
    return true;
}

bool QuadEscController::setMotorThrottleUs(uint8_t motor_id, uint16_t pulse_us) {
    if (_state != ESC_STATE_ARMED) {
        Serial.println("[ESC_QUAD] Tu choi lenh: Dong co dang DISARMED! Nhap 'arm' truoc khi len ga.");
        return false;
    }
    if (motor_id < 1 || motor_id > ESC_NUM_MOTORS) {
        Serial.printf("[ESC_QUAD] ID dong co khong hop le: %d (Chi ho tro 1..4)\n", motor_id);
        return false;
    }

    if (_motor_test_active && motor_id != _active_test_motor) {
        return false;
    }

    _motors[motor_id - 1].setTargetPulseUs(pulse_us);
    _last_cmd_ms      = millis();
    _watchdog_enabled = true;
    return true;
}

bool QuadEscController::testSingleMotor(uint8_t motor_id, float pct, uint32_t duration_ms) {
    if (motor_id < 1 || motor_id > ESC_NUM_MOTORS) {
        Serial.printf("[ESC_QUAD] ID motor khong hop le: %d (Chi ho tro 1..4)\n", motor_id);
        return false;
    }

    // Nếu đang DISARMED, tự động chuyển sang ARMED để cho phép test motor
    if (_state != ESC_STATE_ARMED) {
        _state = ESC_STATE_ARMED;
    }

    _motor_test_active = true;
    _active_test_motor = motor_id;
    _motor_test_end_ms = millis() + duration_ms;
    _last_cmd_ms       = millis();
    _watchdog_enabled  = false; // Tạm tắt watchdog mất kết nối khi đang test

    // Dừng triệt để tất cả các motor khác về MIN 1000us để ĐỨNG YÊN HOÀN TOÀN (0 RPM)
    for (int i = 0; i < ESC_NUM_MOTORS; i++) {
        if (i == (motor_id - 1)) {
            _motors[i].setTargetThrottlePct(pct);
        } else {
            _motors[i].setTargetPulseUs(ESC_PULSE_MIN_US); // 1000us -> Dừng hẳn (0 RPM)
        }
    }

    Serial.printf("[ESC_TEST] >>> DANG TEST MOTOR %d o %.1f%% ga trong %u ms (3 motor con lai: DUNG YEN 1000us / 0 RPM)\n",
                  motor_id, pct, duration_ms);
    return true;
}

void QuadEscController::update() {
    uint32_t now_ms = millis();
    uint32_t now_us = micros();
    float dt = (float)(now_us - _last_update_us) / 1000000.0f;
    _last_update_us = now_us;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.005f;

    // Kiểm tra hết thời gian Motor Test tự động ngắt
    if (_motor_test_active) {
        if (now_ms >= _motor_test_end_ms) {
            _motor_test_active = false;
            _active_test_motor = 0;
            for (int i = 0; i < ESC_NUM_MOTORS; i++) {
                _motors[i].setTargetPulseUs(ESC_PULSE_MIN_US);
            }
            Serial.println("[ESC_TEST] >>> Ket thuc thoi gian test motor! Da dung toan bo dong co.");
        }
    }

    // A. Quản lý trạng thái Arming (tức thì)
    if (_state == ESC_STATE_ARMING) {
        _state            = ESC_STATE_ARMED;
        _last_cmd_ms      = now_ms;
        _watchdog_enabled = true;
        for (int i = 0; i < ESC_NUM_MOTORS; i++) {
            _motors[i].setTargetPulseUs(ESC_PULSE_IDLE_US);
        }
        return;
    }

    // B. Failsafe Watchdog: Mất kết nối quá 3s khi đang ARMED
    if (_state == ESC_STATE_ARMED && _watchdog_enabled) {
        if (now_ms - _last_cmd_ms > ESC_WATCHDOG_TIMEOUT_MS) {
            _state            = ESC_STATE_FAILSAFE;
            _watchdog_enabled = false;

            for (int i = 0; i < ESC_NUM_MOTORS; i++) {
                _motors[i].stop();
            }

            Serial.println("\n[ESC_QUAD] [CANH BAO AN TOAN] Mat ket noi lenh qua 3s!");
            Serial.println("[ESC_QUAD] >>> Kich hoat FAILSAFE: Ngat ga ca 4 dong co ve 900us!\n");
            return;
        }
    }

    // C. Khóa an toàn DISARMED / FAILSAFE
    if (_state == ESC_STATE_DISARMED || _state == ESC_STATE_FAILSAFE) {
        for (int i = 0; i < ESC_NUM_MOTORS; i++) {
            _motors[i].stop(_disarmed_pulse_us);
        }
        return;
    }

    // D. Chế độ hiệu chuẩn CALIBRATION (xuất xung trực tiếp, bỏ qua Ramp)
    if (_state == ESC_STATE_CALIBRATION) {
        return;
    }

    // E. Cập nhật Soft Ramp Limiter cho từng động cơ
    for (int i = 0; i < ESC_NUM_MOTORS; i++) {
        _motors[i].updateRamp(dt);
    }
}

void QuadEscController::startCalibration() {
    _state            = ESC_STATE_CALIBRATION;
    _watchdog_enabled = false;

    for (int i = 0; i < ESC_NUM_MOTORS; i++) {
        _motors[i].writePulseUs(ESC_PULSE_MAX_US); // 2000 µs
    }

    Serial.println("\n[ESC_CALIB] ========================================================");
    Serial.println("[ESC_CALIB] >>> BAT CHE DO HIEU CHUAN DONG LOAT 4 ESC (THROTTLE RANGE)");
    Serial.println("[ESC_CALIB] 1. ESP32 dang phat xung MAX 2000us tren ca 4 chan (7, 6, 5, 4).");
    Serial.println("[ESC_CALIB] 2. Hay CAM NGUON PIN LiPo vao ESC.");
    Serial.println("[ESC_CALIB] 3. Lang nghe ca 4 ESC phat 2 tieng bip ngan (Bip-Bip).");
    Serial.println("[ESC_CALIB] 4. Ngay sau do, nhap 'calib_finish' tren Serial de luu vao EEPROM!");
    Serial.println("[ESC_CALIB] ========================================================\n");
}

void QuadEscController::finishCalibration() {
    if (_state != ESC_STATE_CALIBRATION) {
        Serial.println("[ESC_CALIB] [LOI] He thong khong o trong che do hieu chuan!");
        return;
    }

    for (int i = 0; i < ESC_NUM_MOTORS; i++) {
        _motors[i].writePulseUs(ESC_PULSE_DISARM_US); // 900 µs
    }
    _state = ESC_STATE_DISARMED;

    Serial.println("\n[ESC_CALIB] ========================================================");
    Serial.println("[ESC_CALIB] >>> DA HA XUNG CA 4 CHAN VE MIN 900us!");
    Serial.println("[ESC_CALIB] Cac ESC se phat 1 tieng bip dai xac nhan da luu vao EEPROM.");
    Serial.println("[ESC_CALIB] Hieu chuan thanh cong! 4 dong co ve DISARMED an toan.");
    Serial.println("[ESC_CALIB] ========================================================\n");
}

void QuadEscController::feedWatchdog() {
    _last_cmd_ms = millis();
}

const char* QuadEscController::getStateStr() const {
    switch (_state) {
        case ESC_STATE_ARMED:       return "ARMED";
        case ESC_STATE_ARMING:      return "ARMING...";
        case ESC_STATE_CALIBRATION: return "CALIB";
        case ESC_STATE_FAILSAFE:    return "FAILSAFE";
        default:                    return "DISARMED";
    }
}

const EscMotor& QuadEscController::getMotor(uint8_t index) const {
    if (index >= ESC_NUM_MOTORS) return _motors[0];
    return _motors[index];
}

EscMotor& QuadEscController::getMotor(uint8_t index) {
    if (index >= ESC_NUM_MOTORS) return _motors[0];
    return _motors[index];
}
