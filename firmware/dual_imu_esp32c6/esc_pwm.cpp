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
      _min_us(ESC_PULSE_MIN_US), _max_us(ESC_PULSE_MAX_US),
      _current_us(ESC_PULSE_MIN_US), _target_us(ESC_PULSE_MIN_US),
      _throttle_pct(0.0f), _attached(false) {
}

uint32_t EscMotor::usToDuty(uint16_t pulse_us) const {
    if (pulse_us > _period_us) {
        pulse_us = (uint16_t)_period_us;
    }
    return (uint32_t)(((uint64_t)pulse_us * _max_duty) / _period_us);
}

bool EscMotor::init(uint8_t id, uint8_t pin, MotorDirection dir, uint32_t freq_hz,
                    uint16_t min_us, uint16_t max_us) {
    _id          = id;
    _pin         = pin;
    _direction   = dir;
    _freq_hz     = freq_hz;
    _period_us   = 1000000UL / freq_hz;
    _max_duty    = (1UL << ESC_RESOLUTION_BITS) - 1UL;
    _min_us      = min_us;
    _max_us      = max_us;
    _current_us  = _min_us;
    _target_us   = _min_us;
    _throttle_pct= 0.0f;

    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);

    // Gán kênh phần cứng LEDC Timer trên ESP32 Core 3.x
    _attached = ledcAttach(_pin, _freq_hz, ESC_RESOLUTION_BITS);
    if (!_attached) {
        Serial.printf("[ESC_MOTOR_%d] [LOI] Khong the cau hinh LEDC tren GPIO %d @ %u Hz!\n", _id, _pin, _freq_hz);
        return false;
    }

    // Phát ngay mức xung khóa/0% riêng của từng loại ESC (M1,2,4=1000us | M3=850us)
    ledcWrite(_pin, usToDuty(_min_us));

    Serial.printf("[ESC_MOTOR_%d] Khoi tao thanh cong: GPIO %d | Huong: %s | Dai xung: %u..%u us | Freq: %u Hz\n",
                  _id, _pin, getDirectionStr(), _min_us, _max_us, _freq_hz);
    return true;
}

void EscMotor::setPulseRange(uint16_t min_us, uint16_t max_us) {
    if (min_us >= 750 && max_us > min_us && max_us <= 2250) {
        _min_us = min_us;
        _max_us = max_us;
    }
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

    if (_current_us <= _min_us) {
        _throttle_pct = 0.0f;
    } else {
        _throttle_pct = ((float)(_current_us - _min_us) * 100.0f) /
                        (float)(_max_us - _min_us);
    }
    ledcWrite(_pin, usToDuty(_current_us));
}

void EscMotor::setTargetPulseUs(uint16_t pulse_us) {
    if (pulse_us < _min_us) pulse_us = _min_us;
    if (pulse_us > _max_us) pulse_us = _max_us;
    _target_us = pulse_us;
}

void EscMotor::setTargetThrottlePct(float pct) {
    if (pct < 0.0f)   pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;

    uint16_t pulse_us = _min_us + (uint16_t)((pct * (float)(_max_us - _min_us)) / 100.0f);
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

    if (_current_us <= _min_us) {
        _throttle_pct = 0.0f;
    } else {
        _throttle_pct = ((float)(_current_us - _min_us) * 100.0f) /
                        (float)(_max_us - _min_us);
    }
}

void EscMotor::stop() {
    _current_us   = _min_us;
    _target_us    = _min_us;
    _throttle_pct = 0.0f;
    ledcWrite(_pin, usToDuty(_min_us));
}

// ============================================================================
// OOP IMPLEMENTATION: QuadEscController
// ============================================================================
QuadEscController::QuadEscController()
    : _state(ESC_STATE_UNINITIALIZED), _freq_hz(ESC_DEFAULT_FREQ_HZ),
      _arming_start_ms(0), _last_cmd_ms(0), _last_update_us(0),
      _watchdog_enabled(false) {
}

bool QuadEscController::init(uint32_t freq_hz) {
    _freq_hz = freq_hz;
    bool success = true;

    Serial.println("\n[ESC_QUAD] ========================================================");
    Serial.println("[ESC_QUAD] KHOI TAO HE THONG 4 DONG CO QUADCOPTER (CHUAN PX4):");

    // Khởi tạo 4 động cơ theo đúng cấu hình phần cứng hỗn hợp của người dùng:
    // Motor 1: GPIO 7, CCW - ESC Vàng 30A (1000..2000 us)
    // Motor 2: GPIO 6, CCW - ESC Vàng 30A (1000..2000 us)
    // Motor 3: GPIO 5, CW  - ESC Đen  40A ( 850..1850 us)
    // Motor 4: GPIO 4, CW  - ESC Vàng 30A (1000..2000 us)
    success &= _motors[0].init(1, MOTOR1_PIN, MOTOR_DIR_CCW, freq_hz, ESC_PULSE_MIN_US,    ESC_PULSE_MAX_US);
    success &= _motors[1].init(2, MOTOR2_PIN, MOTOR_DIR_CCW, freq_hz, ESC_PULSE_MIN_US,    ESC_PULSE_MAX_US);
    success &= _motors[2].init(3, MOTOR3_PIN, MOTOR_DIR_CW,  freq_hz, ESC_M3_PULSE_MIN_US, ESC_M3_PULSE_MAX_US);
    success &= _motors[3].init(4, MOTOR4_PIN, MOTOR_DIR_CW,  freq_hz, ESC_PULSE_MIN_US,    ESC_PULSE_MAX_US);

    _state            = ESC_STATE_DISARMED;
    _watchdog_enabled = false;
    _last_update_us   = micros();

    Serial.println("[ESC_QUAD] Ca 4 ESC da o moc 0% an toan (M1,2,4=1000us | M3=850us).");
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
        _last_cmd_ms = millis();
        return true;
    }

    _state            = ESC_STATE_ARMED;
    _arming_start_ms  = millis();
    _last_cmd_ms      = millis();
    _watchdog_enabled = true;

    for (int i = 0; i < ESC_NUM_MOTORS; i++) {
        _motors[i].writePulseUs(_motors[i].getMinPulseUs()); // M1,2,4=1000us | M3=850us (0% ga - đứng yên)
    }

    Serial.println("\n[ESC_QUAD] ========================================================");
    Serial.println("[ESC_QUAD] >>> CA 4 ESC DA MO KHOA HOAN TAT (ARMED)!");
    Serial.printf("[ESC_QUAD]     Motor 1 (GPIO 7 - CCW - ESC Vang) : %u us (0%%)\n", _motors[0].getMinPulseUs());
    Serial.printf("[ESC_QUAD]     Motor 2 (GPIO 6 - CCW - ESC Vang) : %u us (0%%)\n", _motors[1].getMinPulseUs());
    Serial.printf("[ESC_QUAD]     Motor 3 (GPIO 5 - CW  - ESC Den)  : %u us (0%%)\n", _motors[2].getMinPulseUs());
    Serial.printf("[ESC_QUAD]     Motor 4 (GPIO 4 - CW  - ESC Vang) : %u us (0%%)\n", _motors[3].getMinPulseUs());
    Serial.println("[ESC_QUAD]     San sang nhan lenh dieu toc ngay lap tuc (0% - 100%)...");
    Serial.println("[ESC_QUAD] ========================================================\n");
    return true;
}

void QuadEscController::disarm() {
    _state            = ESC_STATE_DISARMED;
    _watchdog_enabled = false;

    for (int i = 0; i < ESC_NUM_MOTORS; i++) {
        _motors[i].stop(); // M1,2,4=1000us | M3=850us
    }
    Serial.println("[ESC_QUAD] >>> DA KHOA AN TOAN CA 4 DONG CO (DISARMED)! M1,2,4=1000us | M3=850us.");
}

bool QuadEscController::setAllThrottlePct(float pct) {
    if (_state != ESC_STATE_ARMED) {
        Serial.println("[ESC_QUAD] Tu choi lenh: Dong co dang DISARMED! Nhap 'arm' truoc khi len ga.");
        return false;
    }

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

    EscMotor& m = _motors[motor_id - 1];
    // Tự động dịch dải chuẩn (1000..2000us từ PID) sang dải riêng của ESC (VD: M3 850..1850us)
    if (m.getMinPulseUs() != ESC_PULSE_MIN_US && pulse_us >= ESC_PULSE_MIN_US) {
        int32_t offset = (int32_t)m.getMinPulseUs() - (int32_t)ESC_PULSE_MIN_US;
        int32_t mapped = (int32_t)pulse_us + offset;
        if (mapped < (int32_t)m.getMinPulseUs()) mapped = m.getMinPulseUs();
        if (mapped > (int32_t)m.getMaxPulseUs()) mapped = m.getMaxPulseUs();
        pulse_us = (uint16_t)mapped;
    }

    m.setTargetPulseUs(pulse_us);
    _last_cmd_ms      = millis();
    _watchdog_enabled = true;
    return true;
}

void QuadEscController::update() {
    uint32_t now_ms = millis();
    uint32_t now_us = micros();
    float dt = (float)(now_us - _last_update_us) / 1000000.0f;
    _last_update_us = now_us;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.005f;

    // A. Quản lý trạng thái Arming (Giữ mức 0% riêng của từng ESC)
    if (_state == ESC_STATE_ARMING) {
        for (int i = 0; i < ESC_NUM_MOTORS; i++) {
            _motors[i].writePulseUs(_motors[i].getMinPulseUs());
        }

        if (now_ms - _arming_start_ms >= ESC_ARMING_TIME_MS) {
            _state            = ESC_STATE_ARMED;
            _last_cmd_ms      = now_ms;
            _watchdog_enabled = true;

            for (int i = 0; i < ESC_NUM_MOTORS; i++) {
                _motors[i].setTargetPulseUs(_motors[i].getMinPulseUs()); // 0% ga - ĐỨNG YÊN HOÀN TOÀN KHI VỪA ARM
            }

            Serial.println("\n[ESC_QUAD] ========================================================");
            Serial.println("[ESC_QUAD] >>> CA 4 ESC DA MO KHOA HOAN TAT (ARMED)!");
            Serial.printf("[ESC_QUAD]     Motor 1 (GPIO 7 - CCW - ESC Vang) : %u us (0%%)\n", _motors[0].getMinPulseUs());
            Serial.printf("[ESC_QUAD]     Motor 2 (GPIO 6 - CCW - ESC Vang) : %u us (0%%)\n", _motors[1].getMinPulseUs());
            Serial.printf("[ESC_QUAD]     Motor 3 (GPIO 5 - CW  - ESC Den)  : %u us (0%%)\n", _motors[2].getMinPulseUs());
            Serial.printf("[ESC_QUAD]     Motor 4 (GPIO 4 - CW  - ESC Vang) : %u us (0%%)\n", _motors[3].getMinPulseUs());
            Serial.println("[ESC_QUAD]     San sang nhan lenh dieu toc (0% - 100%)...");
            Serial.println("[ESC_QUAD] ========================================================\n");
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
            _motors[i].stop();
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
        _motors[i].writePulseUs(_motors[i].getMaxPulseUs()); // M1,2,4=2000us | M3=1850us
    }

    Serial.println("\n[ESC_CALIB] ========================================================");
    Serial.println("[ESC_CALIB] >>> BAT CHE DO HIEU CHUAN DONG LOAT 4 ESC (THROTTLE RANGE)");
    Serial.println("[ESC_CALIB] 1. ESP32 dang phat xung MAX tren ca 4 chan (7, 6, 5, 4).");
    Serial.println("[ESC_CALIB] 2. Hay CAM NGUON PIN LiPo vao ESC.");
    Serial.println("[ESC_CALIB] 3. Lang nghe ca 4 ESC phat 2 tieng bip ngan (Bip-Bip).");
    Serial.println("[ESC_CALIB] 4. Ngay sau do, nhap 'L' hoac 'calib_finish' tren Serial de luu!");
    Serial.println("[ESC_CALIB] ========================================================\n");
}

void QuadEscController::finishCalibration() {
    for (int i = 0; i < ESC_NUM_MOTORS; i++) {
        _motors[i].writePulseUs(_motors[i].getMinPulseUs()); // M1,2,4=1000us | M3=850us
    }
    _state = ESC_STATE_ARMED;
    _last_cmd_ms = millis();
    _watchdog_enabled = false;

    Serial.println("\n[ESC_CALIB] ========================================================");
    Serial.println("[ESC_CALIB] >>> DA HA XUNG CA 4 CHAN VE MOC 0% (M1,2,4=1000us | M3=850us)!");
    Serial.println("[ESC_CALIB] Cac ESC se phat 1 tieng bip dai xac nhan da luu vao EEPROM.");
    Serial.println("[ESC_CALIB] Hieu chuan thanh cong! 4 dong co da san sang.");
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
