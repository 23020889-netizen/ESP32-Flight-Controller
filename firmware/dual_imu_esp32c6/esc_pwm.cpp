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
      _current_us(ESC_PULSE_MIN_US), _target_us(ESC_PULSE_MIN_US),
      _ramp_us((float)ESC_PULSE_MIN_US),
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
    _current_us  = ESC_PULSE_MIN_US;
    _target_us   = ESC_PULSE_MIN_US;
    _ramp_us     = (float)ESC_PULSE_MIN_US;
    _throttle_pct= 0.0f;

    pinMode(_pin, OUTPUT);
    digitalWrite(_pin, LOW);

    _attached = ledcAttach(_pin, _freq_hz, ESC_RESOLUTION_BITS);
    if (!_attached) {
        Serial.printf("[ESC_MOTOR_%d] [LOI] Khong the cau hinh LEDC tren GPIO %d @ %u Hz!\n", _id, _pin, _freq_hz);
        return false;
    }

    ledcWrite(_pin, usToDuty(ESC_PULSE_MIN_US));
    Serial.printf("[ESC_MOTOR_%d] Khoi tao thanh cong: GPIO %d | Huong: %s | Range: 1000..2000 us | Freq: %u Hz\n",
                  _id, _pin, getDirectionStr(), _freq_hz);
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
    if (pulse_us < ESC_PULSE_MIN_US) pulse_us = ESC_PULSE_MIN_US;
    if (pulse_us > ESC_PULSE_MAX_US) pulse_us = ESC_PULSE_MAX_US;
    _current_us = pulse_us;
    _target_us  = pulse_us;
    _ramp_us    = (float)pulse_us;

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

void EscMotor::setNormalizedCommand(float u, bool immediate) {
    uint16_t pulse = ESC_PULSE_MIN_US;
    if (!(isnan(u) || isinf(u) || u <= 0.001f)) {
        if (u > 1.0f) u = 1.0f;
        pulse = ESC_PULSE_MIN_US + (uint16_t)((u * (float)(ESC_PULSE_MAX_US - ESC_PULSE_MIN_US)) + 0.5f);
    }
    if (immediate) {
        if (pulse != _current_us || pulse != _target_us) writePulseUs(pulse);
    } else {
        setTargetPulseUs(pulse);
    }
}

void EscMotor::setTargetThrottlePct(float pct) {
    if (pct <= 0.2f) {
        setTargetPulseUs(ESC_PULSE_MIN_US);
        return;
    }
    if (pct > 100.0f) pct = 100.0f;
    uint16_t pulse_us = ESC_PULSE_MIN_US + (uint16_t)((pct * (float)(ESC_PULSE_MAX_US - ESC_PULSE_MIN_US)) / 100.0f);
    setTargetPulseUs(pulse_us);
}

void EscMotor::updateRamp(float dt) {
    if (_current_us != _target_us) {
        // Tích lũy dạng float: tốc độ ramp đúng 2500 us/s bất kể tần số loop()
        // (bản cũ ép (uint16_t)max_step >= 1 => tốc độ ramp phụ thuộc tốc độ loop).
        const float max_step = (float)ESC_SLEW_RATE_US_PER_SEC * dt;
        const float diff = (float)_target_us - _ramp_us;
        if (fabsf(diff) <= max_step) {
            _ramp_us = (float)_target_us;
        } else {
            _ramp_us += (diff > 0.0f) ? max_step : -max_step;
        }
        const uint16_t next_us = (uint16_t)(_ramp_us + 0.5f);
        if (next_us != _current_us) {
            _current_us = next_us;
            ledcWrite(_pin, usToDuty(_current_us));
        }
    } else {
        _ramp_us = (float)_current_us;
    }

    if (_current_us <= ESC_PULSE_MIN_US) {
        _throttle_pct = 0.0f;
    } else {
        _throttle_pct = ((float)(_current_us - ESC_PULSE_MIN_US) * 100.0f) /
                        (float)(ESC_PULSE_MAX_US - ESC_PULSE_MIN_US);
    }
}

void EscMotor::stop() {
    _current_us   = ESC_PULSE_MIN_US;
    _target_us    = ESC_PULSE_MIN_US;
    _ramp_us      = (float)ESC_PULSE_MIN_US;
    _throttle_pct = 0.0f;
    ledcWrite(_pin, usToDuty(ESC_PULSE_MIN_US));
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
    Serial.println("[ESC_QUAD] KHOI TAO HE THONG 4 DONG CO QUADCOPTER (CHUAN PX4 1000..2000us):");

    success &= _motors[0].init(1, MOTOR1_PIN, MOTOR_DIR_CCW, freq_hz);
    success &= _motors[1].init(2, MOTOR2_PIN, MOTOR_DIR_CCW, freq_hz);
    success &= _motors[2].init(3, MOTOR3_PIN, MOTOR_DIR_CW,  freq_hz);
    success &= _motors[3].init(4, MOTOR4_PIN, MOTOR_DIR_CW,  freq_hz);

    _state            = success ? ESC_STATE_DISARMED : ESC_STATE_FAILSAFE;
    _watchdog_enabled = false;
    _last_update_us   = micros();

    if (success) {
        Serial.println("[ESC_QUAD] Ca 4 PWM da attach; ca 4 ESC o moc an toan 1000us.");
    } else {
        Serial.println("[ESC_QUAD] [LOI] It nhat mot PWM pin khong attach duoc; ARM bi khoa.");
    }
    Serial.println("[ESC_QUAD] ========================================================\n");

    return success;
}

bool QuadEscController::outputsReady() const {
    for (int i = 0; i < ESC_NUM_MOTORS; ++i) {
        if (!_motors[i].isAttached()) return false;
    }
    return true;
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
    if (!outputsReady()) {
        _state = ESC_STATE_FAILSAFE;
        _watchdog_enabled = false;
        Serial.println("[ESC_QUAD] [LOI] Tu choi ARM: chua attach du ca 4 ngo PWM.");
        return false;
    }
    if (_state == ESC_STATE_ARMED) {
        _last_cmd_ms = millis();
        return true;
    }

    _state            = ESC_STATE_ARMED;
    _arming_start_ms  = millis();
    _last_cmd_ms      = millis();
    _watchdog_enabled = true;

    for (int i = 0; i < ESC_NUM_MOTORS; i++) {
        _motors[i].writePulseUs(ESC_PULSE_MIN_US);
    }

    Serial.println("\n[ESC_QUAD] ========================================================");
    Serial.println("[ESC_QUAD] >>> CA 4 ESC DA MO KHOA HOAN TAT (ARMED)!");
    Serial.printf("[ESC_QUAD]     Motor 1 (GPIO 7 - CCW) : %u us (0%%)\n", ESC_PULSE_MIN_US);
    Serial.printf("[ESC_QUAD]     Motor 2 (GPIO 6 - CCW) : %u us (0%%)\n", ESC_PULSE_MIN_US);
    Serial.printf("[ESC_QUAD]     Motor 3 (GPIO 5 - CW ) : %u us (0%%)\n", ESC_PULSE_MIN_US);
    Serial.printf("[ESC_QUAD]     Motor 4 (GPIO 4 - CW ) : %u us (0%%)\n", ESC_PULSE_MIN_US);
    Serial.println("[ESC_QUAD]     San sang nhan lenh dieu toc ngay lap tuc (0% - 100%)...");
    Serial.println("[ESC_QUAD] ========================================================\n");
    return true;
}

void QuadEscController::disarm() {
    _state            = ESC_STATE_DISARMED;
    _watchdog_enabled = false;

    for (int i = 0; i < ESC_NUM_MOTORS; i++) {
        _motors[i].stop();
    }
    Serial.println("[ESC_QUAD] >>> DA KHOA AN TOAN CA 4 DONG CO (DISARMED)! Dong nhat 1000us.");
}

bool QuadEscController::setAllThrottlePct(float pct) {
    if (_state != ESC_STATE_ARMED) {
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
        return false;
    }

    for (int i = 0; i < ESC_NUM_MOTORS; i++) {
        _motors[i].setTargetPulseUs(pulse_us);
    }
    _last_cmd_ms      = millis();
    _watchdog_enabled = true;
    return true;
}

bool QuadEscController::setAllNormalized(const float u[ESC_NUM_MOTORS]) {
    if (_state != ESC_STATE_ARMED) {
        return false;
    }
    for (int i = 0; i < ESC_NUM_MOTORS; i++) {
        _motors[i].setNormalizedCommand(u[i], true); // Đầu ra vòng kín: không qua Soft-Ramp
    }
    _last_cmd_ms      = millis();
    _watchdog_enabled = true;
    return true;
}

bool QuadEscController::setMotorThrottlePct(uint8_t motor_id, float pct) {
    if (_state != ESC_STATE_ARMED) {
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
        return false;
    }
    if (motor_id < 1 || motor_id > ESC_NUM_MOTORS) {
        Serial.printf("[ESC_QUAD] ID dong co khong hop le: %d (Chi ho tro 1..4)\n", motor_id);
        return false;
    }

    _motors[motor_id - 1].setTargetPulseUs(pulse_us);
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

    // A. Quản lý trạng thái Arming
    if (_state == ESC_STATE_ARMING) {
        for (int i = 0; i < ESC_NUM_MOTORS; i++) {
            _motors[i].writePulseUs(ESC_PULSE_MIN_US);
        }

        if (now_ms - _arming_start_ms >= ESC_ARMING_TIME_MS) {
            _state            = ESC_STATE_ARMED;
            _last_cmd_ms      = now_ms;
            _watchdog_enabled = true;

            for (int i = 0; i < ESC_NUM_MOTORS; i++) {
                _motors[i].setTargetPulseUs(ESC_PULSE_MIN_US);
            }
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
            Serial.println("[ESC_QUAD] >>> Kich hoat FAILSAFE: Ngat ga ca 4 dong co ve 1000us!\n");
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
        _motors[i].writePulseUs(ESC_PULSE_MAX_US);
    }

    Serial.println("\n[ESC_CALIB] ========================================================");
    Serial.println("[ESC_CALIB] >>> BAT CHE DO HIEU CHUAN DONG LOAT 4 ESC (THROTTLE RANGE)");
    Serial.println("[ESC_CALIB] 1. ESP32 dang phat xung MAX (2000us) tren ca 4 chan (7, 6, 5, 4).");
    Serial.println("[ESC_CALIB] 2. Hay CAM NGUON PIN LiPo vao ESC.");
    Serial.println("[ESC_CALIB] 3. Lang nghe ca 4 ESC phat 2 tieng bip ngan (Bip-Bip).");
    Serial.println("[ESC_CALIB] 4. Ngay sau do, nhap 'calib_finish' hoac 'l' tren Serial de luu!");
    Serial.println("[ESC_CALIB] ========================================================\n");
}

void QuadEscController::finishCalibration() {
    for (int i = 0; i < ESC_NUM_MOTORS; i++) {
        _motors[i].writePulseUs(ESC_PULSE_MIN_US);
    }
    _state = ESC_STATE_ARMED;
    _last_cmd_ms = millis();
    _watchdog_enabled = false;

    Serial.println("\n[ESC_CALIB] ========================================================");
    Serial.println("[ESC_CALIB] >>> DA HA XUNG CA 4 CHAN VE MOC 0% (Dong nhat M1..M4 = 1000us)!");
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
