#ifndef ESC_PWM_H
#define ESC_PWM_H

#include <Arduino.h>

// ============================================================================
// 1. PX4 AUTOPILOT STANDARD ESC PWM SPECIFICATIONS (UNIFORM 4 MOTORS)
// ============================================================================
#define ESC_NUM_MOTORS           4

// CẤU HÌNH CHÂN GPIO ĐỘNG CƠ (Quad-X - Đồng bộ 100% 4 ESC Vàng 30A):
#define MOTOR1_PIN               7  // Front-Left  (CCW - Trái)
#define MOTOR2_PIN               6  // Rear-Right  (CCW - Phải)
#define MOTOR3_PIN               5  // Rear-Left   (CW  - Trái)
#define MOTOR4_PIN               4  // Front-Right (CW  - Phải)

#define ESC_DEFAULT_FREQ_HZ      50       // 50 Hz tiêu chuẩn
#define ESC_PX4_FREQ_HZ          50       // Giữ cố định 50 Hz cho ESC vàng 30A tiêu chuẩn
#define ESC_RESOLUTION_BITS      14       // Độ phân giải 14-bit (16384 nấc)

// Cả 4 ESC dùng chung dải xung chuẩn 1000..2000 µs (Section 1 & 7 Rebuild Guide):
#define ESC_PULSE_DISARM_US      1000     // Xung ngắt an toàn (0% ga)
#define ESC_PULSE_MIN_US         1000     // Xung tối thiểu (0.0% Throttle)
#define ESC_PULSE_MAX_US         2000     // Xung tối đa (100.0% Throttle)
#define ESC_PULSE_IDLE_US        1080     // Xung quay chậm không tải (Idle Spin ~ 8% - 1080us đảm bảo A2212 luôn quay ổn định)

#define ESC_ARMING_TIME_MS       0        // Mở khóa ARM tức thì
#define ESC_WATCHDOG_TIMEOUT_MS  3000     // Failsafe: ngắt ga nếu mất lệnh > 3s
#define ESC_SLEW_RATE_US_PER_SEC 20000    // Soft Ramp: 20000 µs/s (triệt tiêu trễ pha 40ms, giúp vòng PID phản ứng tức thì)

// ============================================================================
// 2. ENUMS
// ============================================================================
enum MotorDirection {
    MOTOR_DIR_CCW = 0,
    MOTOR_DIR_CW  = 1
};

enum EscState {
    ESC_STATE_UNINITIALIZED = 0,
    ESC_STATE_DISARMED,
    ESC_STATE_ARMING,
    ESC_STATE_ARMED,
    ESC_STATE_CALIBRATION,
    ESC_STATE_FAILSAFE
};

// ============================================================================
// 3. OOP CLASS: EscMotor
// ============================================================================
class EscMotor {
public:
    EscMotor();

    bool init(uint8_t id, uint8_t pin, MotorDirection dir, uint32_t freq_hz);
    bool setFrequency(uint32_t freq_hz);

    void writePulseUs(uint16_t pulse_us);
    void setTargetPulseUs(uint16_t pulse_us);
    void setNormalizedCommand(float u);
    void setTargetThrottlePct(float pct);
    void updateRamp(float dt);
    void stop();

    // Getters
    uint8_t        getId() const             { return _id; }
    uint8_t        getPin() const            { return _pin; }
    MotorDirection getDirection() const      { return _direction; }
    const char*    getDirectionStr() const   { return (_direction == MOTOR_DIR_CCW) ? "CCW" : "CW"; }
    uint16_t       getCurrentPulseUs() const { return _current_us; }
    uint16_t       getTargetPulseUs() const  { return _target_us; }
    float          getThrottlePct() const    { return _throttle_pct; }
    bool           isAttached() const        { return _attached; }
    uint16_t       getMinPulseUs() const     { return ESC_PULSE_MIN_US; }
    uint16_t       getMaxPulseUs() const     { return ESC_PULSE_MAX_US; }

private:
    uint8_t        _id;
    uint8_t        _pin;
    MotorDirection _direction;
    uint32_t       _freq_hz;
    uint32_t       _period_us;
    uint32_t       _max_duty;
    uint16_t       _current_us;
    uint16_t       _target_us;
    float          _throttle_pct;
    bool           _attached;

    uint32_t usToDuty(uint16_t pulse_us) const;
};

// ============================================================================
// 4. OOP CLASS: QuadEscController
// ============================================================================
class QuadEscController {
public:
    QuadEscController();

    bool init(uint32_t freq_hz = ESC_DEFAULT_FREQ_HZ);
    bool outputsReady() const;
    bool setFrequency(uint32_t freq_hz);
    bool arm();
    void disarm();

    bool setAllThrottlePct(float pct);
    bool setAllThrottleUs(uint16_t pulse_us);
    bool setAllNormalized(const float u[ESC_NUM_MOTORS]);
    bool setMotorThrottlePct(uint8_t motor_id, float pct);
    bool setMotorThrottleUs(uint8_t motor_id, uint16_t pulse_us);

    void update();
    void startCalibration();
    void finishCalibration();
    void feedWatchdog();

    // Getters
    EscState        getState() const         { return _state; }
    const char*     getStateStr() const;
    uint32_t        getFrequency() const     { return _freq_hz; }
    const EscMotor& getMotor(uint8_t index) const;
    EscMotor&       getMotor(uint8_t index);

private:
    EscMotor _motors[ESC_NUM_MOTORS];
    EscState _state;
    uint32_t _freq_hz;
    uint32_t _arming_start_ms;
    uint32_t _last_cmd_ms;
    uint32_t _last_update_us;
    bool     _watchdog_enabled;
};

extern QuadEscController quadMotors;

#endif // ESC_PWM_H
