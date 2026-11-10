#ifndef ESC_PWM_H
#define ESC_PWM_H

#include <Arduino.h>

// ============================================================================
// 1. PX4 AUTOPILOT STANDARD ESC PWM SPECIFICATIONS
// ============================================================================
#define ESC_NUM_MOTORS           4

// CẤU HÌNH CHÂN GPIO & ĐỘNG CƠ THEO YÊU CẦU:
// Motor 1: GPIO 7 (CCW - Ngược chiều kim đồng hồ)
// Motor 2: GPIO 6 (CCW - Ngược chiều kim đồng hồ)
// Motor 3: GPIO 5 (CW  - Thuận chiều kim đồng hồ)
// Motor 4: GPIO 4 (CW  - Thuận chiều kim đồng hồ)
#define MOTOR1_PIN               7
#define MOTOR2_PIN               6
#define MOTOR3_PIN               5
#define MOTOR4_PIN               4

#define ESC_DEFAULT_FREQ_HZ      50       // 50 Hz: Tương thích 100% mọi dòng ESC Vàng 30A (có thể chuyển sang 400Hz qua lệnh 'freq 400')
#define ESC_PX4_FREQ_HZ          400      // 400 Hz: Chuẩn PX4 Multirotor (Chu kỳ 2500 µs)
#define ESC_RESOLUTION_BITS      14       // Độ phân giải phần cứng 14-bit (16384 nấc, bước nhảy siêu mịn 0.15 µs)

// CẤU HÌNH DẢI XUNG ESC THEO PHẦN CỨNG THỰC TẾ CỦA NGƯỜI DÙNG:
// - Motor 1, 2, 4 (ESC Vàng 30A): Min/Disarm = 1000 µs, SpinMin = 1000 µs, Max = 2000 µs
// - Motor 3 (GPIO 5 - ESC Đen 40A): Min/Disarm = 850 µs, SpinMin = 890 µs, Max = 1000 µs
//   (Khi ở 0% ga xuất 850µs để nhận Arm; ngay khi ga > 0% nhảy qua vùng chết lên 890..1000µs để quay đồng tốc ngay từ 3-5% ga!)
#define ESC_PULSE_DISARM_US      1000     // Xung khóa an toàn mặc định cho ESC Vàng (1000 µs)
#define ESC_PULSE_MIN_US         1000     // Xung ga tối thiểu chuẩn (1000 µs = 0.0% Throttle)
#define ESC_PULSE_MAX_US         2000     // Xung ga tối đa chuẩn (2000 µs = 100.0% Throttle)
#define ESC_M3_PULSE_MIN_US      850      // Xung khóa / nhận Arm cho Motor 3 - ESC Đen 40A (850 µs = 0.0% Throttle)
#define ESC_M3_PULSE_SPIN_US     890      // Ngưỡng bắt đầu quay của Motor 3 - ESC Đen 40A (bỏ qua vùng chết 850..890 µs)
#define ESC_M3_PULSE_MAX_US      1000     // Xung ga tối đa riêng cho Motor 3 - ESC Đen 40A (1000 µs = 100.0% Throttle)
#define ESC_PULSE_IDLE_US        1050     // Xung ga quay chậm không tải (Idle Spin ~ 5%)

#define ESC_ARMING_TIME_MS       0        // Mở khóa ARM tức thì (vì init() đã giữ mốc 0% từ lúc khởi động)
#define ESC_WATCHDOG_TIMEOUT_MS  3000     // Failsafe: Tự động ngắt ga về 0% nếu mất lệnh điều khiển quá 3s
#define ESC_SLEW_RATE_US_PER_SEC 2500     // Tốc độ tăng giảm ga an toàn (Soft Ramp): 2500 µs/s

// ============================================================================
// 2. ENUMS
// ============================================================================
enum MotorDirection {
    MOTOR_DIR_CCW = 0, // Ngược chiều kim đồng hồ (Counter-Clockwise)
    MOTOR_DIR_CW  = 1  // Thuận chiều kim đồng hồ (Clockwise)
};

enum EscState {
    ESC_STATE_UNINITIALIZED = 0,
    ESC_STATE_DISARMED,       // Động cơ bị khóa an toàn (xung 900µs, không nhận lệnh ga)
    ESC_STATE_ARMING,         // Đang gửi chuỗi xung 1000µs chờ ESC phát tiếng bíp sẵn sàng
    ESC_STATE_ARMED,          // Đã mở khóa, cho phép điều khiển ga 0% - 100%
    ESC_STATE_CALIBRATION,    // Chế độ hiệu chuẩn hành trình ga Max/Min
    ESC_STATE_FAILSAFE        // Trạng thái ngắt khẩn cấp khi mất kết nối/lỗi an toàn
};

// ============================================================================
// 3. OOP CLASS: EscMotor (Đại diện cho từng kênh động cơ đơn lẻ)
// ============================================================================
class EscMotor {
public:
    EscMotor();

    // Khởi tạo kênh động cơ với ID, chân GPIO, chiều quay, tần số và dải xung Min/SpinMin/Max riêng
    bool init(uint8_t id, uint8_t pin, MotorDirection dir, uint32_t freq_hz,
              uint16_t min_us = ESC_PULSE_MIN_US, uint16_t max_us = ESC_PULSE_MAX_US,
              uint16_t spin_min_us = 0);

    // Cấu hình lại dải xung Min/Max riêng cho ESC
    void setPulseRange(uint16_t min_us, uint16_t max_us, uint16_t spin_min_us = 0);
    uint16_t getMinPulseUs() const     { return _min_us; }
    uint16_t getSpinMinPulseUs() const { return _spin_min_us; }
    uint16_t getMaxPulseUs() const     { return _max_us; }

    // Cập nhật lại tần số xung PWM phần cứng
    bool setFrequency(uint32_t freq_hz);

    // Xuất xung trực tiếp ra phần cứng (bỏ qua lọc Ramp)
    void writePulseUs(uint16_t pulse_us);

    // Đặt mục tiêu xung ga (qua bộ lọc Soft Ramp)
    void setTargetPulseUs(uint16_t pulse_us);

    // Đặt mục tiêu ga theo phần trăm (0.0% .. 100.0%)
    void setTargetThrottlePct(float pct);

    // Cập nhật tốc độ tăng ga từ từ (Soft Ramp Limiter)
    void updateRamp(float dt);

    // Dừng khẩn cấp động cơ (xuất 900 µs ngay lập tức)
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

private:
    uint8_t        _id;
    uint8_t        _pin;
    MotorDirection _direction;
    uint32_t       _freq_hz;
    uint32_t       _period_us;
    uint32_t       _max_duty;
    uint16_t       _min_us;
    uint16_t       _spin_min_us;
    uint16_t       _max_us;
    uint16_t       _current_us;
    uint16_t       _target_us;
    float          _throttle_pct;
    bool           _attached;

    uint32_t usToDuty(uint16_t pulse_us) const;
};

// ============================================================================
// 4. OOP CLASS: QuadEscController (Quản lý 4 động cơ Quadcopter)
// ============================================================================
class QuadEscController {
public:
    QuadEscController();

    // Khởi tạo cả 4 động cơ: M1(7-CCW), M2(6-CCW), M3(5-CW), M4(4-CW)
    bool init(uint32_t freq_hz = ESC_DEFAULT_FREQ_HZ);

    // Đổi tần số PWM đồng loạt cho 4 động cơ (50Hz hoặc 400Hz)
    bool setFrequency(uint32_t freq_hz);

    // Bắt đầu chuỗi mở khóa an toàn (Arming) chuẩn PX4
    bool arm();

    // Khóa an toàn khẩn cấp cả 4 động cơ (ngắt ga về 900µs)
    void disarm();

    // Đặt ga đồng loạt cả 4 động cơ theo phần trăm (0.0% .. 100.0%)
    bool setAllThrottlePct(float pct);

    // Đặt ga đồng loạt cả 4 động cơ theo microsecond (1000 .. 2000 µs)
    bool setAllThrottleUs(uint16_t pulse_us);

    // Đặt ga riêng cho từng động cơ (motor_id: 1, 2, 3, 4)
    bool setMotorThrottlePct(uint8_t motor_id, float pct);
    bool setMotorThrottleUs(uint8_t motor_id, uint16_t pulse_us);

    // Cập nhật vòng lặp loop() cho FSM, Soft Ramp và Watchdog
    void update();

    // Hiệu chuẩn hành trình ga ESC đồng loạt 4 kênh
    void startCalibration();
    void finishCalibration();

    // Nuôi Watchdog an toàn
    void feedWatchdog();

    // Getters
    EscState        getState() const         { return _state; }
    const char*     getStateStr() const;
    uint32_t        getFrequency() const     { return _freq_hz; }
    const EscMotor& getMotor(uint8_t index) const; // index 0..3 (tương ứng Motor 1..4)
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

// Đối tượng Controller toàn cục
extern QuadEscController quadMotors;

#endif // ESC_PWM_H
