#ifndef PX4_PID_H
#define PX4_PID_H

#include <Arduino.h>
#include "px4_filters.h"
#include "esc_pwm.h"

// Hằng số chuyển đổi góc
#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.295779513082320876798154814105f
#endif

/**
 * @brief Cấu trúc lưu trữ hệ số PID cho 1 trục (Chuẩn đơn vị Radian/giây)
 */
struct PidGains {
    float kp;
    float ki;
    float kd;
    float i_max;    // Giới hạn chống tràn tích phân (Anti-windup limit)
};

/**
 * @brief Bộ điều khiển PID tốc độ góc (Angular Rate PID Loop) chuẩn PX4
 * Tích hợp Derivative on Measurement và bộ lọc Low-pass 1st-order cho D-term.
 * Đơn vị tính toán: Radian/giây (rad/s)
 */
class PX4_RatePID {
public:
    PX4_RatePID();

    void init(float kp, float ki, float kd, float i_max, float d_cutoff_hz, float loop_rate_hz);
    void setGains(float kp, float ki, float kd, float i_max = 0.3f);
    void setLoopRate(float loop_rate_hz);

    /**
     * @brief Tính toán ngõ ra mô-men [-1.0, 1.0]
     * @param rate_sp Tốc độ góc mục tiêu (rad/s)
     * @param rate_meas Tốc độ góc đo được từ IMU (rad/s)
     * @param dt Khoảng thời gian chu kỳ (giây)
     * @param allow_integral Cho phép tích phân hay không
     */
    float update(float rate_sp, float rate_meas, float dt, bool allow_integral);

    void reset();

    // Getters
    float getP() const { return _p_out; }
    float getI() const { return _integral; }
    float getD() const { return _d_out; }
    const PidGains& getGains() const { return _gains; }

private:
    PidGains _gains;
    float _integral;
    float _last_rate_meas;
    float _p_out;
    float _d_out;
    LowPassFilter1p _d_lpf;
    bool  _first_run;
};

/**
 * @brief Bộ điều khiển góc (Attitude Angle P Controller) chuẩn PX4
 * Chuyển đổi sai số góc nghiêng (Radian) thành tốc độ góc mong muốn (rad/s)
 */
class PX4_AttitudeControl {
public:
    PX4_AttitudeControl();

    void setAngleGains(float roll_p, float pitch_p, float yaw_p);
    void setMaxRates(float max_roll_rate_rad, float max_pitch_rate_rad, float max_yaw_rate_rad);

    /**
     * @brief Tính toán tốc độ góc mục tiêu
     * @param target_euler [Roll, Pitch, Yaw] mục tiêu (Độ)
     * @param actual_euler [Roll, Pitch, Yaw] thực tế từ PX4 EKF (Độ)
     * @param rate_sp_out [Roll, Pitch, Yaw] tốc độ góc đầu ra (rad/s)
     */
    void update(const float target_euler[3], const float actual_euler[3], float rate_sp_out[3]);

    void reset();

private:
    float _roll_p;
    float _pitch_p;
    float _yaw_p;

    float _max_roll_rate_rad;
    float _max_pitch_rate_rad;
    float _max_yaw_rate_rad;

    float wrapAngle180(float angle_deg);
};

/**
 * @brief Bộ phối trộn 4 động cơ Quad-X chuẩn PX4 kèm thuật toán chống bão hòa (Desaturation)
 */
class PX4_QuadMixer {
public:
    PX4_QuadMixer();

    /**
     * @brief Phối trộn ga và mô-men 3 trục ra xung cho 4 động cơ
     * @param throttle Mức ga cơ bản [0.0 .. 1.0]
     * @param roll_torque Mô-men nghiêng ngang [-1.0 .. 1.0]
     * @param pitch_torque Mô-men chúc ngóc [-1.0 .. 1.0]
     * @param yaw_torque Mô-men xoay thân [-1.0 .. 1.0]
     * @param motor_out_us Mảng đầu ra 4 kênh xung (1000 .. 2000 us)
     */
    void mix(float throttle, float roll_torque, float pitch_torque, float yaw_torque, uint16_t motor_out_us[4]);
};

/**
 * @brief Bộ điều khiển bay toàn diện PX4 Multicopter Flight Controller
 * Tích hợp bộ lọc 2nd Butterworth IMU, Cascaded PID (Angle P -> Rate PID), và Motor Mixer
 */
class PX4_FlightController {
public:
    PX4_FlightController();

    // Khởi tạo bộ lọc và PID với tần số chu kỳ (mặc định 200Hz - 5ms)
    void init(float loop_freq_hz = 200.0f);

    // Cài đặt góc mục tiêu (Setpoint)
    void setTargetAttitude(float roll_deg, float pitch_deg, float yaw_deg);
    void lockCurrentHeading(float current_yaw_deg);
    void setPitchInversion(bool invert) { _invert_pitch = invert; }
    bool isPitchInverted() const { return _invert_pitch; }
    void togglePitchInversion() { _invert_pitch = !_invert_pitch; } // Khóa hướng Yaw hiện tại khi Arm
    void setTargetThrottle(float throttle_pct);     // 0.0 .. 100.0%

    // Chế độ test thủ công được quản lý trực tiếp qua quadMotors (QuadEscController)

    // Cài đặt hệ số PID thời gian thực
    void setRatePidRoll(float kp, float ki, float kd);
    void setRatePidPitch(float kp, float ki, float kd);
    void setRatePidYaw(float kp, float ki, float kd);
    void setAngleGains(float roll_p, float pitch_p, float yaw_p);

    /**
     * @brief Chạy chu trình điều khiển bay thời gian thực
     * @param actual_roll Góc Roll từ EKF (độ)
     * @param actual_pitch Góc Pitch từ EKF (độ)
     * @param actual_yaw Góc Yaw từ EKF (độ)
     * @param raw_gx Tốc độ góc Gyro X (deg/s)
     * @param raw_gy Tốc độ góc Gyro Y (deg/s)
     * @param raw_gz Tốc độ góc Gyro Z (deg/s)
     * @param dt Chu kỳ thời gian (giây)
     * @param is_armed Trạng thái đã mở khóa động cơ hay chưa
     */
    void update(float actual_roll, float actual_pitch, float actual_yaw,
                float raw_gx, float raw_gy, float raw_gz,
                float dt, bool is_armed);

    void reset();

    // Getters cho giám sát và Telemetry
    float getFilteredGx() const { return _filtered_gx; }
    float getFilteredGy() const { return _filtered_gy; }
    float getFilteredGz() const { return _filtered_gz; }

    void getTargetAttitude(float& r, float& p, float& y) const {
        r = _target_roll; p = _target_pitch; y = _target_yaw;
    }
    float getTargetThrottle() const { return _target_throttle * 100.0f; }

    const uint16_t* getMotorOutputsUs() const { return _motor_outputs_us; }

    // Truy cập trực tiếp các bộ PID
    PX4_RatePID& getRatePidRoll()  { return _rate_pid_roll; }
    PX4_RatePID& getRatePidPitch() { return _rate_pid_pitch; }
    PX4_RatePID& getRatePidYaw()   { return _rate_pid_yaw; }

private:
    float _target_roll;
    float _target_pitch;
    float _target_yaw;
    float _target_throttle; // 0.0 .. 1.0

    // Bộ lọc thông thấp bậc 2 Butterworth cho con quay hồi chuyển Gyro
    LowPassFilter2p _gyro_filter_x;
    LowPassFilter2p _gyro_filter_y;
    LowPassFilter2p _gyro_filter_z;

    float _filtered_gx;
    float _filtered_gy;
    float _filtered_gz;

    // Vòng ngoài: Góc sang tốc độ góc
    PX4_AttitudeControl _att_control;

    // Vòng trong: PID tốc độ góc
    PX4_RatePID _rate_pid_roll;
    PX4_RatePID _rate_pid_pitch;
    PX4_RatePID _rate_pid_yaw;

    // Bộ phối trộn
    PX4_QuadMixer _mixer;

    uint16_t _motor_outputs_us[4];
    bool     _invert_pitch;
};

// Instance toàn cục
extern PX4_FlightController px4Controller;

#endif // PX4_PID_H
