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
    void setGains(float kp, float ki, float kd, float i_max = -1.0f);
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
     * @param actual_euler [Roll, Pitch, Yaw] thực tế từ cảm biến (Độ)
     * @param rate_sp_out [Roll, Pitch, Yaw] tốc độ góc đầu ra (rad/s)
     * @param yaw_valid Trạng thái hợp lệ của cảm biến góc Yaw
     */
    void update(const float target_euler[3], const float actual_euler[3], float rate_sp_out[3], bool yaw_valid = true);

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
 * Tách biệt hoàn toàn khỏi Controller: nhận Collective Thrust & Torques, xuất Normalized [0..1]
 */
class PX4_QuadMixer {
public:
    PX4_QuadMixer();

    /**
     * @brief Phối trộn ga và mô-men 3 trục ra lệnh chuẩn hóa [0.0 .. 1.0] cho 4 động cơ
     * @param throttle Mức ga cơ bản [0.0 .. 1.0]
     * @param roll_torque Mô-men nghiêng ngang [-1.0 .. 1.0]
     * @param pitch_torque Mô-men chúc ngóc [-1.0 .. 1.0]
     * @param yaw_torque Mô-men xoay thân [-1.0 .. 1.0]
     * @param motor_norm_out Mảng đầu ra 4 kênh chuẩn hóa [0.0 .. 1.0]
     */
    void mixNormalized(float throttle, float roll_torque, float pitch_torque, float yaw_torque, float motor_norm_out[4]);

    /**
     * @brief Phối trộn ra xung PWM trực tiếp (tương thích ngược)
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
     * @param actual_roll Góc Roll từ cảm biến (độ)
     * @param actual_pitch Góc Pitch từ cảm biến (độ)
     * @param actual_yaw Góc Yaw từ cảm biến (độ)
     * @param raw_gx Tốc độ góc Gyro X (deg/s)
     * @param raw_gy Tốc độ góc Gyro Y (deg/s)
     * @param raw_gz Tốc độ góc Gyro Z (deg/s)
     * @param dt Chu kỳ thời gian (giây)
     * @param is_armed Trạng thái đã mở khóa động cơ hay chưa
     * @param yaw_valid Trạng thái hợp lệ của góc Yaw
     */
    void update(float actual_roll, float actual_pitch, float actual_yaw,
                float raw_gx, float raw_gy, float raw_gz,
                float dt, bool is_armed, bool yaw_valid = true);

    void reset();

    // Getters cho giám sát và Telemetry
    float getFilteredGx() const { return _filtered_gx; }
    float getFilteredGy() const { return _filtered_gy; }
    float getFilteredGz() const { return _filtered_gz; }

    void getTargetAttitude(float& r, float& p, float& y) const {
        r = _target_roll; p = _target_pitch; y = _target_yaw;
    }
    float getTargetThrottle() const { return _target_throttle * 100.0f; }

    // Lệnh chuẩn hóa u1..u4 [0.0 .. 1.0] từ Mixer
    const float* getNormalizedMotorOutputs() const { return _motor_norm; }

    // Xung vi giây xuất cho ESC
    const uint16_t* getMotorOutputsUs() const { return _motor_outputs_us; }

    // Mô-men ảo thân bay [tau_roll, tau_pitch, tau_yaw]
    void getTorques(float& tr, float& tp, float& ty) const {
        tr = _tau[0]; tp = _tau[1]; ty = _tau[2];
    }

    // Tốc độ góc mục tiêu [p_sp, q_sp, r_sp] (rad/s)
    void getRateSetpoints(float& p, float& q, float& r) const {
        p = _rate_sp_rad[0]; q = _rate_sp_rad[1]; r = _rate_sp_rad[2];
    }

    // Truy cập trực tiếp các bộ PID & Mixer
    PX4_RatePID& getRatePidRoll()  { return _rate_pid_roll; }
    PX4_RatePID& getRatePidPitch() { return _rate_pid_pitch; }
    PX4_RatePID& getRatePidYaw()   { return _rate_pid_yaw; }
    PX4_QuadMixer& getMixer()      { return _mixer; }
    PX4_AttitudeControl& getAttitudeControl() { return _att_control; }

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

    float    _rate_sp_rad[3];
    float    _tau[3];
    float    _motor_norm[4];
    uint16_t _motor_outputs_us[4];
    bool     _invert_pitch;
};

// Instance toàn cục
extern PX4_FlightController px4Controller;

#endif // PX4_PID_H
