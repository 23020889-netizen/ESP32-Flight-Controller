#include "px4_pid.h"
#include <math.h>

// Instance toàn cục
PX4_FlightController px4Controller;

// ============================================================================
// 1. PX4 RATE PID (INNER LOOP) - TÍNH TOÁN TRÊN ĐƠN VỊ RADIAN/S
// ============================================================================
PX4_RatePID::PX4_RatePID()
    : _integral(0.0f), _last_rate_meas(0.0f),
      _p_out(0.0f), _d_out(0.0f), _first_run(true) {
    // Hệ số chuẩn PX4 cho đơn vị rad/s
    _gains = { 0.15f, 0.20f, 0.003f, 0.30f };
}

void PX4_RatePID::init(float kp, float ki, float kd, float i_max, float d_cutoff_hz, float loop_rate_hz) {
    setGains(kp, ki, kd, i_max);
    setLoopRate(loop_rate_hz);
    _d_lpf.setCutoffFrequency(loop_rate_hz, d_cutoff_hz);
    reset();
}

void PX4_RatePID::setGains(float kp, float ki, float kd, float i_max) {
    _gains.kp = kp;
    _gains.ki = ki;
    _gains.kd = kd;
    _gains.i_max = i_max;
}

void PX4_RatePID::setLoopRate(float loop_rate_hz) {
    _d_lpf.setCutoffFrequency(loop_rate_hz, 20.0f); // 20Hz D-term smoothing
}

float PX4_RatePID::update(float rate_sp, float rate_meas, float dt, bool allow_integral) {
    if (isnan(rate_meas) || isinf(rate_meas) || isnan(rate_sp)) return 0.0f;
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.005f; // Dự phòng 200Hz
    }

    // Sai số tốc độ góc (rad/s)
    float err = rate_sp - rate_meas;

    // Feedback error tinh khiet (Deadband chi dat tren Setpoint, khong dat trong Feedback loop)

    // 1. Proportional term
    _p_out = _gains.kp * err;

    // 2. Integral term (kèm Anti-Windup và ngắt khi throttle thấp)
    if (allow_integral) {
        _integral += _gains.ki * err * dt;
        if (_integral > _gains.i_max)  _integral = _gains.i_max;
        if (_integral < -_gains.i_max) _integral = -_gains.i_max;
    } else {
        _integral = 0.0f;
    }

    // 3. Derivative on Measurement (Tránh spike giật khi đổi Setpoint)
    if (_first_run) {
        _last_rate_meas = rate_meas;
        _first_run = false;
        _d_out = 0.0f;
    } else {
        float d_raw = -_gains.kd * ((rate_meas - _last_rate_meas) / dt);
        _last_rate_meas = rate_meas;
        _d_out = _d_lpf.apply(d_raw);
    }

    // Tổng hợp mô-men và bão hòa trong [-1.0, 1.0]
    float torque_out = _p_out + _integral + _d_out;
    if (torque_out > 1.0f)  torque_out = 1.0f;
    if (torque_out < -1.0f) torque_out = -1.0f;

    return torque_out;
}

void PX4_RatePID::reset() {
    _integral = 0.0f;
    _p_out = 0.0f;
    _d_out = 0.0f;
    _first_run = true;
    _d_lpf.reset(0.0f);
}

// ============================================================================
// 2. PX4 ATTITUDE CONTROL (OUTER LOOP) - TÍNH TOÁN RA RADIAN/S
// ============================================================================
PX4_AttitudeControl::PX4_AttitudeControl()
    : _roll_p(6.5f), _pitch_p(6.5f), _yaw_p(2.8f) {
    _max_roll_rate_rad  = 220.0f * DEG_TO_RAD; // ~3.84 rad/s
    _max_pitch_rate_rad = 220.0f * DEG_TO_RAD; // ~3.84 rad/s
    _max_yaw_rate_rad   = 150.0f * DEG_TO_RAD; // ~2.62 rad/s
}

void PX4_AttitudeControl::setAngleGains(float roll_p, float pitch_p, float yaw_p) {
    _roll_p = roll_p;
    _pitch_p = pitch_p;
    _yaw_p = yaw_p;
}

void PX4_AttitudeControl::setMaxRates(float max_roll_rate_rad, float max_pitch_rate_rad, float max_yaw_rate_rad) {
    _max_roll_rate_rad  = max_roll_rate_rad;
    _max_pitch_rate_rad = max_pitch_rate_rad;
    _max_yaw_rate_rad   = max_yaw_rate_rad;
}

float PX4_AttitudeControl::wrapAngle180(float angle_deg) {
    if (isnan(angle_deg) || isinf(angle_deg)) return 0.0f;
    while (angle_deg > 180.0f)  angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

void PX4_AttitudeControl::update(const float target_euler[3], const float actual_euler[3], float rate_sp_out[3]) {
    // Chuyển đổi sai số góc sang Radian
    float err_roll_rad  = (target_euler[0] - actual_euler[0]) * DEG_TO_RAD;
    float err_pitch_rad = (target_euler[1] - actual_euler[1]) * DEG_TO_RAD;
    float err_yaw_rad   = wrapAngle180(target_euler[2] - actual_euler[2]) * DEG_TO_RAD;

    // Outer P Control ra target angular rate (rad/s)
    rate_sp_out[0] = constrain(err_roll_rad  * _roll_p,  -_max_roll_rate_rad,  _max_roll_rate_rad);
    rate_sp_out[1] = constrain(err_pitch_rad * _pitch_p, -_max_pitch_rate_rad, _max_pitch_rate_rad);
    rate_sp_out[2] = constrain(err_yaw_rad   * _yaw_p,   -_max_yaw_rate_rad,   _max_yaw_rate_rad);
}

void PX4_AttitudeControl::reset() {
    // Không giữ trạng thái nội bộ
}

// ============================================================================
// 3. PX4 QUAD-X MIXER & DESATURATION
// ============================================================================
PX4_QuadMixer::PX4_QuadMixer() {}

void PX4_QuadMixer::mix(float throttle, float roll_torque, float pitch_torque, float yaw_torque, uint16_t motor_out_us[4]) {
    // Nếu throttle quá nhỏ (< 2%), cả 4 động cơ giữ xung Idle
    if (throttle < 0.02f) {
        for (int i = 0; i < 4; i++) {
            motor_out_us[i] = ESC_PULSE_IDLE_US;
        }
        return;
    }

    // Giới hạn mô-men Roll/Pitch/Yaw theo tỷ lệ mức ga để chống dập tắt động cơ khi ở mức ga thấp
    float max_authority = throttle * 0.70f;
    if (max_authority < 0.06f) max_authority = 0.06f;
    if (max_authority > 0.35f) max_authority = 0.35f;

    roll_torque  = constrain(roll_torque,  -max_authority, max_authority);
    pitch_torque = constrain(pitch_torque, -max_authority, max_authority);
    yaw_torque   = constrain(yaw_torque,   -max_authority * 0.5f, max_authority * 0.5f);

        // Cấu hình Quad-X chuẩn PX4 với Phản Hồi Âm Tự Cân Bằng (Negative Feedback 100%):
    // - Bên nào nghiêng THẤP xuống -> Động cơ bên đó TĂNG GA đẩy lên thăng bằng!
    // - Khi nghiêng PHẢI (Roll > 0, tau_roll < 0) -> Cánh phải thấp: M1(FR) & M4(RR) TĂNG ga, M2(RL) & M3(FL) GIẢM ga.
    // - Khi nghiêng TRÁI (Roll < 0, tau_roll > 0) -> Cánh trái thấp: M2(RL) & M3(FL) TĂNG ga, M1(FR) & M4(RR) GIẢM ga.
    // - Khi ngóc MŨI    (Pitch > 0, tau_pitch < 0) -> Đuôi thấp:     M2(RL) & M4(RR) TĂNG ga, M1(FR) & M3(FL) GIẢM ga.
    // - Khi chúc MŨI    (Pitch < 0, tau_pitch > 0) -> Mũi thấp:      M1(FR) & M3(FL) TĂNG ga, M2(RL) & M4(RR) GIẢM ga.
    // - Khi xoay YAW CW (Yaw > 0, tau_yaw < 0)    -> Hãm xoay:     M3(FL) & M4(RR) CW TĂNG ga, M1(FR) & M2(RL) CCW GIẢM ga.
    //
    // Motor 1: GPIO 7 (Front-Right, CCW) = T - R + P + Y
    // Motor 2: GPIO 6 (Rear-Left,   CCW) = T + R - P + Y
    // Motor 3: GPIO 5 (Front-Left,  CW)  = T + R + P - Y
    // Motor 4: GPIO 4 (Rear-Right, CW)   = T - R - P - Y
    float m[4];
    m[0] = throttle - roll_torque + pitch_torque + yaw_torque;
    m[1] = throttle + roll_torque - pitch_torque + yaw_torque;
    m[2] = throttle + roll_torque + pitch_torque - yaw_torque;
    m[3] = throttle - roll_torque - pitch_torque - yaw_torque;

    // Thuật toán Desaturation chuẩn PX4: Co giãn bảo toàn hình học mô-men (Dynamic Scale & Shift)
    float max_val = m[0];
    float min_val = m[0];
    for (int i = 1; i < 4; i++) {
        if (m[i] > max_val) max_val = m[i];
        if (m[i] < min_val) min_val = m[i];
    }

    const float min_floor = 0.02f; // Sàn ga tối thiểu 2% để cánh luôn quay ổn định khi có lệnh ga
    float span = max_val - min_val;
    if (span > (1.0f - min_floor)) {
        float scale = (1.0f - min_floor) / span;
        float center = (max_val + min_val) * 0.5f;
        for (int i = 0; i < 4; i++) {
            m[i] = center + (m[i] - center) * scale;
        }
        max_val = center + (max_val - center) * scale;
        min_val = center + (min_val - center) * scale;
    }

    if (max_val > 1.0f) {
        float shift = max_val - 1.0f;
        for (int i = 0; i < 4; i++) m[i] -= shift;
    } else if (min_val < min_floor) {
        float shift = min_floor - min_val;
        for (int i = 0; i < 4; i++) m[i] += shift;
    }

    // Chuyển đổi sang xung Microsecond (1000 .. 2000 us)
    for (int i = 0; i < 4; i++) {
        m[i] = constrain(m[i], min_floor, 1.0f);
        motor_out_us[i] = ESC_PULSE_MIN_US + (uint16_t)(m[i] * (float)(ESC_PULSE_MAX_US - ESC_PULSE_MIN_US));
    }
}

// ============================================================================
// 4. PX4 FLIGHT CONTROLLER (UNIFIED ENGINE)
// ============================================================================
PX4_FlightController::PX4_FlightController()
    : _target_roll(0.0f), _target_pitch(0.0f), _target_yaw(0.0f), _target_throttle(0.0f),
      _filtered_gx(0.0f), _filtered_gy(0.0f), _filtered_gz(0.0f),
      _invert_pitch(false) {
    for (int i = 0; i < 4; i++) {
        _motor_outputs_us[i] = ESC_PULSE_DISARM_US;
    }
}

void PX4_FlightController::init(float loop_freq_hz) {
    // Cài đặt bộ lọc Butterworth bậc 2 cắt ở 30Hz cho Gyro
    _gyro_filter_x.setCutoffFrequency(loop_freq_hz, 30.0f);
    _gyro_filter_y.setCutoffFrequency(loop_freq_hz, 30.0f);
    _gyro_filter_z.setCutoffFrequency(loop_freq_hz, 30.0f);

    // Cài đặt Inner Rate PID loops (Đơn vị rad/s)
    // Trục Roll & Pitch: Kp=0.15, Ki=0.20, Kd=0.003
    _rate_pid_roll.init(0.15f, 0.20f, 0.003f, 0.30f, 20.0f, loop_freq_hz);
    _rate_pid_pitch.init(0.15f, 0.20f, 0.003f, 0.30f, 20.0f, loop_freq_hz);

    // Trục Yaw: Kp=0.20, Ki=0.10, Kd=0.000
    _rate_pid_yaw.init(0.20f, 0.10f, 0.000f, 0.25f, 20.0f, loop_freq_hz);

    // Cài đặt Outer Angle Control: Roll_P=6.5, Pitch_P=6.5, Yaw_P=2.8
    _att_control.setAngleGains(6.5f, 6.5f, 2.8f);

    reset();
}

void PX4_FlightController::setTargetAttitude(float roll_deg, float pitch_deg, float yaw_deg) {
    _target_roll = roll_deg;
    _target_pitch = pitch_deg;
    _target_yaw = yaw_deg;
}

void PX4_FlightController::lockCurrentHeading(float current_yaw_deg) {
    _target_yaw = current_yaw_deg;
}

void PX4_FlightController::setTargetThrottle(float throttle_pct) {
    _target_throttle = constrain(throttle_pct / 100.0f, 0.0f, 1.0f);
}



void PX4_FlightController::setRatePidRoll(float kp, float ki, float kd) {
    _rate_pid_roll.setGains(kp, ki, kd);
}

void PX4_FlightController::setRatePidPitch(float kp, float ki, float kd) {
    _rate_pid_pitch.setGains(kp, ki, kd);
}

void PX4_FlightController::setRatePidYaw(float kp, float ki, float kd) {
    _rate_pid_yaw.setGains(kp, ki, kd);
}

void PX4_FlightController::setAngleGains(float roll_p, float pitch_p, float yaw_p) {
    _att_control.setAngleGains(roll_p, pitch_p, yaw_p);
}

void PX4_FlightController::reset() {
    _gyro_filter_x.reset(0.0f);
    _gyro_filter_y.reset(0.0f);
    _gyro_filter_z.reset(0.0f);
    _rate_pid_roll.reset();
    _rate_pid_pitch.reset();
    _rate_pid_yaw.reset();
    _att_control.reset();

    for (int i = 0; i < 4; i++) {
        _motor_outputs_us[i] = ESC_PULSE_DISARM_US;
    }
}

void PX4_FlightController::update(float actual_roll, float actual_pitch, float actual_yaw,
                                 float raw_gx, float raw_gy, float raw_gz,
                                 float dt, bool is_armed) {
    // 1. Lọc nhiễu rung cơ học trên Gyro bằng bộ lọc 2nd-Order Butterworth LPF (30Hz)
    _filtered_gx = _gyro_filter_x.apply(raw_gx);
    _filtered_gy = _gyro_filter_y.apply(raw_gy);
    _filtered_gz = _gyro_filter_z.apply(raw_gz);

    // Chuyển tốc độ góc thực sang Radian/giây (rad/s)
    float actual_gx_rad = _filtered_gx * DEG_TO_RAD;
    float actual_gy_rad = _filtered_gy * DEG_TO_RAD;
    float actual_gz_rad = _filtered_gz * DEG_TO_RAD;

    // Nếu chưa Arm hoặc ga dưới ngưỡng cất cánh (< 5%): Không can thiệp động cơ
    if (!is_armed || _target_throttle < 0.05f) {
        reset();
        for (int i = 0; i < 4; i++) {
            _motor_outputs_us[i] = is_armed ? ESC_PULSE_IDLE_US : ESC_PULSE_DISARM_US;
        }
        return;
    }

    // 2. VÒNG NGOÀI (Outer Angle P-Loop): Tính tốc độ góc mục tiêu (rad/s)
    float target_att[3] = { _target_roll, _target_pitch, _target_yaw };
    float actual_att[3] = { actual_roll,  actual_pitch,  actual_yaw  };
    float rate_sp_rad[3] = { 0.0f, 0.0f, 0.0f };

    _att_control.update(target_att, actual_att, rate_sp_rad);

    // 3. VÒNG TRONG (Inner Angular Rate PID-Loop): Tính mô-men điều khiển
    bool allow_integral = (_target_throttle >= 0.10f); // Chỉ cho phép tích phân khi ga >= 10%
    float tau_roll  = _rate_pid_roll.update(rate_sp_rad[0], actual_gx_rad, dt, allow_integral);
    float tau_pitch = _rate_pid_pitch.update(rate_sp_rad[1], actual_gy_rad, dt, allow_integral);
    if (_invert_pitch) tau_pitch = -tau_pitch;
    float tau_yaw   = _rate_pid_yaw.update(rate_sp_rad[2],  actual_gz_rad, dt, allow_integral);

    // 4. BỘ PHỐI TRỘN (Mixer): Phân bổ mô-men và ga ra 4 động cơ
    _mixer.mix(_target_throttle, tau_roll, tau_pitch, tau_yaw, _motor_outputs_us);
}
