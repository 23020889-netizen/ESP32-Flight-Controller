#include "px4_pid.h"
#include <math.h>

// Instance toàn cục
PX4_FlightController px4Controller;

static inline float clampf(float v, float lo, float hi) {
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static inline bool fc_finite(float v) {
    return !(isnan(v) || isinf(v));
}

// ============================================================================
// 1. PX4 RATE PID (INNER LOOP) - TÍNH TOÁN TRÊN ĐƠN VỊ RADIAN/S
// ============================================================================
PX4_RatePID::PX4_RatePID()
    : _integral(0.0f), _last_rate_meas(0.0f),
      _p_out(0.0f), _d_out(0.0f),
      _d_cutoff_hz(20.0f), _loop_rate_hz(200.0f),
      _first_run(true), _saturated(false) {
    // Hệ số chuẩn PX4 cho đơn vị rad/s
    _gains = { 0.15f, 0.20f, 0.003f, 0.30f };
    _d_lpf.setCutoffFrequency(_loop_rate_hz, _d_cutoff_hz);
}

void PX4_RatePID::init(float kp, float ki, float kd, float i_max, float d_cutoff_hz, float loop_rate_hz) {
    setGains(kp, ki, kd, i_max);
    _d_cutoff_hz = d_cutoff_hz;
    setLoopRate(loop_rate_hz);
    reset();
}

// Đổi Kp/Ki/Kd nhưng GIỮ nguyên i_max đã cấu hình cho trục
// (bản cũ ghi đè i_max = 0.30 cho mọi trục, kể cả Yaw = 0.25).
void PX4_RatePID::setGains(float kp, float ki, float kd) {
    if (!fc_finite(kp) || !fc_finite(ki) || !fc_finite(kd)) return;
    _gains.kp = fmaxf(kp, 0.0f);
    _gains.ki = fmaxf(ki, 0.0f);
    _gains.kd = fmaxf(kd, 0.0f);
    // Tích phân đang lưu theo đơn vị mô-men (đã nhân Ki) nên không cần scale lại khi đổi Ki.
}

void PX4_RatePID::setGains(float kp, float ki, float kd, float i_max) {
    setGains(kp, ki, kd);
    if (fc_finite(i_max)) _gains.i_max = fabsf(i_max);
    _integral = clampf(_integral, -_gains.i_max, _gains.i_max);
}

void PX4_RatePID::setLoopRate(float loop_rate_hz) {
    if (loop_rate_hz > 0.0f) _loop_rate_hz = loop_rate_hz;
    // Giữ tần số cắt D đã cấu hình (bản cũ luôn ép về 20 Hz)
    _d_lpf.setCutoffFrequency(_loop_rate_hz, _d_cutoff_hz);
}

float PX4_RatePID::update(float rate_sp, float rate_meas, float dt, bool allow_integral, float out_limit) {
    if (!fc_finite(rate_meas) || !fc_finite(rate_sp)) {
        // Dữ liệu IMU hỏng: không đưa ra mô-men, không làm bẩn trạng thái tích phân
        _p_out = 0.0f;
        _d_out = 0.0f;
        _saturated = false;
        return clampf(_integral, -out_limit, out_limit);
    }
    if (!(dt > 0.0f) || dt > 0.1f) {
        dt = 1.0f / _loop_rate_hz;
    }
    if (!(out_limit > 0.0f)) out_limit = 1.0f;
    if (out_limit > 1.0f) out_limit = 1.0f;

    // Sai số tốc độ góc (rad/s)
    const float err = rate_sp - rate_meas;

    // 1. Proportional term
    _p_out = _gains.kp * err;

    // 2. Derivative on Measurement (tránh "derivative kick" khi đổi setpoint)
    if (_first_run) {
        _last_rate_meas = rate_meas;
        _first_run = false;
        _d_out = 0.0f;
        _d_lpf.reset(0.0f);
    } else {
        const float d_raw = -_gains.kd * ((rate_meas - _last_rate_meas) / dt);
        _last_rate_meas = rate_meas;
        _d_out = _d_lpf.apply(d_raw);
    }

    // 3. Integral term + anti-windup (conditional integration)
    if (allow_integral) {
        const float unsat = _p_out + _integral + _d_out;
        const bool push_high = (unsat >=  out_limit) && (err > 0.0f);
        const bool push_low  = (unsat <= -out_limit) && (err < 0.0f);
        if (!push_high && !push_low) {
            _integral += _gains.ki * err * dt;
        }
        _integral = clampf(_integral, -_gains.i_max, _gains.i_max);
    } else {
        _integral = 0.0f;
    }

    // 4. Tổng hợp mô-men và bão hòa theo quyền điều khiển thực tế của Mixer
    const float torque_unsat = _p_out + _integral + _d_out;
    const float torque_out = clampf(torque_unsat, -out_limit, out_limit);
    _saturated = (torque_out != torque_unsat);
    return torque_out;
}

void PX4_RatePID::reset() {
    _integral = 0.0f;
    _p_out = 0.0f;
    _d_out = 0.0f;
    _first_run = true;
    _saturated = false;
    _d_lpf.reset(0.0f);
}

// ============================================================================
// 2. PX4 ATTITUDE CONTROL (OUTER LOOP) - TÍNH TOÁN RA RADIAN/S (BODY RATES)
// ============================================================================
PX4_AttitudeControl::PX4_AttitudeControl()
    : _roll_p(6.5f), _pitch_p(6.5f), _yaw_p(2.8f) {
    _max_roll_rate_rad  = 220.0f * DEG_TO_RAD; // ~3.84 rad/s
    _max_pitch_rate_rad = 220.0f * DEG_TO_RAD; // ~3.84 rad/s
    _max_yaw_rate_rad   = 150.0f * DEG_TO_RAD; // ~2.62 rad/s
}

void PX4_AttitudeControl::setAngleGains(float roll_p, float pitch_p, float yaw_p) {
    if (!fc_finite(roll_p) || !fc_finite(pitch_p) || !fc_finite(yaw_p)) return;
    _roll_p  = fmaxf(roll_p, 0.0f);
    _pitch_p = fmaxf(pitch_p, 0.0f);
    _yaw_p   = fmaxf(yaw_p, 0.0f);
}

void PX4_AttitudeControl::setMaxRates(float max_roll_rate_rad, float max_pitch_rate_rad, float max_yaw_rate_rad) {
    _max_roll_rate_rad  = fabsf(max_roll_rate_rad);
    _max_pitch_rate_rad = fabsf(max_pitch_rate_rad);
    _max_yaw_rate_rad   = fabsf(max_yaw_rate_rad);
}

float PX4_AttitudeControl::wrapAngle180(float angle_deg) {
    if (!fc_finite(angle_deg)) return 0.0f;
    angle_deg = fmodf(angle_deg + 180.0f, 360.0f);
    if (angle_deg < 0.0f) angle_deg += 360.0f;
    return angle_deg - 180.0f;
}

void PX4_AttitudeControl::update(const float target_euler[3], const float actual_euler[3], float rate_sp_out[3], bool yaw_valid) {
    // Góc thực tế lỗi => không sinh lệnh tốc độ (vòng trong vẫn giữ rate = 0, tức là damping)
    if (!fc_finite(actual_euler[0]) || !fc_finite(actual_euler[1])) {
        rate_sp_out[0] = rate_sp_out[1] = rate_sp_out[2] = 0.0f;
        return;
    }

    // Sai số góc (rad). Roll được wrap để chịu được biểu diễn ±180°.
    const float err_roll_rad  = wrapAngle180(target_euler[0] - actual_euler[0]) * DEG_TO_RAD;
    const float err_pitch_rad = (target_euler[1] - actual_euler[1]) * DEG_TO_RAD;
    float err_yaw_rad = 0.0f;
    if (yaw_valid && fc_finite(actual_euler[2]) && fc_finite(target_euler[2])) {
        err_yaw_rad = wrapAngle180(target_euler[2] - actual_euler[2]) * DEG_TO_RAD;
    }

    // Outer P Control: đạo hàm góc Euler mong muốn (rad/s)
    const float phi_dot   = err_roll_rad  * _roll_p;
    const float theta_dot = err_pitch_rad * _pitch_p;
    const float psi_dot   = err_yaw_rad   * _yaw_p;

    // Chuyển Euler-rate -> Body-rate (cùng hệ trục với Gyro mà vòng trong đang đo)
    const float phi   = actual_euler[0] * DEG_TO_RAD;
    const float theta = actual_euler[1] * DEG_TO_RAD;
    const float sphi = sinf(phi),   cphi = cosf(phi);
    const float sth  = sinf(theta), cth  = cosf(theta);

    const float p = phi_dot - sth * psi_dot;
    const float q = cphi * theta_dot + sphi * cth * psi_dot;
    const float r = -sphi * theta_dot + cphi * cth * psi_dot;

    rate_sp_out[0] = clampf(p, -_max_roll_rate_rad,  _max_roll_rate_rad);
    rate_sp_out[1] = clampf(q, -_max_pitch_rate_rad, _max_pitch_rate_rad);
    rate_sp_out[2] = clampf(r, -_max_yaw_rate_rad,   _max_yaw_rate_rad);
}

void PX4_AttitudeControl::reset() {
    // Không giữ trạng thái nội bộ
}

// ============================================================================
// 3. PX4 QUAD-X MIXER & DESATURATION
// ============================================================================
PX4_QuadMixer::PX4_QuadMixer() {}

float PX4_QuadMixer::torqueAuthority(float throttle) {
    // Giới hạn mô-men Roll/Pitch theo mức ga để chống tắt động cơ khi ga thấp
    float a = throttle * 0.70f;
    if (a < 0.06f) a = 0.06f;
    if (a > 0.35f) a = 0.35f;
    return a;
}

void PX4_QuadMixer::mixNormalized(float throttle, float roll_torque, float pitch_torque, float yaw_torque, float motor_norm_out[4]) {
    if (!fc_finite(throttle) || !fc_finite(roll_torque) || !fc_finite(pitch_torque) || !fc_finite(yaw_torque)) {
        for (int i = 0; i < 4; i++) motor_norm_out[i] = 0.0f;
        return;
    }

    // Nếu throttle quá nhỏ (< 2%), cả 4 động cơ giữ mức 0
    if (throttle < 0.02f) {
        for (int i = 0; i < 4; i++) {
            motor_norm_out[i] = 0.0f;
        }
        return;
    }
    if (throttle > 1.0f) throttle = 1.0f;

    const float max_authority = torqueAuthority(throttle);
    roll_torque  = clampf(roll_torque,  -max_authority, max_authority);
    pitch_torque = clampf(pitch_torque, -max_authority, max_authority);
    yaw_torque   = clampf(yaw_torque,   -max_authority * 0.5f, max_authority * 0.5f);

    // Cấu hình Quad-X theo bố trí vật lý thực tế (M1,M3=TRÁI; M2,M4=PHẢI):
    // Motor 1: GPIO 7 (Front-Left,  CCW) = T + R + P + Y
    // Motor 2: GPIO 6 (Rear-Right,  CCW) = T - R - P + Y
    // Motor 3: GPIO 5 (Rear-Left,   CW)  = T + R - P - Y
    // Motor 4: GPIO 4 (Front-Right, CW)  = T - R + P - Y
    float m[4];
    m[0] = throttle + roll_torque + pitch_torque + yaw_torque;
    m[1] = throttle - roll_torque - pitch_torque + yaw_torque;
    m[2] = throttle + roll_torque - pitch_torque - yaw_torque;
    m[3] = throttle - roll_torque + pitch_torque - yaw_torque;

    // Desaturation: co giãn bảo toàn tỉ lệ mô-men rồi dịch về dải [min_floor .. 1]
    float max_val = m[0];
    float min_val = m[0];
    for (int i = 1; i < 4; i++) {
        if (m[i] > max_val) max_val = m[i];
        if (m[i] < min_val) min_val = m[i];
    }

    const float min_floor = 0.02f; // Sàn ga tối thiểu 2% để cánh luôn quay ổn định khi có lệnh ga
    const float span = max_val - min_val;
    if (span > (1.0f - min_floor)) {
        const float scale = (1.0f - min_floor) / span;
        const float center = (max_val + min_val) * 0.5f;
        for (int i = 0; i < 4; i++) {
            m[i] = center + (m[i] - center) * scale;
        }
        max_val = center + (max_val - center) * scale;
        min_val = center + (min_val - center) * scale;
    }

    if (max_val > 1.0f) {
        const float shift = max_val - 1.0f;
        for (int i = 0; i < 4; i++) m[i] -= shift;
    } else if (min_val < min_floor) {
        const float shift = min_floor - min_val;
        for (int i = 0; i < 4; i++) m[i] += shift;
    }

    for (int i = 0; i < 4; i++) {
        motor_norm_out[i] = clampf(m[i], 0.0f, 1.0f);
    }
}

void PX4_QuadMixer::mix(float throttle, float roll_torque, float pitch_torque, float yaw_torque, uint16_t motor_out_us[4]) {
    float norm[4];
    mixNormalized(throttle, roll_torque, pitch_torque, yaw_torque, norm);
    for (int i = 0; i < 4; i++) {
        if (norm[i] <= 0.001f) {
            motor_out_us[i] = ESC_PULSE_IDLE_US;
        } else {
            motor_out_us[i] = ESC_PULSE_MIN_US + (uint16_t)(norm[i] * (float)(ESC_PULSE_MAX_US - ESC_PULSE_MIN_US));
        }
    }
}

// ============================================================================
// 4. PX4 FLIGHT CONTROLLER (UNIFIED ENGINE)
// ============================================================================
PX4_FlightController::PX4_FlightController()
    : _target_roll(0.0f), _target_pitch(0.0f), _target_yaw(0.0f), _target_throttle(0.0f),
      _filtered_gx(0.0f), _filtered_gy(0.0f), _filtered_gz(0.0f),
      _invert_pitch(false) {
    _rate_sp_rad[0] = 0.0f; _rate_sp_rad[1] = 0.0f; _rate_sp_rad[2] = 0.0f;
    _tau[0] = 0.0f; _tau[1] = 0.0f; _tau[2] = 0.0f;
    _last_raw_g[0] = 0.0f; _last_raw_g[1] = 0.0f; _last_raw_g[2] = 0.0f;
    for (int i = 0; i < 4; i++) {
        _motor_norm[i] = 0.0f;
        _motor_outputs_us[i] = ESC_PULSE_DISARM_US;
    }
}

void PX4_FlightController::init(float loop_freq_hz) {
    // Bộ lọc Butterworth bậc 2 cắt ở 30Hz cho Gyro
    _gyro_filter_x.setCutoffFrequency(loop_freq_hz, 30.0f);
    _gyro_filter_y.setCutoffFrequency(loop_freq_hz, 30.0f);
    _gyro_filter_z.setCutoffFrequency(loop_freq_hz, 30.0f);

    // Inner Rate PID loops (Đơn vị rad/s)
    _rate_pid_roll.init(0.15f, 0.20f, 0.003f, 0.30f, 20.0f, loop_freq_hz);
    _rate_pid_pitch.init(0.15f, 0.20f, 0.003f, 0.30f, 20.0f, loop_freq_hz);
    _rate_pid_yaw.init(0.20f, 0.10f, 0.000f, 0.25f, 20.0f, loop_freq_hz);

    // Outer Angle Control
    _att_control.setAngleGains(6.5f, 6.5f, 2.8f);

    reset();
}

void PX4_FlightController::setTargetAttitude(float roll_deg, float pitch_deg, float yaw_deg) {
    if (fc_finite(roll_deg))  _target_roll  = roll_deg;
    if (fc_finite(pitch_deg)) _target_pitch = pitch_deg;
    if (fc_finite(yaw_deg))   _target_yaw   = yaw_deg;
}

void PX4_FlightController::lockCurrentHeading(float current_yaw_deg) {
    if (fc_finite(current_yaw_deg)) _target_yaw = current_yaw_deg;
}

void PX4_FlightController::setTargetThrottle(float throttle_pct) {
    if (!fc_finite(throttle_pct)) throttle_pct = 0.0f;
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

void PX4_FlightController::resetControllers() {
    _rate_pid_roll.reset();
    _rate_pid_pitch.reset();
    _rate_pid_yaw.reset();
    _att_control.reset();

    _rate_sp_rad[0] = 0.0f; _rate_sp_rad[1] = 0.0f; _rate_sp_rad[2] = 0.0f;
    _tau[0] = 0.0f; _tau[1] = 0.0f; _tau[2] = 0.0f;
}

void PX4_FlightController::reset() {
    // Nạp lại bộ lọc bằng mẫu Gyro gần nhất (KHÔNG phải 0): tránh xung quá độ
    // khi ARM và tránh telemetry Gyro bị kéo về ~0 khi đang DISARMED.
    _gyro_filter_x.reset(_last_raw_g[0]);
    _gyro_filter_y.reset(_last_raw_g[1]);
    _gyro_filter_z.reset(_last_raw_g[2]);
    _filtered_gx = _last_raw_g[0];
    _filtered_gy = _last_raw_g[1];
    _filtered_gz = _last_raw_g[2];

    resetControllers();

    for (int i = 0; i < 4; i++) {
        _motor_norm[i] = 0.0f;
        _motor_outputs_us[i] = ESC_PULSE_DISARM_US;
    }
}

void PX4_FlightController::update(float actual_roll, float actual_pitch, float actual_yaw,
                                  float raw_gx, float raw_gy, float raw_gz,
                                  float dt, bool is_armed, bool yaw_valid) {
    // 0. Bảo vệ NaN/Inf từ cảm biến: giữ mẫu hợp lệ gần nhất
    if (!fc_finite(raw_gx)) raw_gx = _last_raw_g[0];
    if (!fc_finite(raw_gy)) raw_gy = _last_raw_g[1];
    if (!fc_finite(raw_gz)) raw_gz = _last_raw_g[2];
    _last_raw_g[0] = raw_gx; _last_raw_g[1] = raw_gy; _last_raw_g[2] = raw_gz;

    // 1. Lọc nhiễu rung cơ học trên Gyro bằng 2nd-Order Butterworth LPF (30Hz).
    //    Bộ lọc LUÔN chạy (kể cả khi DISARMED) để telemetry và lúc ARM đều đúng.
    _filtered_gx = _gyro_filter_x.apply(raw_gx);
    _filtered_gy = _gyro_filter_y.apply(raw_gy);
    _filtered_gz = _gyro_filter_z.apply(raw_gz);

    const float actual_gx_rad = _filtered_gx * DEG_TO_RAD;
    const float actual_gy_rad = _filtered_gy * DEG_TO_RAD;
    const float actual_gz_rad = _filtered_gz * DEG_TO_RAD;

    // Chưa Arm: khóa cứng an toàn, chỉ reset bộ điều khiển (không đụng bộ lọc)
    if (!is_armed) {
        resetControllers();
        for (int i = 0; i < 4; i++) {
            _motor_norm[i] = 0.0f;
            _motor_outputs_us[i] = ESC_PULSE_DISARM_US;
        }
        return;
    }

    // Armed nhưng ga < 2%: Idle Spin, không tích phân PID
    if (_target_throttle < 0.02f) {
        resetControllers();
        for (int i = 0; i < 4; i++) {
            _motor_norm[i] = 0.0f;
            _motor_outputs_us[i] = ESC_PULSE_IDLE_US;
        }
        return;
    }

    // 2. VÒNG NGOÀI (Outer Angle P-Loop): góc -> tốc độ góc thân mục tiêu (rad/s)
    const float target_att[3] = { _target_roll, _target_pitch, _target_yaw };
    const float actual_att[3] = { actual_roll,  actual_pitch,  actual_yaw  };
    _att_control.update(target_att, actual_att, _rate_sp_rad, yaw_valid);

    // 3. VÒNG TRONG (Inner Angular Rate PID-Loop): tốc độ góc -> mô-men
    //    Giới hạn đầu ra = quyền điều khiển thực tế của Mixer => anti-windup chính xác.
    const bool allow_integral = (_target_throttle >= 0.10f); // Chỉ tích phân khi ga >= 10%
    const float auth_rp = PX4_QuadMixer::torqueAuthority(_target_throttle);
    const float auth_y  = PX4_QuadMixer::yawAuthority(_target_throttle);

    _tau[0] = _rate_pid_roll.update(_rate_sp_rad[0], actual_gx_rad, dt, allow_integral, auth_rp);
    _tau[1] = _rate_pid_pitch.update(_rate_sp_rad[1], actual_gy_rad, dt, allow_integral, auth_rp);
    // Đảo dấu cơ cấu chấp hành Pitch (khi chiều motor/mixer trục Pitch bị lắp ngược)
    if (_invert_pitch) _tau[1] = -_tau[1];
    _tau[2] = _rate_pid_yaw.update(_rate_sp_rad[2], actual_gz_rad, dt, allow_integral, auth_y);

    // 4. BỘ PHỐI TRỘN (Mixer / Allocator)
    _mixer.mixNormalized(_target_throttle, _tau[0], _tau[1], _tau[2], _motor_norm);

    for (int i = 0; i < 4; i++) {
        _motor_outputs_us[i] = ESC_PULSE_MIN_US + (uint16_t)(_motor_norm[i] * (float)(ESC_PULSE_MAX_US - ESC_PULSE_MIN_US));
    }
}
