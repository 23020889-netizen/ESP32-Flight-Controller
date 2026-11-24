#include "px4_pid.h"
#include <math.h>

// Instance toàn cục
PX4_FlightController px4Controller;

// ============================================================================
// 1. PX4 RATE PID (INNER LOOP) - TÍNH TOÁN TRÊN ĐƠN VỊ RADIAN/S
// ============================================================================
PX4_RatePID::PX4_RatePID()
    : _integral(0.0f), _last_rate_meas(0.0f),
      _p_out(0.0f), _d_out(0.0f), _d_cutoff_hz(30.0f), _first_run(true) {
    // Mặc định Roll/Pitch (đơn vị rad/s), xem PX4_FlightController::init()
    _gains = { 0.040f, 0.080f, 0.0010f, 0.20f };
}

void PX4_RatePID::init(float kp, float ki, float kd, float i_max, float d_cutoff_hz, float loop_rate_hz) {
    setGains(kp, ki, kd, i_max);
    if (d_cutoff_hz > 0.0f) _d_cutoff_hz = d_cutoff_hz;
    setLoopRate(loop_rate_hz);
    reset();
}

void PX4_RatePID::setGains(float kp, float ki, float kd, float i_max) {
    _gains.kp = kp;
    _gains.ki = ki;
    _gains.kd = kd;
    if (i_max > 0.0f) {
        _gains.i_max = i_max;
    }
}

void PX4_RatePID::setLoopRate(float loop_rate_hz) {
    // Giữ tần số cắt D đã cấu hình (bản cũ luôn ép 20 Hz)
    _d_lpf.setCutoffFrequency(loop_rate_hz, _d_cutoff_hz);
}

float PX4_RatePID::update(float rate_sp, float rate_meas, float dt, bool allow_integral, float out_limit) {
    if (isnan(rate_meas) || isinf(rate_meas) || isnan(rate_sp)) return 0.0f;
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.005f; // Dự phòng 200Hz
    }

    // Sai số tốc độ góc (rad/s)
    float err = rate_sp - rate_meas;

    // Feedback error tinh khiet (Deadband chi dat tren Setpoint, khong dat trong Feedback loop)

    // 1. Proportional term
    _p_out = _gains.kp * err;

    if (!(out_limit > 0.0f) || out_limit > 1.0f) out_limit = 1.0f;

    // 2. Derivative on Measurement (Tránh spike giật khi đổi Setpoint)
    if (_first_run) {
        _last_rate_meas = rate_meas;
        _first_run = false;
        _d_out = 0.0f;
        _d_lpf.reset(0.0f);
    } else {
        float d_raw = -_gains.kd * ((rate_meas - _last_rate_meas) / dt);
        _last_rate_meas = rate_meas;
        _d_out = _d_lpf.apply(d_raw);
    }

    // 3. Integral term: ngắt khi ga thấp + anti-windup theo giới hạn mô-men thực của Mixer
    if (allow_integral) {
        const float unsat = _p_out + _integral + _d_out;
        const bool push_high = (unsat >=  out_limit) && (err > 0.0f);
        const bool push_low  = (unsat <= -out_limit) && (err < 0.0f);
        if (!push_high && !push_low) {
            _integral += _gains.ki * err * dt;
        }
        if (_integral > _gains.i_max)  _integral = _gains.i_max;
        if (_integral < -_gains.i_max) _integral = -_gains.i_max;
    } else {
        _integral = 0.0f;
    }

    // Tổng hợp mô-men và bão hòa theo quyền điều khiển của Mixer
    float torque_out = _p_out + _integral + _d_out;
    if (torque_out > out_limit)  torque_out = out_limit;
    if (torque_out < -out_limit) torque_out = -out_limit;

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
    : _roll_p(3.0f), _pitch_p(3.0f), _yaw_p(1.5f) {
    _max_roll_rate_rad  = 60.0f * DEG_TO_RAD;  // Giới hạn 60 deg/s phù hợp ESC 50Hz
    _max_pitch_rate_rad = 60.0f * DEG_TO_RAD;  // Giới hạn 60 deg/s
    _max_yaw_rate_rad   = 45.0f * DEG_TO_RAD;  // Giới hạn 45 deg/s
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

void PX4_AttitudeControl::update(const float target_euler[3], const float actual_euler[3], float rate_sp_out[3], bool yaw_valid) {
    // Chuyển đổi sai số góc sang Radian
    float err_roll_rad  = (target_euler[0] - actual_euler[0]) * DEG_TO_RAD;
    float err_pitch_rad = (target_euler[1] - actual_euler[1]) * DEG_TO_RAD;
    float err_yaw_rad   = 0.0f;
    if (yaw_valid) {
        err_yaw_rad = wrapAngle180(target_euler[2] - actual_euler[2]) * DEG_TO_RAD;
    }

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

void PX4_QuadMixer::mixNormalized(float throttle, float roll_torque, float pitch_torque, float yaw_torque, float motor_norm_out[4]) {
    // Nếu throttle quá nhỏ (< 2%), cả 4 động cơ giữ mức 0
    if (throttle < 0.02f) {
        for (int i = 0; i < 4; i++) {
            motor_norm_out[i] = 0.0f;
        }
        return;
    }

    // Giới hạn mô-men Roll/Pitch/Yaw theo tỷ lệ mức ga để chống dập tắt động cơ khi ở mức ga thấp
    const float max_authority = torqueAuthority(throttle);

    roll_torque  = constrain(roll_torque,  -max_authority, max_authority);
    pitch_torque = constrain(pitch_torque, -max_authority, max_authority);
    yaw_torque   = constrain(yaw_torque,   -max_authority * 0.5f, max_authority * 0.5f);

    // Cấu hình Quad-X theo bố trí vật lý ĐÃ XÁC MINH trên khung thật (2026-11-24), nhìn từ trên, đầu hướng lên:
    //      M1 GPIO7 Trước-Trái CCW    M4 GPIO4 Trước-Phải CW
    //      M3 GPIO5 Sau-Trái   CW     M2 GPIO6 Sau-Phải   CCW
    // Quy ước dấu cảm biến Xsens (đã xác minh):
    //   Roll  + : cánh PHẢI hạ xuống         => motor TRÁI (M1,M3) tăng tạo Roll +
    //   Pitch + : CHÚC đầu xuống (ngóc = âm)  => motor SAU  (M2,M3) tăng tạo Pitch +
    //   Yaw   + : xoay CÙNG chiều kim đồng hồ nhìn từ trên (chiều la bàn; firmware đảo dấu Yaw của
    //             Xsens bằng xsens_mti_set_yaw_inversion(true))
    //             => cặp cánh CCW (M1,M2) tăng: phản mô-men làm thân xoay CW => Yaw +
    // Mỗi mô-men dương do PID xuất ra PHẢI tạo gia tốc góc dương trên đúng trục cảm biến (hồi tiếp âm).
    // Bản cũ đặt +P cho motor TRƯỚC => vòng Pitch HỒI TIẾP DƯƠNG (tự lật tới ngưỡng failsafe 60°).
    // Cột Yaw phải đổi dấu CÙNG LÚC với dấu Yaw của cảm biến, nếu không vòng Yaw thành hồi tiếp dương.
    //
    // Motor 1: GPIO 7 (Front-Left,  CCW) = T + R - P + Y   // Front nhận -P
    // Motor 2: GPIO 6 (Rear-Right,  CCW) = T - R + P + Y   // Rear  nhận +P
    // Motor 3: GPIO 5 (Rear-Left,   CW)  = T + R + P - Y   // Rear  nhận +P
    // Motor 4: GPIO 4 (Front-Right, CW)  = T - R - P - Y   // Front nhận -P
    float m[4];
    m[0] = throttle + roll_torque - pitch_torque + yaw_torque;
    m[1] = throttle - roll_torque + pitch_torque + yaw_torque;
    m[2] = throttle + roll_torque + pitch_torque - yaw_torque;
    m[3] = throttle - roll_torque - pitch_torque - yaw_torque;

    // Thuật toán Desaturation chuẩn PX4: Co giãn bảo toàn hình học mô-men (Dynamic Scale & Shift)
    float max_val = m[0];
    float min_val = m[0];
    for (int i = 1; i < 4; i++) {
        if (m[i] > max_val) max_val = m[i];
        if (m[i] < min_val) min_val = m[i];
    }

    const float min_floor = 0.08f; // Sàn ga tối thiểu 8% (~1080 us) để motor luôn quay, chống ngắt tắt động cơ khi nghiêng ở ga thấp
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

    // Xuất ra mảng lệnh chuẩn hóa [0.0 .. 1.0]
    for (int i = 0; i < 4; i++) {
        motor_norm_out[i] = constrain(m[i], 0.0f, 1.0f);
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
      _target_throttle_raw(0.0f), _throttle_slew_per_sec(2.0f),
      _integral_min_throttle(PidDefaults::INTEGRAL_MIN_THROTTLE),
      _filtered_gx(0.0f), _filtered_gy(0.0f), _filtered_gz(0.0f),
      _invert_pitch(false), _invert_yaw(false), _yaw_torque_enabled(true), _yaw_rate_ff(0.0f), _pid_enabled(true) {
    _rate_sp_rad[0] = 0.0f; _rate_sp_rad[1] = 0.0f; _rate_sp_rad[2] = 0.0f;
    _tau[0] = 0.0f; _tau[1] = 0.0f; _tau[2] = 0.0f;
    for (int i = 0; i < 4; i++) {
        _motor_norm[i] = 0.0f;
        _motor_outputs_us[i] = ESC_PULSE_DISARM_US;
    }
}

void PX4_FlightController::init(float loop_freq_hz) {
    // Cài đặt bộ lọc Butterworth bậc 2 cắt ở 30Hz cho Gyro
    _gyro_filter_x.setCutoffFrequency(loop_freq_hz, 30.0f);
    _gyro_filter_y.setCutoffFrequency(loop_freq_hz, 30.0f);
    _gyro_filter_z.setCutoffFrequency(loop_freq_hz, 30.0f);

    // Inner Rate PID Roll & Pitch (rad/s): Kp, Ki, Kd, i_max, d_cutoff, loop_freq — xem PidDefaults
    // - Kp thấp hơn bản cũ (0.06 -> 0.040): trễ tổng ~35 ms (ESC 50 Hz + motor + Xsens 100 Hz) giới hạn
    //   băng thông vòng trong ~10 rad/s; Kp 0.06 mất ổn định trên khung nhẹ.
    // - Kd 0.0010 với lọc 30 Hz (bản cũ lọc 10 Hz làm mất pha dẫn của D) bù pha cho trễ.
    // - Ki/i_max lớn hơn để thắng mô-men trọng lực khi trọng tâm lệch trục xoay của giá thử.
    _rate_pid_roll.init (PidDefaults::RP_KP, PidDefaults::RP_KI, PidDefaults::RP_KD, PidDefaults::RP_IMAX, PidDefaults::RP_DCUT, loop_freq_hz);
    _rate_pid_pitch.init(PidDefaults::RP_KP, PidDefaults::RP_KI, PidDefaults::RP_KD, PidDefaults::RP_IMAX, PidDefaults::RP_DCUT, loop_freq_hz);

    // Yaw: mô-men Yaw do chênh lệch lực cản cánh quạt yếu hơn Roll/Pitch nhiều lần, nên cần hệ số
    // lớn hơn (bản cũ Kp=0.10, Ki=0.02, i_max=0.08 => mô-men tối đa ~0.08, không thắng nổi ma sát
    // ổ trục giá thử => drone không xoay). Băng thông Yaw thấp (< 2 Hz) nên ESC 50 Hz không gây mất ổn định.
    _rate_pid_yaw.init  (PidDefaults::Y_KP, PidDefaults::Y_KI, PidDefaults::Y_KD, PidDefaults::Y_IMAX, PidDefaults::Y_DCUT, loop_freq_hz);

    // Vòng ngoài (angle): 4.0 (bản cũ 3.0) — đủ cứng để chống con lắc trọng lực trên giá thử
    _att_control.setAngleGains(PidDefaults::ANG_RP, PidDefaults::ANG_RP, PidDefaults::ANG_Y);

    // Đặt giới hạn tốc độ góc thấp phù hợp ESC 50Hz (gọi sau init)
    _att_control.setMaxRates(60.0f * DEG_TO_RAD, 60.0f * DEG_TO_RAD, 45.0f * DEG_TO_RAD);

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
    // Chỉ lưu yêu cầu của người dùng; việc làm mượt diễn ra trong update(), TRƯỚC khi vào mixer
    _target_throttle_raw = constrain(throttle_pct / 100.0f, 0.0f, 0.65f); // Giới hạn ga an toàn tối đa 65% cho pin 4S
}

void PX4_FlightController::setThrottleSlewRate(float slew_per_sec) {
    if (slew_per_sec > 0.0f) {
        _throttle_slew_per_sec = slew_per_sec;
    }
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

    _target_throttle = 0.0f;
    _target_throttle_raw = 0.0f;

    _rate_sp_rad[0] = 0.0f; _rate_sp_rad[1] = 0.0f; _rate_sp_rad[2] = 0.0f;
    _tau[0] = 0.0f; _tau[1] = 0.0f; _tau[2] = 0.0f;
    _yaw_rate_ff = 0.0f;

    for (int i = 0; i < 4; i++) {
        _motor_norm[i] = 0.0f;
        _motor_outputs_us[i] = ESC_PULSE_DISARM_US;
    }
}

void PX4_FlightController::resetIntegrators() {
    // KHÔNG reset bộ lọc Gyro về 0 ở đây: lệnh này được gọi lúc đang bay; ép bộ lọc về 0 tạo bước nhảy
    // giả trên tốc độ góc đã lọc => xung P/D đúng lúc vừa bật PID.
    _rate_pid_roll.reset();
    _rate_pid_pitch.reset();
    _rate_pid_yaw.reset();
    _att_control.reset();

    _rate_sp_rad[0] = 0.0f; _rate_sp_rad[1] = 0.0f; _rate_sp_rad[2] = 0.0f;
    _tau[0] = 0.0f; _tau[1] = 0.0f; _tau[2] = 0.0f;
    _yaw_rate_ff = 0.0f;
    // GIỮ NGUYÊN _target_throttle và _target_throttle_raw để duy trì lực nâng khi khôi phục ổn định
}

void PX4_FlightController::update(float actual_roll, float actual_pitch, float actual_yaw,
                                  float raw_gx, float raw_gy, float raw_gz,
                                  float dt, bool is_armed, bool yaw_valid) {
    // 1. Lọc nhiễu rung cơ học trên Gyro bằng bộ lọc 2nd-Order Butterworth LPF (30Hz)
    _filtered_gx = _gyro_filter_x.apply(raw_gx);
    _filtered_gy = _gyro_filter_y.apply(raw_gy);
    _filtered_gz = _gyro_filter_z.apply(raw_gz);

    // Chuyển tốc độ góc thực sang Radian/giây (rad/s)
    float actual_gx_rad = _filtered_gx * DEG_TO_RAD;
    float actual_gy_rad = _filtered_gy * DEG_TO_RAD;
    float actual_gz_rad = _filtered_gz * DEG_TO_RAD;

    // Nếu chưa Arm: Khóa cứng an toàn
    if (!is_armed) {
        reset();
        for (int i = 0; i < 4; i++) {
            _motor_norm[i] = 0.0f;
            _motor_outputs_us[i] = ESC_PULSE_DISARM_US;
        }
        return;
    }

    // ===== THÊM: LÀM MƯỢT SETPOINT GA (Throttle Slew Limiter) =====
    // Đây là bước khắc phục chính cho hiện tượng mất ổn định khi đẩy ga đột ngột:
    // làm mượt NGAY TẠI setpoint (một giá trị scalar duy nhất) trước khi mixer dùng nó để
    // tính max_authority và phân bổ cho 4 động cơ, tránh méo lệch tỷ lệ mô-men giữa các cánh.
    {
        float max_step = _throttle_slew_per_sec * dt;
        float diff = _target_throttle_raw - _target_throttle;
        if (fabsf(diff) <= max_step) {
            _target_throttle = _target_throttle_raw;
        } else {
            _target_throttle += (diff > 0.0f) ? max_step : -max_step;
        }
    }
    // ===== HẾT PHẦN THÊM =====

    // Nếu Armed nhưng mức ga < 2%: Giữ động cơ ở mức Idle Spin an toàn, không tích phân PID
    if (_target_throttle < 0.02f) {
        _rate_pid_roll.reset();
        _rate_pid_pitch.reset();
        _rate_pid_yaw.reset();
        _rate_sp_rad[0] = 0.0f; _rate_sp_rad[1] = 0.0f; _rate_sp_rad[2] = 0.0f;
        _tau[0] = 0.0f; _tau[1] = 0.0f; _tau[2] = 0.0f;
        for (int i = 0; i < 4; i++) {
            _motor_norm[i] = 0.0f;
            _motor_outputs_us[i] = ESC_PULSE_IDLE_US;
        }
        return;
    }

    // 2. VÒNG NGOÀI (Outer Angle P-Loop): Tính tốc độ góc mục tiêu (rad/s)
    float target_att[3] = { _target_roll, _target_pitch, _target_yaw };
    float actual_att[3] = { actual_roll,  actual_pitch,  actual_yaw  };

    _att_control.update(target_att, actual_att, _rate_sp_rad, yaw_valid);

    // Feed-forward tốc độ Yaw từ cần gạt: drone bắt đầu xoay NGAY khi gạt cần, không phải chờ
    // sai số góc tích lũy đủ lớn. Khi Yaw không hợp lệ, đây là chế độ rate thuần (vẫn điều khiển được).
    {
        const float yaw_max = _att_control.getMaxYawRateRad();
        _rate_sp_rad[2] = constrain(_rate_sp_rad[2] + _yaw_rate_ff, -yaw_max, yaw_max);
    }

    // 3. VÒNG TRONG (Inner Angular Rate PID-Loop): Tính mô-men điều khiển
    // Tích phân bật từ 15% ga (bản cũ 25%): trên giá thử thường chạy dưới 25% ga, khi đó tích phân bị
    // tắt nên không bù được mô-men trọng lực do lệch trọng tâm => drone ngả dần rồi lật.
    // Đầu ra mỗi vòng giới hạn đúng quyền điều khiển của Mixer => anti-windup chính xác.
    bool allow_integral = (_target_throttle >= _integral_min_throttle);
    const float auth_rp = PX4_QuadMixer::torqueAuthority(_target_throttle);
    const float auth_y  = auth_rp * 0.5f;
    if (_pid_enabled) {
        _tau[0] = _rate_pid_roll.update(_rate_sp_rad[0], actual_gx_rad, dt, allow_integral, auth_rp);
        _tau[1] = _rate_pid_pitch.update(_rate_sp_rad[1], actual_gy_rad, dt, allow_integral, auth_rp);
        if (_invert_pitch) _tau[1] = -_tau[1];
    } else {
        _rate_pid_roll.reset();
        _rate_pid_pitch.reset();
        _tau[0] = 0.0f;
        _tau[1] = 0.0f;
    }
    if (_yaw_torque_enabled) {
        _tau[2] = _rate_pid_yaw.update(_rate_sp_rad[2],  actual_gz_rad, dt, allow_integral, auth_y);
        if (_invert_yaw) _tau[2] = -_tau[2];
    } else {
        _rate_pid_yaw.reset();
        _tau[2] = 0.0f;
    }

    // 4. BỘ PHỐI TRỘN (Mixer / Allocator): Phân bổ mô-men và ga ra 4 kênh chuẩn hóa [0..1]
    _mixer.mixNormalized(_target_throttle, _tau[0], _tau[1], _tau[2], _motor_norm);

    // Ánh xạ ra xung Microsecond cho telemetry
    for (int i = 0; i < 4; i++) {
        _motor_outputs_us[i] = ESC_PULSE_MIN_US + (uint16_t)(_motor_norm[i] * (float)(ESC_PULSE_MAX_US - ESC_PULSE_MIN_US));
    }
}

// ============================================================================
// 5. YAW SETPOINT SHAPING (cần gạt = tốc độ xoay)
// ============================================================================
static float wrap180f(float a) {
    if (isnan(a) || isinf(a)) return 0.0f;
    while (a > 180.0f)  a -= 360.0f;
    while (a < -180.0f) a += 360.0f;
    return a;
}

float yawSetpointStep(float yaw_sp_deg, float rate_cmd_dps, float yaw_deg, float dt, float max_lead_deg) {
    if (isnan(rate_cmd_dps) || isinf(rate_cmd_dps) || !(dt > 0.0f) || dt > 0.1f) return wrap180f(yaw_sp_deg);
    float sp = yaw_sp_deg + rate_cmd_dps * dt;
    const float lead = wrap180f(sp - yaw_deg);
    if (lead >  max_lead_deg) sp = yaw_deg + max_lead_deg;
    if (lead < -max_lead_deg) sp = yaw_deg - max_lead_deg;
    return wrap180f(sp);
}
