#include "pid_autotuner.h"
#include <math.h>

PID_AutoTuner autoTuner;

PID_AutoTuner::PID_AutoTuner() {
    _state              = TUNE_STATE_IDLE;
    _axis               = TUNE_NONE;
    _abort_reason       = ABORT_NONE;
    _relay_amplitude    = 0.15f;
    _min_cycles         = 8;
    _abort_angle_deg    = 25.0f;
    _hysteresis_rad     = 0.04f;
    reset();
}

void PID_AutoTuner::configure(float relay_amplitude, int min_cycles, float abort_angle_deg, float hysteresis_rad) {
    _relay_amplitude = constrain(relay_amplitude, 0.05f, 0.30f);
    _min_cycles      = constrain(min_cycles, 4, 20);
    _abort_angle_deg = constrain(abort_angle_deg, 10.0f, 45.0f);
    _hysteresis_rad  = constrain(hysteresis_rad, 0.01f, 0.15f);
}

void PID_AutoTuner::start(TuneAxis axis) {
    if (axis == TUNE_NONE) return;

    reset();
    _axis        = axis;
    _relay_sign  = -1.0f; // Bắt đầu bằng âm để ép kéo về cân bằng
    _state       = TUNE_STATE_RELAY;
    _abort_reason= ABORT_NONE;

    _setAxisName();

    Serial.println("\n==================================================================");
    Serial.printf("[TUNE] BAT DAU RELAY AUTO-TUNE (Astrom-Hagglund)\n");
    Serial.printf("[TUNE] Truc: %s | Bien do relay: %.2f (%.0f%%) | Chu ky can: %d\n",
                  _result.axis_name, _relay_amplitude, _relay_amplitude * 100.0f, _min_cycles);
    Serial.printf("[TUNE] Bang tre: %.3f rad/s | Goc ngat an toan: %.1f deg\n",
                  _hysteresis_rad, _abort_angle_deg);
    Serial.println("==================================================================\n");
}

void PID_AutoTuner::stop(TuneAbortReason reason) {
    if (_state == TUNE_STATE_RELAY || _state == TUNE_STATE_ANALYZING) {
        _state        = TUNE_STATE_ABORTED;
        _abort_reason = reason;
        const char* reasonStr = "User Stop";
        if (reason == ABORT_TILT_LIMIT)   reasonStr = "Nghieng qua 25 deg";
        if (reason == ABORT_TIMEOUT)      reasonStr = "Timeout (khong dao dong)";
        if (reason == ABORT_LOW_THROTTLE) reasonStr = "Ga qua thap (< 2%)";
        Serial.printf("[TUNE] NGAT AUTO-TUNE! Ly do: %s\n", reasonStr);
    }
}

float PID_AutoTuner::update(float rate_meas, float angle_abs, float dt) {
    if (_state != TUNE_STATE_RELAY) return 0.0f;

    // 1. WATCHDOG 1: Kiểm tra góc an toàn tuyệt đối
    if (angle_abs > _abort_angle_deg) {
        stop(ABORT_TILT_LIMIT);
        return 0.0f;
    }

    _time_elapsed += dt;

    // 2. WATCHDOG 2: Kiểm tra timeout không có đảo chiều (3.5 giây)
    if ((_time_elapsed - _last_transition_time) > 3.5f) {
        stop(ABORT_TIMEOUT);
        return 0.0f;
    }

    // 3. RELAY HỒI TIẾP ÂM CÓ BĂNG TRỄ (Hysteresis Restoring Bang-Bang):
    // rate_meas > +hysteresis: drone đang quay theo chiều dương -> phát mô-men âm để kéo lại
    // rate_meas < -hysteresis: drone đang quay theo chiều âm -> phát mô-men dương để kéo lại
    float new_sign = _relay_sign;
    if (rate_meas > _hysteresis_rad) {
        new_sign = -1.0f;
    } else if (rate_meas < -_hysteresis_rad) {
        new_sign = 1.0f;
    }

    // Theo dõi giá trị đỉnh tuyệt đối trong nửa chu kỳ hiện tại
    float abs_rate = fabsf(rate_meas);
    if (abs_rate > _current_half_peak) {
        _current_half_peak = abs_rate;
    }

    // 4. PHÁT HIỆN CHUYỂN TRẠNG THÁI RELAY (Zero-crossing nửa chu kỳ)
    if (new_sign != _relay_sign) {
        float half_period = _time_elapsed - _last_transition_time;

        // Chỉ chấp nhận nửa chu kỳ thực tế (20ms đến 2000ms)
        if (half_period >= 0.025f && half_period <= 2.0f && _last_transition_time > 0.0f) {
            _half_period_sum += half_period;
            _half_period_count++;

            if (_current_half_peak > _hysteresis_rad) {
                _peak_sum += _current_half_peak;
                _peak_count++;
            }

            // Mỗi 2 nửa chu kỳ = 1 chu kỳ hoàn chỉnh
            if (_half_period_count % 2 == 0) {
                _cycle_count = _half_period_count / 2;
                _result.cycles_completed = _cycle_count;

                float tu_curr = (_half_period_sum / _half_period_count) * 2.0f;
                float a_curr  = (_peak_count > 0) ? (_peak_sum / _peak_count) : 0.01f;
                float ku_curr = (4.0f * _relay_amplitude) / (3.14159265f * a_curr);

                Serial.printf("[TUNE] Chu ky %d/%d: Tu=%.3fs | A=%.3f rad/s (%.1f deg/s) | Ku=%.4f\n",
                              _cycle_count, _min_cycles, tu_curr, a_curr, a_curr * 57.2958f, ku_curr);
            }
        }

        _relay_sign          = new_sign;
        _last_transition_time= _time_elapsed;
        _current_half_peak   = 0.0f;
    }

    // 5. KIỂM TRA ĐIỀU KIỆN KẾT THÚC THÀNH CÔNG
    if (_cycle_count >= _min_cycles) {
        _state = TUNE_STATE_ANALYZING;
        _analyze();
        _state = TUNE_STATE_DONE;
        return 0.0f;
    }

    // Xuất mô-men relay
    return _relay_sign * _relay_amplitude;
}

void PID_AutoTuner::_analyze() {
    if (_half_period_count < 4 || _peak_count < 2) {
        _result.valid = false;
        Serial.println("[TUNE] Loi: Khong du so lieu dao dong hop le de tinh toan!");
        return;
    }

    float avg_half_period = _half_period_sum / (float)_half_period_count;
    float avg_peak        = _peak_sum / (float)_peak_count;

    // Chu kỳ tới hạn Tu (giây)
    float tu = avg_half_period * 2.0f;

    // Biên độ sau khi bù băng trễ: A_eff = sqrt(A^2 - eps^2)
    float a_eff = avg_peak;
    if (avg_peak > _hysteresis_rad) {
        a_eff = sqrtf((avg_peak * avg_peak) - (_hysteresis_rad * _hysteresis_rad));
    }
    if (a_eff < 0.005f) a_eff = 0.005f;

    // Hệ số khuếch đại tới hạn Ku
    float ku = (4.0f * _relay_amplitude) / (3.1415926535f * a_eff);

    _result.ku = ku;
    _result.tu = tu;

    // 1. ZIEGLER-NICHOLS STANDARD (Chuẩn cổ điển)
    _result.kp = 0.60f * ku;
    _result.ki = 1.20f * ku / tu;
    _result.kd = 0.075f * ku * tu;

    // 2. ZIEGLER-NICHOLS SMOOTH (Khuyên dùng cho Quadcopter để chống rung dải cao)
    _result.kp_smooth = 0.40f * ku;
    _result.ki_smooth = 0.60f * ku / tu;
    _result.kd_smooth = 0.050f * ku * tu;

    // 3. GỢI Ý OUTER ANGLE P (Dựa trên băng thông vòng trong: P_angle ≈ 1.05 / Tu)
    // Giới hạn trong khoảng hợp lý [3.5 .. 9.0]
    _result.angle_p = constrain(1.05f / tu, 3.5f, 9.0f);

    _result.valid = true;

    Serial.println("\n==================================================================");
    Serial.printf("[TUNE] HOAN THANH AUTO-TUNE CHO TRUC %s!\n", _result.axis_name);
    Serial.printf("[TUNE] Thong so vat ly: Tu = %.3f s | Ku = %.4f | Peak_A = %.3f rad/s\n",
                  tu, ku, avg_peak);
    Serial.println("------------------------------------------------------------------");
    Serial.printf("[TUNE] [1] Bo so ZN CHUAN (Fast / Dynamic):\n");
    Serial.printf("           Kp = %.4f | Ki = %.4f | Kd = %.5f\n",
                  _result.kp, _result.ki, _result.kd);
    Serial.printf("[TUNE] [2] Bo so ZN MUOT MA (Smooth / Recommended):\n");
    Serial.printf("           Kp = %.4f | Ki = %.4f | Kd = %.5f\n",
                  _result.kp_smooth, _result.ki_smooth, _result.kd_smooth);
    Serial.printf("[TUNE] [3] Outer Angle P khuyen nghi: %.2f\n", _result.angle_p);
    Serial.println("------------------------------------------------------------------");
    Serial.printf("[TUNE] Lenh ap dung nhanh: 'tune apply' hoac 'tune apply 50'\n");
    Serial.println("==================================================================\n");
}

void PID_AutoTuner::reset() {
    _relay_sign           = -1.0f;
    _last_transition_time = 0.0f;
    _time_elapsed         = 0.0f;
    _half_period_sum      = 0.0f;
    _half_period_count    = 0;
    _cycle_count          = 0;
    _peak_sum             = 0.0f;
    _peak_count           = 0;
    _current_half_peak    = 0.0f;

    _result = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false, "", 0 };
    _setAxisName();
}

void PID_AutoTuner::_setAxisName() {
    switch (_axis) {
        case TUNE_ROLL_RATE:  strncpy(_result.axis_name, "roll",  11); break;
        case TUNE_PITCH_RATE: strncpy(_result.axis_name, "pitch", 11); break;
        case TUNE_YAW_RATE:   strncpy(_result.axis_name, "yaw",   11); break;
        case TUNE_ANGLE_P:    strncpy(_result.axis_name, "angle", 11); break;
        default:              strncpy(_result.axis_name, "none",  11); break;
    }
    _result.axis_name[11] = '\0';
}

int PID_AutoTuner::buildStatusJson(char* buf, int buf_len) const {
    bool active = (_state == TUNE_STATE_RELAY || _state == TUNE_STATE_ANALYZING);
    int n = snprintf(buf, buf_len,
        "\"tune\":{"
        "\"active\":%s,"
        "\"axis\":\"%s\","
        "\"cycle\":%d,"
        "\"min\":%d,"
        "\"ku\":%.4f,"
        "\"tu\":%.3f,"
        "\"kp\":%.4f,"
        "\"ki\":%.4f,"
        "\"kd\":%.5f,"
        "\"kp_s\":%.4f,"
        "\"ki_s\":%.4f,"
        "\"kd_s\":%.5f,"
        "\"ang_p\":%.2f,"
        "\"valid\":%s,"
        "\"state\":%d,"
        "\"err\":%d"
        "}",
        active ? "true" : "false",
        _result.axis_name,
        _cycle_count,
        _min_cycles,
        _result.ku,
        _result.tu,
        _result.kp,
        _result.ki,
        _result.kd,
        _result.kp_smooth,
        _result.ki_smooth,
        _result.kd_smooth,
        _result.angle_p,
        _result.valid ? "true" : "false",
        (int)_state,
        (int)_abort_reason
    );
    return n;
}
