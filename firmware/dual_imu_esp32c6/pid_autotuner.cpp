#include "pid_autotuner.h"
#include <math.h>

extern void telemetry_wifi_broadcast_msg(const char* msg);

PID_AutoTuner autoTuner;

PID_AutoTuner::PID_AutoTuner() {
    _state              = TUNE_STATE_IDLE;
    _axis               = TUNE_NONE;
    _abort_reason       = ABORT_NONE;
    _relay_amplitude    = AMP_START;
    _relay_amp_max      = 0.15f;
    _amp_settle_halves  = 0;
    _bias               = 0.0f;
    _min_cycles         = 8;
    _abort_angle_deg    = 25.0f;
    _hysteresis_rad     = 0.04f;
    reset();
}

void PID_AutoTuner::configure(float relay_amplitude, int min_cycles, float abort_angle_deg, float hysteresis_rad) {
    // relay_amplitude = biên độ TỐI ĐA; relay bắt đầu nhỏ và tự tăng/giảm để dao động vừa phải
    _relay_amp_max   = constrain(relay_amplitude, AMP_MIN, 0.30f);
    _min_cycles      = constrain(min_cycles, 4, 20);
    _abort_angle_deg = constrain(abort_angle_deg, 10.0f, 45.0f);
    _hysteresis_rad  = constrain(hysteresis_rad, 0.01f, 0.15f);
}

void PID_AutoTuner::start(TuneAxis axis, float initial_bias) {
    if (axis == TUNE_NONE) return;

    reset();
    _relay_amplitude   = (AMP_START < _relay_amp_max) ? AMP_START : _relay_amp_max;
    _amp_settle_halves = 0;
    _bias              = (isnan(initial_bias) || isinf(initial_bias)) ? 0.0f : constrain(initial_bias, -BIAS_MAX, BIAS_MAX);
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

    // 1. WATCHDOG 1: góc an toàn. Khi vượt ngưỡng, firmware TRẢ QUYỀN cho PID giữ thăng bằng (không
    //    DISARM), nên luôn tốt hơn để trục trôi tới ngưỡng failsafe 60° rồi bị ngắt motor.
    if (angle_abs > _abort_angle_deg) {
        stop(ABORT_TILT_LIMIT);
        return 0.0f;
    }

    _time_elapsed += dt;

    // 2. WATCHDOG 2: Kiểm tra timeout không có đảo chiều (8.0 giây)
    if ((_time_elapsed - _last_transition_time) > 8.0f) {
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
    // WATCHDOG 3: dao động phân kỳ (tốc độ góc quá lớn) => ngắt ngay, trả quyền cho PID
    if (abs_rate > RATE_ABORT) {
        stop(ABORT_TILT_LIMIT);
        return 0.0f;
    }
    if (abs_rate > _current_half_peak) {
        _current_half_peak = abs_rate;
    }

    // 4. PHÁT HIỆN CHUYỂN TRẠNG THÁI RELAY (Zero-crossing nửa chu kỳ)
    if (new_sign != _relay_sign) {
        float half_period = _time_elapsed - _last_transition_time;

        // Thích nghi biên độ relay theo đỉnh vừa đo; nếu vừa đổi biên độ thì đo lại từ đầu
        // (Ku = 4d/(pi*A) chỉ đúng khi d không đổi trong suốt các chu kỳ được lấy trung bình).
        bool amp_changed = false;
        if (_last_transition_time > 0.0f) amp_changed = _adaptAmplitude(_current_half_peak);

        // Chỉ chấp nhận nửa chu kỳ thực tế (20ms đến 2000ms) khi biên độ đã ổn định
        if (!amp_changed && _amp_settle_halves >= 2 &&
            half_period >= 0.025f && half_period <= 2.0f && _last_transition_time > 0.0f) {
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
                char cycMsg[128];
                snprintf(cycMsg, sizeof(cycMsg), "[TUNE] Chu ky %d/%d: Tu=%.3fs | A=%.1f deg/s | Ku=%.4f",
                         _cycle_count, _min_cycles, tu_curr, a_curr * 57.2958f, ku_curr);
                telemetry_wifi_broadcast_msg(cycMsg);
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

    // Luật chỉnh định được chọn bằng mô phỏng (tests/host_firmware/stand_sim.cpp): chạy chính auto-tune này trên
    // khung độ cứng 200..1300 rad/s^2 (ESC 50 Hz, Xsens 100 Hz, có/không con lắc trọng lực), áp từng luật lên
    // Ku/Tu đo được rồi kiểm tra vòng kín ở cả mô hình danh nghĩa lẫn khung cứng hơn 30% + trễ thêm 10 ms.
    // - ZN cổ điển (0.6Ku) mất ổn định ở 3/10 khung khi lệch mô hình => KHÔNG dùng.
    // - Tyreus-Luyben (tích phân yếu) không thắng nổi con lắc trọng lực ở khung cứng => KHÔNG dùng.
    // - "ZN không vọt lố" 0.40Ku và bản mềm 0.7x: ổn định 10/10 khung, cả khi lệch mô hình.
    auto clampf = [](float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); };

    // 1. CHUẨN (lệnh "tune apply"): Kp = 0.40 Ku, Ti = Tu/1.5, Td = Tu/8
    _result.kp = clampf(0.40f * ku,              0.008f, 0.15f);
    _result.ki = clampf(0.60f * ku / tu,         0.0f,   1.0f);
    _result.kd = clampf(0.050f * ku * tu,        0.0f,   0.006f);

    // 2. MỀM (lệnh "tune apply 50"): 0.7 x bộ chuẩn, thêm biên độ an toàn
    _result.kp_smooth = clampf(0.28f * ku,       0.008f, 0.15f);
    _result.ki_smooth = clampf(0.42f * ku / tu,  0.0f,   1.0f);
    _result.kd_smooth = clampf(0.035f * ku * tu, 0.0f,   0.006f);

    // 3. GỢI Ý OUTER ANGLE P theo băng thông vòng trong: 1.05 / Tu, giới hạn [3.0 .. 6.0]
    //    (bản cũ cho tới 9.0 — quá cứng với trễ của ESC 50 Hz)
    _result.angle_p = clampf(1.05f / tu, 3.0f, 6.0f);

    _result.valid = true;

    Serial.println("\n==================================================================");
    Serial.printf("[TUNE] HOAN THANH AUTO-TUNE CHO TRUC %s!\n", _result.axis_name);
    Serial.printf("[TUNE] Thong so vat ly: Tu = %.3f s | Ku = %.4f | Peak_A = %.3f rad/s\n",
                  tu, ku, avg_peak);
    Serial.println("------------------------------------------------------------------");
    Serial.printf("[TUNE] [1] Bo so CHUAN (0.40 Ku, 'tune apply'):\n");
    Serial.printf("           Kp = %.4f | Ki = %.4f | Kd = %.5f\n",
                  _result.kp, _result.ki, _result.kd);
    Serial.printf("[TUNE] [2] Bo so MEM (0.7x, 'tune apply 50'):\n");
    Serial.printf("           Kp = %.4f | Ki = %.4f | Kd = %.5f\n",
                  _result.kp_smooth, _result.ki_smooth, _result.kd_smooth);
    Serial.printf("[TUNE] [3] Outer Angle P khuyen nghi: %.2f\n", _result.angle_p);
    Serial.println("------------------------------------------------------------------");
    Serial.printf("[TUNE] Lenh ap dung nhanh: 'tune apply' hoac 'tune apply 50'\n");
    Serial.println("==================================================================\n");

    char donMsg[160];
    snprintf(donMsg, sizeof(donMsg), "[TUNE] HOAN THANH %s: Kp=%.4f Ki=%.4f Kd=%.5f | Angle_P=%.2f",
             _result.axis_name, _result.kp_smooth, _result.ki_smooth, _result.kd_smooth, _result.angle_p);
    telemetry_wifi_broadcast_msg(donMsg);
}

void PID_AutoTuner::_resetMeasurement() {
    _half_period_sum   = 0.0f;
    _half_period_count = 0;
    _cycle_count       = 0;
    _peak_sum          = 0.0f;
    _peak_count        = 0;
    _result.cycles_completed = 0;
}

bool PID_AutoTuner::_adaptAmplitude(float half_peak) {
    float new_amp = _relay_amplitude;
    if (half_peak > AMP_TARGET_HI)                    new_amp = _relay_amplitude * 0.75f;
    else if (half_peak < AMP_TARGET_LO)               new_amp = _relay_amplitude * 1.25f;
    if (new_amp > _relay_amp_max) new_amp = _relay_amp_max;
    if (new_amp < AMP_MIN)        new_amp = AMP_MIN;
    if (fabsf(new_amp - _relay_amplitude) > 1e-5f) {
        _relay_amplitude = new_amp;
        _amp_settle_halves = 0;
        _resetMeasurement();
        return true;
    }
    if (_amp_settle_halves < 1000) _amp_settle_halves++;
    return false;
}

float PID_AutoTuner::updateWithAngleHold(float rate_meas_rad, float axis_angle_deg, float tilt_abs_deg, float dt) {
    const float ang_rad = axis_angle_deg * 0.017453292519943295f;
    float hold = -HOLD_P * ang_rad;
    if (hold >  HOLD_MAX_RAD) hold =  HOLD_MAX_RAD;
    if (hold < -HOLD_MAX_RAD) hold = -HOLD_MAX_RAD;
    const float relay = update(rate_meas_rad - hold, tilt_abs_deg, dt);
    if (_state != TUNE_STATE_RELAY) return 0.0f;      // kết thúc/ngắt: không xuất mô-men tune
    if (dt > 0.0f && dt < 0.1f) {
        _bias -= BIAS_KI * ang_rad * dt;
        if (_bias >  BIAS_MAX) _bias =  BIAS_MAX;
        if (_bias < -BIAS_MAX) _bias = -BIAS_MAX;
    }
    return _bias + relay;
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
