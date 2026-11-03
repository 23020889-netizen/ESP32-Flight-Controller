#ifndef PX4_FILTERS_H
#define PX4_FILTERS_H

#include <Arduino.h>

/**
 * @brief Bộ lọc thông thấp bậc 2 Butterworth (2nd-Order Butterworth Low Pass Filter)
 * Khớp 100% thuật toán math::LowPassFilter2p trong mã nguồn PX4 Autopilot.
 * Được sử dụng để lọc triệt để rung lắc cơ khí của động cơ/cánh quạt trên Gyro và Accel.
 */
class LowPassFilter2p {
public:
    LowPassFilter2p();
    LowPassFilter2p(float sample_freq, float cutoff_freq);

    // Cấu hình tần số lấy mẫu và tần số cắt
    void setCutoffFrequency(float sample_freq, float cutoff_freq);

    // Lọc mẫu mới
    float apply(float sample);

    // Đặt lại trạng thái bộ lọc
    void reset(float sample = 0.0f);

    float getCutoffFreq() const { return _cutoff_freq; }
    float getSampleFreq() const { return _sample_freq; }

private:
    float _cutoff_freq;
    float _sample_freq;
    float _a1;
    float _a2;
    float _b0;
    float _b1;
    float _b2;
    float _delay_element_1;
    float _delay_element_2;
};

/**
 * @brief Bộ lọc thông thấp bậc 1 (1st-Order Alpha/RC Low Pass Filter)
 * Sử dụng cho D-term smoothing và các chỉ số phụ trợ.
 */
class LowPassFilter1p {
public:
    LowPassFilter1p();
    LowPassFilter1p(float sample_freq, float cutoff_freq);

    void setCutoffFrequency(float sample_freq, float cutoff_freq);
    float apply(float sample);
    void reset(float sample = 0.0f);

private:
    float _alpha;
    float _state;
    bool  _initialized;
};

#endif // PX4_FILTERS_H
