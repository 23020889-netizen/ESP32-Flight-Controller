#include "px4_filters.h"
#include <math.h>

// ==========================================
// LOW PASS FILTER 2ND ORDER (BUTTERWORTH)
// ==========================================

LowPassFilter2p::LowPassFilter2p()
    : _cutoff_freq(0.0f), _sample_freq(0.0f),
      _a1(0.0f), _a2(0.0f), _b0(1.0f), _b1(0.0f), _b2(0.0f),
      _delay_element_1(0.0f), _delay_element_2(0.0f) {}

LowPassFilter2p::LowPassFilter2p(float sample_freq, float cutoff_freq)
    : _cutoff_freq(0.0f), _sample_freq(0.0f),
      _a1(0.0f), _a2(0.0f), _b0(1.0f), _b1(0.0f), _b2(0.0f),
      _delay_element_1(0.0f), _delay_element_2(0.0f) {
    setCutoffFrequency(sample_freq, cutoff_freq);
}

void LowPassFilter2p::setCutoffFrequency(float sample_freq, float cutoff_freq) {
    _cutoff_freq = cutoff_freq;
    _sample_freq = sample_freq;

    if (_cutoff_freq <= 0.0f || _sample_freq <= 0.0f) {
        // Bypass filter
        _b0 = 1.0f;
        _b1 = 0.0f;
        _b2 = 0.0f;
        _a1 = 0.0f;
        _a2 = 0.0f;
        return;
    }

    // Tần số Nyquist giới hạn: cutoff không được vượt quá fs / 2
    if (_cutoff_freq >= _sample_freq * 0.5f) {
        _cutoff_freq = _sample_freq * 0.499f;
    }

    const float fr = _sample_freq / _cutoff_freq;
    const float ohm = tanf((float)M_PI / fr);
    const float sqrt2 = 1.41421356237f;
    const float c = 1.0f + sqrt2 * ohm + ohm * ohm;

    _b0 = (ohm * ohm) / c;
    _b1 = 2.0f * _b0;
    _b2 = _b0;
    _a1 = 2.0f * (ohm * ohm - 1.0f) / c;
    _a2 = (1.0f - sqrt2 * ohm + ohm * ohm) / c;
}

float LowPassFilter2p::apply(float sample) {
    if (_cutoff_freq <= 0.0f) {
        return sample;
    }

    // Direct Form II
    float delay_element_0 = sample - _delay_element_1 * _a1 - _delay_element_2 * _a2;

    if (isnan(delay_element_0) || isinf(delay_element_0)) {
        reset(sample);
        return sample;
    }

    float output = delay_element_0 * _b0 + _delay_element_1 * _b1 + _delay_element_2 * _b2;
    _delay_element_2 = _delay_element_1;
    _delay_element_1 = delay_element_0;

    return output;
}

void LowPassFilter2p::reset(float sample) {
    float denom = 1.0f + _a1 + _a2;
    if (fabsf(denom) > 1e-6f) {
        _delay_element_1 = sample / denom;
        _delay_element_2 = _delay_element_1;
    } else {
        _delay_element_1 = sample;
        _delay_element_2 = sample;
    }
}

// ==========================================
// LOW PASS FILTER 1ST ORDER (ALPHA / RC)
// ==========================================

LowPassFilter1p::LowPassFilter1p()
    : _alpha(1.0f), _state(0.0f), _initialized(false) {}

LowPassFilter1p::LowPassFilter1p(float sample_freq, float cutoff_freq)
    : _alpha(1.0f), _state(0.0f), _initialized(false) {
    setCutoffFrequency(sample_freq, cutoff_freq);
}

void LowPassFilter1p::setCutoffFrequency(float sample_freq, float cutoff_freq) {
    if (cutoff_freq <= 0.0f || sample_freq <= 0.0f) {
        _alpha = 1.0f;
        return;
    }
    const float dt = 1.0f / sample_freq;
    const float rc = 1.0f / (2.0f * (float)M_PI * cutoff_freq);
    _alpha = dt / (rc + dt);
    if (_alpha > 1.0f) _alpha = 1.0f;
    if (_alpha < 0.0f) _alpha = 0.0f;
}

float LowPassFilter1p::apply(float sample) {
    if (!_initialized) {
        _state = sample;
        _initialized = true;
        return _state;
    }
    _state += _alpha * (sample - _state);
    return _state;
}

void LowPassFilter1p::reset(float sample) {
    _state = sample;
    _initialized = true;
}
