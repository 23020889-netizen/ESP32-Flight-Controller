#pragma once

#include <cmath>
#include <algorithm>

/**
 * @brief Mathematical utilities for Integral Terminal Sliding Mode Control.
 */
class ItsmcMath {
public:
    /**
     * @brief Computes terminal non-linear power function: |x|^alpha * sign(x).
     * @param value Input scalar value.
     * @param alpha Fractional power exponent (e.g. 0.5).
     * @return Transformed terminal scalar value.
     */
    static inline float terminalPower(float value, float alpha) {
        if (std::abs(value) < 1.0e-6f) {
            return 0.0f;
        }
        float magnitude = std::pow(std::abs(value), alpha);
        return (value >= 0.0f) ? magnitude : -magnitude;
    }

    /**
     * @brief Clamps a float value between minimum and maximum bounds.
     * @param value Input value.
     * @param minimum Lower bound.
     * @param maximum Upper bound.
     * @return Clamped value.
     */
    static inline float clamp(float value, float minimum, float maximum) {
        return std::max(minimum, std::min(maximum, value));
    }

    /**
     * @brief Hyperbolic tangent function for smooth boundary layer switching.
     * @param value Input sliding surface value.
     * @return Output smooth switching value in range (-1.0, 1.0).
     */
    static inline float smoothSign(float value) {
        return std::tanh(value);
    }

    /**
     * @brief Safe arc-sine with boundary clamping to avoid NaN.
     * @param value Ratio argument.
     * @return Angle in radians.
     */
    static inline float safeArcSine(float value) {
        return std::asin(clamp(value, -0.999f, 0.999f));
    }
};

