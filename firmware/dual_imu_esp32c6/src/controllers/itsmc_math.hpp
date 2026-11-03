#pragma once

#include <cmath>
#include <algorithm>

/**
 * @brief Non-singular mathematical utilities for robust Quadrotor ITSMC flight control.
 */
class ItsmcMath {
public:
    /**
     * @brief Computes non-singular terminal power with linear transition near zero error.
     * Prevents infinite derivative gain and eliminates 4Hz limit cycle oscillations.
     * @param error Error value in radians or meters.
     * @param linearThreshold Linear boundary threshold (default 0.035 rad ~ 2.0 deg).
     * @return Non-singular terminal value.
     */
    static inline float terminalPowerNonSingular(float error, float linearThreshold = 0.035f) {
        float absoluteError = std::abs(error);
        if (absoluteError < 1.0e-6f) {
            return 0.0f;
        }
        if (absoluteError < linearThreshold) {
            float sqrtThreshold = std::sqrt(linearThreshold);
            return error / sqrtThreshold; // Smooth linear slope
        }
        float magnitude = std::sqrt(absoluteError);
        return (error >= 0.0f) ? magnitude : -magnitude;
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
     * @brief Smooth boundary layer hyperbolic tangent switching: tanh(s / phi_b).
     * Eliminates high-frequency mechanical chattering and actuator saturation.
     * @param slidingSurface Sliding surface value s.
     * @param boundaryThickness Boundary layer thickness phi_b.
     * @return Smooth switching signal in range (-1.0, 1.0).
     */
    static inline float smoothSign(float slidingSurface, float boundaryThickness = 0.20f) {
        if (boundaryThickness <= 1.0e-4f) boundaryThickness = 0.20f;
        return std::tanh(slidingSurface / boundaryThickness);
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
