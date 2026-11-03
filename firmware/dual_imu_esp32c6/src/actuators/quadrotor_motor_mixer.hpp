#pragma once

#include <stdint.h>

/**
 * @brief Normalized motor throttle outputs for 4 quadrotor motors (range 0.0 to 1.0).
 */
struct QuadMotorThrottleOutput {
    float motor1FrontRight = 0.0f; // CCW (GPIO 7)
    float motor2RearLeft = 0.0f;   // CCW (GPIO 6)
    float motor3FrontLeft = 0.0f;  // CW  (GPIO 5)
    float motor4RearRight = 0.0f;  // CW  (GPIO 4)
};

/**
 * @brief Quadrotor X-geometry Motor Mixer mapping thrust and 3-axis torques to motors.
 */
class QuadrotorMotorMixer {
public:
    /**
     * @brief Constructs Quadrotor Motor Mixer.
     * @param armLengthMeters Center-to-motor arm distance.
     */
    explicit QuadrotorMotorMixer(float armLengthMeters = 0.2f);

    /**
     * @brief Calculates normalized throttle outputs for 4 motors with desaturation.
     * @param normalizedThrust Base thrust request (0.0 to 1.0).
     * @param rollTorque Normalized roll torque command.
     * @param pitchTorque Normalized pitch torque command.
     * @param yawTorque Normalized yaw torque command.
     * @return QuadMotorThrottleOutput struct containing motor throttles in range [0.0, 1.0].
     */
    QuadMotorThrottleOutput computeMixer(
        float normalizedThrust,
        float rollTorque,
        float pitchTorque,
        float yawTorque
    );

    /**
     * @brief Converts normalized motor throttle (0.0 to 1.0) to PWM microseconds (1000 to 2000 us).
     * @param normalizedThrottle Throttle value in range 0.0 to 1.0.
     * @return Pulse width in microseconds.
     */
    static inline uint16_t throttleToMicroseconds(float normalizedThrottle) {
        float clamped = (normalizedThrottle < 0.0f) ? 0.0f : ((normalizedThrottle > 1.0f) ? 1.0f : normalizedThrottle);
        return static_cast<uint16_t>(1000.0f + clamped * 1000.0f);
    }

private:
    float armLengthMeters_;
};

