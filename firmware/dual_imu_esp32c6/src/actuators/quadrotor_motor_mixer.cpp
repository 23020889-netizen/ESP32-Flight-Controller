#include "quadrotor_motor_mixer.hpp"
#include <algorithm>

QuadrotorMotorMixer::QuadrotorMotorMixer(float armLengthMeters)
    : armLengthMeters_(armLengthMeters) {}

QuadMotorThrottleOutput QuadrotorMotorMixer::computeMixer(
    float normalizedThrust,
    float rollTorque,
    float pitchTorque,
    float yawTorque
) {
    // Quad-X mixer matrix equations matching PX4 airframe
    float motorRaw[4];
    motorRaw[0] = normalizedThrust - rollTorque + pitchTorque - yawTorque; // M1 Front-Right (CCW)
    motorRaw[1] = normalizedThrust + rollTorque - pitchTorque - yawTorque; // M2 Rear-Left   (CCW)
    motorRaw[2] = normalizedThrust + rollTorque + pitchTorque + yawTorque; // M3 Front-Left  (CW)
    motorRaw[3] = normalizedThrust - rollTorque - pitchTorque + yawTorque; // M4 Rear-Right  (CW)

    // Anti-saturation: find minimum and maximum motor demands
    float minimumMotor = motorRaw[0];
    float maximumMotor = motorRaw[0];
    for (int i = 1; i < 4; ++i) {
        if (motorRaw[i] < minimumMotor) minimumMotor = motorRaw[i];
        if (motorRaw[i] > maximumMotor) maximumMotor = motorRaw[i];
    }

    // Shift thrust if below zero to prevent motor cutoff during active attitude control
    if (minimumMotor < 0.0f) {
        float upwardOffset = -minimumMotor;
        for (int i = 0; i < 4; ++i) {
            motorRaw[i] += upwardOffset;
        }
        maximumMotor += upwardOffset;
    }

    // Scale down if exceeding maximum throttle while preserving control torque balance
    if (maximumMotor > 1.0f) {
        float scaleFactor = 1.0f / maximumMotor;
        for (int i = 0; i < 4; ++i) {
            motorRaw[i] *= scaleFactor;
        }
    }

    QuadMotorThrottleOutput output;
    output.motor1FrontRight = std::max(0.0f, std::min(1.0f, motorRaw[0]));
    output.motor2RearLeft   = std::max(0.0f, std::min(1.0f, motorRaw[1]));
    output.motor3FrontLeft  = std::max(0.0f, std::min(1.0f, motorRaw[2]));
    output.motor4RearRight  = std::max(0.0f, std::min(1.0f, motorRaw[3]));
    return output;
}

