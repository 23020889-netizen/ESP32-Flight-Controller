#pragma once

/**
 * @brief Configuration parameters for Quadcopter ITSMC flight control.
 */
struct ItsmcParameters {
    // Physical vehicle properties (matching itsmc.m benchmark)
    float vehicleMassKg = 1.7f;
    float gravityAccelerationMps2 = 9.8f;
    float armLengthMeters = 0.2f;
    float thrustCoefficient = 1.0f;
    float torqueCoefficient = 1.0f;

    // Moments of inertia (kg * m^2)
    float inertiaJx = 4.8e-3f;
    float inertiaJy = 4.8e-3f;
    float inertiaJz = 9.8e-3f;

    // Position / Altitude loop ITSMC parameters
    float positionAlpha = 0.5f;
    float positionKp11 = 1.7f;
    float positionKp12 = 0.5f;
    float positionKp2 = 2.0f;
    float positionKp3 = 0.2f;

    // Attitude loop ITSMC parameters
    float attitudeAlpha = 0.5f;
    float attitudeKe11 = 3.0f;
    float attitudeKe12 = 0.5f;
    float attitudeKe2 = 5.0f;
    float attitudeKe3 = 0.2f;

    // Safety and physical limits
    float maximumTiltAngleRadians = 0.4363f; // ~25 degrees
    float minimumThrustRatio = 0.05f;
    float maximumThrustRatio = 0.90f;
    float integralLimit = 2.0f;
};

