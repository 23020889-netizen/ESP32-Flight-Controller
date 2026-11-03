#pragma once

/**
 * @brief Benchmark tuned parameters for x500 Quadrotor ITSMC flight control.
 * Incorporates non-singular linear transitions and boundary layer anti-chattering.
 */
struct ItsmcParameters {
    // Physical vehicle properties
    float vehicleMassKg = 1.7f;
    float gravityAccelerationMps2 = 9.8f;
    float armLengthMeters = 0.2f;

    // Roll and Pitch attitude loop parameters (Damping=0.080, Stiffness=0.384)
    float attitudeK11 = 12.0f;
    float attitudeK12 = 0.30f;
    float attitudeK2 = 8.0f;
    float attitudeK3 = 0.15f;
    float attitudeBinv = 0.0040f;
    float attitudeBoundaryPhiB = 0.20f;
    float attitudeLinearEpsilon = 0.035f; // ~2.0 deg

    // Yaw rotation attitude loop parameters
    float yawK11 = 3.0f;
    float yawK12 = 0.30f;
    float yawK2 = 2.0f;
    float yawK3 = 0.10f;
    float yawBinv = 0.030f;
    float yawBoundaryPhiB = 0.15f;
    float yawLinearEpsilon = 0.050f;

    // Altitude hold loop parameters
    float altitudeK11 = 2.0f;
    float altitudeK12 = 0.40f;
    float altitudeK2 = 2.0f;
    float altitudeK3 = 0.40f;
    float altitudeBinv = 0.080f;
    float altitudeBoundaryPhiB = 0.20f;
    float altitudeLinearEpsilon = 0.050f;
    float hoverThrustRatio = 0.59f;

    // Safety and physical limits
    float maximumTiltAngleRadians = 0.4363f; // ~25 deg
    float maximumNormalizedTorque = 0.25f;   // ESC headroom
    float integralLimit = 1.5f;
};
