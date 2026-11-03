#pragma once

#include "itsmc_parameters.hpp"

/**
 * @brief Output command from ITSMC position and altitude controller.
 */
struct PositionControlCommand {
    float totalThrustNewtons = 0.0f;
    float normalizedThrustRatio = 0.0f;
    float targetRollAngleRadians = 0.0f;
    float targetPitchAngleRadians = 0.0f;
};

/**
 * @brief Integral Terminal Sliding Mode Controller for Quadcopter Position and Altitude Loop.
 */
class ItsmcPositionController {
public:
    /**
     * @brief Constructs position controller with parameter reference.
     * @param parameters Reference to vehicle configuration parameters.
     */
    explicit ItsmcPositionController(const ItsmcParameters& parameters);

    /**
     * @brief Resets internal sliding surfaces and terminal integrators.
     */
    void reset();

    /**
     * @brief Computes desired thrust and attitude reference from position error.
     * @param currentPosition Array of [x, y, z] in meters.
     * @param currentVelocity Array of [vx, vy, vz] in m/s.
     * @param targetPosition Array of desired [x, y, z] in meters.
     * @param targetVelocity Array of desired [vx, vy, vz] in m/s.
     * @param currentAttitude Array of [roll, pitch, yaw] in radians.
     * @param deltaTimeSec Loop period in seconds.
     * @return PositionControlCommand struct with total thrust and target tilt angles.
     */
    PositionControlCommand computeCommand(
        const float currentPosition[3],
        const float currentVelocity[3],
        const float targetPosition[3],
        const float targetVelocity[3],
        const float currentAttitude[3],
        float deltaTimeSec
    );

private:
    const ItsmcParameters& config_;
    float integratedTerminalError_[3];
};

