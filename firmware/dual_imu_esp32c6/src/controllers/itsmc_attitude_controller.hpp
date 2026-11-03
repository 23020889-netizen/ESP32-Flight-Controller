#pragma once

#include "itsmc_parameters.hpp"

/**
 * @brief Output command from ITSMC attitude controller.
 */
struct AttitudeControlTorque {
    float rollTorqueNm = 0.0f;
    float pitchTorqueNm = 0.0f;
    float yawTorqueNm = 0.0f;
};

/**
 * @brief Integral Terminal Sliding Mode Controller for Quadcopter Attitude Loop.
 */
class ItsmcAttitudeController {
public:
    /**
     * @brief Constructs attitude controller with parameter reference.
     * @param parameters Reference to vehicle configuration parameters.
     */
    explicit ItsmcAttitudeController(const ItsmcParameters& parameters);

    /**
     * @brief Resets internal sliding surfaces and terminal integrators.
     */
    void reset();

    /**
     * @brief Computes 3-axis control torques via terminal sliding surfaces.
     * @param currentAngles Array of [roll, pitch, yaw] in radians.
     * @param angularRates Array of [p, q, r] gyro rates in rad/s.
     * @param targetAngles Array of desired [roll, pitch, yaw] in radians.
     * @param deltaTimeSec Loop period in seconds.
     * @return AttitudeControlTorque struct with 3-axis torques.
     */
    AttitudeControlTorque computeTorque(
        const float currentAngles[3],
        const float angularRates[3],
        const float targetAngles[3],
        float deltaTimeSec
    );

private:
    const ItsmcParameters& config_;
    float integratedTerminalError_[3];
    float previousTargetAngles_[3];
    bool isFirstIteration_;
};

