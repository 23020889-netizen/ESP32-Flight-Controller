#pragma once

#include <stdint.h>
#include "../estimation/px4_ekf.h"
#include "../controllers/itsmc_parameters.hpp"
#include "../controllers/itsmc_attitude_controller.hpp"
#include "../controllers/itsmc_position_controller.hpp"
#include "../actuators/quadrotor_motor_mixer.hpp"

enum class FlightControlMode {
    MANUAL_CLI,
    ITSMC_ATTITUDE_HOLD,
    ITSMC_POSITION_HOLD,
    EMERGENCY_DISARM
};

/**
 * @brief High-level flight management coordinator integrating estimation, ITSMC, and ESCs.
 */
class FlightControllerManager {
public:
    FlightControllerManager();

    void initialize();
    void setFlightMode(FlightControlMode newMode);
    FlightControlMode getFlightMode() const { return currentMode_; }

    void setTargetAttitude(float rollDeg, float pitchDeg, float yawDeg);
    void setTargetPosition(float xMeters, float yMeters, float altitudeMeters);
    void setManualThrottle(float throttleRatio);

    void update(const FusedData& estimatedState, float deltaTimeSec);

    const QuadMotorThrottleOutput& getMotorOutputs() const { return motorOutputs_; }
    const PositionControlCommand& getLastPositionCommand() const { return lastPositionCommand_; }
    const AttitudeControlTorque& getLastAttitudeTorque() const { return lastAttitudeTorque_; }

private:
    ItsmcParameters parameters_;
    ItsmcAttitudeController attitudeController_;
    ItsmcPositionController positionController_;
    QuadrotorMotorMixer motorMixer_;

    FlightControlMode currentMode_;
    float targetAttitudeDegrees_[3];
    float targetPositionMeters_[3];
    float manualThrottleRatio_;

    QuadMotorThrottleOutput motorOutputs_;
    PositionControlCommand lastPositionCommand_;
    AttitudeControlTorque lastAttitudeTorque_;
};

extern FlightControllerManager flightManager;

