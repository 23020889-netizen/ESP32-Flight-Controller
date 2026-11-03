#include "flight_controller_manager.hpp"
#include "../actuators/esc_pwm.h"
#include <cmath>

FlightControllerManager flightManager;

FlightControllerManager::FlightControllerManager()
    : attitudeController_(parameters_),
      positionController_(parameters_),
      motorMixer_(parameters_.armLengthMeters),
      currentMode_(FlightControlMode::MANUAL_CLI),
      manualThrottleRatio_(0.0f) {
    targetAttitudeDegrees_[0] = 0.0f;
    targetAttitudeDegrees_[1] = 0.0f;
    targetAttitudeDegrees_[2] = 0.0f;
    targetPositionMeters_[0] = 0.0f;
    targetPositionMeters_[1] = 0.0f;
    targetPositionMeters_[2] = 1.0f;
}

void FlightControllerManager::initialize() {
    attitudeController_.reset();
    positionController_.reset();
}

void FlightControllerManager::setFlightMode(FlightControlMode newMode) {
    currentMode_ = newMode;
    if (newMode == FlightControlMode::EMERGENCY_DISARM) {
        quadMotors.disarm();
    } else if (newMode == FlightControlMode::ITSMC_ATTITUDE_HOLD ||
               newMode == FlightControlMode::ITSMC_POSITION_HOLD) {
        attitudeController_.reset();
        positionController_.reset();
        if (quadMotors.getState() != ESC_STATE_ARMED) {
            quadMotors.arm();
        }
    }
}

void FlightControllerManager::setTargetAttitude(float rollDeg, float pitchDeg, float yawDeg) {
    targetAttitudeDegrees_[0] = rollDeg;
    targetAttitudeDegrees_[1] = pitchDeg;
    targetAttitudeDegrees_[2] = yawDeg;
}

void FlightControllerManager::setTargetPosition(float xMeters, float yMeters, float altitudeMeters) {
    targetPositionMeters_[0] = xMeters;
    targetPositionMeters_[1] = yMeters;
    targetPositionMeters_[2] = altitudeMeters;
}

void FlightControllerManager::setManualThrottle(float throttleRatio) {
    manualThrottleRatio_ = (throttleRatio < 0.0f) ? 0.0f : ((throttleRatio > 1.0f) ? 1.0f : throttleRatio);
}

void FlightControllerManager::update(const FusedData& state, float deltaTimeSec) {
    if (currentMode_ == FlightControlMode::MANUAL_CLI || currentMode_ == FlightControlMode::EMERGENCY_DISARM) {
        return;
    }

    constexpr float degreesToRadians = 0.0174532925f;
    float currentAnglesRad[3] = { state.roll * degreesToRadians, state.pitch * degreesToRadians, state.yaw * degreesToRadians };
    float angularRatesRad[3] = { state.gx * degreesToRadians, state.gy * degreesToRadians, state.gz * degreesToRadians };

    float targetAnglesRad[3];
    float baseThrustRatio = manualThrottleRatio_;

    if (currentMode_ == FlightControlMode::ITSMC_POSITION_HOLD) {
        float currentPos[3] = { 0.0f, 0.0f, 0.0f };
        float currentVel[3] = { 0.0f, 0.0f, 0.0f };
        float targetVel[3]  = { 0.0f, 0.0f, 0.0f };

        lastPositionCommand_ = positionController_.computeCommand(
            currentPos, currentVel, targetPositionMeters_, targetVel, currentAnglesRad, deltaTimeSec
        );
        targetAnglesRad[0] = lastPositionCommand_.targetRollAngleRadians;
        targetAnglesRad[1] = lastPositionCommand_.targetPitchAngleRadians;
        targetAnglesRad[2] = targetAttitudeDegrees_[2] * degreesToRadians;
        baseThrustRatio = lastPositionCommand_.normalizedThrustRatio;
    } else {
        targetAnglesRad[0] = targetAttitudeDegrees_[0] * degreesToRadians;
        targetAnglesRad[1] = targetAttitudeDegrees_[1] * degreesToRadians;
        targetAnglesRad[2] = targetAttitudeDegrees_[2] * degreesToRadians;
    }

    lastAttitudeTorque_ = attitudeController_.computeTorque(
        currentAnglesRad, angularRatesRad, targetAnglesRad, deltaTimeSec
    );

    motorOutputs_ = motorMixer_.computeMixer(
        baseThrustRatio, lastAttitudeTorque_.rollTorqueNm, lastAttitudeTorque_.pitchTorqueNm, lastAttitudeTorque_.yawTorqueNm
    );

    quadMotors.setMotorThrottlePct(1, motorOutputs_.motor1FrontRight * 100.0f);
    quadMotors.setMotorThrottlePct(2, motorOutputs_.motor2RearLeft * 100.0f);
    quadMotors.setMotorThrottlePct(3, motorOutputs_.motor3FrontLeft * 100.0f);
    quadMotors.setMotorThrottlePct(4, motorOutputs_.motor4RearRight * 100.0f);
}

