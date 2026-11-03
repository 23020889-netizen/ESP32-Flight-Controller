#include "itsmc_position_controller.hpp"
#include "itsmc_math.hpp"

ItsmcPositionController::ItsmcPositionController(const ItsmcParameters& parameters)
    : config_(parameters) {
    reset();
}

void ItsmcPositionController::reset() {
    for (int i = 0; i < 3; ++i) {
        integratedTerminalError_[i] = 0.0f;
    }
}

PositionControlCommand ItsmcPositionController::computeCommand(
    const float currentPosition[3],
    const float currentVelocity[3],
    const float targetPosition[3],
    const float targetVelocity[3],
    const float currentAttitude[3],
    float deltaTimeSec
) {
    if (deltaTimeSec <= 0.0001f) {
        deltaTimeSec = 0.005f;
    }

    float virtualAcceleration[3];
    for (int axis = 0; axis < 3; ++axis) {
        float positionError = currentPosition[axis] - targetPosition[axis];
        float velocityError = currentVelocity[axis] - targetVelocity[axis];

        float terminalTerm = ItsmcMath::terminalPower(positionError, config_.positionAlpha);
        integratedTerminalError_[axis] += terminalTerm * deltaTimeSec;
        integratedTerminalError_[axis] = ItsmcMath::clamp(
            integratedTerminalError_[axis], -config_.integralLimit, config_.integralLimit
        );

        float slidingSurface = velocityError + config_.positionKp11 * positionError +
                               config_.positionKp12 * integratedTerminalError_[axis];

        float equivalentControl = -config_.positionKp11 * velocityError - config_.positionKp12 * terminalTerm;
        float switchingControl = -config_.positionKp2 * slidingSurface -
                                 config_.positionKp3 * ItsmcMath::smoothSign(slidingSurface);

        float gravityComponent = (axis == 2) ? config_.gravityAccelerationMps2 : 0.0f;
        virtualAcceleration[axis] = (config_.vehicleMassKg / config_.thrustCoefficient) *
                                   (equivalentControl + switchingControl + gravityComponent);
    }

    float phi = currentAttitude[0];
    float theta = currentAttitude[1];
    float psi = currentAttitude[2];

    float cosPhi = std::max(0.3f, std::cos(phi));
    float cosTheta = std::max(0.3f, std::cos(theta));
    float cosPsi = std::cos(psi);
    float sinPsi = std::sin(psi);

    float rawUz = virtualAcceleration[2] / (cosPhi * cosTheta);
    float minimumThrust = config_.vehicleMassKg * config_.gravityAccelerationMps2 * 0.2f;
    float maximumThrust = config_.vehicleMassKg * config_.gravityAccelerationMps2 * 2.2f;
    float totalThrust = ItsmcMath::clamp(rawUz, minimumThrust, maximumThrust);

    float phiRef = ItsmcMath::safeArcSine(
        ((cosPhi * std::sin(theta) * sinPsi) - (virtualAcceleration[1] / totalThrust)) /
        std::max(0.2f, cosPsi)
    );
    float thetaRef = ItsmcMath::safeArcSine(
        ((virtualAcceleration[0] / totalThrust) - (std::sin(phi) * sinPsi)) /
        std::max(0.2f, cosPhi * cosPsi)
    );

    PositionControlCommand command;
    command.totalThrustNewtons = totalThrust;
    command.normalizedThrustRatio = ItsmcMath::clamp(
        totalThrust / maximumThrust, config_.minimumThrustRatio, config_.maximumThrustRatio
    );
    command.targetRollAngleRadians = ItsmcMath::clamp(
        phiRef, -config_.maximumTiltAngleRadians, config_.maximumTiltAngleRadians
    );
    command.targetPitchAngleRadians = ItsmcMath::clamp(
        thetaRef, -config_.maximumTiltAngleRadians, config_.maximumTiltAngleRadians
    );
    return command;
}

