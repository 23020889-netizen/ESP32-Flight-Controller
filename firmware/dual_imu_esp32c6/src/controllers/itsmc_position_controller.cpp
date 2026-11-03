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
    if (deltaTimeSec <= 0.0001f) deltaTimeSec = 0.005f;

    // Altitude (Z-axis) ITSMC control
    float altitudeError = currentPosition[2] - targetPosition[2];
    float vzError = currentVelocity[2] - targetVelocity[2];

    float zTerminal = ItsmcMath::terminalPowerNonSingular(altitudeError, config_.altitudeLinearEpsilon);
    integratedTerminalError_[2] += zTerminal * deltaTimeSec;
    integratedTerminalError_[2] = ItsmcMath::clamp(
        integratedTerminalError_[2], -config_.integralLimit, config_.integralLimit
    );

    float zSlidingSurface = vzError + config_.altitudeK11 * altitudeError +
                            config_.altitudeK12 * integratedTerminalError_[2];

    float zEquivalent = -config_.altitudeK11 * vzError - config_.altitudeK12 * zTerminal;
    float zSwitching  = -config_.altitudeK2 * zSlidingSurface -
                        config_.altitudeK3 * ItsmcMath::smoothSign(zSlidingSurface, config_.altitudeBoundaryPhiB);

    float deltaThrust = config_.altitudeBinv * (zEquivalent + zSwitching);
    float cosPhi = std::max(0.65f, std::cos(currentAttitude[0]));
    float cosTheta = std::max(0.65f, std::cos(currentAttitude[1]));

    float totalThrustRatio = (config_.hoverThrustRatio - deltaThrust) / (cosPhi * cosTheta);
    totalThrustRatio = ItsmcMath::clamp(totalThrustRatio, 0.10f, 0.85f);

    PositionControlCommand command;
    command.normalizedThrustRatio = totalThrustRatio;
    command.totalThrustNewtons = totalThrustRatio * (config_.vehicleMassKg * config_.gravityAccelerationMps2 / config_.hoverThrustRatio);
    command.targetRollAngleRadians = 0.0f;
    command.targetPitchAngleRadians = 0.0f;
    return command;
}
