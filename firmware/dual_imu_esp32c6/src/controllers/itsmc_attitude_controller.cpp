#include "itsmc_attitude_controller.hpp"
#include "itsmc_math.hpp"

ItsmcAttitudeController::ItsmcAttitudeController(const ItsmcParameters& parameters)
    : config_(parameters), isFirstIteration_(true) {
    reset();
}

void ItsmcAttitudeController::reset() {
    for (int i = 0; i < 3; ++i) {
        integratedTerminalError_[i] = 0.0f;
        previousTargetAngles_[i] = 0.0f;
    }
    isFirstIteration_ = true;
}

AttitudeControlTorque ItsmcAttitudeController::computeTorque(
    const float currentAngles[3],
    const float angularRates[3],
    const float targetAngles[3],
    float deltaTimeSec
) {
    if (deltaTimeSec <= 0.0001f) {
        deltaTimeSec = 0.005f; // Fallback to 200 Hz
    }

    AttitudeControlTorque outputTorques;
    float momentsOfInertia[3] = { config_.inertiaJx, config_.inertiaJy, config_.inertiaJz };

    for (int axis = 0; axis < 3; ++axis) {
        float angleError = currentAngles[axis] - targetAngles[axis];
        float targetRate = isFirstIteration_ ? 0.0f : (targetAngles[axis] - previousTargetAngles_[axis]) / deltaTimeSec;
        float rateError = angularRates[axis] - targetRate;

        // Terminal non-linear integral: |e|^alpha * sign(e) * dt
        float terminalTerm = ItsmcMath::terminalPower(angleError, config_.attitudeAlpha);
        integratedTerminalError_[axis] += terminalTerm * deltaTimeSec;
        integratedTerminalError_[axis] = ItsmcMath::clamp(
            integratedTerminalError_[axis], -config_.integralLimit, config_.integralLimit
        );

        // Sliding surface: s = de + Ke11 * e + Ke12 * int(|e|^alpha * sign(e))
        float slidingSurface = rateError + config_.attitudeKe11 * angleError +
                               config_.attitudeKe12 * integratedTerminalError_[axis];

        // Equivalent and switching control laws from itsmc.m
        float equivalentControl = -config_.attitudeKe11 * rateError - config_.attitudeKe12 * terminalTerm;
        float switchingControl = -config_.attitudeKe2 * slidingSurface -
                                 config_.attitudeKe3 * ItsmcMath::smoothSign(slidingSurface);

        float angularAcceleration = equivalentControl + switchingControl;
        float torque = momentsOfInertia[axis] * angularAcceleration;

        if (axis == 0) outputTorques.rollTorqueNm = ItsmcMath::clamp(torque, -1.5f, 1.5f);
        else if (axis == 1) outputTorques.pitchTorqueNm = ItsmcMath::clamp(torque, -1.5f, 1.5f);
        else outputTorques.yawTorqueNm = ItsmcMath::clamp(torque, -0.8f, 0.8f);

        previousTargetAngles_[axis] = targetAngles[axis];
    }

    isFirstIteration_ = false;
    return outputTorques;
}

