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
    for (int axis = 0; axis < 3; ++axis) {
        bool isYaw = (axis == 2);
        float k11    = isYaw ? config_.yawK11 : config_.attitudeK11;
        float k12    = isYaw ? config_.yawK12 : config_.attitudeK12;
        float k2     = isYaw ? config_.yawK2  : config_.attitudeK2;
        float k3     = isYaw ? config_.yawK3  : config_.attitudeK3;
        float bInv   = isYaw ? config_.yawBinv : config_.attitudeBinv;
        float phiB   = isYaw ? config_.yawBoundaryPhiB : config_.attitudeBoundaryPhiB;
        float epsLin = isYaw ? config_.yawLinearEpsilon : config_.attitudeLinearEpsilon;

        float angleError = currentAngles[axis] - targetAngles[axis];
        float targetRate = isFirstIteration_ ? 0.0f : (targetAngles[axis] - previousTargetAngles_[axis]) / deltaTimeSec;
        float rateError  = angularRates[axis] - targetRate;

        // 1. Non-singular linear terminal term (eliminates 4Hz Limit Cycle near zero)
        float terminalTerm = ItsmcMath::terminalPowerNonSingular(angleError, epsLin);
        integratedTerminalError_[axis] += terminalTerm * deltaTimeSec;
        integratedTerminalError_[axis] = ItsmcMath::clamp(
            integratedTerminalError_[axis], -config_.integralLimit, config_.integralLimit
        );

        // 2. Sliding surface: s = de + k11 * e + k12 * int(terminal)
        float slidingSurface = rateError + k11 * angleError + k12 * integratedTerminalError_[axis];

        // 3. Equivalent & boundary-layer switching control (eliminates 12Hz Chattering)
        float equivalentControl = -k11 * rateError - k12 * terminalTerm;
        float switchingControl  = -k2 * slidingSurface - k3 * ItsmcMath::smoothSign(slidingSurface, phiB);

        // 4. Scaled control torque: u = B_inv * (u_eq + u_sw)
        float normalizedTorque = bInv * (equivalentControl + switchingControl);
        normalizedTorque = ItsmcMath::clamp(
            normalizedTorque, -config_.maximumNormalizedTorque, config_.maximumNormalizedTorque
        );

        if (axis == 0) outputTorques.rollTorqueNm = normalizedTorque;
        else if (axis == 1) outputTorques.pitchTorqueNm = normalizedTorque;
        else outputTorques.yawTorqueNm = normalizedTorque;

        previousTargetAngles_[axis] = targetAngles[axis];
    }

    isFirstIteration_ = false;
    return outputTorques;
}
