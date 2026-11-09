#include "flight_controller_manager.hpp"
#include "../actuators/esc_pwm.h"
#include <cmath>
#include <algorithm>

FlightControllerManager flightManager;

FlightControllerManager::FlightControllerManager()
    : attitudeController_(parameters_),
      positionController_(parameters_),
      motorMixer_(parameters_.armLengthMeters),
      currentMode_(FlightControlMode::DIRECT_MANUAL),
      currentFailsafe_(FailsafeLevel::NONE),
      altitudeSource_(AltitudeSensorSource::INERTIAL_ACCEL_ESTIMATED_BENCH_TEST),
      manualThrottleRatio_(0.0f),
      hoverThrustRatio_(0.45f), // Mac dinh 45% (Hieu chinh rieng theo trong luong va dong co thuc te)
      rcSnapshot_({0}),
      rcMux_(portMUX_INITIALIZER_UNLOCKED),
      rcThrottle_(0.0f),
      rcRoll_(0.0f),
      rcPitch_(0.0f),
      rcYaw_(0.0f),
      lastRcPacketMs_(0),
      targetAltitudeMeters_(1.0f),
      targetYawDegrees_(0.0f),
      vzIntegral_(0.0f),
      prevVzError_(0.0f),
      rollRateIntegral_(0.0f),
      pitchRateIntegral_(0.0f),
      yawRateIntegral_(0.0f),
      prevRollRateError_(0.0f),
      prevPitchRateError_(0.0f),
      prevYawRateError_(0.0f),
      prevFilteredDRoll_(0.0f),
      prevFilteredDPitch_(0.0f) {
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
    vzIntegral_ = 0.0f;
    prevVzError_ = 0.0f;
    rollRateIntegral_ = 0.0f;
    pitchRateIntegral_ = 0.0f;
    yawRateIntegral_ = 0.0f;
    prevRollRateError_ = 0.0f;
    prevPitchRateError_ = 0.0f;
    prevYawRateError_ = 0.0f;
    prevFilteredDRoll_ = 0.0f;
    prevFilteredDPitch_ = 0.0f;
    targetAltitudeMeters_ = 1.0f;
    currentFailsafe_ = FailsafeLevel::NONE;
}

void FlightControllerManager::setFlightMode(FlightControlMode newMode) {
    currentMode_ = newMode;
    if (newMode == FlightControlMode::EMERGENCY_DISARM) {
        quadMotors.disarm();
    } else if (newMode == FlightControlMode::DIRECT_MANUAL) {
        rcThrottle_ = 0.0f;
        rcRoll_     = 0.0f;
        rcPitch_    = 0.0f;
        rcYaw_      = 0.0f;
    } else if (newMode == FlightControlMode::ALTITUDE_HOLD) {
        vzIntegral_ = 0.0f;
        prevVzError_ = 0.0f;
        rollRateIntegral_ = 0.0f;
        pitchRateIntegral_ = 0.0f;
        yawRateIntegral_ = 0.0f;
        prevRollRateError_ = 0.0f;
        prevPitchRateError_ = 0.0f;
        prevYawRateError_ = 0.0f;
        prevFilteredDRoll_ = 0.0f;
        prevFilteredDPitch_ = 0.0f;
        targetAltitudeMeters_ = 1.0f;
    } else if (newMode == FlightControlMode::ITSMC_POSITION_HOLD) {
        attitudeController_.reset();
        positionController_.reset();
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

void FlightControllerManager::updateRcSnapshot(const RcSnapshot& snapshot) {
    portENTER_CRITICAL(&rcMux_);
    rcSnapshot_ = snapshot;
    portEXIT_CRITICAL(&rcMux_);
}

bool FlightControllerManager::getRcSnapshotAtomic(RcSnapshot& out) const {
    portENTER_CRITICAL(&rcMux_);
    out = rcSnapshot_;
    portEXIT_CRITICAL(&rcMux_);
    return out.valid;
}

void FlightControllerManager::setRcChannels(float throttlePct, float rollPct, float pitchPct, float yawPct) {
    RcSnapshot snap{};
    snap.sessionId = 0;
    snap.seq = 0;
    snap.receivedAtMs = millis();
    snap.throttle = (throttlePct < 0.0f) ? 0.0f : ((throttlePct > 100.0f) ? 100.0f : throttlePct);
    snap.roll     = (rollPct < -35.0f) ? -35.0f : ((rollPct > 35.0f) ? 35.0f : rollPct);
    snap.pitch    = (pitchPct < -35.0f) ? -35.0f : ((pitchPct > 35.0f) ? 35.0f : pitchPct);
    snap.yaw      = (yawPct < -35.0f) ? -35.0f : ((yawPct > 35.0f) ? 35.0f : yawPct);
    snap.source   = RC_SOURCE_SERIAL;
    snap.valid    = true;
    updateRcSnapshot(snap);
}

void FlightControllerManager::getRcChannels(float& throttlePct, float& rollPct, float& pitchPct, float& yawPct) const {
    RcSnapshot snap{};
    if (getRcSnapshotAtomic(snap)) {
        throttlePct = snap.throttle;
        rollPct     = snap.roll;
        pitchPct    = snap.pitch;
        yawPct      = snap.yaw;
    } else {
        throttlePct = rcThrottle_;
        rollPct     = rcRoll_;
        pitchPct    = rcPitch_;
        yawPct      = rcYaw_;
    }
}

void FlightControllerManager::updateDirectManual() {
    if (!quadMotors.isArmed() || currentMode_ == FlightControlMode::EMERGENCY_DISARM) {
        return;
    }

    float t = rcThrottle_; // 0..100%
    float r = rcRoll_;     // -100..+100%
    float p = rcPitch_;    // -100..+100%
    float y = rcYaw_;      // -100..+100%

    // Base pulse: 1050 us (Idle) -> 2000 us (Full ga) theo truc can ga trai
    float basePulse = 1050.0f + (t * 9.50f);

    // Quad-X Mixer dap ung tuc thi 0ms theo tay cam (khong can bang PID)
    // Pitch toi (+p): 2 motor truoc (M1, M3) giam, 2 motor sau (M2, M4) tang
    // Roll phai (+r): 2 motor trai (M3, M2) tang, 2 motor phai (M1, M4) giam
    // Yaw phai (+y): 2 motor CCW (M1, M2) tang, 2 motor CW (M3, M4) giam
    float dRoll  = r * 1.8f;
    float dPitch = p * 1.8f;
    float dYaw   = y * 1.2f;

    float pwm1 = basePulse - dRoll - dPitch + dYaw; // M1 Front Right (CCW)
    float pwm2 = basePulse + dRoll + dPitch + dYaw; // M2 Rear Left   (CCW)
    float pwm3 = basePulse + dRoll - dPitch - dYaw; // M3 Front Left  (CW)
    float pwm4 = basePulse - dRoll + dPitch - dYaw; // M4 Rear Right  (CW)

    uint16_t m1 = (uint16_t)std::max(1050.0f, std::min(2000.0f, pwm1));
    uint16_t m2 = (uint16_t)std::max(1050.0f, std::min(2000.0f, pwm2));
    uint16_t m3 = (uint16_t)std::max(1050.0f, std::min(2000.0f, pwm3));
    uint16_t m4 = (uint16_t)std::max(1050.0f, std::min(2000.0f, pwm4));

    quadMotors.setMotorThrottleUs(1, m1);
    quadMotors.setMotorThrottleUs(2, m2);
    quadMotors.setMotorThrottleUs(3, m3);
    quadMotors.setMotorThrottleUs(4, m4);

    motorOutputs_.motor1FrontRight = (float)(m1 - 1050) / 950.0f;
    motorOutputs_.motor2RearLeft   = (float)(m2 - 1050) / 950.0f;
    motorOutputs_.motor3FrontLeft  = (float)(m3 - 1050) / 950.0f;
    motorOutputs_.motor4RearRight  = (float)(m4 - 1050) / 950.0f;

    // Feed ESC output watchdog sau khi da tinh toan xung hop le
    quadMotors.feedWatchdog();
}

void FlightControllerManager::updateAltitudeHold(const FusedData& state, float dt) {
    if (!quadMotors.isArmed() || currentMode_ == FlightControlMode::EMERGENCY_DISARM) {
        return;
    }

    // A. XỬ LÝ CẦN GA TRÁI & FAILSAFE
    if (rcThrottle_ < 4.0f && state.altitudeZ < 0.20f) {
        // Chi ngat ve 1050us neu cac can lai Roll/Pitch/Yaw deu o giua (trung tinh)
        if (fabsf(rcRoll_) < 2.0f && fabsf(rcPitch_) < 2.0f && fabsf(rcYaw_) < 2.0f) {
            quadMotors.setMotorThrottleUs(1, 1050);
            quadMotors.setMotorThrottleUs(2, 1050);
            quadMotors.setMotorThrottleUs(3, 1050);
            quadMotors.setMotorThrottleUs(4, 1050);
            vzIntegral_ = 0.0f;
            prevVzError_ = 0.0f;
            rollRateIntegral_ = 0.0f;
            pitchRateIntegral_ = 0.0f;
            yawRateIntegral_ = 0.0f;
            motorOutputs_.motor1FrontRight = 0.05f;
            motorOutputs_.motor2RearLeft   = 0.05f;
            motorOutputs_.motor3FrontLeft  = 0.05f;
            motorOutputs_.motor4RearRight  = 0.05f;
            return;
        }
    }

    float vzTarget = 0.0f;
    if (currentFailsafe_ == FailsafeLevel::LEVEL2_AUTO_DESCENT) {
        // Failsafe Level 2: Tu dong ha canh tu tu -0.6 m/s
        vzTarget = -0.6f;
        targetAltitudeMeters_ += vzTarget * dt;
    } else {
        // Vùng Deadband ở giữa (45% .. 55%): Khóa và giữ nguyên độ cao mục tiêu (ALT HOLD LOCK)
        if (rcThrottle_ >= 45.0f && rcThrottle_ <= 55.0f) {
            vzTarget = 0.0f;
        } else if (rcThrottle_ > 55.0f) {
            vzTarget = ((rcThrottle_ - 55.0f) / 45.0f) * 2.0f; // Tối đa +2.0 m/s
            targetAltitudeMeters_ += vzTarget * dt;
        } else if (rcThrottle_ < 45.0f) {
            vzTarget = ((rcThrottle_ - 45.0f) / 45.0f) * 1.5f; // Tối đa -1.5 m/s
            targetAltitudeMeters_ += vzTarget * dt;
        }
    }

    if (targetAltitudeMeters_ < 0.0f)  targetAltitudeMeters_ = 0.0f;
    if (targetAltitudeMeters_ > 15.0f) targetAltitudeMeters_ = 15.0f;

    // B. BỘ ĐIỀU KHIỂN ĐỘ CAO PID TẦNG (CASCADED ALTITUDE & VERTICAL VELOCITY PID)
    float ez = targetAltitudeMeters_ - state.altitudeZ;
    float vzCmd = 1.2f * ez + vzTarget;
    vzCmd = std::max(-1.5f, std::min(1.5f, vzCmd));

    float evz = vzCmd - state.velocityVz;
    vzIntegral_ += 0.05f * evz * dt;
    vzIntegral_ = std::max(-0.15f, std::min(0.15f, vzIntegral_));

    float dVz = 0.02f * (evz - prevVzError_) / dt;
    prevVzError_ = evz;

    // Truc tiep phan hoi xung ga khi nguoi dung gat can ga (ngay ca khi test tren ban do cao Z chua kip doi)
    float thrFeedforward = (rcThrottle_ - 50.0f) * 0.007f;
    float tBase = hoverThrustRatio_ + 0.20f * evz + vzIntegral_ + dVz + thrFeedforward;

    constexpr float degToRad = 0.0174532925f;
    float cosRoll  = cosf(state.roll * degToRad);
    float cosPitch = cosf(state.pitch * degToRad);
    float tiltComp = std::max(0.65f, cosRoll * cosPitch);
    float tTotal   = std::max(0.12f, std::min(0.85f, tBase / tiltComp));

    // C. BỘ ĐIỀU KHIỂN CÂN BẰNG GÓC PID TẦNG (ATTITUDE ANGLE -> RATE PID)
    float targetRoll  = rcRoll_ * 0.70f;
    float targetPitch = rcPitch_ * 0.70f;

    float yawRateCmd = rcYaw_ * 2.5f;
    targetYawDegrees_ += yawRateCmd * dt;
    while (targetYawDegrees_ > 180.0f)  targetYawDegrees_ -= 360.0f;
    while (targetYawDegrees_ < -180.0f) targetYawDegrees_ += 360.0f;

    // 1. Vòng ngoài (Attitude Angle Controller -> Desired Angular Rates)
    float errRoll  = targetRoll - state.roll;
    float errPitch = targetPitch - state.pitch;
    float errYaw   = targetYawDegrees_ - state.yaw;
    while (errYaw > 180.0f)  errYaw -= 360.0f;
    while (errYaw < -180.0f) errYaw += 360.0f;

    // Gain vòng ngoài (Angle error -> Desired rate): Tăng độ nhạy cân bằng
    float spRollRate  = std::max(-180.0f, std::min(180.0f, 6.0f * errRoll));
    float spPitchRate = std::max(-180.0f, std::min(180.0f, 6.0f * errPitch));
    float spYawRate   = std::max(-90.0f,  std::min(90.0f,  2.5f * errYaw));

    // 2. Vòng trong (Angular Rate PID Controller với D-term Filter & Anti-windup): Tăng lực bù mô-men
    float eRateR = spRollRate - state.gx;
    rollRateIntegral_ += 0.05f * eRateR * dt;
    rollRateIntegral_ = std::max(-0.15f, std::min(0.15f, rollRateIntegral_));

    // Lọc thông thấp bậc 1 cho D-term để chống khuếch đại nhiễu Gyro
    float rawDRoll = 0.005f * (eRateR - prevRollRateError_) / dt;
    float filteredDRoll = 0.70f * prevFilteredDRoll_ + 0.30f * rawDRoll;
    prevFilteredDRoll_ = filteredDRoll;
    prevRollRateError_ = eRateR;
    float tauRoll = 0.0065f * eRateR + rollRateIntegral_ + filteredDRoll;

    float eRateP = spPitchRate - state.gy;
    pitchRateIntegral_ += 0.05f * eRateP * dt;
    pitchRateIntegral_ = std::max(-0.15f, std::min(0.15f, pitchRateIntegral_));

    float rawDPitch = 0.005f * (eRateP - prevPitchRateError_) / dt;
    float filteredDPitch = 0.70f * prevFilteredDPitch_ + 0.30f * rawDPitch;
    prevFilteredDPitch_ = filteredDPitch;
    prevPitchRateError_ = eRateP;
    float tauPitch = 0.0065f * eRateP + pitchRateIntegral_ + filteredDPitch;

    float eRateY = spYawRate - state.gz;
    yawRateIntegral_ += 0.02f * eRateY * dt;
    yawRateIntegral_ = std::max(-0.08f, std::min(0.08f, yawRateIntegral_));
    float tauYaw = 0.0050f * eRateY + yawRateIntegral_;

    // D. HÒA TRỘN QUAD-X VÀ XUẤT XUNG PWM RA 4 ĐỘNG CƠ
    motorOutputs_ = motorMixer_.computeMixer(tTotal, tauRoll, tauPitch, tauYaw);

    uint16_t pwm1 = static_cast<uint16_t>(1050.0f + motorOutputs_.motor1FrontRight * 950.0f);
    uint16_t pwm2 = static_cast<uint16_t>(1050.0f + motorOutputs_.motor2RearLeft   * 950.0f);
    uint16_t pwm3 = static_cast<uint16_t>(1050.0f + motorOutputs_.motor3FrontLeft  * 950.0f);
    uint16_t pwm4 = static_cast<uint16_t>(1050.0f + motorOutputs_.motor4RearRight  * 950.0f);

    pwm1 = std::max((uint16_t)1050, std::min((uint16_t)2000, pwm1));
    pwm2 = std::max((uint16_t)1050, std::min((uint16_t)2000, pwm2));
    pwm3 = std::max((uint16_t)1050, std::min((uint16_t)2000, pwm3));
    pwm4 = std::max((uint16_t)1050, std::min((uint16_t)2000, pwm4));

    quadMotors.setMotorThrottleUs(1, pwm1);
    quadMotors.setMotorThrottleUs(2, pwm2);
    quadMotors.setMotorThrottleUs(3, pwm3);
    quadMotors.setMotorThrottleUs(4, pwm4);

    // Feed ESC output watchdog sau khi da tinh toan xung hop le
    quadMotors.feedWatchdog();
}

void FlightControllerManager::update(const FusedData& state, float deltaTimeSec) {
    if (currentMode_ == FlightControlMode::EMERGENCY_DISARM || quadMotors.isMotorTestActive()) {
        return;
    }

    // Đọc trạng thái Atomic RC Snapshot từ CommTask
    RcSnapshot snap{};
    bool hasRc = getRcSnapshotAtomic(snap);
    uint32_t nowMs = millis();
    uint32_t ageMs = (hasRc && nowMs >= snap.receivedAtMs) ? (nowMs - snap.receivedAtMs) : 999999;

    // RC TIMEOUT FAILSAFE STATE MACHINE (TIERED FAILSAFE CHO DÒNG LỆNH 50 HZ)
    if (quadMotors.isArmed()) {
        if (!hasRc || ageMs > 1500) {
            // Level 3: Mất tín hiệu RC quá 1500ms (~75 chu kỳ gói) -> Ngắt động cơ khẩn cấp
            currentFailsafe_ = FailsafeLevel::LEVEL3_MOTOR_CUT;
            setFlightMode(FlightControlMode::EMERGENCY_DISARM);
            quadMotors.disarm();
            Serial.printf("[FAILSAFE] >>> LEVEL 3 TRIGGERED: Mat tin hieu RC (age=%u ms)! Disarm khan cap.\n", ageMs);
            return;
        } else if (ageMs > 600) {
            // Level 2: Mất tín hiệu quá 600ms (~30 chu kỳ gói) -> Tự động giảm độ cao hạ cánh
            currentFailsafe_ = FailsafeLevel::LEVEL2_AUTO_DESCENT;
            rcRoll_  = 0.0f;
            rcPitch_ = 0.0f;
            rcYaw_   = 0.0f;
        } else if (ageMs > 200) {
            // Level 1: Mất tín hiệu quá 200ms (~10 chu kỳ gói) -> Thăng bằng góc, giữ nguyên độ cao
            currentFailsafe_ = FailsafeLevel::LEVEL1_ATTITUDE_LEVEL;
            rcRoll_  = 0.0f;
            rcPitch_ = 0.0f;
            rcYaw_   = 0.0f;
        } else {
            // Tín hiệu RC Fresh và Hợp lệ (< 200ms)
            currentFailsafe_ = FailsafeLevel::NONE;
            rcThrottle_ = snap.throttle;
            rcRoll_     = snap.roll;
            rcPitch_    = snap.pitch;
            rcYaw_      = snap.yaw;
            lastRcPacketMs_ = snap.receivedAtMs;
        }
    }

    if (currentMode_ == FlightControlMode::DIRECT_MANUAL) {
        updateDirectManual();
        return;
    }

    if (currentMode_ == FlightControlMode::ALTITUDE_HOLD) {
        updateAltitudeHold(state, deltaTimeSec);
        return;
    }

    if (currentMode_ == FlightControlMode::ITSMC_POSITION_HOLD) {
        constexpr float degreesToRadians = 0.0174532925f;
        float currentAnglesRad[3] = { state.roll * degreesToRadians, state.pitch * degreesToRadians, state.yaw * degreesToRadians };
        float angularRatesRad[3] = { state.gx * degreesToRadians, state.gy * degreesToRadians, state.gz * degreesToRadians };

        float currentPos[3] = { 0.0f, 0.0f, state.altitudeZ };
        float currentVel[3] = { 0.0f, 0.0f, state.velocityVz };
        float targetVel[3]  = { 0.0f, 0.0f, 0.0f };

        lastPositionCommand_ = positionController_.computeCommand(
            currentPos, currentVel, targetPositionMeters_, targetVel, currentAnglesRad, deltaTimeSec
        );

        float targetAnglesRad[3] = {
            lastPositionCommand_.targetRollAngleRadians,
            lastPositionCommand_.targetPitchAngleRadians,
            targetAttitudeDegrees_[2] * degreesToRadians
        };

        lastAttitudeTorque_ = attitudeController_.computeTorque(
            currentAnglesRad, angularRatesRad, targetAnglesRad, deltaTimeSec
        );

        motorOutputs_ = motorMixer_.computeMixer(
            lastPositionCommand_.normalizedThrustRatio,
            lastAttitudeTorque_.rollTorqueNm,
            lastAttitudeTorque_.pitchTorqueNm,
            lastAttitudeTorque_.yawTorqueNm
        );

        quadMotors.setMotorThrottlePct(1, motorOutputs_.motor1FrontRight * 100.0f);
        quadMotors.setMotorThrottlePct(2, motorOutputs_.motor2RearLeft   * 100.0f);
        quadMotors.setMotorThrottlePct(3, motorOutputs_.motor3FrontLeft  * 100.0f);
        quadMotors.setMotorThrottlePct(4, motorOutputs_.motor4RearRight  * 100.0f);
    }
}
