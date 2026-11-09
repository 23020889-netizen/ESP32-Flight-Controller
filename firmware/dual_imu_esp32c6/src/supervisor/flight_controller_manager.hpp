#pragma once

#include <stdint.h>
#include "../estimation/px4_ekf.h"
#include "../controllers/itsmc_parameters.hpp"
#include "../controllers/itsmc_attitude_controller.hpp"
#include "../controllers/itsmc_position_controller.hpp"
#include "../actuators/quadrotor_motor_mixer.hpp"

enum class FlightControlMode {
    MANUAL_CLI,
    DIRECT_MANUAL,          // Chế độ bay tay cầm trực tiếp (Open-loop Gamepad, không cần IMU)
    ALTITUDE_HOLD,          // Chế độ giữ độ cao (Cascaded Altitude PID chuẩn PX4)
    ITSMC_ATTITUDE_HOLD = ALTITUDE_HOLD, // Alias tương thích ngược
    ITSMC_POSITION_HOLD,
    EMERGENCY_DISARM
};

// Cấp độ Failsafe khi mất kết nối tay cầm / Wi-Fi
enum class FailsafeLevel {
    NONE = 0,
    LEVEL1_ATTITUDE_LEVEL,  // Mất tín hiệu > 600ms: Tự động thăng bằng Roll=0, Pitch=0, giữ nguyên độ cao
    LEVEL2_AUTO_DESCENT,    // Mất tín hiệu > 1500ms: Tự động hạ cánh từ từ Vz = -0.6 m/s
    LEVEL3_MOTOR_CUT        // Mất tín hiệu > 3000ms: Khóa khẩn cấp Disarm, ngắt động cơ
};

// Nguồn cảm biến đo độ cao
enum class AltitudeSensorSource {
    BAROMETER_HEALTHY,                  // Cảm biến khí áp BMP280/MS5611 (Chuẩn ngoài trời)
    RANGEFINDER_HEALTHY,                // Cảm biến khoảng cách ToF/LiDAR TFmini (Chuẩn bay thấp)
    INERTIAL_ACCEL_ESTIMATED_BENCH_TEST // Ước lượng quán tính từ gia tốc kế (Chỉ dùng trên bàn testbed HITL)
};

constexpr FlightControlMode ITSMC_ATTITUDE_HOLD = FlightControlMode::ALTITUDE_HOLD;

/**
 * @brief High-level flight management coordinator integrating estimation, Cascaded PID, Failsafe, and ESCs.
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

    // Cấu hình điểm lực đẩy Hover Thrust (không đồng nhất cứng với 1500us/50%)
    void setHoverThrustRatio(float ratio) { hoverThrustRatio_ = (ratio < 0.15f) ? 0.15f : ((ratio > 0.85f) ? 0.85f : ratio); }
    float getHoverThrustRatio() const { return hoverThrustRatio_; }

    // Kênh điều khiển RC từ Gamepad / Web UI (Cần trái: Throttle/Yaw, Cần phải: Roll/Pitch)
    void setRcChannels(float throttlePct, float rollPct, float pitchPct, float yawPct);
    void getRcChannels(float& throttlePct, float& rollPct, float& pitchPct, float& yawPct) const;

    void setDirectManualRc(float throttlePct, float rollPct, float pitchPct, float yawPct) {
        setRcChannels(throttlePct, rollPct, pitchPct, yawPct);
    }
    void getDirectManualRc(float& throttlePct, float& rollPct, float& pitchPct, float& yawPct) const {
        getRcChannels(throttlePct, rollPct, pitchPct, yawPct);
    }

    void update(const FusedData& estimatedState, float deltaTimeSec);

    const QuadMotorThrottleOutput& getMotorOutputs() const { return motorOutputs_; }
    const PositionControlCommand& getLastPositionCommand() const { return lastPositionCommand_; }
    const AttitudeControlTorque& getLastAttitudeTorque() const { return lastAttitudeTorque_; }

    float getTargetAltitude() const { return targetAltitudeMeters_; }
    FailsafeLevel getFailsafeLevel() const { return currentFailsafe_; }
    AltitudeSensorSource getAltitudeSensorSource() const { return altitudeSource_; }

private:
    ItsmcParameters parameters_;
    ItsmcAttitudeController attitudeController_;
    ItsmcPositionController positionController_;
    QuadrotorMotorMixer motorMixer_;

    FlightControlMode currentMode_;
    FailsafeLevel currentFailsafe_;
    AltitudeSensorSource altitudeSource_;

    float targetAttitudeDegrees_[3];
    float targetPositionMeters_[3];
    float manualThrottleRatio_;
    float hoverThrustRatio_; // Tỉ lệ lực đẩy cân bằng trọng lực (T_hover)

    // Kênh lệnh RC hiện tại từ Gamepad
    float rcThrottle_; // 0.0 .. 100.0%
    float rcRoll_;     // -35.0 .. +35.0%
    float rcPitch_;    // -35.0 .. +35.0%
    float rcYaw_;      // -35.0 .. +35.0%
    uint32_t lastRcPacketMs_;

    // Trạng thái bộ điều khiển PID tầng giữ độ cao & cân bằng góc
    float targetAltitudeMeters_;
    float targetYawDegrees_;
    float vzIntegral_;
    float prevVzError_;

    // Bộ điều khiển Rate PID với bộ lọc D-term Filter & Anti-windup
    float rollRateIntegral_;
    float pitchRateIntegral_;
    float yawRateIntegral_;
    float prevRollRateError_;
    float prevPitchRateError_;
    float prevYawRateError_;
    float prevFilteredDRoll_;
    float prevFilteredDPitch_;

    QuadMotorThrottleOutput motorOutputs_;
    PositionControlCommand lastPositionCommand_;
    AttitudeControlTorque lastAttitudeTorque_;

    void updateDirectManual();
    void updateAltitudeHold(const FusedData& state, float dt);
};

extern FlightControllerManager flightManager;
