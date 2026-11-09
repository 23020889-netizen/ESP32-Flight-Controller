#include "serial_telemetry_reporter.hpp"
#include "../comm/wifi_control.hpp"
#include <Arduino.h>

SerialTelemetryReporter telemetryReporter;

SerialTelemetryReporter::SerialTelemetryReporter()
    : lastReportTimestampMs_(0) {}

void SerialTelemetryReporter::update(
    uint32_t currentTimeMs,
    const MpuData& mpuData,
    const XsensData& xsensData,
    const FusedData& fusedData,
    const FlightControllerManager& manager
) {
    if (currentTimeMs - lastReportTimestampMs_ < REPORT_INTERVAL_MS) {
        return;
    }
    lastReportTimestampMs_ = currentTimeMs;

    Serial.println("-----------------------------------------------------------------------------------------------------------------");
    Serial.printf("[PX4_EKF FUSED] R:%6.1f P:%6.1f Y:%6.1f | Alt:%5.2fm Vz:%+5.2fm/s | Gyr:[%+5.1f,%+5.1f,%+5.1f] (100%% Fused Single-Source)\n",
                  fusedData.roll, fusedData.pitch, fusedData.yaw,
                  fusedData.altitudeZ, fusedData.velocityVz,
                  fusedData.gx, fusedData.gy, fusedData.gz);

    const char* modeStr = "MANUAL";
    if (manager.getFlightMode() == FlightControlMode::ALTITUDE_HOLD ||
        manager.getFlightMode() == FlightControlMode::ITSMC_ATTITUDE_HOLD) modeStr = "ALT_HOLD";
    else if (manager.getFlightMode() == FlightControlMode::DIRECT_MANUAL) modeStr = "DIRECT_RC";
    else if (manager.getFlightMode() == FlightControlMode::ITSMC_POSITION_HOLD) modeStr = "ITSMC_POS";
    else if (manager.getFlightMode() == FlightControlMode::EMERGENCY_DISARM) modeStr = "EMERGENCY";

    const QuadMotorThrottleOutput& out = manager.getMotorOutputs();
    float rcThrottle = 0.0f, rcRoll = 0.0f, rcPitch = 0.0f, rcYaw = 0.0f;
    manager.getRcChannels(rcThrottle, rcRoll, rcPitch, rcYaw);
    const char* escState = quadMotors.isArmed() ? "ARMED" : "DISARMED";
    Serial.printf("[FC_INPUT] ESC:%s | RC T:%.1f R:%.1f P:%.1f Y:%.1f | Failsafe:%d\n",
                  escState, rcThrottle, rcRoll, rcPitch, rcYaw,
                  static_cast<int>(manager.getFailsafeLevel()));
    Serial.printf("[ITSMC_CTRL] Mode:%-10s | M1:%.1f%% | M2:%.1f%% | M3:%.1f%% | M4:%.1f%%\n",
                  modeStr,
                  out.motor1FrontRight * 100.0f,
                  out.motor2RearLeft * 100.0f,
                  out.motor3FrontLeft * 100.0f,
                  out.motor4RearRight * 100.0f);

    Serial.printf("[ESC_QUAD ] State:%-8s | M1:%4dus | M2:%4dus | M3:%4dus | M4:%4dus @ %dHz\n",
                  quadMotors.getStateStr(),
                  quadMotors.getMotor(0).getCurrentPulseUs(),
                  quadMotors.getMotor(1).getCurrentPulseUs(),
                  quadMotors.getMotor(2).getCurrentPulseUs(),
                  quadMotors.getMotor(3).getCurrentPulseUs(),
                  quadMotors.getFrequency());

    // Xuat JSON telemetry cho WebSerial UI de cap nhat do thi, dong bo IP va xung dong co truc quan
    Serial.printf("{\"r\":%.2f,\"p\":%.2f,\"y\":%.2f,\"alt\":%.2f,\"vz\":%.2f,\"m1\":%d,\"m2\":%d,\"m3\":%d,\"m4\":%d,\"state\":\"%s\",\"mode\":\"%s\",\"arm\":%d,\"ip\":\"%s\"}\n",
                  fusedData.roll, fusedData.pitch, fusedData.yaw,
                  fusedData.altitudeZ, fusedData.velocityVz,
                  quadMotors.getMotor(0).getCurrentPulseUs(),
                  quadMotors.getMotor(1).getCurrentPulseUs(),
                  quadMotors.getMotor(2).getCurrentPulseUs(),
                  quadMotors.getMotor(3).getCurrentPulseUs(),
                  quadMotors.getStateStr(),
                  modeStr,
                  quadMotors.isArmed() ? 1 : 0,
                  wifiControl.getIpAddress().toString().c_str());
}

