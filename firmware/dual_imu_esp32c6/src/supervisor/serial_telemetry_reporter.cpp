#include "serial_telemetry_reporter.hpp"
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
    if (mpuData.ready) {
        Serial.printf("[MPU6050] R:%6.1f P:%6.1f Y:%6.1f | Acc:[%6.2f,%6.2f,%6.2f] | Gyr:[%6.1f,%6.1f,%6.1f]\n",
                      mpuData.roll, mpuData.pitch, mpuData.yaw,
                      mpuData.ax, mpuData.ay, mpuData.az,
                      mpuData.gx, mpuData.gy, mpuData.gz);
    }
    if (xsensData.has_orientation || xsensData.has_accel) {
        Serial.printf("[Xsens  ] R:%6.1f P:%6.1f Y:%6.1f | Acc:[%6.2f,%6.2f,%6.2f] | Gyr:[%6.1f,%6.1f,%6.1f]\n",
                      xsensData.roll, xsensData.pitch, xsensData.yaw,
                      xsensData.ax, xsensData.ay, xsensData.az,
                      xsensData.gx, xsensData.gy, xsensData.gz);
    }
    Serial.printf("[PX4_EKF] R:%6.1f P:%6.1f Y:%6.1f | Acc:[%6.2f,%6.2f,%6.2f] | Gyr:[%6.1f,%6.1f,%6.1f] (OPTIMAL)\n",
                  fusedData.roll, fusedData.pitch, fusedData.yaw,
                  fusedData.ax, fusedData.ay, fusedData.az,
                  fusedData.gx, fusedData.gy, fusedData.gz);

    const char* modeStr = "MANUAL";
    if (manager.getFlightMode() == FlightControlMode::ITSMC_ATTITUDE_HOLD) modeStr = "ITSMC_ATT";
    else if (manager.getFlightMode() == FlightControlMode::ITSMC_POSITION_HOLD) modeStr = "ITSMC_POS";

    const QuadMotorThrottleOutput& out = manager.getMotorOutputs();
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
}

