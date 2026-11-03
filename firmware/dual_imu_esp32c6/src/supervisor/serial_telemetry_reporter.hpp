#pragma once

#include <stdint.h>
#include "../sensors/mpu6050.h"
#include "../sensors/xsens_mti.h"
#include "../estimation/px4_ekf.h"
#include "../actuators/esc_pwm.h"
#include "flight_controller_manager.hpp"

/**
 * @brief Telemetry logging and serial monitor reporting service.
 */
class SerialTelemetryReporter {
public:
    SerialTelemetryReporter();

    /**
     * @brief Formats and prints complete telemetry status to Serial if interval elapsed.
     * @param currentTimeMs Current system timestamp in milliseconds.
     * @param mpuData Data struct from MPU-6050.
     * @param xsensData Data struct from Xsens MTi-3.
     * @param fusedData Optimal estimated state from PX4 EKF.
     * @param flightManager Flight control supervisor instance.
     */
    void update(
        uint32_t currentTimeMs,
        const MpuData& mpuData,
        const XsensData& xsensData,
        const FusedData& fusedData,
        const FlightControllerManager& flightManager
    );

private:
    uint32_t lastReportTimestampMs_;
    static constexpr uint32_t REPORT_INTERVAL_MS = 50; // 20 Hz
};

extern SerialTelemetryReporter telemetryReporter;

