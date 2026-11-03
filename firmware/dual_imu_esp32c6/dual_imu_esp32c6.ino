#include <Arduino.h>
#include "src/sensors/mpu6050.h"
#include "src/sensors/xsens_mti.h"
#include "src/estimation/px4_ekf.h"
#include "src/actuators/esc_pwm.h"
#include "src/supervisor/flight_controller_manager.hpp"
#include "src/supervisor/serial_telemetry_reporter.hpp"
#include "src/supervisor/serial_command_interpreter.hpp"

// Hardware Pin Configuration (ESP32-C6)
#define MPU_SDA_PIN     14
#define MPU_SCL_PIN     15
#define XSENS_RX_PIN    2
#define XSENS_TX_PIN    3
#define XSENS_BAUDRATE  115200

MpuData   mpuData   = {0};
XsensData xsensData = {0};
FusedData fusedData = {0};
PX4_EKF   ekf;

static uint32_t previousLoopMicros = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n[SYSTEM] ESP32-C6 DUAL-IMU FUSION & ITSMC FLIGHT CONTROLLER");

    // 1. Initialize MPU-6050 via I2C (GPIO 14, 15) and calibrate Zero-Bias
    if (mpu6050_init(MPU_SDA_PIN, MPU_SCL_PIN)) {
        mpu6050_calibrate_gyro(100);
    }

    // 2. Initialize Xsens MTi-3 via UART1 (RX=GPIO 2, TX=GPIO 3)
    xsens_mti_init(XSENS_RX_PIN, XSENS_TX_PIN, XSENS_BAUDRATE);
    xsens_mti_set_axis_inversion(true, false, true);

    // 3. Configure PX4-EKF weights (Xsens 85%, MPU-6050 15%)
    ekf.setSensorWeights(0.85f, 0.15f);

    // 4. Initialize ESC PWM drivers for 4 Quad-X motors
    quadMotors.init(ESC_DEFAULT_FREQ_HZ);

    // 5. Initialize ITSMC Flight Controller Supervisor
    flightManager.initialize();

    previousLoopMicros = micros();
    Serial.println("[SYSTEM] Initialization complete. Type 'help' for CLI commands.\n");
}

void loop() {
    uint32_t currentMicros = micros();
    float deltaTimeSec = (currentMicros - previousLoopMicros) * 1.0e-6f;
    if (deltaTimeSec <= 0.0001f || deltaTimeSec > 0.05f) {
        deltaTimeSec = 0.005f; // Safe fallback (200 Hz)
    }
    previousLoopMicros = currentMicros;

    // 1. Acquire sensor readings
    mpu6050_read(mpuData);
    xsens_mti_update(xsensData);

    // 2. Execute sensor fusion (PX4 EKF)
    ekf.update(mpuData, xsensData, fusedData);

    // 3. Execute ITSMC Flight Control (Attitude & Position loops)
    flightManager.update(fusedData, deltaTimeSec);

    // 4. Update ESC motor PWM state machine & slew limiter
    quadMotors.update();

    // 5. Parse serial CLI user commands
    commandInterpreter.processIncomingCommands(flightManager);

    // 6. Report status and telemetry periodically (20 Hz)
    telemetryReporter.update(millis(), mpuData, xsensData, fusedData, flightManager);
}
