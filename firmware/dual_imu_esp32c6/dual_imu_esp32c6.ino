#include <Arduino.h>
#include "src/sensors/mpu6050.h"
#include "src/sensors/xsens_mti.h"
#include "src/estimation/px4_ekf.h"
#include "src/actuators/esc_pwm.h"
#include "src/supervisor/flight_controller_manager.hpp"
#include "src/supervisor/serial_telemetry_reporter.hpp"
#include "src/supervisor/serial_command_interpreter.hpp"
#include "src/comm/wifi_control.hpp"

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

TaskHandle_t xFlightTaskHandle = NULL;
TaskHandle_t xCommTaskHandle   = NULL;

// ============================================================================
// TASK 1: DETERMINISTIC FLIGHT CONTROL (High Priority, 200 Hz - 5ms Period)
// Ưu tiên cao nhất (Priority 5): Ngắt tiền định mọi tác vụ truyền thông/mạng
// ============================================================================
void flightControlTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(5); // 200 Hz hard-realtime
    uint32_t prevMicros = micros();

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        uint32_t currentMicros = micros();
        float deltaTimeSec = (currentMicros - prevMicros) * 1.0e-6f;
        if (deltaTimeSec <= 0.001f || deltaTimeSec > 0.02f) {
            deltaTimeSec = 0.005f; // Safe fallback (200 Hz)
        }
        prevMicros = currentMicros;

        // 1. Acquire sensor readings
        mpu6050_read(mpuData);
        xsens_mti_update(xsensData);

        // 2. Execute sensor fusion (PX4 EKF)
        ekf.update(mpuData, xsensData, fusedData);

        // 3. Execute Cascaded Flight Control (Attitude & Altitude loops)
        flightManager.update(fusedData, deltaTimeSec);

        // 4. Update ESC motor PWM state machine & slew limiter
        quadMotors.update();
    }
}

// ============================================================================
// TASK 2: NETWORK & TELEMETRY COMMUNICATION (Low Priority, 40 Hz - 25ms Period)
// Chạy ở mức ưu tiên thấp (Priority 1): Không bao giờ làm trễ vòng lặp bay
// ============================================================================
void commTelemetryTask(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(25); // 40 Hz

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        // 1. Parse serial CLI user commands
        commandInterpreter.processIncomingCommands(flightManager);

        // 2. Report status and telemetry periodically (1 Hz Serial heartbeat)
        telemetryReporter.update(millis(), mpuData, xsensData, fusedData, flightManager);

        // 3. Process Wi-Fi WebSocket & UDP RC packets & broadcast telemetry (40 Hz)
        wifiControl.update(fusedData, flightManager, quadMotors);
    }
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("\n[SYSTEM] ESP32-C6 MULTI-TASK REAL-TIME FLIGHT CONTROLLER");

    // 1. Initialize MPU-6050 via Fast I2C (400 kHz) and calibrate Zero-Bias
    if (mpu6050_init(MPU_SDA_PIN, MPU_SCL_PIN, 400000)) {
        mpu6050_calibrate_gyro(100);
    }

    // 2. Initialize Xsens MTi-3 via UART1 (RX=GPIO 2, TX=GPIO 3)
    xsens_mti_init(XSENS_RX_PIN, XSENS_TX_PIN, XSENS_BAUDRATE);
    xsens_mti_set_axis_inversion(true, false, true);

    // 3. Configure PX4-EKF weights (Xsens 85%, MPU-6050 15%)
    ekf.setSensorWeights(0.85f, 0.15f);

    // 4. Initialize ESC PWM drivers for 4 Quad-X motors
    quadMotors.init(ESC_DEFAULT_FREQ_HZ);

    // 5. Initialize Flight Controller Supervisor
    flightManager.initialize();

    // 6. Initialize Wi-Fi (Auto-scan & Connect to 'VNU-IS TTDBCLKT' + SoftAP 'ISILAB_DRONE_AP')
    wifiControl.begin("VNU-IS TTDBCLKT", "Ttdbclkt9", "ISILAB_DRONE_AP", "isilab2026");

    // 7. Create FreeRTOS Real-Time Tasks (Preemptive Multitasking)
    // Task 1: Flight Control (Priority 5 - Preempts all communication tasks)
    xTaskCreate(
        flightControlTask,
        "FlightCtrlTask",
        8192,
        NULL,
        5,
        &xFlightTaskHandle
    );

    // Task 2: Network & Telemetry (Priority 1 - Runs in background)
    xTaskCreate(
        commTelemetryTask,
        "CommTelemTask",
        8192,
        NULL,
        1,
        &xCommTaskHandle
    );

    Serial.println("[SYSTEM] FreeRTOS Multi-Tasking started: FlightControl (200Hz, Pri=5), CommTelem (40Hz, Pri=1)");
    Serial.println("[SYSTEM] Initialization complete. Type 'help' for CLI commands.\n");
}

void loop() {
    // Both tasks run independently in FreeRTOS. Sleep Arduino main loop task.
    vTaskDelay(pdMS_TO_TICKS(1000));
}
