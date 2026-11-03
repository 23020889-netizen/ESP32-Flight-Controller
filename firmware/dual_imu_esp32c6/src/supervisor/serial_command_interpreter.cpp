#include "serial_command_interpreter.hpp"
#include "../actuators/esc_pwm.h"
#include <Arduino.h>

SerialCommandInterpreter commandInterpreter;

void SerialCommandInterpreter::displayHelpInformation() const {
    Serial.println("\n============= SERIAL COMMAND CONSOLE =============");
    Serial.println("  arm / disarm     : Motor safety arm / emergency kill");
    Serial.println("  mode att         : Switch to ITSMC Attitude Hold Mode");
    Serial.println("  mode pos         : Switch to ITSMC Position Hold Mode");
    Serial.println("  mode manual      : Switch to Manual CLI Mode");
    Serial.println("  throttle <0..100>: Set base throttle percentage");
    Serial.println("  target <r> <p> <y: Set target roll/pitch/yaw angles");
    Serial.println("  set <0..100>     : Set all 4 motor throttle directly");
    Serial.println("  status / help    : Display status or help menu");
    Serial.println("===================================================\n");
}

void SerialCommandInterpreter::displayStatusInformation() const {
    Serial.printf("\nSystem State: %s | Frequency: %u Hz\n", quadMotors.getStateStr(), quadMotors.getFrequency());
    for (int i = 0; i < ESC_NUM_MOTORS; ++i) {
        const EscMotor& m = quadMotors.getMotor(i);
        Serial.printf("Motor %d | Pin: GPIO %-2d | Pulse: %4d us | Throttle: %5.1f%%\n",
                      m.getId(), m.getPin(), m.getCurrentPulseUs(), m.getThrottlePct());
    }
}

void SerialCommandInterpreter::processIncomingCommands(FlightControllerManager& manager) {
    if (!Serial.available()) return;
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.length() == 0) return;

    if (cmd.equalsIgnoreCase("arm")) {
        quadMotors.arm();
    } else if (cmd.equalsIgnoreCase("disarm") || cmd.equalsIgnoreCase("stop")) {
        manager.setFlightMode(FlightControlMode::EMERGENCY_DISARM);
    } else if (cmd.equalsIgnoreCase("mode att")) {
        manager.setFlightMode(FlightControlMode::ITSMC_ATTITUDE_HOLD);
        Serial.println("[CMD] Switched to ITSMC Attitude Hold Mode.");
    } else if (cmd.equalsIgnoreCase("mode pos")) {
        manager.setFlightMode(FlightControlMode::ITSMC_POSITION_HOLD);
        Serial.println("[CMD] Switched to ITSMC Position Hold Mode.");
    } else if (cmd.equalsIgnoreCase("mode manual")) {
        manager.setFlightMode(FlightControlMode::MANUAL_CLI);
        Serial.println("[CMD] Switched to Manual CLI Mode.");
    } else if (cmd.startsWith("throttle ")) {
        float pct = cmd.substring(9).toFloat();
        manager.setManualThrottle(pct * 0.01f);
        Serial.printf("[CMD] Target throttle set to: %.1f%%\n", pct);
    } else if (cmd.startsWith("target ")) {
        float r = 0.0f, p = 0.0f, y = 0.0f;
        sscanf(cmd.c_str() + 7, "%f %f %f", &r, &p, &y);
        manager.setTargetAttitude(r, p, y);
        Serial.printf("[CMD] Target attitude: Roll=%.1f Pitch=%.1f Yaw=%.1f deg\n", r, p, y);
    } else if (cmd.startsWith("set ")) {
        float pct = cmd.substring(4).toFloat();
        quadMotors.setAllThrottlePct(pct);
    } else if (cmd.equalsIgnoreCase("status")) {
        displayStatusInformation();
    } else if (cmd.equalsIgnoreCase("help") || cmd.equalsIgnoreCase("?")) {
        displayHelpInformation();
    } else {
        Serial.printf("[CMD] Unknown command: '%s'. Enter 'help'.\n", cmd.c_str());
    }
}

