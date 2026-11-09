#include "serial_command_interpreter.hpp"
#include "../actuators/esc_pwm.h"
#include <Arduino.h>

SerialCommandInterpreter commandInterpreter;

void SerialCommandInterpreter::displayHelpInformation() const {
    Serial.println("\n============= SERIAL COMMAND CONSOLE =============");
    Serial.println("  arm / disarm     : Motor safety arm / emergency kill");
    Serial.println("  test <1..4> <pct>: QGC-style single motor test (e.g. 'test 1 10 3')");
    Serial.println("  m1 / m2 / m3 / m4: Quick single motor test (e.g. 'm1 15')");
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
    static char rxBuf[128];
    static uint8_t rxIdx = 0;

    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c != '\n') {
            if (rxIdx < sizeof(rxBuf) - 1) {
                rxBuf[rxIdx++] = c;
            } else {
                rxIdx = 0;
            }
            continue;
        }

        rxBuf[rxIdx] = '\0';
        String cmd = String(rxBuf);
        rxIdx = 0;
        cmd.trim();
        if (cmd.length() == 0) continue;

    // Web UI dùng chung giao thức dòng cho WebSocket và WebSerial.
    // Nhận RC,<throttle>,<roll>,<pitch>,<yaw> thay vì bỏ qua như lệnh lạ.
    if (cmd.startsWith("RC,") || cmd.startsWith("rc,")) {
        int c1 = cmd.indexOf(',', 3);
        int c2 = (c1 != -1) ? cmd.indexOf(',', c1 + 1) : -1;
        int c3 = (c2 != -1) ? cmd.indexOf(',', c2 + 1) : -1;
        if (c1 != -1 && c2 != -1 && c3 != -1) {
            float t = cmd.substring(3, c1).toFloat();
            float r = cmd.substring(c1 + 1, c2).toFloat();
            float p = cmd.substring(c2 + 1, c3).toFloat();
            float y = cmd.substring(c3 + 1).toFloat();

            quadMotors.feedWatchdog();
            manager.setRcChannels(t, r, p, y);
            static uint32_t lastRcLogMs = 0;
            const uint32_t now = millis();
            if (now - lastRcLogMs >= 1000) {
                lastRcLogMs = now;
                Serial.printf("[RC_RX_USB] T:%.1f R:%.1f P:%.1f Y:%.1f | Armed:%d Mode:%d\n",
                              t, r, p, y, quadMotors.isArmed() ? 1 : 0,
                              static_cast<int>(manager.getFlightMode()));
            }
        } else {
            Serial.printf("[RC_RX_USB] Invalid packet: %s\n", cmd.c_str());
        }
        return;
    }

    // Web UI gửi MODE,<name>; hỗ trợ thêm giao thức này cho WebSerial.
    if (cmd.startsWith("MODE,") || cmd.startsWith("mode,")) {
        String modeName = cmd.substring(5);
        modeName.trim();
        if (modeName.equalsIgnoreCase("DIRECT_MANUAL") || modeName.equalsIgnoreCase("DIRECT")) {
            manager.setFlightMode(FlightControlMode::DIRECT_MANUAL);
        } else if (modeName.equalsIgnoreCase("ALTITUDE") || modeName.equalsIgnoreCase("ALT_HOLD") ||
                   modeName.equalsIgnoreCase("ALT") || modeName.equalsIgnoreCase("STABILIZED")) {
            manager.setFlightMode(FlightControlMode::ALTITUDE_HOLD);
        } else if (modeName.equalsIgnoreCase("POSHOLD") || modeName.equalsIgnoreCase("ITSMC_POS")) {
            manager.setFlightMode(FlightControlMode::ITSMC_POSITION_HOLD);
        } else if (modeName.equalsIgnoreCase("MANUAL_CLI")) {
            manager.setFlightMode(FlightControlMode::MANUAL_CLI);
        } else {
            Serial.printf("[MODE_USB] Unknown mode: %s\n", modeName.c_str());
            return;
        }
        Serial.printf("[MODE_USB] Active mode: %s\n", modeName.c_str());
        return;
    }

    if (cmd.equalsIgnoreCase("arm")) {
        if (manager.getFlightMode() == FlightControlMode::EMERGENCY_DISARM ||
            manager.getFlightMode() == FlightControlMode::MANUAL_CLI) {
            manager.setFlightMode(FlightControlMode::ALTITUDE_HOLD);
        }
        quadMotors.arm();
        Serial.println("[CMD] >>> DA MO KHOA DONG CO (ARMED) qua USB Serial!");
    } else if (cmd.equalsIgnoreCase("disarm") || cmd.equalsIgnoreCase("stop") || cmd.equalsIgnoreCase("kill")) {
        manager.setFlightMode(FlightControlMode::EMERGENCY_DISARM);
        quadMotors.disarm();
        Serial.println("[CMD] >>> DA KHOA DONG CO (DISARMED) qua USB Serial!");
    } else if (cmd.startsWith("test ") || cmd.startsWith("motortest ") || cmd.startsWith("TEST ")) {
        // Chuyển sang MANUAL_CLI để tránh ITSMC mixer can thiệp ghi đè ga
        manager.setFlightMode(FlightControlMode::MANUAL_CLI);

        int motorId = 1;
        float pct = 10.0f;
        int durationSec = 3;

        int firstSpace = cmd.indexOf(' ');
        int secondSpace = cmd.indexOf(' ', firstSpace + 1);
        if (secondSpace == -1) {
            motorId = cmd.substring(firstSpace + 1).toInt();
        } else {
            motorId = cmd.substring(firstSpace + 1, secondSpace).toInt();
            int thirdSpace = cmd.indexOf(' ', secondSpace + 1);
            if (thirdSpace == -1) {
                pct = cmd.substring(secondSpace + 1).toFloat();
            } else {
                pct = cmd.substring(secondSpace + 1, thirdSpace).toFloat();
                durationSec = cmd.substring(thirdSpace + 1).toInt();
            }
        }
        if (durationSec <= 0) durationSec = 3;
        quadMotors.testSingleMotor(motorId, pct, durationSec * 1000);
    } else if (cmd.startsWith("m1 ") || cmd.startsWith("M1 ")) {
        manager.setFlightMode(FlightControlMode::MANUAL_CLI);
        float pct = cmd.substring(3).toFloat();
        quadMotors.testSingleMotor(1, pct, 5000);
    } else if (cmd.startsWith("m2 ") || cmd.startsWith("M2 ")) {
        manager.setFlightMode(FlightControlMode::MANUAL_CLI);
        float pct = cmd.substring(3).toFloat();
        quadMotors.testSingleMotor(2, pct, 5000);
    } else if (cmd.startsWith("m3 ") || cmd.startsWith("M3 ")) {
        manager.setFlightMode(FlightControlMode::MANUAL_CLI);
        float pct = cmd.substring(3).toFloat();
        quadMotors.testSingleMotor(3, pct, 5000);
    } else if (cmd.startsWith("m4 ") || cmd.startsWith("M4 ")) {
        manager.setFlightMode(FlightControlMode::MANUAL_CLI);
        float pct = cmd.substring(3).toFloat();
        quadMotors.testSingleMotor(4, pct, 5000);
    } else if (cmd.equalsIgnoreCase("mode direct") || cmd.equalsIgnoreCase("mode rc")) {
        manager.setFlightMode(FlightControlMode::DIRECT_MANUAL);
        Serial.println("[CMD] Switched to DIRECT MANUAL (Open-loop Gamepad, No IMU) Mode.");
    } else if (cmd.startsWith("rc ") || cmd.startsWith("RC ")) {
        float t = 0.0f, r = 0.0f, p = 0.0f, y = 0.0f;
        int parsed = sscanf(cmd.c_str() + 3, "%f %f %f %f", &t, &r, &p, &y);
        if (parsed >= 1) {
            manager.setDirectManualRc(t, r, p, y);
            Serial.printf("[CMD] Direct RC -> Thr:%.1f%% Roll:%.1f%% Pitch:%.1f%% Yaw:%.1f%%\n", t, r, p, y);
        }
    } else if (cmd.equalsIgnoreCase("mode att") || cmd.equalsIgnoreCase("mode alt") || cmd.equalsIgnoreCase("mode altitude")) {
        manager.setFlightMode(FlightControlMode::ALTITUDE_HOLD);
        Serial.println("[CMD] Switched to Altitude Hold (Cascaded PID) Mode.");
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
        String args = cmd.substring(4);
        args.trim();
        int spaceIdx = args.indexOf(' ');
        if (spaceIdx != -1) {
            String firstPart = args.substring(0, spaceIdx);
            String secondPart = args.substring(spaceIdx + 1);
            if (firstPart.equalsIgnoreCase("all")) {
                float pct = secondPart.toFloat();
                manager.setFlightMode(FlightControlMode::MANUAL_CLI);
                quadMotors.setAllThrottlePct(pct);
                Serial.printf("[CMD] Set ALL motors to %.1f%%\n", pct);
            } else {
                int motorId = firstPart.toInt();
                float pct = secondPart.toFloat();
                if (motorId >= 1 && motorId <= 4) {
                    manager.setFlightMode(FlightControlMode::MANUAL_CLI);
                    quadMotors.testSingleMotor(motorId, pct, 3000);
                } else {
                    float pct = firstPart.toFloat();
                    manager.setFlightMode(FlightControlMode::MANUAL_CLI);
                    quadMotors.setAllThrottlePct(pct);
                    Serial.printf("[CMD] Set ALL motors to %.1f%%\n", pct);
                }
            }
        } else {
            float pct = args.toFloat();
            manager.setFlightMode(FlightControlMode::MANUAL_CLI);
            quadMotors.setAllThrottlePct(pct);
            Serial.printf("[CMD] Set ALL motors to %.1f%%\n", pct);
        }
    } else if (cmd.equalsIgnoreCase("calib_start") || cmd.equalsIgnoreCase("calib")) {
        manager.setFlightMode(FlightControlMode::MANUAL_CLI);
        quadMotors.startCalibration();
    } else if (cmd.equalsIgnoreCase("calib_finish") || cmd.equalsIgnoreCase("done")) {
        quadMotors.finishCalibration();
    } else if (cmd.startsWith("hover ")) {
        float h = cmd.substring(6).toFloat();
        if (h > 1.0f) h *= 0.01f;
        manager.setHoverThrustRatio(h);
        Serial.printf("[CMD] Configured Hover Thrust Ratio: %.2f (%.1f%%)\n", h, h * 100.0f);
    } else if (cmd.startsWith("disarm_pulse ")) {
        int us = cmd.substring(13).toInt();
        if (us >= 800 && us <= 1100) {
            quadMotors.setDisarmPulseUs((uint16_t)us);
            Serial.printf("[CMD] Configured Disarm Pulse: %d us\n", us);
        } else {
            Serial.println("[CMD] Invalid pulse (allowed: 800..1100 us)");
        }
    } else if (cmd.equalsIgnoreCase("status")) {
        displayStatusInformation();
    } else if (cmd.equalsIgnoreCase("help") || cmd.equalsIgnoreCase("?")) {
        displayHelpInformation();
    } else {
        Serial.printf("[CMD] Unknown command: '%s'. Enter 'help'.\n", cmd.c_str());
    }
    }
}


