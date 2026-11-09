@echo off
title ISILAB Drone - Xbox Gamepad to ESP32 Tester
cd /d "%~dp0\.."
echo ======================================================================
echo  ISILAB UAV - XBOX GAMEPAD TO ESP32 DIRECT TEST LAUNCHER
echo ======================================================================
echo  1. Kiem tra Broker MQTT tai 127.0.0.1:1883
echo  2. Doc can analog tu tay cam Xbox 360
echo  3. Ban goi tin Protocol V2 vao topic isilab/drone/rc
echo  4. ESP32 se in truc tiep ra Serial Monitor (115200)!
echo ======================================================================
echo.
python tools\test_gamepad_to_esp32.py
pause

