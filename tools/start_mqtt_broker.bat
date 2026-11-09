@echo off
title ISILAB UAV MQTT Broker
echo ======================================================================
echo    KHOI DONG ISILAB UAV MQTT BROKER (TCP 1883 + WS 9001)
echo ======================================================================
python "%~dp0mqtt_broker.py"
pause
