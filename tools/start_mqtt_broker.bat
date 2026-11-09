@echo off
title ISILAB UAV MQTT Broker
echo ======================================================================
echo    KHOI DONG ISILAB UAV MQTT BROKER (TCP 1883 + WS 9001)
echo ======================================================================

:: Tu dong don dep port cu neu co process bi treo
for /f "tokens=5" %%a in ('netstat -ano ^| findstr ":1883" ^| findstr "LISTENING"') do (
    taskkill /F /PID %%a >nul 2>&1
)
for /f "tokens=5" %%a in ('netstat -ano ^| findstr ":9001" ^| findstr "LISTENING"') do (
    taskkill /F /PID %%a >nul 2>&1
)

python "%~dp0mqtt_broker.py"
pause
