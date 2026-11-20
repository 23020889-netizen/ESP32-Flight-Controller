#!/usr/bin/env bash
# Biên dịch các module điều khiển của firmware cho máy tính và chạy mô phỏng vòng kín.
set -euo pipefail
cd "$(dirname "$0")"
FW=../../firmware/dual_imu_esp32c6
OUT="${TMPDIR:-/tmp}/fc_host_sim"
g++ -std=c++17 -O1 -Wall -Wextra -Wno-unused-parameter -I. -include Arduino.h \
    sim_main.cpp "$FW/px4_pid.cpp" "$FW/px4_filters.cpp" "$FW/pid_autotuner.cpp" "$FW/telemetry_json.cpp" \
    -o "$OUT"
"$OUT"
