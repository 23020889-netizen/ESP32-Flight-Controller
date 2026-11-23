#!/usr/bin/env bash
# Biên dịch mã điều khiển của firmware (px4_pid.cpp, px4_filters.cpp) cho máy tính và chạy mô phỏng Yaw.
set -euo pipefail
cd "$(dirname "$0")"
FW=../../firmware/dual_imu_esp32c6
OUT="${TMPDIR:-/tmp}/fc_yaw_sim"
g++ -std=c++17 -O1 -Wall -Wextra -Wno-unused-parameter -I. -include Arduino.h \
    yaw_sim.cpp "$FW/px4_pid.cpp" "$FW/px4_filters.cpp" -o "$OUT"
"$OUT"
