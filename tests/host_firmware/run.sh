#!/usr/bin/env bash
# Biên dịch mã điều khiển của firmware (px4_pid.cpp, px4_filters.cpp) cho máy tính và chạy mô phỏng:
#  - frame_sim: dấu vòng kín 3 trục theo vật lý đã xác minh trên khung thật
#  - yaw_sim:   điều khiển Yaw (cần gạt = tốc độ, ESC 50 Hz, ma sát giá thử)
#  - xsens_sign_test: driver Xsens thật, dấu Yaw/gz theo chiều la bàn
set -euo pipefail
cd "$(dirname "$0")"
FW=../../firmware/dual_imu_esp32c6
OUT="${TMPDIR:-/tmp}"
status=0
for t in frame_sim yaw_sim; do
    g++ -std=c++17 -O1 -Wall -Wextra -Wno-unused-parameter -I. -include Arduino.h \
        "$t.cpp" "$FW/px4_pid.cpp" "$FW/px4_filters.cpp" -o "$OUT/fc_$t"
    "$OUT/fc_$t" || status=1
done
g++ -std=c++17 -O1 -Wall -Wextra -Wno-unused-parameter -I. -include Arduino.h \
    xsens_sign_test.cpp "$FW/xsens_mti.cpp" -o "$OUT/fc_xsens_sign_test"
"$OUT/fc_xsens_sign_test" || status=1
exit $status
