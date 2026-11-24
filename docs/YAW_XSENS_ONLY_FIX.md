# Sửa lỗi Yaw không xoay & dùng IMU duy nhất từ Xsens

Áp dụng trên nhánh `main` (commit `cb7d7e0`). ESC Vàng 30A giữ nguyên **50 Hz**.

## 1. Nguồn IMU: chỉ Xsens MTi-3

- Góc Roll/Pitch/Yaw, Gyro và gia tốc cho bộ điều khiển lấy **100% từ Xsens**. Không dung hợp và không dự phòng bằng MPU6050.
- Dữ liệu Xsens được coi là mất nếu gói góc **hoặc** gói Gyro cũ hơn 100 ms. Khi đó failsafe sẵn có sẽ **AUTO DISARM**, và Serial báo rõ phần nào bị mất (góc/gyro).
- MPU6050 chỉ còn được đọc ở 50 Hz để hiển thị so sánh trên Web. Muốn bỏ hẳn MPU6050 khỏi `loop()`, đặt `MPU6050_MONITOR_ENABLED 0` trong `dual_imu_esp32c6.ino`.

> Ghi chú: trên `main` trước đây, góc Yaw của MPU6050 **không** đi vào vòng điều khiển (Yaw đã lấy từ Xsens). Vì vậy nhiễu Yaw của MPU không phải là nguyên nhân khiến Yaw không xoay. Nguyên nhân thật nằm ở mục 2.

## 2. Vì sao Yaw "không xoay" (tái hiện bằng mô phỏng trên mã `main`)

| Nguyên nhân | Chi tiết |
|---|---|
| Mốc Yaw tăng **theo gói RC**, không theo thời gian | `yaw_sp += ysp*0.04` mỗi gói. Khi giữ yên cần, Web chỉ gửi heartbeat mỗi 200 ms → full cần (45) chỉ cho **~9°/s**. |
| Mô-men Yaw quá nhỏ | Vòng trong Yaw cũ: Kp=0.10, Ki=0.02, i_max=0.08 → cần sai số góc ~25° mới đủ mô-men thắng ma sát ổ trục. Mô phỏng: **đứng yên ~3.5 s** rồi mới xoay chậm. |
| Mốc Yaw chạy trước không giới hạn | Nếu drone không theo kịp, sai số vượt ±180° → đường ngắn nhất đổi chiều → drone xoay ngược. |
| Auto-tune Yaw | Watchdog dùng `|yaw|` (hướng la bàn) → bị ngắt ngay nếu mũi không hướng gần 0°. |

## 3. Cách sửa

- **Cần Yaw = lệnh tốc độ xoay:** ±45 trên Web tương ứng ±45°/s.
  - Mốc hướng được tích phân theo **thời gian thực** trong vòng 200 Hz.
  - Lệnh tốc độ được cộng thẳng vào vòng trong dưới dạng **feed-forward**.
- **Giới hạn mốc Yaw:** mốc không được dẫn trước hướng thực quá 10°.
  - Khi **nhả cần** hoặc **mất gói RC > 450 ms**: khóa hướng tại góc thực tế lúc đó.
- **Hệ số vòng trong Yaw:** Kp=0.20, Ki=0.10, i_max=0.15 (theo mặc định PX4).
  - Băng thông Yaw < 2 Hz, nên ESC 50 Hz (trễ 20 ms) không gây mất ổn định.
- **Bảo vệ Yaw quay mất kiểm soát:** nếu |gz| > 120°/s liên tục 0.5 s thì **ngắt mô-men Yaw** (Roll/Pitch vẫn giữ).
  - Mô-men Yaw tự bật lại sau khi DISARM.

## 4. Kiểm tra dấu mô-men Yaw trên giá thử (`yaw check`)

Theo `ESP32_FLIGHT_CONTROL_REBUILD_GUIDE.md`, dấu Roll/Pitch/Yaw phải được chứng minh trên khung thật, không suy ra từ nhãn M1..M4. Dấu Yaw chưa từng được kiểm tra. Mô phỏng cho thấy nếu dấu sai, Yaw sẽ quay mất kiểm soát.

1. Cho phép trục Yaw của giá thử quay tự do. ARM ở chế độ PID và đặt ga **≥ 25%**.
2. Gõ lệnh `yaw check` qua Serial hoặc Web CLI.
   - Firmware áp mô-men Yaw dương hở vòng trong 1.5 s. Roll/Pitch vẫn do PID giữ.
   - Firmware đo chiều `gz` và **tự đặt + lưu vào NVS** dấu Yaw.
3. Đọc kết quả:
   - `THUAN`: dấu hiện tại đúng.
   - `NGUOC, da dao`: firmware đã tự đảo dấu.
   - `KHONG ket luan`: tăng ga hoặc giảm ma sát rồi chạy lại.
4. Lệnh liên quan:
   - `invert yaw`: đảo tay dấu Yaw (cũng lưu NVS).
   - `pid`: xem trạng thái Yaw.

## 5. Kiểm thử

```bash
tests/host_firmware/run.sh
```

Biên dịch chính `px4_pid.cpp` và `px4_filters.cpp` của firmware cho máy tính, rồi mô phỏng:
- gói RC đúng nhịp của Web (30 ms khi đổi cần, heartbeat 200 ms);
- ESC 50 Hz, motor τ = 40 ms;
- ma sát ổ trục giá thử.

Kết quả:

| Trường hợp | Kết quả |
|---|---|
| Full cần | Bắt đầu xoay sau **0.3 s**, tốc độ **~41°/s** (bản cũ: ~3.5 s và ~9°/s) |
| Nhả cần | Trôi theo quán tính 6° rồi giữ hướng |
| Mất gói RC | Dừng xoay |
| Khung yếu/mạnh (k_yaw 12..60) | Đáp ứng tắt dần |
| Dấu ngược + `invert yaw` | Điều khiển đúng |

## 6. Sửa dấu Pitch & Yaw của Mixer (xác minh trên khung thật, 2026-11-24)

Câu trả lời xác minh trên khung thật:

| Hạng mục | Kết quả trên khung thật |
|---|---|
| Bố trí động cơ (nhìn từ trên, đầu hướng lên) | M1 GPIO7 Trước-Trái CCW · M4 GPIO4 Trước-Phải CW · M3 GPIO5 Sau-Trái CW · M2 GPIO6 Sau-Phải CCW |
| Nghiêng cánh phải hạ xuống | Roll **+**, gx **+** |
| Ngóc đầu lên | Pitch **−**, gy **−** |
| Xoay cùng chiều kim đồng hồ (nhìn từ trên) | Yaw **giảm** |
| `invert pitch` | không dùng |

Đối chiếu với Mixer cũ:

| Trục | Mixer cũ khi mô-men dương | Kết quả vật lý | Hướng PID cần | Kết luận |
|---|---|---|---|---|
| Roll | tăng motor trái (M1, M3) | cánh phải hạ → Roll + | Roll + | ✅ đúng |
| Pitch | tăng motor trước (M1, M4) | ngóc đầu → Pitch − | Pitch + | ❌ hồi tiếp dương |
| Yaw | tăng cặp CCW (M1, M2) | thân xoay CW → Yaw giảm | Yaw tăng | ❌ hồi tiếp dương |

Mô phỏng 3 trục với vật lý trên (`tests/host_firmware/frame_sim.cpp`):
- **Mixer cũ:** Pitch tự lật tới −60.8° và chạm failsafe 60°. Yaw xoay ngược chiều (−37.8° thay vì về +20°).
- **Mixer mới:** cả 3 trục về đúng mốc.

Mixer mới (`px4_pid.cpp`):

```
M1 (Trước-Trái, CCW) = T + R − P − Y
M2 (Sau-Phải,   CCW) = T − R + P − Y
M3 (Sau-Trái,   CW)  = T + R + P + Y
M4 (Trước-Phải, CW)  = T − R − P + Y
```

- **Quy ước cảm biến:** Roll+ cánh phải hạ, Pitch+ chúc đầu. Yaw: xem mục 7 (đã đổi sang chiều la bàn).
- **Khóa NVS đổi thành `yaw_inv2`:** dấu Yaw đã lưu bằng `yaw check`/`invert yaw` trước bản sửa này được đo với Mixer cũ, nên bị bỏ qua.
- **Sau khi nạp firmware:** chạy lại `yaw check`. Kết quả mong đợi là **THUAN**.

## 7. Đảo dấu Yaw theo chiều la bàn (2026-11-24)

Sau khi nạp bản mục 6, người dùng báo trên khung thật:
- Xoay drone **cùng chiều kim đồng hồ** thì góc Yaw **giảm**.
- Mô hình 3D trên Web quay **ngược chiều** drone thật.

Nguyên nhân: Xsens xuất Yaw theo chuẩn ENU (tăng khi xoay ngược chiều kim đồng hồ). Còn mô hình 3D quay model theo `-yaw` (`three_dimensional_scene_manager.js`), tức là giả định Yaw tăng theo chiều kim đồng hồ (chiều la bàn).

Cách sửa:
- **Cảm biến:** `xsens_mti_set_yaw_inversion(true)` trong `setup()` đảo dấu **góc Yaw và Gyro Z cùng lúc**. Từ nay xoay cùng chiều kim đồng hồ → Yaw **tăng**, gz **dương**, và mô hình 3D quay cùng chiều drone thật.
  - Roll, Pitch và gia tốc không đổi.
- **Mixer:** cột Yaw đổi dấu theo, để vòng Yaw vẫn là hồi tiếp âm. Mô-men Yaw dương làm tăng cặp cánh CCW (M1, M2), phản mô-men làm thân xoay CW, tức Yaw tăng.

```
M1 (Trước-Trái, CCW) = T + R + P + Y
M2 (Sau-Phải,   CCW) = T − R − P + Y
M3 (Sau-Trái,   CW)  = T + R − P − Y
M4 (Trước-Phải, CW)  = T − R + P − Y
```

Hệ quả:
- **Cần Yaw:** gạt sang phải (Web gửi giá trị dương) → drone quay **phải** (cùng chiều kim đồng hồ).
- **Kết quả `yaw check` đã lưu (`yaw_inv2`) vẫn đúng:** cảm biến và cơ cấu chấp hành cùng đổi dấu, nên dấu vòng kín không đổi. Kết quả mong đợi vẫn là **THUAN**.
- **Log CSV cũ** (ví dụ `IMU_Telemetry_Log_20261102_231104.csv`) được ghi theo dấu Yaw cũ. Khi phát lại, Yaw sẽ quay ngược so với bản ghi mới.
- **Khi tích hợp VIO/giữ vị trí sau này:** quy ước Roll+/Pitch+ (FLU) kèm Yaw+ theo chiều kim đồng hồ không phải hệ trục thuận phải. Cần chuyển đổi rõ ràng khi đưa IMU vào VIO.

Kiểm thử: `tests/host_firmware/xsens_sign_test.cpp` đưa gói Xbus thật qua driver `xsens_mti.cpp`.
- Chưa đảo: Yaw 0 → −20° khi xoay cùng chiều kim đồng hồ (đúng lỗi đã báo).
- Đã đảo: Yaw 0 → +20°, gz +28.6°/s, mô hình 3D quay cùng chiều kim đồng hồ.
