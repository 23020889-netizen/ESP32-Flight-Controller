# Báo cáo rà soát & sửa lỗi: PID 2 vòng, xử lý IMU, truyền telemetry lên Web

Phạm vi: `firmware/dual_imu_esp32c6/*`, `tools/hitl_monitor_web/.../telemetry_data_store.js`.
Kiểm chứng: `tests/host_firmware/run.sh` (biên dịch **mã firmware thật** cho máy tính + mô phỏng vòng kín)
và `node tests/test_telemetry_store.js`.

## 1. Kiến trúc điều khiển sau khi sửa

```
IMU (Xsens AHRS 100 Hz + MPU6050 gyro)  ──► chọn nguồn theo độ "tươi" (<100 ms)
        │ góc (deg)              │ gyro (deg/s) ──► Butterworth 30 Hz (luôn chạy)
        ▼                        ▼
 VÒNG NGOÀI  Angle-P:  e_góc * Kp_angle = [φ̇, θ̇, ψ̇]_sp
             ──► chuyển Euler-rate → Body-rate [p, q, r]_sp   (MỚI)
        ▼
 VÒNG TRONG  Rate-PID (rad/s): P + I(anti-windup theo quyền Mixer) + D-on-measurement (LPF 20 Hz)
        ▼  τ = [τ_roll, τ_pitch, τ_yaw]  (giới hạn = quyền điều khiển của Mixer tại mức ga hiện tại)
 MIXER Quad-X + desaturation  ──► u1..u4 ──► ESC (ghi thẳng, KHÔNG qua Soft-Ramp)   (MỚI)
```

## 2. Lỗi điều khiển (PID) đã sửa

| # | Mức độ | Lỗi | Hậu quả | Sửa |
|---|---|---|---|---|
| C1 | Cao | Tích phân vòng trong chỉ kẹp `±i_max`, không biết Mixer đã cắt mô-men về `±0.06..0.35` | Integral windup khi ga thấp/bão hòa → vọt lố, chậm hồi | Anti-windup *conditional integration* với giới hạn = `PX4_QuadMixer::torqueAuthority(throttle)` |
| C2 | Cao | `PX4_FlightController::reset()` (gọi **mỗi chu kỳ** khi DISARMED) reset bộ lọc Gyro về 0 | Gyro gửi lên Web khi chưa ARM chỉ còn ~13% giá trị thật (đo bằng mô phỏng: 6.6 thay vì 50 dps); lúc ARM bộ lọc khởi động từ 0 | Bộ lọc luôn chạy; `resetControllers()` chỉ xóa PID; `reset()` nạp lại bộ lọc bằng mẫu Gyro gần nhất |
| C3 | TB | Vòng ngoài xuất đạo hàm góc Euler nhưng vòng trong so với Gyro thân (body rate) | Khi nghiêng lớn, lệnh Yaw "rò" sang Roll/Pitch | Ma trận động học Euler-rate → body-rate |
| C4 | TB | Đầu ra PID đi qua Soft-Ramp ESC (bộ giới hạn tốc độ 2500 µs/s), bước ramp bị ép `(uint16_t)>=1 µs` mỗi lần gọi | Thêm khâu phi tuyến trong vòng kín, tốc độ ramp phụ thuộc tần số `loop()` | `setAllNormalized()` ghi thẳng ra PWM; ramp chỉ dùng cho chế độ tay và tích lũy dạng float; ga tổng được giới hạn tăng 120 %/s tại setpoint |
| C5 | TB | Auto-tune: trục không tune bị đặt mô-men = 0 | Drone trôi/lật trục kia khi đang tune | Trục không tune vẫn do PID giữ |
| C6 | Cao | Auto-tune Yaw: watchdog nghiêng dùng `|yaw|` (hướng la bàn) | Abort ngay nếu mũi không hướng gần 0° | Watchdog dùng `max(|roll|,|pitch|)` cho mọi trục |
| C7 | Cao | Auto-tune abort → ép IDLE cả 4 motor | Rơi tự do giữa không trung | Trả quyền lại cho PID ngay trong chu kỳ đó |
| C8 | TB | Lệnh `x m1 m2 m3 m4` bị vòng PID ghi đè IDLE ở chu kỳ kế tiếp | Lệnh test 4 motor không có tác dụng | `activeTestMotor = MANUAL_MIX_ALL` nhận quyền ghi ESC |
| C9 | Thấp | `setRatePid*()` qua CLI ghi đè `i_max = 0.30` (Yaw lẽ ra 0.25) | Sai giới hạn tích phân Yaw | Overload `setGains(kp,ki,kd)` giữ `i_max` |
| C10 | Thấp | `setLoopRate()` luôn ép cắt D = 20 Hz | Không cấu hình được D-cutoff | Lưu `_d_cutoff_hz` |
| C11 | Thấp | Lệnh `level` reset PID giữa lúc bay | Giật khi xóa tích phân | Chỉ đổi setpoint |
| C12 | TB | Vòng PID ARM bằng tay cầm RC khi đang ở chế độ test motor | PID không sở hữu đầu ra, ga RC bị bỏ qua | ARM qua RC trả `activeTestMotor = 0` |

## 3. Lỗi xử lý IMU đã sửa

| # | Mức độ | Lỗi | Sửa |
|---|---|---|---|
| I1 | Cao | `xsens.has_orientation` không bao giờ bị xóa → mất UART Xsens thì dùng góc đóng băng mãi mãi | Timestamp riêng `last_orient_ms`/`last_gyro_ms`; quá 100 ms → rơi về MPU6050 |
| I2 | Cao | Trộn Gyro 85% Xsens + 15% MPU theo cờ *orientation*: nếu Xsens không xuất Rate-of-Turn thì `gx = 0` → phản hồi vòng trong co còn 15% | Chỉ trộn khi cả hai gyro hợp lệ, nếu không dùng nguồn còn lại |
| I3 | Cao | Mất toàn bộ IMU khi ARMED: không xử lý | Failsafe DISARM sau 300 ms |
| I4 | TB | MPU6050 ±250 dps / ±2 g | Gyro bão hòa ở 250 dps (vòng ngoài cho lệnh tới 220 dps + vọt lố); Accel ±2 g bị rung bão hòa → **±1000 dps / ±8 g** |
| I5 | TB | Complementary filter MPU: `alpha = 0.96` mỗi lần gọi (loop chạy nhanh hơn 200 Hz) → hằng số thời gian phụ thuộc tốc độ loop; tin accel cả khi rung mạnh | `alpha = τ/(τ+dt)` với τ = 0.5 s; bỏ qua accel khi `|a|` lệch > 25% g |
| I6 | TB | Lỗi I2C liên tiếp vẫn để `mpuData.ready = true` | 10 lỗi liên tiếp → `ready = false` |
| I7 | Thấp | Không đếm lỗi checksum Xbus | `checksum_errors`, `packet_count`, gửi lên Web |

**Lưu ý dấu trục Xsens:** đã kiểm tra log thực `IMU_Telemetry_Log_20261102_231104.csv` — tương quan giữa
Δgóc và ∫gyro: Roll 0.99, Pitch 0.94, Yaw 0.81 (cùng dấu) → phép đổi trục hiện tại nhất quán, **không thay đổi**.

## 4. Lỗi truyền dữ liệu IMU lên Web đã sửa

| # | Mức độ | Lỗi | Sửa |
|---|---|---|---|
| T1 | Cao | Handshake WebSocket chặn `loop()` tới **400 ms** mỗi khi trình duyệt kết nối/kết nối lại | Handshake không chặn (máy trạng thái theo từng client) |
| T2 | Cao | JSON 800 byte + khối tune 240 byte: `snprintf` có thể cắt cụt → JSON hỏng, Web bỏ qua **im lặng** | Module `telemetry_json.cpp` (kiểm thử được), buffer 1400 B, cụt → không gửi |
| T3 | Cao | Số NaN/Inf in thành `nan` → `JSON.parse` lỗi | Thay bằng 0 trước khi đóng gói |
| T4 | Cao | `pushSample()` trên Web loại bỏ `gx/gy/gz`, `tune`, `tau`, `u`, setpoint | Panel Auto-Tune luôn đọc gx=0 & không thấy trạng thái tune → giữ đủ trường |
| T5 | TB | Serial in ~600 B/100 ms bằng `printf` chặn → giật vòng PID | In xoay vòng 1 dòng/16 ms, chỉ ghi khi bộ đệm TX còn chỗ (TX buffer 1024 B) |
| T6 | TB | Firmware in tag `[ATTITUDE]` kèm `(Yaw:..)` chen giữa; Web (WebSerial) và `vio_system/imu_stream.py` chờ `[PX4_EKF] ... | Acc:[..] | Gyr:[..]` → không parse được | In đúng định dạng `[PX4_EKF]`; Web vẫn hỗ trợ dòng cũ |
| T7 | TB | Header + payload WS gửi bằng 2 lần `write()` (2 segment TCP, Nagle) | 1 lần ghi, `setNoDelay(true)`; ghi thiếu → đóng client (tránh lệch khung) |
| T8 | TB | Slot client tái sử dụng không xóa buffer nhận → byte rác của client cũ dính sang client mới | `ws_release_slot()` |
| T9 | Thấp | `WiFi.status()` + `localIP().toString()` mỗi vòng loop (cấp phát String liên tục) | Kiểm tra 1 lần/giây |
| T10 | Thấp | Bản tin `{"log":...}` không escape dấu `"` | Escape chuẩn JSON |
| T11 | Thấp | Lỗi parse JSON trên Web bị nuốt im lặng | Đếm `jsonParseErrors`, hiện trên thanh trạng thái |

Trường telemetry mới: `t` (ms trên ESP32, phát hiện mất gói không phụ thuộc trễ WiFi), `rsp` (rate setpoint deg/s),
`pid_i`, `imu_src` (1 = Xsens, 2 = MPU, 0 = mất), `xs_age`, `xs_err`, `loop_hz`, `loop_max_us`.

## 5. Kiểm thử

```bash
tests/host_firmware/run.sh
```

```bash
node tests/test_telemetry_store.js
```

Mô phỏng vòng kín (ESC 50 Hz zero-order-hold, motor τ = 40 ms, trễ cảm biến 1 chu kỳ, nhiễu gyro):
bước Roll 10°/Pitch −8°/Yaw 20° bám < 0.1°; nhiễu mô-men hằng được tích phân khử về 0°;
auto-tune Roll hoàn tất trong khi Pitch giữ < 2°; JSON với NaN/Inf hợp lệ theo `json.load`.

## 6. Khuyến nghị chưa áp dụng (cần thử trên phần cứng)

- **Tần số PWM ESC 50 Hz** làm trễ tới 20 ms trong vòng 200 Hz. Nếu ESC Vàng 30A hỗ trợ, dùng lệnh `freq 400`
  (đã có sẵn) rồi tăng dần Kp vòng trong. Không đổi mặc định vì chưa xác nhận ESC chịu được 400 Hz.
- Chưa biên dịch bằng toolchain ESP32 trong lần sửa này (chỉ kiểm tra cú pháp bằng stub Arduino/WiFi trên máy tính):
  cần biên dịch lại bằng Arduino IDE/`arduino-cli` (core esp32 3.x) và chạy thử trên giá gimbal trước khi bay.
