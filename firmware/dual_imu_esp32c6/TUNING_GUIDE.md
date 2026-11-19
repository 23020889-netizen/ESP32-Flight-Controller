# TUNING_GUIDE.md — Hướng dẫn Tuning PID Auto-Tune cho UAV ESP32-C6

> **Phiên bản:** 1.0 | **Ngày:** 2026-11-19  
> **Phương pháp:** Relay / Ziegler-Nichols (tương tự Betaflight Blackbox Auto-Tune)

---

## 1. Lý thuyết — Tại sao drone rung khi arm?

Hiện tại:
```
Rate PID: Kp=0.15, Ki=0.20, Kd=0.003
Angle P:  Roll_P=6.5, Pitch_P=6.5
```
**Vấn đề:** Ki/Kp = 1.33 — tỷ lệ này cực kỳ cao cho rate loop. Vòng ngoài
(Angle P=6.5) yêu cầu tốc độ góc 6.5× sai số → vòng trong (Rate PID) không
đuổi kịp → hệ thống tích phân tăng → oscillation tự duy trì.

### 1.1 Phương pháp Relay (Åström-Hägglund)

Thay vì chạy PID bình thường trên 1 trục, ta áp **relay bang-bang**:

```
relay output = +d  nếu gyro_rate < 0
             = -d  nếu gyro_rate > 0
```

Relay cưỡng bức oscillation tần số tự nhiên của hệ → đo:
- **Tu** = chu kỳ dao động (giây)
- **A** = biên độ gyro tốc độ góc (rad/s)
- **Ku** = 4·d / (π·A)  — hệ số khuếch đại tới hạn

Từ đó tính **ZN Rule (conservative PD)**:
```
Kp = 0.60 × Ku
Ki = 1.20 × Ku / Tu
Kd = 0.075 × Ku × Tu
```

> **Note:** Áp dụng cho Rate PID (vòng trong) TRƯỚC. Chỉ tune Angle P SAU khi
> rate loop ổn định.

---

## 2. Yêu cầu an toàn — PHẢI ĐỌC TRƯỚC

| # | Điều kiện | Chi tiết |
|---|-----------|----------|
| 1 | **Không lắp cánh quạt** | Relay test được thực hiện WITHOUT props. Chỉ dùng props khi bay kiểm tra sau tuning. |
| 2 | **Kẹp drone vào giá đỡ** | Giá đỡ cố định body frame, chỉ cho phép 1 trục tự do (Roll hoặc Pitch) ±25°. |
| 3 | **Failsafe góc** | Auto-tuner tự động ABORT nếu \|roll\| > 25° hoặc \|pitch\| > 25°. |
| 4 | **Throttle an toàn** | Dùng 25–35% ga trong relay test. KHÔNG dùng < 20% (RPM không đủ tạo torque). |
| 5 | **Nguồn ổn định** | Kết nối LiPo đầy (4S: >15.0V) trước khi bắt đầu. |
| 6 | **Người giám sát** | Luôn có người ở gần kill switch (DISARM nút trên web) khi test. |

---

## 3. Kiến trúc hệ thống Auto-Tuner

```
┌─────────────────────────────────────────────────────────────────────┐
│                        ESP32-C6 (200 Hz loop)                        │
│                                                                       │
│  ┌──────────────┐    override tau[axis]    ┌────────────────────────┐│
│  │ PID_AutoTuner│ ──────────────────────► │   PX4 Mixer            ││
│  │  (new files) │                          │   Motor Outputs        ││
│  │              │ ◄── gyro_rate (rad/s) ── │   Normal PID (paused) ││
│  └──────┬───────┘                          └────────────────────────┘│
│         │ tune_status JSON (via telemetry_wifi)                       │
└─────────┼───────────────────────────────────────────────────────────┘
          │ WebSocket (port 81)
┌─────────┴───────────────────────────────────────────────────────────┐
│                   Web Browser (UAV HITL Monitor)                      │
│                                                                       │
│   ┌─────────────────────────────────────────────────────────────┐   │
│   │ PID TUNER panel (mới thêm vào HTML)                         │   │
│   │  - Chọn trục: Roll Rate / Pitch Rate / Yaw Rate / Angle P   │   │
│   │  - Slider relay amplitude (5–20%)                           │   │
│   │  - Live graph gyro rate + relay output                      │   │
│   │  - Hiển thị: Ku, Tu → Kp, Ki, Kd gợi ý                    │   │
│   │  - Nút [Bắt đầu] [Dừng] [Áp dụng] [Lưu về firmware]       │   │
│   └─────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
```

**Lệnh WebSocket điều khiển tuner:**
```
tune roll [amplitude_pct]   → bắt đầu tune Roll Rate PID
tune pitch [amplitude_pct]  → bắt đầu tune Pitch Rate PID
tune yaw [amplitude_pct]    → bắt đầu tune Yaw Rate PID
tune angle                  → bắt đầu tune Angle P (sau khi rate đã ổn)
tune stop                   → dừng ngay lập tức
tune apply                  → ghi kết quả vào PID đang chạy
```

---

## 4. Quy trình triển khai từng bước

### Bước 1 — Chuẩn bị phần cứng

1. Tháo toàn bộ **4 cánh quạt**.
2. Kẹp drone vào giá đỡ, trục **Roll** tự do (±25°), Pitch & Yaw khóa cứng.
3. Kết nối LiPo 4S đầy (≥15.0V).
4. Bật nguồn ESP32 → chờ WiFi kết nối (LED xanh).
5. Mở web: `http://10.11.162.210`

### Bước 2 — Flash firmware có Auto-Tuner

```
Arduino IDE → chọn board ESP32-C6 → Flash firmware dual_imu_esp32c6.ino
```

Firmware mới có thêm:
- File `pid_autotuner.h` và `pid_autotuner.cpp`
- `#include "pid_autotuner.h"` trong `.ino`
- Lệnh `tune *` trong `handleWirelessCommands()` / `handleSerialCommands()`
- JSON field `"tune"` trong telemetry

### Bước 3 — Tune Roll Rate PID (vòng trong)

1. Mở web → panel **"PID TUNER"** (phía dưới panel Motor Test)
2. Chọn axis: **Roll Rate**
3. Đặt relay amplitude: **15%** (mặc định)
4. Đặt throttle: **30%** trên slider Throttle
5. Bấm **[ARM]** → chờ idle spin
6. Bấm **[Bắt đầu Tune]** → theo dõi graph
7. Chờ **≥ 8 chu kỳ dao động** đầy đủ (khoảng 5–15 giây)
8. Web hiển thị: `Ku = X.XX | Tu = X.XX s → Kp = X.XX | Ki = X.XX | Kd = X.XXX`
9. Kiểm tra giá trị xem có bất thường không
10. Bấm **[Áp dụng]** → drone nhận PID mới ngay lập tức
11. Bấm **[DISARM]**

### Bước 4 — Tune Pitch Rate PID

Giống Bước 3 nhưng:
- Xoay giá đỡ → trục **Pitch** tự do, Roll & Yaw khóa
- Chọn axis: **Pitch Rate**

### Bước 5 — Tune Yaw Rate PID (tùy chọn)

Yaw thường ít oscillation hơn. Nếu cần:
- Cho phép trục Yaw xoay tự do (hoặc test trên mặt bằng phẳng)
- Chọn axis: **Yaw Rate**, amplitude **10%**

### Bước 6 — Tune Angle P (vòng ngoài)

Chỉ thực hiện SAU khi Rate PID ổn định:

1. Lắp **1/2 cánh quạt** (hoặc dùng cánh nhỏ hơn) → test bay thử nhẹ
2. Chọn axis: **Angle P**
3. Phương pháp: **step response** — đặt setpoint step 15° → đo thời gian đáp ứng
4. Web hiển thị rise time và overshoot → tự động đề xuất Roll_P, Pitch_P mới

> **Lưu ý:** Angle P thường trong khoảng 4.0–8.0 (giá trị hiện tại 6.5 là hợp lý,
> chỉ cần điều chỉnh ±20% nếu có overshoot sau khi rate loop đã ổn).

### Bước 7 — Bay thử thực tế

1. Lắp đủ 4 cánh quạt
2. Ra khoảng trống (ngoài giá đỡ)
3. Bay hover nhẹ 20–30cm
4. Quan sát: drone phải giữ được tư thế, phản ứng với lệnh stick mượt

---

## 5. Giá trị kỳ vọng sau khi tune

| Thông số | Trước tune | Sau tune (kỳ vọng) |
|----------|-----------|---------------------|
| Kp (Rate Roll/Pitch) | 0.15 | 0.20–0.40 |
| Ki (Rate Roll/Pitch) | 0.20 | 0.05–0.15 |
| Kd (Rate Roll/Pitch) | 0.003 | 0.005–0.020 |
| Angle P Roll/Pitch | 6.5 | 4.0–7.0 |
| Oscillation khi hover | Dữ dội | Không còn |

---

## 6. Serial output trong khi tune

```
[TUNE] Bắt đầu Relay Test → Axis=ROLL | Amplitude=0.15 | Throttle=30%
[TUNE] Chu kỳ 1: T_half = 0.085s | Peak = 1.24 rad/s
[TUNE] Chu kỳ 2: T_half = 0.082s | Peak = 1.31 rad/s
...
[TUNE] Chu kỳ 8: T_half = 0.083s | Peak = 1.28 rad/s
[TUNE] Kết quả: Tu=0.165s | Ku=0.37 → Kp=0.222 | Ki=2.70 | Kd=0.0046
[TUNE] Áp dụng: setRatePidRoll(0.222, 2.70, 0.0046)
```

---

## 7. Xử lý sự cố

| Triệu chứng | Nguyên nhân | Giải pháp |
|-------------|-------------|-----------|
| Không có dao động | Amplitude quá nhỏ hoặc throttle quá thấp | Tăng amplitude lên 20%, throttle 35% |
| ABORT ngay sau 2–3 giây | Góc lệch quá 25° | Kiểm tra giá đỡ — drone không đủ cứng |
| Ku quá nhỏ (<0.1) | Nhiễu gyro lớn | Kiểm tra chắc chắn các con ốc motor, bọc foam chống rung |
| Kp sau tune vẫn rung | ZN cho K=100% → thử K=50% (Kp×0.5) | Dùng nút "Áp dụng 50%" trên web |
| Tu quá dài (>2s) | Throttle không đủ | Tăng throttle lên 35–40% |

---

## 8. Lưu kết quả vĩnh viễn

Sau khi bay thử ổn, bấm **[Lưu về Firmware]** trên web. Lệnh này gửi:
```
tune save
```
Firmware sẽ in ra Serial các lệnh `setGains()` với giá trị mới → copy vào
`px4_pid.cpp` tại hàm `init()` và flash lại.

**Hoặc** tự sửa tay trong `px4_pid.cpp`:
```cpp
// Inner Rate PID — cập nhật theo kết quả auto-tune
_rate_pid_roll.init(0.222f, 2.70f, 0.0046f, 0.30f, 20.0f, loop_freq_hz);
```

---

## 9. Thứ tự tuning (quan trọng)

```
[1] Rate Roll PID   ← TRƯỚC TIÊN (trục dễ quan sát nhất)
[2] Rate Pitch PID  ← Thường tương tự Roll
[3] Rate Yaw PID   ← Tùy chọn
[4] Angle Roll P   ← SAU KHI Rate đã ổn
[5] Angle Pitch P  ← Thường bằng Roll P
[6] Bay thử thực tế
```

> **KHÔNG bao giờ tune Angle P trước Rate PID** — sẽ không có kết quả.

---

*Hướng dẫn này đi kèm với code `pid_autotuner.h`, `pid_autotuner.cpp` và
panel "PID TUNER" trong `UAV HITL Testbed Monitor.html`.*
