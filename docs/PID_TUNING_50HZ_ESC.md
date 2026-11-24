# Thông số PID cho ESC 50 Hz & vì sao drone lộn nhào khi tune

Áp dụng cho `firmware/dual_imu_esp32c6` trên nhánh `claude/bold-ramanujan-nwwhoj`.

Mọi con số dưới đây được kiểm chứng bằng `tests/host_firmware/run.sh`. Script này biên dịch chính `px4_pid.cpp` và `pid_autotuner.cpp` của firmware rồi chạy trên mô hình sát thực tế:

| Thành phần | Thông số trong mô hình |
|---|---|
| Xsens | 100 Hz, trễ 8–20 ms |
| Vòng PID | 200 Hz |
| ESC | chốt lệnh mỗi 20 ms (50 Hz) |
| Motor + cánh A2212 | hằng số thời gian 40–120 ms |
| Giá thử | con lắc trọng lực, lệch trọng tâm |
| Độ cứng khung k | 200 (giá thử nặng) … 1300 (drone nhẹ, bay tự do), đơn vị rad/s² cho mô-men chuẩn hóa 1.0 |

## 1. Nguyên nhân lộn nhào (theo mức ảnh hưởng)

| # | Nguyên nhân | Hậu quả | Đã sửa |
|---|---|---|---|
| 1 | Sau khi ARM, **PID mặc định TẮT** (`pid_enabled = false`). ARM bằng tay cầm (gói `RC`), lệnh `arm` hay `throttle` đều không bật PID | Đẩy ga mà Roll/Pitch **không có mô-men hiệu chỉnh nào**, trọng lực kéo lật | PID **BẬT mặc định** khi khởi động và sau mỗi lần DISARM. `pid off` vẫn dùng được |
| 2 | Hệ số cũ Kp 0.06, Ki 0.03, Kd 0, Angle 3.0 | Tổng trễ ~35 ms (ESC 50 Hz + motor + Xsens) giới hạn băng thông vòng trong ~10 rad/s. Kp 0.06 **dao động và lật ở 6/9 trường hợp** (mọi khung k ≥ 700) | Bộ mới (mục 2): ổn định 9/9 |
| 3 | Auto-tune relay chỉ dựa trên tốc độ góc, không có thông tin góc | Trọng lực, lệch trọng tâm và trễ làm trục đang tune trôi rồi lật | Relay quanh **mốc giữ góc** + **bù lệch** |
| 4 | Biên độ relay cố định ±0.10 | Khung nhẹ dao động > 200°/s, nghiêng > 40° trước khi đo xong | **Biên độ tự thích nghi**, giữ dao động 20–90°/s |
| 5 | Đã bỏ ngắt góc (`angle_abs` bị bỏ qua) | Trục tune trôi tới ngưỡng failsafe 60°, firmware **DISARM**, drone rơi | Ngắt ở 35° (và khi > 250°/s), **PID nhận lại quyền**, không DISARM |
| 6 | `tune roll/pitch` xả tích phân PID về 0 | Mất mô-men bù trọng lực ngay lúc vào tune | Relay bắt đầu từ tích phân PID; khi kết thúc thì bàn giao ngược lại |
| 7 | `tune apply` dùng Ziegler–Nichols cổ điển (0.6·Ku); Angle P tới 9 | Mất ổn định ở 3/10 khung khi mô hình lệch 30% | Luật 0.40·Ku (mềm hơn: 0.28·Ku); Angle P 3–6; kẹp vùng an toàn |
| 8 | Hệ số tune hỏng được lưu NVS và **nạp lại ở mỗi lần khởi động** | Lật ở mọi lần thử | NVS có phiên bản; hệ số cũ bị **bỏ qua** (in ra Serial), hệ số ngoài vùng an toàn bị từ chối |
| 9 | Tích phân chỉ bật khi ga ≥ 25% | Trên giá thử thường chạy dưới 25% ga, không bù được lệch trọng tâm, drone ngả dần | Bật từ 15% ga + anti-windup theo giới hạn thực của Mixer |
| 10 | `level` / `pid on` xả tích phân và ép bộ lọc Gyro về 0 giữa lúc bay | Hụt mô-men + xung P/D | Chỉ xả tích phân khi chuyển từ ACRO sang; bộ lọc Gyro chạy liên tục |

## 2. Thông số đề xuất (mặc định mới, `PidDefaults` trong `px4_pid.h`)

| Vòng | Trục | Kp | Ki | Kd | Ghi chú |
|---|---|---|---|---|---|
| Rate | Roll, Pitch | **0.040** | **0.080** | **0.0010** | I-max 0.20, lọc D 30 Hz (bản cũ lọc 10 Hz làm mất pha dẫn của D) |
| Rate | Yaw | 0.20 | 0.10 | 0 | I-max 0.15 |
| Angle | Roll, Pitch | **4.0** | – | – | tối đa 60°/s |
| Angle | Yaw | 1.5 | – | – | tối đa 45°/s |

Kết quả mô phỏng: nghiêng ban đầu 10°, ga 45%. "Lệch TT" là lệch trọng tâm (6% chênh lệch ga để giữ), "con lắc" là trọng tâm nằm trên trục xoay của giá.

| Trường hợp | Bộ cũ | Bộ mới |
|---|---|---|
| k=200 cân bằng | ổn định | ổn định, 0.4 s |
| k=200 lệch TT + con lắc | ổn định | ổn định, max 37°, 2.9 s |
| k=400 con lắc mạnh | ổn định | ổn định, max 18°, 0.8 s |
| k=700 cân bằng | **dao động** | ổn định, 0.4 s |
| k=700 lệch TT + con lắc | **dao động** | ổn định, max 34°, 1.4 s |
| k=1000 cân bằng | **dao động** | ổn định, 0.4 s |
| k=1300 | **lật** | ổn định, 1.3 s |
| k=700, motor chậm 0.12 s | **lật** | ổn định, 1.5 s |
| k=700, trễ Xsens 20 ms | **lật** | ổn định, 1.1 s |

Bộ mới vẫn dao động ở k ≈ 1600. Nếu khung thật nhẹ/cứng như vậy, auto-tune (mục 3) sẽ tự hạ Kp.

## 3. Quy trình trên giá thử

0. **Nạp firmware, mở Serial 115200.**
   - Nếu thấy `[NVS] BO QUA he so PID da luu…`: firmware đã bỏ qua hệ số cũ và in giá trị cũ ra.
   - Dòng `[SETUP] PID dang dung: …` phải là `Roll 0.0400/0.0800/0.00100` và `Angle 4.00/4.00/1.50`.
1. **Kiểm tra cơ khí.**
   - Trục xoay của giá nên đi qua trọng tâm drone, hoặc cao hơn một chút (drone "treo" dưới trục thì tự ổn định).
   - Trọng tâm càng nằm cao trên trục xoay thì trọng lực càng kéo lật mạnh. Với ESC 50 Hz và motor A2212, có mức lệch mà không bộ PID nào giữ nổi.
2. **ARM** (PID đã bật sẵn) rồi tăng ga từ từ lên **20–30%**.
   - Drone giữ thăng bằng: chuyển sang bước 3.
   - Rung nhanh (2–5 Hz, kêu rè rè): giảm Kp, ví dụ `pid roll 0.030 0.060 0.0008` và `pid pitch 0.030 0.060 0.0008`.
   - Ngả chậm, kéo về yếu: tăng `pid angle 5 5 1.5`, hoặc tăng Ki (ví dụ `pid roll 0.040 0.120 0.0010`).
3. **Auto-tune từng trục** (ga ≥ 15%, tốt nhất gần ga treo, drone đang thăng bằng):
   1. Gõ `tune roll`. Firmware từ chối nếu chưa ARM, ga < 15% hoặc nghiêng > 15°.
   2. Chờ `[TUNE] HOAN THANH …`. Trong lúc tune, drone dao động nhẹ quanh 0° (mô phỏng: lệch tối đa 2–7°).
   3. Gõ `tune apply` (bộ chuẩn 0.40·Ku) hoặc `tune apply 50` (mềm hơn). Hệ số được lưu NVS.
   4. Thử lại bước 2, sau đó tune tiếp `tune pitch` và `tune yaw`.
   5. Nếu bị ngắt (`[TUNE] Ngat …`), PID tự giữ thăng bằng. Có thể giảm biên độ relay tối đa (ví dụ `tune roll 5`, tức 5%) rồi chạy lại.
4. **Các lệnh hữu ích.**
   - `pid`: xem hệ số đang dùng.
   - `pid reset`: về mặc định trên và xóa NVS.
   - `pid off`: chế độ ACRO, **không có ổn định Roll/Pitch**. Chỉ dùng khi biết rõ mình đang làm gì.

## 4. Giới hạn của các con số trên

- Đây là kết quả **mô phỏng**. Độ cứng khung, trễ motor và vị trí trọng tâm thật chưa đo được, nên quy trình mục 3 (đặc biệt là auto-tune) là cách để hệ số khớp với khung thật.
- Firmware mới chỉ được kiểm tra cú pháp bằng stub Arduino/WiFi. Cần biên dịch bằng Arduino IDE / `arduino-cli` (core esp32 3.x) trước khi nạp.
