# Kế hoạch sửa mạng, throttle và đầu ra ESC

## Mục tiêu

Sửa ba lỗi được báo: ESP32 cần dùng IP cố định trên Wi-Fi `VNU-IS TTDBCLKT`; throttle từ tay cầm phải tới bộ điều khiển và được phản ánh trên PWM; trạng thái ARM/đầu ra bốn ESC phải đồng nhất. Web, ESP qua mạng và telemetry phải dùng cùng một hợp đồng dữ liệu.

## Baseline trong source

- Firmware hiện cấu hình STA tĩnh `10.11.162.210/24`, gateway/DNS `10.11.162.1`, SSID `VNU-IS TTDBCLKT`; dự phòng SoftAP `192.168.4.1`, WebSocket port `81`.
- Trang monitor hiện mặc định `10.11.162.210`, nhưng cho phép địa chỉ đã lưu trong `localStorage` ghi đè. Vì vậy địa chỉ firmware và địa chỉ Web có thể lệch.
- Gamepad đang tính `totalThr` 0–100% và gửi `RC,<thr>,<roll>,<pitch>,<yaw>,<arm>`. Firmware parse gói RC, nhưng phải xác minh tiếp giá trị throttle được gán vào flight controller ở mọi lần nhận gói.
- Dải ESC đã thống nhất: bốn kênh cùng 1000–2000 µs; GPIO theo thứ tự M1=7, M2=6, M3=5, M4=4.

## Chỉ thị triển khai

### 1. Cố định và đồng bộ IP trên VNU

1. Dùng một cấu hình duy nhất cho SSID, IP/mask, gateway, DNS, WebSocket port và địa chỉ dự phòng SoftAP; loại bỏ địa chỉ mặc định/ghi đè cũ trong UI.
2. Đặt Web tự kết nối tới `ws://10.11.162.210:81` và hiển thị IP/SSID/port đang kết nối cùng trạng thái STA hoặc SoftAP. Không để `localStorage` âm thầm thay đổi IP.
3. Firmware phải in ra trạng thái cấu hình IP tĩnh và IP thực tế sau khi kết nối; nếu STA không lên, hiển thị rõ SoftAP fallback để máy tính biết đổi sang `192.168.4.1`.
4. Không đổi sang DHCP khi cấu hình IP tĩnh thất bại mà không báo rõ. Ghi chú vận hành: IP cố định phải được dành riêng/loại khỏi DHCP pool của mạng VNU để tránh xung đột.

### 2. Sửa và chứng minh đường throttle

1. Chốt mapping đầu vào trên Gamepad API: nguồn throttle, trục/nút, deadzone, chiều tăng và dải chuẩn 0–100%. Hiển thị giá trị thô và giá trị chuẩn hóa trên Web để phân biệt lỗi tay cầm với lỗi truyền.
2. Giữ một lệnh RC duy nhất trên WebSocket và Serial: `RC,<throttle_pct>,<roll_deg>,<pitch_deg>,<yaw_deg>,<armed>`; xác thực đủ trường, finite và giới hạn dải ở ESP.
3. Khi ESP nhận gói hợp lệ, cập nhật setpoint throttle ngay cả khi Roll/Pitch/Yaw không đổi; lệnh heartbeat không được bỏ mất thay đổi ga. Gói lỗi hoặc timeout không được giữ trạng thái ARM vô hạn.
4. Telemetry phải trả lại giá trị throttle nhận được, mode/ARM, normalized output `u[4]` và PWM hiện tại `us[4]`; Web hiển thị riêng lệnh nhận và đầu ra thực tế.
5. Kiểm tra với throttle 0%, 10%, 25%, 50% và 75% khi props đã tháo: Web input → RC frame → ESP setpoint → allocator → bốn pulse phải đổi cùng chiều theo yêu cầu. Khi throttle 0% và armed, không tạo ga bay ngoài idle đã cấu hình.

### 3. Chẩn đoán ARM và bốn kênh motor

1. Quy định ARM chỉ mở trạng thái điều khiển; ARM không được tự tăng throttle hoặc chạy motor ở mức ga bay. DISARM luôn đưa cả bốn kênh về 1000 µs.
2. Xác nhận log/telemetry khi ARM báo cùng trạng thái và pulse cho M1–M4; đối chiếu mapping GPIO 7/6/5/4 với đúng ESC vật lý.
3. Với throttle bằng 0, bốn kênh phải cùng ở disarm/min hoặc cùng idle theo logic đã chốt. Với throttle dương, phải phát bốn lệnh từ mixer, không để M1/M3 bị bỏ qua hoặc bị task khác ghi đè.
4. Thêm chẩn đoán bench riêng từng kênh có timeout và khóa an toàn; thực hiện khi tháo cánh. Không đổi thứ tự motor, chiều motor hoặc dấu mixer chỉ để làm bốn động cơ quay giống nhau.
5. Nếu telemetry đo được pulse giống nhau nhưng ESC 1/3 không phản ứng như ESC 2/4, tách lỗi khỏi phần mềm điều khiển: kiểm tra tín hiệu/ground, chân GPIO, nguồn/ESC calibration và hoán đổi kênh có kiểm soát trên bench.

### 4. Hợp đồng Web ↔ ESP

- Một nguồn lệnh và cùng ý nghĩa đơn vị trên WebSocket/Serial; WebSocket là transport, không đổi format payload.
- Lệnh có xác thực và phản hồi/trạng thái kết nối; telemetry là snapshot có `armed/state`, setpoint RC, `u[4]`, `us[4]`, tần số và trạng thái mạng.
- Parser WebSocket phải xử lý frame phân mảnh/mask đúng, giới hạn kích thước và không làm nghẽn vòng PID. UDP chỉ được giữ nếu cùng hợp đồng payload và được kiểm tra tương đương.
- Khi mất link hoặc RC timeout, ESP vào failsafe đã định nghĩa; Web phải hiện mất kết nối, không giả hiển thị ARMED hay dùng telemetry cũ như dữ liệu hiện thời.

## Kịch bản nghiệm thu

1. **Mạng VNU:** ESP kết nối STA, in đúng IP cố định; Web mở đúng `ws://10.11.162.210:81`, nhận telemetry và gửi được lệnh. Ngắt VNU phải báo rõ và chuyển dự phòng SoftAP theo cấu hình.
2. **Đồng bộ command/telemetry:** gửi ARM, DISARM, RC và thay đổi throttle trên cả WebSocket và Serial; xác nhận ESP phản hồi cùng trạng thái/payload, UI không báo thành công nếu send thất bại.
3. **Throttle:** thử 0/10/25/50/75%; so sánh giá trị tay cầm, RC nhận, setpoint, `u[4]` và `us[4]`. Giá trị thay đổi phải đi xuyên suốt, không chỉ đổi hình joystick.
4. **Bốn ESC:** tháo cánh, đo từng GPIO bằng logic analyzer/oscilloscope. ARM/ DISARM/ throttle tạo đúng trạng thái và pulse cho đủ M1–M4; M1/M3 không mất xung.
5. **Failsafe:** ngắt WebSocket/RC giữa chừng; ESP chuyển trạng thái an toàn trong timeout, Web báo link mất và không tự ARM lại.
6. Ghi lại build output, log Serial và bảng đối chiếu `[UI input, RC received, setpoint, u1..u4, us1..us4]`. Không tuyên bố đã xác nhận quay motor thực tế nếu chỉ chạy kiểm thử mô phỏng hoặc compile.

## Phạm vi xác nhận phần cứng

Kiểm thử bằng code có thể xác nhận mapping, frame, trạng thái, GPIO/PWM target và timeout. Việc ESC/motor vật lý phản hồi cần đo trực tiếp trên bench; tháo cánh quạt trong mọi lần thử.
