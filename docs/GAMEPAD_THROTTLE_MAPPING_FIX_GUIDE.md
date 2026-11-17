# Hướng dẫn sửa lỗi throttle tay cầm → Web → ESP32 → PWM

## Mục tiêu

Đảm bảo thay đổi cần ga trên tay cầm Xbox nối USB với máy tính đi liên tục qua trình duyệt/WebSocket tới ESP32, cập nhật setpoint throttle, qua bộ điều khiển/mixer và làm đổi xung PWM. Không coi số hiển thị trên hình joystick là bằng chứng ESP đã nhận lệnh.

## Kết quả rà soát code hiện tại

1. **Tay cầm được đọc trong trình duyệt:** `dashboard_application_controller.js` đọc `navigator.getGamepads()`. Với Xbox phổ thông, throttle được tính bằng giá trị lớn hơn giữa cò RT (`buttons[7].value`) và phần đẩy lên của cần trái Y (`axes[1]`, sau deadzone). Giá trị được chuẩn hóa thành `0..100%` chỉ khi biến `isGamepadArmed` ở trình duyệt là true.
2. **Có thể hiển thị throttle dù chưa gửi được:** `setThrottle()` cập nhật thanh trượt/nhãn tại máy tính. Gamepad loop cũng cập nhật UI trước/sau khi gọi `sendCommand`; vì vậy nhãn `T:...%` chứng minh trình duyệt đọc tay cầm, nhưng không chứng minh gói đã tới ESP.
3. **Lệnh có thể bị bỏ im lặng:** `TelemetryDataStore.sendCommand()` chỉ gửi khi `connectionMode` là `ws` và WebSocket đang `OPEN`, hoặc Serial đã mở. Ở chế độ `demo`, khi chưa kết nối hoặc socket chưa mở, hàm trả `false`. Gamepad loop hiện không kiểm tra kết quả này và vẫn hiện ARM/throttle theo trạng thái cục bộ.
4. **Gói RC và parser ESP có hợp đồng khớp:** Web gửi `RC,<thr>,<roll>,<pitch>,<yaw>,<arm>`; `executeCommand()` yêu cầu đủ năm trường sau `RC,`, giới hạn throttle 0–100%, cập nhật `setpointState.throttle_pct` và tăng `acceptedRcSequence` khi parse thành công.
5. **Setpoint được đẩy vào controller:** vòng điều khiển gọi `px4Controller.setTargetThrottle(setpointState.throttle_pct)`. Khi throttle dưới 2%, controller cố ý xuất `ESC_PULSE_IDLE_US` (1050 µs) cho cả bốn kênh; trên 2% mới dùng normalized motor outputs để điều khiển PWM. Vì vậy kiểm tra ga nhỏ hơn 2% bằng thay đổi motor RPM sẽ cho kết quả sai lệch kỳ vọng; telemetry xung vẫn phải phản ánh idle.
6. **Có tín hiệu xác nhận từ ESP:** telemetry JSON có `rc_seq`, `sp_t`, `u[4]`, `us[4]`, `state`, `pwm_ok`. `rc_seq` tăng mới là xác nhận firmware nhận gói RC; `sp_t` cho biết throttle được chấp nhận; `u` và `us` cho biết lệnh motor và xung đang xuất.

## Nguyên nhân ưu tiên cần sửa

Lỗi rõ nhất trong đường hiện tại là **mất phân biệt giữa trạng thái giao diện và trạng thái kết nối/ARM thật**: giao diện có thể báo tay cầm ARM và đổi throttle khi chưa có transport, còn lỗi send không được báo. Bên triển khai phải thêm cổng an toàn điều khiển theo transport đang mở, xác nhận gửi thành công, đồng bộ ARM theo telemetry ESP và làm rõ thông báo khi lệnh chưa được gửi. Không tự ARM ESP chỉ vì biến local của trình duyệt đổi.

Kế đến, xác minh mapping vật lý Xbox: browser cần log `gp.id`, `axes[]`, `buttons[7].value`, giá trị sau deadzone và `thr_pct`. Các loại tay cầm có thể dùng layout trục khác; không áp mapping Radiomaster vào Xbox hoặc ngược lại. Một lần nhấn RB/LB chỉ thay base throttle theo code hiện hành; không dùng nút này làm bằng chứng joystick throttle đã đúng.

## Chỉ thị chỉnh sửa

1. Trong Web, gom việc đọc tay cầm thành một hàm thuần có input mẫu Gamepad và output `{thrPct, rollDeg, pitchDeg, yawDeg}`. Phân biệt Xbox thường và RadioMaster bằng loại thiết bị; log trục/nút trong chế độ chẩn đoán.
2. Throttle mapping Xbox phải có đặc tả cố định: RT là `buttons[7].value` trong `[0,1]`; cần trái Y chỉ dùng nếu người dùng chọn; deadzone/chiều phải được thể hiện rõ. Kẹp và làm tròn một lần thành `[0,100]`.
3. Trước khi cho phép truyền RC, yêu cầu transport WebSocket/Serial ở trạng thái mở. Khi transport đóng hoặc mất, báo `NO LINK`, ngừng báo ARMED cục bộ và yêu cầu người vận hành ARM lại sau khi link phục hồi. Mọi `sendCommand()` phải kiểm tra kết quả, ghi log nếu thất bại.
4. Chỉ một đường truyền throttle được sử dụng trong chế độ gamepad: gói RC đầy đủ chứa throttle và arm. Không gửi thêm `throttle <pct>` từ cùng một vòng lặp, tránh hai lệnh cạnh tranh.
5. ESP xác nhận bằng telemetry: `rc_seq` tăng, `sp_t` khớp `thrPct` (sau giới hạn ga), ESC ở `ARMED`. UI hiện riêng các giá trị **Gamepad input**, **RC received/setpoint**, **PWM output**; không gộp chúng thành một nhãn.
6. Khi ARM mới, throttle phải khởi đầu ở 0; không dùng giá trị joystick đang giữ để tự tăng ga cùng thời điểm ARM. Sau ARM, chỉ lệnh RC hợp lệ mới đổi setpoint.
7. Xác minh `pwm_ok=1`. Khi `sp_t < 2%`, kỳ vọng `us=[1050,1050,1050,1050]` ở trạng thái ARMED theo logic hiện tại; khi DISARM kỳ vọng xung min 1000 µs. Với ga từ 5% trở lên, đối chiếu `u[4]` và `us[4]` thay đổi.
8. Giữ failsafe: nếu hết RC heartbeat quá `RC_WATCHDOG_TIMEOUT_MS`, firmware phải vào hành vi failsafe an toàn hiện có; không nới timeout để che lỗi truyền.

## Quy trình kiểm chứng theo từng chặng

Tháo cánh quạt trong toàn bộ thử nghiệm PWM.

| Chặng | Thao tác | Tiêu chí đạt |
|---|---|---|
| Gamepad → Browser | Gạt throttle 0%, 25%, 50%, 75%; ghi axes/buttons thô và `thrPct` | `thrPct` đi đúng chiều, chạm đúng 0/100, không nhiễu quanh deadzone |
| Browser → Transport | Mở WebSocket tới IP ESP; gạt throttle | Socket `OPEN`; send trả thành công; gửi đúng một `RC,...` mỗi lần cập nhật/heartbeat |
| Transport → ESP parser | Quan sát `rc_seq` trong telemetry | `rc_seq` tăng liên tục khi có heartbeat, không tăng ở `NO LINK` hoặc gói sai format |
| ESP → Setpoint | So sánh Web `thrPct` và telemetry `sp_t` | Giá trị khớp sau giới hạn ga; ví dụ 25% input → 25% `sp_t` |
| Controller → PWM | So telemetry `u[4]`, `us[4]`, đo GPIO 7/6/5/4 | `<2%` giữ idle 1050 µs; ga >=5% làm target và pulse đổi tương ứng; disarm về 1000 µs |
| Link loss | Ngắt Wi-Fi/WebSocket khi ARMED | UI báo mất link; ESP timeout sang failsafe; không tự ARM lại |

## Bộ log bắt buộc khi bàn giao

Ghi cùng thời điểm: `gamepad id`, axes/buttons gốc, `thrPct`, trạng thái transport, chuỗi RC đã gửi, kết quả send, `rc_seq`, `sp_t`, `state`, `u1..u4`, `us1..us4`, `pwm_ok`. Nếu dừng ở một chặng, sửa đúng chặng đó trước; không chỉnh PID/mixer để chữa lỗi truyền throttle.

## Kiểm tra bằng mã nguồn hiện có

- Web: `tools/hitl_monitor_web/monitor_Web_resource/js/dashboard_application_controller.js` — Gamepad API, mapping và phát RC.
- Web transport/telemetry: `tools/hitl_monitor_web/monitor_Web_resource/js/telemetry_data_store.js`.
- ESP parser/setpoint/flight loop: `firmware/dual_imu_esp32c6/dual_imu_esp32c6.ino`.
- WebSocket framing/telemetry JSON: `firmware/dual_imu_esp32c6/telemetry_wifi.cpp` và `.h`.
- Mapping mixer/PWM: `firmware/dual_imu_esp32c6/px4_pid.cpp`, `esc_pwm.cpp`, `esc_pwm.h`.

Tài liệu này nêu chẩn đoán tĩnh từ source; hoàn tất sửa chỉ khi đạt các bước kiểm chứng trên máy tính và đo được PWM thực trên ESP32.
