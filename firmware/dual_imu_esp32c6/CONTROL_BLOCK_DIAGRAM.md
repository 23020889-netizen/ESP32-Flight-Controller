# Sơ đồ khối điều khiển và file C++ tương ứng

`dual_imu_esp32c6.ino` là entry point và nơi ghép các khối. Vòng điều khiển chính chạy ở 200 Hz; nhận lệnh mạng/Serial, phát telemetry và các tác vụ giao tiếp được gọi trong cùng `loop()`.

```mermaid
flowchart LR
    subgraph INPUT["Lệnh điều khiển"]
        WEB["Web / Xbox gamepad<br/>RC,thr,roll,pitch,yaw,arm"]
        SERIAL["Serial CLI"]
        NET["Wi-Fi WebSocket / UDP"]
    end

    subgraph COMMAND["Nhận lệnh, setpoint và an toàn<br/>dual_imu_esp32c6.ino"]
        RX["handleWirelessCommands<br/>handleSerialCommands<br/>executeCommand"]
        SP["SetpointState<br/>Throttle / Roll / Pitch / Yaw<br/>ARM, test motor, RC watchdog"]
    end

    subgraph SENSORS["Thu thập và tạo trạng thái"]
        MPU["MPU-6050<br/>I2C GPIO 14/15<br/>mpu6050.cpp"]
        XSENS["Xsens MTi-3<br/>UART GPIO 2/3<br/>xsens_mti.cpp"]
        EST["State estimator hiện dùng<br/>Xsens orientation + gyro trộn<br/>85% Xsens / 15% MPU<br/>dual_imu_esp32c6.ino"]
    end

    subgraph CONTROL["Điều khiển bay cascade 200 Hz<br/>px4_pid.cpp"]
        ATT["Attitude P loop<br/>góc đặt → rate setpoint"]
        RATE["Rate PID<br/>rate setpoint − gyro lọc → torque"]
        MIX["Quad-X mixer / allocator<br/>T + Roll + Pitch + Yaw → u1..u4"]
    end

    subgraph OUTPUT["Xuất ESC và motor"]
        GATE["Output safety gate<br/>ARM / idle / test motor<br/>dual_imu_esp32c6.ino"]
        ESC["PWM, slew-rate, watchdog,<br/>ARM/DISARM/CALIB FSM<br/>esc_pwm.cpp"]
        MOTORS["M1 GPIO 7 · M2 GPIO 6<br/>M3 GPIO 5 · M4 GPIO 4<br/>1000–2000 µs chung"]
    end

    subgraph TELEMETRY["Telemetry và mạng<br/>telemetry_wifi.cpp"]
        TEL["JSON: state, setpoint,<br/>u1..u4, PWM us1..us4"]
    end

    WEB --> NET --> RX --> SP
    SERIAL --> RX
    MPU --> EST
    XSENS --> EST
    SP --> ATT
    EST --> ATT
    EST --> RATE
    ATT --> RATE --> MIX
    SP --> MIX
    MIX --> GATE --> ESC --> MOTORS
    SP --> TEL
    EST --> TEL
    MIX --> TEL
    ESC --> TEL
    TEL --> NET
```

## Bảng ghép khối với source

| Khối | File thực thi | Nhiệm vụ chính |
|---|---|---|
| Entry point, lệnh, setpoint, safety gate | `dual_imu_esp32c6.ino` | `setup()`, `loop()`, parse RC/CLI, `SetpointState`, watchdog RC, gọi controller và đưa lệnh cuối tới ESC |
| Đọc MPU-6050 | `mpu6050.cpp` | I2C, hiệu chuẩn gyro, dữ liệu accelerometer/gyro, Roll/Pitch khi chạy MPU độc lập |
| Đọc Xsens MTi-3 | `xsens_mti.cpp` | UART/Xbus, orientation, acceleration và gyro từ AHRS Xsens |
| Lọc dữ liệu | `px4_filters.cpp` | Low-pass filter bậc hai dùng cho gyro/D-term trong controller |
| Vòng góc, rate PID, mixer | `px4_pid.cpp` | Attitude P loop, rate PID, anti-windup, giới hạn mô-men, Quad-X allocation/desaturation |
| PWM và ESC state machine | `esc_pwm.cpp` | LEDC PWM, dải 1000–2000 µs chung, soft ramp, ARM/DISARM, failsafe, calibration |
| Wi-Fi và WebSocket | `telemetry_wifi.cpp` | STA/AP, WebSocket server, queue lệnh mạng, UDP, telemetry JSON |

## Module có trong thư mục nhưng chưa nằm trên luồng đang chạy

`px4_ekf.cpp` có lớp `PX4_EKF`, nhưng file `.ino` hiện không include hoặc gọi `PX4_EKF::update()`. Trạng thái attitude đang dùng được dựng trực tiếp trong `loop()` từ orientation Xsens và gyro trộn. Vì vậy không nên vẽ `px4_ekf.cpp` là khối đang hoạt động cho đến khi nó được nối vào `dual_imu_esp32c6.ino`.

## Chuỗi điều khiển cần quan sát khi debug throttle

```mermaid
sequenceDiagram
    participant G as Xbox / Web
    participant N as telemetry_wifi.cpp
    participant I as dual_imu_esp32c6.ino
    participant P as px4_pid.cpp
    participant E as esc_pwm.cpp

    G->>N: RC,thr,roll,pitch,yaw,arm
    N->>I: command queue
    I->>I: SetpointState.throttle_pct = thr
    I->>P: setTargetThrottle(thr), update(...)
    P->>I: normalized u1..u4
    I->>E: setAllNormalized(u1..u4)
    E->>E: PWM target/current, soft ramp
    E-->>N: us1..us4 + motor percent
    N-->>G: JSON telemetry: rc_seq, sp_t, u[], us[]
```

Khi ga chưa tới 2%, `dual_imu_esp32c6.ino` áp dụng idle chung `1050 µs`. Khi DISARM, `esc_pwm.cpp` giữ bốn kênh tại `1000 µs`.
