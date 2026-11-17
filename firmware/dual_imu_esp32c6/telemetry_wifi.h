#ifndef TELEMETRY_WIFI_H
#define TELEMETRY_WIFI_H

#include <Arduino.h>

/**
 * @brief Cấu trúc dữ liệu Telemetry phát lên Giao diện Web 3D
 */
struct TelemetryPacket {
    // 1. Góc dung hợp tối ưu PX4 EKF (Dung hợp MPU6050 + Xsens MTi-3)
    float roll;             // Góc Roll EKF (độ)
    float pitch;            // Góc Pitch EKF (độ)
    float yaw;              // Góc Yaw EKF (độ)
    float ax, ay, az;       // Gia tốc 3 trục (m/s^2)
    float gx, gy, gz;       // Vận tốc góc 3 trục (deg/s)

    // 2. Dữ liệu thô từ từng cảm biến độc lập (Cho Web Monitor so sánh)
    float mpu_roll, mpu_pitch, mpu_yaw;    // Cảm biến MPU-6050 (I2C)
    float xs_roll,  xs_pitch,  xs_yaw;     // Cảm biến Xsens MTi-3 AHRS (UART1)

    // 3. Trạng thái bay và động cơ
    float z;                // Độ cao ước tính (mét)
    float vz;               // Vận tốc thẳng đứng (m/s)
    uint16_t m_us[4];       // Độ rộng xung 4 motor (µs)
    float m_pct[4];         // Tỷ lệ ga 4 motor (0 - 100%)
    const char* state_str;  // Trạng thái ESC: DISARMED, ARMED, FAILSAFE
    uint32_t frequency_hz;  // Tần số xung PWM (Hz)

    // 4. Setpoint và Bộ điều khiển (Chuẩn hóa theo Guide)
    float sp_roll;          // Góc Roll mục tiêu (độ)
    float sp_pitch;         // Góc Pitch mục tiêu (độ)
    float sp_yaw;           // Góc Yaw mục tiêu (độ)
    float sp_throttle;      // Mức ga mục tiêu (0 - 100%)
    float tau[3];           // Mô-men điều khiển [Roll, Pitch, Yaw]
    float u_norm[4];        // Lệnh chuẩn hóa 4 động cơ từ Allocator [0.0 .. 1.0]
    bool  yaw_valid;        // Trạng thái hợp lệ của cảm biến góc Yaw
    uint32_t rc_sequence;   // Bộ đếm gói RC hợp lệ đã được ESP chấp nhận
    bool pwm_outputs_ready; // Cả bốn PWM pin đã attach thành công
};

/**
 * @brief Khởi tạo mạng không dây WiFi trên ESP32-C6:
 * - Tự động kết nối mạng phòng Lab: "VNU-IS TTDBCLKT" (Mật khẩu: "Ttdbclkt9")
 * - Đồng thời phát SoftAP dự phòng: "ESP32-UAV-FC" (Mật khẩu: "dronebridge", IP 192.168.4.1)
 * - Mở WebSocket Server tại Port 81 cho Web Browser kết nối trực tiếp.
 */
void telemetry_wifi_init(const char* sta_ssid = "VNU-IS TTDBCLKT",
                         const char* sta_pass = "Ttdbclkt9",
                         const char* ap_ssid  = "ESP32-UAV-FC",
                         const char* ap_pass  = "dronebridge",
                         uint16_t wsPort = 81);

/**
 * @brief Phát dữ liệu Telemetry JSON định kỳ lên Web (25 Hz) và kiểm tra các Client WebSocket
 */
void telemetry_wifi_update(const TelemetryPacket& packet);

/**
 * @brief Kiểm tra có lệnh điều khiển từ Web (qua WebSocket) hoặc UDP không
 */
bool telemetry_wifi_has_command();

/**
 * @brief Lấy chuỗi lệnh từ hàng đợi
 */
String telemetry_wifi_get_command();

/**
 * @brief Gửi thông điệp log về Web Browser
 */
void telemetry_wifi_broadcast_msg(const char* msg);

/**
 * @brief Lấy địa chỉ IP hiện tại của ESP32 (ưu tiên IP mạng TTDB nếu đã kết nối)
 */
String telemetry_wifi_get_ip();

#endif // TELEMETRY_WIFI_H
