#ifndef TELEMETRY_WIFI_H
#define TELEMETRY_WIFI_H

#include <Arduino.h>
#include "mpu6050.h"
#include "xsens_mti.h"

/**
 * @brief Cấu trúc dữ liệu phản hồi từ Máy Tính Chủ (Host PC EKF2 & Gamepad Controller)
 * Máy tính tính toán EKF2 và đọc tay cầm, gửi ngược về cho ESP32.
 */
struct HostFeedback {
    // 1. Kết quả ước lượng EKF2 từ máy tính
    float roll;             // Góc Roll ước lượng (độ)
    float pitch;            // Góc Pitch ước lượng (độ)
    float yaw;              // Góc Yaw ước lượng (độ)
    float gx, gy, gz;       // Vận tốc góc đã khử bias (deg/s)

    // 2. Lệnh điều khiển Setpoint từ tay cầm (A124 / Xbox)
    float throttle;         // Mức ga mục tiêu (0.0% .. 100.0%)
    float roll_sp;          // Góc Roll mục tiêu (-35.0° .. +35.0°)
    float pitch_sp;         // Góc Pitch mục tiêu (-35.0° .. +35.0°)
    float yaw_sp;           // Góc Yaw mục tiêu (-180.0° .. +180.0°)
    int   arm;              // 1 = ARMED, 0 = DISARMED

    uint32_t last_packet_ms;// Thời điểm nhận gói tin cuối cùng (millis)
    bool  has_ekf;          // Đã có dữ liệu EKF2 hợp lệ từ máy tính
    bool  has_rc;           // Đã có lệnh tay cầm từ máy tính
};

/**
 * @brief Cấu trúc dữ liệu Telemetry tổng hợp cho máy tính mặt đất và giao diện Web 3D
 */
struct TelemetryPacket {
    float roll;             // Góc Roll (độ)
    float pitch;            // Góc Pitch (độ)
    float yaw;              // Góc Yaw (độ)
    float ax, ay, az;       // Gia tốc 3 trục (m/s^2)
    float gx, gy, gz;       // Vận tốc góc 3 trục (deg/s)
    float z;                // Độ cao ước tính (mét)
    float vz;               // Vận tốc thẳng đứng (m/s)
    uint16_t m_us[4];       // Độ rộng xung 4 motor (µs)
    float m_pct[4];         // Tỷ lệ ga 4 motor (0 - 100%)
    const char* state_str;  // Trạng thái ESC: DISARMED, ARMED, FAILSAFE
    uint32_t frequency_hz;  // Tần số xung PWM (Hz)
};

/**
 * @brief Khởi tạo hệ thống truyền thông không dây WiFi & Telemetry trên ESP32-C6
 * Hỗ trợ chế độ kép AP_STA: Phát SoftAP "ESP32-UAV-FC" VÀ tự động kết nối Router nếu có.
 * @param ssid Tên mạng WiFi phát ra từ ESP32-C6 (mặc định: "ESP32-UAV-FC")
 * @param password Mật khẩu WiFi (mặc định: "dronebridge")
 * @param udpPort Cổng UDP bắn telemetry và nhận lệnh (mặc định: 14550)
 * @param wsPort Cổng WebSocket cho Web 3D Dashboard (mặc định: 81)
 */
void telemetry_wifi_init(const char* ssid = "ESP32-UAV-FC", 
                         const char* password = "dronebridge",
                         uint16_t udpPort = 14550,
                         uint16_t wsPort = 81);

/**
 * @brief Bắn luồng dữ liệu cảm biến thô (Raw MPU6050 + Raw Xsens) lên Máy Tính Chủ qua UDP (100Hz)
 * Máy tính sẽ nhận luồng này để chạy thuật toán PX4 EKF2 trên PC, giảm tải 100% cho ESP32.
 */
void telemetry_wifi_send_raw(const MpuData& mpu, const XsensData& xsens, const uint16_t m_us[4], bool is_armed);

/**
 * @brief Cập nhật nhận/phát dữ liệu không dây (UDP & WebSocket)
 * @param packet Dữ liệu Telemetry tổng hợp
 */
void telemetry_wifi_update(const TelemetryPacket& packet);

/**
 * @brief Kiểm tra và lấy dữ liệu phản hồi EKF2 & RC mới nhất từ Máy Tính Chủ
 * @param out Cấu trúc lưu trữ dữ liệu phản hồi
 * @return true nếu kết nối với máy tính còn sống (< 1500ms)
 */
bool telemetry_wifi_get_host_feedback(HostFeedback& out);

/**
 * @brief Kiểm tra máy tính chủ có đang kết nối và truyền EKF2/RC hay không
 */
bool telemetry_wifi_is_host_connected();

/**
 * @brief Kiểm tra xem có lệnh điều khiển CLI nào trong hàng đợi không (arm, disarm, pid, sp...)
 */
bool telemetry_wifi_has_command();

/**
 * @brief Lấy chuỗi lệnh điều khiển từ hàng đợi mạng không dây
 */
String telemetry_wifi_get_command();

/**
 * @brief Gửi thông điệp log / phản hồi ra các client WebSocket và UDP
 */
void telemetry_wifi_broadcast_msg(const char* msg);

#endif // TELEMETRY_WIFI_H
