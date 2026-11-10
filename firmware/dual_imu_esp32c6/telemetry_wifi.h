#ifndef TELEMETRY_WIFI_H
#define TELEMETRY_WIFI_H

#include <Arduino.h>

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
 * Hỗ trợ Dual-Mode (WIFI_AP_STA): Tự động kết nối vào mạng Wi-Fi phòng Lab "VNU-IS TTDBCLKT"
 * và đồng thời duy trì SoftAP dự phòng "ESP32-UAV-FC" (192.168.4.1).
 * 
 * @param sta_ssid Tên mạng WiFi trạm Station cần kết nối (mặc định: "VNU-IS TTDBCLKT")
 * @param sta_pass Mật khẩu WiFi trạm Station (mặc định: "Ttdbclkt9")
 * @param ap_ssid Tên mạng SoftAP phát từ ESP32-C6 (mặc định: "ESP32-UAV-FC")
 * @param ap_pass Mật khẩu SoftAP (mặc định: "dronebridge")
 * @param udpPort Cổng UDP bắn telemetry (mặc định: 14550)
 * @param wsPort Cổng WebSocket cho Web 3D Dashboard (mặc định: 81)
 */
void telemetry_wifi_init(const char* sta_ssid = "VNU-IS TTDBCLKT",
                         const char* sta_pass = "Ttdbclkt9",
                         const char* ap_ssid  = "ESP32-UAV-FC", 
                         const char* ap_pass  = "dronebridge",
                         uint16_t udpPort = 14550,
                         uint16_t wsPort  = 81);

/**
 * @brief Cập nhật và phát gói tin Telemetry ra mạng không dây (UDP & WebSocket)
 * @param packet Dữ liệu Telemetry mới nhất từ EKF và ESC
 */
void telemetry_wifi_update(const TelemetryPacket& packet);

/**
 * @brief Kiểm tra xem có lệnh điều khiển nào được gửi từ mạng không dây không (UDP / WebSocket)
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
