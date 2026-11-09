#ifndef XSENS_MTI_H
#define XSENS_MTI_H

#include <Arduino.h>

// Cấu trúc chứa dữ liệu đo từ Xsens MTi-3
struct XsensData {
    float roll;     // Góc Roll (độ, đã căn chỉnh trục)
    float pitch;    // Góc Pitch (độ, đã căn chỉnh trục)
    float yaw;      // Góc Yaw (độ, chuẩn la bàn số)
    float ax, ay, az; // Gia tốc ba trục (m/s^2, đã căn chỉnh trục)
    float gx, gy, gz; // Vận tốc góc ba trục (deg/s, đã căn chỉnh trục)
    bool has_orientation;
    bool has_accel;
    bool has_gyro;
    uint32_t last_packet_ms;
};

// Khởi tạo cổng UART1 giao tiếp Xsens MTi-3
bool xsens_mti_init(uint8_t rx_pin = 2, uint8_t tx_pin = 3, uint32_t baudrate = 115200);

// Cấu hình đổi dấu trục để khớp hệ quy chiếu với MPU-6050
void xsens_mti_set_axis_inversion(bool invert_x = true, bool invert_y = false, bool invert_z = true);

// Hàm máy trạng thái FSM đọc luồng byte UART non-blocking
bool xsens_mti_update(XsensData &data);

// Thống kê luồng truyền thông UART Xsens MTi-3
void xsens_mti_get_stats(uint32_t &validPackets, uint32_t &checksumErrors, uint32_t &bytesReceived);

#endif // XSENS_MTI_H
