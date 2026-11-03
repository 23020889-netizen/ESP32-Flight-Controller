#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <Wire.h>

// Cấu trúc chứa dữ liệu đo từ MPU-6050
struct MpuData {
    float roll;     // Góc Roll (độ)
    float pitch;    // Góc Pitch (độ)
    float yaw;      // Góc Yaw (độ, tích phân Gyro Z)
    float ax, ay, az; // Gia tốc ba trục (m/s^2)
    float gx, gy, gz; // Vận tốc góc ba trục (deg/s)
    bool ready;
};

// Khởi tạo bus I2C và đánh thức cảm biến MPU-6050
bool mpu6050_init(uint8_t sda_pin = 14, uint8_t scl_pin = 15, uint32_t freq = 100000);

// Đo và tự động khử Zero-Bias tĩnh của Gyroscope
void mpu6050_calibrate_gyro(uint16_t sample_count = 100);

// Đọc toàn bộ 14 thanh ghi đo của MPU-6050 và cập nhật góc Roll/Pitch
bool mpu6050_read(MpuData &data);

// Thiết lập góc Yaw mốc (ví dụ đồng bộ theo Xsens)
void mpu6050_set_yaw(float initial_yaw);

#endif // MPU6050_H
