#ifndef PX4_EKF_H
#define PX4_EKF_H

#include <Arduino.h>
#include "../sensors/mpu6050.h"
#include "../sensors/xsens_mti.h"

// Cấu trúc chứa dữ liệu dung hợp chuẩn EKF
struct FusedData {
    float roll;       // Góc Roll tối ưu (độ)
    float pitch;      // Góc Pitch tối ưu (độ)
    float yaw;        // Góc Yaw tối ưu (độ)
    float q[4];       // Quaternion trạng thái [q0, q1, q2, q3]
    float ax, ay, az; // Gia tốc dung hợp hệ thân (m/s^2)
    float gx, gy, gz; // Vận tốc góc dung hợp (deg/s)
    float altitudeZ;  // Độ cao ước tính quán tính (m)
    float velocityVz; // Vận tốc thẳng đứng ước tính (m/s)
};

class PX4_EKF {
public:
    PX4_EKF();

    // Khởi tạo bộ lọc với góc ban đầu (từ Xsens)
    void init(float init_roll = 0.0f, float init_pitch = 0.0f, float init_yaw = 0.0f);

    // Cấu hình trọng số tin cậy (Hệ số Xsens mặc định 0.85 > MPU6050 0.15)
    void setSensorWeights(float xsens_weight = 0.85f, float mpu_weight = 0.15f);

    // Bước dự báo (Predict) & cập nhật (Update) EKF dung hợp cả 2 cảm biến
    void update(const MpuData &mpu, const XsensData &xsens, FusedData &fused);

    // Lấy góc Euler hiện tại
    void getEuler(float &r, float &p, float &y) const;

    // Reset ước tính độ cao
    void resetAltitude();

private:
    float state_q[4];      // State Quaternion [q0, q1, q2, q3]
    float covP[3];         // Error covariance diagonal for roll, pitch, yaw [rad^2]
    float covQ[3];         // Process noise covariance (Gyro drift)
    float covR_acc;        // Measurement noise covariance (Gravity vector)
    float covR_xsens;      // Measurement noise covariance Xsens
    float covR_mpu;        // Measurement noise covariance MPU

    float weight_xsens;    // Trọng số Xsens (mặc định 0.85)
    float weight_mpu;      // Trọng số MPU (mặc định 0.15)

    // Bộ lọc làm mịn góc mượt mà chống rung động cơ
    float smooth_roll;
    float smooth_pitch;
    float smooth_yaw;

    // Ước lượng độ cao & vận tốc thẳng đứng
    float estimated_vz;
    float estimated_z;

    uint32_t last_time_us;
    bool is_initialized;

    void normalizeQuaternion();
    void quaternionToEuler(float &roll, float &pitch, float &yaw) const;
    void eulerToQuaternion(float roll, float pitch, float yaw, float *q);
};

#endif // PX4_EKF_H
