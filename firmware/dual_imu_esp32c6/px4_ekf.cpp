#include "px4_ekf.h"
#include <math.h>

static const float DEG_TO_RAD_F = 0.0174532925f;
static const float RAD_TO_DEG_F = 57.2957795f;
static const float GRAVITY = 9.80665f;
static const float PI_F = 3.14159265358979323846f;

PX4_EKF::PX4_EKF() {
    state_q[0] = 1.0f; state_q[1] = 0.0f; state_q[2] = 0.0f; state_q[3] = 0.0f;
    covP[0] = 0.01f; covP[1] = 0.01f; covP[2] = 0.01f; // Covariance ban đầu
    covQ[0] = 0.001f; covQ[1] = 0.001f; covQ[2] = 0.001f; // Nhiễu quá trình (Gyro angle random walk)

    // Ma trận nhiễu đo R:
    // Theo chuẩn PX4: R càng nhỏ thì bộ lọc càng tin tưởng cảm biến đó
    covR_xsens = 0.005f; // Xsens là cảm biến công nghiệp cao cấp -> Hệ số tin cậy rất cao
    covR_mpu   = 0.080f; // MPU6050 tiêu dùng -> Nhiễu lớn hơn
    covR_acc   = 0.050f; // Nhiễu vector gia tốc trọng trường

    weight_xsens = 0.85f;  // Trọng số dung hợp Gyro/Accel: Xsens chiếm 85%
    weight_mpu   = 0.15f;  // MPU6050 chiếm 15%

    last_time_us = 0;
    is_initialized = false;
}

void PX4_EKF::eulerToQuaternion(float roll, float pitch, float yaw, float *q) {
    float r = roll  * DEG_TO_RAD_F * 0.5f;
    float p = pitch * DEG_TO_RAD_F * 0.5f;
    float y = yaw   * DEG_TO_RAD_F * 0.5f;

    float cr = cosf(r), sr = sinf(r);
    float cp = cosf(p), sp = sinf(p);
    float cy = cosf(y), sy = sinf(y);

    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

void PX4_EKF::init(float init_roll, float init_pitch, float init_yaw) {
    eulerToQuaternion(init_roll, init_pitch, init_yaw, state_q);
    normalizeQuaternion();
    covP[0] = 0.001f; covP[1] = 0.001f; covP[2] = 0.001f;
    last_time_us = micros();
    is_initialized = true;
    Serial.printf("[PX4_EKF] Khoi tao EKF thanh cong! R=%.1f, P=%.1f, Y=%.1f\n",
                  init_roll, init_pitch, init_yaw);
}

void PX4_EKF::setSensorWeights(float xsens_weight, float mpu_weight) {
    float sum = xsens_weight + mpu_weight;
    if (sum > 0.001f) {
        weight_xsens = xsens_weight / sum;
        weight_mpu   = mpu_weight / sum;
    }
}

void PX4_EKF::normalizeQuaternion() {
    float norm = sqrtf(state_q[0] * state_q[0] + state_q[1] * state_q[1] + state_q[2] * state_q[2] + state_q[3] * state_q[3]);
    if (norm > 1e-6f) {
        float inv = 1.0f / norm;
        state_q[0] *= inv;
        state_q[1] *= inv;
        state_q[2] *= inv;
        state_q[3] *= inv;
    } else {
        state_q[0] = 1.0f; state_q[1] = 0.0f; state_q[2] = 0.0f; state_q[3] = 0.0f;
    }
}

void PX4_EKF::quaternionToEuler(float &roll, float &pitch, float &yaw) const {
    roll  = atan2f(2.0f * (state_q[0] * state_q[1] + state_q[2] * state_q[3]), 1.0f - 2.0f * (state_q[1] * state_q[1] + state_q[2] * state_q[2])) * RAD_TO_DEG_F;
    float sinP = 2.0f * (state_q[0] * state_q[2] - state_q[3] * state_q[1]);
    if (fabsf(sinP) >= 1.0f)
        pitch = copysignf(90.0f, sinP);
    else
        pitch = asinf(sinP) * RAD_TO_DEG_F;
    yaw   = atan2f(2.0f * (state_q[0] * state_q[3] + state_q[1] * state_q[2]), 1.0f - 2.0f * (state_q[2] * state_q[2] + state_q[3] * state_q[3])) * RAD_TO_DEG_F;
}

void PX4_EKF::getEuler(float &r, float &p, float &y) const {
    quaternionToEuler(r, p, y);
}

void PX4_EKF::update(const MpuData &mpu, const XsensData &xsens, FusedData &fused) {
    uint32_t nowUs = micros();
    if (!is_initialized) {
        if (xsens.has_orientation) {
            init(xsens.roll, xsens.pitch, xsens.yaw);
        } else if (mpu.ready) {
            init(mpu.roll, mpu.pitch, 0.0f); // Khong dung goc Yaw cua MPU6050
        } else {
            return;
        }
    }

    float dt = (nowUs - last_time_us) * 1e-6f;
    last_time_us = nowUs;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.005f;

    // ========================================================================
    // BƯỚC 1: DỰ BÁO ĐỘNG HỌC (PREDICTION / TIME PROPAGATION)
    // - Roll/Pitch: Dung hợp Gyro X, Y (Xsens 85%, MPU 15%)
    // - Yaw: BỎ HOÀN TOÀN MPU6050, chỉ dùng Gyro Z của Xsens MTi-3
    // ========================================================================
    float gx_fused = 0.0f, gy_fused = 0.0f, gz_fused = 0.0f;
    bool xsens_alive = (millis() - xsens.last_packet_ms < 300) && xsens.has_gyro;

    if (xsens_alive && mpu.ready) {
        gx_fused = weight_xsens * xsens.gx + weight_mpu * mpu.gx;
        gy_fused = weight_xsens * xsens.gy + weight_mpu * mpu.gy;
        // Bỏ góc Yaw của MPU-6050: 100% Gyro Z từ Xsens MTi-3
        gz_fused = xsens.gz;
    } else if (xsens_alive) {
        gx_fused = xsens.gx; gy_fused = xsens.gy; gz_fused = xsens.gz;
    } else if (mpu.ready) {
        gx_fused = mpu.gx; gy_fused = mpu.gy; gz_fused = 0.0f;
    }

    // Chuyển sang radian/giây cho động học Quaternion
    float wx = gx_fused * DEG_TO_RAD_F;
    float wy = gy_fused * DEG_TO_RAD_F;
    float wz = gz_fused * DEG_TO_RAD_F;

    // Tích phân Quaternion động học: dq/dt = 0.5 * q * w
    float q0 = state_q[0], q1 = state_q[1], q2 = state_q[2], q3 = state_q[3];
    state_q[0] += 0.5f * dt * (-q1 * wx - q2 * wy - q3 * wz);
    state_q[1] += 0.5f * dt * ( q0 * wx + q2 * wz - q3 * wy);
    state_q[2] += 0.5f * dt * ( q0 * wy - q1 * wz + q3 * wx);
    state_q[3] += 0.5f * dt * ( q0 * wz + q1 * wy - q2 * wx);
    normalizeQuaternion();

    // Lan truyền hiệp phương sai sai số P = P + Q * dt
    covP[0] += covQ[0] * dt;
    covP[1] += covQ[1] * dt;
    covP[2] += covQ[2] * dt;

    // ========================================================================
    // BƯỚC 2: CẬP NHẬT ĐO LƯỜNG (MEASUREMENT UPDATE) CHUẨN PX4
    // ========================================================================
    float current_r, current_p, current_y;
    quaternionToEuler(current_r, current_p, current_y);

    float delta_theta[3] = {0.0f, 0.0f, 0.0f};

    // --- Cập nhật 1: Quan sát góc từ Xsens MTi-3 (Trọng số cao, R_xsens rất nhỏ) ---
    if (xsens_alive && xsens.has_orientation) {
        // Sai số góc (Innovation) giữa Xsens và trạng thái EKF hiện tại
        float err_r = (xsens.roll  - current_r) * DEG_TO_RAD_F;
        float err_p = (xsens.pitch - current_p) * DEG_TO_RAD_F;
        float err_y = (xsens.yaw   - current_y) * DEG_TO_RAD_F;

        // Chuẩn hóa sai số góc Yaw trong khoảng [-pi, +pi]
        while (err_y >  PI_F) err_y -= (2.0f * PI_F);
        while (err_y < -PI_F) err_y += (2.0f * PI_F);

        // Kalman Gain từng trục: K = P / (P + R)
        float K_r = covP[0] / (covP[0] + covR_xsens);
        float K_p = covP[1] / (covP[1] + covR_xsens);
        float K_y = covP[2] / (covP[2] + 0.002f); // R_yaw rất nhỏ -> Bám 100% Xsens Yaw

        // Cập nhật góc hiệu chỉnh
        delta_theta[0] += K_r * err_r;
        delta_theta[1] += K_p * err_p;
        delta_theta[2] += K_y * err_y;

        // Cập nhật hiệp phương sai: P = (1 - K) * P
        covP[0] = (1.0f - K_r) * covP[0];
        covP[1] = (1.0f - K_p) * covP[1];
        covP[2] = (1.0f - K_y) * covP[2];
    }

    // --- Cập nhật 2: Bù gia tốc trọng trường (Gravity Fusion) từ 2 Accelerometer ---
    float ax_fused = (xsens_alive && mpu.ready) ? (weight_xsens * xsens.ax + weight_mpu * mpu.ax) : (xsens_alive ? xsens.ax : mpu.ax);
    float ay_fused = (xsens_alive && mpu.ready) ? (weight_xsens * xsens.ay + weight_mpu * mpu.ay) : (xsens_alive ? xsens.ay : mpu.ay);
    float az_fused = (xsens_alive && mpu.ready) ? (weight_xsens * xsens.az + weight_mpu * mpu.az) : (xsens_alive ? xsens.az : mpu.az);

    float acc_norm = sqrtf(ax_fused * ax_fused + ay_fused * ay_fused + az_fused * az_fused);

    // Chuẩn PX4 Innovation Gating: Chỉ dùng Accel khi tổng gia tốc nằm trong ngưỡng 0.8g -> 1.2g
    // (Bảo vệ EKF không bị méo góc khi rung động mạnh hoặc chuyển động gia tốc tịnh tiến)
    if (acc_norm > 0.8f * GRAVITY && acc_norm < 1.2f * GRAVITY) {
        float rollAcc  = atan2f(ay_fused, az_fused) * RAD_TO_DEG_F;
        float pitchAcc = atan2f(-ax_fused, sqrtf(ay_fused * ay_fused + az_fused * az_fused)) * RAD_TO_DEG_F;

        float err_r_acc = (rollAcc  - current_r) * DEG_TO_RAD_F;
        float err_p_acc = (pitchAcc - current_p) * DEG_TO_RAD_F;

        float K_acc_r = covP[0] / (covP[0] + covR_acc);
        float K_acc_p = covP[1] / (covP[1] + covR_acc);

        delta_theta[0] += K_acc_r * err_r_acc * 0.1f; // Tác động gia tốc làm mịn thêm
        delta_theta[1] += K_acc_p * err_p_acc * 0.1f;

        covP[0] = (1.0f - K_acc_r * 0.1f) * covP[0];
        covP[1] = (1.0f - K_acc_p * 0.1f) * covP[1];
    }

    // ========================================================================
    // BƯỚC 3: ÁP DỤNG HIỆU CHỈNH LÊN QUATERNION & XUẤT KẾT QUẢ
    // ========================================================================
    // Hiệu chỉnh quaternion: q = q * delta_q
    float dq[4];
    dq[0] = 1.0f;
    dq[1] = 0.5f * delta_theta[0];
    dq[2] = 0.5f * delta_theta[1];
    dq[3] = 0.5f * delta_theta[2];

    float nq0 = state_q[0] * dq[0] - state_q[1] * dq[1] - state_q[2] * dq[2] - state_q[3] * dq[3];
    float nq1 = state_q[0] * dq[1] + state_q[1] * dq[0] + state_q[2] * dq[3] - state_q[3] * dq[2];
    float nq2 = state_q[0] * dq[2] - state_q[1] * dq[3] + state_q[2] * dq[0] + state_q[3] * dq[1];
    float nq3 = state_q[0] * dq[3] + state_q[1] * dq[2] - state_q[2] * dq[1] + state_q[3] * dq[0];

    state_q[0] = nq0; state_q[1] = nq1; state_q[2] = nq2; state_q[3] = nq3;
    normalizeQuaternion();

    // Xuất dữ liệu dung hợp
    quaternionToEuler(fused.roll, fused.pitch, fused.yaw);
    // Bỏ góc Yaw của MPU6050: Góc Yaw đầu ra của bộ lọc EKF luôn luôn lấy 100% từ Xsens MTi-3
    if (xsens_alive && xsens.has_orientation) {
        fused.yaw = xsens.yaw;
    }

    fused.q[0] = state_q[0]; fused.q[1] = state_q[1]; fused.q[2] = state_q[2]; fused.q[3] = state_q[3];
    fused.ax = ax_fused; fused.ay = ay_fused; fused.az = az_fused;
    fused.gx = gx_fused; fused.gy = gy_fused; fused.gz = gz_fused;
}
