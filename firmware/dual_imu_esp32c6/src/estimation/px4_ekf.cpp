#include "px4_ekf.h"
#include <math.h>

static const float DEG_TO_RAD_F = 0.0174532925f;
static const float RAD_TO_DEG_F = 57.2957795f;
static const float GRAVITY = 9.80665f;
static const float PI_F = 3.14159265358979323846f;

PX4_EKF::PX4_EKF() {
    state_q[0] = 1.0f; state_q[1] = 0.0f; state_q[2] = 0.0f; state_q[3] = 0.0f;
    covP[0] = 0.01f; covP[1] = 0.01f; covP[2] = 0.01f; // Covariance ban đầu
    covQ[0] = 0.001f; covQ[1] = 0.001f; covQ[2] = 0.001f; // Nhiễu quá trình

    covR_xsens = 0.005f; // Xsens la cam bien cong nghiep cao cap -> He so tin cay rat cao
    covR_mpu   = 0.080f; // MPU6050 tieu dung -> Nhieu lon hon
    covR_acc   = 0.050f; // Nhieu gia toc trong truong

    weight_xsens = 0.85f;  // Trong so dung hop Gyro/Accel: Xsens 85%
    weight_mpu   = 0.15f;  // MPU6050 15%

    smooth_roll  = 0.0f;
    smooth_pitch = 0.0f;
    smooth_yaw   = 0.0f;

    estimated_vz = 0.0f;
    estimated_z  = 0.0f;

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

    smooth_roll  = init_roll;
    smooth_pitch = init_pitch;
    smooth_yaw   = init_yaw;
    estimated_vz = 0.0f;
    estimated_z  = 0.0f;

    last_time_us = micros();
    is_initialized = true;
    Serial.printf("[PX4_EKF] Khoi tao EKF thanh cong! R=%.1f, P=%.1f, Y=%.1f\n",
                  init_roll, init_pitch, init_yaw);
}

void PX4_EKF::resetAltitude() {
    estimated_vz = 0.0f;
    estimated_z  = 0.0f;
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
    r = smooth_roll;
    p = smooth_pitch;
    y = smooth_yaw;
}

void PX4_EKF::update(const MpuData &mpu, const XsensData &xsens, FusedData &fused) {
    uint32_t nowUs = micros();
    if (!is_initialized) {
        if (xsens.has_orientation) {
            init(xsens.roll, xsens.pitch, xsens.yaw);
        } else if (mpu.ready) {
            init(mpu.roll, mpu.pitch, 0.0f);
        } else {
            return;
        }
    }

    float dt = (nowUs - last_time_us) * 1e-6f;
    last_time_us = nowUs;
    if (dt <= 0.0f || dt > 0.1f) dt = 0.005f;

    // ========================================================================
    // BƯỚC 1: DỰ BÁO ĐỘNG HỌC (PREDICTION / TIME PROPAGATION)
    // ========================================================================
    float gx_fused = 0.0f, gy_fused = 0.0f, gz_fused = 0.0f;
    bool xsens_alive = (millis() - xsens.last_packet_ms < 300) && xsens.has_gyro;

    if (xsens_alive && mpu.ready) {
        // Dung hop van toc goc ca 2 cam bien (Xsens 85%, MPU 15%)
        gx_fused = weight_xsens * xsens.gx + weight_mpu * mpu.gx;
        gy_fused = weight_xsens * xsens.gy + weight_mpu * mpu.gy;
        gz_fused = weight_xsens * xsens.gz + weight_mpu * mpu.gz;
    } else if (xsens_alive) {
        gx_fused = xsens.gx; gy_fused = xsens.gy; gz_fused = xsens.gz;
    } else if (mpu.ready) {
        gx_fused = mpu.gx; gy_fused = mpu.gy; gz_fused = mpu.gz;
    }

    float wx = gx_fused * DEG_TO_RAD_F;
    float wy = gy_fused * DEG_TO_RAD_F;
    float wz = gz_fused * DEG_TO_RAD_F;

    float q0 = state_q[0], q1 = state_q[1], q2 = state_q[2], q3 = state_q[3];
    state_q[0] += 0.5f * dt * (-q1 * wx - q2 * wy - q3 * wz);
    state_q[1] += 0.5f * dt * ( q0 * wx + q2 * wz - q3 * wy);
    state_q[2] += 0.5f * dt * ( q0 * wy - q1 * wz + q3 * wx);
    state_q[3] += 0.5f * dt * ( q0 * wz + q1 * wy - q2 * wx);
    normalizeQuaternion();

    covP[0] += covQ[0] * dt;
    covP[1] += covQ[1] * dt;
    covP[2] += covQ[2] * dt;

    // ========================================================================
    // BƯỚC 2: CẬP NHẬT ĐO LƯỜNG (MEASUREMENT UPDATE)
    // ========================================================================
    float current_r, current_p, current_y;
    quaternionToEuler(current_r, current_p, current_y);

    float delta_theta[3] = {0.0f, 0.0f, 0.0f};

    if (xsens_alive && xsens.has_orientation) {
        float err_r = (xsens.roll  - current_r) * DEG_TO_RAD_F;
        float err_p = (xsens.pitch - current_p) * DEG_TO_RAD_F;
        float err_y = (xsens.yaw   - current_y) * DEG_TO_RAD_F;

        while (err_y >  PI_F) err_y -= (2.0f * PI_F);
        while (err_y < -PI_F) err_y += (2.0f * PI_F);

        float K_r = covP[0] / (covP[0] + covR_xsens);
        float K_p = covP[1] / (covP[1] + covR_xsens);
        float K_y = covP[2] / (covP[2] + 0.002f);

        delta_theta[0] += K_r * err_r;
        delta_theta[1] += K_p * err_p;
        delta_theta[2] += K_y * err_y;

        covP[0] = (1.0f - K_r) * covP[0];
        covP[1] = (1.0f - K_p) * covP[1];
        covP[2] = (1.0f - K_y) * covP[2];
    }

    float ax_fused = (xsens_alive && mpu.ready) ? (weight_xsens * xsens.ax + weight_mpu * mpu.ax) : (xsens_alive ? xsens.ax : mpu.ax);
    float ay_fused = (xsens_alive && mpu.ready) ? (weight_xsens * xsens.ay + weight_mpu * mpu.ay) : (xsens_alive ? xsens.ay : mpu.ay);
    float az_fused = (xsens_alive && mpu.ready) ? (weight_xsens * xsens.az + weight_mpu * mpu.az) : (xsens_alive ? xsens.az : mpu.az);

    float acc_norm = sqrtf(ax_fused * ax_fused + ay_fused * ay_fused + az_fused * az_fused);
    if (acc_norm > 0.8f * GRAVITY && acc_norm < 1.2f * GRAVITY) {
        float rollAcc  = atan2f(ay_fused, az_fused) * RAD_TO_DEG_F;
        float pitchAcc = atan2f(-ax_fused, sqrtf(ay_fused * ay_fused + az_fused * az_fused)) * RAD_TO_DEG_F;

        float err_r_acc = (rollAcc  - current_r) * DEG_TO_RAD_F;
        float err_p_acc = (pitchAcc - current_p) * DEG_TO_RAD_F;

        float K_acc_r = covP[0] / (covP[0] + covR_acc);
        float K_acc_p = covP[1] / (covP[1] + covR_acc);

        delta_theta[0] += K_acc_r * err_r_acc * 0.05f;
        delta_theta[1] += K_acc_p * err_p_acc * 0.05f;

        covP[0] = (1.0f - K_acc_r * 0.05f) * covP[0];
        covP[1] = (1.0f - K_acc_p * 0.05f) * covP[1];
    }

    // Hiệu chỉnh quaternion
    float dq[4] = { 1.0f, 0.5f * delta_theta[0], 0.5f * delta_theta[1], 0.5f * delta_theta[2] };
    float nq0 = state_q[0] * dq[0] - state_q[1] * dq[1] - state_q[2] * dq[2] - state_q[3] * dq[3];
    float nq1 = state_q[0] * dq[1] + state_q[1] * dq[0] + state_q[2] * dq[3] - state_q[3] * dq[2];
    float nq2 = state_q[0] * dq[2] - state_q[1] * dq[3] + state_q[2] * dq[0] + state_q[3] * dq[1];
    float nq3 = state_q[0] * dq[3] + state_q[1] * dq[2] - state_q[2] * dq[1] + state_q[3] * dq[0];

    state_q[0] = nq0; state_q[1] = nq1; state_q[2] = nq2; state_q[3] = nq3;
    normalizeQuaternion();

    // ========================================================================
    // BƯỚC 3: TRÍCH XUẤT GÓC EULER TỪ TRẠNG THÁI QUATERNION EKF (ZERO-LAG ESTIMATOR)
    // ========================================================================
    // Lấy góc trực tiếp từ Quaternion state_q được EKF cập nhật liên tục ở 200Hz
    float ekf_r, ekf_p, ekf_y;
    quaternionToEuler(ekf_r, ekf_p, ekf_y);

    // Bộ lọc thích nghi trơn tru liên tục (Smooth Continuous Adaptive Tracking):
    // Tránh nhảy bậc (hard-switch discontinuity) tại ngưỡng vận tốc góc.
    // Khi tĩnh (gyroMag -> 0): lpf_alpha -> 0.08 (hấp thụ vi rung cơ học)
    // Khi quay nhanh (gyroMag >= 10 deg/s): lpf_alpha -> 0 (bám sát tức thời Quaternion EKF 200 Hz)
    float gyroMag = sqrtf(gx_fused * gx_fused + gy_fused * gy_fused + gz_fused * gz_fused);
    const float lpf_alpha = 0.08f / (1.0f + (gyroMag * gyroMag * 0.01f));

    smooth_roll  = lpf_alpha * smooth_roll  + (1.0f - lpf_alpha) * ekf_r;
    smooth_pitch = lpf_alpha * smooth_pitch + (1.0f - lpf_alpha) * ekf_p;

    float dy = ekf_y - smooth_yaw;
    while (dy > 180.0f)  dy -= 360.0f;
    while (dy < -180.0f) dy += 360.0f;
    smooth_yaw += (1.0f - lpf_alpha) * dy;
    if (smooth_yaw > 180.0f)  smooth_yaw -= 360.0f;
    if (smooth_yaw < -180.0f) smooth_yaw += 360.0f;

    // ========================================================================
    // BƯỚC 4: ƯỚC LƯỢNG ĐỘ CAO Z VÀ VẬN TỐC THẲNG ĐỨNG VZ (STRAPDOWN INERTIAL ESTIMATOR)
    // ========================================================================
    float r_rad = smooth_roll * DEG_TO_RAD_F;
    float p_rad = smooth_pitch * DEG_TO_RAD_F;
    float cosR = cosf(r_rad), sinR = sinf(r_rad);
    float cosP = cosf(p_rad), sinP = sinf(p_rad);

    // Gia tốc thẳng đứng trong hệ quy chiếu mặt đất (Earth frame Z)
    float az_world = -ax_fused * sinP + ay_fused * sinR * cosP + az_fused * cosR * cosP - GRAVITY;

    // Deadband gia tốc thẳng đứng để chống trôi khi đặt yên tĩnh trên bàn
    if (fabsf(az_world) < 0.12f) {
        az_world = 0.0f;
    }

    // Tích phân vận tốc thẳng đứng có cản rò (leaky velocity integration chống trôi dài hạn)
    estimated_vz += (az_world - 0.06f * estimated_vz) * dt;
    estimated_z  += estimated_vz * dt;

    if (estimated_z < 0.0f) {
        estimated_z = 0.0f;
        if (estimated_vz < 0.0f) estimated_vz = 0.0f;
    }
    if (estimated_z > 25.0f) estimated_z = 25.0f;

    // Xuất kết quả
    fused.roll       = smooth_roll;
    fused.pitch      = smooth_pitch;
    fused.yaw        = smooth_yaw;
    fused.altitudeZ  = estimated_z;
    fused.velocityVz = estimated_vz;

    fused.q[0] = state_q[0]; fused.q[1] = state_q[1]; fused.q[2] = state_q[2]; fused.q[3] = state_q[3];
    fused.ax = ax_fused; fused.ay = ay_fused; fused.az = az_fused;
    fused.gx = gx_fused; fused.gy = gy_fused; fused.gz = gz_fused;
}
