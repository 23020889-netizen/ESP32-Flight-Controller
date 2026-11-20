#include "mpu6050.h"
#include <math.h>

#define MPU_ADDR 0x68

static const float GRAVITY = 9.80665f;
static const float RAD_TO_DEG_F = 57.2957795f;

// Dải đo: Gyro ±1000 dps (32.8 LSB/dps), Accel ±8 g (4096 LSB/g).
// Bản cũ dùng ±250 dps / ±2 g: Gyro bão hòa ở 250 dps trong khi vòng ngoài cho phép
// lệnh tới 220 dps (+ overshoot), và Accel ±2 g bị rung động cơ làm bão hòa
// => góc nghiêng từ gia tốc sai, vòng trong mất phản hồi khi quay nhanh.
static const float GYRO_LSB_PER_DPS = 32.8f;
static const float ACC_LSB_PER_G    = 4096.0f;

static float mpu_gx_bias = 0.0f;
static float mpu_gy_bias = 0.0f;
static float mpu_gz_bias = 0.0f;

static float current_roll = 0.0f;
static float current_pitch = 0.0f;
static float current_yaw = 0.0f;
static uint32_t last_update_us = 0;
static bool is_ready = false;

static bool writeReg(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

static bool readReg(uint8_t reg, uint8_t &val) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission() != 0) return false;
    if (Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1) != 1) return false;
    val = Wire.read();
    return true;
}

bool mpu6050_init(uint8_t sda_pin, uint8_t scl_pin, uint32_t freq) {
    Wire.begin(sda_pin, scl_pin);
    Wire.setClock(freq);
    delay(50);

    uint8_t who = 0;
    if (!readReg(0x75, who)) {
        Serial.println("[MPU6050] Khong tim thay thiet bi qua I2C!");
        is_ready = false;
        return false;
    }

    if (who != 0x68 && who != 0x69 && who != 0x70 && who != 0x72) {
        Serial.printf("[MPU6050] WHO_AM_I khong hop le: 0x%02X\n", who);
        is_ready = false;
        return false;
    }

    // Wake up MPU, chon clock PLL X
    writeReg(0x6B, 0x01);
    delay(10);

    // Accel +/- 8g
    writeReg(0x1C, 0x10);
    // Gyro +/- 1000 deg/s
    writeReg(0x1B, 0x10);
    // DLPF 42Hz Bandwidth
    writeReg(0x1A, 0x03);
    // Sample Rate Divider (200Hz)
    writeReg(0x19, 0x04);

    delay(20);
    is_ready = true;
    Serial.printf("[MPU6050] Khoi tao thanh cong! (WHO_AM_I = 0x%02X)\n", who);
    return true;
}

void mpu6050_calibrate_gyro(uint16_t sample_count) {
    if (!is_ready) return;
    Serial.println("[MPU6050] Dang lay mau do Zero-Bias Gyro (giu yen mach 0.5s)...");

    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;
    int count = 0;

    for (uint16_t i = 0; i < sample_count; i++) {
        Wire.beginTransmission(MPU_ADDR);
        Wire.write(0x43);
        if (Wire.endTransmission(false) == 0 && Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)6) == 6) {
            int16_t raw_gx = (int16_t)((Wire.read() << 8) | Wire.read());
            int16_t raw_gy = (int16_t)((Wire.read() << 8) | Wire.read());
            int16_t raw_gz = (int16_t)((Wire.read() << 8) | Wire.read());

            sum_x += (raw_gx / GYRO_LSB_PER_DPS);
            sum_y += (raw_gy / GYRO_LSB_PER_DPS);
            sum_z += (raw_gz / GYRO_LSB_PER_DPS);
            count++;
        }
        delay(5);
    }

    if (count > 0) {
        mpu_gx_bias = sum_x / count;
        mpu_gy_bias = sum_y / count;
        mpu_gz_bias = sum_z / count;
        Serial.printf("[MPU6050] Gyro Bias da luu: GX=%.2f, GY=%.2f, GZ=%.2f deg/s\n",
                      mpu_gx_bias, mpu_gy_bias, mpu_gz_bias);
    }
}

void mpu6050_set_yaw(float initial_yaw) {
    current_yaw = initial_yaw;
}

bool mpu6050_read(MpuData &data) {
    if (!is_ready) {
        data.ready = false;
        return false;
    }

    // Lỗi I2C liên tiếp => đánh dấu mất cảm biến (bản cũ giữ data.ready=true với dữ liệu cũ mãi mãi)
    static uint8_t fail_count = 0;
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    if (Wire.endTransmission(false) != 0 ||
        Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14) != 14) {
        if (fail_count < 255) fail_count++;
        if (fail_count >= 10) data.ready = false;
        return false;
    }
    fail_count = 0;

    uint8_t buf[14];
    for (int i = 0; i < 14; i++) {
        buf[i] = Wire.read();
    }

    int16_t raw_ax = (int16_t)((buf[0]  << 8) | buf[1]);
    int16_t raw_ay = (int16_t)((buf[2]  << 8) | buf[3]);
    int16_t raw_az = (int16_t)((buf[4]  << 8) | buf[5]);

    int16_t raw_gx = (int16_t)((buf[8]  << 8) | buf[9]);
    int16_t raw_gy = (int16_t)((buf[10] << 8) | buf[11]);
    int16_t raw_gz = (int16_t)((buf[12] << 8) | buf[13]);

    data.ax = (raw_ax / ACC_LSB_PER_G) * GRAVITY;
    data.ay = (raw_ay / ACC_LSB_PER_G) * GRAVITY;
    data.az = (raw_az / ACC_LSB_PER_G) * GRAVITY;

    data.gx = (raw_gx / GYRO_LSB_PER_DPS) - mpu_gx_bias;
    data.gy = (raw_gy / GYRO_LSB_PER_DPS) - mpu_gy_bias;
    data.gz = (raw_gz / GYRO_LSB_PER_DPS) - mpu_gz_bias;

    // Tinh Roll & Pitch tu gia toc
    float rollAcc  = atan2f(data.ay, data.az) * RAD_TO_DEG_F;
    float pitchAcc = atan2f(-data.ax, sqrtf(data.ay * data.ay + data.az * data.az)) * RAD_TO_DEG_F;

    uint32_t nowUs = micros();
    if (last_update_us == 0) {
        current_roll  = rollAcc;
        current_pitch = pitchAcc;
        last_update_us = nowUs;
    } else {
        float dt = (nowUs - last_update_us) * 1e-6f;
        last_update_us = nowUs;
        if (dt <= 0.0f || dt > 0.1f) dt = 0.005f;

        // Chỉ tin gia tốc khi |a| gần 1 g (không bị rung/gia tốc tuyến tính lớn)
        const float a_norm = sqrtf(data.ax * data.ax + data.ay * data.ay + data.az * data.az);
        const bool acc_trust = fabsf(a_norm - GRAVITY) < 0.25f * GRAVITY;
        // Hằng số thời gian 0.5 s, độc lập với tần số gọi hàm (loop() chạy nhanh hơn 200 Hz)
        const float tau_cf = 0.5f;
        const float alpha = acc_trust ? (tau_cf / (tau_cf + dt)) : 1.0f;
        current_roll  = alpha * (current_roll  + data.gx * dt) + (1.0f - alpha) * rollAcc;
        current_pitch = alpha * (current_pitch + data.gy * dt) + (1.0f - alpha) * pitchAcc;
        if (current_roll > 180.0f)  current_roll -= 360.0f;
        if (current_roll < -180.0f) current_roll += 360.0f;
        current_yaw  += data.gz * dt;

        if (current_yaw > 180.0f)  current_yaw -= 360.0f;
        if (current_yaw < -180.0f) current_yaw += 360.0f;
    }

    data.roll  = current_roll;
    data.pitch = current_pitch;
    data.yaw   = current_yaw;
    data.ready = true;

    return true;
}
