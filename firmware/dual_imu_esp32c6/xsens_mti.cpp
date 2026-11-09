#include "xsens_mti.h"
#include <math.h>
#include <string.h>

static HardwareSerial XsensUart(1);

static bool opt_invert_x = true;
static bool opt_invert_y = false;
static bool opt_invert_z = true;

static XsensData current_xsens = {0};

static const float RAD_TO_DEG_F = 57.2957795f;

// Máy trạng thái FSM đọc luồng byte UART Xbus non-blocking
enum XbusState {
    XBUS_PREAMBLE,
    XBUS_BID,
    XBUS_MID,
    XBUS_LEN,
    XBUS_EXT_LEN_H,
    XBUS_EXT_LEN_L,
    XBUS_PAYLOAD,
    XBUS_CHECKSUM
};

static float readFloatBE(const uint8_t *p) {
    uint32_t u = ((uint32_t)p[0] << 24) |
                 ((uint32_t)p[1] << 16) |
                 ((uint32_t)p[2] << 8)  |
                 ((uint32_t)p[3]);
    float val;
    memcpy(&val, &u, sizeof(float));
    return val;
}

static void parseMTData2Payload(const uint8_t *payload, uint16_t len) {
    uint16_t idx = 0;
    while (idx + 3 <= len) {
        uint16_t dataId = ((uint16_t)payload[idx] << 8) | payload[idx + 1];
        uint8_t dataLen = payload[idx + 2];
        idx += 3;

        if (idx + dataLen > len) break;
        const uint8_t *data = &payload[idx];
        idx += dataLen;

        uint16_t typeId = dataId & 0xFFF0;

        // 1. Góc Euler (0x2030): Roll, Pitch, Yaw (Degrees)
        if (typeId == 0x2030 && dataLen >= 12) {
            float raw_r = readFloatBE(&data[0]);
            float raw_p = readFloatBE(&data[4]);
            float raw_y = readFloatBE(&data[8]);

            if (opt_invert_z) {
                float r_norm = (raw_r > 0.0f) ? (raw_r - 180.0f) : (raw_r + 180.0f);
                current_xsens.roll = -r_norm;
            } else {
                current_xsens.roll = raw_r;
            }

            current_xsens.pitch = (opt_invert_y ? -raw_p : raw_p);
            current_xsens.yaw   = raw_y;
            current_xsens.has_orientation = true;
            current_xsens.last_packet_ms = millis();
        }
        // 2. Quaternion (0x2010): q0, q1, q2, q3 -> Chuyển sang Euler nếu cấu hình Quat
        else if (typeId == 0x2010 && dataLen >= 16) {
            float q0 = readFloatBE(&data[0]);
            float q1 = readFloatBE(&data[4]);
            float q2 = readFloatBE(&data[8]);
            float q3 = readFloatBE(&data[12]);

            float raw_r = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2)) * RAD_TO_DEG_F;
            float sinP  = 2.0f * (q0 * q2 - q3 * q1);
            float raw_p = 0.0f;
            if (fabsf(sinP) >= 1.0f)
                raw_p = copysignf(90.0f, sinP);
            else
                raw_p = asinf(sinP) * RAD_TO_DEG_F;
            float raw_y = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3)) * RAD_TO_DEG_F;

            if (opt_invert_z) {
                float r_norm = (raw_r > 0.0f) ? (raw_r - 180.0f) : (raw_r + 180.0f);
                current_xsens.roll = -r_norm;
            } else {
                current_xsens.roll = raw_r;
            }

            current_xsens.pitch = (opt_invert_y ? -raw_p : raw_p);
            current_xsens.yaw   = raw_y;
            current_xsens.has_orientation = true;
            current_xsens.last_packet_ms = millis();
        }
        // 3. Gia tốc Calibrated Acceleration (0x4020, 0x4030, 0x4040, 0x4010) (m/s^2)
        else if ((typeId == 0x4020 || typeId == 0x4030 || typeId == 0x4040 || typeId == 0x4010) && dataLen >= 12) {
            float raw_ax = readFloatBE(&data[0]);
            float raw_ay = readFloatBE(&data[4]);
            float raw_az = readFloatBE(&data[8]);

            current_xsens.ax = (opt_invert_x ? -raw_ax : raw_ax);
            current_xsens.ay = (opt_invert_y ? -raw_ay : raw_ay);
            current_xsens.az = (opt_invert_z ? -raw_az : raw_az);
            current_xsens.has_accel = true;
            current_xsens.last_packet_ms = millis();
        }
        // 4. Vận tốc góc Calibrated Rate of Turn / Gyro (0x8020, 0x8040) (rad/s -> deg/s)
        else if ((typeId == 0x8020 || typeId == 0x8040) && dataLen >= 12) {
            float gx_rad = readFloatBE(&data[0]);
            float gy_rad = readFloatBE(&data[4]);
            float gz_rad = readFloatBE(&data[8]);

            float raw_gx = gx_rad * RAD_TO_DEG_F;
            float raw_gy = gy_rad * RAD_TO_DEG_F;
            float raw_gz = gz_rad * RAD_TO_DEG_F;

            current_xsens.gx = (opt_invert_x ? -raw_gx : raw_gx);
            current_xsens.gy = (opt_invert_y ? -raw_gy : raw_gy);
            current_xsens.gz = (opt_invert_z ? -raw_gz : raw_gz);
            current_xsens.has_gyro = true;
            current_xsens.last_packet_ms = millis();
        }
    }
}

bool xsens_mti_init(uint8_t rx_pin, uint8_t tx_pin, uint32_t baudrate) {
    XsensUart.begin(baudrate, SERIAL_8N1, rx_pin, tx_pin);
    delay(100);

    // Gửi lệnh GoToMeasurement (FA FF 10 00 F1) kích hoạt truyền MTData2
    const uint8_t goToMeasurementCmd[] = { 0xFA, 0xFF, 0x10, 0x00, 0xF1 };
    XsensUart.write(goToMeasurementCmd, sizeof(goToMeasurementCmd));
    delay(50);
    XsensUart.write(goToMeasurementCmd, sizeof(goToMeasurementCmd));

    Serial.printf("[XSENS] Da khoi tao UART1 (RX=%d, TX=%d, Baud=%d)\n", rx_pin, tx_pin, baudrate);
    return true;
}

void xsens_mti_set_axis_inversion(bool invert_x, bool invert_y, bool invert_z) {
    opt_invert_x = invert_x;
    opt_invert_y = invert_y;
    opt_invert_z = invert_z;
    Serial.printf("[XSENS] Cau hinh truc: Invert_X=%d, Invert_Y=%d, Invert_Z=%d\n",
                  invert_x, invert_y, invert_z);
}

bool xsens_mti_update(XsensData &data) {
    static XbusState state = XBUS_PREAMBLE;
    static uint8_t bid = 0;
    static uint8_t mid = 0;
    static uint16_t length = 0;
    static uint16_t payloadIdx = 0;
    static uint8_t payloadBuf[256];
    static uint8_t checksumAcc = 0;
    static uint32_t lastByteTimeMs = 0;
    bool new_packet = false;

    if (state != XBUS_PREAMBLE && (millis() - lastByteTimeMs > 50)) {
        state = XBUS_PREAMBLE;
    }

    while (XsensUart.available() > 0) {
        uint8_t b = XsensUart.read();
        lastByteTimeMs = millis();

        switch (state) {
            case XBUS_PREAMBLE:
                if (b == 0xFA) {
                    state = XBUS_BID;
                    checksumAcc = 0;
                }
                break;

            case XBUS_BID:
                bid = b;
                checksumAcc += b;
                state = XBUS_MID;
                break;

            case XBUS_MID:
                mid = b;
                checksumAcc += b;
                state = XBUS_LEN;
                break;

            case XBUS_LEN:
                checksumAcc += b;
                if (b == 0xFF) {
                    state = XBUS_EXT_LEN_H;
                } else {
                    length = b;
                    payloadIdx = 0;
                    if (length > sizeof(payloadBuf)) {
                        state = XBUS_PREAMBLE;
                    } else if (length == 0) {
                        state = XBUS_CHECKSUM;
                    } else {
                        state = XBUS_PAYLOAD;
                    }
                }
                break;

            case XBUS_EXT_LEN_H:
                checksumAcc += b;
                length = (uint16_t)b << 8;
                state = XBUS_EXT_LEN_L;
                break;

            case XBUS_EXT_LEN_L:
                checksumAcc += b;
                length |= b;
                payloadIdx = 0;
                if (length > sizeof(payloadBuf)) {
                    state = XBUS_PREAMBLE;
                } else if (length == 0) {
                    state = XBUS_CHECKSUM;
                } else {
                    state = XBUS_PAYLOAD;
                }
                break;

            case XBUS_PAYLOAD:
                checksumAcc += b;
                payloadBuf[payloadIdx++] = b;
                if (payloadIdx >= length) {
                    state = XBUS_CHECKSUM;
                }
                break;

            case XBUS_CHECKSUM:
                checksumAcc += b;
                if ((checksumAcc & 0xFF) == 0) {
                    if (mid == 0x36) {
                        parseMTData2Payload(payloadBuf, length);
                        new_packet = true;
                    }
                }
                state = XBUS_PREAMBLE;
                break;
        }
    }

    data = current_xsens;
    return new_packet;
}
