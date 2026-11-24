// Kiểm thử driver Xsens thật (xsens_mti.cpp): dấu Yaw & Gyro Z theo chiều LA BÀN.
// Xác minh trên khung thật (2026-11-24): khi chưa đảo, xoay CÙNG chiều kim đồng hồ thì Yaw GIẢM
// và mô hình 3D trên Web quay ngược drone thật. Sau khi đảo: Yaw TĂNG, gz DƯƠNG.
#include "Arduino.h"
#include "../../firmware/dual_imu_esp32c6/xsens_mti.h"

uint32_t g_fake_millis = 0;
FakeSerial Serial;
std::deque<uint8_t> g_fake_uart_rx;
static int g_fail = 0;
#define CHECK(cond, ...) do { if (!(cond)) { g_fail++; printf("  [FAIL] " __VA_ARGS__); } \
                              else { printf("  [ OK ] " __VA_ARGS__); } printf("\n"); } while (0)

static void putFloatBE(std::vector<uint8_t>& v, float f) {
    uint32_t u; memcpy(&u, &f, 4);
    for (int s = 24; s >= 0; s -= 8) v.push_back((uint8_t)(u >> s));
}
// Gói MTData2 (MID 0x36): Euler (0x2030, độ) + Rate of Turn (0x8020, rad/s)
static void sendPacket(float roll, float pitch, float yaw, float gx, float gy, float gz) {
    std::vector<uint8_t> pl;
    pl.push_back(0x20); pl.push_back(0x30); pl.push_back(12);
    putFloatBE(pl, roll); putFloatBE(pl, pitch); putFloatBE(pl, yaw);
    pl.push_back(0x80); pl.push_back(0x20); pl.push_back(12);
    putFloatBE(pl, gx); putFloatBE(pl, gy); putFloatBE(pl, gz);
    std::vector<uint8_t> f = { 0xFA, 0xFF, 0x36, (uint8_t)pl.size() };
    f.insert(f.end(), pl.begin(), pl.end());
    uint8_t sum = 0; for (size_t i = 1; i < f.size(); i++) sum += f[i];
    f.push_back((uint8_t)(0x100 - sum));
    for (uint8_t b : f) g_fake_uart_rx.push_back(b);
}

int main() {
    xsens_mti_init(2, 3, 115200);
    xsens_mti_set_axis_inversion(true, false, true);   // giống setup() trong dual_imu_esp32c6.ino
    XsensData d = {};

    // Xoay CÙNG chiều kim đồng hồ: dữ liệu THÔ của Xsens có yaw giảm dần (đúng điều đã quan sát:
    // trước khi sửa, firmware báo Yaw giảm) và gyro Z thô dương (vì firmware cũ đảo gz qua invert_z).
    const float raw_yaw[3] = { 0.0f, -10.0f, -20.0f };
    const float raw_gz = 0.5f;   // rad/s

    printf("\n[1] Khong dao Yaw (hanh vi cu): xoay cung chieu kim dong ho\n");
    xsens_mti_set_yaw_inversion(false);
    float y_old[3], gz_old = 0;
    for (int i = 0; i < 3; i++) { sendPacket(180.0f, 0, raw_yaw[i], 0, 0, raw_gz); xsens_mti_update(d); y_old[i] = d.yaw; gz_old = d.gz; }
    CHECK(y_old[2] < y_old[0] && gz_old < 0, "Yaw %.1f -> %.1f (GIAM), gz %.1f deg/s: dung loi da bao cao", y_old[0], y_old[2], gz_old);

    printf("\n[2] Da dao Yaw (xsens_mti_set_yaw_inversion(true)): xoay cung chieu kim dong ho\n");
    xsens_mti_set_yaw_inversion(true);
    float y_new[3], gz_new = 0;
    for (int i = 0; i < 3; i++) { sendPacket(180.0f, 0, raw_yaw[i], 0, 0, raw_gz); xsens_mti_update(d); y_new[i] = d.yaw; gz_new = d.gz; }
    CHECK(y_new[0] < y_new[1] && y_new[1] < y_new[2], "Yaw %.1f -> %.1f -> %.1f (TANG)", y_new[0], y_new[1], y_new[2]);
    CHECK(gz_new > 0 && fabsf(gz_new - 0.5f * 57.2957795f) < 0.01f, "gz = %+.2f deg/s (DUONG, cung chieu voi Yaw)", gz_new);
    CHECK(fabsf(d.roll) < 1e-3f && fabsf(d.pitch) < 1e-3f, "Roll/Pitch khong doi (R=%.2f P=%.2f)", d.roll, d.pitch);

    printf("\n[3] Mo hinh 3D Web quay theo -yaw (three_dimensional_scene_manager.js): cung chieu drone that\n");
    // three.js: góc dương quanh trục Y (hướng lên) = NGƯỢC chiều kim đồng hồ nhìn từ trên.
    // Model dùng -yaw => Yaw tăng => góc model giảm => model quay CÙNG chiều kim đồng hồ.
    const float model_delta = -(y_new[2] - y_new[0]);
    CHECK(model_delta < 0, "goc model three.js thay doi %.1f deg (am = cung chieu kim dong ho, khop drone that)", model_delta);

    printf("\n==== XSENS YAW: %s (%d loi) ====\n", g_fail ? "THAT BAI" : "TAT CA DAT", g_fail);
    return g_fail ? 1 : 0;
}
